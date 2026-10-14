// =============================================================================
//  engine/physics/bake.cpp — see bake.h for the design notes.
// =============================================================================
#include "engine/physics/bake.h"

#include "engine/world/chunk.h"

#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <dirent.h>

namespace mumbai::phys {

namespace J = JPH;

namespace {

// The chunk's collision records are cell-local; physics lives in the world
// frame, so every record is translated by the cell origin on the way in.  The
// records are then grouped into compounds: a cell can carry thousands of boxes
// (a dense block of buildings), and one body per box would cost a broadphase
// entry each, while one body per cell would rebuild a huge compound on any edit.
std::vector<CollisionBox> toWorldFrame(const CollisionBox* boxes, u32 count,
                                       const Vec2d& origin) {
    std::vector<CollisionBox> out;
    out.reserve(count);
    for (u32 i = 0; i < count; ++i) {
        CollisionBox b = boxes[i];
        b.cx += static_cast<f32>(origin.x);
        b.cz += static_cast<f32>(origin.y);
        out.push_back(b);
    }
    return out;
}

// A Jolt mesh shape from one of the chunk's mesh sections.  Jolt takes the
// triangles by value in a fixed layout; the world's vertex stride is 44 bytes
// with position first, so the copy is a gather rather than a memcpy.
J::RefConst<J::Shape> meshShapeFrom(const ChunkReader::MeshView& view, const Vec2d& origin,
                              BakeStats& stats) {
    if (!view.valid() || view.indexCount < 3) return nullptr;
    J::VertexList vertices;
    J::IndexedTriangleList triangles;
    vertices.reserve(view.vertexCount);
    for (u32 i = 0; i < view.vertexCount; ++i) {
        const MeshVertex& v = view.vertices[i];
        vertices.push_back(J::Float3(v.px + static_cast<f32>(origin.x), v.py,
                                  v.pz + static_cast<f32>(origin.y)));
    }
    triangles.reserve(view.indexCount / 3);
    for (u32 i = 0; i + 2 < view.indexCount; i += 3) {
        const u32 a = view.indices[i], b = view.indices[i + 1], c = view.indices[i + 2];
        if (a >= view.vertexCount || b >= view.vertexCount || c >= view.vertexCount) continue;
        if (a == b || b == c || a == c) continue;         // degenerate: rejected
        // A sliver is the same defect with three different indices: a triangle
        // whose area is nil has no normal, and Jolt's EPA asserts on it when a
        // query overlaps it (`JPH_ASSERT(!ioV.IsNearZero())` in
        // EPAPenetrationDepth).  The threshold is an absolute area — 1 cm^2 —
        // because a road that is metres wide cannot need a triangle smaller than
        // that, and a *relative* test would keep tiny triangles in a tiny mesh.
        const J::Float3& pa = vertices[a];
        const J::Float3& pb = vertices[b];
        const J::Float3& pc = vertices[c];
        const J::Vec3 ab = J::Vec3(pb) - J::Vec3(pa);
        const J::Vec3 ac = J::Vec3(pc) - J::Vec3(pa);
        if (ab.Cross(ac).LengthSq() < 4.0e-8f) {          // area < 1 cm^2
            ++stats.roadSlivers;
            continue;
        }
        triangles.push_back(J::IndexedTriangle(a, b, c, 0));
    }
    if (triangles.empty()) return nullptr;
    stats.roadTriangles += triangles.size();
    J::MeshShapeSettings settings(std::move(vertices), std::move(triangles));
    // A road surface is thin and a vehicle drives along it: the default 1 mm
    // convex radius is right; the active-edge tracking is what keeps a wheel
    // from falling through a triangle it is sliding along.
    J::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError()) return nullptr;
    return result.Get();
}

J::RefConst<J::Shape> heightfieldShapeFrom(const ChunkReader& chunk, BakeStats& stats) {
    u32 count = 0;
    const u8* raw = nullptr;
    const SectionEntry* e = chunk.find(SectionType::TerrainHeight);
    if (!e || e->size < sizeof(HeightfieldHeader)) return nullptr;
    raw = chunk.raw() + e->offset;
    HeightfieldHeader hh{};
    std::memcpy(&hh, raw, sizeof(hh));
    if (hh.encoding != 1 || hh.width < 2 || hh.height < 2) return nullptr;
    const u64 samples = static_cast<u64>(hh.width) * hh.height;
    if (sizeof(HeightfieldHeader) + samples * sizeof(u16) > e->size) return nullptr;
    const u16* heights = reinterpret_cast<const u16*>(raw + sizeof(HeightfieldHeader));
    count = static_cast<u32>(samples);

    // Jolt height fields are square: mSampleCount x mSampleCount samples.
    // Ours are too (the chunk format writes width == height == n + 1), so the
    // square API applies directly; a non-square field would have to be padded,
    // and padding a height field invents terrain.
    if (hh.width != hh.height) {
        stats.firstError = "heightfield is not square";
        return nullptr;
    }
    std::vector<f32> samplesF(static_cast<size_t>(count));
    for (u32 i = 0; i < count; ++i)
        samplesF[i] = hh.baseY + static_cast<f32>(heights[i]) * hh.scaleY;
    // inOffset + inScale * (x, sample, y): x and y are the integer sample
    // indices, so the scale carries the spacing and the offset the chunk origin.
    J::HeightFieldShapeSettings settings(samplesF.data(), J::Vec3(hh.originX, 0.0f, hh.originZ),
                                      J::Vec3(hh.stepX, 1.0f, hh.stepZ), hh.width);
    J::ShapeSettings::ShapeResult result = settings.Create();
    if (result.HasError()) return nullptr;
    ++stats.terrainShapes;
    return result.Get();
}

}  // namespace

bool readChunkInfo(const std::string& path, ChunkInfo& out) {
    ChunkReader chunk;
    if (!chunk.open(path, nullptr)) return false;
    const ChunkHeader& h = chunk.header();
    out.cellX = h.cellX;
    out.cellZ = h.cellZ;
    out.cellSize = h.cellSize;
    out.minHeight = h.minHeight;
    out.maxHeight = h.maxHeight;
    out.bytes = h.fileSize;
    return true;
}

bool bakeChunk(World& world, const std::string& path, const BakeOptions& options,
               BakeStats& stats) {
    ChunkReader chunk;
    std::string error;
    if (!chunk.open(path, &error)) {
        stats.firstError = path + ": " + error;
        return false;
    }
    ++stats.chunksRead;
    stats.minY = std::min(stats.minY, static_cast<f64>(chunk.header().minHeight));
    stats.maxY = std::max(stats.maxY, static_cast<f64>(chunk.header().maxHeight));
    const Vec2d origin{static_cast<f64>(chunk.header().cellX) * MACRO_CELL_SIZE,
                       static_cast<f64>(chunk.header().cellZ) * MACRO_CELL_SIZE};

    // --- static boxes, deck slabs, pylon legs and cable capsules --------------
    if (options.boxes) {
        u32 count = 0;
        const CollisionBox* boxes = chunk.array<CollisionBox>(SectionType::BuildingCollision, &count);
        if (boxes && count > 0 && stats.boxes < options.maxBoxes) {
            const std::vector<CollisionBox> world_boxes = toWorldFrame(boxes, count, origin);
            for (const CollisionBox& b : world_boxes) {
                if (b.flags & 8u) ++stats.capsules; else ++stats.boxes;
                stats.minY = std::min(stats.minY, static_cast<f64>(b.cy - b.hy));
                stats.maxY = std::max(stats.maxY, static_cast<f64>(b.cy + b.hy));
            }
            const u32 per = std::max<u32>(1, options.maxBoxesPerBody);
            for (u32 i = 0; i < world_boxes.size(); i += per) {
                const u32 n = std::min<u32>(per, static_cast<u32>(world_boxes.size()) - i);
                std::vector<CollisionBox> group(world_boxes.begin() + i,
                                                world_boxes.begin() + i + n);
                const BodyHandle h = world.addStaticCompound(group);
                if (h.valid()) { ++stats.staticBodies; ++stats.compoundGroups; }
            }
        }
    }

    // --- ground ---------------------------------------------------------------
    if (options.terrain) {
        J::RefConst<J::Shape> ground = heightfieldShapeFrom(chunk, stats);
        if (ground != nullptr) {
            // The height field's own offset is cell-local (the format writes
            // originX/originZ = 0), so the *body* carries the macro-cell origin.
            // Getting this wrong puts every cell's ground at the world origin:
            // the collision would look plausible in the first cell and the rest
            // of the city would have no floor at all.
            J::BodyCreationSettings bcs(ground, J::RVec3(origin.x, 0.0, origin.y), J::Quat::sIdentity(),
                                        J::EMotionType::Static, LayerStatic);
            if (world.bodies().CreateAndAddBody(bcs, J::EActivation::DontActivate).IsInvalid())
                MLOG_WARN("physics: heightfield body rejected in %s", path.c_str());
            else
                ++stats.staticBodies;
        }
    }

    // --- road surfaces --------------------------------------------------------
    if (options.roads) {
        for (u32 i = 0; i < chunk.meshCount(SectionType::RoadMesh); ++i) {
            J::RefConst<J::Shape> road = meshShapeFrom(chunk.mesh(SectionType::RoadMesh, i), origin, stats);
            if (road == nullptr) continue;
            J::BodyCreationSettings bcs(road, J::RVec3::sZero(), J::Quat::sIdentity(),
                                        J::EMotionType::Static, LayerStatic);
            if (!world.bodies().CreateAndAddBody(bcs, J::EActivation::DontActivate).IsInvalid()) {
                ++stats.staticBodies;
                ++stats.roadShapes;
            }
        }
    }
    return true;
}

bool bakeDirectory(World& world, const std::string& dir, const BakeOptions& options,
                   BakeStats& stats) {
    const auto t0 = std::chrono::steady_clock::now();
    DIR* d = ::opendir(dir.c_str());
    if (d == nullptr) {
        stats.firstError = "cannot open directory: " + dir;
        return false;
    }
    std::vector<std::string> files;
    while (dirent* ent = ::readdir(d)) {
        const std::string name = ent->d_name;
        if (name.size() > 13 && name.compare(name.size() - 13, 13, ".mumbai_chunk") == 0)
            files.push_back(dir + "/" + name);
    }
    ::closedir(d);
    std::sort(files.begin(), files.end());   // determinism: never filesystem order
    for (const std::string& f : files) bakeChunk(world, f, options, stats);
    stats.seconds = std::chrono::duration<f64>(std::chrono::steady_clock::now() - t0).count();
    return true;
}

}  // namespace mumbai::phys
