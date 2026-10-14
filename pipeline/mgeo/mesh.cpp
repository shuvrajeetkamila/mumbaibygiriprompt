// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/mesh.cpp
// =============================================================================
#include "pipeline/mgeo/mesh.h"

namespace mumbai::mesh {

u32 MeshBuilder::addVertex(const Vec3& position, const Vec3& normal, const Vec2& uv,
                           u32 materialId, u32 zoneId, u8 flags, u8 ao) {
    MeshVertex v{};
    v.px = position.x; v.py = position.y; v.pz = position.z;
    v.nx = normal.x;   v.ny = normal.y;   v.nz = normal.z;
    v.u = uv.x;        v.v = uv.y;
    // Second UV set: a low-frequency macro variation coordinate derived from
    // world position, which is what stops long straight highways and repetitive
    // facades from showing obvious tiling (Section 1B).
    v.u2 = position.x * 0.0016f;
    v.v2 = position.z * 0.0016f;
    v.materialId = static_cast<u8>(std::min<u32>(materialId, 255));
    v.zoneId = static_cast<u8>(std::min<u32>(zoneId, 255));
    v.flags = flags;
    v.ao = ao;
    m_vertices.push_back(v);
    return static_cast<u32>(m_vertices.size() - 1);
}

void MeshBuilder::addTriangle(u32 a, u32 b, u32 c) {
    // Submesh bookkeeping: an open submesh adopts the first index emitted while
    // it is open, so material ranges stay contiguous.  Submeshes nest LIFO, so
    // only the innermost open one is considered.
    if (!m_open.empty() && !m_open.back().open) {
        m_open.back().open = true;
        m_open.back().firstIndex = static_cast<u32>(m_indices.size());
    }
    m_indices.push_back(a);
    m_indices.push_back(b);
    m_indices.push_back(c);
}

void MeshBuilder::addQuad(u32 a, u32 b, u32 c, u32 d) {
    addTriangle(a, b, c);
    addTriangle(a, c, d);
}

void MeshBuilder::addQuad(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                          const Vec3& normal, u32 materialId, u32 zoneId, u8 flags,
                          f32 uvScale, u8 ao) {
    // Planar UV projection along the dominant tangent pair, at the material's
    // physical tile size: one UV unit is one tile (A8).  The caller's scale is
    // the fallback for a mesh built without a material library.
    const f32 s = materialUvScale(materialId, uvScale);
    auto uvOf = [&](const Vec3& p) -> Vec2 {
        return {p.x * s, (p.z + p.y) * s};
    };
    const u32 a = addVertex(p0, normal, uvOf(p0), materialId, zoneId, flags, ao);
    const u32 b = addVertex(p1, normal, uvOf(p1), materialId, zoneId, flags, ao);
    const u32 c = addVertex(p2, normal, uvOf(p2), materialId, zoneId, flags, ao);
    const u32 d = addVertex(p3, normal, uvOf(p3), materialId, zoneId, flags, ao);
    addQuad(a, b, c, d);
}

void MeshBuilder::addDoubleSidedQuad(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3,
                                     u32 materialId, u32 zoneId, u8 flags, f32 uvScale) {
    const Vec3 n = (p1 - p0).cross(p3 - p0).normalized();
    addQuad(p0, p1, p2, p3, n, materialId, zoneId, flags, uvScale);
    addQuad(p3, p2, p1, p0, -n, materialId, zoneId, flags, uvScale);
}

void MeshBuilder::extrudeWalls(const std::vector<Vec2d>& ring, f64 baseY, f64 topY,
                               u32 materialId, u32 zoneId, u8 flags, f32 uvScale) {
    if (ring.size() < 3 || topY <= baseY) return;
    const f64 height = topY - baseY;
    f64 runLength = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const Vec2d& a = ring[i];
        const Vec2d& b = ring[(i + 1) % ring.size()];
        const Vec2d edge = b - a;
        const f64 len = edge.length();
        if (len < 1e-6) continue;
        // Outward normal for a counter-clockwise ring in a Y-up, XZ-plane world
        // is (dy, -dx) normalised.
        const Vec3 n{static_cast<f32>(edge.y / len), 0.0f, static_cast<f32>(-edge.x / len)};

        const Vec3 b0{static_cast<f32>(a.x), static_cast<f32>(baseY), static_cast<f32>(a.y)};
        const Vec3 b1{static_cast<f32>(b.x), static_cast<f32>(baseY), static_cast<f32>(b.y)};
        const Vec3 t0{static_cast<f32>(a.x), static_cast<f32>(topY),  static_cast<f32>(a.y)};
        const Vec3 t1{static_cast<f32>(b.x), static_cast<f32>(topY),  static_cast<f32>(b.y)};

        // UV: U runs with arc length (world-scaled so stone courses and brick
        // rows keep constant size regardless of facade length), V with height —
        // and scaled so that one UV unit is one *physical* tile of the material
        // (A8), which is what makes a brick 230 mm on every facade in the city.
        const f32 s = materialUvScale(materialId, uvScale);
        const f32 u0 = static_cast<f32>(runLength * s);
        const f32 u1 = static_cast<f32>((runLength + len) * s);
        const f32 v0 = 0.0f;
        const f32 v1 = static_cast<f32>(height * s);

        auto push = [&](const Vec3& p, f32 u, f32 v) {
            return addVertex(p, n, {u, v}, materialId, zoneId, flags);
        };
        const u32 i0 = push(b0, u0, v0);   // bottom, edge start
        const u32 i1 = push(b1, u1, v0);   // bottom, edge end
        const u32 i2 = push(t1, u1, v1);   // top, edge end
        const u32 i3 = push(t0, u0, v1);   // top, edge start
        // Winding (bottom-a, top-a, top-b, bottom-b) makes the geometric normal
        // equal the outward normal computed above.  Verified in
        // tests/test_geometry.cpp::wallsFaceOutward — getting this wrong would
        // invert every facade in the city.
        addQuad(i0, i3, i2, i1);
        runLength += len;
    }
}

void MeshBuilder::addRoofCap(const std::vector<Vec2d>& ring, f64 y, u32 materialId, u32 zoneId,
                             u8 flags, f32 uvScale, f64 inset, f64 lipHeight) {
    if (ring.size() < 3) return;
    std::vector<Vec2d> cap = ring;
    if (inset > 0.0) cap = geo2d::offsetRing(ring, -inset);
    if (cap.size() < 3) cap = ring;

    geo2d::Mesh2 tris = geo2d::triangulateRing(cap);
    const Vec3 n{0, 1, 0};
    const f32 us = materialUvScale(materialId, uvScale);
    std::vector<u32> mapping(tris.vertices.size());
    for (size_t i = 0; i < tris.vertices.size(); ++i) {
        const Vec2d& p = tris.vertices[i];
        mapping[i] = addVertex({static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)},
                               n, {static_cast<f32>(p.x * us), static_cast<f32>(p.y * us)},
                               materialId, zoneId, flags);
    }
    // The ear-clipper emits counter-clockwise triangles in the 2D footprint
    // frame; mapped to world XZ with Y up, that orientation faces downwards, so
    // the emission order is reversed to make roofs face the sky.
    for (size_t i = 0; i + 2 < tris.indices.size(); i += 3)
        addTriangle(mapping[tris.indices[i + 2]], mapping[tris.indices[i + 1]], mapping[tris.indices[i]]);

    if (lipHeight > 0.0) {   // parapet wall around the roof edge
        extrudeWalls(cap, y, y + lipHeight, materialId, zoneId, flags, uvScale);
        std::vector<Vec2d> lipInner = geo2d::offsetRing(cap, -0.35);
        if (lipInner.size() >= 3) {
            geo2d::Mesh2 lipTris = geo2d::triangulateRing(lipInner);
            std::vector<u32> lipMap(lipTris.vertices.size());
            for (size_t i = 0; i < lipTris.vertices.size(); ++i) {
                const Vec2d& p = lipTris.vertices[i];
                lipMap[i] = addVertex({static_cast<f32>(p.x), static_cast<f32>(y + lipHeight), static_cast<f32>(p.y)},
                                      n, {static_cast<f32>(p.x * us), static_cast<f32>(p.y * us)},
                                      materialId, zoneId, flags);
            }
            for (size_t i = 0; i + 2 < lipTris.indices.size(); i += 3)
                addTriangle(lipMap[lipTris.indices[i + 2]], lipMap[lipTris.indices[i + 1]], lipMap[lipTris.indices[i]]);
        }
    }
}

void MeshBuilder::beginSubmesh(u32 materialId, u32 zoneId) {
    OpenSubmesh s;
    s.materialId = materialId;
    s.zoneId = zoneId;
    s.firstVertex = static_cast<u32>(m_vertices.size());
    s.firstIndex = static_cast<u32>(m_indices.size());
    s.open = false;
    m_open.push_back(s);
}

void MeshBuilder::endSubmesh() {
    if (m_open.empty()) return;
    const OpenSubmesh s = m_open.back();
    m_open.pop_back();
    if (!s.open) return;   // nothing was emitted inside this range
    SubMesh sm{};
    sm.materialId = s.materialId;
    sm.zoneId = s.zoneId;
    sm.indexOffset = s.firstIndex;
    sm.indexCount = static_cast<u32>(m_indices.size()) - s.firstIndex;
    sm.vertexOffset = s.firstVertex;
    if (sm.indexCount > 0) m_submeshes.push_back(sm);
}

void MeshBuilder::bakeAmbientOcclusion(f64 groundY, f64 falloff) {
    // Cheap, deterministic AO: a vertex is darkened by how close it sits to the
    // ground plane and how "enclosed" its footprint is. The runtime treats
    // ao == 255 as fully lit, so this only ever subtracts.
    for (MeshVertex& v : m_vertices) {
        const f64 h = std::max(0.0, static_cast<f64>(v.py) - groundY);
        const f64 groundFactor = clamp(h / std::max(0.5, falloff), 0.35, 1.0);
        const f64 value = clamp(groundFactor, 0.0, 1.0) * 255.0;
        v.ao = static_cast<u8>(std::min<f64>(value, 255.0));
    }
}

std::vector<u8> MeshBuilder::serialize(u32 lodLevel) const {
    std::vector<const MeshBuilder*> one{this};
    std::vector<u32> lods{lodLevel};
    return serializeMeshGroup(one, lods);
}

void MeshBuilder::appendTranslated(MeshBuilder& dst, const Vec3& offset) const {
    const u32 vertexBase = static_cast<u32>(dst.m_vertices.size());
    const u32 indexBase = static_cast<u32>(dst.m_indices.size());

    dst.m_vertices.reserve(dst.m_vertices.size() + m_vertices.size());
    for (const MeshVertex& v : m_vertices) {
        MeshVertex t = v;
        t.px += offset.x;
        t.py += offset.y;
        t.pz += offset.z;
        // Macro-variation UVs are world-space by definition; keep them in the
        // destination frame so that variation does not restart per chunk.
        t.u2 += offset.x * 0.0016f;
        t.v2 += offset.z * 0.0016f;
        dst.m_vertices.push_back(t);
    }
    dst.m_indices.reserve(dst.m_indices.size() + m_indices.size());
    for (u32 i : m_indices) dst.m_indices.push_back(i + vertexBase);

    dst.m_submeshes.reserve(dst.m_submeshes.size() + m_submeshes.size());
    for (const SubMesh& s : m_submeshes) {
        SubMesh t = s;
        t.indexOffset += indexBase;    // rebase both ranges into the destination
        t.vertexOffset += vertexBase;
        dst.m_submeshes.push_back(t);
    }
}

// -----------------------------------------------------------------------------
std::vector<u8> serializeMeshGroup(const std::vector<const MeshBuilder*>& meshes,
                                   const std::vector<u32>& lodLevels) {
    MCHECK(meshes.size() == lodLevels.size(), "mesh group arity mismatch");

    MeshGroupHeader hdr{};
    hdr.meshCount = static_cast<u32>(meshes.size());
    hdr.flags = 0;

    const u64 entryBytes = static_cast<u64>(meshes.size()) * sizeof(MeshEntry);
    u64 cursor = sizeof(MeshGroupHeader) + entryBytes;
    cursor = (cursor + 15) & ~15ull;

    struct Plan { u64 vertexOff, indexOff, submeshOff; };
    std::vector<Plan> plans(meshes.size());
    for (size_t i = 0; i < meshes.size(); ++i) {
        const MeshBuilder& mb = *meshes[i];
        plans[i].vertexOff = cursor;
        cursor = (cursor + mb.vertexData().size() * sizeof(MeshVertex) + 15) & ~15ull;
        plans[i].indexOff = cursor;
        cursor = (cursor + mb.indexData().size() * sizeof(u32) + 15) & ~15ull;
        plans[i].submeshOff = cursor;
        cursor = (cursor + mb.submeshData().size() * sizeof(SubMesh) + 15) & ~15ull;
    }

    std::vector<u8> out(static_cast<size_t>(cursor), 0);
    std::memcpy(out.data(), &hdr, sizeof(hdr));

    std::vector<MeshEntry> entries(meshes.size());
    for (size_t i = 0; i < meshes.size(); ++i) {
        const MeshBuilder& mb = *meshes[i];
        MeshEntry& e = entries[i];
        e.vertexCount = static_cast<u32>(mb.vertexData().size());
        e.indexCount = static_cast<u32>(mb.indexData().size());
        e.submeshCount = static_cast<u32>(mb.submeshData().size());
        e.lodLevel = lodLevels[i];
        e.verticesOffset = plans[i].vertexOff;
        e.indicesOffset = plans[i].indexOff;
        e.submeshesOffset = plans[i].submeshOff;

        if (e.vertexCount)
            std::memcpy(out.data() + plans[i].vertexOff, mb.vertexData().data(),
                        mb.vertexData().size() * sizeof(MeshVertex));
        if (e.indexCount)
            std::memcpy(out.data() + plans[i].indexOff, mb.indexData().data(),
                        mb.indexData().size() * sizeof(u32));
        if (e.submeshCount)
            std::memcpy(out.data() + plans[i].submeshOff, mb.submeshData().data(),
                        mb.submeshData().size() * sizeof(SubMesh));
    }
    std::memcpy(out.data() + sizeof(MeshGroupHeader), entries.data(), entryBytes);
    return out;
}

} // namespace mumbai::mesh
