// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/export.cpp
//
//  Viewer export.  This stage does not read the in-memory build structures: it
//  opens the manifest, re-opens every .mumbai_chunk through the *runtime*
//  reader (mmap, typed views, per-section hash verification) and re-derives the
//  world from the files on disk.  The export is therefore an end-to-end test of
//  the chunk contract: if the viewer shows Mumbai, then the bytes the runtime
//  reads are the bytes the pipeline wrote.
//
//  Output (viewer/data/):
//      mumbai.json  — metadata: extent, grids, counts, byte ranges, landmarks,
//                     build statistics and the QA summary
//      mumbai.bin   — packed payload: terrain, building instances, street
//                     centreline graph, landmark meshes
//
//  Payload layout (offsets in the JSON, all little-endian):
//      terrain   u16[TW*TH]   heights, y = -(32) + value/65535 * 192 m
//      buildings 14 B each    u16 x, u16 z (metres from extent min), u16 w, u16 d,
//                             u16 h, u16 groundY (metres + 32), u8 materialId,
//                             u8 yaw (256ths of a turn)
//      nodes     6 B each     i16 x, i16 z, i16 y  (metres from extent min)
//      edges     12 B each    u32 from, u32 to, u16 lanes, u8 roadClass, u8 flags
//      hero      32 B/vertex  f32 x, f32 y, f32 z, f32 nx, f32 ny, f32 nz,
//                             u8 materialId, u8 zoneId, u16 pad   (indexed)
//      heroIdx   4 B/triangle u32[a], u32[b], u32[c]
// =============================================================================
#include "pipeline/pipeline.h"
#include "pipeline/mgeo/synth.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace mumbai::pipeline {
namespace {

struct BinWriter {
    std::vector<u8> data;
    void i16v(i16 v) { const u16 u = static_cast<u16>(v); data.push_back(u8(u)); data.push_back(u8(u >> 8)); }
    void u16v(u16 v) { data.push_back(u8(v)); data.push_back(u8(v >> 8)); }
    void u32v(u32 v) { for (int i = 0; i < 4; ++i) data.push_back(u8(v >> (8 * i))); }
    void f32v(f32 v) { u32 u; std::memcpy(&u, &v, 4); u32v(u); }
    void u8v(u8 v) { data.push_back(v); }
    size_t size() const { return data.size(); }
};

// Hero anchors, mirrored from pipeline/stages/hero.cpp so the viewer can label
// them.  Coordinates are the published positions of the real assets.
struct Anchor { const char* name; f64 lat, lon; };
const Anchor kAnchors[] = {
    {"Gateway of India", 18.9220, 72.8347},
    {"Taj Mahal Palace", 18.9216, 72.8332},
    {"Chhatrapati Shivaji Maharaj Terminus", 18.9398, 72.8355},
    {"Bandra-Worli Sea Link", 19.0350, 72.8180},
    {"Marine Drive", 18.9430, 72.8225},
};

}  // namespace

bool exportViewer(Context& ctx) {
    ScopedTimer timer("exportViewer");

    const std::string manifestPath = ctx.cfg.outDir + "/world_manifest.json";
    const std::string text = readFileToString(manifestPath);
    if (text.empty()) {
        ctx.qa.warn("viewer export skipped: manifest missing (" + manifestPath + ")");
        return true;
    }
    Json root;
    std::string err;
    if (!Json::parse(text, root, &err)) {
        ctx.qa.warn("viewer export skipped: manifest unreadable — " + err);
        return true;
    }

    const f64 minX = root["extent"]["minX"].asNumber(0.0);
    const f64 minZ = root["extent"]["minZ"].asNumber(0.0);
    const f64 maxX = root["extent"]["maxX"].asNumber(0.0);
    const f64 maxZ = root["extent"]["maxZ"].asNumber(0.0);
    if (!(maxX > minX) || !(maxZ > minZ)) {
        ctx.qa.warn("viewer export skipped: extent invalid");
        return true;
    }

    // --- terrain grid --------------------------------------------------------
    const u32 TN = 32;                                   // heightfield samples per cell edge
    const u32 TX = static_cast<u32>(std::llround((maxX - minX) / MACRO_CELL_SIZE));
    const u32 TZ = static_cast<u32>(std::llround((maxZ - minZ) / MACRO_CELL_SIZE));
    const u32 TW = TX * TN + 1, TH = TZ * TN + 1;
    const f32 heightBias = -32.0f, heightSpan = 192.0f;   // global u16 -> metres
    std::vector<u16> terrain(static_cast<size_t>(TW) * TH, 0u);
    std::vector<u8> terrainValid(static_cast<size_t>(TW) * TH, 0u);

    BinWriter buildings, nodes, edges, heroVerts, heroIdx;
    u64 cells = 0, buildingCount = 0, nodeCount = 0, edgeCount = 0, heroVertexCount = 0, heroIndexCount = 0;
    u64 fallbackCells = 0;
    f64 terrainMin = 1e30, terrainMax = -1e30;

    for (const Json& cell : root["cells"].items()) {
        const std::string file = cell["file"].asString();
        if (file.empty()) continue;
        const std::string path = ctx.cfg.outDir + "/cells/" + file;
        ChunkReader reader;
        std::string openErr;
        if (!reader.open(path, &openErr)) {
            ctx.qa.warn("viewer export: cannot open " + path + " — " + openErr);
            continue;
        }
        if (!reader.verifyHashes(&openErr)) {
            ctx.qa.fail("viewer export: hash mismatch in " + path + " — " + openErr);
            continue;
        }
        ++cells;
        const ChunkHeader& hdr = reader.header();
        const f64 cellOX = static_cast<f64>(hdr.cellX) * MACRO_CELL_SIZE;
        const f64 cellOZ = static_cast<f64>(hdr.cellZ) * MACRO_CELL_SIZE;
        const auto* base = reader.raw();

        // --- terrain ----------------------------------------------------------
        if (const SectionEntry* e = reader.find(SectionType::TerrainHeight)) {
            const u8* p = base + e->offset;
            HeightfieldHeader hh{};
            std::memcpy(&hh, p, sizeof(hh));
            const u16* hgrid = reinterpret_cast<const u16*>(p + sizeof(HeightfieldHeader));
            const i64 ti = static_cast<i64>(std::llround((cellOX - minX) / MACRO_CELL_SIZE));
            const i64 tj = static_cast<i64>(std::llround((cellOZ - minZ) / MACRO_CELL_SIZE));
            if (ti < 0 || tj < 0) continue;
            for (u32 j = 0; j <= TN && j < hh.height; ++j) {
                for (u32 i = 0; i <= TN && i < hh.width; ++i) {
                    const u32 gx = static_cast<u32>(ti) * TN + i;
                    const u32 gz = static_cast<u32>(tj) * TN + j;
                    if (gx >= TW || gz >= TH) continue;
                    const f32 h = hh.baseY + hgrid[j * hh.width + i] * hh.scaleY;
                    if (h < terrainMin) terrainMin = h;
                    if (h > terrainMax) terrainMax = h;
                    const f32 norm = std::max(0.0f, std::min(1.0f, (h - heightBias) / heightSpan));
                    terrain[static_cast<size_t>(gz) * TW + gx] = static_cast<u16>(norm * 65535.0f);
                    terrainValid[static_cast<size_t>(gz) * TW + gx] = 1;
                }
            }
        }

        // --- buildings --------------------------------------------------------
        // Oriented proxies are the normal case.  The collision-box fallback
        // exists for cells with no proxy section at all (a cell with no
        // buildings but, say, a hero collider) and only accepts boxes flagged
        // as building colliders (flags bit 0) — counting every collision box as
        // a building is what made the payload report 164,058 buildings against
        // the build's 164,057.
        u32 proxyCount = 0;
        if (const BuildingProxy* proxies = reader.array<BuildingProxy>(SectionType::BuildingProxy,
                                                                      &proxyCount)) {
            for (u32 b = 0; b < proxyCount; ++b) {
                const BuildingProxy& bp = proxies[b];
                const f64 wx = cellOX + bp.cx, wz = cellOZ + bp.cz;
                const f32 w = bp.hx * 2.0f, d = bp.hz * 2.0f, h = bp.hy * 2.0f;
                if (w < 1.5f || d < 1.5f || h < 2.0f) continue;
                if (wx < minX || wx > maxX || wz < minZ || wz > maxZ) continue;
                buildings.u16v(static_cast<u16>(std::lround(wx - minX)));
                buildings.u16v(static_cast<u16>(std::lround(wz - minZ)));
                buildings.u16v(static_cast<u16>(std::min(65000.0f, w)));
                buildings.u16v(static_cast<u16>(std::min(65000.0f, d)));
                buildings.u16v(static_cast<u16>(std::min(65000.0f, h)));
                buildings.u16v(static_cast<u16>(std::min(65000.0f, bp.baseY + 32.0f)));   // ground
                buildings.u8v(static_cast<u8>(std::min<u32>(255, bp.materialId)));
                buildings.u8v(static_cast<u8>(std::lround(
                    (bp.yaw / static_cast<f32>(TAU)) * 256.0f) & 0xFF));
                ++buildingCount;
            }
        } else {
            ++fallbackCells;
            u32 boxCount = 0;
            if (const CollisionBox* boxes = reader.array<CollisionBox>(SectionType::BuildingCollision,
                                                                       &boxCount)) {
                for (u32 b = 0; b < boxCount; ++b) {
                    const CollisionBox& cb = boxes[b];
                    if ((cb.flags & 1u) == 0u) continue;      // not a building collider
                    const f64 wx = cellOX + cb.cx, wz = cellOZ + cb.cz;
                    const f32 w = cb.hx * 2.0f, d = cb.hz * 2.0f, h = cb.hy * 2.0f;
                    if (w < 1.0f || d < 1.0f || h < 2.0f) continue;
                    if (wx < minX || wx > maxX || wz < minZ || wz > maxZ) continue;
                    buildings.u16v(static_cast<u16>(std::lround(wx - minX)));
                    buildings.u16v(static_cast<u16>(std::lround(wz - minZ)));
                    buildings.u16v(static_cast<u16>(std::min(65000.0f, w)));
                    buildings.u16v(static_cast<u16>(std::min(65000.0f, d)));
                    buildings.u16v(static_cast<u16>(std::min(65000.0f, h)));
                    buildings.u16v(static_cast<u16>(std::lround(cb.cy - cb.hy + 32.0f)));  // ground
                    buildings.u8v(static_cast<u8>(std::min<u32>(255, cb.materialId)));
                    buildings.u8v(0);
                    ++buildingCount;
                }
            }
        }

        // --- street graph -----------------------------------------------------
        if (const SectionEntry* e = reader.find(SectionType::RoadNavGraph); e && e->size >= sizeof(NavGraphHeader)) {
            const u8* p = base + e->offset;
            NavGraphHeader gh{};
            std::memcpy(&gh, p, sizeof(gh));
            const NavNode* nn = reinterpret_cast<const NavNode*>(p + sizeof(NavGraphHeader));
            const NavEdge* ne = reinterpret_cast<const NavEdge*>(
                p + sizeof(NavGraphHeader) + static_cast<size_t>(gh.nodeCount) * sizeof(NavNode));
            const u32 nodeBase = static_cast<u32>(nodeCount);
            for (u32 i = 0; i < gh.nodeCount; ++i) {
                nodes.i16v(static_cast<i16>(std::lround(std::max(-32000.0f, std::min(32000.0f,
                                       static_cast<f32>(cellOX) + nn[i].x - static_cast<f32>(minX))))));
                nodes.i16v(static_cast<i16>(std::lround(std::max(-32000.0f, std::min(32000.0f,
                                       static_cast<f32>(cellOZ) + nn[i].z - static_cast<f32>(minZ))))));
                nodes.i16v(static_cast<i16>(std::lround(std::max(-32000.0f, std::min(32000.0f, nn[i].y)))));
                ++nodeCount;
            }
            for (u32 i = 0; i < gh.edgeCount; ++i) {
                edges.u32v(nodeBase + i);                      // from = the edge's own node
                edges.u32v(nodeBase + ne[i].toNode);
                edges.u16v(ne[i].laneCount);
                edges.u8v(static_cast<u8>(std::min<u32>(255, ne[i].roadClass)));
                edges.u8v(static_cast<u8>(ne[i].direction));
                ++edgeCount;
            }
        }

        // --- hero / landmark meshes -------------------------------------------
        // Read through ChunkReader's typed MeshView: the vertex layout is 44
        // bytes (position, normal, two UV sets, material, zone, flags, AO) and
        // hand-rolled stride maths here would silently mangle it.
        const u32 heroMeshes = reader.meshCount(SectionType::HeroAssets);
        for (u32 m = 0; m < heroMeshes; ++m) {
            const ChunkReader::MeshView view = reader.mesh(SectionType::HeroAssets, m);
            if (!view.valid() || view.lodLevel != 0 || view.indexCount < 3) continue;
            const u32 vbase = static_cast<u32>(heroVertexCount);
            for (u32 v = 0; v < view.vertexCount; ++v) {
                const MeshVertex& mv = view.vertices[v];
                heroVerts.f32v(static_cast<f32>(cellOX) + mv.px - static_cast<f32>(minX));
                heroVerts.f32v(mv.py);
                heroVerts.f32v(static_cast<f32>(cellOZ) + mv.pz - static_cast<f32>(minZ));
                heroVerts.f32v(mv.nx);
                heroVerts.f32v(mv.ny);
                heroVerts.f32v(mv.nz);
                heroVerts.u8v(mv.materialId);
                heroVerts.u8v(static_cast<u8>(mv.ao));
                heroVerts.u8v(mv.flags);
                heroVerts.u8v(0);
                heroVerts.u32v(0);            // pad the record to the declared stride
                ++heroVertexCount;
            }
            for (u32 t = 0; t + 2 < view.indexCount; t += 3) {
                heroIdx.u32v(vbase + view.indices[t]);
                heroIdx.u32v(vbase + view.indices[t + 1]);
                heroIdx.u32v(vbase + view.indices[t + 2]);
                heroIndexCount += 3;
            }
        }
    }

    // --- assemble the binary -------------------------------------------------
    // --- material table ------------------------------------------------------
    // Materials are identical across chunks (the pipeline writes one shared
    // table), so it is captured once from the first chunk that carries one.
    // Variant materials (weathering/tint) are included: the facade variety the
    // fabric stage generates lives entirely in these entries.
    Json materials = Json::array();
    {
        Json cell0 = root["cells"].items().empty() ? Json() : root["cells"].items().front();
        const std::string file = cell0["file"].asString();
        if (!file.empty()) {
            ChunkReader r;
            std::string e2;
            if (r.open(ctx.cfg.outDir + "/cells/" + file, &e2)) {
                u32 matCount = 0;
                if (const MaterialDef* defs = r.array<MaterialDef>(SectionType::MaterialTable, &matCount)) {
                    for (u32 i = 0; i < matCount; ++i) {
                        Json m = Json::object();
                        m.set("id", Json(static_cast<f64>(i)));
                        m.set("name", Json(std::string(defs[i].name)));
                        m.set("r", Json(defs[i].baseColor[0]));
                        m.set("g", Json(defs[i].baseColor[1]));
                        m.set("b", Json(defs[i].baseColor[2]));
                        m.set("roughness", Json(defs[i].roughness));
                        m.set("metallic", Json(defs[i].metallic));
                        m.set("wetness", Json(defs[i].wetnessResponse));
                        materials.push(m);
                    }
                }
            }
        }
    }

    BinWriter out;
    const u64 terrainBytes = static_cast<u64>(terrain.size()) * sizeof(u16);
    const u64 terrainOffset = 0;
    const u64 buildingsOffset = terrainOffset + terrainBytes;
    const u64 nodesOffset = buildingsOffset + buildings.size();
    const u64 edgesOffset = nodesOffset + nodes.size();
    const u64 heroVertOffset = edgesOffset + edges.size();
    const u64 heroIdxOffset = heroVertOffset + heroVerts.size();
    out.data.reserve(static_cast<size_t>(heroIdxOffset + heroIdx.size()));
    out.data.insert(out.data.end(), reinterpret_cast<const u8*>(terrain.data()),
                    reinterpret_cast<const u8*>(terrain.data() + terrain.size()));
    out.data.insert(out.data.end(), buildings.data.begin(), buildings.data.end());
    out.data.insert(out.data.end(), nodes.data.begin(), nodes.data.end());
    out.data.insert(out.data.end(), edges.data.begin(), edges.data.end());
    out.data.insert(out.data.end(), heroVerts.data.begin(), heroVerts.data.end());
    out.data.insert(out.data.end(), heroIdx.data.begin(), heroIdx.data.end());

    const std::string dataDir = "viewer/data";
    ensureDir(dataDir);
    writeFile(dataDir + "/mumbai.bin", std::string(reinterpret_cast<const char*>(out.data.data()),
                                                   out.data.size()));

    // --- metadata ------------------------------------------------------------
    Json meta = Json::object();
    meta.set("format", Json(std::string("mumbai-viewer-scene")));
    meta.set("formatVersion", Json(1));
    meta.set("generated", Json(std::string("mumbai_pipeline")));

    Json ext = Json::object();
    ext.set("minX", Json(minX)); ext.set("minZ", Json(minZ));
    ext.set("maxX", Json(maxX)); ext.set("maxZ", Json(maxZ));
    ext.set("seaLevel", Json(ctx.cfg.seaLevel));
    meta.set("extent", ext);

    Json terr = Json::object();
    terr.set("width", Json(static_cast<f64>(TW)));
    terr.set("height", Json(static_cast<f64>(TH)));
    terr.set("step", Json((maxX - minX) / (TW - 1)));
    terr.set("bias", Json(static_cast<f64>(heightBias)));
    terr.set("span", Json(static_cast<f64>(heightSpan)));
    terr.set("offset", Json(static_cast<f64>(terrainOffset)));
    terr.set("minHeight", Json(terrainMin));
    terr.set("maxHeight", Json(terrainMax));
    meta.set("terrain", terr);

    Json b = Json::object();
    b.set("stride", Json(14.0));
    b.set("count", Json(static_cast<f64>(buildingCount)));
    b.set("offset", Json(static_cast<f64>(buildingsOffset)));
    meta.set("buildings", b);

    Json n = Json::object();
    n.set("stride", Json(6.0));
    n.set("count", Json(static_cast<f64>(nodeCount)));
    n.set("offset", Json(static_cast<f64>(nodesOffset)));
    meta.set("nodes", n);

    Json e = Json::object();
    e.set("stride", Json(12.0));
    e.set("count", Json(static_cast<f64>(edgeCount)));
    e.set("offset", Json(static_cast<f64>(edgesOffset)));
    meta.set("edges", e);

    Json h = Json::object();
    h.set("vertexStride", Json(32.0));  // 6 floats + 4 bytes, padded to 32
    h.set("vertexCount", Json(static_cast<f64>(heroVertexCount)));
    h.set("vertexOffset", Json(static_cast<f64>(heroVertOffset)));
    h.set("indexOffset", Json(static_cast<f64>(heroIdxOffset)));
    h.set("indexCount", Json(static_cast<f64>(heroIndexCount)));
    meta.set("hero", h);

    Json anchors = Json::array();
    for (const Anchor& a : kAnchors) {
        Json o = Json::object();
        o.set("name", Json(std::string(a.name)));
        const Vec2d p = ctx.proj.forward(a.lat, a.lon);
        o.set("x", Json(p.x - minX));
        o.set("z", Json(p.y - minZ));
        o.set("lat", Json(a.lat));
        o.set("lon", Json(a.lon));
        anchors.push(o);
    }
    meta.set("landmarks", anchors);
    meta.set("materials", materials);

    Json stats = Json::object();
    stats.set("chunks", Json(static_cast<f64>(cells)));
    stats.set("buildings", Json(static_cast<f64>(buildingCount)));
    stats.set("navNodes", Json(static_cast<f64>(nodeCount)));
    stats.set("navEdges", Json(static_cast<f64>(edgeCount)));
    meta.set("stats", stats);

    // Compact QA summary for the viewer's status line.  This is a snapshot as
    // of the export; finaliseQa() rewrites it with the finished numbers so the
    // payload never disagrees with the console.
    Json qa = Json::object();
    qa.set("counters", Json(static_cast<f64>(ctx.qa.counters.size())));
    qa.set("warnings", Json(static_cast<f64>(ctx.qa.warnings.size())));
    qa.set("failures", Json(static_cast<f64>(ctx.qa.failures.size())));
    qa.set("passed", Json(ctx.qa.ok()));
    meta.set("qa", qa);

    writeFile(dataDir + "/mumbai.json", meta.dump(1));

    MLOG_INFO("viewer export: %llu chunks, %llu buildings, %llu nav nodes, %llu edges, "
              "terrain %u x %u, %.1f MB",
              (unsigned long long)cells, (unsigned long long)buildingCount,
              (unsigned long long)nodeCount, (unsigned long long)edgeCount,
              TW, TH, out.size() / 1e6);
    MLOG_INFO("viewer export: %zu materials, %llu hero vertices", materials.size(),
              (unsigned long long)heroVertexCount);
    ctx.qa.add("viewer.exportBuildings", static_cast<f64>(buildingCount));
    ctx.qa.add("viewer.exportFallbackCells", static_cast<f64>(fallbackCells));
    ctx.qa.add("viewer.exportBytes", static_cast<f64>(out.size()));
    (void)terrainValid;
    return true;
}

// -----------------------------------------------------------------------------
//  finaliseQa — publish the finished report
// -----------------------------------------------------------------------------
// The QA report is serialised in three places: the console summary, the world
// manifest and the viewer payload.  The manifest and the payload are produced by
// stages that run before the last per-stage timing counters exist, so taken
// as-is they under-report — the payload read 63 counters while the run printed
// 66 (the export stage's own timing, the two viewer.* counters and this step).
// The build therefore ends here: the timing entry for this step is added first,
// both JSON files are re-opened, their QA blocks are replaced with the finished
// report, and the final count now agrees everywhere.
bool finaliseQa(Context& ctx, f64 stageStart) {
    ctx.qa.add("stage.finaliseQa.seconds", ScopedTimer::now() - stageStart);

    const Json full = qaReportJson(ctx.qa);
    Json summary = Json::object();
    summary.set("counters", Json(static_cast<f64>(ctx.qa.counters.size())));
    summary.set("warnings", Json(static_cast<f64>(ctx.qa.warnings.size())));
    summary.set("failures", Json(static_cast<f64>(ctx.qa.failures.size())));
    summary.set("passed", Json(ctx.qa.ok()));

    auto rewrite = [&](const std::string& path, const char* key, const Json& block) -> bool {
        const std::string text = readFileToString(path);
        if (text.empty()) {
            ctx.qa.warn("finaliseQa: cannot read " + path);
            return false;
        }
        Json root;
        std::string err;
        if (!Json::parse(text, root, &err)) {
            ctx.qa.warn("finaliseQa: cannot parse " + path + " — " + err);
            return false;
        }
        root.set(key, block);
        if (!writeFile(path, root.dump(1))) {
            ctx.qa.fail("finaliseQa: cannot rewrite " + path);
            return false;
        }
        return true;
    };

    const bool manifestOk = rewrite(pathJoin(ctx.cfg.outDir, "world_manifest.json"),
                                    "quality", full);
    const bool payloadOk = rewrite("viewer/data/mumbai.json", "qa", summary);
    if (manifestOk && payloadOk) {
        MLOG_INFO("qa report published: %zu counters, %zu warnings, %zu failures — %s "
                  "(manifest 'quality', payload 'qa')",
                  ctx.qa.counters.size(), ctx.qa.warnings.size(), ctx.qa.failures.size(),
                  ctx.qa.ok() ? "PASS" : "FAIL");
    }
    return true;   // publication failure is reported through the QA report itself
}

}  // namespace mumbai::pipeline
