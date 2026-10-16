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
//      nodes     6 B each     u16 x, u16 z, u16 y? (metres from extent min; x and z
//                             are non-negative because the payload is clipped to
//                             the extent, and the world is 43 km tall — an i16
//                             would clamp the northern third of the street graph
//                             onto one line)
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
    u64 clipNodes = 0, clipEdges = 0;
    std::vector<u32> edgeFrom, remap;      // per chunk, reused so the export never allocates
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
            // The published grid is `TN` nodes per cell; the chunk's heightfield
            // is not — it is the *collision* grid, and today that is 129 samples
            // at 8 m (n = 128).  Reading it with `i` as the published index (the
            // first version did) took the first 33 samples of the cell and
            // stretched them across all 1024 m of it: the whole terrain raster was
            // a quarter-cell strip magnified four times, and the last 768 m of
            // every cell was never read at all.  The stride is derived, not
            // assumed, and it is what every later consumer of this raster — the
            // viewer, and the flood QA's own sea test — has been measuring (D-035).
            const u32 strideX = hh.width > 1 ? std::max<u32>(1, (hh.width - 1) / TN) : 1;
            const u32 strideZ = hh.height > 1 ? std::max<u32>(1, (hh.height - 1) / TN) : 1;
            for (u32 j = 0; j <= TN && j * strideZ < hh.height; ++j) {
                for (u32 i = 0; i <= TN && i * strideX < hh.width; ++i) {
                    const u32 gx = static_cast<u32>(ti) * TN + i;
                    const u32 gz = static_cast<u32>(tj) * TN + j;
                    if (gx >= TW || gz >= TH) continue;
                    const f32 h = hh.baseY +
                                  hgrid[(j * strideZ) * hh.width + (i * strideX)] * hh.scaleY;
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
            // Which node each edge leaves from.  A NavEdge names only where it
            // goes — the record is deliberately small — so its origin is the node
            // whose out-range contains it.  Writing the edge's own index into that
            // field, which is what this stage did at first, is right only by
            // accident; the payload verifier caught the 3,683 edges where it was
            // not, 3,631 of them feeding straight back into themselves.
            //
            // remap holds *global* indices into the payload's node array, which is
            // why the writes below use it directly and add nothing to it: as
            // `nodeBase + remap[...]` the same value was counted twice, and the
            // second and later chunks sent two fifths of the graph's edges to
            // node indices that do not exist (max 248,518 for 124,348 nodes).
            edgeFrom.assign(gh.edgeCount, 0xFFFFFFFFu);
            for (u32 i = 0; i < gh.nodeCount; ++i) {
                for (u32 k = 0; k < nn[i].edgeCount; ++k) {
                    const u32 idx = nn[i].firstEdge + k;
                    if (idx < gh.edgeCount) edgeFrom[idx] = i;
                }
            }
            // Clip to the extent.  The payload is the world the viewer draws, and
            // the world is the box the manifest publishes: 4,221 nodes of the
            // source graph lie outside it (roads that arrive from the region
            // beyond the box), and a node outside the canvas has no place on it.
            // Their edges go with them, which is why the edges are remapped rather
            // than renumbered in place.
            remap.assign(gh.nodeCount, 0xFFFFFFFFu);
            for (u32 i = 0; i < gh.nodeCount; ++i) {
                const f64 wx = cellOX + static_cast<f64>(nn[i].x);
                const f64 wz = cellOZ + static_cast<f64>(nn[i].z);
                if (wx < minX - 1.0 || wx > maxX + 1.0 || wz < minZ - 1.0 || wz > maxZ + 1.0) {
                    ++clipNodes;
                    continue;
                }
                remap[i] = static_cast<u32>(nodeCount);
                nodes.u16v(static_cast<u16>(std::lround(std::clamp(wx - minX, 0.0, 65535.0))));
                nodes.u16v(static_cast<u16>(std::lround(std::clamp(wz - minZ, 0.0, 65535.0))));
                nodes.i16v(static_cast<i16>(std::lround(std::clamp(static_cast<f64>(nn[i].y), -32768.0, 32767.0))));
                ++nodeCount;
            }
            for (u32 i = 0; i < gh.edgeCount; ++i) {
                const u32 from = edgeFrom[i];
                const u32 to = ne[i].toNode;
                if (from >= gh.nodeCount || to >= gh.nodeCount ||
                    remap[from] == 0xFFFFFFFFu || remap[to] == 0xFFFFFFFFu) {
                    ++clipEdges;
                    continue;
                }
                edges.u32v(remap[from]);
                edges.u32v(remap[to]);
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
                        // A8: the texture the material samples, as a tile index
                        // into the base-colour atlas the export copies alongside
                        // this payload, and the physical size of one tile.  -1
                        // means the material is untextured (id 0, "unknown").
                        const i32 layer = ctx.materials.textureLayerIndex(i);
                        m.set("tex", Json(static_cast<f64>(layer)));
                        m.set("tileMetres", Json(static_cast<f64>(ctx.materials.tileMetres(i))));
                        // The mean linear albedo of the layer.  The shader divides
                        // the sampled tile by it, so the quotient averages 1 and the
                        // surface keeps whatever colour it already had — which is
                        // what lets 74 tinted facade variants share their base
                        // material's texture without losing their tint.
                        f32 mean[3] = {0, 0, 0};
                        if (ctx.materials.textureLayerMean(layer, mean)) {
                            m.set("tileMeanR", Json(static_cast<f64>(mean[0])));
                            m.set("tileMeanG", Json(static_cast<f64>(mean[1])));
                            m.set("tileMeanB", Json(static_cast<f64>(mean[2])));
                        }
                        materials.push(m);
                    }
                }
            }
        }
    }

    // --- monsoon flood raster -------------------------------------------------
    // Peak severe-storm depth, in centimetres, on the same grid as the terrain so
    // the viewer can drape it directly.  This is the Section 3 deliverable made
    // visible: the water that emerges from the DEM at Hindmata, King's Circle and
    // the Andheri subway rather than being painted on.
    // Both design storms travel: the routine monsoon day the city lives with and
    // the 26 July 2005 analogue it does not.  Two rasters cost a megabyte and
    // turn "it floods" into something a reader can compare against their own
    // street.
    struct FloodStats { u64 wet = 0, deep = 0; f64 peakCm = 0.0; };
    const auto buildFloodRaster = [&](const std::vector<u8>& src, std::vector<u8>& dst) {
        FloodStats st;
        if (src.empty()) return st;
        const u32 fw = ctx.floodWidth, fh = ctx.floodHeight;
        const f64 fstep = ctx.floodStep;
        dst.assign(static_cast<size_t>(TW) * TH, 0u);
        for (u32 gz = 0; gz < TH; ++gz) {
            for (u32 gx = 0; gx < TW; ++gx) {
                const f64 x = minX + (maxX - minX) * (gx / static_cast<f64>(TW - 1));
                const f64 z = minZ + (maxZ - minZ) * (gz / static_cast<f64>(TH - 1));
                const i64 fi = static_cast<i64>((x - minX) / fstep);
                const i64 fj = static_cast<i64>((z - minZ) / fstep);
                if (fi < 0 || fj < 0 || fi >= static_cast<i64>(fw) || fj >= static_cast<i64>(fh)) continue;
                const u8 v = src[static_cast<size_t>(fj) * fw + fi];
                const f64 cm = v == 255 ? 255.0 : static_cast<f64>(v);
                if (cm > 1.0) { ++st.wet; if (cm > 50.0) ++st.deep; }
                if (cm > st.peakCm) st.peakCm = cm;
                dst[static_cast<size_t>(gz) * TW + gx] = v;
            }
        }
        return st;
    };
    std::vector<u8> flood, floodMod;
    const FloodStats floodStats = buildFloodRaster(ctx.floodSevere, flood);
    const FloodStats floodModStats = buildFloodRaster(ctx.floodModerate, floodMod);
    const u64 floodWet = floodStats.wet, floodDeep = floodStats.deep;
    const f64 floodPeakCm = floodStats.peakCm;

    BinWriter out;
    const u64 terrainBytes = static_cast<u64>(terrain.size()) * sizeof(u16);
    const u64 terrainOffset = 0;
    const u64 buildingsOffset = terrainOffset + terrainBytes;
    const u64 nodesOffset = buildingsOffset + buildings.size();
    const u64 edgesOffset = nodesOffset + nodes.size();
    const u64 heroVertOffset = edgesOffset + edges.size();
    const u64 heroIdxOffset = heroVertOffset + heroVerts.size();
    const u64 floodOffset = heroIdxOffset + heroIdx.size();
    const u64 floodModOffset = floodOffset + flood.size();
    out.data.reserve(static_cast<size_t>(floodModOffset + floodMod.size()));
    out.data.insert(out.data.end(), reinterpret_cast<const u8*>(terrain.data()),
                    reinterpret_cast<const u8*>(terrain.data() + terrain.size()));
    out.data.insert(out.data.end(), buildings.data.begin(), buildings.data.end());
    out.data.insert(out.data.end(), nodes.data.begin(), nodes.data.end());
    out.data.insert(out.data.end(), edges.data.begin(), edges.data.end());
    out.data.insert(out.data.end(), heroVerts.data.begin(), heroVerts.data.end());
    out.data.insert(out.data.end(), heroIdx.data.begin(), heroIdx.data.end());
    out.data.insert(out.data.end(), flood.begin(), flood.end());
    out.data.insert(out.data.end(), floodMod.begin(), floodMod.end());

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

    Json fl = Json::object();
    fl.set("width", Json(static_cast<f64>(TW)));
    fl.set("height", Json(static_cast<f64>(TH)));
    fl.set("offset", Json(static_cast<f64>(floodOffset)));
    fl.set("units", Json(std::string("cm")));          // value = peak depth in cm
    fl.set("storm", Json(std::string("severe")));
    fl.set("wetCells", Json(static_cast<f64>(floodWet)));
    fl.set("cellsOver50cm", Json(static_cast<f64>(floodDeep)));
    fl.set("peakCm", Json(floodPeakCm));
    meta.set("flood", fl);

    Json fm = Json::object();
    fm.set("width", Json(static_cast<f64>(TW)));
    fm.set("height", Json(static_cast<f64>(TH)));
    fm.set("offset", Json(static_cast<f64>(floodModOffset)));
    fm.set("units", Json(std::string("cm")));
    fm.set("storm", Json(std::string("moderate")));
    fm.set("wetCells", Json(static_cast<f64>(floodModStats.wet)));
    fm.set("cellsOver50cm", Json(static_cast<f64>(floodModStats.deep)));
    fm.set("peakCm", Json(floodModStats.peakCm));
    meta.set("floodModerate", fm);

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

    // --- texture atlas (A8) ---------------------------------------------------
    // The browser preview gets the procedural material library as one packed
    // image (the runtime reads the .mumbaitex arrays instead).  The grid is
    // published with it so the viewer can address a tile per material without
    // guessing, and the atlas itself is copied next to the payload.
    {
        Json atlas = Json::object();
        const u32 textured = ctx.materials.textureLayerCount();
        if (textured) {
            const u32 cols = static_cast<u32>(std::ceil(std::sqrt(static_cast<f64>(textured))));
            const u32 rows = (textured + cols - 1) / cols;
            atlas.set("url", Json(std::string("data/atlas_basecolor.png")));
            atlas.set("cols", Json(static_cast<f64>(cols)));
            atlas.set("rows", Json(static_cast<f64>(rows)));
            atlas.set("count", Json(static_cast<f64>(textured)));
            atlas.set("tileSize", Json(static_cast<f64>(ctx.materials.textureAtlasTile())));
            atlas.set("pixels", Json(static_cast<f64>(ctx.cfg.textureSize)));
            const std::string src = ctx.cfg.outDir + "/textures/atlas_basecolor.png";
            const std::string dst = dataDir + "/atlas_basecolor.png";
            const std::string bytes = readFileToString(src);
            if (bytes.empty() || !writeFile(dst, bytes.data(), bytes.size()))
                ctx.qa.warn("texture atlas could not be copied into the viewer payload");
        }
        meta.set("atlas", atlas);
    }

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
    if (clipNodes || clipEdges)
        MLOG_INFO("viewer export: clipped %llu nodes and %llu edges outside the extent",
                  (unsigned long long)clipNodes, (unsigned long long)clipEdges);
    MLOG_INFO("viewer export: %zu materials, %llu hero vertices", materials.size(),
              (unsigned long long)heroVertexCount);
    ctx.qa.add("viewer.exportBuildings", static_cast<f64>(buildingCount));
    ctx.qa.add("viewer.exportFallbackCells", static_cast<f64>(fallbackCells));
    ctx.qa.add("viewer.exportBytes", static_cast<f64>(out.size()));
    ctx.qa.add("viewer.exportClippedNodes", static_cast<f64>(clipNodes));
    ctx.qa.add("viewer.exportClippedEdges", static_cast<f64>(clipEdges));
    ctx.qa.add("viewer.exportFloodWetCells", static_cast<f64>(floodWet));
    ctx.qa.add("viewer.exportFloodPeakCm", floodPeakCm);
    MLOG_INFO("viewer export: flood rasters — severe %llu wet cells / peak %.0f cm, "
              "moderate %llu / peak %.0f cm",
              (unsigned long long)floodWet, floodPeakCm,
              (unsigned long long)floodModStats.wet, floodModStats.peakCm);
    if (!ctx.floodModerate.empty() && floodModStats.wet == 0)
        ctx.qa.fail("viewer: the moderate storm flooded nothing at all");
    if (!ctx.floodSevere.empty() && floodWet == 0)
        ctx.qa.fail("viewer: the flood raster is empty - the severe storm wetted nothing");
    if (!ctx.floodSevere.empty() && floodPeakCm < 50.0)
        ctx.qa.warn("viewer: the severe storm peaked below 50 cm everywhere");
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
    ctx.qa.addTiming("stage.finaliseQa.seconds", ScopedTimer::now() - stageStart);

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
