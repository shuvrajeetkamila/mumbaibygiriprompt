// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/output.cpp
//
//  Stage group 4: serialisation.
//    * writeChunks  — pack every cell into a .mumbai_chunk file and verify it by
//                     re-opening it with the runtime reader
//    * writeManifest— the world manifest the runtime and the viewer consume
//    * printReport  — human-readable build report
// =============================================================================
#include "pipeline/pipeline.h"

#include <ctime>

namespace mumbai::pipeline {

namespace {
std::string cellFileName(i64 relX, i64 relZ) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "cell_%04lld_%04lld.mumbai_chunk",
                  static_cast<long long>(relX), static_cast<long long>(relZ));
    return buf;
}
// -----------------------------------------------------------------------------
//  Roads graded into the drivable ground.
//
//  The chunk's heightfield is not only a query surface: the physics world builds
//  a Jolt height field from it and every vehicle in the game drives on it.  A
//  real road is *graded* — the ground under the carriageway is the road's own
//  level, not the terrain's — and the navigation graph already knows that level
//  (its nodes carry absolute road-surface heights).  Grading it in here is what
//  makes the two layers agree by construction instead of by tolerance: the
//  survey in D-034 found the model's own lanes disagreeing with the collision
//  ground by 0.3-1 m on 12 % of samples, which is exactly the error a car feels
//  as a wheel leaving the road.
//
//  Each segment is a straight run between two navigation nodes, the same line the
//  traffic model drives (`SimEdge`), so the graded surface and the model's road
//  are the same line rather than two approximations of it.
// -----------------------------------------------------------------------------
void gradeRoadsIntoGrid(const CellData& cd, u32 n, f32 step, std::vector<f32>& grid) {
    const size_t dim = static_cast<size_t>(n) + 1;
    if (cd.navNodes.size() < 2 || cd.navEdges.empty()) return;
    std::vector<f32> weight(dim * dim, 0.0f);
    std::vector<f32> target(dim * dim, 0.0f);

    const f64 kBlend = 10.0;   // metres over which a graded road rejoins the DEM
    for (const NavNode& u : cd.navNodes) {
        for (u32 k = 0; k < u.edgeCount; ++k) {
            const u32 ei = u.firstEdge + k;
            if (ei >= cd.navEdges.size()) break;
            const NavEdge& e = cd.navEdges[ei];
            if (e.toNode >= cd.navNodes.size()) continue;
            const NavNode& v = cd.navNodes[e.toNode];
            // The carriageway, plus its shoulders: a vehicle's wheels run on the
            // whole width, and the kerb is a step the collision should still see.
            const f64 lanes = std::max<f64>(e.laneCount == 0 ? 2 : e.laneCount, 1.0);
            const f64 flat = lanes * 3.5 * 0.5 + 1.5;
            const f64 ax = u.x, az = u.z, ay = u.y;
            const f64 bx = v.x, bz = v.z, by = v.y;
            const f64 dx = bx - ax, dz = bz - az;
            const f64 len2 = dx * dx + dz * dz;
            if (len2 < 1e-6) continue;
            const f64 reach = flat + kBlend;
            const i64 i0 = static_cast<i64>(std::floor((std::min(ax, bx) - reach) / step));
            const i64 i1 = static_cast<i64>(std::ceil((std::max(ax, bx) + reach) / step));
            const i64 j0 = static_cast<i64>(std::floor((std::min(az, bz) - reach) / step));
            const i64 j1 = static_cast<i64>(std::ceil((std::max(az, bz) + reach) / step));
            for (i64 i = std::max<i64>(0, i0); i <= std::min<i64>(n, i1); ++i) {
                for (i64 j = std::max<i64>(0, j0); j <= std::min<i64>(n, j1); ++j) {
                    const f64 px = i * step, pz = j * step;
                    f64 t = ((px - ax) * dx + (pz - az) * dz) / len2;
                    t = std::clamp(t, 0.0, 1.0);
                    const f64 cx = ax + dx * t, cz = az + dz * t;
                    const f64 d = std::hypot(px - cx, pz - cz);
                    if (d > reach) continue;
                    const f64 w = std::clamp((reach - d) / kBlend, 0.0, 1.0);
                    const size_t at = static_cast<size_t>(j) * dim + static_cast<size_t>(i);
                    if (w <= weight[at]) continue;
                    weight[at] = static_cast<f32>(w);
                    target[at] = static_cast<f32>(ay + (by - ay) * t);
                }
            }
        }
    }
    for (size_t i = 0; i < grid.size(); ++i) {
        if (weight[i] <= 0.0f) continue;
        const f32 w = weight[i] > 1.0f ? 1.0f : weight[i];
        grid[i] = grid[i] * (1.0f - w) + target[i] * w;
    }
}

} // namespace

bool writeChunks(Context& ctx) {
    ScopedTimer timer("writeChunks");

    const std::string cellsDir = pathJoin(ctx.cfg.outDir, "cells");
    ensureDir(cellsDir);

    const std::vector<u8> materialTable = ctx.materials.serialize();
    const std::vector<u8> emptyTerrainAO;   // AO is per-vertex in MeshVertex

    u64 written = 0, bytes = 0, verified = 0, failed = 0;
    u64 totalTriangles = 0;

    for (auto& [key, cd] : ctx.cells) {
        if (cd.empty()) continue;

        ChunkWriter writer(cd.cellX, cd.cellZ, ctx.sourceHash);
        writer.setHeightRange(cd.minHeight, cd.maxHeight);

        // --- ChunkInfo --------------------------------------------------------
        CellInfo info{};
        info.buildingCount = cd.counts.buildings;
        info.roadWayCount = cd.counts.roads;
        info.junctionCount = cd.counts.roadJunctions;
        info.waterPolyCount = cd.counts.waterPolys;
        info.railWayCount = cd.counts.railWays;
        info.landusePolyCount = cd.counts.landusePolys;
        info.instanceCount = cd.counts.instances;
        info.heroAssetCount = cd.counts.heroAssets;
        info.triangleCountLod0 = static_cast<u32>(cd.terrain[0].triangleCount() +
                                                  cd.buildings[0].triangleCount() +
                                                  cd.roads.triangleCount());
        info.triangleCountTotal = static_cast<u32>(cd.terrain[0].triangleCount() + cd.terrain[1].triangleCount() +
                                                   cd.terrain[2].triangleCount() +
                                                   cd.buildings[0].triangleCount() + cd.buildings[1].triangleCount() +
                                                   cd.buildings[2].triangleCount() + cd.buildings[3].triangleCount() +
                                                   cd.roads.triangleCount() + cd.water.triangleCount() +
                                                   cd.rail.triangleCount() + cd.hero.triangleCount() +
                                                   cd.impostors.triangleCount());
        info.minHeight = cd.minHeight;
        info.maxHeight = cd.maxHeight;
        info.centreX = static_cast<f32>(MACRO_CELL_SIZE * 0.5);
        info.centreZ = static_cast<f32>(MACRO_CELL_SIZE * 0.5);
        for (u32 z = 0; z < 16; ++z) info.zoneHistogram[z] = 0;
        info.buildSchema = 1;
        writer.addSection(SectionType::ChunkInfo, &info, sizeof(info), 1, sizeof(info));

        // --- Terrain heightfield (quantised u16 for runtime queries) ----------
        if (!cd.terrain[0].empty()) {
            // 8 m samples rather than 32 m: this grid is the ground a vehicle
            // drives on, and a road graded into a 32 m grid is not a road.  It
            // costs 33 KB per cell (31 MB over the city) and it is the difference
            // between wheels that follow the road and wheels that leave it.
            const u32 n = 128;
            const f32 minH = cd.minHeight, maxH = cd.maxHeight;
            const f32 range = std::max(1e-3f, maxH - minH);
            HeightfieldHeader hh{};
            hh.width = n + 1;
            hh.height = n + 1;
            hh.originX = 0.0f;
            hh.originZ = 0.0f;
            hh.stepX = static_cast<f32>(MACRO_CELL_SIZE / n);
            hh.stepZ = hh.stepX;
            hh.baseY = minH;
            hh.scaleY = range / 65535.0f;
            hh.encoding = 1;
            std::vector<u8> payload(sizeof(HeightfieldHeader) + static_cast<size_t>(n + 1) * (n + 1) * sizeof(u16));
            std::memcpy(payload.data(), &hh, sizeof(hh));
            u16* heights = reinterpret_cast<u16*>(payload.data() + sizeof(HeightfieldHeader));
            std::vector<f32> grid(static_cast<size_t>(n + 1) * (n + 1));
            for (u32 j = 0; j <= n; ++j) {
                for (u32 i = 0; i <= n; ++i) {
                    const f64 wx = cd.origin.x + i * hh.stepX;
                    const f64 wz = cd.origin.y + j * hh.stepZ;
                    grid[static_cast<size_t>(j) * (n + 1) + i] = ctx.terrainHF.sample(wx, wz);
                }
            }
            gradeRoadsIntoGrid(cd, n, hh.stepX, grid);
            for (size_t k = 0; k < grid.size(); ++k)
                heights[k] = static_cast<u16>(clamp((grid[k] - minH) / range, 0.0f, 1.0f) * 65535.0f);
            writer.addSection(SectionType::TerrainHeight, payload.data(), payload.size(),
                              (n + 1) * (n + 1), sizeof(u16));
        }

        // --- Mesh sections ----------------------------------------------------
        auto addMeshGroup = [&](SectionType type, const std::vector<const mesh::MeshBuilder*>& builders,
                                const std::vector<u32>& lods, const char* label) {
            std::vector<const mesh::MeshBuilder*> nonEmpty;
            std::vector<u32> nonEmptyLods;
            for (size_t i = 0; i < builders.size(); ++i) {
                if (builders[i]->empty()) continue;
                nonEmpty.push_back(builders[i]);
                nonEmptyLods.push_back(lods[i]);
            }
            if (nonEmpty.empty()) return;
            const std::vector<u8> payload = mesh::serializeMeshGroup(nonEmpty, nonEmptyLods);
            u64 triangles = 0;
            for (const mesh::MeshBuilder* mb : nonEmpty) triangles += mb->triangleCount();
            totalTriangles += triangles;
            writer.addSection(type, payload.data(), payload.size(),
                              static_cast<u32>(nonEmpty.size()), 0);
            (void)label;
        };

        addMeshGroup(SectionType::TerrainMesh,
                     {&cd.terrain[0], &cd.terrain[1], &cd.terrain[2]}, {0, 1, 2}, "terrain");
        addMeshGroup(SectionType::RoadMesh, {&cd.roads}, {0}, "roads");
        addMeshGroup(SectionType::BuildingMesh,
                     {&cd.buildings[0], &cd.buildings[1], &cd.buildings[2]}, {0, 1, 2}, "buildings");
        addMeshGroup(SectionType::BuildingImpostor, {&cd.impostors}, {3}, "impostors");
        addMeshGroup(SectionType::WaterMesh, {&cd.water}, {0}, "water");
        addMeshGroup(SectionType::RailMesh, {&cd.rail}, {0}, "rail");
        addMeshGroup(SectionType::HeroAssets, {&cd.hero}, {0}, "hero");

        // Terrain collision mirrors the coarsest terrain LOD: physics queries
        // use the heightfield section, and triangle collision is only needed
        // where the player can walk or a vehicle can leave the road.
        if (!cd.terrain[2].empty()) {
            const std::vector<u8> payload = cd.terrain[2].serialize(2);
            writer.addSection(SectionType::TerrainCollision, payload.data(), payload.size(), 1, 0);
        }

        // --- Surface material regions ----------------------------------------
        if (!cd.regions.empty()) {
            struct RegionHeader { u32 width, height; f32 originX, originZ, step; };
            RegionHeader rh{};
            rh.width = cd.regionWidth;
            rh.height = cd.regionHeight;
            rh.originX = 0;
            rh.originZ = 0;
            rh.step = static_cast<f32>(MACRO_CELL_SIZE / cd.regionWidth);
            std::vector<u8> payload(sizeof(RegionHeader) + cd.regions.size());
            std::memcpy(payload.data(), &rh, sizeof(rh));
            std::memcpy(payload.data() + sizeof(rh), cd.regions.data(), cd.regions.size());
            writer.addSection(SectionType::MaterialRegions, payload.data(), payload.size(),
                              static_cast<u32>(cd.regions.size()), 1);
        }

        // --- Navigation graph -------------------------------------------------
        if (!cd.navNodes.empty()) {
            NavGraphHeader nh{};
            nh.nodeCount = static_cast<u32>(cd.navNodes.size());
            nh.edgeCount = static_cast<u32>(cd.navEdges.size());
            nh.signalCount = static_cast<u32>(cd.signals.size());
            std::vector<u8> payload(sizeof(NavGraphHeader) + cd.navNodes.size() * sizeof(NavNode) +
                                    cd.navEdges.size() * sizeof(NavEdge) +
                                    cd.signals.size() * sizeof(NavSignalPhase));
            size_t cursor = 0;
            std::memcpy(payload.data() + cursor, &nh, sizeof(nh)); cursor += sizeof(nh);
            std::memcpy(payload.data() + cursor, cd.navNodes.data(),
                        cd.navNodes.size() * sizeof(NavNode)); cursor += cd.navNodes.size() * sizeof(NavNode);
            std::memcpy(payload.data() + cursor, cd.navEdges.data(),
                        cd.navEdges.size() * sizeof(NavEdge)); cursor += cd.navEdges.size() * sizeof(NavEdge);
            if (!cd.signals.empty())
                std::memcpy(payload.data() + cursor, cd.signals.data(),
                            cd.signals.size() * sizeof(NavSignalPhase));
            writer.addSection(SectionType::RoadNavGraph, payload.data(), payload.size(), nh.nodeCount,
                              sizeof(NavNode), 1,
                              SectionFlagMandatory | SectionFlagPrefixed);
        }

        // --- Collision --------------------------------------------------------
        if (!cd.collision.empty()) {
            writer.addSection(SectionType::BuildingCollision, cd.collision.data(),
                              cd.collision.size() * sizeof(CollisionBox),
                              static_cast<u32>(cd.collision.size()), sizeof(CollisionBox));
        }

        // --- Building proxies (oriented massing blocks) -----------------------
        // The distance-LOD and impostor passes read these directly; they carry
        // the footprint orientation the AABB collider necessarily discards.
        if (!cd.proxies.empty()) {
            writer.addSection(SectionType::BuildingProxy, cd.proxies.data(),
                              cd.proxies.size() * sizeof(BuildingProxy),
                              static_cast<u32>(cd.proxies.size()), sizeof(BuildingProxy));
        }

        // --- Land use ---------------------------------------------------------
        if (!cd.landusePolys.empty()) {
            LanduseHeader lh{};
            lh.polygonCount = static_cast<u32>(cd.landusePolys.size());
            lh.totalVertices = static_cast<u32>(cd.landuseVerts.size());
            std::vector<u8> payload(sizeof(LanduseHeader) +
                                    cd.landusePolys.size() * sizeof(LandusePoly) +
                                    cd.landuseVerts.size() * sizeof(f32) * 2);
            size_t cursor = 0;
            std::memcpy(payload.data() + cursor, &lh, sizeof(lh)); cursor += sizeof(lh);
            std::memcpy(payload.data() + cursor, cd.landusePolys.data(),
                        cd.landusePolys.size() * sizeof(LandusePoly));
            cursor += cd.landusePolys.size() * sizeof(LandusePoly);
            for (const Vec2d& v : cd.landuseVerts) {
                const f32 pair[2] = {static_cast<f32>(v.x - cd.origin.x),
                                     static_cast<f32>(v.y - cd.origin.y)};
                std::memcpy(payload.data() + cursor, pair, sizeof(pair));
                cursor += sizeof(pair);
            }
            writer.addSection(SectionType::LandusePolys, payload.data(), payload.size(),
                              lh.polygonCount, sizeof(LandusePoly));
        }

        // --- Instances --------------------------------------------------------
        if (!cd.instances.empty()) {
            writer.addSection(SectionType::InstanceTable, cd.instances.data(),
                              cd.instances.size() * sizeof(InstanceRecord),
                              static_cast<u32>(cd.instances.size()), sizeof(InstanceRecord));
        }

        // --- Material table (every chunk is self-contained) -------------------
        writer.addSection(SectionType::MaterialTable, materialTable.data(), materialTable.size(),
                          static_cast<u32>(ctx.materials.count()), sizeof(MaterialDef));

        // --- QA -----------------------------------------------------------------
        if (!ctx.qa.counters.empty()) {
            writer.addSection(SectionType::QAReport, ctx.qa.counters.data(),
                              ctx.qa.counters.size() * sizeof(QaCounter),
                              static_cast<u32>(ctx.qa.counters.size()), sizeof(QaCounter));
        }

        const std::string path = pathJoin(cellsDir, cellFileName(cd.cellX - ctx.originCell.x,
                                                                 cd.cellZ - ctx.originCell.z));
        const u64 size = writer.write(path);
        if (size == 0) {
            ctx.qa.fail("chunk write failed: " + path);
            ++failed;
            continue;
        }
        ++written;
        bytes += size;

        // --- Verification: reopen with the runtime reader and hash-check ------
        if (ctx.cfg.verifyChunks) {
            ChunkReader reader;
            std::string err;
            if (!reader.open(path, &err)) {
                ctx.qa.fail("chunk reopen failed: " + path + " — " + err);
                ++failed;    // a chunk that cannot be reopened is a failed chunk
            } else {
                bool ok = true;
                if (reader.header().cellX != cd.cellX || reader.header().cellZ != cd.cellZ) {
                    ctx.qa.fail("chunk cell id mismatch: " + path);
                    ok = false;
                }
                if (!reader.verifyHashes(&err)) {
                    // Every section is hashed on load: a mismatch means the file
                    // on disk is not the file the writer believed it wrote, so
                    // this counts as a failure (it used to be recorded only as a
                    // QA message, which is why the counters read 0 while every
                    // chunk was in fact failing verification).
                    ctx.qa.fail("chunk hash mismatch: " + path + " — " + err);
                    ok = false;
                }
                // Spot-check that the mesh views resolve and stay in bounds.
                for (u32 i = 0; i < reader.meshCount(SectionType::BuildingMesh); ++i) {
                    const auto view = reader.mesh(SectionType::BuildingMesh, i);
                    if (!view.valid()) {
                        ctx.qa.fail("building mesh view invalid: " + path);
                        ok = false;
                        break;
                    }
                }
                if (ok) ++verified; else ++failed;
            }
        }
    }

    // --- Source hash sidecar --------------------------------------------------
    char hashLine[64];
    std::snprintf(hashLine, sizeof(hashLine), "%016llx", static_cast<unsigned long long>(ctx.sourceHash));
    writeFile(pathJoin(ctx.cfg.outDir, "source_hash.txt"), std::string(hashLine) + "\n");

    ctx.stats.chunkCount = written;
    ctx.stats.chunkBytes = bytes;
    ctx.stats.totalTriangles = totalTriangles;

    MLOG_INFO("chunks: %llu written (%.1f MB), %llu verified, %llu failed",
              (unsigned long long)written, bytes / 1e6,
              (unsigned long long)verified, (unsigned long long)failed);
    ctx.qa.add("chunks.written", static_cast<f64>(written));
    ctx.qa.add("chunks.bytes", static_cast<f64>(bytes));
    ctx.qa.add("chunks.verified", static_cast<f64>(verified));
    ctx.qa.add("chunks.failed", static_cast<f64>(failed), failed > 0 ? 2u : 0u);
    return failed == 0;
}

// -----------------------------------------------------------------------------
bool writeManifest(Context& ctx) {
    ScopedTimer timer("writeManifest");

    Json root = Json::object();
    root.set("format", Json(std::string("mumbai-world-manifest")));
    root.set("formatVersion", Json(1));
    root.set("schema", Json(1));

    // --- projection ---------------------------------------------------------
    Json proj = Json::object();
    proj.set("name", Json(std::string("TransverseMercator")));
    proj.set("latOriginDeg", Json(ctx.proj.params().latOriginDeg));
    proj.set("lonOriginDeg", Json(ctx.proj.params().lonOriginDeg));
    proj.set("scaleFactor", Json(ctx.proj.params().k0));
    proj.set("semiMajorAxis", Json(ctx.proj.params().a));
    proj.set("flattening", Json(ctx.proj.params().f));
    proj.set("units", Json(std::string("metres")));
    root.set("projection", proj);

    // --- extent -------------------------------------------------------------
    Json extent = Json::object();
    extent.set("minX", Json(ctx.extent.min.x));
    extent.set("minZ", Json(ctx.extent.min.y));
    extent.set("maxX", Json(ctx.extent.max.x));
    extent.set("maxZ", Json(ctx.extent.max.y));
    extent.set("sizeX", Json(ctx.extent.size().x));
    extent.set("sizeZ", Json(ctx.extent.size().y));
    extent.set("squareKilometres", Json(ctx.extent.size().x * ctx.extent.size().y / 1e6));
    extent.set("macroCellSize", Json(static_cast<f64>(MACRO_CELL_SIZE)));
    root.set("extent", extent);

    Json origin = Json::object();
    origin.set("cellX", Json(static_cast<i64>(ctx.originCell.x)));
    origin.set("cellZ", Json(static_cast<i64>(ctx.originCell.z)));
    origin.set("worldOriginX", Json(static_cast<f64>(ctx.originCell.x) * MACRO_CELL_SIZE));
    origin.set("worldOriginZ", Json(static_cast<f64>(ctx.originCell.z) * MACRO_CELL_SIZE));
    root.set("origin", origin);

    // --- source provenance --------------------------------------------------
    Json sources = Json::object();
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(ctx.sourceHash));
        sources.set("osmSourceHash", Json(std::string(buf)));
    }
    sources.set("osmFile", Json(std::string("data/raw/mumbai_region.osm")));
    sources.set("wardsFile", Json(std::string("data/raw/bmc_wards.geojson")));
    sources.set("slumsFile", Json(std::string("data/raw/slum_clusters.geojson")));
    sources.set("terrainControlPoints", Json(std::string("data/authored/terrain_control_points.json")));
    root.set("sources", sources);

    // --- cells --------------------------------------------------------------
    Json cells = Json::array();
    u64 totalBytes = 0;
    for (const auto& [key, cd] : ctx.cells) {
        if (cd.empty()) continue;
        Json c = Json::object();
        c.set("x", Json(static_cast<i64>(cd.cellX)));
        c.set("z", Json(static_cast<i64>(cd.cellZ)));
        c.set("relX", Json(static_cast<i64>(cd.cellX - ctx.originCell.x)));
        c.set("relZ", Json(static_cast<i64>(cd.cellZ - ctx.originCell.z)));
        c.set("file", Json(cellFileName(cd.cellX - ctx.originCell.x, cd.cellZ - ctx.originCell.z)));
        c.set("bytes", Json(static_cast<i64>(fileSize(pathJoin(pathJoin(ctx.cfg.outDir, "cells"),
                                                              cellFileName(cd.cellX - ctx.originCell.x,
                                                                           cd.cellZ - ctx.originCell.z))))));
        c.set("buildings", Json(static_cast<i64>(cd.counts.buildings)));
        c.set("roads", Json(static_cast<i64>(cd.counts.roads)));
        c.set("instances", Json(static_cast<i64>(cd.counts.instances)));
        Json tris = Json::object();
        tris.set("terrain", Json(static_cast<i64>(cd.terrain[0].triangleCount())));
        tris.set("roads", Json(static_cast<i64>(cd.roads.triangleCount())));
        tris.set("buildings", Json(static_cast<i64>(cd.buildings[0].triangleCount())));
        tris.set("water", Json(static_cast<i64>(cd.water.triangleCount())));
        tris.set("rail", Json(static_cast<i64>(cd.rail.triangleCount())));
        tris.set("hero", Json(static_cast<i64>(cd.hero.triangleCount())));
        c.set("triangles", std::move(tris));
        c.set("minHeight", Json(static_cast<f64>(cd.minHeight)));
        c.set("maxHeight", Json(static_cast<f64>(cd.maxHeight)));
        totalBytes += fileSize(pathJoin(pathJoin(ctx.cfg.outDir, "cells"),
                                        cellFileName(cd.cellX - ctx.originCell.x,
                                                     cd.cellZ - ctx.originCell.z)));
        cells.push(std::move(c));
    }
    root.setArray("cells", std::move(cells));

    // --- totals -------------------------------------------------------------
    Json totals = Json::object();
    totals.set("cellCount", Json(static_cast<i64>(ctx.stats.chunkCount)));
    totals.set("bytes", Json(static_cast<i64>(totalBytes)));
    totals.set("buildings", Json(static_cast<i64>(ctx.stats.buildingCount)));
    totals.set("roadWays", Json(static_cast<i64>(ctx.stats.roadWays)));
    totals.set("roadCentrelineMetres", Json(static_cast<i64>(ctx.stats.roadLengthMetres)));
    totals.set("junctions", Json(static_cast<i64>(ctx.stats.junctions)));
    totals.set("navNodes", Json(static_cast<i64>(ctx.stats.navNodes)));
    totals.set("navEdges", Json(static_cast<i64>(ctx.stats.navEdges)));
    totals.set("collisionBoxes", Json(static_cast<i64>(ctx.stats.collisionBoxes)));
    totals.set("instances", Json(static_cast<i64>(ctx.stats.instanceCount)));
    totals.set("heroAssets", Json(static_cast<i64>(ctx.stats.heroAssetCount)));
    totals.set("landusePolygons", Json(static_cast<i64>(ctx.stats.landusePolys)));
    totals.set("railWays", Json(static_cast<i64>(ctx.stats.railWays)));
    totals.set("triangles", Json(static_cast<i64>(ctx.stats.totalTriangles)));
    totals.set("terrainMinHeight", Json(ctx.stats.terrainMinHeight));
    totals.set("terrainMaxHeight", Json(ctx.stats.terrainMaxHeight));
    root.set("totals", totals);

    Json zones = Json::object();
    for (const auto& [name, count] : ctx.stats.zoneCounts) zones.set(name, Json(static_cast<i64>(count)));
    root.set("zones", std::move(zones));

    Json surfaces = Json::object();
    for (const auto& [name, count] : ctx.stats.surfaceCounts) surfaces.set(name, Json(static_cast<i64>(count)));
    root.set("surfaces", std::move(surfaces));

    // --- drainage basins (the monsoon flood model's input) -------------------
    Json basins = Json::array();
    for (size_t i = 0; i < std::min<size_t>(ctx.lowPoints.size(), 64); ++i) {
        const auto& lp = ctx.lowPoints[i];
        Json b = Json::object();
        b.set("x", Json(lp.position.x));
        b.set("z", Json(lp.position.y));
        b.set("height", Json(static_cast<f64>(lp.height)));
        b.set("areaSquareMetres", Json(lp.basinArea));
        basins.push(std::move(b));
    }
    root.setArray("drainageBasins", std::move(basins));

    // --- QA -----------------------------------------------------------------
    // Snapshot of the report at manifest-write time.  finaliseQa() rewrites it
    // once the build is over so the file on disk carries the finished numbers.
    root.set("quality", qaReportJson(ctx.qa));

    const std::string path = pathJoin(ctx.cfg.outDir, "world_manifest.json");
    const std::string text = root.dump(1);
    if (!writeFile(path, text)) {
        MLOG_ERR("manifest write failed: %s", path.c_str());
        return false;
    }
    MLOG_INFO("manifest: %s (%.2f MB, %llu cells)", path.c_str(), text.size() / 1e6,
              (unsigned long long)ctx.stats.chunkCount);
    return true;
}

// -----------------------------------------------------------------------------
Json qaReportJson(const QaReport& qa) {
    Json out = Json::object();
    Json counters = Json::array();
    for (const auto& c : qa.counters) {
        Json j = Json::object();
        j.set("name", Json(std::string(c.name)));
        j.set("value", Json(c.value));
        j.set("severity", Json(static_cast<i64>(c.severity)));
        counters.push(std::move(j));
    }
    out.setArray("counters", std::move(counters));
    Json warnings = Json::array();
    for (const auto& w : qa.warnings) warnings.push(Json(w));
    out.setArray("warnings", std::move(warnings));
    Json failures = Json::array();
    for (const auto& f : qa.failures) failures.push(Json(f));
    out.setArray("failures", std::move(failures));
    out.set("ok", Json(qa.ok()));
    return out;
}

void printReport(const Context& ctx) {
    const auto& s = ctx.stats;
    std::printf("\n");
    std::printf("=======================================================================\n");
    std::printf(" MUMBAI 1:1 EXACT — world build report\n");
    std::printf("=======================================================================\n");
    std::printf(" extent            : %.0f x %.0f m  (%.1f km^2)\n",
                ctx.extent.size().x, ctx.extent.size().y,
                ctx.extent.size().x * ctx.extent.size().y / 1e6);
    std::printf(" macro cells       : %llu x %.0f m\n",
                (unsigned long long)s.chunkCount, static_cast<f64>(MACRO_CELL_SIZE));
    std::printf(" storage           : %.2f GB (%.2f MB) in %llu chunk files\n",
                s.chunkBytes / 1e9, s.chunkBytes / 1e6, (unsigned long long)s.chunkCount);
    std::printf("------------------------------------------------------------------------\n");
    std::printf(" buildings         : %llu  (%llu LOD0 triangles, %llu collision proxies)\n",
                (unsigned long long)s.buildingCount, (unsigned long long)s.buildingTriangles,
                (unsigned long long)s.collisionBoxes);
    std::printf(" road network      : %llu ways, %.1f km centreline, %llu junctions\n",
                (unsigned long long)s.roadWays, s.roadLengthMetres / 1000.0,
                (unsigned long long)s.junctions);
    std::printf(" navigation graph  : %llu nodes, %llu edges\n",
                (unsigned long long)s.navNodes, (unsigned long long)s.navEdges);
    std::printf(" railway           : %llu alignments, %llu triangles\n",
                (unsigned long long)s.railWays, (unsigned long long)s.railTriangles);
    std::printf(" water             : %llu triangles\n", (unsigned long long)s.waterTriangles);
    std::printf(" terrain           : %llu LOD0 triangles, elevation %.2f .. %.2f m\n",
                (unsigned long long)s.terrainTriangles, s.terrainMinHeight, s.terrainMaxHeight);
    std::printf(" props / instances : %llu\n", (unsigned long long)s.instanceCount);
    std::printf(" hero assets       : %llu (%llu triangles)\n",
                (unsigned long long)s.heroAssetCount, (unsigned long long)s.heroTriangles);
    std::printf(" landuse polygons  : %llu\n", (unsigned long long)s.landusePolys);
    std::printf(" total triangles   : %llu\n", (unsigned long long)s.totalTriangles);
    std::printf("------------------------------------------------------------------------\n");
    std::printf(" zones:\n");
    for (const auto& [name, count] : s.zoneCounts)
        std::printf("   %-24s %8llu buildings\n", name.c_str(), (unsigned long long)count);
    std::printf(" surfaces:\n");
    for (const auto& [name, count] : s.surfaceCounts)
        std::printf("   %-24s %8llu ways\n", name.c_str(), (unsigned long long)count);
    std::printf("------------------------------------------------------------------------\n");
    std::printf(" QA: %zu counters, %zu warnings, %zu failures — %s\n",
                ctx.qa.counters.size(), ctx.qa.warnings.size(), ctx.qa.failures.size(),
                ctx.qa.ok() ? "PASS" : "FAIL");
    for (const auto& w : ctx.qa.warnings) std::printf("   [warn]  %s\n", w.c_str());
    for (const auto& f : ctx.qa.failures) std::printf("   [FAIL]  %s\n", f.c_str());
    std::printf("=======================================================================\n\n");
}

} // namespace mumbai::pipeline
