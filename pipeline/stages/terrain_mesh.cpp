// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/terrain_mesh.cpp
//
//  Converts the heightfield into per-cell, per-LOD triangle meshes, a quantised
//  heightfield for runtime queries, a surface-material region mask, and baked
//  ambient occlusion.
//
//  Tiling rule: every cell mesh covers exactly its 1024 m tile with
//  samplesPerCell segments, so neighbouring chunks share edge vertices to the
//  millimetre and there are no cracks or T-junctions between chunks. LOD
//  transitions happen per cell; adjacent cells at different LOD do not share
//  edge vertices, so the runtime skirt hides the sub-metre gap (documented in
//  DOCS/03_CHUNK_FORMAT.md).
// =============================================================================
#include "pipeline/pipeline.h"

namespace mumbai::pipeline {

bool emitTerrainMeshes(Context& ctx) {
    if (!ctx.cfg.buildTerrain) return true;
    ScopedTimer timer("emitTerrainMeshes");

    const u32 matRock = ctx.materials.id("rock");
    const u32 matEarth = ctx.materials.id("earth");
    const u32 matSand = ctx.materials.id("sand");
    const u32 matReclaim = ctx.materials.id("reclamation_fill");
    const u32 matMud = ctx.materials.id("wetland_mud");
    const u32 matSeaFloor = ctx.materials.id("wetland_mud");

    // LOD sample counts per cell edge (power-of-two so cells tile exactly).
    const u32 lodSamples[3] = {32, 16, 8};

    const i64 cx0 = static_cast<i64>(std::floor(ctx.extent.min.x / MACRO_CELL_SIZE));
    const i64 cz0 = static_cast<i64>(std::floor(ctx.extent.min.y / MACRO_CELL_SIZE));
    const i64 cx1 = static_cast<i64>(std::floor((ctx.extent.max.x - 1) / MACRO_CELL_SIZE));
    const i64 cz1 = static_cast<i64>(std::floor((ctx.extent.max.y - 1) / MACRO_CELL_SIZE));

    u64 cellCount = 0, terraTriangles = 0;

    for (i64 cz = cz0; cz <= cz1; ++cz) {
        for (i64 cx = cx0; cx <= cx1; ++cx) {
            const Vec2d origin{static_cast<f64>(cx) * MACRO_CELL_SIZE,
                               static_cast<f64>(cz) * MACRO_CELL_SIZE};
            CellData& cd = ctx.cellAt(cx, cz);

            f32 minH = 1e30f, maxH = -1e30f;

            for (int lod = 0; lod < 3; ++lod) {
                const u32 n = lodSamples[lod];
                const f64 step = MACRO_CELL_SIZE / n;
                mesh::MeshBuilder& mb = cd.terrain[lod];

                // Vertices: (n+1)^2 grid.
                std::vector<u32> grid(static_cast<size_t>(n + 1) * (n + 1));
                for (u32 j = 0; j <= n; ++j) {
                    for (u32 i = 0; i <= n; ++i) {
                        const f64 wx = origin.x + i * step;
                        const f64 wz = origin.y + j * step;
                        const f32 h = ctx.terrainHF.sample(wx, wz);
                        const Vec3 nrm = ctx.terrainHF.sampleNormal(wx, wz);

                        // Surface material from proximity rules
                        SurfaceMaterial surf = SurfaceMaterial::Earth;
                        const bool onLand = ctx.cityMask.contains(Vec2d{wx, wz});
                        if (!onLand) {
                            surf = SurfaceMaterial::WetlandMud;
                        } else {
                            const f64 shoreDist = ctx.cityMask.distanceToBoundary(Vec2d{wx, wz}, 120.0);
                            if (shoreDist < 60.0) surf = SurfaceMaterial::Sand;
                            else if (h > 38.0f) surf = SurfaceMaterial::Rock;
                            else if (ctx.slumMask.contains(Vec2d{wx, wz})) surf = SurfaceMaterial::Dirt;
                            else if (h < 2.5f) surf = SurfaceMaterial::ReclamationFill;
                            else surf = SurfaceMaterial::Earth;
                        }

                        u32 mat = matEarth;
                        switch (surf) {
                            case SurfaceMaterial::Rock: mat = matRock; break;
                            case SurfaceMaterial::Sand: mat = matSand; break;
                            case SurfaceMaterial::ReclamationFill: mat = matReclaim; break;
                            case SurfaceMaterial::WetlandMud: mat = onLand ? matMud : matSeaFloor; break;
                            case SurfaceMaterial::Dirt: mat = matEarth; break;
                            default: mat = matEarth; break;
                        }
                        // Baked AO: slopes and shorelines darken slightly, as do
                        // the terrain cavities the drainage analysis found.
                        f64 ao = 1.0;
                        const f64 slope = std::acos(clamp<f64>(nrm.y, -1.0, 1.0)) * RAD2DEG;
                        ao -= clamp(slope / 180.0, 0.0, 0.35);
                        for (size_t k = 0; k < std::min<size_t>(ctx.lowPoints.size(), 64); ++k) {
                            const auto& lp = ctx.lowPoints[k];
                            const f64 d = (Vec2d{wx, wz} - lp.position).length();
                            if (d < 60.0) ao -= (1.0 - d / 60.0) * 0.25;
                        }
                        const u8 aoByte = static_cast<u8>(clamp<f64>(ao, 0.0, 1.0) * 255.0);

                        grid[static_cast<size_t>(j) * (n + 1) + i] = mb.addVertex(
                            Vec3{static_cast<f32>(i * step), h, static_cast<f32>(j * step)},
                            nrm,
                            Vec2{static_cast<f32>(wx * 0.05), static_cast<f32>(wz * 0.05)},
                            mat, static_cast<u32>(ZoningZone::Unknown), 1, aoByte);
                        if (lod == 0) { minH = std::min(minH, h); maxH = std::max(maxH, h); }
                    }
                }

                // Indices: consistent +Y winding, two triangles per quad.
                mb.beginSubmesh(matEarth, 0);
                for (u32 j = 0; j < n; ++j) {
                    for (u32 i = 0; i < n; ++i) {
                        const u32 v00 = grid[static_cast<size_t>(j) * (n + 1) + i];
                        const u32 v01 = grid[static_cast<size_t>(j + 1) * (n + 1) + i];
                        const u32 v11 = grid[static_cast<size_t>(j + 1) * (n + 1) + i + 1];
                        const u32 v10 = grid[static_cast<size_t>(j) * (n + 1) + i + 1];
                        // (x0z0, x0z1, x1z1, x1z0) order => +Y
                        mb.addTriangle(v00, v01, v11);
                        mb.addTriangle(v00, v11, v10);
                    }
                }
                mb.endSubmesh();
                if (lod == 0) terraTriangles += static_cast<u64>(n) * n * 2;
            }

            cd.minHeight = minH;
            cd.maxHeight = maxH;

            // --- surface material region mask (16 m per texel) ----------------
            cd.regionWidth = cd.regionHeight = 64;
            cd.regions.assign(64 * 64, static_cast<u8>(SurfaceMaterial::Earth));
            const f64 texel = MACRO_CELL_SIZE / 64.0;
            for (u32 j = 0; j < 64; ++j) {
                for (u32 i = 0; i < 64; ++i) {
                    const Vec2d w{origin.x + (i + 0.5) * texel, origin.y + (j + 0.5) * texel};
                    SurfaceMaterial surf = SurfaceMaterial::Earth;
                    if (!ctx.cityMask.contains(w)) {
                        surf = SurfaceMaterial::WetlandMud;
                    } else if (ctx.cityMask.distanceToBoundary(w, 60.0) < 60.0) {
                        surf = SurfaceMaterial::Sand;
                    } else if (ctx.terrainHF.sample(w.x, w.y) > 38.0f) {
                        surf = SurfaceMaterial::Rock;
                    } else if (ctx.slumMask.contains(w)) {
                        surf = SurfaceMaterial::Dirt;
                    } else if (ctx.terrainHF.sample(w.x, w.y) < 2.5f) {
                        surf = SurfaceMaterial::ReclamationFill;
                    }
                    cd.regions[static_cast<size_t>(j) * 64 + i] = static_cast<u8>(surf);
                }
            }
            ++cellCount;
        }
    }

    ctx.stats.terrainTriangles = terraTriangles;
    MLOG_INFO("terrain meshes: %llu cells, %llu LOD0 triangles (%llu verts)",
              (unsigned long long)cellCount, (unsigned long long)terraTriangles,
              (unsigned long long)(cellCount * 33ull * 33ull));
    ctx.qa.add("terrain.meshCells", static_cast<f64>(cellCount));
    ctx.qa.add("terrain.lod0Triangles", static_cast<f64>(terraTriangles));
    return true;
}

} // namespace mumbai::pipeline
