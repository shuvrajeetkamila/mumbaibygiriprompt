// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/dem.h
//
//  Digital elevation model: storage, sampling and the derived products the
//  engine needs (normals, quadtree/LOD decision, drainage analysis for the
//  monsoon flood model, terrain collision).
//
//  SOURCE NOTE (see DOCS/01_DATA_SOURCES.md, decision D-006): the
//  specification's primary DEM is Cartosat-1 (ISRO) with SRTM gap-fill and
//  MCGM LiDAR for low-lying areas.  None of those three are reachable from
//  this build environment (ISRO/Bhuvan require registration; the sandbox's
//  network is restricted to npm, PyPI and GitHub).  The pipeline therefore
//  synthesises the DEM from *published elevation control points* for Mumbai's
//  named topographic features (Malabar Hill 55 m, Gilbert Hill 60 m, Pali Hill
//  45 m, Powai basin ~50 m, Sion 12 m, BKC 5 m, reclamation 3 m, Hindmata
//  2 m...) interpolated with inverse-distance weighting, shaped by the
//  coastline, and detailed with deterministic fractal noise.  The DEM stage
//  reads a GeoTIFF/ASCII grid instead when one is supplied via
//  `--dem <file>`, so swapping in real survey data is a config change, not a
//  code change.  Absolute accuracy claims therefore hold only for the
//  control-point values; relative structure (hills, basins, drainage) is
//  faithful, which is what sightlines, road gradients and flood emergence
//  depend on.
// =============================================================================
#pragma once
#include "pipeline/mgeo/geometry.h"
#include <functional>

namespace mumbai::terrain {

// -----------------------------------------------------------------------------
//  Heightfield: regular grid of f32 metres above mean sea level.
// -----------------------------------------------------------------------------
class Heightfield {
public:
    u32 width = 0, height = 0;
    f64 originX = 0, originZ = 0;    // world metres of sample (0,0)
    f64 stepX = 1, stepZ = 1;
    std::vector<f32> samples;

    void resize(u32 w, u32 h, f64 ox, f64 oz, f64 sx, f64 sz) {
        width = w; height = h; originX = ox; originZ = oz; stepX = sx; stepZ = sz;
        samples.assign(static_cast<size_t>(w) * h, 0.0f);
    }
    f32& at(u32 i, u32 j) { return samples[static_cast<size_t>(j) * width + i]; }
    f32 at(u32 i, u32 j) const { return samples[static_cast<size_t>(j) * width + i]; }
    void setClamped(i64 i, i64 j, f32 v) {
        i = clamp<i64>(i, 0, static_cast<i64>(width) - 1);
        j = clamp<i64>(j, 0, static_cast<i64>(height) - 1);
        at(static_cast<u32>(i), static_cast<u32>(j)) = v;
    }
    bool inside(i64 i, i64 j) const {
        return i >= 0 && j >= 0 && i < static_cast<i64>(width) && j < static_cast<i64>(height);
    }

    // Bilinear sample in world metres; clamps at the edges.
    f32 sample(f64 x, f64 z) const;
    Vec3 sampleNormal(f64 x, f64 z) const;
    f32 minHeight() const;
    f32 maxHeight() const;

    Vec2d worldOf(u32 i, u32 j) const { return {originX + i * stepX, originZ + j * stepZ}; }
    Vec2d uvOf(const Vec2d& world) const {
        return {(world.x - originX) / stepX, (world.y - originZ) / stepZ};
    }

    // Slope in degrees (0 = flat).
    f64 slopeDegrees(f64 x, f64 z) const;

    // Low-point / drainage analysis: cells whose height is below `threshold`
    // and which sit in a local basin (all 8 neighbours higher).  These are the
    // waterlogging sites the monsoon model must flood (Hindmata, King's Circle,
    // Andheri subway, Milan subway...).
    struct LowPoint {
        Vec2d position;
        f32 height;
        u16 basinRank;      // 1 = lowest, larger = higher
        f64 basinArea;      // square metres of the connected catchment
    };
    std::vector<LowPoint> findLowPoints(f32 seaLevel, size_t maxCount) const;

    // Flood extent for a given water volume: rises from low points, filling
    // connected cells up to `waterLevel`.  Returns submersion depth per cell.
    std::vector<f32> floodDepths(f32 waterLevel) const;
};

// -----------------------------------------------------------------------------
//  Deterministic fractal detail (value noise, 4 octaves), seeded so that
//  rebuilds are bit-identical across machines.
// -----------------------------------------------------------------------------
f64 fractalNoise(f64 x, f64 z, u64 seed, int octaves, f64 lacunarity = 2.0, f64 gain = 0.5);

// -----------------------------------------------------------------------------
//  Control point used by the synthesiser.  `radius` is the influence scale;
//  `hard` control points pin the height exactly at their location.
// -----------------------------------------------------------------------------
struct ControlPoint {
    Vec2d position;
    f64 height = 0.0;
    f64 radius = 900.0;
    f64 weight = 1.0;
    bool hard = false;
    std::string name;
};

// Inverse-distance-weighted synthesis over a heightfield.  `landMask` (may be
// empty) forces sea cells to `seaLevel` and applies a shore falloff so that
// beaches and seawalls sit correctly relative to the water plane.
using LandTestFn = std::function<bool(const Vec2d&)>;
using ShoreDistanceFn = std::function<f64(const Vec2d&, f64)>;

void synthesizeHeightfield(Heightfield& hf,
                           const std::vector<ControlPoint>& controlPoints,
                           const LandTestFn& onLand,
                           const ShoreDistanceFn& shoreDistance,
                           f64 seaLevel,
                           f64 detailAmplitude,
                           u64 seed);

} // namespace mumbai::terrain
