// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/dem.cpp
// =============================================================================
#include "pipeline/mgeo/dem.h"

#include <queue>

namespace mumbai::terrain {

// -----------------------------------------------------------------------------
//  Value noise.  Hash-based (no permutation table), deterministic across
//  platforms because it uses only integer arithmetic and f64 lerps.
// -----------------------------------------------------------------------------
namespace {
inline u64 hash2(i64 x, i64 z, u64 seed) {
    u64 h = seed ^ (static_cast<u64>(x) * 0x9E3779B97F4A7C15ull) ^
                     (static_cast<u64>(z) * 0xC2B2AE3D27D4EB4Full);
    h ^= h >> 30; h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27; h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return h;
}
inline f64 grad01(i64 x, i64 z, u64 seed) {
    return static_cast<f64>(hash2(x, z, seed) >> 11) * (1.0 / 9007199254740992.0);
}
inline f64 fade(f64 t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }
} // namespace

f64 fractalNoise(f64 x, f64 z, u64 seed, int octaves, f64 lacunarity, f64 gain) {
    f64 sum = 0.0, amplitude = 1.0, frequency = 1.0, norm = 0.0;
    for (int o = 0; o < octaves; ++o) {
        const f64 fx = x * frequency, fz = z * frequency;
        const i64 ix = static_cast<i64>(std::floor(fx));
        const i64 iz = static_cast<i64>(std::floor(fz));
        const f64 tx = fade(fx - ix), tz = fade(fz - iz);
        const f64 v00 = grad01(ix,     iz,     seed + o * 131);
        const f64 v10 = grad01(ix + 1, iz,     seed + o * 131);
        const f64 v01 = grad01(ix,     iz + 1, seed + o * 131);
        const f64 v11 = grad01(ix + 1, iz + 1, seed + o * 131);
        const f64 a = lerp(v00, v10, tx);
        const f64 b = lerp(v01, v11, tx);
        sum += lerp(a, b, tz) * amplitude;
        norm += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }
    return norm > 0 ? (sum / norm) * 2.0 - 1.0 : 0.0;   // [-1, 1]
}

// -----------------------------------------------------------------------------
//  Heightfield sampling
// -----------------------------------------------------------------------------
f32 Heightfield::sample(f64 x, f64 z) const {
    if (width == 0 || height == 0) return 0.0f;
    const f64 fx = (x - originX) / stepX;
    const f64 fz = (z - originZ) / stepZ;
    const i64 i0 = static_cast<i64>(std::floor(fx));
    const i64 j0 = static_cast<i64>(std::floor(fz));
    const f64 tx = fx - i0, tz = fz - j0;
    const i64 i1 = clamp<i64>(i0 + 1, 0, width - 1);
    const i64 j1 = clamp<i64>(j0 + 1, 0, height - 1);
    const i64 ic = clamp<i64>(i0, 0, width - 1);
    const i64 jc = clamp<i64>(j0, 0, height - 1);
    const f64 h00 = at(static_cast<u32>(ic), static_cast<u32>(jc));
    const f64 h10 = at(static_cast<u32>(i1), static_cast<u32>(jc));
    const f64 h01 = at(static_cast<u32>(ic), static_cast<u32>(j1));
    const f64 h11 = at(static_cast<u32>(i1), static_cast<u32>(j1));
    const f64 a = lerp(h00, h10, tx);
    const f64 b = lerp(h01, h11, tx);
    return static_cast<f32>(lerp(a, b, tz));
}

Vec3 Heightfield::sampleNormal(f64 x, f64 z) const {
    const f64 d = std::max(stepX, stepZ) * 0.5;
    const f64 hL = sample(x - d, z), hR = sample(x + d, z);
    const f64 hD = sample(x, z - d), hU = sample(x, z + d);
    // Central differences -> surface normal (Y up).
    const Vec3 n{static_cast<f32>(-(hR - hL) / (2.0 * d)), 1.0f,
                 static_cast<f32>(-(hU - hD) / (2.0 * d))};
    return n.normalized();
}

f32 Heightfield::minHeight() const {
    f32 m = std::numeric_limits<f32>::max();
    for (f32 v : samples) m = std::min(m, v);
    return samples.empty() ? 0.0f : m;
}
f32 Heightfield::maxHeight() const {
    f32 m = std::numeric_limits<f32>::lowest();
    for (f32 v : samples) m = std::max(m, v);
    return samples.empty() ? 0.0f : m;
}

f64 Heightfield::slopeDegrees(f64 x, f64 z) const {
    const Vec3 n = sampleNormal(x, z);
    return std::acos(clamp<f64>(n.y, -1.0, 1.0)) * RAD2DEG;
}

// -----------------------------------------------------------------------------
//  Low-point / drainage analysis.
//
//  Flooding in Mumbai is not scripted: it happens where the terrain cannot
//  drain.  The engine reproduces this by finding genuine basins in the DEM and
//  filling them from the rainfall model (Section 3A).  A cell qualifies as a
//  basin when it is below sea level + threshold and lower than all eight
//  neighbours; connected basins are then merged so that a single depression
//  (Hindmata junction is a large shallow bowl spanning several cells) is
//  reported once, not as dozens of independent pits.
// -----------------------------------------------------------------------------
std::vector<Heightfield::LowPoint> Heightfield::findLowPoints(f32 seaLevelThreshold,
                                                              size_t maxCount) const {
    std::vector<LowPoint> out;
    if (samples.empty()) return out;
    // NOTE: callers must pass a land mask so that the sea floor is not reported
    // as a "basin". The filtering happens in the caller (buildTerrain), which
    // sets non-land samples to -1.5 m; here we additionally require the pit to
    // sit at or above `minLandHeight` so a stray sub-sea sample cannot seed a
    // 500 km^2 pseudo-basin.

    std::vector<u8> visited(samples.size(), 0);
    auto idx = [&](u32 i, u32 j) { return static_cast<size_t>(j) * width + i; };

    for (u32 j = 1; j + 1 < height; ++j) {
        for (u32 i = 1; i + 1 < width; ++i) {
            const size_t id = idx(i, j);
            if (visited[id]) continue;
            const f32 h = samples[id];
            if (h > seaLevelThreshold) continue;
            // The sea floor is not a drainage basin; only land can flood.
            if (h < -0.75f) continue;

            // Local minimum test (8-neighbourhood).
            bool isMin = true;
            for (int dj = -1; dj <= 1 && isMin; ++dj)
                for (int di = -1; di <= 1; ++di) {
                    if (di == 0 && dj == 0) continue;
                    if (samples[idx(static_cast<u32>(static_cast<i64>(i) + di),
                                    static_cast<u32>(static_cast<i64>(j) + dj))] < h) {
                        isMin = false; break;
                    }
                }
            if (!isMin) continue;

            // BFS the connected catchment that drains into this pit: cells
            // reachable without climbing above the pit height + 1.5 m.
            std::queue<std::pair<u32, u32>> q;
            q.push({i, j});
            visited[id] = 1;
            f64 area = 0.0;
            f64 sumX = 0, sumZ = 0;
            f32 lowest = h;
            const f32 ceiling = h + 1.5f;
            while (!q.empty()) {
                const auto [ci, cj] = q.front();
                q.pop();
                const f64 wx = originX + ci * stepX;
                const f64 wz = originZ + cj * stepZ;
                sumX += wx; sumZ += wz;
                area += stepX * stepZ;
                lowest = std::min(lowest, samples[idx(ci, cj)]);
                for (int dj = -1; dj <= 1; ++dj) {
                    for (int di = -1; di <= 1; ++di) {
                        const i64 ni = static_cast<i64>(ci) + di;
                        const i64 nj = static_cast<i64>(cj) + dj;
                        if (!inside(ni, nj)) continue;
                        const size_t nid = idx(static_cast<u32>(ni), static_cast<u32>(nj));
                        if (visited[nid]) continue;
                        if (samples[nid] > ceiling) continue;
                        visited[nid] = 1;
                        q.push({static_cast<u32>(ni), static_cast<u32>(nj)});
                    }
                }
            }
            if (area < 400.0) continue;   // ignore sub-puddle depressions

            LowPoint lp;
            lp.position = {sumX / (area / (stepX * stepZ)), sumZ / (area / (stepX * stepZ))};
            lp.height = lowest;
            lp.basinArea = area;
            lp.basinRank = 0;
            out.push_back(lp);
        }
    }

    // Rank by depth below sea level (deepest first) — the deepest basins are
    // the ones that flood first and worst.
    std::sort(out.begin(), out.end(), [](const LowPoint& a, const LowPoint& b) {
        if (a.height != b.height) return a.height < b.height;
        return a.basinArea > b.basinArea;
    });
    for (size_t k = 0; k < out.size(); ++k)
        out[k].basinRank = static_cast<u16>(std::min<size_t>(k + 1, 65535));
    if (out.size() > maxCount) out.resize(maxCount);
    return out;
}

std::vector<f32> Heightfield::floodDepths(f32 waterLevel) const {
    std::vector<f32> depths(samples.size(), 0.0f);
    for (size_t k = 0; k < samples.size(); ++k) {
        const f32 d = waterLevel - samples[k];
        depths[k] = d > 0.0f ? d : 0.0f;
    }
    return depths;
}

// -----------------------------------------------------------------------------
//  Synthesis
// -----------------------------------------------------------------------------
void synthesizeHeightfield(Heightfield& hf,
                           const std::vector<ControlPoint>& controlPoints,
                           const LandTestFn& onLand,
                           const ShoreDistanceFn& shoreDistance,
                           f64 seaLevel,
                           f64 detailAmplitude,
                           u64 seed) {
    const size_t total = static_cast<size_t>(hf.width) * hf.height;
    if (total == 0) return;

    // Spatial binning of control points keeps the per-sample cost proportional
    // to the nearby set instead of all points: with a 20 m grid over 600 km^2
    // the naive O(N*M) form would be 1.5M samples x 40 points every build.
    struct Bin { std::vector<u32> ids; };
    const f64 binSize = 1024.0;
    std::unordered_map<u64, Bin> bins;
    auto binKey = [](i64 x, i64 z) { return static_cast<u64>(x) * 73856093ull ^ static_cast<u64>(z) * 19349663ull; };
    for (u32 k = 0; k < controlPoints.size(); ++k) {
        const ControlPoint& cp = controlPoints[k];
        const i64 bx = static_cast<i64>(std::floor(cp.position.x / binSize));
        const i64 bz = static_cast<i64>(std::floor(cp.position.y / binSize));
        bins[binKey(bx, bz)].ids.push_back(k);
    }

    for (u32 j = 0; j < hf.height; ++j) {
        for (u32 i = 0; i < hf.width; ++i) {
            const Vec2d world = hf.worldOf(i, j);

            // --- land / sea gate ------------------------------------------
            f64 landDistance = 1e9;
            if (onLand && !onLand(world)) {
                // Sea.  The profile is driven by distance to the shore so the
                // harbour gets a shallow shelf and the Arabian Sea a proper
                // depth, which the ocean shader turns into colour and foam.
                landDistance = shoreDistance ? shoreDistance(world, 4000.0) : 1000.0;
                hf.at(i, j) = static_cast<f32>(seaLevel - 1.5 - std::min(14.0, landDistance * 0.008));
                continue;
            }
            const f64 shoreDist = shoreDistance ? shoreDistance(world, 4000.0) : 1e9;

            // --- IDW over nearby control points ---------------------------
            const i64 bx = static_cast<i64>(std::floor(world.x / binSize));
            const i64 bz = static_cast<i64>(std::floor(world.y / binSize));
            f64 wsum = 0.0, hsum = 0.0;
            f64 hardHeight = 0.0;
            bool hasHard = false;
            for (i64 dz = -1; dz <= 1; ++dz) {
                for (i64 dx = -1; dx <= 1; ++dx) {
                    auto it = bins.find(binKey(bx + dx, bz + dz));
                    if (it == bins.end()) continue;
                    for (u32 id : it->second.ids) {
                        const ControlPoint& cp = controlPoints[id];
                        const f64 dist = (world - cp.position).length();
                        if (cp.hard && dist < cp.radius) {
                            const f64 t = clamp(1.0 - dist / cp.radius, 0.0, 1.0);
                            const f64 v = lerp(seaLevel + 1.0, cp.height, smoothstep(t));
                            if (!hasHard || v > hardHeight) { hardHeight = v; hasHard = true; }
                            continue;
                        }
                        const f64 w = cp.weight / (std::pow(dist, 2.2) + 1.0);
                        wsum += w;
                        hsum += w * cp.height;
                    }
                }
            }
            f64 base = wsum > 1e-12 ? hsum / wsum : (seaLevel + 4.0);

            if (hasHard) base = std::max(base, hardHeight);

            // --- shore falloff -------------------------------------------
            // Within 400 m of the shoreline, pull the terrain toward a
            // reclamation/beach profile (2-3 m above the sea) so the waterline
            // is plausible instead of a cliff.
            if (shoreDist < 400.0) {
                const f64 t = smoothstep(shoreDist / 400.0);
                base = lerp(seaLevel + 2.0, base, t);
            }

            // --- fractal detail ------------------------------------------
            // The detail amplitude fades toward the shore.  Without this the
            // reclaimed flats (Nariman Point, Cuffe Parade, BKC) and the
            // low-lying tracts (Hindmata, King's Circle, Andheri subway) end up
            // as one giant plane clamped at the minimum land height, which both
            // looks wrong and robs the flood model of the small-scale relief it
            // needs.  Hills keep full detail; the coastal strip keeps 15%.
            const f64 detailScale = clamp(shoreDist / 900.0, 0.15, 1.0);
            const f64 detail = fractalNoise(world.x * 0.0022, world.y * 0.0022, seed, 4) *
                               detailAmplitude * detailScale;
            f64 h = base + detail;

            // Land may sit slightly below the water plane: that is precisely how
            // Mumbai floods. The drainage analysis later finds these places, and
            // the monsoon model fills them. Only a hard floor is enforced so
            // that no land sits more than 0.75 m under the sea in a dry world.
            if (h < seaLevel - 0.75) h = seaLevel - 0.75;

            hf.at(i, j) = static_cast<f32>(h);
        }
        if ((j & 255) == 0) MLOG_TRACE("terrain row %u/%u", j, hf.height);
    }
}

} // namespace mumbai::terrain
