// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/hydro.cpp
// =============================================================================
#include "pipeline/mgeo/hydro.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace mumbai::hydro {
namespace {

// D8 neighbour offsets, indexed by direction code: E NE N NW W SW S SE.
constexpr int kDx[8] = { 1,  1,  0, -1, -1, -1,  0,  1};
constexpr int kDy[8] = { 0,  1,  1,  1,  0, -1, -1, -1};

struct QueueEntry {
    f32 h;
    u32 idx;
    bool operator>(const QueueEntry& o) const {
        return h != o.h ? h > o.h : idx > o.idx;   // deterministic on ties
    }
};

inline f64 cellDistance(int dir, f64 step) {
    return (dir & 1) ? step * 1.4142135623730951 : step;   // odd codes are diagonal
}

} // namespace

// -----------------------------------------------------------------------------
//  Surface construction
// -----------------------------------------------------------------------------
Surface buildSurface(const terrain::Heightfield& hf, const SurfaceOptions& opts) {
    Surface s;
    s.width = hf.width;
    s.height = hf.height;
    s.originX = hf.originX;
    s.originZ = hf.originZ;
    s.step = hf.stepX;
    const size_t n = s.cellCount();
    const auto idx = [&](i64 i, i64 j) {
        return static_cast<u32>(j * static_cast<i64>(s.width) + i);
    };

    s.ground.assign(hf.samples.begin(), hf.samples.end());
    s.filled.assign(n, 0.0f);
    s.isBoundary.assign(n, 0);
    s.flowDir.assign(n, 8);
    s.accum.assign(n, 0);
    s.watershed.assign(n, 0);
    s.isSea.assign(n, 0);
    s.isShore.assign(n, 0);
    s.isChannel.assign(n, 0);
    s.order.assign(n, 0xFFFFFFFFu);

    // --- outlets: the sea, the creeks, and (optionally) the domain edge ------
    if (opts.openBoundaries) {
        for (u32 j = 0; j < s.height; ++j)
            for (u32 i = 0; i < s.width; ++i)
                if (i == 0 || j == 0 || i + 1 == s.width || j + 1 == s.height)
                    s.isBoundary[j * s.width + i] = 1;
    }
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;
    std::vector<u8> settled(n, 0);
    u32 counter = 0;
    for (u32 j = 0; j < s.height; ++j) {
        for (u32 i = 0; i < s.width; ++i) {
            const u32 k = j * s.width + i;
            const f32 h = hf.samples[k];
            const bool edge = opts.openBoundaries &&
                              (i == 0 || j == 0 || i + 1 == s.width || j + 1 == s.height);
            // Outlets seed the flood with their own elevation.  Because the
            // queue is ordered by height, a cell is always reached from its
            // lowest escape first, so seeding the edge cannot flatten terrain
            // that has a lower way out.
            if (h <= static_cast<f32>(opts.seaLevel - opts.seaPickThreshold) || edge) {
                s.isSea[k] = static_cast<u8>(h <= opts.seaLevel - opts.seaPickThreshold ? 1 : 0);
                s.filled[k] = h;
                s.order[k] = counter++;
                settled[k] = 1;
                pq.push({h, k});
            }
        }
    }
    if (pq.empty()) {
        // No sea in this heightfield (inland test terrain): seed the lowest cell
        // so the surface still has an outlet.
        u32 lowest = 0;
        for (u32 k = 1; k < n; ++k) if (hf.samples[k] < hf.samples[lowest]) lowest = k;
        s.isSea[lowest] = 1;
        s.filled[lowest] = hf.samples[lowest];
        s.order[lowest] = counter++;
        settled[lowest] = 1;
        pq.push({hf.samples[lowest], lowest});
    }

    // --- priority flood ------------------------------------------------------
    while (!pq.empty()) {
        const QueueEntry e = pq.top();
        pq.pop();
        const u32 k = e.idx;
        const i64 i = static_cast<i64>(k % s.width);
        const i64 j = static_cast<i64>(k / s.width);
        for (int d = 0; d < 8; ++d) {
            const i64 ni = i + kDx[d], nj = j + kDy[d];
            if (ni < 0 || nj < 0 || ni >= static_cast<i64>(s.width) || nj >= static_cast<i64>(s.height))
                continue;
            const u32 nk = idx(ni, nj);
            if (settled[nk]) continue;
            const f32 filled = std::max(hf.samples[nk], e.h);
            s.filled[nk] = filled;
            s.order[nk] = counter++;
            settled[nk] = 1;
            pq.push({filled, nk});
        }
    }
    // --- shoreline ------------------------------------------------------------
    for (u32 j = 0; j < s.height; ++j) {
        for (u32 i = 0; i < s.width; ++i) {
            const u32 k = j * s.width + i;
            if (s.isSea[k]) continue;
            for (int d = 0; d < 8; ++d) {
                const i64 ni = static_cast<i64>(i) + kDx[d], nj = static_cast<i64>(j) + kDy[d];
                if (ni < 0 || nj < 0 || ni >= static_cast<i64>(s.width) || nj >= static_cast<i64>(s.height)) continue;
                if (s.isSea[static_cast<u32>(nj * static_cast<i64>(s.width) + ni)]) { s.isShore[k] = 1; break; }
            }
        }
    }

    // --- tidal inlets ---------------------------------------------------------
    // Which shores the tide can reach.  A creek or backwater is narrow; the open
    // sea is not.  For every sea cell we measure how much sea lies within a
    // Chebyshev radius of four cells (9 x 9 window, computed separably), and a
    // shore cell fronting water that is not mostly sea is an inlet: the tide
    // comes in there through the creek mouth, exactly as it does at Mahim, in
    // Thane Creek and up the Mithi.  Everywhere else the sea wall holds.
    {
        const i64 w = static_cast<i64>(s.width), h = static_cast<i64>(s.height);
        const int R = 4;
        const int W = 2 * R + 1;
        std::vector<f32> rowSum(n, 0.0f), window(n, 0.0f);
        // Centred box mean, horizontally then vertically.  Both passes use two
        // pointers over a clamped window.  The sloppy version of this — slide a
        // fixed-width window and subtract the element W back — computes a window
        // flush with the edge of the grid instead of one centred on the cell, and
        // that biases every coastal cell's sea fraction landward.  It is not a
        // cosmetic difference: it made the whole sea wall read as a creek mouth,
        // and the model then drowned the coast on every tide.
        const auto boxPass = [&](const std::vector<f32>& src, std::vector<f32>& dst,
                                 bool horizontal) {
            const i64 outer = horizontal ? h : w;      // lines to walk
            const i64 inner = horizontal ? w : h;      // positions along a line
            for (i64 line = 0; line < outer; ++line) {
                i64 lo = 0, hi = -1;
                f64 run = 0.0;
                for (i64 t = 0; t < inner; ++t) {
                    const i64 nlo = std::max<i64>(0, t - R);
                    const i64 nhi = std::min<i64>(inner - 1, t + R);
                    const auto idx = [&](i64 u) {
                        return horizontal ? (line * w + u) : (u * w + line);
                    };
                    while (hi < nhi) { ++hi; run += src[idx(hi)]; }
                    while (lo < nlo) { run -= src[idx(lo)]; ++lo; }
                    dst[idx(t)] = static_cast<f32>(run / static_cast<f64>(nhi - nlo + 1));
                }
            }
        };
        std::vector<f32> seaF(n, 0.0f);
        for (size_t k = 0; k < n; ++k) seaF[k] = s.isSea[k] ? 1.0f : 0.0f;
        std::vector<f32> tmp(n, 0.0f);
        boxPass(seaF, tmp, true);        // horizontal means into tmp
        boxPass(tmp, window, false);     // vertical means of those into window
        s.isTidalInlet.assign(n, 0);
        for (u32 j = 0; j < s.height; ++j) {
            for (u32 i = 0; i < s.width; ++i) {
                const u32 k = j * s.width + i;
                if (s.isSea[k] || !s.isShore[k]) continue;
                f32 narrowest = 1.0f;
                for (int d = 0; d < 8; ++d) {
                    const i64 ni = static_cast<i64>(i) + kDx[d], nj = static_cast<i64>(j) + kDy[d];
                    if (ni < 0 || nj < 0 || ni >= w || nj >= h) continue;
                    const u32 nk = static_cast<u32>(nj * w + ni);
                    if (!s.isSea[nk]) continue;
                    narrowest = std::min(narrowest, window[nk]);
                }
                if (narrowest < 0.55f) s.isTidalInlet[k] = 1;
            }
        }
    }

    // --- D8 flow directions --------------------------------------------------
    // Downstream is either a strictly lower neighbour (steepest descent) or, on
    // a priority-flood flat, the equal-height neighbour that was settled first —
    // which is guaranteed to exist and guarantees an acyclic flow graph.
    for (u32 j = 0; j < s.height; ++j) {
        for (u32 i = 0; i < s.width; ++i) {
            const u32 k = j * s.width + i;
            if (s.isSea[k] || s.isBoundary[k]) { s.flowDir[k] = 8; continue; }
            f64 bestSlope = 0.0;
            int bestDir = -1;
            u32 bestOrder = 0xFFFFFFFFu;
            int flatDir = -1;
            for (int d = 0; d < 8; ++d) {
                const i64 ni = static_cast<i64>(i) + kDx[d], nj = static_cast<i64>(j) + kDy[d];
                if (ni < 0 || nj < 0 || ni >= static_cast<i64>(s.width) || nj >= static_cast<i64>(s.height))
                    continue;
                const u32 nk = static_cast<u32>(nj * static_cast<i64>(s.width) + ni);
                const f64 drop = static_cast<f64>(s.filled[k]) - s.filled[nk];
                if (drop > 1e-6) {
                    const f64 slope = drop / cellDistance(d, s.step);
                    if (slope > bestSlope) { bestSlope = slope; bestDir = d; }
                } else if (drop > -1e-6 && s.order[nk] < s.order[k] && s.order[nk] < bestOrder) {
                    bestOrder = s.order[nk];
                    flatDir = d;
                }
            }
            s.flowDir[k] = static_cast<u8>(bestDir >= 0 ? bestDir : (flatDir >= 0 ? flatDir : 8));
        }
    }

    // --- flow accumulation ---------------------------------------------------
    // The priority-flood order is a topological order of the flow graph: every
    // cell's downstream neighbour was settled strictly earlier, so iterating the
    // settle order backwards accumulates every upstream contribution before it
    // is passed on.  No sorting, no recursion.
    std::vector<u32> byOrder(n, 0);
    for (u32 k = 0; k < n; ++k) byOrder[s.order[k]] = k;
    for (u32 k = 0; k < n; ++k) s.accum[k] = (s.isSea[k] || s.isBoundary[k]) ? 0u : 1u;
    for (size_t pos = static_cast<size_t>(counter); pos-- > 1;) {
        const u32 k = byOrder[pos];
        const u8 d = s.flowDir[k];
        if (d > 7) continue;
        const i64 i = static_cast<i64>(k % s.width) + kDx[d];
        const i64 j = static_cast<i64>(k / s.width) + kDy[d];
        if (i < 0 || j < 0 || i >= static_cast<i64>(s.width) || j >= static_cast<i64>(s.height)) continue;
        const u32 nk = static_cast<u32>(j * static_cast<i64>(s.width) + i);
        s.accum[nk] += s.accum[k];
    }

    // --- channels ------------------------------------------------------------
    const f64 channelCells = std::max(4.0, opts.channelAreaKm2 * 1e6 / s.cellArea());
    for (u32 k = 0; k < n; ++k) {
        if (!s.isSea[k] && !s.isBoundary[k] &&
            static_cast<f64>(s.accum[k]) >= channelCells) s.isChannel[k] = 1;
    }

    // --- watersheds ----------------------------------------------------------
    // 0 = sea / direct drainage with no channel.  Channel cells that share a
    // downstream path get one id; every other land cell inherits the id of the
    // cell it drains into, walking downstream-first (ascending settle order).
    u32 nextId = 1;
    for (size_t pos = 0; pos < static_cast<size_t>(counter); ++pos) {
        const u32 k = byOrder[pos];
        if (!s.isChannel[k]) continue;
        const u8 d = s.flowDir[k];
        if (d > 7) { s.watershed[k] = 0; continue; }
        const i64 i = static_cast<i64>(k % s.width) + kDx[d];
        const i64 j = static_cast<i64>(k / s.width) + kDy[d];
        if (i < 0 || j < 0 || i >= static_cast<i64>(s.width) || j >= static_cast<i64>(s.height)) continue;
        const u32 nk = static_cast<u32>(j * static_cast<i64>(s.width) + i);
        if (s.isSea[nk] || s.isBoundary[nk]) { s.watershed[k] = nextId++; }   // terminal reach
        else if (s.isChannel[nk]) s.watershed[k] = s.watershed[nk];   // downstream was processed first
        else s.watershed[k] = 0;
    }
    for (size_t pos = 0; pos < static_cast<size_t>(counter); ++pos) {
        const u32 k = byOrder[pos];
        if (s.isSea[k] || s.isBoundary[k] || s.isChannel[k]) continue;
        const u8 d = s.flowDir[k];
        if (d > 7) continue;
        const i64 i = static_cast<i64>(k % s.width) + kDx[d];
        const i64 j = static_cast<i64>(k / s.width) + kDy[d];
        if (i < 0 || j < 0 || i >= static_cast<i64>(s.width) || j >= static_cast<i64>(s.height)) continue;
        const u32 nk = static_cast<u32>(j * static_cast<i64>(s.width) + i);
        if (!s.isSea[nk] && !s.isBoundary[nk]) s.watershed[k] = s.watershed[nk];
    }
    return s;
}

// -----------------------------------------------------------------------------
//  Storm definitions
// -----------------------------------------------------------------------------
StormSpec stormSpecModerate() {
    StormSpec s;
    s.name = "moderate";
    // A routine heavy monsoon day: 80 mm in six hours with a 32 mm/h peak. This
    // is the event class that closes the Andheri subway several times a season.
    s.rainMmPerHour = {6, 12, 22, 32, 30, 20, 10, 4, 2, 1};
    s.tideAmplitude = 2.1;
    s.tidePhaseHours = 4.5;      // rain arrives on the rising tide
    s.stepSeconds = 300.0;
    return s;
}

StormSpec stormSpecSevere() {
    StormSpec s;
    s.name = "severe";
    // 26 July 2005: 944 mm in 24 h, with a burst above 150 mm/h. The hyetograph
    // below peaks at 190 mm/h and totals ~700 mm over the modelled 12 hours,
    // which is the shape of that event rather than its full depth.
    s.rainMmPerHour = {20, 45, 95, 190, 175, 120, 70, 45, 30, 22, 16, 12};
    s.tideAmplitude = 2.3;       // spring tide, as on the day
    s.tidePhaseHours = 2.0;      // high water just as the burst arrives
    s.stepSeconds = 300.0;
    return s;
}

// -----------------------------------------------------------------------------
//  Channel capacity
// -----------------------------------------------------------------------------
f64 channelCapacityFor(u32 contributingCells, f64 cellArea, f64 bedSlope,
                       const DrainSpec& spec) {
    // Trapezoidal channel: bottom width 2 m, side slopes 1:1.5, and a top width
    // that grows with the square root of the catchment (a bigger catchment has a
    // bigger channel — the standard hydraulic-geometry relation).
    const f64 area = static_cast<f64>(contributingCells) * cellArea;
    const f64 topWidth = clamp(3.0 + 0.05 * std::sqrt(area), 3.0, 60.0);
    const f64 depth = std::min(spec.naturalChannelDepth, topWidth / 6.0);
    const f64 bottom = std::max(1.0, topWidth - 3.0 * depth);
    const f64 wetted = bottom + depth * std::sqrt(1.0 + 1.5 * 1.5) * 2.0;
    const f64 section = 0.5 * (bottom + topWidth) * depth;
    const f64 hydraulicRadius = section / std::max(0.1, wetted);
    const f64 slope = std::max(1e-4, bedSlope);
    return section * (1.0 / spec.naturalChannelManningN) *
           std::pow(hydraulicRadius, 2.0 / 3.0) * std::sqrt(slope);
}

// -----------------------------------------------------------------------------
//  Land properties
// -----------------------------------------------------------------------------
LandSpec rasteriseLand(const Surface& s, const CoverageInput& in, const DrainSpec& spec) {
    LandSpec land;
    const size_t n = s.cellCount();
    land.resize(n);
    for (u32 j = 0; j < s.height; ++j) {
        for (u32 i = 0; i < s.width; ++i) {
            const size_t k = static_cast<size_t>(j) * s.width + i;
            const Vec2d w{s.originX + i * s.step, s.originZ + j * s.step};
            const bool road = in.isRoad && in.isRoad(w);
            const bool building = in.isBuilding && in.isBuilding(w);
            const bool green = in.isGreen && in.isGreen(w);

            // Runoff coefficient: impervious surfaces shed almost everything,
            // gardens and hillside absorb half.
            // Unbuilt ground absorbs: a cell carrying no road and no building
            // is open land — hillside, mangrove, mudflat, the park — and most of
            // the rain that falls on it never becomes runoff.  Assuming a third
            // of it runs off is what put a sheet of water on the hills.
            f64 impervious = 0.20;
            if (!road && !building) impervious = 0.14;
            if (green) impervious = 0.08;
            if (building) impervious = std::max(impervious, 0.78);
            if (road) impervious = std::max(impervious, 0.88);
            land.impervious[k] = static_cast<f32>(clamp(impervious, 0.05, 0.95));

            // Roughness follows the surface, and the surface is what decides
            // whether water can travel.
            f64 roughness = spec.perviousRoughness;
            if (road) roughness = spec.pavedRoughness;
            else if (building) roughness = 0.020;      // roofs shed to the street
            land.roughness[k] = static_cast<f32>(roughness);

            // Man-made drainage follows the street network.
            f64 mmPerHour = 0.0;
            if (road && in.roadClass) {
                switch (in.roadClass(w)) {
                    case 0: mmPerHour = spec.motorwayMmPerHour; break;    // motorway
                    case 1: mmPerHour = spec.motorwayMmPerHour * 0.8; break;  // trunk
                    case 2: mmPerHour = spec.arterialMmPerHour; break;    // primary
                    case 3: mmPerHour = spec.arterialMmPerHour * 0.8; break;  // secondary
                    case 4: mmPerHour = spec.arterialMmPerHour * 0.6; break;  // tertiary
                    case 5: mmPerHour = spec.residentialMmPerHour; break;
                    default: mmPerHour = spec.serviceMmPerHour; break;
                }
            }
            land.drainCapacity[k] = static_cast<f32>(mmPerHour / 1000.0 / 3600.0);   // m/s
            if (road) {
                land.drainNetwork[k] = 1;
                // ...and the street *conveys* at the same design rate it drains
                // at.  Without this the imposed gutter gradient drove an unbounded
                // flux — eight times the city's rainfall came round again through
                // the network — and shallow water that the city is famous for
                // holding drained away as if the streets were chutes.
                land.designFlow[k] = static_cast<f32>(mmPerHour / 1000.0 / 3600.0 * s.cellArea());
            }

            if (s.isChannel[k]) {
                land.drainNetwork[k] = 1;
                land.roughness[k] = static_cast<f32>(spec.channelRoughness);
                const f64 bedSlope = (s.flowDir[k] <= 7 && s.accum[k] > 1)
                    ? std::max(1e-5, static_cast<f64>(s.ground[k]) - s.ground[k + 1]) / s.step
                    : 1e-4;
                land.channelCapacity[k] = static_cast<f32>(
                    channelCapacityFor(s.accum[k], s.cellArea(), bedSlope, spec));
                land.designFlow[k] = land.channelCapacity[k];
                // A channel also intercepts direct rainfall at its own rate.
                land.drainCapacity[k] = std::max(land.drainCapacity[k],
                                                 static_cast<f32>(spec.residentialMmPerHour / 1000.0 / 3600.0));
            }
        }
    }

    // --- where each cell's rainfall goes -------------------------------------
    // Walk the D8 drainage path until it reaches a network cell (or leaves the
    // modelled surface).  Flow directions come from the priority-flood filled
    // surface, so this walk always descends and terminates; the step cap is a
    // belt-and-braces guard against a tie between two equal cells.
    land.networkOutlet.resize(n);
    std::vector<u8> greenCell;
    if (in.isGreen) {
        greenCell.assign(n, 0);
        for (u32 j = 0; j < s.height; ++j)
            for (u32 i = 0; i < s.width; ++i) {
                const Vec2d c{s.originX + (i + 0.5) * s.step, s.originZ + (j + 0.5) * s.step};
                if (in.isGreen(c)) greenCell[static_cast<size_t>(j) * s.width + i] = 1;
            }
    }
    for (size_t k = 0; k < n; ++k) {
        if (s.isSea[k] || s.isBoundary[k]) { land.networkOutlet[k] = static_cast<u32>(k); continue; }
        if (land.drainNetwork[k]) { land.networkOutlet[k] = static_cast<u32>(k); continue; }
        if (!greenCell.empty() && !greenCell[k]) {
            u32 cur = static_cast<u32>(k);
            for (int guard = 0; guard < spec.maxOutletWalk; ++guard) {
                if (land.drainNetwork[cur] || s.isSea[cur] || s.isBoundary[cur]) break;
                const u8 d = s.flowDir[cur];
                if (d > 7) break;
                const i64 i = static_cast<i64>(cur % s.width) + kDx[d];
                const i64 j = static_cast<i64>(cur / s.width) + kDy[d];
                if (i < 0 || j < 0 || i >= static_cast<i64>(s.width) || j >= static_cast<i64>(s.height))
                    break;
                const u32 nk = static_cast<u32>(j * static_cast<i64>(s.width) + i);
                if (nk == cur) break;
                cur = nk;
            }
            land.networkOutlet[k] = cur;
        } else {
            // Gardens and open ground keep their rain: the soil takes the
            // runoff share and the rest stays where it fell.
            land.networkOutlet[k] = static_cast<u32>(k);
            continue;
        }
    }
    return land;
}

// -----------------------------------------------------------------------------
//  Storm simulation
// -----------------------------------------------------------------------------
FloodResult simulate(const Surface& s, const LandSpec& land, const StormSpec& storm,
                     const SimOptions& opt) {
    const size_t n = s.cellCount();
    FloodResult r;
    r.maxDepth.assign(n, 0.0f);
    r.finalDepth.assign(n, 0.0f);
    r.timeOfPeakHours.assign(n, 0.0f);

    // Depth is kept in f64: a millimetre of rain spread over a 576 m^2 cell is
    // 0.6 m^3, and rounding that to f32 every step is a visible leak out of the
    // mass balance over a twelve-hour storm.
    std::vector<f64> depth(n, 0.0);
    std::vector<f64> remainingLoss(n, storm.initialLossMm / 1000.0);

    const f64 dt = storm.stepSeconds;
    const f64 area = s.cellArea();
    const size_t steps = static_cast<size_t>(std::max(1.0, storm.rainMmPerHour.size() * 3600.0 / dt));

    const auto tideAt = [&](f64 hours) {
        return storm.tideAmplitude *
               std::cos(TAU * (hours + storm.tidePhaseHours) / storm.tidePeriodHours);
    };
    const auto rainAt = [&](f64 hours) {
        const size_t h0 = static_cast<size_t>(hours);
        if (h0 >= storm.rainMmPerHour.size()) return 0.0;
        const f64 t = hours - h0;
        const f64 a = storm.rainMmPerHour[h0];
        const f64 b = (h0 + 1 < storm.rainMmPerHour.size()) ? storm.rainMmPerHour[h0 + 1] : a * 0.5;
        return a + (b - a) * t;                       // linear ramp between hours
    };

    // Sea cells hold the tide; the sea is an infinite reservoir, so every change
    // in its level is a transfer across the domain boundary.
    f64 tideLevel = tideAt(0.0);
    for (size_t k = 0; k < n; ++k) {
        if (!s.isSea[k]) continue;
        depth[k] = std::max(0.0, tideLevel - s.ground[k]);
    }
    r.peakTideLevel = tideLevel;

    // Active-cell bookkeeping: only cells that hold water or touch water are
    // swept, which keeps the drying tail of a storm cheap.
    std::vector<u32> active;
    active.reserve(n / 2);
    std::vector<u8> touched(n, 0);

    for (size_t step = 0; step < steps; ++step) {
        const f64 hours = step * dt / 3600.0;
        const f64 rainMm = rainAt(hours) * dt / 3600.0;
        tideLevel = tideAt(hours);
        r.peakTideLevel = std::max(r.peakTideLevel, tideLevel);

        // --- 1. rain on land -------------------------------------------------
        const bool haveOutlet = land.networkOutlet.size() == n;
        if (rainMm > 0.0) {
            for (u32 j = 0; j < s.height; ++j) {
                for (u32 i = 0; i < s.width; ++i) {
                    const size_t k = static_cast<size_t>(j) * s.width + i;
                    // Rain falls on the modelled land only.  Sea and open-edge
                    // cells are the outside world: giving them rain they can
                    // never shed is a silent hole in the mass balance.
                    if (s.isSea[k] || s.isBoundary[k]) continue;
                    f64 gross = rainMm / 1000.0;
                    if (remainingLoss[k] > 0.0) {
                        const f64 taken = std::min(remainingLoss[k], gross);
                        remainingLoss[k] -= taken;
                        gross -= taken;
                    }
                    if (gross <= 0.0) continue;
                    const f64 net = gross * land.impervious[k];
                    r.rainfallM3 += net * area;
                    // Runoff reaches the gutter, not the puddle: it is banked to
                    // the cell that actually drains, which is where a resident
                    // would see it arrive.
                    const u32 target = haveOutlet ? land.networkOutlet[k] : k;
                    if (s.isSea[target] || s.isBoundary[target])
                        r.outfallM3 += net * area;      // a coastal outfall: gone to the sea
                    else
                        depth[target] += net;
                }
            }
        }

        // --- 2. the sea holds its level --------------------------------------
        // The sea is a level-controlled reservoir, not a cell that accumulates:
        // its surface sits at the tide, and every change in that level is an
        // exchange with the outside world.  Because the sea never stores water
        // that arrived from the land (see the routing step), this signed term is
        // the only boundary input, and the mass balance closes exactly.
        for (u32 j = 0; j < s.height; ++j) {
            for (u32 i = 0; i < s.width; ++i) {
                const size_t k = static_cast<size_t>(j) * s.width + i;
                if (!s.isSea[k]) continue;
                const f64 want = std::max(0.0, tideLevel - s.ground[k]);
                r.tideExchangeM3 += (want - depth[k]) * area;
                depth[k] = want;
            }
        }

        const i32 subSteps = std::max<i32>(1, storm.routingSubSteps);
        const f64 dtSub = dt / static_cast<f64>(subSteps);
        for (i32 sub = 0; sub < subSteps; ++sub) {
        // --- 2b. tidal inundation of the shore -------------------------------
        // The boundary condition for the land: a shore cell cannot stand lower
        // than the sea beside it.  Whatever the tide rises above the ground of
        // such a cell is sea water that has entered the city, and it is logged
        // as an input; when the tide falls, the routing step drains it back out
        // through the same shore line.  Doing this by level rather than by flux
        // is what keeps the interface stable — a diffusive flux from an infinite
        // reservoir with a two-metre head is unbounded, and overshoots the cell
        // depth by three orders of magnitude in a single step.
        if (opt.tideLockOutfalls && sub == 0) {
            for (u32 k = 0; k < n; ++k) {
                if (!s.isShore[k] || s.isSea[k]) continue;
                // The open coast is walled: only a creek mouth admits the tide,
                // and only a tide above the wall crest comes over the top.
                if (!s.isTidalInlet[k] && tideLevel <= opt.seawallCrest) continue;
                // Land that the DEM holds at its below-datum floor is behind the
                // shore line, not on it (see SurfaceOptions::landFloor).
                if (s.ground[k] <= static_cast<f32>(opt.landFloor) + 0.01f) continue;
                const f64 want = std::max(0.0, tideLevel - s.ground[k]);
                if (depth[k] < want) {
                    r.tideFloodM3 += (want - depth[k]) * area;
                    depth[k] = want;
                    touched[k] = 1;
                }
            }
        }

        // --- 3. routing ------------------------------------------------------
        // Diffusive wave over the eight neighbours: flux is Manning's equation on
        // the cell face with the water-surface gradient as the slope.  Depressions
        // therefore fill and spill instead of being drained by their D8 escape
        // route, and a surcharged outfall backs water up into the city.
        // Outflow from a cell is capped at half of what it holds, which makes the
        // explicit scheme unconditionally stable and mass conserving.
        touched.assign(n, 0);
        active.clear();
        const bool perCellRoughness = land.roughness.size() == n;
        const bool haveNetwork = land.drainNetwork.size() == n;
        const bool haveDesign = land.designFlow.size() == n;
        for (size_t k = 0; k < n; ++k) {
            // The sea is not a routing source: it interacts with the land by
            // level (below), which is bounded and stable, instead of by a
            // diffusive flux that has no storage limit behind it.
            if (!s.isSea[k] && depth[k] > opt.minDepth) {
                active.push_back(static_cast<u32>(k));
                touched[k] = 1;
            }
        }
        for (size_t a = 0; a < active.size(); ++a) {
            const u32 k = active[a];
            const f64 dk = depth[k];
            if (dk <= opt.minDepth) continue;
            const i64 i = static_cast<i64>(k % s.width);
            const i64 j = static_cast<i64>(k / s.width);
            const bool onNetwork = haveNetwork && land.drainNetwork[k] != 0;
            // A cell the priority flood had to raise is inside a basin: its water
            // has to reach the spill level before it goes anywhere.  The imposed
            // gutter gradient must not be applied to such a cell, or the model
            // drains the very puddles it exists to predict — Hindmata is a bowl,
            // and the whole point of that bowl is that the water sits in it.
            const bool impounded = s.filled[k] > s.ground[k] + 1e-4f;
            const f64 wsK = s.ground[k] + dk;          // water surface, raw terrain
            f64 budget = dk * area * 0.5;                            // stability limiter
            if (land.channelCapacity[k] > 0.0f)                      // conveyance cap
                budget = std::min(budget, static_cast<f64>(land.channelCapacity[k]) * dtSub);

            f64 moved = 0.0;
            for (int d = 0; d < 8 && moved < budget; ++d) {
                const i64 ni = i + kDx[d], nj = j + kDy[d];
                if (ni < 0 || nj < 0 || ni >= static_cast<i64>(s.width) || nj >= static_cast<i64>(s.height))
                    continue;
                const u32 nk = static_cast<u32>(nj * static_cast<i64>(s.width) + ni);
                const f64 wsN = s.ground[nk] + depth[nk];
                const f64 head = wsK - wsN;
                if (head <= 1e-4) continue;
                // Flow depth over the sill between the two cells.
                const f64 sill = std::max(static_cast<f64>(s.ground[k]), static_cast<f64>(s.ground[nk]));
                const f64 dEff = std::max(0.0, wsK - sill);
                if (dEff <= 1e-4) continue;
                const f64 length = cellDistance(d, s.step);
                const f64 nHere = perCellRoughness ? std::max(0.010, static_cast<f64>(land.roughness[k]))
                                                   : opt.roughnessManningsN;
                // Engineered conveyance: the drainage network carries water at
                // its design gradient even when the sheet on top of it is flat
                // (see SimOptions::minDrainSlope).  Only downhill, so this can
                // never move water up a slope.
                // Flow driven by the water that is actually there...
                const f64 conveyance = (1.0 / nHere) * s.step * std::pow(dEff, 5.0 / 3.0);
                f64 q = conveyance * std::sqrt(head / length);
                // ...plus the flow the graded gutter itself carries, bounded by
                // the drain it belongs to.
                if (opt.minDrainSlope > 0.0 && onNetwork && !impounded && s.ground[nk] < s.ground[k]) {
                    const f64 design = (haveDesign ? static_cast<f64>(land.designFlow[k]) : 0.0);
                    const f64 spare = std::max(0.0, design - q);
                    const f64 qGrad = conveyance * std::sqrt(opt.minDrainSlope);
                    q += std::min(qGrad, spare);
                }
                // Velocity limiter.  Manning's equation applied across one 32 m
                // face of a metre-deep sheet returns tens of metres per second,
                // and the diffusive wave it comes from assumes the opposite:
                // shallow, subcritical flow.  The face velocity is therefore
                // capped both by the configured ceiling and by critical flow
                // sqrt(g d) — overland water does not outrun its own wave.  This
                // is what stops a 2 m tidal head at the shore from crossing the
                // whole low plain in a single step and pinning the entire
                // coastal belt at tide level.
                const f64 vCapFace = std::min(opt.velocityCap, std::sqrt(9.81 * dEff));
                f64 vol = std::min(q, vCapFace * s.step * dEff) * dtSub;
                if (vol > budget - moved) vol = budget - moved;
                if (vol <= 0.0) continue;
                // Water that reaches the sea or an open boundary has left the
                // modelled surface; it is not stored in the receiving cell.
                if (s.isSea[nk] || s.isBoundary[nk]) {
                    r.outfallM3 += vol;              // land water leaves the surface
                } else {
                    depth[nk] += vol / area;
                }
                moved += vol;
                touched[nk] = 1;
                if (s.isChannel[k]) r.channelledM3 += vol;   // load on the nullah network
            }
            if (moved > 0.0) {
                depth[k] = dk - moved / area;
                if (!s.isBoundary[k] && depth[k] < opt.minDepth) {
                    // Retiring the last film of a drying cell is a numerical
                    // closure, not a flux: the water is gone from the state, so
                    // it must be gone from the books too.  Over a storm this is
                    // sub-millimetre water on hundreds of thousands of cells and
                    // it is exactly the term that made the balance look broken.
                    r.filmLossM3 += depth[k] * area;
                    depth[k] = 0.0;
                }
            }
        }

        // --- 4. man-made drains ---------------------------------------------
        // Gravity outfalls, so their capacity collapses as the sea rises: at the
        // top of a spring tide they are surcharged and remove very little.  This
        // is the term that makes the same rainfall harmless at low water and
        // catastrophic at high water.
        const f64 derate = opt.tideLockOutfalls
            ? clamp(1.0 - (tideLevel - opt.outfallInvert) / std::max(0.1, opt.outfallDesignHead),
                    0.05, 1.0)
            : 1.0;
        f64 drainOut = 0.0;
        for (u32 k = 0; k < n; ++k) {
            if (s.isSea[k] || s.isBoundary[k] || depth[k] <= opt.minDepth) continue;
            const f64 capacity = static_cast<f64>(land.drainCapacity[k]) * derate;
            if (capacity <= 0.0) continue;
            const f64 removed = std::min<f64>(depth[k], capacity * dtSub);
            depth[k] -= removed;
            drainOut += removed * area;
            touched[k] = 1;
        }
        r.drainedM3 += drainOut;
        }   // routing sub-steps

        // --- 5. sea export and bookkeeping -----------------------------------
        f64 stored = 0.0;
        for (size_t k = 0; k < n; ++k) {
            if (s.isSea[k] || s.isBoundary[k]) continue;   // at the tide level / open edge
            stored += depth[k] * area;
            if (depth[k] > r.maxDepth[k]) {
                r.maxDepth[k] = static_cast<f32>(depth[k]);
                r.timeOfPeakHours[k] = static_cast<f32>(hours);
            }
        }
        r.storedM3 = stored;
        r.peakStorageM3 = std::max(r.peakStorageM3, stored);
    }

    for (size_t k = 0; k < n; ++k) {
        if (s.isSea[k] || s.isBoundary[k]) continue;
        r.finalDepth[k] = static_cast<f32>(depth[k]);
        if (r.maxDepth[k] > 0.05f) {
            ++r.floodedCells;
            r.floodedAreaKm2 += area / 1e6;
        }
        if (r.maxDepth[k] > 0.15f) r.floodedArea150mmKm2 += area / 1e6;
        if (r.maxDepth[k] > 0.50f) r.floodedArea500mmKm2 += area / 1e6;
        if (r.maxDepth[k] > 1.00f) r.floodedArea1000mmKm2 += area / 1e6;
    }
    r.hoursSimulated = steps * dt / 3600.0;
    return r;
}

} // namespace mumbai::hydro
