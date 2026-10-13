// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/synth.cpp
//  See synth.h for the rationale; DOCS/D-020 for the data-availability decision.
// =============================================================================
#include "pipeline/mgeo/synth.h"

#include <algorithm>
#include <cmath>

namespace mumbai::synth {

f64 latticeJitter(i64 index, u64 seed) {
    u64 h = static_cast<u64>(index) * 0x9E3779B97F4A7C15ull + seed * 0xD1B54A32D192ED03ull;
    h ^= h >> 30; h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 27; h *= 0x94D049BB133111EBull;
    h ^= h >> 31;
    return static_cast<f64>(h >> 11) * (1.0 / 9007199254740992.0) - 0.5;   // [-0.5, 0.5)
}

std::vector<Vec2d> smoothThrough(const std::vector<Vec2d>& control, f64 maxEdge) {
    std::vector<Vec2d> out;
    if (control.size() < 2) return control;
    if (maxEdge < 1.0) maxEdge = 1.0;
    out.reserve(control.size() * 8);

    const size_t n = control.size();
    for (size_t i = 0; i + 1 < n; ++i) {
        const Vec2d& p0 = control[i == 0 ? 0 : i - 1];
        const Vec2d& p1 = control[i];
        const Vec2d& p2 = control[i + 1];
        const Vec2d& p3 = control[i + 2 < n ? i + 2 : n - 1];

        const f64 segLen = (p2 - p1).length();
        const int steps = std::max(1, static_cast<int>(std::ceil(segLen / maxEdge)));
        for (int s = 0; s < steps; ++s) {
            const f64 t = static_cast<f64>(s) / static_cast<f64>(steps);
            // Catmull-Rom -> cubic Bezier control points, then de Casteljau.
            const Vec2d b0 = p1;
            const Vec2d b1 = p1 + (p2 - p0) * (1.0 / 6.0);
            const Vec2d b2 = p2 - (p3 - p1) * (1.0 / 6.0);
            const Vec2d b3 = p2;
            const f64 u = 1.0 - t;
            const Vec2d pt = b0 * (u * u * u) + b1 * (3.0 * u * u * t) +
                             b2 * (3.0 * u * t * t) + b3 * (t * t * t);
            out.push_back(pt);
        }
    }
    out.push_back(control.back());
    return out;
}

namespace {

// Interval of a scan line that lies inside the polygon (holes honoured by the
// even-odd rule applied across every ring).
struct Span { f64 lo, hi; };

std::vector<Span> spansOnLine(const std::vector<geo2d::Ring>& rings, f64 y) {
    std::vector<f64> xs;
    for (const geo2d::Ring& r : rings) {
        for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
            const Vec2d& a = r[j];
            const Vec2d& b = r[i];
            if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
                const f64 t = (y - a.y) / (b.y - a.y);
                xs.push_back(a.x + t * (b.x - a.x));
            }
        }
    }
    std::sort(xs.begin(), xs.end());
    std::vector<Span> spans;
    for (size_t i = 0; i + 1 < xs.size(); i += 2) {
        if (xs[i + 1] - xs[i] > 1e-6) spans.push_back({xs[i], xs[i + 1]});
    }
    return spans;
}

// Uniform-grid proximity test for "is this generated street sitting on top of
// an authored arterial?".
struct ArterialIndex {
    static constexpr f64 CELL = 64.0;
    std::unordered_map<u64, std::vector<std::pair<Vec2d, Vec2d>>> buckets;

    static u64 key(i64 i, i64 j) {
        return (static_cast<u64>(i) * 0x9E3779B97F4A7C15ull) ^ static_cast<u64>(j) * 0xC2B2AE3D27D4EB4Full;
    }
    void add(const std::vector<std::vector<Vec2d>>& chains) {
        for (const auto& chain : chains) {
            for (size_t i = 0; i + 1 < chain.size(); ++i) {
                const Vec2d& a = chain[i];
                const Vec2d& b = chain[i + 1];
                // Split long edges so every bucket entry stays short.
                const f64 len = (b - a).length();
                const int steps = std::max(1, static_cast<int>(len / CELL));
                for (int s = 0; s < steps; ++s) {
                    const Vec2d p = a + (b - a) * (static_cast<f64>(s) / steps);
                    const Vec2d q = a + (b - a) * (static_cast<f64>(s + 1) / steps);
                    const i64 ci = static_cast<i64>(std::floor(p.x / CELL));
                    const i64 cj = static_cast<i64>(std::floor(p.y / CELL));
                    buckets[key(ci, cj)].push_back({p, q});
                }
            }
        }
    }
    bool near(const Vec2d& p, f64 radius) const {
        const i64 ci = static_cast<i64>(std::floor(p.x / CELL));
        const i64 cj = static_cast<i64>(std::floor(p.y / CELL));
        const i64 span = static_cast<i64>(std::ceil(radius / CELL));
        for (i64 i = ci - span; i <= ci + span; ++i) {
            for (i64 j = cj - span; j <= cj + span; ++j) {
                auto it = buckets.find(key(i, j));
                if (it == buckets.end()) continue;
                for (const auto& seg : it->second) {
                    if (geo2d::distancePointSegment(p, seg.first, seg.second) < radius) return true;
                }
            }
        }
        return false;
    }
};

}  // namespace

FabricOut buildUrbanFabric(const geo2d::Polygon& land, f64 bearing, f64 spacing,
                           f64 streetJitter, f64 arterialDist,
                           const std::vector<std::vector<Vec2d>>& arterials,
                           f64 coverage, f64 margin, u32 lots,
                           const std::function<bool(const Vec2d&)>& accept,
                           Rng& rng) {
    FabricOut out;
    if (land.outer.size() < 3 || spacing < 8.0) return out;
    lots = std::max<u32>(1, std::min<u32>(lots, 8));

    const f64 cs = std::cos(bearing), sn = std::sin(bearing);
    auto toLattice = [&](const Vec2d& p) {
        return Vec2d{p.x * cs + p.y * sn, -p.x * sn + p.y * cs};
    };
    auto toWorld = [&](const Vec2d& p) {
        return Vec2d{p.x * cs - p.y * sn, p.x * sn + p.y * cs};
    };

    std::vector<geo2d::Ring> ringsL;
    ringsL.reserve(1 + land.holes.size());
    {
        geo2d::Ring r;
        r.reserve(land.outer.size());
        for (const Vec2d& p : land.outer) r.push_back(toLattice(p));
        ringsL.push_back(std::move(r));
    }
    for (const geo2d::Ring& h : land.holes) {
        geo2d::Ring r;
        r.reserve(h.size());
        for (const Vec2d& p : h) r.push_back(toLattice(p));
        ringsL.push_back(std::move(r));
    }

    const geo2d::Aabb2d bb = geo2d::bounds(ringsL[0]);

    // --- jittered street positions, shared by the lattice and the parcels ----
    const i64 i0 = static_cast<i64>(std::floor(bb.min.x / spacing)) - 1;
    const i64 i1 = static_cast<i64>(std::ceil(bb.max.x / spacing)) + 1;
    const i64 j0 = static_cast<i64>(std::floor(bb.min.y / spacing)) - 1;
    const i64 j1 = static_cast<i64>(std::ceil(bb.max.y / spacing)) + 1;
    if ((i1 - i0) > 4096 || (j1 - j0) > 4096) return out;    // degenerate input guard

    std::vector<f64> xs, ys;
    xs.reserve(static_cast<size_t>(i1 - i0));
    ys.reserve(static_cast<size_t>(j1 - j0));
    for (i64 i = i0; i <= i1; ++i)
        xs.push_back((i + latticeJitter(i, 0x51ull) * streetJitter) * spacing);
    for (i64 j = j0; j <= j1; ++j)
        ys.push_back((j + latticeJitter(j, 0x9Cull) * streetJitter) * spacing);

    ArterialIndex arIndex;
    if (arterialDist > 0.5) arIndex.add(arterials);
    const f64 minStreetLen = spacing * 0.35;

    auto emitStreet = [&](const Vec2d& aL, const Vec2d& bL, u8 rank) {
        if ((bL - aL).length() < minStreetLen) return;
        const Vec2d aw = toWorld(aL), bw = toWorld(bL);
        const Vec2d mid = (aw + bw) * 0.5;
        if (!accept(mid)) return;
        if (arterialDist > 0.5 && (arIndex.near(mid, arterialDist) ||
                                   arIndex.near(aw, arterialDist) ||
                                   arIndex.near(bw, arterialDist)))
            return;
        StreetSeg s;
        s.a = aw; s.b = bw; s.rank = rank;
        s.halfWidth = rank == 0 ? 4.0 : 2.6;
        out.streets.push_back(s);
    };

    // --- streets -------------------------------------------------------------
    // Horizontal family: one scan line per Y, split at every X lattice position
    // that falls inside the polygon so that crossings become real junctions.
    for (f64 Y : ys) {
        for (const Span& sp : spansOnLine(ringsL, Y)) {
            f64 cursor = sp.lo;
            for (f64 X : xs) {
                if (X <= cursor + 1e-6) continue;
                if (X >= sp.hi) break;
                emitStreet(Vec2d{cursor, Y}, Vec2d{X, Y}, 1);
                cursor = X;
            }
            emitStreet(Vec2d{cursor, Y}, Vec2d{sp.hi, Y}, 1);
        }
    }
    // Vertical family: the same scan applied to the transposed rings, which
    // yields the y-intervals that are inside the polygon at x == X.
    std::vector<geo2d::Ring> ringsT;
    ringsT.reserve(ringsL.size());
    for (const geo2d::Ring& r : ringsL) {
        geo2d::Ring t;
        t.reserve(r.size());
        for (const Vec2d& p : r) t.push_back(Vec2d{p.y, p.x});
        ringsT.push_back(std::move(t));
    }
    for (f64 X : xs) {
        for (const Span& sp : spansOnLine(ringsT, X)) {
            f64 cursor = sp.lo;
            for (f64 Y : ys) {
                if (Y <= cursor + 1e-6) continue;
                if (Y >= sp.hi) break;
                emitStreet(Vec2d{X, cursor}, Vec2d{X, Y}, 1);
                cursor = Y;
            }
            emitStreet(Vec2d{X, cursor}, Vec2d{X, sp.hi}, 1);
        }
    }

    // --- building parcels ----------------------------------------------------
    // Every lattice cell is subdivided into `lots` x `lots` plots.  A plot is
    // inset by `margin` from the street and by a further quarter of the residual
    // gap so neighbours never interpenetrate.
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        for (size_t j = 0; j + 1 < ys.size(); ++j) {
            const f64 cw = xs[i + 1] - xs[i];
            const f64 ch = ys[j + 1] - ys[j];
            if (cw <= 2.0 * margin || ch <= 2.0 * margin) continue;

            const Vec2d centreL{(xs[i] + xs[i + 1]) * 0.5, (ys[j] + ys[j + 1]) * 0.5};
            if (!accept(toWorld(centreL))) continue;

            const f64 pw = cw / lots, ph = ch / lots;
            for (u32 a = 0; a < lots; ++a) {
                for (u32 b = 0; b < lots; ++b) {
                    if (!rng.chance(static_cast<f32>(coverage))) continue;
                    const f64 gapX = std::max(0.4, (pw - 2.0 * margin) * 0.06);
                    const f64 gapY = std::max(0.4, (ph - 2.0 * margin) * 0.06);
                    const f64 x0 = xs[i] + a * pw + margin;
                    const f64 x1 = xs[i] + (a + 1) * pw - margin - gapX;
                    const f64 y0 = ys[j] + b * ph + margin;
                    const f64 y1 = ys[j] + (b + 1) * ph - margin - gapY;
                    if (x1 - x0 < 3.0 || y1 - y0 < 3.0) continue;

                    // Randomise the plot a little: real plots are not all the
                    // same width, and the inset varies street by street.
                    const f64 shrink = 0.82 + 0.18 * rng.nextF32();
                    const f64 cxL = (x0 + x1) * 0.5, cyL = (y0 + y1) * 0.5;
                    const f64 hw = (x1 - x0) * 0.5 * shrink;
                    const f64 hh = (y1 - y0) * 0.5 * shrink;

                    geo2d::Ring quad;
                    quad.reserve(4);
                    quad.push_back(toWorld(Vec2d{cxL - hw, cyL - hh}));
                    quad.push_back(toWorld(Vec2d{cxL + hw, cyL - hh}));
                    quad.push_back(toWorld(Vec2d{cxL + hw, cyL + hh}));
                    quad.push_back(toWorld(Vec2d{cxL - hw, cyL + hh}));

                    const Vec2d cw2 = toWorld(Vec2d{cxL, cyL});
                    bool ok = accept(cw2);
                    for (const Vec2d& c : quad) ok = ok && accept(c);
                    if (!ok) continue;
                    geo2d::ensureCounterClockwise(quad);
                    out.footprints.push_back(std::move(quad));
                    out.footprintCentre.push_back(cw2);
                }
            }
        }
    }
    return out;
}

}  // namespace mumbai::synth
