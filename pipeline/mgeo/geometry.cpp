// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/geometry.cpp
// =============================================================================
#include "pipeline/mgeo/geometry.h"

namespace mumbai::geo2d {

// -----------------------------------------------------------------------------
//  Measures
// -----------------------------------------------------------------------------
f64 signedArea(const Ring& r) {
    if (r.size() < 3) return 0.0;
    f64 a = 0.0;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
        a += (r[j].x * r[i].y) - (r[i].x * r[j].y);
    return a * 0.5;
}

f64 polygonArea(const Polygon& p) {
    f64 a = std::fabs(signedArea(p.outer));
    for (const Ring& h : p.holes) a -= std::fabs(signedArea(h));
    return std::max(0.0, a);
}

Vec2d centroid(const Ring& r) {
    if (r.empty()) return {};
    const f64 a = signedArea(r);
    if (std::fabs(a) < 1e-12) {   // degenerate: fall back to vertex average
        Vec2d c;
        for (const Vec2d& p : r) c = c + p;
        return c / static_cast<f64>(r.size());
    }
    Vec2d c;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
        const f64 cross = r[j].x * r[i].y - r[i].x * r[j].y;
        c = c + (r[j] + r[i]) * cross;
    }
    return c / (6.0 * a);
}

f64 perimeter(const Ring& r) {
    f64 p = 0.0;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
        p += (r[i] - r[j]).length();
    return p;
}

Aabb2d bounds(const Ring& r) {
    Aabb2d b;
    for (const Vec2d& p : r) b.expand(p);
    return b;
}

Aabb2d bounds(const Polygon& p) { return bounds(p.outer); }

// -----------------------------------------------------------------------------
//  Classification
// -----------------------------------------------------------------------------
bool pointInRing(const Ring& r, const Vec2d& p) {
    bool inside = false;
    for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
        const Vec2d& a = r[i];
        const Vec2d& b = r[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-30) + a.x))
            inside = !inside;
    }
    return inside;
}

bool polygonContains(const Polygon& p, const Vec2d& q) {
    if (!pointInRing(p.outer, q)) return false;
    for (const Ring& h : p.holes)
        if (pointInRing(h, q)) return false;
    return true;
}

bool ringIsClockwise(const Ring& r) { return signedArea(r) < 0.0; }

void ensureCounterClockwise(Ring& r) {
    if (ringIsClockwise(r)) std::reverse(r.begin(), r.end());
}

// -----------------------------------------------------------------------------
//  Simplification
// -----------------------------------------------------------------------------
namespace {
void dpRecurse(const std::vector<Vec2d>& pts, size_t first, size_t last,
               f64 tolerance, std::vector<bool>& keep) {
    if (last <= first + 1) return;
    const Vec2d a = pts[first], b = pts[last];
    f64 maxDist = -1.0;
    size_t index = first;
    for (size_t i = first + 1; i < last; ++i) {
        const f64 d = distancePointSegment(pts[i], a, b);
        if (d > maxDist) { maxDist = d; index = i; }
    }
    if (maxDist > tolerance) {
        keep[index] = true;
        dpRecurse(pts, first, index, tolerance, keep);
        dpRecurse(pts, index, last, tolerance, keep);
    }
}
} // namespace

std::vector<Vec2d> simplifyOpen(const std::vector<Vec2d>& pts, f64 tolerance) {
    if (pts.size() < 3) return pts;
    std::vector<bool> keep(pts.size(), false);
    keep.front() = keep.back() = true;
    dpRecurse(pts, 0, pts.size() - 1, tolerance, keep);
    std::vector<Vec2d> out;
    out.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        if (keep[i]) out.push_back(pts[i]);
    return out;
}

Ring simplify(const Ring& r, f64 tolerance) {
    if (r.size() < 4) return r;
    // Closed rings: split at the two extreme points to avoid the degenerate
    // first == last case of a naive pass.
    const Aabb2d b = bounds(r);
    size_t iMin = 0, iMax = 0;
    f64 dMin = 1e300, dMax = -1e300;
    for (size_t i = 0; i < r.size(); ++i) {
        const f64 d = r[i].x + r[i].y;
        if (d < dMin) { dMin = d; iMin = i; }
        if (d > dMax) { dMax = d; iMax = i; }
    }
    if (iMin == iMax) return r;

    std::vector<Vec2d> pathA, pathB;
    for (size_t i = iMin;; i = (i + 1) % r.size()) {
        pathA.push_back(r[i]);
        if (i == iMax) break;
    }
    for (size_t i = iMax;; i = (i + 1) % r.size()) {
        pathB.push_back(r[i]);
        if (i == iMin) break;
    }
    auto sa = simplifyOpen(pathA, tolerance);
    auto sb = simplifyOpen(pathB, tolerance);
    // Drop duplicated endpoints then stitch.
    if (sa.size() > 1) sa.pop_back();
    if (sb.size() > 1) sb.pop_back();
    Ring out = sa;
    out.insert(out.end(), sb.begin(), sb.end());
    if (out.size() < 3) return r;
    (void)b;
    return out;
}

// -----------------------------------------------------------------------------
//  Resampling (equal-length edges) — keeps road ribbons and facades uniform,
//  which is what makes the texture / UV budget predictable.
// -----------------------------------------------------------------------------
std::vector<Vec2d> resampleOpen(const std::vector<Vec2d>& pts, f64 maxEdge) {
    if (pts.size() < 2) return pts;
    std::vector<Vec2d> out;
    out.push_back(pts.front());
    for (size_t i = 1; i < pts.size(); ++i) {
        const Vec2d a = pts[i - 1], b = pts[i];
        const f64 len = (b - a).length();
        const int steps = std::max(1, static_cast<int>(std::ceil(len / maxEdge)));
        for (int s = 1; s <= steps; ++s) {
            const f64 t = static_cast<f64>(s) / steps;
            out.push_back(a + (b - a) * t);
        }
    }
    return out;
}

Ring resampleClosed(const Ring& r, f64 maxEdge) {
    if (r.size() < 3) return r;
    Ring out;
    for (size_t i = 0; i < r.size(); ++i) {
        const Vec2d a = r[i];
        const Vec2d b = r[(i + 1) % r.size()];
        const f64 len = (b - a).length();
        const int steps = std::max(1, static_cast<int>(std::ceil(len / maxEdge)));
        for (int s = 0; s < steps; ++s) {
            const f64 t = static_cast<f64>(s) / steps;
            out.push_back(a + (b - a) * t);
        }
    }
    return out.size() >= 3 ? out : r;
}

Ring chaikin(const Ring& r, int iterations) {
    Ring cur = r;
    for (int it = 0; it < iterations; ++it) {
        if (cur.size() < 3) break;
        Ring next;
        next.reserve(cur.size() * 2);
        for (size_t i = 0; i < cur.size(); ++i) {
            const Vec2d& p = cur[i];
            const Vec2d& q = cur[(i + 1) % cur.size()];
            next.push_back(p * 0.75 + q * 0.25);
            next.push_back(p * 0.25 + q * 0.75);
        }
        cur.swap(next);
    }
    return cur;
}

Ring rotateRing(const Ring& r, const Vec2d& about, f64 radians) {
    const f64 c = std::cos(radians), s = std::sin(radians);
    Ring out;
    out.reserve(r.size());
    for (const Vec2d& p : r) {
        const Vec2d d = p - about;
        out.push_back(Vec2d{about.x + d.x * c - d.y * s, about.y + d.x * s + d.y * c});
    }
    return out;
}

Ring offsetRing(const Ring& r, f64 delta) {
    if (r.size() < 3) return r;
    Ring out;
    out.reserve(r.size());
    const bool ccw = !ringIsClockwise(r);
    const f64 sgn = ccw ? 1.0 : -1.0;
    for (size_t i = 0; i < r.size(); ++i) {
        const Vec2d& prev = r[(i + r.size() - 1) % r.size()];
        const Vec2d& cur = r[i];
        const Vec2d& next = r[(i + 1) % r.size()];
        const Vec2d d1 = (cur - prev).normalized();
        const Vec2d d2 = (next - cur).normalized();
        Vec2d n1{d1.y, -d1.x};    // right-hand normal
        Vec2d n2{d2.y, -d2.x};
        Vec2d n = (n1 + n2).normalized();
        if (n.lengthSq() < 1e-12) n = n1;
        // Miter compensation, clamped to avoid spikes on sharp corners
        const f64 cosHalf = std::max(0.3, n.dot(n1));
        out.push_back(cur - n * (delta * sgn / cosHalf));
    }
    return out;
}

// -----------------------------------------------------------------------------
//  Clipping
// -----------------------------------------------------------------------------
namespace {
std::vector<Vec2d> clipEdge(const std::vector<Vec2d>& poly, int axis, f64 value, bool keepGreater) {
    std::vector<Vec2d> out;
    if (poly.empty()) return out;
    for (size_t i = 0; i < poly.size(); ++i) {
        const Vec2d& cur = poly[i];
        const Vec2d& prev = poly[(i + poly.size() - 1) % poly.size()];
        const f64 cv = axis == 0 ? cur.x : cur.y;
        const f64 pv = axis == 0 ? prev.x : prev.y;
        const bool curIn = keepGreater ? (cv >= value) : (cv <= value);
        const bool prevIn = keepGreater ? (pv >= value) : (pv <= value);
        if (curIn) {
            if (!prevIn) {
                const f64 t = (value - pv) / (cv - pv + 1e-30);
                out.push_back(prev + (cur - prev) * t);
            }
            out.push_back(cur);
        } else if (prevIn) {
            const f64 t = (value - pv) / (cv - pv + 1e-30);
            out.push_back(prev + (cur - prev) * t);
        }
    }
    return out;
}
} // namespace

std::vector<Ring> clipRingToBox(const Ring& subject, const Aabb2d& clip) {
    std::vector<Vec2d> poly(subject.begin(), subject.end());
    poly = clipEdge(poly, 0, clip.min.x, true);
    poly = clipEdge(poly, 0, clip.max.x, false);
    poly = clipEdge(poly, 1, clip.min.y, true);
    poly = clipEdge(poly, 1, clip.max.y, false);
    std::vector<Ring> out;
    if (poly.size() >= 3 && std::fabs(signedArea(poly)) > 1e-6)
        out.push_back(std::move(poly));
    return out;
}

std::vector<Aabb2d> decomposeToBoxes(const Polygon& p, f64 maxBoxSize, int maxBoxes) {
    std::vector<Aabb2d> out;
    const Aabb2d b = bounds(p);
    if (!b.valid()) return out;
    const Vec2d sz = b.size();
    const int nx = std::max(1, std::min(static_cast<int>(std::ceil(sz.x / maxBoxSize)), 8));
    const int ny = std::max(1, std::min(static_cast<int>(std::ceil(sz.y / maxBoxSize)), 8));
    const f64 dx = sz.x / nx, dy = sz.y / ny;
    for (int y = 0; y < ny && static_cast<int>(out.size()) < maxBoxes; ++y) {
        for (int x = 0; x < nx && static_cast<int>(out.size()) < maxBoxes; ++x) {
            const Vec2d c{b.min.x + (x + 0.5) * dx, b.min.y + (y + 0.5) * dy};
            if (!polygonContains(p, c)) continue;

            // A cell whose centre is inside can still overhang the footprint:
            // its corners may fall outside an L-shaped or curved plan.  A
            // collision box that overhangs is a box the player walks into in
            // mid-air, so shrink the cell until its boundary samples are all
            // inside, and drop it if it becomes too small to matter.
            Aabb2d cell;
            f64 hx = dx * 0.5, hz = dy * 0.5;
            bool placed = false;
            for (int attempt = 0; attempt < 14; ++attempt) {
                cell.min = {c.x - hx, c.y - hz};
                cell.max = {c.x + hx, c.y + hz};
                bool contained = true;
                for (int sx = -1; sx <= 1 && contained; ++sx) {
                    for (int sz = -1; sz <= 1; ++sz) {
                        const Vec2d probe{c.x + sx * hx, c.y + sz * hz};
                        if (!polygonContains(p, probe)) { contained = false; break; }
                    }
                }
                if (contained) { placed = true; break; }
                hx *= 0.75; hz *= 0.75;
                if (hx < 0.35 || hz < 0.35) break;
            }
            if (!placed) continue;
            out.push_back(cell);
        }
    }
    return out;
}

// -----------------------------------------------------------------------------
//  Triangulation — ear clipping with hole bridging.
//
//  Hole handling: each hole's rightmost vertex is bridged to a mutually visible
//  vertex on the outer ring, producing a single simple polygon. This is the
//  classic approach used by earcut-style triangulators and is sufficient for
//  architectural footprints (buildings with courtyards, water bodies with
//  islands). Robustness measures: collinear/zero-area ears are skipped, and the
//  process falls back to a fan triangulation if no ear can be found, which
//  guarantees output geometry even for pathological input (better a slightly
//  wrong roof than a hole in the world).
// -----------------------------------------------------------------------------
namespace {

bool pointInTriangle(const Vec2d& p, const Vec2d& a, const Vec2d& b, const Vec2d& c) {
    const f64 d1 = (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
    const f64 d2 = (p.x - c.x) * (b.y - c.y) - (b.x - c.x) * (p.y - c.y);
    const f64 d3 = (p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y);
    const bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(hasNeg && hasPos);
}

f64 cross2(const Vec2d& o, const Vec2d& a, const Vec2d& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Bridge a hole into the outer ring: find the hole's leftmost vertex (a
// rightmost hole vertex would be inside a CCW outer ring's concave region in
// general; leftmost is the conventional safe choice for CCW rings) and connect
// it to the closest outer vertex for which the connecting segment does not
// cross any edge.
Ring bridgeHole(const Ring& outer, const Ring& hole) {
    if (hole.empty()) return outer;
    size_t hi = 0;
    for (size_t i = 1; i < hole.size(); ++i)
        if (hole[i].x < hole[hi].x) hi = i;

    const Vec2d hp = hole[hi];
    size_t best = SIZE_MAX;
    f64 bestDist = 1e300;
    for (size_t oi = 0; oi < outer.size(); ++oi) {
        const Vec2d& op = outer[oi];
        const f64 d = (op - hp).lengthSq();
        if (d >= bestDist) continue;
        // Visibility test against outer edges (excluding edges incident to oi).
        bool blocked = false;
        for (size_t ei = 0; ei < outer.size() && !blocked; ++ei) {
            const size_t ej = (ei + 1) % outer.size();
            if (ei == oi || ej == oi) continue;
            if (segmentsIntersect(hp, op, outer[ei], outer[ej])) blocked = true;
        }
        for (size_t ei = 0; ei < hole.size() && !blocked; ++ei) {
            const size_t ej = (ei + 1) % hole.size();
            if (ei == hi || ej == hi) continue;
            if (segmentsIntersect(hp, op, hole[ei], hole[ej])) blocked = true;
        }
        if (!blocked) { best = oi; bestDist = d; }
    }
    if (best == SIZE_MAX) {
        // No visible vertex (should not happen for valid input): fall back to
        // the nearest vertex without the visibility test.
        f64 d2 = 1e300;
        for (size_t oi = 0; oi < outer.size(); ++oi) {
            const f64 d = (outer[oi] - hp).lengthSq();
            if (d < d2) { d2 = d; best = oi; }
        }
        if (best == SIZE_MAX) return outer;
    }

    Ring merged;
    merged.reserve(outer.size() + hole.size() + 2);
    for (size_t i = 0; i <= best; ++i) merged.push_back(outer[i]);
    for (size_t k = 0; k < hole.size(); ++k) merged.push_back(hole[(hi + k) % hole.size()]);
    merged.push_back(hole[hi]);
    for (size_t i = best; i < outer.size(); ++i) merged.push_back(outer[i]);
    return merged;
}

Mesh2 earClip(const Ring& ringIn) {
    Mesh2 mesh;
    Ring ring = ringIn;
    if (ring.size() < 3) return mesh;
    ensureCounterClockwise(ring);

    // Remove exact duplicates that would create zero-area ears.
    Ring clean;
    for (size_t i = 0; i < ring.size(); ++i) {
        const Vec2d& p = ring[i];
        if (!clean.empty() && (clean.back() - p).lengthSq() < 1e-12) continue;
        clean.push_back(p);
    }
    if (clean.size() >= 2 && (clean.front() - clean.back()).lengthSq() < 1e-12)
        clean.pop_back();
    if (clean.size() < 3) return mesh;

    mesh.vertices = clean;
    std::vector<u32> idx(clean.size());
    for (u32 i = 0; i < idx.size(); ++i) idx[i] = i;

    size_t guard = 0;
    const size_t maxIterations = clean.size() * clean.size() + 16;
    while (idx.size() > 3 && guard++ < maxIterations) {
        bool clipped = false;
        const size_t count = idx.size();
        for (size_t i = 0; i < count; ++i) {
            const u32 i0 = idx[(i + count - 1) % count];
            const u32 i1 = idx[i];
            const u32 i2 = idx[(i + 1) % count];
            const Vec2d& a = clean[i0];
            const Vec2d& b = clean[i1];
            const Vec2d& c = clean[i2];

            const f64 area2 = cross2(a, b, c);
            if (area2 <= 1e-14) continue;   // reflex or degenerate

            // No other vertex may lie inside the candidate ear.
            bool containsVertex = false;
            for (size_t k = 0; k < count && !containsVertex; ++k) {
                const u32 ik = idx[k];
                if (ik == i0 || ik == i1 || ik == i2) continue;
                if (pointInTriangle(clean[ik], a, b, c)) containsVertex = true;
            }
            if (containsVertex) continue;

            mesh.indices.push_back(i0);
            mesh.indices.push_back(i1);
            mesh.indices.push_back(i2);
            idx.erase(idx.begin() + static_cast<long>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            // Fallback: fan the remainder so the surface stays watertight.
            for (size_t i = 1; i + 1 < idx.size(); ++i) {
                mesh.indices.push_back(idx[0]);
                mesh.indices.push_back(idx[i]);
                mesh.indices.push_back(idx[i + 1]);
            }
            return mesh;
        }
    }
    if (idx.size() == 3) {
        mesh.indices.push_back(idx[0]);
        mesh.indices.push_back(idx[1]);
        mesh.indices.push_back(idx[2]);
    }
    return mesh;
}

} // namespace

Mesh2 triangulateRing(const Ring& r) { return earClip(r); }

Mesh2 triangulate(const Polygon& p) {
    if (p.empty()) return {};
    Ring outer = p.outer;
    ensureCounterClockwise(outer);
    for (const Ring& h : p.holes) {
        if (h.size() < 3) continue;
        // Holes run clockwise, the outer ring counter-clockwise.  The bridging
        // walk relies on that opposition: splicing a co-oriented hole into the
        // outer ring leaves the hole's interior filled (which is how a courtyard
        // turns into solid roof).
        Ring hole = h;
        if (!ringIsClockwise(hole)) std::reverse(hole.begin(), hole.end());
        outer = bridgeHole(outer, hole);
    }
    return earClip(outer);
}

MeshStats meshStats(const Mesh2& m) {
    MeshStats s;
    s.vertices = m.vertices.size();
    s.triangles = m.indices.size() / 3;
    return s;
}

// -----------------------------------------------------------------------------
//  Queries
// -----------------------------------------------------------------------------
f64 distanceSqPointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b) {
    const Vec2d ab = b - a;
    const f64 len2 = ab.lengthSq();
    if (len2 < 1e-18) return (p - a).lengthSq();
    f64 t = (p - a).dot(ab) / len2;
    t = clamp(t, 0.0, 1.0);
    const Vec2d proj = a + ab * t;
    return (p - proj).lengthSq();
}

f64 distancePointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b) {
    return std::sqrt(distanceSqPointSegment(p, a, b));
}

Vec2d closestPointOnSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b) {
    const Vec2d ab = b - a;
    const f64 len2 = ab.lengthSq();
    if (len2 < 1e-18) return a;
    const f64 t = clamp((p - a).dot(ab) / len2, 0.0, 1.0);
    return a + ab * t;
}

bool segmentsIntersect(const Vec2d& a, const Vec2d& b, const Vec2d& c, const Vec2d& d) {
    auto orient = [](const Vec2d& p, const Vec2d& q, const Vec2d& r) {
        const f64 v = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        return v > 1e-12 ? 1 : (v < -1e-12 ? -1 : 0);
    };
    const int o1 = orient(a, b, c);
    const int o2 = orient(a, b, d);
    const int o3 = orient(c, d, a);
    const int o4 = orient(c, d, b);
    return o1 != o2 && o3 != o4;
}

// -----------------------------------------------------------------------------
//  GridIndex
// -----------------------------------------------------------------------------
void GridIndex::build(const std::vector<Aabb2d>& boxes, f64 cellSize) {
    m_cells.clear();
    m_bounds = Aabb2d{};
    m_cell = std::max(1.0, cellSize);
    for (const Aabb2d& b : boxes) m_bounds.expand(b);
    if (!m_bounds.valid()) return;
    for (u32 i = 0; i < boxes.size(); ++i) {
        const Aabb2d& b = boxes[i];
        const i64 x0 = static_cast<i64>(std::floor(b.min.x / m_cell));
        const i64 x1 = static_cast<i64>(std::floor(b.max.x / m_cell));
        const i64 y0 = static_cast<i64>(std::floor(b.min.y / m_cell));
        const i64 y1 = static_cast<i64>(std::floor(b.max.y / m_cell));
        for (i64 y = y0; y <= y1; ++y)
            for (i64 x = x0; x <= x1; ++x)
                m_cells[Key{x, y}].push_back(i);
    }
}

void GridIndex::query(const Aabb2d& q, std::vector<u32>& out) const {
    out.clear();
    if (m_cells.empty()) return;
    const i64 x0 = static_cast<i64>(std::floor(q.min.x / m_cell));
    const i64 x1 = static_cast<i64>(std::floor(q.max.x / m_cell));
    const i64 y0 = static_cast<i64>(std::floor(q.min.y / m_cell));
    const i64 y1 = static_cast<i64>(std::floor(q.max.y / m_cell));
    for (i64 y = y0; y <= y1; ++y) {
        for (i64 x = x0; x <= x1; ++x) {
            auto it = m_cells.find(Key{x, y});
            if (it == m_cells.end()) continue;
            out.insert(out.end(), it->second.begin(), it->second.end());
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

void GridIndex::queryPoint(const Vec2d& p, std::vector<u32>& out) const {
    Aabb2d q;
    q.expand(p);
    query(q, out);
}

} // namespace mumbai::geo2d
