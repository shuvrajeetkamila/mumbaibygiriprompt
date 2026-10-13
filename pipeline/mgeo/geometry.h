// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/geometry.h
//
//  2D geometry toolkit used by every pipeline stage: polygon area/centroid
//  tests, Douglas-Peucker simplification, arc-length resampling, offsetting,
//  ear-clipping triangulation with hole support, clipping and a uniform grid
//  index.  All in f64 — the pipeline is offline and precision is free there;
//  quantisation to f32 happens exactly once, at chunk serialisation.
// =============================================================================
#pragma once
#include "engine/core/math.h"

namespace mumbai::geo2d {

using Ring = std::vector<Vec2d>;

struct Aabb2d {
    Vec2d min{ 1e30,  1e30};
    Vec2d max{-1e30, -1e30};
    void expand(const Vec2d& p) {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y);
    }
    void expand(const Aabb2d& o) { expand(o.min); expand(o.max); }
    bool valid() const { return max.x >= min.x && max.y >= min.y; }
    bool contains(const Vec2d& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
    }
    bool overlaps(const Aabb2d& o, f64 slack = 0.0) const {
        return min.x - slack <= o.max.x && max.x + slack >= o.min.x &&
               min.y - slack <= o.max.y && max.y + slack >= o.min.y;
    }
    Vec2d center() const { return (min + max) * 0.5; }
    Vec2d size() const { return max - min; }
    f64 area() const { const Vec2d s = size(); return std::max(0.0, s.x) * std::max(0.0, s.y); }
    void grow(f64 r) { min.x -= r; min.y -= r; max.x += r; max.y += r; }
};

struct Polygon {
    Ring outer;
    std::vector<Ring> holes;
    bool empty() const { return outer.size() < 3; }
};

struct Mesh2 {
    std::vector<Vec2d> vertices;
    std::vector<u32> indices;    // triangles
};

// --- measures ---------------------------------------------------------------
f64 signedArea(const Ring& r);
f64 polygonArea(const Polygon& p);
Vec2d centroid(const Ring& r);
f64 perimeter(const Ring& r);
Aabb2d bounds(const Ring& r);
Aabb2d bounds(const Polygon& p);

// --- classification ---------------------------------------------------------
bool pointInRing(const Ring& r, const Vec2d& p);
bool polygonContains(const Polygon& p, const Vec2d& q);   // hole-aware
bool ringIsClockwise(const Ring& r);
void ensureCounterClockwise(Ring& r);

// --- transforms and sampling -------------------------------------------------
Ring simplify(const Ring& r, f64 tolerance);            // Douglas-Peucker, closed
std::vector<Vec2d> simplifyOpen(const std::vector<Vec2d>& pts, f64 tolerance);
std::vector<Vec2d> resampleOpen(const std::vector<Vec2d>& pts, f64 maxEdge);
Ring resampleClosed(const Ring& r, f64 maxEdge);
Ring offsetRing(const Ring& r, f64 delta);              // +delta = outward
Ring chaikin(const Ring& r, int iterations);
Ring rotateRing(const Ring& r, const Vec2d& about, f64 radians);

// --- boolean-ish helpers ----------------------------------------------------
// Sutherland-Hodgman clip of a convex-clipped ring; returns 0 or more rings
// (a concave subject can produce several pieces, so the caller gets a list).
std::vector<Ring> clipRingToBox(const Ring& subject, const Aabb2d& clip);
// Merges a polygon set into convex pieces by simple grid decomposition. Used
// for road/building collision proxies where exact CSG is unnecessary.
std::vector<Aabb2d> decomposeToBoxes(const Polygon& p, f64 maxBoxSize, int maxBoxes);

// --- triangulation ----------------------------------------------------------
Mesh2 triangulate(const Polygon& p);        // ear clipping with hole bridging
Mesh2 triangulateRing(const Ring& r);       // fast path, no holes

// --- queries ----------------------------------------------------------------
f64 distanceSqPointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b);
f64 distancePointSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b);
bool segmentsIntersect(const Vec2d& a, const Vec2d& b, const Vec2d& c, const Vec2d& d);
Vec2d closestPointOnSegment(const Vec2d& p, const Vec2d& a, const Vec2d& b);

// --- uniform grid index -----------------------------------------------------
class GridIndex {
public:
    void build(const std::vector<Aabb2d>& boxes, f64 cellSize);
    void query(const Aabb2d& q, std::vector<u32>& out) const;
    void queryPoint(const Vec2d& p, std::vector<u32>& out) const;
    const Aabb2d& boundsAll() const { return m_bounds; }
    f64 cellSize() const { return m_cell; }

private:
    struct Key {
        i64 x, y;
        bool operator==(const Key& o) const { return x == o.x && y == o.y; }
    };
    struct KeyHash {
        size_t operator()(const Key& k) const {
            return static_cast<size_t>(fnv1a64(static_cast<u64>(k.x) * 0x9E3779B97F4A7C15ull ^
                                               static_cast<u64>(k.y)));
        }
    };
    std::unordered_map<Key, std::vector<u32>, KeyHash> m_cells;
    f64 m_cell = 32.0;
    Aabb2d m_bounds;
};

// --- misc -------------------------------------------------------------------
// Number of triangles / vertices a mesh contains (used for budget reporting).
struct MeshStats { u64 vertices = 0, triangles = 0; };
MeshStats meshStats(const Mesh2& m);

} // namespace mumbai::geo2d
