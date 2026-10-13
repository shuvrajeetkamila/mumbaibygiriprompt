// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/geojson.h
//
//  GeoJSON reader for the administrative and settlement layers (BMC wards,
//  slum clusters).  Handles FeatureCollection / Feature / Polygon /
//  MultiPolygon / LineString / MultiLineString / Point, holes, and the
//  `properties` bag the zoning rules key off.
// =============================================================================
#pragma once
#include "pipeline/mgeo/geometry.h"
#include "engine/core/geo.h"
#include "engine/core/json.h"

namespace mumbai::geojson {

struct Feature {
    geo2d::Polygon polygon;                 // filled for polygon geometries
    std::vector<geo2d::Ring> lines;         // filled for line geometries
    std::vector<Vec2d> points;              // filled for point geometries
    std::map<std::string, std::string> properties;
    std::string name;                       // convenience copy of `name`
};

struct Collection {
    std::vector<Feature> features;
    std::string geometryType;               // dominant geometry kind seen
};

// Reads a GeoJSON file, projecting WGS84 degrees into world metres with
// `proj`.  Returns false with a diagnostic on malformed input.
bool load(const std::string& path, Collection& out, const TransverseMercator& proj,
          std::string* error = nullptr);

// A set of polygons with a uniform-grid index, used as a point-in-region and
// distance-to-region query for the whole city (the BMC ward union is ~600 km^2
// of geometry across 24 multi-polygons; a true polygon union is both
// unnecessary and fragile, so queries are answered against the indexed set).
class MultiMask {
public:
    void build(std::vector<geo2d::Polygon> polys, f64 gridSize = 512.0);

    // Rasterises the mask over `extent` and precomputes a distance transform to
    // the nearest boundary.  This is what makes the terrain synthesiser able to
    // evaluate the land/sea decision and the shore falloff for ~2 million
    // samples in seconds instead of hours: queries become array lookups, with
    // the exact polygon test retained only for cells near the boundary (where
    // the raster is ambiguous), so results stay exact where it matters.
    void rasterize(const geo2d::Aabb2d& extent, f64 cellSize = 32.0);

    bool contains(const Vec2d& p) const;
    // Distance to the nearest boundary, capped at maxSearch.
    f64 distanceToBoundary(const Vec2d& p, f64 maxSearch) const;
    bool anyWithin(const Vec2d& p, f64 radius) const;
    size_t polygonCount() const { return m_polys.size(); }
    const std::vector<geo2d::Polygon>& polygons() const { return m_polys; }
    bool empty() const { return m_polys.empty(); }
    bool hasRaster() const { return !m_raster.empty(); }
    f64 rasterCellSize() const { return m_rastCell; }

private:
    std::vector<u32> candidates(const Vec2d& p, f64 pad) const;
    bool exactContains(const Vec2d& p) const;

    std::vector<geo2d::Polygon> m_polys;
    std::vector<geo2d::Aabb2d> m_bounds;
    geo2d::GridIndex m_index;
    f64 m_grid = 512.0;

    // Raster acceleration structure.
    std::vector<u8> m_raster;        // 1 = inside mask, 0 = outside
    std::vector<f32> m_distance;     // metres to nearest boundary
    Vec2d m_rastOrigin{0, 0};
    f64 m_rastCell = 32.0;
    u32 m_rastW = 0, m_rastH = 0;
};

} // namespace mumbai::geojson
