// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/geojson.cpp
// =============================================================================
#include "pipeline/mgeo/geojson.h"

namespace mumbai::geojson {

namespace {

geo2d::Ring readRing(const Json& coords, const TransverseMercator& proj) {
    geo2d::Ring ring;
    ring.reserve(coords.size());
    for (const Json& pt : coords.items()) {
        if (pt.size() < 2) continue;
        const f64 lon = pt[0].asNumber();
        const f64 lat = pt[1].asNumber();
        ring.push_back(proj.forward(lat, lon));
    }
    return ring;
}

geo2d::Polygon readPolygon(const Json& coords, const TransverseMercator& proj) {
    geo2d::Polygon poly;
    for (size_t i = 0; i < coords.size(); ++i) {
        geo2d::Ring ring = readRing(coords[i], proj);
        if (ring.size() < 3) continue;
        if (i == 0) poly.outer = std::move(ring);
        else poly.holes.push_back(std::move(ring));
    }
    if (!poly.outer.empty()) {
        // GeoJSON convention is counter-clockwise outer / clockwise holes;
        // enforce it here so every downstream consumer can assume it.
        geo2d::ensureCounterClockwise(poly.outer);
    }
    return poly;
}

void readProperties(const Json& props, Feature& f) {
    if (!props.isObject()) return;
    for (const auto& [k, v] : props.fields()) {
        if (v.isNull()) continue;
        if (v.isString()) f.properties[k] = v.asString();
        else if (v.isNumber()) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.10g", v.asNumber());
            f.properties[k] = buf;
        } else if (v.isBool()) {
            f.properties[k] = v.asBool() ? "true" : "false";
        }
    }
    auto it = f.properties.find("name");
    if (it != f.properties.end()) f.name = it->second;
}

void readGeometry(const Json& geom, Feature& f, const TransverseMercator& proj, Collection& out) {
    const std::string type = geom["type"].asStringOr("");
    if (type == "Polygon") {
        f.polygon = readPolygon(geom["coordinates"], proj);
        out.geometryType = "Polygon";
    } else if (type == "MultiPolygon") {
        // One feature may hold several disjoint parts (BMC wards routinely do,
        // because of reclaimed islands and creek-side parcels).  The first part
        // becomes this Feature's polygon; the rest are emitted as siblings that
        // inherit the same properties, which keeps every downstream stage
        // working in terms of simple polygons.
        const Json& coords = geom["coordinates"];
        for (size_t i = 0; i < coords.size(); ++i) {
            geo2d::Polygon poly = readPolygon(coords[i], proj);
            if (poly.outer.size() < 3) continue;
            if (i == 0) {
                f.polygon = std::move(poly);
            } else {
                Feature extra;
                extra.polygon = std::move(poly);
                extra.properties = f.properties;
                extra.name = f.name;
                out.features.push_back(std::move(extra));
            }
        }
        out.geometryType = "MultiPolygon";
    } else if (type == "LineString") {
        f.lines.push_back(readRing(geom["coordinates"], proj));
        out.geometryType = "LineString";
    } else if (type == "MultiLineString") {
        for (const Json& line : geom["coordinates"].items())
            f.lines.push_back(readRing(line, proj));
        out.geometryType = "MultiLineString";
    } else if (type == "Point") {
        const Json& c = geom["coordinates"];
        if (c.size() >= 2) f.points.push_back(proj.forward(c[1].asNumber(), c[0].asNumber()));
        out.geometryType = "Point";
    }
}

} // namespace

bool load(const std::string& path, Collection& out, const TransverseMercator& proj,
          std::string* error) {
    const std::string text = readFileToString(path);
    if (text.empty()) {
        if (error) *error = "cannot read " + path;
        return false;
    }
    Json root;
    std::string parseError;
    if (!Json::parse(text, root, &parseError)) {
        if (error) *error = path + ": " + parseError;
        return false;
    }

    const Json& features = root["features"];
    if (!features.isArray()) {
        if (error) *error = path + ": no features array";
        return false;
    }
    out.features.reserve(features.size());
    for (const Json& jf : features.items()) {
        Feature f;
        readProperties(jf["properties"], f);
        const Json& geom = jf["geometry"];
        if (geom.isNull()) continue;
        readGeometry(geom, f, proj, out);
        if (f.polygon.outer.size() >= 3 || !f.lines.empty() || !f.points.empty())
            out.features.push_back(std::move(f));
    }
    MLOG_INFO("GeoJSON %s: %zu features (%s)", path.c_str(), out.features.size(),
              out.geometryType.c_str());
    return true;
}

// -----------------------------------------------------------------------------
//  MultiMask
// -----------------------------------------------------------------------------
void MultiMask::build(std::vector<geo2d::Polygon> polys, f64 gridSize) {
    m_grid = gridSize;
    m_polys = std::move(polys);
    m_bounds.clear();
    m_bounds.reserve(m_polys.size());
    for (const geo2d::Polygon& p : m_polys) {
        geo2d::Aabb2d b = geo2d::bounds(p);
        // Degenerate parts are dropped rather than indexed: an empty bbox would
        // poison the grid.
        if (!b.valid() || b.size().lengthSq() < 1e-9) { b.grow(1.0); }
        m_bounds.push_back(b);
    }
    m_index.build(m_bounds, m_grid);
}

// -----------------------------------------------------------------------------
//  Rasterisation + distance transform.
//  Scanline fill of each polygon into a u8 grid, followed by a two-pass chamfer
//  distance transform (the classic 3x3 approximation with 1 / sqrt(2) weights).
//  Accuracy is ~1 cell (32 m), which is far finer than the terrain grid it feeds.
// -----------------------------------------------------------------------------
void MultiMask::rasterize(const geo2d::Aabb2d& extent, f64 cellSize) {
    m_rastCell = std::max(1.0, cellSize);
    m_rastOrigin = extent.min - Vec2d{m_rastCell, m_rastCell};   // 1-cell border
    m_rastW = static_cast<u32>(std::ceil(extent.size().x / m_rastCell)) + 2;
    m_rastH = static_cast<u32>(std::ceil(extent.size().y / m_rastCell)) + 2;
    m_raster.assign(static_cast<size_t>(m_rastW) * m_rastH, 0);

    std::vector<f64> crossings;
    for (const geo2d::Polygon& poly : m_polys) {
        // Every ring contributes to the even-odd fill of this polygon.
        std::vector<const geo2d::Ring*> rings;
        rings.push_back(&poly.outer);
        for (const geo2d::Ring& h : poly.holes) rings.push_back(&h);

        geo2d::Aabb2d pb = geo2d::bounds(poly);
        if (!pb.valid()) continue;
        const i64 j0 = clamp<i64>(static_cast<i64>(std::floor((pb.min.y - m_rastOrigin.y) / m_rastCell)), 0,
                                  static_cast<i64>(m_rastH) - 1);
        const i64 j1 = clamp<i64>(static_cast<i64>(std::ceil((pb.max.y - m_rastOrigin.y) / m_rastCell)), 0,
                                  static_cast<i64>(m_rastH) - 1);
        for (i64 j = j0; j <= j1; ++j) {
            const f64 y = m_rastOrigin.y + (j + 0.5) * m_rastCell;
            crossings.clear();
            for (const geo2d::Ring* ring : rings) {
                const geo2d::Ring& r = *ring;
                for (size_t i = 0, k = r.size() - 1; i < r.size(); k = i++) {
                    const Vec2d& a = r[k];
                    const Vec2d& b = r[i];
                    if ((a.y > y) == (b.y > y)) continue;
                    const f64 t = (y - a.y) / (b.y - a.y);
                    crossings.push_back(a.x + t * (b.x - a.x));
                }
            }
            if (crossings.size() < 2) continue;
            std::sort(crossings.begin(), crossings.end());
            for (size_t c = 0; c + 1 < crossings.size(); c += 2) {
                const i64 i0 = clamp<i64>(static_cast<i64>(std::ceil((crossings[c] - m_rastOrigin.x) / m_rastCell)),
                                          0, static_cast<i64>(m_rastW) - 1);
                const i64 i1 = clamp<i64>(static_cast<i64>(std::floor((crossings[c + 1] - m_rastOrigin.x) / m_rastCell)),
                                          0, static_cast<i64>(m_rastW) - 1);
                for (i64 i = i0; i <= i1; ++i)
                    m_raster[static_cast<size_t>(j) * m_rastW + i] = 1;
            }
        }
    }

    // Distance transform: seed with 0 on boundary-adjacent cells, then two passes.
    const f64 diag = m_rastCell * 1.41421356237;
    m_distance.assign(m_raster.size(), 1e9f);
    auto idx = [&](i64 i, i64 j) { return static_cast<size_t>(j) * m_rastW + i; };
    for (u32 j = 0; j < m_rastH; ++j) {
        for (u32 i = 0; i < m_rastW; ++i) {
            const bool inside = m_raster[idx(i, j)] != 0;
            bool edge = false;
            for (int dj = -1; dj <= 1 && !edge; ++dj) {
                for (int di = -1; di <= 1; ++di) {
                    const i64 ni = static_cast<i64>(i) + di, nj = static_cast<i64>(j) + dj;
                    if (ni < 0 || nj < 0 || ni >= static_cast<i64>(m_rastW) || nj >= static_cast<i64>(m_rastH)) {
                        edge = true;
                        break;
                    }
                    if ((m_raster[idx(ni, nj)] != 0) != inside) { edge = true; break; }
                }
            }
            if (edge) m_distance[idx(i, j)] = 0.0f;
        }
    }
    for (u32 j = 0; j < m_rastH; ++j) {
        for (u32 i = 0; i < m_rastW; ++i) {
            f32 d = m_distance[idx(i, j)];
            if (i > 0) d = std::min(d, static_cast<f32>(m_distance[idx(i - 1, j)] + m_rastCell));
            if (j > 0) d = std::min(d, static_cast<f32>(m_distance[idx(i, j - 1)] + m_rastCell));
            if (i > 0 && j > 0)
                d = std::min(d, static_cast<f32>(m_distance[idx(i - 1, j - 1)] + diag));
            if (i + 1 < m_rastW && j > 0)
                d = std::min(d, static_cast<f32>(m_distance[idx(i + 1, j - 1)] + diag));
            m_distance[idx(i, j)] = d;
        }
    }
    for (i64 j = static_cast<i64>(m_rastH) - 1; j >= 0; --j) {
        for (i64 i = static_cast<i64>(m_rastW) - 1; i >= 0; --i) {
            f32 d = m_distance[idx(i, j)];
            if (i + 1 < static_cast<i64>(m_rastW))
                d = std::min(d, static_cast<f32>(m_distance[idx(i + 1, j)] + m_rastCell));
            if (j + 1 < static_cast<i64>(m_rastH))
                d = std::min(d, static_cast<f32>(m_distance[idx(i, j + 1)] + m_rastCell));
            if (i + 1 < static_cast<i64>(m_rastW) && j + 1 < static_cast<i64>(m_rastH))
                d = std::min(d, static_cast<f32>(m_distance[idx(i + 1, j + 1)] + diag));
            if (i > 0 && j + 1 < static_cast<i64>(m_rastH))
                d = std::min(d, static_cast<f32>(m_distance[idx(i - 1, j + 1)] + diag));
            m_distance[idx(i, j)] = d;
        }
    }
    MLOG_INFO("MultiMask raster: %u x %u cells at %.0f m, %zu polygons",
              m_rastW, m_rastH, m_rastCell, m_polys.size());
}

bool MultiMask::exactContains(const Vec2d& p) const {
    for (u32 id : candidates(p, 0.0)) {
        if (geo2d::polygonContains(m_polys[id], p)) return true;
    }
    return false;
}

std::vector<u32> MultiMask::candidates(const Vec2d& p, f64 pad) const {
    geo2d::Aabb2d q;
    q.expand(p);
    q.grow(pad);
    std::vector<u32> ids;
    m_index.query(q, ids);
    return ids;
}

bool MultiMask::contains(const Vec2d& p) const {
    // Fast path: the raster answers authoritatively when the query point is
    // more than one cell away from any boundary; near a boundary the exact
    // polygon test decides, so the result is never wrong, only sometimes slow.
    if (!m_raster.empty()) {
        const i64 i = static_cast<i64>(std::floor((p.x - m_rastOrigin.x) / m_rastCell));
        const i64 j = static_cast<i64>(std::floor((p.y - m_rastOrigin.y) / m_rastCell));
        if (i < 0 || j < 0 || i >= static_cast<i64>(m_rastW) || j >= static_cast<i64>(m_rastH))
            return false;
        const size_t id = static_cast<size_t>(j) * m_rastW + i;
        if (m_distance[id] > m_rastCell * 1.5) return m_raster[id] != 0;
    }
    return exactContains(p);
}

f64 MultiMask::distanceToBoundary(const Vec2d& p, f64 maxSearch) const {
    if (!m_distance.empty()) {
        const i64 i = static_cast<i64>(std::floor((p.x - m_rastOrigin.x) / m_rastCell));
        const i64 j = static_cast<i64>(std::floor((p.y - m_rastOrigin.y) / m_rastCell));
        if (i >= 0 && j >= 0 && i < static_cast<i64>(m_rastW) && j < static_cast<i64>(m_rastH))
            return std::min<f64>(m_distance[static_cast<size_t>(j) * m_rastW + i], maxSearch);
    }
    f64 best = maxSearch;
    for (u32 id : candidates(p, maxSearch)) {
        const geo2d::Polygon& poly = m_polys[id];
        for (size_t i = 0, j = poly.outer.size() - 1; i < poly.outer.size(); j = i++) {
            const f64 d = geo2d::distancePointSegment(p, poly.outer[j], poly.outer[i]);
            if (d < best) best = d;
        }
        for (const geo2d::Ring& h : poly.holes) {
            for (size_t i = 0, j = h.size() - 1; i < h.size(); j = i++) {
                const f64 d = geo2d::distancePointSegment(p, h[j], h[i]);
                if (d < best) best = d;
            }
        }
    }
    return best;
}

bool MultiMask::anyWithin(const Vec2d& p, f64 radius) const {
    return distanceToBoundary(p, radius) <= radius;
}

} // namespace mumbai::geojson
