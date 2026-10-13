// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/sources.cpp
//
//  Stage group 1: source ingest, terrain, zoning, water.
//    * loadSources   — OSM extract, BMC wards, slum clusters, control points
//    * buildTerrain  — synthesise the DEM, analyse drainage basins
//    * buildZoning   — per-cell zoning classification driven by real ward codes
//    * buildWater    — sea/creek surface, inland water bodies, river ribbons
// =============================================================================
#include "pipeline/pipeline.h"

namespace mumbai::pipeline {

// -----------------------------------------------------------------------------
//  BuildConfig
// -----------------------------------------------------------------------------
BuildConfig BuildConfig::fromJson(const Json& j) {
    BuildConfig c;
    if (j.isNull()) return c;
    c.rawDir      = j["rawDir"].asStringOr(c.rawDir);
    c.authoredDir = j["authoredDir"].asStringOr(c.authoredDir);
    c.outDir      = j["outDir"].asStringOr(c.outDir);
    if (j.has("terrain")) {
        const Json& t = j["terrain"];
        c.terrainResolution = t["resolutionMetres"].asNumber(c.terrainResolution);
        c.terrainDetail     = t["detailAmplitude"].asNumber(c.terrainDetail);
        c.seaLevel          = t["seaLevel"].asNumber(c.seaLevel);
    }
    if (j.has("zoning")) c.zoningResolution = j["zoning"]["resolutionMetres"].asNumber(c.zoningResolution);
    // Hydrology: DEM conditioning depth for the flood model (D-026).
    if (j.has("flood")) c.pitFillDepth = j["flood"]["pitFillDepth"].asNumber(c.pitFillDepth);
    if (j.has("seed")) c.seed = static_cast<u64>(j["seed"].asInt(static_cast<i64>(c.seed)));
    if (j.has("simplify")) {
        c.footprintSimplify = j["simplify"]["footprintMetres"].asNumber(c.footprintSimplify);
        c.roadSimplify      = j["simplify"]["roadMetres"].asNumber(c.roadSimplify);
    }
    if (j.has("minBuildingAreaMetres")) c.minBuildingArea = j["minBuildingAreaMetres"].asNumber(c.minBuildingArea);
    if (j.has("stages")) {
        const Json& s = j["stages"];
        c.buildTerrain   = s["terrain"].asBool(true);
        c.buildRoads     = s["roads"].asBool(true);
        c.buildBuildings = s["buildings"].asBool(true);
        c.buildProps     = s["props"].asBool(true);
        c.buildHero      = s["heroAssets"].asBool(true);
        c.verifyChunks   = s["verifyChunks"].asBool(true);
    }
    return c;
}

// -----------------------------------------------------------------------------
//  MaterialLibrary
// -----------------------------------------------------------------------------
namespace {
void fillDef(MaterialDef& d, const char* name, f32 r, f32 g, f32 b, f32 roughness,
             f32 metallic, f32 macroVariation, f32 wetness, u32 flags = 0) {
    std::snprintf(d.name, sizeof(d.name), "%s", name);
    d.baseColor[0] = r; d.baseColor[1] = g; d.baseColor[2] = b; d.baseColor[3] = 1.0f;
    d.roughness = roughness;
    d.metallic = metallic;
    d.specular = 0.5f;
    d.normalScale = 1.0f;
    d.macroVariation = macroVariation;
    d.wetnessResponse = wetness;
    d.textureSetIndex = 0.0f;
    d.flags = flags;
}
} // namespace

MaterialLibrary::MaterialLibrary() {
    m_defs.emplace_back();           // id 0 = unknown
    std::snprintf(m_defs[0].name, sizeof(m_defs[0].name), "unknown");
    auto addMaterial = [&](const char* name, MaterialDef d) {
        m_lookup[name] = static_cast<u32>(m_defs.size());
        m_defs.push_back(d);
    };
    MaterialDef d{};

    // --- road surfaces (Section 1B) ----------------------------------------
    fillDef(d, "asphalt_smooth",   0.085f, 0.088f, 0.092f, 0.62f, 0.0f, 0.55f, 0.95f); addMaterial("asphalt_smooth", d);
    fillDef(d, "asphalt_patched",  0.115f, 0.112f, 0.110f, 0.78f, 0.0f, 0.75f, 0.92f); addMaterial("asphalt_patched", d);
    fillDef(d, "asphalt_sealink",  0.072f, 0.075f, 0.080f, 0.55f, 0.0f, 0.35f, 0.95f); addMaterial("asphalt_sealink", d);
    fillDef(d, "concrete_road",    0.420f, 0.420f, 0.415f, 0.72f, 0.0f, 0.40f, 0.85f); addMaterial("concrete_road", d);
    fillDef(d, "paver_block",      0.360f, 0.340f, 0.320f, 0.80f, 0.0f, 0.60f, 0.80f); addMaterial("paver_block", d);
    fillDef(d, "gravel",           0.300f, 0.280f, 0.255f, 0.90f, 0.0f, 0.65f, 0.70f); addMaterial("gravel", d);
    fillDef(d, "dirt",             0.250f, 0.205f, 0.160f, 0.94f, 0.0f, 0.70f, 0.75f); addMaterial("dirt", d);
    fillDef(d, "rock",             0.230f, 0.225f, 0.215f, 0.82f, 0.0f, 0.55f, 0.55f); addMaterial("rock", d);
    fillDef(d, "earth",            0.235f, 0.190f, 0.145f, 0.92f, 0.0f, 0.60f, 0.80f); addMaterial("earth", d);
    fillDef(d, "sand",             0.640f, 0.585f, 0.470f, 0.85f, 0.0f, 0.45f, 0.70f); addMaterial("sand", d);
    fillDef(d, "wetland_mud",      0.120f, 0.135f, 0.090f, 0.88f, 0.0f, 0.55f, 0.90f); addMaterial("wetland_mud", d);
    fillDef(d, "reclamation_fill", 0.330f, 0.320f, 0.300f, 0.88f, 0.0f, 0.60f, 0.80f); addMaterial("reclamation_fill", d);
    fillDef(d, "grass",            0.180f, 0.255f, 0.115f, 0.90f, 0.0f, 0.55f, 0.60f); addMaterial("grass", d);

    // --- building materials (Section 2A) -----------------------------------
    fillDef(d, "plaster_painted",  0.720f, 0.660f, 0.560f, 0.86f, 0.0f, 0.70f, 0.75f); addMaterial("plaster_painted", d);
    fillDef(d, "stone_ashlar",     0.560f, 0.480f, 0.380f, 0.80f, 0.0f, 0.60f, 0.70f); addMaterial("stone_ashlar", d);
    fillDef(d, "stone_rubble",     0.400f, 0.355f, 0.310f, 0.88f, 0.0f, 0.65f, 0.72f); addMaterial("stone_rubble", d);
    fillDef(d, "curtain_wall_glass",0.180f, 0.240f, 0.260f, 0.08f, 0.35f, 0.25f, 0.35f); addMaterial("curtain_wall_glass", d);
    fillDef(d, "brick_exposed",    0.470f, 0.245f, 0.170f, 0.88f, 0.0f, 0.60f, 0.78f); addMaterial("brick_exposed", d);
    fillDef(d, "corrugated_metal", 0.330f, 0.330f, 0.345f, 0.55f, 0.55f, 0.50f, 0.85f); addMaterial("corrugated_metal", d);
    fillDef(d, "concrete_raw",     0.480f, 0.470f, 0.450f, 0.85f, 0.0f, 0.55f, 0.80f); addMaterial("concrete_raw", d);

    // --- shared architecture / props ---------------------------------------
    fillDef(d, "kerb_concrete",    0.520f, 0.515f, 0.500f, 0.85f, 0.0f, 0.40f, 0.80f); addMaterial("kerb_concrete", d);
    fillDef(d, "sidewalk_concrete",0.480f, 0.475f, 0.460f, 0.87f, 0.0f, 0.45f, 0.82f); addMaterial("sidewalk_concrete", d);
    fillDef(d, "median_barrier",   0.450f, 0.450f, 0.440f, 0.84f, 0.0f, 0.35f, 0.78f); addMaterial("median_barrier", d);
    fillDef(d, "water_surface",    0.050f, 0.110f, 0.130f, 0.05f, 0.0f, 0.20f, 0.20f); addMaterial("water_surface", d);
    fillDef(d, "inland_water",     0.075f, 0.120f, 0.100f, 0.12f, 0.0f, 0.35f, 0.25f); addMaterial("inland_water", d);
    fillDef(d, "rail_ballast",     0.230f, 0.220f, 0.210f, 0.92f, 0.0f, 0.55f, 0.65f); addMaterial("rail_ballast", d);
    fillDef(d, "rail_steel",       0.400f, 0.400f, 0.420f, 0.35f, 0.90f, 0.20f, 0.45f); addMaterial("rail_steel", d);
    fillDef(d, "rail_sleeper",     0.300f, 0.270f, 0.240f, 0.90f, 0.0f, 0.50f, 0.70f); addMaterial("rail_sleeper", d);
    fillDef(d, "water_tank_plastic",0.180f, 0.260f, 0.230f, 0.45f, 0.0f, 0.35f, 0.40f); addMaterial("water_tank_plastic", d);
    fillDef(d, "ac_unit",          0.620f, 0.620f, 0.610f, 0.60f, 0.30f, 0.30f, 0.55f); addMaterial("ac_unit", d);
    fillDef(d, "metal_railing",    0.220f, 0.220f, 0.230f, 0.45f, 0.85f, 0.25f, 0.50f); addMaterial("metal_railing", d);
    fillDef(d, "signage_painted",  0.700f, 0.600f, 0.250f, 0.80f, 0.0f, 0.70f, 0.70f); addMaterial("signage_painted", d);
    fillDef(d, "tree_foliage",     0.115f, 0.190f, 0.085f, 0.92f, 0.0f, 0.60f, 0.40f); addMaterial("tree_foliage", d);
    fillDef(d, "tree_bark",        0.180f, 0.140f, 0.105f, 0.95f, 0.0f, 0.55f, 0.55f); addMaterial("tree_bark", d);
    fillDef(d, "streetlamp_pole",  0.300f, 0.300f, 0.295f, 0.55f, 0.65f, 0.25f, 0.55f); addMaterial("streetlamp_pole", d);
    fillDef(d, "streetlamp_glow",  1.000f, 0.870f, 0.620f, 0.30f, 0.0f, 0.10f, 0.20f,
            1u /* emissive */); addMaterial("streetlamp_glow", d);
    fillDef(d, "hero_stone",       0.580f, 0.520f, 0.430f, 0.78f, 0.0f, 0.55f, 0.68f); addMaterial("hero_stone", d);
    fillDef(d, "hero_terracotta",  0.430f, 0.245f, 0.175f, 0.82f, 0.0f, 0.50f, 0.62f); addMaterial("hero_terracotta", d);
    fillDef(d, "hero_concrete",    0.520f, 0.515f, 0.505f, 0.82f, 0.0f, 0.45f, 0.70f); addMaterial("hero_concrete", d);
    fillDef(d, "sea_link_pylon",   0.720f, 0.700f, 0.660f, 0.70f, 0.0f, 0.35f, 0.60f); addMaterial("sea_link_pylon", d);
    fillDef(d, "sea_link_cable",   0.500f, 0.500f, 0.520f, 0.40f, 0.80f, 0.20f, 0.35f); addMaterial("sea_link_cable", d);
}

u32 MaterialLibrary::id(const std::string& name) const {
    auto it = m_lookup.find(name);
    return it == m_lookup.end() ? 0u : it->second;
}

u32 MaterialLibrary::add(const MaterialDef& def) {
    m_defs.push_back(def);
    return static_cast<u32>(m_defs.size() - 1);
}

std::vector<u8> MaterialLibrary::serialize() const {
    std::vector<u8> out(m_defs.size() * sizeof(MaterialDef));
    std::memcpy(out.data(), m_defs.data(), out.size());
    return out;
}

// -----------------------------------------------------------------------------
//  ZoningField
// -----------------------------------------------------------------------------
void ZoningField::init(const WorldExtent& extent, f64 resolution) {
    step = std::max(4.0, resolution);
    originX = extent.min.x;
    originZ = extent.min.y;
    width = static_cast<u32>(std::ceil(extent.size().x / step)) + 1;
    height = static_cast<u32>(std::ceil(extent.size().y / step)) + 1;
    zones.assign(static_cast<size_t>(width) * height, static_cast<u8>(ZoningZone::Unknown));
}

// -----------------------------------------------------------------------------
//  QA
// -----------------------------------------------------------------------------
void QaReport::add(const std::string& name, f64 value, u32 severity) {
    QaCounter c{};
    std::snprintf(c.name, sizeof(c.name), "%s", name.c_str());
    c.value = value;
    c.severity = severity;
    counters.push_back(c);
}
void QaReport::warn(const std::string& msg) { warnings.push_back(msg); MLOG_WARN("QA: %s", msg.c_str()); }
void QaReport::fail(const std::string& msg) { failures.push_back(msg); MLOG_ERR("QA FAILURE: %s", msg.c_str()); }
void QaReport::logSummary() const {
    MLOG_INFO("QA: %zu counters, %zu warnings, %zu failures",
              counters.size(), warnings.size(), failures.size());
    for (const auto& c : counters)
        MLOG_INFO("  %-40s %14.3f%s", c.name, c.value,
                  c.severity == 2 ? "  [FAIL]" : (c.severity == 1 ? "  [warn]" : ""));
}

// -----------------------------------------------------------------------------
//  Context helpers
// -----------------------------------------------------------------------------
CellData& Context::cellAt(i64 cx, i64 cz) {
    const CellKey k{cx, cz};
    auto it = cells.find(k);
    if (it != cells.end()) return it->second;
    CellData cd;
    cd.cellX = cx;
    cd.cellZ = cz;
    cd.origin = cellOrigin(cx, cz);
    cd.minHeight = 1e30f;
    cd.maxHeight = -1e30f;
    auto [ins, _] = cells.emplace(k, std::move(cd));
    return ins->second;
}

CellData& Context::cell(const Vec2d& worldPos) {
    return cellAt(static_cast<i64>(std::floor(worldPos.x / MACRO_CELL_SIZE)),
                  static_cast<i64>(std::floor(worldPos.y / MACRO_CELL_SIZE)));
}

// -----------------------------------------------------------------------------
//  loadSources
// -----------------------------------------------------------------------------
namespace {
std::string pj(const std::string& dir, const std::string& file) { return pathJoin(dir, file); }
} // namespace

bool loadSources(Context& ctx) {
    const BuildConfig& cfg = ctx.cfg;
    std::string err;

    // --- control points -----------------------------------------------------
    const std::string cpPath = pj(cfg.authoredDir, "terrain_control_points.json");
    const std::string cpText = readFileToString(cpPath);
    if (!cpText.empty()) {
        Json j;
        if (Json::parse(cpText, j, &err)) {
            for (const Json& p : j["points"].items()) {
                terrain::ControlPoint cp;
                const f64 lat = p["lat"].asNumber();
                const f64 lon = p["lon"].asNumber();
                cp.position = ctx.proj.forward(lat, lon);
                cp.height = p["height"].asNumber();
                cp.radius = p["radius"].asNumber(800.0);
                cp.hard = p["hard"].asBool(false);
                cp.flat = p["flat"].asBool(false);
                cp.name = p["name"].asStringOr("");
                cp.weight = cp.hard ? 4.0 : 1.0;
                ctx.controlPoints.push_back(cp);
            }
            MLOG_INFO("loaded %zu terrain control points from %s",
                      ctx.controlPoints.size(), cpPath.c_str());
        } else {
            ctx.qa.warn("control point parse error: " + err);
        }
    } else {
        ctx.qa.warn("no terrain control points found at " + cpPath);
    }

    // --- OSM ----------------------------------------------------------------
    const std::string osmPath = pj(cfg.rawDir, "mumbai_region.osm");
    if (!ctx.osm.load(osmPath, &err)) {
        MLOG_ERR("OSM load failed: %s", err.c_str());
        return false;
    }
    ctx.osm.classify();
    ctx.sourceHash = ctx.osm.sourceHash();

    // --- BMC wards: defines the world extent and the zoning base ------------
    const std::string wardPath = pj(cfg.rawDir, "bmc_wards.geojson");
    geojson::Collection wards;
    if (!geojson::load(wardPath, wards, ctx.proj, &err)) {
        MLOG_ERR("ward load failed: %s", err.c_str());
        return false;
    }
    std::vector<geo2d::Polygon> wardPolys;
    for (auto& f : wards.features) {
        if (f.polygon.outer.size() < 3) continue;
        wardPolys.push_back(f.polygon);
        ctx.wardFeatures.push_back(std::move(f));
    }
    ctx.cityMask.build(std::move(wardPolys), 512.0);

    // Extent = ward bounding box, snapped outward to whole macro-cells so that
    // every chunk is a full 1024 m tile.
    geo2d::Aabb2d wb;
    for (const auto& p : ctx.cityMask.polygons()) wb.expand(geo2d::bounds(p));
    {
        // Rasterise the land mask before it is queried a few million times by
        // the terrain synthesiser, the mesh emitter and the zoning raster.
        geo2d::Aabb2d rasterExtent = wb;
        rasterExtent.grow(1024.0);
        ctx.cityMask.rasterize(rasterExtent, 32.0);
    }
    const i64 cx0 = static_cast<i64>(std::floor(wb.min.x / MACRO_CELL_SIZE));
    const i64 cz0 = static_cast<i64>(std::floor(wb.min.y / MACRO_CELL_SIZE));
    const i64 cx1 = static_cast<i64>(std::floor(wb.max.x / MACRO_CELL_SIZE));
    const i64 cz1 = static_cast<i64>(std::floor(wb.max.y / MACRO_CELL_SIZE));
    ctx.originCell = {cx0, cz0};
    ctx.extent.min = {static_cast<f64>(cx0) * MACRO_CELL_SIZE, static_cast<f64>(cz0) * MACRO_CELL_SIZE};
    ctx.extent.max = {static_cast<f64>(cx1 + 1) * MACRO_CELL_SIZE, static_cast<f64>(cz1 + 1) * MACRO_CELL_SIZE};

    const f64 areaKm2 = ctx.extent.size().x * ctx.extent.size().y / 1e6;
    MLOG_INFO("world extent: %.0f m x %.0f m (%.1f km^2), cells %lld x %lld",
              ctx.extent.size().x, ctx.extent.size().y, areaKm2,
              (long long)(cx1 - cx0 + 1), (long long)(cz1 - cz0 + 1));
    ctx.qa.add("world.extentSquareKm", areaKm2);
    ctx.qa.add("world.macroCellCount", static_cast<f64>((cx1 - cx0 + 1) * (cz1 - cz0 + 1)));

    // --- slum clusters: the informal-settlement zoning override ------------
    const std::string slumPath = pj(cfg.rawDir, "slum_clusters.geojson");
    geojson::Collection slums;
    if (geojson::load(slumPath, slums, ctx.proj, &err)) {
        std::vector<geo2d::Polygon> polys;
        polys.reserve(slums.features.size());
        for (auto& f : slums.features) {
            if (f.polygon.outer.size() < 3) continue;
            polys.push_back(f.polygon);
            ctx.slumFeatures.push_back(std::move(f));
        }
        ctx.slumMask.build(std::move(polys), 256.0);
        // Slum clusters are 2542 small polygons queried once per zoning cell and
        // once per terrain vertex, so they get a raster too (8 m cells: these
        // boundaries matter at building scale).
        geo2d::Aabb2d slumExtent;
        for (const auto& p : ctx.slumMask.polygons()) slumExtent.expand(geo2d::bounds(p));
        slumExtent.grow(128.0);
        ctx.slumMask.rasterize(slumExtent, 8.0);
        ctx.qa.add("zoning.slumPolygons", static_cast<f64>(ctx.slumMask.polygonCount()));
    } else {
        ctx.qa.warn("slum cluster layer unavailable: " + err);
    }
    return true;
}

// -----------------------------------------------------------------------------
//  buildTerrain
// -----------------------------------------------------------------------------
bool buildTerrain(Context& ctx) {
    if (!ctx.cfg.buildTerrain) return true;

    const Vec2d size = ctx.extent.size();
    const f64 res = ctx.cfg.terrainResolution;
    const u32 w = static_cast<u32>(std::ceil(size.x / res)) + 1;
    const u32 h = static_cast<u32>(std::ceil(size.y / res)) + 1;
    ctx.terrainHF.resize(w, h, ctx.extent.min.x, ctx.extent.min.y, res, res);
    MLOG_INFO("terrain grid: %u x %u (%.1f m resolution, %zu samples)",
              w, h, res, static_cast<size_t>(w) * h);

    // The city mask is the land/sea authority.  It is queried through the
    // MultiMask (raster fast path + exact polygon test near boundaries) so the
    // 603 km^2 ward union — 25 polygons, 22,640 vertices — is honoured in full.
    terrain::LandTestFn onLand;
    terrain::ShoreDistanceFn shoreDistance;
    if (!ctx.cityMask.empty()) {
        geojson::MultiMask* mask = &ctx.cityMask;
        onLand = [mask](const Vec2d& p) { return mask->contains(p); };
        shoreDistance = [mask](const Vec2d& p, f64 maxSearch) {
            return mask->distanceToBoundary(p, maxSearch);
        };
    }

    {
        ScopedTimer timer("terrain synthesis");
        terrain::synthesizeHeightfield(ctx.terrainHF, ctx.controlPoints, onLand, shoreDistance,
                                       ctx.cfg.seaLevel, ctx.cfg.terrainDetail,
                                       ctx.cfg.seed ^ 0x7A11E7ull);
    }

    // Re-apply the true city mask: cells outside any ward are sea or creek.
    for (u32 j = 0; j < h; ++j) {
        for (u32 i = 0; i < w; ++i) {
            const Vec2d world = ctx.terrainHF.worldOf(i, j);
            if (!ctx.cityMask.contains(world)) {
                // Sea, creek or mangrove: below the water plane, deepening
                // offshore so the ocean shader can do depth-based colouring.
                const f64 offshore = ctx.cityMask.distanceToBoundary(world, 3000.0);
                ctx.terrainHF.at(i, j) = static_cast<f32>(
                    ctx.cfg.seaLevel - 1.5 - clamp(offshore * 0.02, 0.0, 12.0));
            } else if (ctx.terrainHF.at(i, j) < ctx.cfg.seaLevel - 0.75) {
                ctx.terrainHF.at(i, j) = static_cast<f32>(ctx.cfg.seaLevel - 0.75);
            }
        }
    }

    ctx.stats.terrainMinHeight = ctx.terrainHF.minHeight();
    ctx.stats.terrainMaxHeight = ctx.terrainHF.maxHeight();
    MLOG_INFO("terrain height range: %.2f m .. %.2f m",
              ctx.stats.terrainMinHeight, ctx.stats.terrainMaxHeight);
    ctx.qa.add("terrain.minHeightMetres", ctx.stats.terrainMinHeight);
    ctx.qa.add("terrain.maxHeightMetres", ctx.stats.terrainMaxHeight);
    ctx.qa.add("terrain.sampleCount", static_cast<f64>(static_cast<size_t>(w) * h));

    if (ctx.stats.terrainMaxHeight < 40.0)
        ctx.qa.warn("terrain maximum height below 40 m — hill structure may be missing");

    // Drainage basins: the flood model needs them (Section 3A).
    {
        ScopedTimer timer("drainage analysis");
        ctx.lowPoints = ctx.terrainHF.findLowPoints(
            static_cast<f32>(ctx.cfg.seaLevel + 4.5), 256);
    }
    MLOG_INFO("drainage: %zu low-lying basins above %.1f m^(2)",
              ctx.lowPoints.size(), 400.0);
    for (size_t i = 0; i < std::min<size_t>(ctx.lowPoints.size(), 8); ++i) {
        const auto& lp = ctx.lowPoints[i];
        f64 lat = 0, lon = 0;
        ctx.proj.inverse(lp.position.x, lp.position.y, lat, lon);
        MLOG_INFO("  basin %zu: h=%.2f m area=%.0f m^2 at %.5f,%.5f",
                  i, lp.height, lp.basinArea, lat, lon);
    }
    ctx.qa.add("terrain.basinCount", static_cast<f64>(ctx.lowPoints.size()));
    return true;
}

// -----------------------------------------------------------------------------
//  buildZoning
// -----------------------------------------------------------------------------
namespace detail {

// Official BMC ward codes as present in the DataMeet layer, mapped to the
// specification's zoning taxonomy (Section 2A).  The correspondence follows the
// ward geography: A-E are the southern colonial precincts, H/E contains the
// Bandra-Kurla Complex, and the rest of the city is the suburban belt.
ZoningZone zoneForWard(const std::string& wardCode, const Vec2d& world, const Context& ctx) {
    const Vec2d airport{0, 0};   // placeholder to keep the signature explicit
    (void)airport;
    if (wardCode == "A" || wardCode == "B" || wardCode == "C" ||
        wardCode == "D" || wardCode == "E")
        return ZoningZone::SouthBombayColonial;
    if (wardCode == "H/E")
        return ZoningZone::BKCCommercial;
    (void)world;
    (void)ctx;
    return ZoningZone::SuburbanResidential;
}

} // namespace detail

bool buildZoning(Context& ctx) {
    ctx.zoning.init(ctx.extent, ctx.cfg.zoningResolution);

    // Airport precincts: CSMIA (Sahar) and Juhu aerodrome, whose published
    // coordinates are stable public geography.
    const Vec2d csmia = ctx.proj.forward(19.0887, 72.8679);
    const Vec2d juhu  = ctx.proj.forward(19.0981, 72.8341);
    const f64 csmiaRadius = 2600.0, juhuRadius = 1100.0;

    // Find each ward's dominant code by testing candidate ward polygons.
    for (u32 j = 0; j < ctx.zoning.height; ++j) {
        for (u32 i = 0; i < ctx.zoning.width; ++i) {
            const Vec2d world{ctx.zoning.originX + (i + 0.5) * ctx.zoning.step,
                              ctx.zoning.originZ + (j + 0.5) * ctx.zoning.step};

            if (!ctx.cityMask.contains(world)) {
                // Outside the city: creek, bay or mangrove.
                ctx.zoning.set(i, j, ZoningZone::MangroveWetland);
                continue;
            }
            if (ctx.slumMask.contains(world)) {
                ctx.zoning.set(i, j, ZoningZone::InformalSettlement);
                continue;
            }
            if ((world - csmia).length() < csmiaRadius || (world - juhu).length() < juhuRadius) {
                ctx.zoning.set(i, j, ZoningZone::Airport);
                continue;
            }

            // Ward code lookup.
            std::string code;
            for (const auto& f : ctx.wardFeatures) {
                if (geo2d::polygonContains(f.polygon, world)) {
                    code = f.properties.count("name") ? f.properties.at("name") : std::string();
                    break;
                }
            }
            ZoningZone z = detail::zoneForWard(code, world, ctx);

            // Terrain-driven override: anything above 35 m reads as hillside
            // (Malabar, Pali, Gilbert, the parks), which changes the building
            // grammar to terraced low-rise.
            const f32 h = ctx.terrainHF.sample(world.x, world.y);
            if (h > 35.0f && z == ZoningZone::SuburbanResidential)
                z = ZoningZone::Hillside;

            ctx.zoning.set(i, j, z);
            ctx.stats.zoneCounts[zoningName(z)] += 1;
        }
    }

    MLOG_INFO("zoning raster: %u x %u at %.0f m", ctx.zoning.width, ctx.zoning.height, ctx.zoning.step);
    for (const auto& [name, count] : ctx.stats.zoneCounts)
        MLOG_INFO("  zone %-24s %8llu cells (%.1f km^2)", name.c_str(),
                  (unsigned long long)count, count * ctx.zoning.step * ctx.zoning.step / 1e6);
    return true;
}

// -----------------------------------------------------------------------------
//  buildWater
// -----------------------------------------------------------------------------
bool buildWater(Context& ctx) {
    const u32 matSea = ctx.materials.id("water_surface");
    const u32 matInland = ctx.materials.id("inland_water");
    const geo2d::Aabb2d wb = ctx.worldBounds();

    // --- 1. Sea / creek surface ---------------------------------------------
    // Grid quads over every part of the extent that the city mask does not
    // cover: the Arabian Sea to the west, Mahim Bay, Thane Creek and the
    // harbour to the east.  Resolution is deliberately coarse (128 m) because
    // the ocean shader supplies displacement detail; tessellating finer would
    // cost storage without changing the silhouette.
    {
        const f64 step = 128.0;
        const i64 nx = static_cast<i64>(std::ceil(wb.size().x / step));
        const i64 nz = static_cast<i64>(std::ceil(wb.size().y / step));
        u64 quads = 0;
        for (i64 j = 0; j < nz; ++j) {
            for (i64 i = 0; i < nx; ++i) {
                const Vec2d c{wb.min.x + (i + 0.5) * step, wb.min.y + (j + 0.5) * step};
                if (ctx.cityMask.contains(c)) continue;
                // Winding (x0z0, x0z1, x1z1, x1z0) yields a +Y geometric normal.
                const Vec2d p0{wb.min.x + i * step, wb.min.y + j * step};
                const Vec2d p1{p0.x, p0.y + step};
                const Vec2d p2{p0.x + step, p0.y + step};
                const Vec2d p3{p0.x + step, p0.y};
                CellData* cdp = ctx.tryCell(Vec2d{(p0.x + p2.x) * 0.5, (p0.y + p2.y) * 0.5});
                if (!cdp) continue;           // outside the world extent
                CellData& cd = *cdp;
                const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
                cd.water.addQuad(
                    Vec3{static_cast<f32>(p0.x), 0.0f, static_cast<f32>(p0.y)} - off,
                    Vec3{static_cast<f32>(p1.x), 0.0f, static_cast<f32>(p1.y)} - off,
                    Vec3{static_cast<f32>(p2.x), 0.0f, static_cast<f32>(p2.y)} - off,
                    Vec3{static_cast<f32>(p3.x), 0.0f, static_cast<f32>(p3.y)} - off,
                    Vec3{0, 1, 0}, matSea, static_cast<u32>(ZoningZone::MangroveWetland), 0,
                    0.004f /* ~250 m UV period, low-frequency swell */);
                ++quads;
            }
        }
        ctx.stats.waterTriangles += quads * 2;
        ctx.qa.add("water.seaQuads", static_cast<f64>(quads));
        MLOG_INFO("sea/creek surface: %llu quads at %.0f m", (unsigned long long)quads, step);
    }

    // --- 2. Inland water polygons (lakes, ponds, docks, reservoirs) ---------
    u64 inlandPolys = 0;
    for (const osm::Way& w : ctx.osm.ways()) {
        const bool isWater = w.naturalClass == osm::NA_Water || w.waterClass != 0 ||
                             w.landuseClass == osm::LU_Reservoir;
        if (!isWater || !w.closed() || w.refCount < 4) continue;
        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        if (pts.size() < 4) continue;
        geo2d::Ring ring = geo2d::simplify(pts, ctx.cfg.footprintSimplify);
        if (ring.size() < 3 || std::fabs(geo2d::signedArea(ring)) < 50.0) continue;

        f64 cx = 0, cy = 0;
        for (const Vec2d& p : ring) { cx += p.x; cy += p.y; }
        cx /= ring.size(); cy /= ring.size();
        CellData* cdp = ctx.tryCell(Vec2d{cx, cy});
        if (!cdp) continue;                   // outside the world extent
        CellData& cd = *cdp;
        const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};

        cd.water.beginSubmesh(matInland, static_cast<u32>(ZoningZone::Unknown));
        const f64 waterY = std::max<f64>(ctx.cfg.seaLevel + 0.05,
                                         ctx.terrainHF.sample(cx, cy) - 0.4);
        geo2d::Mesh2 tris = geo2d::triangulateRing(ring);
        std::vector<u32> map(tris.vertices.size());
        for (size_t k = 0; k < tris.vertices.size(); ++k) {
            map[k] = cd.water.addVertex(
                Vec3{static_cast<f32>(tris.vertices[k].x), static_cast<f32>(waterY),
                     static_cast<f32>(tris.vertices[k].y)} - off,
                Vec3{0, 1, 0},
                Vec2{static_cast<f32>(tris.vertices[k].x * 0.02), static_cast<f32>(tris.vertices[k].y * 0.02)},
                matInland, static_cast<u32>(ZoningZone::Unknown), 0);
        }
        // Reversed emission: see the winding note in mesh.cpp addRoofCap.
        for (size_t k = 0; k + 2 < tris.indices.size(); k += 3)
            cd.water.addTriangle(map[tris.indices[k + 2]], map[tris.indices[k + 1]], map[tris.indices[k]]);
        cd.water.endSubmesh();
        cd.counts.waterPolys++;
        ctx.stats.waterTriangles += tris.indices.size() / 3;
        ++inlandPolys;
    }
    MLOG_INFO("inland water bodies: %llu", (unsigned long long)inlandPolys);

    // --- 3. Rivers, streams, canals and drains as ribbons -------------------
    // The Mithi River is the important one: it defines the boundary between the
    // western and eastern suburbs and floods in monsoon, so its course must be
    // right.
    u64 ribbons = 0;
    for (const osm::Way& w : ctx.osm.ways()) {
        if (w.closed() || w.refCount < 2) continue;
        if (w.waterClass == osm::WT_None || w.waterClass == osm::WT_Riverbank) continue;
        f64 halfWidth = 1.5;
        switch (w.waterClass) {
            case osm::WT_River:   halfWidth = 12.0; break;
            case osm::WT_Canal:   halfWidth = 6.0;  break;
            case osm::WT_Stream:  halfWidth = 1.8;  break;
            case osm::WT_Drain:   halfWidth = 2.0;  break;
            case osm::WT_Dock:    halfWidth = 25.0; break;
            default: break;
        }
        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        if (pts.size() < 2) continue;
        pts = geo2d::resampleOpen(pts, 12.0);

        for (size_t k = 0; k + 1 < pts.size(); ++k) {
            const Vec2d a = pts[k], b = pts[k + 1];
            const Vec2d dir = (b - a).normalized();
            const Vec2d n = dir.perpendicular() * halfWidth;
            const Vec2d mid = (a + b) * 0.5;
            CellData* cdp = ctx.tryCell(mid);
            if (!cdp) continue;               // outside the world extent
            CellData& cd = *cdp;
            const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
            const f64 y = ctx.terrainHF.sample(mid.x, mid.y) - 0.35;
            const f64 yEnd = ctx.terrainHF.sample(b.x, b.y) - 0.35;
            cd.water.addQuad(
                Vec3{static_cast<f32>(a.x + n.x), static_cast<f32>(y), static_cast<f32>(a.y + n.y)} - off,
                Vec3{static_cast<f32>(b.x + n.x), static_cast<f32>(yEnd), static_cast<f32>(b.y + n.y)} - off,
                Vec3{static_cast<f32>(b.x - n.x), static_cast<f32>(yEnd), static_cast<f32>(b.y - n.y)} - off,
                Vec3{static_cast<f32>(a.x - n.x), static_cast<f32>(y), static_cast<f32>(a.y - n.y)} - off,
                Vec3{0, 1, 0}, matInland, static_cast<u32>(ZoningZone::Unknown), 0, 0.02f);
            ++ribbons;
        }
        ctx.stats.waterTriangles += (pts.size() - 1) * 2;
    }
    MLOG_INFO("waterway ribbons: %llu segments", (unsigned long long)ribbons);
    ctx.qa.add("water.inlandBodies", static_cast<f64>(inlandPolys));
    ctx.qa.add("water.ribbonSegments", static_cast<f64>(ribbons));
    return true;
}

} // namespace mumbai::pipeline
