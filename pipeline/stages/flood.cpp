// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/flood.cpp
//
//  The monsoon flood model, run over the built world (Section 3).
//
//  The specification's requirement is specific: Hindmata, King's Circle and the
//  Andheri subway must flood "as an emergent consequence of the DEM", not
//  because someone painted water there.  So this stage does not touch the
//  flood surface at all — it hands the model
//
//      * the terrain it just built,
//      * the street network it just generated (drains follow the roads),
//      * the building footprints it just placed (impervious cover),
//      * the open land it just classified (pervious),
//      * the real rainfall and tide of the design storms,
//
//  and reports wherever the water ends up.  If the DEM were wrong, the wrong
//  places would flood, and the counters would say so.
//
//  Outputs
//    * QA counters: flooded area by depth band, water removed by the drains and
//      the channels, sea water that came over the shore, and the peak depth and
//      time of peak at the named sites the specification calls out.
//    * `ctx.floodSevere` — a per-cell peak-depth raster (centimetres, capped at
//      255) on the same 32 m grid the viewer payload uses, exported so the
//      result is visible and checkable rather than described.
// =============================================================================
#include "pipeline/pipeline.h"
#include "pipeline/mgeo/hydro.h"

#include <algorithm>
#include <cmath>

namespace mumbai::pipeline {
namespace {

// The sites the specification names, plus three that sharpen the check: a coastal
// promenade that must stay dry, the business district on reclaimed ground, and
// the inland industrial belt.
constexpr struct { const char* name; f64 lat, lon; f64 tolerance; } kSites[] = {
    {"Hindmata junction",   19.0022, 72.8395, 0.0},
    {"King's Circle",       19.0300, 72.8650, 0.0},
    {"Andheri subway",      19.1190, 72.8470, 0.0},
    {"Milan subway",        19.0900, 72.8450, 0.0},
    {"Marine Drive",        18.9430, 72.8225, 0.0},
    {"BKC",                 19.0650, 72.8680, 0.0},
    {"Chembur",             19.0520, 72.9000, 0.0},
};

// --- coverage rasterisers ----------------------------------------------------
// The coverage layer is stamped at the hydro grid's own resolution (32 m), which
// is the scale at which "is this cell a street" is answerable: a lane is often
// narrower than a cell, so a segment marks every cell its line passes through,
// and a footprint marks every cell its ring covers.
struct CoverageRaster {
    u32 width = 0, height = 0;
    f64 originX = 0, originZ = 0, step = 32.0;
    std::vector<u8> road;        // 1 + road class (0 = none)
    std::vector<u8> building;
    std::vector<u8> green;

    size_t cellCount() const { return static_cast<size_t>(width) * height; }
    bool inside(i64 i, i64 j) const {
        return i >= 0 && j >= 0 && i < static_cast<i64>(width) && j < static_cast<i64>(height);
    }
    void stampSegment(const Vec2d& a, const Vec2d& b, std::vector<u8>& target, u8 value) {
        const f64 len = (b - a).length();
        const int steps = static_cast<int>(std::max(1.0, std::ceil(len / (step * 0.5))));
        for (int s = 0; s <= steps; ++s) {
            const f64 t = s / static_cast<f64>(steps);
            const Vec2d p = a + (b - a) * t;
            const i64 i = static_cast<i64>(std::floor((p.x - originX) / step));
            const i64 j = static_cast<i64>(std::floor((p.y - originZ) / step));
            if (!inside(i, j)) continue;
            u8& slot = target[static_cast<size_t>(j) * width + i];
            if (slot == 0 || value < slot) slot = value;      // keep the busier road
        }
    }
    void fillPolygon(const geo2d::Ring& ring, std::vector<u8>& target, u8 value) {
        if (ring.size() < 3) return;
        f64 minX = 1e30, minZ = 1e30, maxX = -1e30, maxZ = -1e30;
        for (const Vec2d& p : ring) {
            minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
            minZ = std::min(minZ, p.y); maxZ = std::max(maxZ, p.y);
        }
        const i64 i0 = std::max<i64>(0, static_cast<i64>(std::floor((minX - originX) / step)));
        const i64 j0 = std::max<i64>(0, static_cast<i64>(std::floor((minZ - originZ) / step)));
        const i64 i1 = std::min<i64>(width - 1, static_cast<i64>(std::ceil((maxX - originX) / step)));
        const i64 j1 = std::min<i64>(height - 1, static_cast<i64>(std::ceil((maxZ - originZ) / step)));
        for (i64 j = j0; j <= j1; ++j) {
            for (i64 i = i0; i <= i1; ++i) {
                const Vec2d c{originX + (i + 0.5) * step, originZ + (j + 0.5) * step};
                if (geo2d::pointInRing(ring, c)) target[static_cast<size_t>(j) * width + i] = value;
            }
        }
    }
};

// The road-class codes used by the nav graph, mapped to the drain capacities in
// DrainSpec.  Class 0 is motorway; the rest follow the taxonomy in geo.h.
u8 roadClassCode(u32 cls) {
    return static_cast<u8>(std::min<u32>(6, cls) + 1);
}

i64 cellIndex(const CoverageRaster& r, const Vec2d& p) {
    const i64 i = static_cast<i64>(std::floor((p.x - r.originX) / r.step));
    const i64 j = static_cast<i64>(std::floor((p.y - r.originZ) / r.step));
    if (!r.inside(i, j)) return -1;
    return j * static_cast<i64>(r.width) + i;
}

// Pervious cover: beaches, grass, mangroves.  Everything else is treated as
// sealed ground by default (the zone rule then raises it further).
bool isPerviousSurface(u32 surface) {
    switch (static_cast<SurfaceMaterial>(surface)) {
        case SurfaceMaterial::Grass:
        case SurfaceMaterial::WetlandMud:
        case SurfaceMaterial::Sand:
        case SurfaceMaterial::Earth:
        case SurfaceMaterial::Dirt:
            return true;
        default:
            return false;
    }
}

}  // namespace

// -----------------------------------------------------------------------------
//  buildFloodModel
// -----------------------------------------------------------------------------
// Runs after the buildings and props exist (it needs their coverage) and before
// the chunks are written (so the raster can be published).
bool buildFloodModel(Context& ctx) {
    if (!ctx.cfg.buildFloodModel) return true;

    ScopedTimer timer("flood model");

    // --- the hydro surface, on the viewer grid -------------------------------
    // 32 m keeps this stage's geometry identical to the published raster and
    // bounds the cost: 0.95 M cells, ~7 s per design storm on two cores.
    const f64 step = 32.0;
    const Vec2d size = ctx.extent.size();
    const u32 w = static_cast<u32>(std::ceil(size.x / step)) + 1;
    const u32 h = static_cast<u32>(std::ceil(size.y / step)) + 1;
    terrain::Heightfield hf;
    hf.resize(w, h, ctx.extent.min.x, ctx.extent.min.y, step, step);
    for (u32 j = 0; j < h; ++j)
        for (u32 i = 0; i < w; ++i)
            hf.at(i, j) = ctx.terrainHF.sample(ctx.extent.min.x + i * step,
                                               ctx.extent.min.y + j * step);

    hydro::SurfaceOptions sopt;
    // Condition the DEM before routing it (D-026).  Shallow closed depressions
    // are an artefact of sampling a rough surface at 32 m — each one holds a
    // pond for ever, and together they were putting 128 km² of the 149 km² of
    // ground above 10 m under water in the 2005 analogue, spread evenly over
    // the city instead of pooling in its low ground.  The four authored
    // depressions are exempt: those are the places the city actually floods.
    sopt.pitFillDepth = ctx.cfg.pitFillDepth;
    sopt.keepDepression = [&ctx](const Vec2d& p) {
        for (const terrain::ControlPoint& cp : ctx.controlPoints) {
            if (cp.flat && (p - cp.position).length() <= cp.radius) return true;
        }
        return false;
    };
    sopt.seaLevel = ctx.cfg.seaLevel;
    sopt.openBoundaries = true;
    const hydro::Surface surface = hydro::buildSurface(hf, sopt);
    {
        u64 landCells = 0;
        for (size_t k = 0; k < surface.cellCount(); ++k)
            if (!surface.isSea[k] && !surface.isBoundary[k]) ++landCells;
        ctx.stats.landAreaKm2 = landCells * surface.cellArea() / 1e6;
    }
    MLOG_INFO("flood: DEM conditioned — %llu cells raised (deepest %.2f m)",
              (unsigned long long)surface.pitsCarved, surface.pitFillMaxM);
    ctx.qa.add("flood.pitsCarved", static_cast<f64>(surface.pitsCarved));
    ctx.qa.add("flood.pitFillMaxM", surface.pitFillMaxM);
    MLOG_INFO("flood: surface %u x %u, %zu sea cells, %zu channel cells, %zu shore cells",
              surface.width, surface.height,
              static_cast<size_t>(std::count(surface.isSea.begin(), surface.isSea.end(), (u8)1)),
              static_cast<size_t>(std::count(surface.isChannel.begin(), surface.isChannel.end(), (u8)1)),
              static_cast<size_t>(std::count(surface.isShore.begin(), surface.isShore.end(), (u8)1)));

    // --- coverage: roads, buildings, green ----------------------------------
    CoverageRaster cover;
    cover.width = w; cover.height = h;
    cover.originX = ctx.extent.min.x; cover.originZ = ctx.extent.min.y; cover.step = step;
    cover.road.assign(cover.cellCount(), 0);
    cover.building.assign(cover.cellCount(), 0);
    cover.green.assign(cover.cellCount(), 0);

    u64 roadSegments = 0;
    for (const auto& [key, cd] : ctx.cells) {
        // Nav edges, in world space: nodes are cell-local.
        for (const NavNode& n : cd.navNodes) {
            const Vec2d a{cd.origin.x + n.x, cd.origin.y + n.z};
            for (u32 e = 0; e < n.edgeCount; ++e) {
                if (n.firstEdge + e >= cd.navEdges.size()) break;
                const NavEdge& edge = cd.navEdges[n.firstEdge + e];
                if (edge.toNode >= cd.navNodes.size()) continue;
                const NavNode& t = cd.navNodes[edge.toNode];
                const Vec2d b{cd.origin.x + t.x, cd.origin.y + t.z};
                cover.stampSegment(a, b, cover.road, roadClassCode(edge.roadClass));
                ++roadSegments;
            }
        }
        // Building footprints and landuse polygons.
        for (const LandusePoly& p : cd.landusePolys) {
            if (p.firstVertex + p.vertexCount > cd.landuseVerts.size() || p.vertexCount < 3) continue;
            geo2d::Ring ring;
            for (u32 k = 0; k < p.vertexCount; ++k) {
                const Vec2d& v = cd.landuseVerts[p.firstVertex + k];
                ring.push_back({cd.origin.x + v.x, cd.origin.y + v.y});
            }
            if (isPerviousSurface(p.surface)) cover.fillPolygon(ring, cover.green, 1);
        }
    }
    // Building cover comes from the oriented massing proxies, which is where
    // every building (surveyed and synthesised) ends up.  ctx.footprints holds
    // only the surveyed set, so using it leaves the synthesised 99 % of the city
    // "pervious" — and a city with no roofs to shed from floods differently.
    u64 proxiesStamped = 0;
    for (const auto& [key, cd] : ctx.cells) {
        for (const BuildingProxy& bp : cd.proxies) {
            const Vec2d c{cd.origin.x + bp.cx, cd.origin.y + bp.cz};
            const f64 hx = std::max<f64>(bp.hx, 1.0), hz = std::max<f64>(bp.hz, 1.0);
            const f64 reach = std::hypot(hx, hz);
            const i64 i0 = static_cast<i64>(std::floor((c.x - reach - cover.originX) / step));
            const i64 i1 = static_cast<i64>(std::ceil((c.x + reach - cover.originX) / step));
            const i64 j0 = static_cast<i64>(std::floor((c.y - reach - cover.originZ) / step));
            const i64 j1 = static_cast<i64>(std::ceil((c.y + reach - cover.originZ) / step));
            const f64 cs = std::cos(-bp.yaw), sn = std::sin(-bp.yaw);
            for (i64 j = j0; j <= j1; ++j) {
                for (i64 i = i0; i <= i1; ++i) {
                    if (!cover.inside(i, j)) continue;
                    const Vec2d p{cover.originX + (i + 0.5) * step, cover.originZ + (j + 0.5) * step};
                    const f64 dx = p.x - c.x, dz = p.y - c.y;
                    const f64 lx = dx * cs - dz * sn, lz = dx * sn + dz * cs;
                    if (std::fabs(lx) <= hx && std::fabs(lz) <= hz)
                        cover.building[static_cast<size_t>(j) * cover.width + i] = 1;
                }
            }
            ++proxiesStamped;
        }
    }
    MLOG_INFO("flood: %llu building proxies rasterised", (unsigned long long)proxiesStamped);
    u64 buildingCells = 0, roadCells = 0, greenCells = 0;
    for (size_t k = 0; k < cover.cellCount(); ++k) {
        buildingCells += cover.building[k] ? 1 : 0;
        roadCells += cover.road[k] ? 1 : 0;
        greenCells += cover.green[k] ? 1 : 0;
    }
    MLOG_INFO("flood: coverage %llu road cells (%llu segments), %llu building cells, %llu green cells",
              (unsigned long long)roadCells, (unsigned long long)roadSegments,
              (unsigned long long)buildingCells, (unsigned long long)greenCells);

    // --- land properties -----------------------------------------------------
    hydro::CoverageInput cov;
    cov.isRoad = [&](const Vec2d& p) {
        const i64 k = cellIndex(cover, p);
        return k >= 0 && cover.road[k] != 0;
    };
    cov.isBuilding = [&](const Vec2d& p) {
        const i64 k = cellIndex(cover, p);
        return k >= 0 && cover.building[k] != 0;
    };
    cov.isGreen = [&](const Vec2d& p) {
        const i64 k = cellIndex(cover, p);
        return k >= 0 && cover.green[k] != 0;
    };
    cov.roadClass = [&](const Vec2d& p) {
        const i64 k = cellIndex(cover, p);
        return k < 0 ? -1 : static_cast<i32>(cover.road[k]) - 1;
    };
    const hydro::LandSpec land = hydro::rasteriseLand(surface, cov, hydro::DrainSpec{});

    // --- run the design storms ----------------------------------------------
    const hydro::StormSpec storms[2] = {hydro::stormSpecModerate(), hydro::stormSpecSevere()};
    hydro::FloodResult results[2];
    for (int s = 0; s < 2; ++s) {
        ScopedTimer run(storms[s].name);
        results[s] = hydro::simulate(surface, land, storms[s], {});
        const hydro::FloodResult& r = results[s];
        const f64 balance = r.rainfallM3 + r.tideFloodM3
                          - r.drainedM3 - r.outfallM3 - r.storedM3 - r.filmLossM3;
        const std::string prefix = std::string("flood.") + storms[s].name + ".";
        ctx.qa.add(prefix + "rainfallM3", r.rainfallM3);
        ctx.qa.add(prefix + "drainedM3", r.drainedM3);
        ctx.qa.add(prefix + "channelledM3", r.channelledM3);
        ctx.qa.add(prefix + "outfallM3", r.outfallM3);
        ctx.qa.add(prefix + "tideFloodM3", r.tideFloodM3);
        ctx.qa.add(prefix + "peakStoredM3", r.peakStorageM3);
        ctx.qa.add(prefix + "areaOver50mmKm2", r.floodedAreaKm2);
        ctx.qa.add(prefix + "areaOver150mmKm2", r.floodedArea150mmKm2);
        ctx.qa.add(prefix + "areaOver500mmKm2", r.floodedArea500mmKm2);
        ctx.qa.add(prefix + "areaOver1000mmKm2", r.floodedArea1000mmKm2);
        ctx.qa.add(prefix + "filmLossM3", r.filmLossM3);
        ctx.qa.add(prefix + "massBalanceErrorM3", balance);

        // Who is under the water.  A depth band on its own says little: half a
        // metre on mangrove mud is a Tuesday, half a metre on a road is a
        // stranded city.  Splitting the same band by surface turns the flood map
        // into something a resident would recognise.
        f64 road50 = 0.0, bldg50 = 0.0, open50 = 0.0, roadAny = 0.0;
        for (size_t k = 0; k < surface.cellCount(); ++k) {
            if (surface.isSea[k] || surface.isBoundary[k]) continue;
            const f64 d = static_cast<f64>(r.maxDepth[k]);
            if (d <= 0.05) continue;
            const f64 km2 = surface.cellArea() / 1e6;
            const bool onRoad = cover.road[k] != 0;
            const bool onBuilding = cover.building[k] != 0;
            if (onRoad) roadAny += km2;
            if (d > 0.50) {
                if (onRoad) road50 += km2;
                else if (onBuilding) bldg50 += km2;
                else open50 += km2;
            }
        }
        // Elevation audit: which ground the water is standing on.  A flood model
        // is only credible if its water is where the city's low ground is, so
        // the wetted area is reported by the height of the ground beneath it.
        {
            static const f64 kEdges[6] = {-1e9, 1.0, 2.0, 3.0, 5.0, 10.0};
            f64 band[6] = {0, 0, 0, 0, 0, 0};
            f64 bandLand[6] = {0, 0, 0, 0, 0, 0};
            for (size_t k = 0; k < surface.cellCount(); ++k) {
                if (surface.isSea[k] || surface.isBoundary[k]) continue;
                const f64 g = surface.ground[k];
                int b = 0;
                for (int e = 1; e < 6; ++e) if (g >= kEdges[e]) b = e;
                bandLand[b] += surface.cellArea() / 1e6;
                if (r.maxDepth[k] > 0.15f) band[b] += surface.cellArea() / 1e6;
            }
            static const char* kNames[6] = {"below1m", "1to2m", "2to3m", "3to5m", "5to10m", "above10m"};
            for (int e = 0; e < 6; ++e) {
                ctx.qa.add(prefix + std::string("on.") + kNames[e] + "Km2", band[e]);
                ctx.qa.add(prefix + std::string("land.") + kNames[e] + "Km2", bandLand[e]);
            }
        }
        ctx.qa.add(prefix + "roadsWetKm2", roadAny);
        ctx.qa.add(prefix + "roadsOver500mmKm2", road50);
        ctx.qa.add(prefix + "buildingsOver500mmKm2", bldg50);
        ctx.qa.add(prefix + "openLandOver500mmKm2", open50);
        MLOG_INFO("flood: %s storm — rain %.4g m3, drains removed %.4g, channels %.4g, "
                  "sea took %.4g, sea pushed in %.4g, peak stored %.4g",
                  storms[s].name, r.rainfallM3, r.drainedM3, r.channelledM3, r.outfallM3,
                  r.tideFloodM3, r.peakStorageM3);
        MLOG_INFO("flood: %s storm — of %.1f km2 of land: >5 cm %.1f km2, >15 cm %.1f, "
                  ">50 cm %.1f, >1 m %.1f",
                  storms[s].name, ctx.stats.landAreaKm2, r.floodedAreaKm2,
                  r.floodedArea150mmKm2, r.floodedArea500mmKm2, r.floodedArea1000mmKm2);
        MLOG_INFO("flood: %s storm — under >50 cm: %.1f km2 of road, %.1f km2 over roofs, "
                  "%.1f km2 open land; %.1f km2 of road wet at any depth",
                  storms[s].name, road50, bldg50, open50, roadAny);
        MLOG_INFO("flood: %s storm — balance error %.4g m3 on %.4g m3 of rain "
                  "(film retired %.4g m3)",
                  storms[s].name, balance, r.rainfallM3, r.filmLossM3);

        // The balance is now exact to floating point, so the gate can be tight:
        // 1e-8 of the rainfall is rounding, not physics.
        if (std::fabs(balance) > 1e-8 * std::max(1.0, r.rainfallM3))
            ctx.qa.fail(std::string("flood.") + storms[s].name + ": mass balance off by " +
                        std::to_string(balance) + " m3");
        // ...and the numerical closure itself is audited: retiring drying films
        // must stay a rounding-scale term, never a drain of its own.
        if (r.filmLossM3 > 0.005 * std::max(1.0, r.rainfallM3))
            ctx.qa.fail(std::string("flood.") + storms[s].name + ": drying-film loss " +
                        std::to_string(r.filmLossM3) + " m3 exceeds 0.5 % of rainfall");
    }

    // --- named sites ---------------------------------------------------------
    struct SiteRead { f64 peak; f64 timeHours; };
    std::vector<SiteRead> severeReads, moderateReads;
    for (const auto& site : kSites) {
        const Vec2d p = ctx.proj.forward(site.lat, site.lon);
        // Read the site as a person there would: the deepest water in the
        // junction and the cells immediately around it, not a single 32 m pixel
        // that the rain might have skipped over.
        const i64 k0 = surface.indexOf(p.x, p.y);
        SiteRead a{0, 0}, b{0, 0};
        if (k0 >= 0) {
            const i64 ci = k0 % static_cast<i64>(surface.width);
            const i64 cj = k0 / static_cast<i64>(surface.width);
            for (i64 dj = -1; dj <= 1; ++dj) {
                for (i64 di = -1; di <= 1; ++di) {
                    const i64 ni = ci + di, nj = cj + dj;
                    if (ni < 0 || nj < 0 || ni >= static_cast<i64>(surface.width) ||
                        nj >= static_cast<i64>(surface.height)) continue;
                    const size_t kk = static_cast<size_t>(nj * static_cast<i64>(surface.width) + ni);
                    if (surface.isSea[kk] || surface.isBoundary[kk]) continue;
                    if (results[1].maxDepth[kk] > a.peak)
                        a = {results[1].maxDepth[kk], results[1].timeOfPeakHours[kk]};
                    if (results[0].maxDepth[kk] > b.peak)
                        b = {results[0].maxDepth[kk], results[0].timeOfPeakHours[kk]};
                }
            }
        }
        severeReads.push_back(a);
        moderateReads.push_back(b);
        const std::string key = std::string("flood.site.") + site.name;
        ctx.qa.add(key + ".peakSevereMm", a.peak * 1000.0);
        ctx.qa.add(key + ".peakModerateMm", b.peak * 1000.0);
        ctx.qa.add(key + ".timeOfPeakHours", a.timeHours);
        MLOG_INFO("flood site: %-20s severe %7.1f mm (peak at %4.1f h)   moderate %7.1f mm",
                  site.name, a.peak * 1000.0, a.timeHours, b.peak * 1000.0);
    }

    // --- the emergent-consequence check -------------------------------------
    // The three sites the specification names must flood, and the promenade on
    // the open coast must not.  These are assertions about the DEM, so they are
    // reported as failures, not as notes: if the terrain changes and Hindmata
    // stops flooding, the build is wrong.
    const auto siteIndex = [](const char* name) {
        for (size_t k = 0; k < sizeof(kSites) / sizeof(kSites[0]); ++k)
            if (std::string(kSites[k].name) == name) return k;
        return sizeof(kSites) / sizeof(kSites[0]);
    };
    const struct { const char* name; f64 minMm; } required[] = {
        {"Hindmata junction", 50.0},
        {"King's Circle",     50.0},
        {"Andheri subway",    50.0},
    };
    for (const auto& req : required) {
        const size_t k = siteIndex(req.name);
        if (k >= severeReads.size()) continue;
        const f64 mm = severeReads[k].peak * 1000.0;
        if (mm < req.minMm)
            ctx.qa.fail(std::string("flood: ") + req.name + " does not flood in the severe storm (" +
                        std::to_string(mm) + " mm < " + std::to_string(req.minMm) + " mm)");
        else
            ctx.qa.add(std::string("flood.emergence.") + req.name, mm);
    }
    {
        const size_t k = siteIndex("Marine Drive");
        if (k < severeReads.size()) {
            const f64 mm = severeReads[k].peak * 1000.0;
            if (mm > 500.0)
                ctx.qa.warn("flood: Marine Drive is under " + std::to_string(mm) +
                            " mm in the severe storm — check the promenade elevation");
            ctx.qa.add("flood.site.Marine Drive.severeMm", mm);
        }
    }

    // --- publish the severe-storm raster ------------------------------------
    ctx.floodModerate.assign(surface.cellCount(), 0);
    for (size_t k = 0; k < surface.cellCount(); ++k) {
        if (surface.isSea[k]) continue;
        ctx.floodModerate[k] = static_cast<u8>(std::min<f64>(255.0, results[0].maxDepth[k] * 100.0));
    }
    ctx.floodSevere.assign(surface.cellCount(), 0);
    for (size_t k = 0; k < surface.cellCount(); ++k) {
        if (surface.isSea[k]) continue;               // open water is drawn as water
        const f64 cm = std::min<f64>(255.0, results[1].maxDepth[k] * 100.0);
        ctx.floodSevere[k] = static_cast<u8>(std::lround(cm));
    }
    ctx.floodWidth = w;
    ctx.floodHeight = h;
    ctx.floodStep = step;
    u64 wet = 0;
    for (u8 v : ctx.floodSevere) wet += v > 0 ? 1 : 0;
    ctx.qa.add("flood.rasterCells", static_cast<f64>(wet));
    MLOG_INFO("flood: published %u x %u severe-storm raster (%llu wet cells)",
              w, h, (unsigned long long)wet);
    return true;
}

} // namespace mumbai::pipeline
