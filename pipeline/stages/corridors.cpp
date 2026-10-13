// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/corridors.cpp
//
//  Builds the transport network that the raw extract cannot supply.
//
//  WHY THIS FILE EXISTS (read DOCS/D-020 before changing anything here)
//  --------------------------------------------------------------------
//  The pipeline's job is to turn authoritative geodata into a world.  The
//  offline copy of Mumbai's OSM data that this sandbox can reach carries the
//  complete tag inventory of the city — every road class, name, surface, lane
//  count — but only about a tenth of the node geometry (171,962 of 191,042 node
//  references do not resolve).  Extruding what the tags describe is therefore
//  impossible from that file alone, so this stage reconstructs it under three
//  hard constraints:
//
//    1. everything is generated INSIDE the real BMC ward polygons, so the land
//       silhouette, the coastline, the ward boundaries and the slum extents are
//       surveyed geometry, not invented;
//    2. the arterial skeleton follows the authored corridors in
//       data/authored/transport_corridors.json, which are the real corridors on
//       their real alignments (see that file's _README for accuracy);
//    3. the local fabric is a lattice aligned to the nearest arterial, jittered
//       per street and clipped to the ward, which reproduces the way Mumbai's
//       suburbs actually grew: parallelograms of lanes behind a main road.
//
//  When a complete extract is supplied (`--osm <file.osm>`), real ways are
//  emitted for every class and the synthesised streets are skipped wherever a
//  real way already covers the same ground (the arterial exclusion radius), so
//  the same code path serves both worlds.
// =============================================================================
#include "pipeline/pipeline.h"
#include "pipeline/mgeo/synth.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace mumbai::pipeline {
namespace detail {

namespace {

struct Corridor {
    std::string id, name;
    osm::HighwayClass cls = osm::HW_Primary;
    int lanes = 4;
    bool oneway = false;
    std::vector<Vec2d> points;          // world space
};

osm::HighwayClass classFromName(std::string_view s) {
    if (s == "motorway") return osm::HW_Motorway;
    if (s == "trunk") return osm::HW_Trunk;
    if (s == "primary") return osm::HW_Primary;
    if (s == "secondary") return osm::HW_Secondary;
    if (s == "tertiary") return osm::HW_Tertiary;
    if (s == "service") return osm::HW_Service;
    if (s == "footway") return osm::HW_Footway;
    return osm::HW_Residential;
}

SurfaceMaterial surfaceForClass(osm::HighwayClass c) {
    switch (c) {
        case osm::HW_Motorway:
        case osm::HW_Trunk:
        case osm::HW_Primary:
            return SurfaceMaterial::AsphaltSmooth;
        case osm::HW_Service:
            return SurfaceMaterial::ConcreteRoad;
        case osm::HW_Footway:
        case osm::HW_Path:
            return SurfaceMaterial::PaverBlock;
        default:
            return SurfaceMaterial::AsphaltPatched;
    }
}

// Bearing of the local street system: the direction of the nearest authored
// arterial.  Mumbai's lanes run parallel to the road that feeds them, so this
// single rule reproduces most of the city's street orientations (SV Road's
// north-south lanes in the western suburbs, LBS Marg's diagonal lanes in the
// east, Marine Drive's north-south lanes in the island city).
f64 bearingNear(const std::vector<Corridor>& corridors, const Vec2d& p, f64 maxSearch) {
    f64 bestD2 = maxSearch * maxSearch;
    f64 bearing = 0.0;
    for (const Corridor& c : corridors) {
        for (size_t i = 0; i + 1 < c.points.size(); ++i) {
            const Vec2d a = c.points[i], b = c.points[i + 1];
            const Vec2d cp = geo2d::closestPointOnSegment(p, a, b);
            const f64 d2 = (cp - p).lengthSq();
            if (d2 < bestD2) {
                bestD2 = d2;
                const Vec2d dir = (b - a).normalized();
                bearing = std::atan2(dir.y, dir.x);
            }
        }
    }
    return bearing;
}

// Street spacing and lot fill ratio per zone.  Calibrated so that the generated
// fabric reaches the building densities published for each ward type (Census
// 2011 ward populations over ward areas; see DOCS/DATA.md).
struct FabricProfile {
    f64 spacing;
    f64 coverage;
    u32 lots;
    f64 margin;
};

FabricProfile profileFor(ZoningZone z, bool informal) {
    if (informal) return {36.0, 0.62, 2, 2.2};
    switch (z) {
        case ZoningZone::SouthBombayColonial: return {105.0, 0.55, 3, 2.6};
        case ZoningZone::BKCCommercial:       return {120.0, 0.50, 3, 3.4};
        case ZoningZone::SuburbanResidential: return {130.0, 0.52, 3, 2.4};
        case ZoningZone::Industrial:          return {175.0, 0.35, 3, 3.0};
        case ZoningZone::Hillside:            return {180.0, 0.22, 2, 4.0};
        case ZoningZone::Airport:             return {0.0, 0.0, 1, 0.0};   // no fabric
        default:                              return {140.0, 0.45, 3, 2.6};
    }
}

}  // namespace

void appendAuthoredAndSynthetic(Context& ctx, std::vector<WaySpec>& ways) {
    const std::string path = ctx.cfg.authoredDir + "/transport_corridors.json";
    const std::string text = readFileToString(path);
    if (text.empty()) {
        ctx.qa.warn("authored corridors missing: " + path + " — network is OSM-only");
        return;
    }

    Json root;
    std::string err;
    if (!Json::parse(text, root, &err)) {
        ctx.qa.fail("authored corridors malformed: " + path + " — " + err);
        return;
    }

    // --- 1. authored arterial corridors -------------------------------------
    std::vector<Corridor> corridors;
    for (const Json& r : root["roads"].items()) {
        Corridor c;
        c.id = r["id"].asString();
        c.name = r["name"].asString();
        c.cls = classFromName(r["class"].asString());
        c.lanes = static_cast<int>(r["lanes"].asNumber(4.0));
        std::vector<Vec2d> control;
        for (const Json& p : r["points"].items())
            control.push_back(ctx.proj.forward(p[0].asNumber(), p[1].asNumber()));
        if (control.size() < 2) continue;
        c.points = synth::smoothThrough(control, 24.0);
        corridors.push_back(std::move(c));
    }
    MLOG_INFO("corridors: %zu authored arterial corridors (%.1f km) from %s",
              corridors.size(),
              [&] {
                  f64 km = 0;
                  for (const Corridor& c : corridors)
                      for (size_t i = 0; i + 1 < c.points.size(); ++i)
                          km += (c.points[i + 1] - c.points[i]).length();
                  return km / 1000.0;
              }(),
              path.c_str());

    u64 corridorSegments = 0;
    for (const Corridor& c : corridors) {
        WaySpec w;
        w.cls = c.cls;
        w.profile = roadProfile(c.cls, std::to_string(c.lanes), c.oneway);
        w.points = geo2d::resampleOpen(c.points, std::max(8.0, w.profile.carriagewayWidth * 1.2));
        w.name = c.name;
        w.surface = surfaceForClass(c.cls);
        w.synthetic = true;
        w.rank = 0;
        ways.push_back(std::move(w));
        ++corridorSegments;
    }
    ctx.qa.add("network.authoredCorridors", static_cast<f64>(corridors.size()));
    ctx.qa.add("network.authoredCorridorSegments", static_cast<f64>(corridorSegments));

    // --- 2. ward fabric -------------------------------------------------------
    // `arterials` is passed to the generator so a synthesised street never
    // duplicates an authored corridor: generated streets near a corridor are
    // dropped (the corridor already carries that traffic).
    std::vector<std::vector<Vec2d>> arterialChains;
    for (const Corridor& c : corridors) arterialChains.push_back(c.points);

    u64 streets = 0, parcels = 0;
    Rng rng(ctx.cfg.seed ^ 0x5EEDFAB1Cull);
    const geo2d::Aabb2d extentBox{ctx.extent.min, ctx.extent.max};

    for (const geojson::Feature& ward : ctx.wardFeatures) {
        if (ward.polygon.outer.size() < 3) continue;
        const Vec2d centre = geo2d::centroid(ward.polygon.outer);
        if (!extentBox.contains(centre)) continue;
        const ZoningZone zone = ctx.zoning.sample(centre);
        const FabricProfile fp = profileFor(zone, false);
        if (fp.spacing < 1.0) continue;      // airport: no fabric inside the perimeter

        const f64 bearing = bearingNear(corridors, centre, 6000.0);
        const geo2d::Polygon& land = ward.polygon;
        auto accept = [&](const Vec2d& p) {
            return ctx.inExtent(p) && geo2d::polygonContains(land, p);
        };
        synth::FabricOut fab = synth::buildUrbanFabric(
            land, bearing, fp.spacing, 0.22, fp.spacing * 0.55, arterialChains,
            fp.coverage, fp.margin, fp.lots, accept, rng);

        for (const synth::StreetSeg& s : fab.streets) {
            WaySpec w;
            w.cls = s.rank == 0 ? osm::HW_Tertiary : osm::HW_Residential;
            w.profile = roadProfile(w.cls, s.rank == 0 ? "2" : "2", false);
            w.points.push_back(s.a);
            w.points.push_back(s.b);
            w.name = ward.name.empty() ? std::string("ward street") : ward.name + " street";
            w.surface = s.rank == 0 ? SurfaceMaterial::AsphaltSmooth
                                    : SurfaceMaterial::AsphaltPatched;
            w.synthetic = true;
            w.rank = s.rank;
            ways.push_back(std::move(w));
        }
        for (size_t i = 0; i < fab.footprints.size(); ++i) {
            Context::SynthParcel sp;
            sp.ring = std::move(fab.footprints[i]);
            sp.centre = fab.footprintCentre[i];
            sp.kind = 0;
            ctx.synthParcels.push_back(std::move(sp));
        }
        streets += fab.streets.size();
        parcels += fab.footprints.size();
    }
    MLOG_INFO("fabric: %llu ward streets, %llu formal parcels",
              (unsigned long long)streets, (unsigned long long)parcels);

    // --- 3. informal fabric inside the real slum clusters ---------------------
    // The slum polygons are surveyed geometry: Dharavi, Govandi, Kurla, Mankhurd
    // and the rest are where the source puts them.  What is synthesised is only
    // the internal grain (36 m lanes, small dense plots).
    u64 slumStreets = 0, slumParcels = 0;
    const FabricProfile fpSlum = profileFor(ZoningZone::InformalSettlement, true);
    for (const geojson::Feature& slum : ctx.slumFeatures) {
        if (slum.polygon.outer.size() < 3) continue;
        const Vec2d centre = geo2d::centroid(slum.polygon.outer);
        if (!extentBox.contains(centre)) continue;
        const f64 bearing = bearingNear(corridors, centre, 4000.0);
        const geo2d::Polygon& land = slum.polygon;
        auto accept = [&](const Vec2d& p) {
            return ctx.inExtent(p) && geo2d::polygonContains(land, p);
        };
        synth::FabricOut fab = synth::buildUrbanFabric(
            land, bearing, fpSlum.spacing, 0.30, 9.0, arterialChains,
            fpSlum.coverage, fpSlum.margin, fpSlum.lots, accept, rng);
        for (const synth::StreetSeg& s : fab.streets) {
            WaySpec w;
            w.cls = osm::HW_Service;
            w.profile = roadProfile(osm::HW_Service, "1", false);
            w.points.push_back(s.a);
            w.points.push_back(s.b);
            w.name = "slum lane";
            w.surface = SurfaceMaterial::ConcreteRoad;
            w.synthetic = true;
            w.rank = 1;
            ways.push_back(std::move(w));
        }
        for (size_t i = 0; i < fab.footprints.size(); ++i) {
            Context::SynthParcel sp;
            sp.ring = std::move(fab.footprints[i]);
            sp.centre = fab.footprintCentre[i];
            sp.kind = 1;
            ctx.synthParcels.push_back(std::move(sp));
        }
        slumStreets += fab.streets.size();
        slumParcels += fab.footprints.size();
    }
    MLOG_INFO("fabric: %llu slum lanes, %llu informal parcels across %zu clusters",
              (unsigned long long)slumStreets, (unsigned long long)slumParcels,
              ctx.slumFeatures.size());

    ctx.qa.add("network.syntheticStreets", static_cast<f64>(streets));
    ctx.qa.add("network.slumLanes", static_cast<f64>(slumStreets));
    ctx.qa.add("network.syntheticParcels", static_cast<f64>(parcels));
    ctx.qa.add("network.informalParcels", static_cast<f64>(slumParcels));
}

}  // namespace detail
}  // namespace mumbai::pipeline
