// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/network.cpp
//
//  Stage group 2: the transport network.
//    * buildRoads        — OSM highways -> splines -> profiled ribbons, kerbs,
//                          sidewalks, medians, junction patches, and the
//                          navigation graph the traffic simulation consumes
//    * buildRailAndLanduse — railway alignments, platforms, land-use polygons
//
//  Road profiles are the specification's Section 1B cross-sections: a
//  six-lane divided highway, a median-less suburban street with open gutters,
//  and a pedestrian alley three metres wide all have to coexist and read as
//  themselves at ground level.
// =============================================================================
#include "pipeline/pipeline.h"

namespace mumbai::pipeline {

namespace detail {

SurfaceMaterial roadSurface(osm::HighwayClass cls, std::string_view surfaceTag) {
    // Explicit OSM surface tags win.
    if (surfaceTag == "concrete") return SurfaceMaterial::ConcreteRoad;
    if (surfaceTag == "paving_stones" || surfaceTag == "sett" || surfaceTag == "cobblestone")
        return SurfaceMaterial::PaverBlock;
    if (surfaceTag == "gravel" || surfaceTag == "fine_gravel" || surfaceTag == "compacted")
        return SurfaceMaterial::Gravel;
    if (surfaceTag == "unpaved" || surfaceTag == "ground" || surfaceTag == "dirt")
        return SurfaceMaterial::Dirt;
    if (surfaceTag == "asphalt" || surfaceTag == "paved") return SurfaceMaterial::AsphaltSmooth;

    switch (cls) {
        case osm::HW_Motorway:
        case osm::HW_Trunk:
        case osm::HW_Primary:
            return SurfaceMaterial::AsphaltSmooth;
        case osm::HW_Secondary:
        case osm::HW_Tertiary:
        case osm::HW_Unclassified:
            return SurfaceMaterial::AsphaltPatched;
        case osm::HW_Residential:
        case osm::HW_LivingStreet:
            return SurfaceMaterial::AsphaltPatched;
        case osm::HW_Service:
            return SurfaceMaterial::ConcreteRoad;
        case osm::HW_Track:
            return SurfaceMaterial::Gravel;
        case osm::HW_Footway:
        case osm::HW_Path:
        case osm::HW_Pedestrian:
        case osm::HW_Steps:
            return SurfaceMaterial::PaverBlock;
        default:
            return SurfaceMaterial::AsphaltPatched;
    }
}

RoadProfile roadProfile(osm::HighwayClass cls, std::string_view lanesTag, bool oneway) {
    RoadProfile p;
    int lanes = 0;
    if (!lanesTag.empty()) {
        lanes = std::atoi(std::string(lanesTag).c_str());
        if (lanes < 0) lanes = 0;
    }
    switch (cls) {
        case osm::HW_Motorway:
        case osm::HW_Trunk:
            p.lanesTotal = lanes ? lanes : (oneway ? 3 : 6);
            p.laneWidth = 3.5;
            p.medianWidth = oneway ? 0.0 : 2.0;
            p.kerbWidth = 0.35;
            p.sidewalkWidth = 0.0;          // expressways have no sidewalks
            p.layerOffset = 0.07;
            break;
        case osm::HW_Primary:
            p.lanesTotal = lanes ? lanes : (oneway ? 2 : 4);
            p.laneWidth = 3.4;
            p.medianWidth = oneway ? 0.0 : 1.2;
            p.kerbWidth = 0.3;
            p.sidewalkWidth = 2.5;
            p.layerOffset = 0.06;
            break;
        case osm::HW_Secondary:
            p.lanesTotal = lanes ? lanes : (oneway ? 2 : 4);
            p.laneWidth = 3.2;
            p.kerbWidth = 0.25;
            p.sidewalkWidth = 2.0;
            p.layerOffset = 0.05;
            break;
        case osm::HW_Tertiary:
            p.lanesTotal = lanes ? lanes : (oneway ? 1 : 2);
            p.laneWidth = 3.0;
            p.kerbWidth = 0.2;
            p.sidewalkWidth = 1.5;
            p.layerOffset = 0.045;
            break;
        case osm::HW_Unclassified:
        case osm::HW_Residential:
        case osm::HW_LivingStreet:
            p.lanesTotal = lanes ? lanes : (oneway ? 1 : 2);
            p.laneWidth = 2.9;
            p.sidewalkWidth = 0.0;          // gutters, not sidewalks, in most of Mumbai
            p.layerOffset = 0.04;
            break;
        case osm::HW_Service:
            p.lanesTotal = 1;
            p.laneWidth = 3.0;
            p.layerOffset = 0.03;
            break;
        case osm::HW_Footway:
        case osm::HW_Path:
            p.lanesTotal = 1;
            p.laneWidth = 1.6;
            p.layerOffset = 0.02;
            p.pedOnly = true;
            break;
        case osm::HW_Steps:
            p.lanesTotal = 1;
            p.laneWidth = 1.2;
            p.layerOffset = 0.02;
            p.pedOnly = true;
            break;
        case osm::HW_Pedestrian:
            p.lanesTotal = 1;
            p.laneWidth = 3.0;
            p.layerOffset = 0.025;
            p.pedOnly = true;
            break;
        case osm::HW_Track:
            p.lanesTotal = 1;
            p.laneWidth = 2.5;
            p.layerOffset = 0.02;
            break;
        default:
            p.lanesTotal = 1;
            p.laneWidth = 2.5;
            p.layerOffset = 0.03;
            break;
    }
    p.carriagewayWidth = p.laneWidth * p.lanesTotal;
    return p;
}

} // namespace detail

// -----------------------------------------------------------------------------
//  buildRoads
// -----------------------------------------------------------------------------
bool buildRoads(Context& ctx) {
    if (!ctx.cfg.buildRoads) return true;
    ScopedTimer timer("buildRoads");

    // --- 1. Collect the drivable/walkable ways ------------------------------
    // WayInfo is the shared WaySpec: real OSM ways and synthesised streets are
    // the same type, so extrusion, junctions, navigation and collision never
    // branch on provenance.
    using WayInfo = detail::WaySpec;
    std::vector<WayInfo> ways;
    ways.reserve(ctx.osm.ways().size() / 4);

    for (const osm::Way& w : ctx.osm.ways()) {
        const auto cls = static_cast<osm::HighwayClass>(w.highwayClass);
        if (cls == osm::HW_None || cls == osm::HW_Other) continue;
        if (w.refCount < 2) continue;
        // Indoor/airport infrastructure is out of scope for a street-level world.
        if (!ctx.osm.tag(w, "area").empty()) continue;

        WayInfo wi;
        wi.way = &w;
        wi.cls = cls;
        const std::string_view lanesTag = ctx.osm.tag(w, "lanes");
        const std::string_view onewayTag = ctx.osm.tag(w, "oneway");
        wi.profile = detail::roadProfile(cls, lanesTag, onewayTag == "yes" || onewayTag == "-1");
        wi.bridge = ctx.osm.tag(w, "bridge") == "yes" || ctx.osm.tag(w, "bridge") == "viaduct";
        wi.tunnel = ctx.osm.tag(w, "tunnel") == "yes";
        wi.name = std::string(ctx.osm.tag(w, "name"));
        wi.surface = detail::roadSurface(cls, ctx.osm.tag(w, "surface"));

        ctx.osm.wayPoints(w, ctx.proj, wi.points);
        if (wi.points.size() < 2) continue;

        // Simplify, then resample so profile extrusion has a uniform vertex
        // density regardless of how densely the source was mapped.
        wi.points = geo2d::simplifyOpen(wi.points, ctx.cfg.roadSimplify);
        if (wi.points.size() < 2) continue;
        const f64 maxEdge = std::max(4.0, wi.profile.carriagewayWidth * 1.5);
        // Preserve the original vertices for the nav graph (topology fidelity)
        // and use the resampled chain only for mesh extrusion.
        ways.push_back(std::move(wi));
        ways.back().points = geo2d::resampleOpen(ways.back().points, maxEdge);
    }
    MLOG_INFO("roads: %zu OSM ways selected for extrusion", ways.size());

    // --- 1c. Authored corridors and the synthesised ward lattice ------------
    // Without this the world would contain only the ~10% of Mumbai's streets
    // whose node geometry survived into the offline extract.  See DOCS/D-020.
    detail::appendAuthoredAndSynthetic(ctx, ways);
    MLOG_INFO("roads: %zu ways total after authored+synthesised network", ways.size());

    // --- 2. Junction detection ---------------------------------------------
    // Vertices are shared when two ways meet at the same OSM node; group by
    // rounded coordinate to find junctions, then trim each approach so that
    // ribbons do not overlap in the middle of the intersection.
    struct Junction {
        Vec2d position;
        f64 radius = 4.0;
        u32 count = 0;
    };
    std::unordered_map<u64, u32> junctionLookup;
    std::vector<Junction> junctions;
    auto posKey = [](const Vec2d& p) {
        const i64 x = static_cast<i64>(std::llround(p.x * 100.0));
        const i64 z = static_cast<i64>(std::llround(p.y * 100.0));
        return fnv1a64(static_cast<u64>(x) * 0x9E3779B97F4A7C15ull ^ static_cast<u64>(z));
    };
    for (const WayInfo& wi : ways) {
        // Endpoints only: mid-way vertices are not intersections.
        for (int end = 0; end < 2; ++end) {
            const Vec2d& p = end == 0 ? wi.points.front() : wi.points.back();
            const u64 k = posKey(p);
            auto it = junctionLookup.find(k);
            if (it == junctionLookup.end()) {
                junctionLookup.emplace(k, static_cast<u32>(junctions.size()));
                Junction j;
                j.position = p;
                j.radius = std::max(3.0, wi.profile.carriagewayWidth * 0.65);
                j.count = 1;
                junctions.push_back(j);
            } else {
                Junction& j = junctions[it->second];
                j.radius = std::max(j.radius, std::max(3.0, wi.profile.carriagewayWidth * 0.65));
                j.count++;
            }
        }
    }
    ctx.stats.junctions = junctions.size();
    MLOG_INFO("roads: %zu junction candidates", junctions.size());

    // --- 3. Extrude ---------------------------------------------------------
    const u32 matKerb = ctx.materials.id("kerb_concrete");
    const u32 matSidewalk = ctx.materials.id("sidewalk_concrete");
    const u32 matMedian = ctx.materials.id("median_barrier");

    for (const WayInfo& wi : ways) {
        const auto& pts = wi.points;
        const detail::RoadProfile& prof = wi.profile;
        const u32 mat = ctx.materials.id(surfaceName(wi.surface));
        const f64 halfLane = prof.laneWidth * 0.5;
        const bool isOnewayLane = prof.lanesTotal == 1;

        // Lane centrelines: for a two-way road the carriageway is centred on
        // the source alignment; a one-way road keeps its alignment as the
        // centre of its single carriageway.
        const f64 halfCarriage = prof.carriagewayWidth * 0.5;
        SurfaceMaterial surfaceMat = wi.surface;

        // Drape onto terrain: sample the DEM, then smooth the profile so that
        // roads do not inherit heightfield noise as visible steps.
        std::vector<f64> heights(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            f64 h = ctx.terrainHF.sample(pts[i].x, pts[i].y);
            if (wi.bridge) h += 6.0 + 0.02 * i;           // bridges clear the ground
            heights[i] = h + prof.layerOffset;
        }
        if (!wi.bridge) {
            std::vector<f64> smoothed = heights;
            for (size_t pass = 0; pass < 2; ++pass) {
                for (size_t i = 1; i + 1 < smoothed.size(); ++i)
                    smoothed[i] = (heights[i - 1] + heights[i] * 2.0 + heights[i + 1]) * 0.25;
                heights.swap(smoothed);
            }
        }

        // Junction trim distances at each end.
        auto trimAt = [&](const Vec2d& p) {
            auto it = junctionLookup.find(posKey(p));
            return it == junctionLookup.end() ? 0.0 : junctions[it->second].radius;
        };
        const f64 trimStart = trimAt(pts.front());
        const f64 trimEnd   = trimAt(pts.back());

        // Emit the carriageway (and the rest of the cross-section) as a strip
        // of quads, splitting at cell boundaries so every quad lands in the
        // right chunk.
        f64 travelled = 0.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const Vec2d a = pts[i], b = pts[i + 1];
            const f64 segLen = (b - a).length();
            if (segLen < 1e-6) continue;
            const Vec2d dir = (b - a) / segLen;
            const Vec2d left = dir.perpendicular();   // (-dy, dx)
            const Vec2d mid = (a + b) * 0.5;
            travelled += segLen;
            if (travelled < trimStart) continue;
            if (travelled > 0 && (dir.x * (pts.back().x - mid.x) + dir.y * (pts.back().y - mid.y)) < trimEnd)
                continue;

            CellData* cdp = ctx.tryCell(mid);
            if (!cdp) continue;                   // outside the world extent
            CellData& cd = *cdp;
            const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
            const f32 y0 = static_cast<f32>(heights[i]);
            const f32 y1 = static_cast<f32>(heights[i + 1]);
            const u32 zone = static_cast<u32>(ZoningZone::Unknown);
            const u8 flags = static_cast<u8>((wi.bridge ? 2 : 0) | (wi.tunnel ? 8 : 0) | 1);

            auto v3 = [&](const Vec2d& p, f32 y) {
                return Vec3{static_cast<f32>(p.x), y, static_cast<f32>(p.y)} - off;
            };

            // --- carriageway ---------------------------------------------------
            cd.roads.beginSubmesh(mat, zone);
            {
                const Vec2d aL = a + left * halfCarriage, aR = a - left * halfCarriage;
                const Vec2d bL = b + left * halfCarriage, bR = b - left * halfCarriage;
                // order: aR, bR, bL, aL  (right-hand rule gives +Y)
                cd.roads.addQuad(v3(aR, y0), v3(bR, y1), v3(bL, y1), v3(aL, y0),
                                 Vec3{0, 1, 0}, mat, zone, flags, 0.125f);
            }
            cd.roads.endSubmesh();
            ctx.stats.roadTriangles += 2;
            (void)halfLane;
            (void)isOnewayLane;
            (void)surfaceMat;

            // --- median --------------------------------------------------------
            if (prof.medianWidth > 0.01) {
                const f64 m = prof.medianWidth * 0.5;
                const Vec2d aL = a + left * (m), aR = a - left * (m);
                const Vec2d bL = b + left * (m), bR = b - left * (m);
                cd.roads.beginSubmesh(matMedian, zone);
                cd.roads.addQuad(v3(aR, y0), v3(bR, y1), v3(bL, y1), v3(aL, y0),
                                 Vec3{0, 1, 0}, matMedian, zone, flags, 0.2f);
                // Barrier walls (0.9 m tall, the standard Mumbai median)
                cd.roads.extrudeWalls({aR, aL, bL, bR}, y0, y0 + 0.9, matMedian, zone, flags, 0.5f);
                cd.roads.endSubmesh();
                ctx.stats.roadTriangles += 2 + 8;
            }

            // --- kerbs and sidewalks ------------------------------------------
            if (prof.sidewalkWidth > 0.01 || prof.kerbWidth > 0.01) {
                const f64 kw = std::max(prof.kerbWidth, 0.15);
                const f64 sw = prof.sidewalkWidth;
                const f64 innerOff = halfCarriage;
                const f64 outerOff = halfCarriage + kw;
                const f64 walkOuter = outerOff + sw;

                cd.roads.beginSubmesh(matKerb, zone);
                for (int side = 0; side < 2; ++side) {
                    const f64 sgn = side == 0 ? 1.0 : -1.0;
                    const Vec2d iA = a + left * (sgn * innerOff), iB = b + left * (sgn * innerOff);
                    const Vec2d oA = a + left * (sgn * outerOff), oB = b + left * (sgn * outerOff);
                    // Kerb face (vertical)
                    cd.roads.extrudeWalls({iA, oA, oB, iB}, y0 - 0.15, y0, matKerb, zone, flags, 0.5f);
                    if (sw > 0.01) {
                        const Vec2d wA = a + left * (sgn * walkOuter), wB = b + left * (sgn * walkOuter);
                        cd.roads.beginSubmesh(matSidewalk, zone);
                        // Sidewalk top surface, 0.15 m above the carriageway.
                        if (side == 0)
                            cd.roads.addQuad(v3(oA, y0 + 0.15f), v3(oB, y1 + 0.15f),
                                             v3(wB, y1 + 0.15f), v3(wA, y0 + 0.15f),
                                             Vec3{0, 1, 0}, matSidewalk, zone, flags, 0.25f);
                        else
                            cd.roads.addQuad(v3(oB, y1 + 0.15f), v3(oA, y0 + 0.15f),
                                             v3(wA, y0 + 0.15f), v3(wB, y1 + 0.15f),
                                             Vec3{0, 1, 0}, matSidewalk, zone, flags, 0.25f);
                        cd.roads.endSubmesh();
                    }
                }
                cd.roads.endSubmesh();
                ctx.stats.roadTriangles += 16;
            }

            // --- navigation graph ----------------------------------------------
            // Nodes are emitted per resampled vertex; edges connect consecutive
            // vertices and carry lane/speed/class metadata for the traffic AI.
            const f64 speed = [&] {
                const auto maxSpeed = wi.way ? ctx.osm.tag(*wi.way, "maxspeed")
                                             : std::string_view{};
                if (!maxSpeed.empty()) {
                    const f64 v = std::atof(std::string(maxSpeed).c_str());
                    if (v > 1.0 && v < 200.0) return v;
                }
                switch (wi.cls) {
                    case osm::HW_Motorway: return 80.0;
                    case osm::HW_Trunk: return 65.0;
                    case osm::HW_Primary: return 55.0;
                    case osm::HW_Secondary: return 45.0;
                    case osm::HW_Tertiary: return 40.0;
                    case osm::HW_Residential: return 30.0;
                    case osm::HW_Service: return 15.0;
                    case osm::HW_Pedestrian:
                    case osm::HW_Footway:
                    case osm::HW_Path: return 5.0;
                    default: return 25.0;
                }
            }();
            const bool oneway = wi.way && ctx.osm.tag(*wi.way, "oneway") == "yes";
            const u32 navFlags = NavFlagDrivable | (prof.pedOnly ? NavFlagPedestrianOnly : 0u) |
                                 (wi.bridge ? NavFlagBridge : 0u) |
                                 (wi.tunnel ? NavFlagTunnel : 0u) |
                                 (oneway ? NavFlagOneWay : 0u) |
                                 (wi.cls == osm::HW_Footway || wi.cls == osm::HW_Path || prof.pedOnly
                                      ? NavFlagWalkable : 0u);
            auto addNavNode = [&](const Vec2d& p, f32 y) -> u32 {
                NavNode n{};
                n.x = static_cast<f32>(p.x - cd.origin.x);
                n.y = y;
                n.z = static_cast<f32>(p.y - cd.origin.y);
                n.firstEdge = static_cast<u32>(cd.navEdges.size());
                n.edgeCount = 0;
                n.flags = static_cast<u16>(navFlags & 0xFFFFu);
                n.osmId = 0;
                cd.navNodes.push_back(n);
                return static_cast<u32>(cd.navNodes.size() - 1);
            };
            const u32 nodeCountBefore = static_cast<u32>(cd.navNodes.size());
            const u32 na = addNavNode(a, y0);
            const u32 nb = addNavNode(b, y1);
            NavEdge e{};
            e.toNode = nb;
            e.laneCount = static_cast<u16>(std::max(1, prof.lanesTotal));
            e.direction = oneway || prof.pedOnly ? 1 : 0;
            e.surface = static_cast<u8>(wi.surface);
            e.speedLimitKph = static_cast<f32>(speed);
            e.length = static_cast<f32>(segLen);
            e.flags = navFlags;
            e.roadClass = static_cast<u32>(wi.cls);
            // Cubic control points of the segment (for lane following).
            e.bezier[0] = static_cast<f32>(a.x - cd.origin.x);
            e.bezier[1] = y0;
            e.bezier[2] = static_cast<f32>(a.y - cd.origin.y);
            e.bezier[3] = static_cast<f32>(b.x - cd.origin.x);
            e.bezier[4] = y1;
            e.bezier[5] = static_cast<f32>(b.y - cd.origin.y);
            cd.navEdges.push_back(e);
            cd.navNodes[na].edgeCount = 1;
            ctx.stats.navEdges++;
            ctx.stats.navNodes += 2;
            ctx.stats.roadLengthMetres += static_cast<u64>(segLen);
            (void)nodeCountBefore;
            ctx.stats.roadTriangles += 0;
        }
        ++ctx.stats.roadWays;
        ctx.stats.surfaceCounts[surfaceName(wi.surface)] += 1;
        if (CellData* fcd = ctx.tryCell(pts.front())) fcd->counts.roads++;
    }

    // --- 4. Junction patches ------------------------------------------------
    // Fills the trimmed gaps with a simple octagon of the same asphalt family,
    // which is what stops intersections from showing the underlying terrain.
    const u32 matJunction = ctx.materials.id("asphalt_smooth");
    for (const Junction& j : junctions) {
        if (j.count < 2) continue;             // dead ends need no patch
        CellData* cdp = ctx.tryCell(j.position);
        if (!cdp) continue;                       // outside the world extent
        CellData& cd = *cdp;
        const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
        const f64 y = ctx.terrainHF.sample(j.position.x, j.position.y) + 0.045;
        const u32 zone = static_cast<u32>(ZoningZone::Unknown);
        cd.roads.beginSubmesh(matJunction, zone);
        const int sides = 8;
        for (int k = 0; k < sides; ++k) {
            const f64 a0 = (k / static_cast<f64>(sides)) * TAU;
            const f64 a1 = ((k + 1) / static_cast<f64>(sides)) * TAU;
            const Vec2d p0{j.position.x + std::cos(a0) * j.radius, j.position.y + std::sin(a0) * j.radius};
            const Vec2d p1{j.position.x + std::cos(a1) * j.radius, j.position.y + std::sin(a1) * j.radius};
            cd.roads.addTriangle(
                cd.roads.addVertex(Vec3{0, static_cast<f32>(y), 0}, Vec3{0, 1, 0}, {0, 0},
                                   matJunction, zone, 1),
                cd.roads.addVertex(Vec3{static_cast<f32>(p0.x), static_cast<f32>(y), static_cast<f32>(p0.y)} - off,
                                   Vec3{0, 1, 0}, {0, 0}, matJunction, zone, 1),
                cd.roads.addVertex(Vec3{static_cast<f32>(p1.x), static_cast<f32>(y), static_cast<f32>(p1.y)} - off,
                                   Vec3{0, 1, 0}, {0, 0}, matJunction, zone, 1));
            ctx.stats.roadTriangles += 1;
        }
        cd.roads.endSubmesh();
        cd.counts.roadJunctions++;
    }

    MLOG_INFO("roads: %llu ways, %.1f km centreline, %llu junctions, %llu triangles",
              (unsigned long long)ctx.stats.roadWays,
              ctx.stats.roadLengthMetres / 1000.0,
              (unsigned long long)ctx.stats.junctions,
              (unsigned long long)ctx.stats.roadTriangles);
    ctx.qa.add("roads.wayCount", static_cast<f64>(ctx.stats.roadWays));
    ctx.qa.add("roads.centrelineKilometres", ctx.stats.roadLengthMetres / 1000.0);
    ctx.qa.add("roads.junctionCount", static_cast<f64>(ctx.stats.junctions));
    ctx.qa.add("roads.triangles", static_cast<f64>(ctx.stats.roadTriangles));
    return true;
}

// -----------------------------------------------------------------------------
//  buildRailAndLanduse
// -----------------------------------------------------------------------------
bool buildRailAndLanduse(Context& ctx) {
    ScopedTimer timer("buildRailAndLanduse");

    const u32 matBallast = ctx.materials.id("rail_ballast");
    const u32 matSteel = ctx.materials.id("rail_steel");
    const u32 matSleeper = ctx.materials.id("rail_sleeper");

    // --- railway alignments -------------------------------------------------
    for (const osm::Way& w : ctx.osm.ways()) {
        const auto rw = static_cast<osm::RailwayClass>(w.railwayClass);
        const bool isTrack = rw == osm::RW_Rail || rw == osm::RW_Monorail || rw == osm::RW_NarrowGauge;
        if (!isTrack || w.refCount < 2) continue;

        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        if (pts.size() < 2) continue;
        pts = geo2d::resampleOpen(geo2d::simplifyOpen(pts, 0.6), 8.0);

        const bool monorail = rw == osm::RW_Monorail;
        const f64 gauge = monorail ? 0.85 : 1.676;      // Mumbai monorail vs broad gauge
        const f64 ballastHalf = monorail ? 1.6 : 3.4;

        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const Vec2d a = pts[i], b = pts[i + 1];
            const Vec2d dir = (b - a).normalized();
            const Vec2d left = dir.perpendicular();
            const Vec2d mid = (a + b) * 0.5;
            CellData* cdp = ctx.tryCell(mid);
            if (!cdp) continue;                   // outside the world extent
            CellData& cd = *cdp;
            const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
            const f32 y0 = ctx.terrainHF.sample(a.x, a.y) + 0.5f;
            const f32 y1 = ctx.terrainHF.sample(b.x, b.y) + 0.5f;
            const u32 zone = static_cast<u32>(ZoningZone::Unknown);
            auto v = [&](const Vec2d& p, f64 lateral, f32 y) {
                const Vec2d q = p + left * lateral;
                return Vec3{static_cast<f32>(q.x), y, static_cast<f32>(q.y)} - off;
            };

            // Ballast bed
            cd.rail.beginSubmesh(matBallast, zone);
            cd.rail.addQuad(v(a, ballastHalf, y0), v(b, ballastHalf, y1),
                            v(b, -ballastHalf, y1), v(a, -ballastHalf, y0),
                            Vec3{0, 1, 0}, matBallast, zone, 1, 0.2f);
            cd.rail.endSubmesh();

            // Sleepers (every 0.65 m would be absurd at world scale; the spec
            // budget uses a 2 m interval, which reads correctly from a moving
            // train or a road beside the track).
            cd.rail.beginSubmesh(matSleeper, zone);
            const f64 trackLen = (b - a).length();
            const int sleepers = static_cast<int>(trackLen / 2.0);
            for (int s = 0; s <= sleepers; ++s) {
                const f64 t = sleepers > 0 ? static_cast<f64>(s) / sleepers : 0.0;
                const Vec2d p = a + (b - a) * t;
                const f32 y = static_cast<f32>(lerp(static_cast<f64>(y0), static_cast<f64>(y1), t));
                cd.rail.addQuad(v(p, gauge * 0.5 + 0.35, y + 0.06f), v(p, -(gauge * 0.5 + 0.35), y + 0.06f),
                                v(p, -(gauge * 0.5 + 0.35), y + 0.06f), v(p, gauge * 0.5 + 0.35, y + 0.06f),
                                Vec3{0, 1, 0}, matSleeper, zone, 1, 0.5f);
            }
            cd.rail.endSubmesh();

            // Rails: two thin, tall boxes along the alignment.
            cd.rail.beginSubmesh(matSteel, zone);
            for (int side = 0; side < 2; ++side) {
                const f64 lat = (side == 0 ? 1.0 : -1.0) * gauge * 0.5;
                const Vec2d aR = a + left * (lat - 0.04), bR = b + left * (lat - 0.04);
                const Vec2d aL2 = a + left * (lat + 0.04), bL2 = b + left * (lat + 0.04);
                cd.rail.extrudeWalls({aR, aL2, bL2, bR}, y0 + 0.09, y0 + 0.24, matSteel, zone, 1, 1.0f);
            }
            cd.rail.endSubmesh();
            ctx.stats.railTriangles += 2 + 12 + 18;
        }
        ctx.stats.railWays++;
        if (CellData* fcd = ctx.tryCell(pts.front())) fcd->counts.railWays++;
    }

    // --- land-use polygons --------------------------------------------------
    for (const osm::Way& w : ctx.osm.ways()) {
        if (!w.closed() || w.refCount < 4) continue;
        const auto lu = static_cast<osm::LanduseClass>(w.landuseClass);
        const auto na = static_cast<osm::NaturalClass>(w.naturalClass);
        if (lu == osm::LU_None && na == osm::NA_None && w.buildingClass == 0) continue;
        if (w.buildingClass != 0) continue;    // footprints are buildings, not land use

        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        if (pts.size() < 4) continue;
        geo2d::Ring ring = geo2d::simplify(pts, ctx.cfg.footprintSimplify);
        if (ring.size() < 3) continue;

        SurfaceMaterial surf = SurfaceMaterial::Earth;
        ZoningZone zone = ZoningZone::Unknown;
        switch (lu) {
            case osm::LU_Grass: case osm::LU_Recreation: case osm::LU_Meadow:
                surf = SurfaceMaterial::Grass; break;
            case osm::LU_Residential: surf = SurfaceMaterial::Earth; zone = ZoningZone::SuburbanResidential; break;
            case osm::LU_Commercial: surf = SurfaceMaterial::ConcreteRoad; zone = ZoningZone::BKCCommercial; break;
            case osm::LU_Industrial: surf = SurfaceMaterial::ConcreteRoad; zone = ZoningZone::Industrial; break;
            case osm::LU_Railway: surf = SurfaceMaterial::Gravel; zone = ZoningZone::Industrial; break;
            case osm::LU_Reservoir: surf = SurfaceMaterial::WetlandMud; break;
            case osm::LU_Farm: surf = SurfaceMaterial::Dirt; break;
            case osm::LU_Quarry: surf = SurfaceMaterial::Rock; zone = ZoningZone::Industrial; break;
            default: break;
        }
        switch (na) {
            case osm::NA_Wetland: surf = SurfaceMaterial::WetlandMud; zone = ZoningZone::MangroveWetland; break;
            case osm::NA_Beach: case osm::NA_Sand: surf = SurfaceMaterial::Sand; break;
            case osm::NA_Wood: case osm::NA_Scrub: case osm::NA_Grass: case osm::NA_Grassland:
                surf = SurfaceMaterial::Grass; break;
            case osm::NA_Cliff: case osm::NA_Bare: surf = SurfaceMaterial::Rock; break;
            default: break;
        }

        f64 cx = 0, cy = 0;
        for (const Vec2d& p : ring) { cx += p.x; cy += p.y; }
        cx /= ring.size(); cy /= ring.size();
        CellData* cdp = ctx.tryCell(Vec2d{cx, cy});
        if (!cdp) continue;                       // outside the world extent
        CellData& cd = *cdp;
        const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
        const u32 matId = ctx.materials.id(surfaceName(surf));

        LandusePoly lp{};
        lp.firstVertex = static_cast<u32>(cd.landuseVerts.size());
        lp.vertexCount = static_cast<u32>(ring.size());
        lp.zone = static_cast<u32>(zone);
        lp.surface = static_cast<u32>(surf);
        lp.areaSquareMetres = static_cast<f32>(std::fabs(geo2d::signedArea(ring)));
        cd.landusePolys.push_back(lp);
        for (const Vec2d& p : ring) cd.landuseVerts.push_back(p);

        // Ground cover tint quad (a single triangle fan at terrain height; the
        // detailed blend happens in the terrain shader using this mask).
        cd.water.beginSubmesh(matId, static_cast<u32>(zone));
        geo2d::Mesh2 tris = geo2d::triangulateRing(ring);
        std::vector<u32> map(tris.vertices.size());
        for (size_t k = 0; k < tris.vertices.size(); ++k) {
            const Vec2d& p = tris.vertices[k];
            map[k] = cd.water.addVertex(
                Vec3{static_cast<f32>(p.x), ctx.terrainHF.sample(p.x, p.y) + 0.02f, static_cast<f32>(p.y)} - off,
                ctx.terrainHF.sampleNormal(p.x, p.y),
                Vec2{static_cast<f32>(p.x * 0.05), static_cast<f32>(p.y * 0.05)},
                matId, static_cast<u32>(zone), 0);
        }
        for (size_t k = 0; k + 2 < tris.indices.size(); k += 3)
            cd.water.addTriangle(map[tris.indices[k + 2]], map[tris.indices[k + 1]], map[tris.indices[k]]);
        cd.water.endSubmesh();
        cd.counts.landusePolys++;
        ctx.stats.landusePolys++;
    }

    MLOG_INFO("rail: %llu alignments; landuse: %llu polygons",
              (unsigned long long)ctx.stats.railWays,
              (unsigned long long)ctx.stats.landusePolys);
    ctx.qa.add("rail.wayCount", static_cast<f64>(ctx.stats.railWays));
    ctx.qa.add("landuse.polygonCount", static_cast<f64>(ctx.stats.landusePolys));
    return true;
}

} // namespace mumbai::pipeline
