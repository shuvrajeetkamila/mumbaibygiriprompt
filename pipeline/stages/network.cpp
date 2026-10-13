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

#include "engine/world/signal_warrant.h"

#include <unordered_set>

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

namespace {

// -----------------------------------------------------------------------------
//  The navigation graph
// -----------------------------------------------------------------------------
// A graph a vehicle can actually drive.  Until now the graph was a set of chains:
// every resampled vertex became two fresh nodes and one edge, so nothing ever
// met anything, every junction was an invisible wall, and no signal could be
// obeyed because there was no junction to signalise.  Here each distinct point on
// the network is ONE node (ways that share a coordinate share the node), each
// node owns a contiguous run of out-edges, and the junctions that have earned a
// signal get one, with a two-phase 90-second plan that the runtime's green /
// amber / red cycle completes.
// -----------------------------------------------------------------------------
//  Crossing census (diagnostic)
// -----------------------------------------------------------------------------
//  At-grade planarisation
// -----------------------------------------------------------------------------
// A city's junctions are where its roads *cross*, not only where the source data
// happens to share one coordinate.  Measured on this network before the pass
// below existed (run 44): 16,344 at-grade crossing pairs, of which 4,994 were
// already jointed by a shared vertex — so ~11,000 crossings carried no junction
// at all.  The authored corridors and the OSM arterial subset, the only roads
// with a class, crossed the ward lattice mid-span every ninety metres and met
// nothing; that is why the signal warrant found exactly one junction to signal
// in a 600 km² city, and why the traffic model had no turning movements.
//
// This pass inserts the crossing point into both roads' *navigation* chains — see
// WaySpec::navPoints — and leaves `points`, the mesh chain, untouched: a junction
// node is a topology fact, and a ribbon needs no extra cross-section for it.
// Crossings are skipped when either road is flagged bridge or tunnel (grade
// separated, and the source says which), and a road that already has a vertex
// within 1.5 m of the crossing keeps it (already jointed, or jointed inside the
// specification's 2 m horizontal tolerance).
void planarizeCrossings(Context& ctx, std::vector<detail::WaySpec>& ways) {
    constexpr f64 kCell = 24.0;
    constexpr f64 kJointed = 1.5;
    const i64 gx0 = static_cast<i64>(std::floor(ctx.extent.min.x / kCell));
    const i64 gz0 = static_cast<i64>(std::floor(ctx.extent.min.y / kCell));
    const i64 gw = static_cast<i64>(std::ceil((ctx.extent.max.x - ctx.extent.min.x) / kCell)) + 2;
    const i64 gh = static_cast<i64>(std::ceil((ctx.extent.max.y - ctx.extent.min.y) / kCell)) + 2;
    std::vector<std::vector<u32>> buckets(static_cast<size_t>(gw * gh));

    struct Sub { u32 way; u32 seg; Vec2d a, b; };
    std::vector<Sub> subs;
    auto cellOf = [&](f64 x, i64 lo, i64 n) {
        return std::clamp<i64>(static_cast<i64>(std::floor(x / kCell)) - lo, 0, n - 1);
    };
    for (u32 w = 0; w < ways.size(); ++w) {
        const std::vector<Vec2d>& pts = ways[w].points;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const Vec2d a = pts[i], b = pts[i + 1];
            const f64 len = (b - a).length();
            const u32 steps = static_cast<u32>(std::max<f64>(1.0, std::ceil(len / (kCell * 0.5))));
            for (u32 k = 0; k < steps; ++k) {
                const Vec2d p0 = a + (b - a) * (static_cast<f64>(k) / steps);
                const Vec2d p1 = a + (b - a) * (static_cast<f64>(k + 1) / steps);
                const u32 idx = static_cast<u32>(subs.size());
                subs.push_back(Sub{w, static_cast<u32>(i), p0, p1});
                const i64 x0 = cellOf(std::min(p0.x, p1.x), gx0, gw), x1 = cellOf(std::max(p0.x, p1.x), gx0, gw);
                const i64 z0 = cellOf(std::min(p0.y, p1.y), gz0, gh), z1 = cellOf(std::max(p0.y, p1.y), gz0, gh);
                for (i64 z = z0; z <= z1; ++z)
                    for (i64 x = x0; x <= x1; ++x) buckets[static_cast<size_t>(z * gw + x)].push_back(idx);
            }
        }
    }

    struct Insert { u32 seg; f64 along; Vec2d p; };
    std::unordered_map<u32, std::vector<Insert>> byWay;
    std::unordered_set<u64> seen;
    u64 crossings = 0, gradeSeparated = 0, jointed = 0, inserted = 0;
    for (const std::vector<u32>& b : buckets) {
        for (size_t i = 0; i < b.size(); ++i) {
            for (size_t j = i + 1; j < b.size(); ++j) {
                const Sub& s0 = subs[b[i]];
                const Sub& s1 = subs[b[j]];
                if (s0.way == s1.way) continue;
                const Vec2d r = s0.b - s0.a, q = s1.b - s1.a;
                const f64 den = r.x * q.y - r.y * q.x;
                const Vec2d o = s1.a - s0.a;
                if (std::fabs(den) < 1e-12) continue;      // parallel
                const f64 t = (o.x * q.y - o.y * q.x) / den;
                const f64 u = (o.x * r.y - o.y * r.x) / den;
                if (t <= 0.0 || t >= 1.0 || u <= 0.0 || u >= 1.0) continue;
                const Vec2d p = s0.a + r * t;
                const u32 wa = std::min(s0.way, s1.way), wb = std::max(s0.way, s1.way);
                const i64 bx = cellOf(p.x, gx0, gw), bz = cellOf(p.y, gz0, gh);
                const u64 key = (static_cast<u64>(wa) << 47) ^ (static_cast<u64>(wb) << 30) ^
                                (static_cast<u64>(bx) << 15) ^ static_cast<u64>(bz);
                if (!seen.insert(key).second) continue;
                ++crossings;
                if (ways[wa].bridge || ways[wa].tunnel || ways[wb].bridge || ways[wb].tunnel) {
                    ++gradeSeparated;      // a flyover over a street is not a junction
                    continue;
                }
                const u32 pair[2] = {wa, wb};
                const u32 segOf[2] = {s0.way == wa ? s0.seg : s1.seg, s0.way == wb ? s0.seg : s1.seg};
                for (int side = 0; side < 2; ++side) {
                    const detail::WaySpec& w = ways[pair[side]];
                    bool has = false;
                    for (const Vec2d& v : w.points)
                        if ((v - p).lengthSq() < kJointed * kJointed) { has = true; break; }
                    if (has) { ++jointed; continue; }
                    const Vec2d& anchor = w.points[segOf[side]];
                    byWay[pair[side]].push_back(Insert{segOf[side], (p - anchor).length(), p});
                    ++inserted;
                }
            }
        }
    }

    for (auto& entry : byWay) {
        detail::WaySpec& w = ways[entry.first];
        std::vector<Insert>& list = entry.second;
        std::sort(list.begin(), list.end(), [](const Insert& a, const Insert& b) {
            return a.seg != b.seg ? a.seg < b.seg : a.along < b.along;
        });
        w.navPoints.reserve(w.points.size() + list.size());
        size_t k = 0;
        for (size_t i = 0; i + 1 < w.points.size(); ++i) {
            w.navPoints.push_back(w.points[i]);
            while (k < list.size() && list[k].seg == i) w.navPoints.push_back(list[k++].p);
        }
        w.navPoints.push_back(w.points.back());
    }

    MLOG_INFO("crossings: %llu at-grade pairs, %llu grade-separated, %llu already jointed, %llu junction vertices inserted into %llu roads",
              (unsigned long long)crossings, (unsigned long long)gradeSeparated,
              (unsigned long long)jointed, (unsigned long long)inserted,
              (unsigned long long)byWay.size());
    ctx.qa.add("roads.crossingsAtGrade", static_cast<f64>(crossings));
    ctx.qa.add("roads.crossingsGradeSeparated", static_cast<f64>(gradeSeparated));
    ctx.qa.add("roads.junctionVerticesInserted", static_cast<f64>(inserted));
    ctx.qa.add("roads.planarisedRoads", static_cast<f64>(byWay.size()));
}

void buildNavGraph(Context& ctx, const std::vector<detail::WaySpec>& ways) {
    std::unordered_map<CellData*, std::vector<const detail::WaySpec*>> byCell;
    for (const detail::WaySpec& wi : ways) {
        if (wi.points.size() < 2) continue;
        CellData* cd = ctx.tryCell(wi.points.front());
        if (!cd) continue;                      // outside the extent: not our world
        byCell[cd].push_back(&wi);
    }

    u64 totalNodes = 0, totalEdges = 0, totalSignals = 0;
    u64 junctions3 = 0, junctionsSignalled = 0, deadEnds = 0;
    u32 maxDegree = 0;
    // Only the branching count matters here (see the report at the end of this
    // function): an unclassified junction needs no signal, whatever its size.
    // The tally index is the warrant's own value, so the counts cannot drift
    // from the rule, and "no axis to phase the junction on" gets a slot of its
    // own past the warrant values rather than aliasing one of them.
    enum WarrantCause : int {
        WC_None = static_cast<int>(navsignal::Warrant::None),
        WC_TwoClassified = static_cast<int>(navsignal::Warrant::TwoClassified),
        WC_ClassifiedWideJunction = static_cast<int>(navsignal::Warrant::ClassifiedWideJunction),
        WC_ThreeCollectors = static_cast<int>(navsignal::Warrant::ThreeCollectors),
        WC_TwoWide = static_cast<int>(navsignal::Warrant::TwoWide),
        WC_NoAxis = static_cast<int>(navsignal::Warrant::Count),
    };
    u64 warrantCause[WC_NoAxis + 1] = {};

    for (auto& entry : byCell) {
        CellData& cd = *entry.first;
        const std::vector<const detail::WaySpec*>& list = entry.second;
        // --- 1. Nodes, deduplicated on position -----------------------------
        // 5 cm quantum, and an injective key, so two points are the same node
        // exactly when they are the same point to within a hand's width.  (The
        // specification's road tolerance is 2 m horizontally; merging at 5 cm
        // cannot move a centreline meaningfully and turns every shared OSM
        // coordinate into a real junction.)
        std::unordered_map<u64, u32> nodeAt;
        auto keyOf = [](const Vec2d& p) {
            const i64 x = static_cast<i64>(std::llround(p.x * 20.0));
            const i64 z = static_cast<i64>(std::llround(p.y * 20.0));
            return (static_cast<u64>(static_cast<u32>(x)) << 32) | static_cast<u32>(z);
        };
        auto nodeFor = [&](const Vec2d& p, f32 y, u16 flags) -> u32 {
            const u64 k = keyOf(p);
            auto it = nodeAt.find(k);
            if (it != nodeAt.end()) {
                NavNode& n = cd.navNodes[it->second];
                n.flags = static_cast<u16>(n.flags | flags);
                // Where a street and a flyover meet at the same plan position, the
                // deck belongs to the ground road: a flyover passes over, it does
                // not sit on the street.
                n.y = std::min(n.y, y);
                return it->second;
            }
            NavNode n{};
            n.x = static_cast<f32>(p.x - cd.origin.x);
            n.y = y;
            n.z = static_cast<f32>(p.y - cd.origin.y);
            n.firstEdge = 0;
            n.edgeCount = 0;
            n.flags = flags;
            n.osmId = 0;
            const u32 idx = static_cast<u32>(cd.navNodes.size());
            cd.navNodes.push_back(n);
            nodeAt.emplace(k, idx);
            return idx;
        };

        struct PendingEdge { u32 from; NavEdge e; };
        std::vector<PendingEdge> pend;

        for (const detail::WaySpec* wp : list) {
            const detail::WaySpec& wi = *wp;
            // The navigation chain, not the mesh chain: a junction exists where
            // the centrelines cross, whether or not the ribbon has a ring there.
            const std::vector<Vec2d>& pts = wi.navPoints.empty() ? wi.points : wi.navPoints;
            const auto maxSpeed = wi.way ? ctx.osm.tag(*wi.way, "maxspeed") : std::string_view{};
            f64 speed = 25.0;
            if (!maxSpeed.empty()) {
                const f64 v = std::atof(std::string(maxSpeed).c_str());
                if (v > 1.0 && v < 200.0) speed = v;
            } else {
                switch (wi.cls) {
                    case osm::HW_Motorway: speed = 80.0; break;
                    case osm::HW_Trunk: speed = 65.0; break;
                    case osm::HW_Primary: speed = 55.0; break;
                    case osm::HW_Secondary: speed = 45.0; break;
                    case osm::HW_Tertiary: speed = 40.0; break;
                    case osm::HW_Residential: speed = 30.0; break;
                    case osm::HW_Service: speed = 15.0; break;
                    case osm::HW_Pedestrian:
                    case osm::HW_Footway:
                    case osm::HW_Path: speed = 5.0; break;
                    default: speed = 25.0; break;
                }
            }
            const bool oneway = wi.way && ctx.osm.tag(*wi.way, "oneway") == "yes";
            const u32 navFlags = NavFlagDrivable | (wi.profile.pedOnly ? NavFlagPedestrianOnly : 0u) |
                                 (wi.bridge ? NavFlagBridge : 0u) | (wi.tunnel ? NavFlagTunnel : 0u) |
                                 (oneway ? NavFlagOneWay : 0u) |
                                 (wi.cls == osm::HW_Footway || wi.cls == osm::HW_Path || wi.profile.pedOnly
                                      ? NavFlagWalkable : 0u);

            auto heightAt = [&](size_t i) {
                f64 h = ctx.terrainHF.sample(pts[i].x, pts[i].y);
                if (wi.bridge) h += 6.0 + 0.02 * static_cast<f64>(i);
                return static_cast<f32>(h + wi.profile.layerOffset);
            };

            for (size_t i = 0; i + 1 < pts.size(); ++i) {
                const f64 segLen = (pts[i + 1] - pts[i]).length();
                if (segLen < 0.5) continue;
                const u32 na = nodeFor(pts[i], heightAt(i), static_cast<u16>(navFlags & 0xFFFFu));
                const u32 nb = nodeFor(pts[i + 1], heightAt(i + 1), static_cast<u16>(navFlags & 0xFFFFu));
                if (na == nb) continue;
                NavEdge e{};
                e.toNode = nb;
                e.laneCount = static_cast<u16>(std::max(1, wi.profile.lanesTotal));
                e.direction = oneway || wi.profile.pedOnly ? 1 : 0;
                e.surface = static_cast<u8>(wi.surface);
                e.speedLimitKph = static_cast<f32>(speed);
                e.length = static_cast<f32>(segLen);
                e.flags = navFlags;
                e.roadClass = static_cast<u32>(wi.cls);
                e.bezier[0] = static_cast<f32>(pts[i].x - cd.origin.x);
                e.bezier[1] = heightAt(i);
                e.bezier[2] = static_cast<f32>(pts[i].y - cd.origin.y);
                e.bezier[3] = static_cast<f32>(pts[i + 1].x - cd.origin.x);
                e.bezier[4] = heightAt(i + 1);
                e.bezier[5] = static_cast<f32>(pts[i + 1].y - cd.origin.y);
                pend.push_back(PendingEdge{na, e});
                ctx.stats.roadLengthMetres += static_cast<u64>(segLen);
            }
        }

        // --- 2. Connect dangling endpoints -----------------------------------
        // OSM ways do not always share a coordinate where they meet, and the
        // synthesised lattice is generated per ward, so thousands of way ends
        // dangle.  A dangling end is a wall to the traffic model — the vehicle
        // reaches it and has nowhere to go — so each one is joined to the nearest
        // node within 25 m (never to its own predecessor, which would just make a
        // two-way stub).  This is graph repair, not invention: the join is a
        // short link at the end of a road that visibly continues.
        {
            const f64 snapRadius = 25.0;
            const u32 nodeCount = static_cast<u32>(cd.navNodes.size());
            std::vector<u32> outDeg(nodeCount, 0), inDeg(nodeCount, 0);
            for (const PendingEdge& p : pend) {
                ++outDeg[p.from];
                ++inDeg[p.e.toNode];
            }
            // Grid index at 25 m, one bucket per node.
            std::unordered_map<u64, std::vector<u32>> grid;
            auto bucket = [](i64 x, i64 z) {
                return (static_cast<u64>(static_cast<u32>(x)) << 32) | static_cast<u32>(z);
            };
            for (u32 i = 0; i < nodeCount; ++i) {
                const NavNode& n = cd.navNodes[i];
                grid[bucket(static_cast<i64>(std::floor(n.x / snapRadius)),
                            static_cast<i64>(std::floor(n.z / snapRadius)))].push_back(i);
            }
            u32 links = 0;
            for (u32 i = 0; i < nodeCount && links < 4000; ++i) {
                if (outDeg[i] != 0 || inDeg[i] == 0) continue;      // not a dangling end
                const NavNode& a = cd.navNodes[i];
                const i64 gx = static_cast<i64>(std::floor(a.x / snapRadius));
                const i64 gz = static_cast<i64>(std::floor(a.z / snapRadius));
                f64 bestD2 = snapRadius * snapRadius;
                u32 bestNode = 0xFFFFFFFFu;
                for (i64 dx = -1; dx <= 1; ++dx)
                    for (i64 dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find(bucket(gx + dx, gz + dz));
                        if (it == grid.end()) continue;
                        for (u32 j : it->second) {
                            if (j == i) continue;
                            const NavNode& b = cd.navNodes[j];
                            const f64 ddx = static_cast<f64>(b.x) - a.x;
                            const f64 ddz = static_cast<f64>(b.z) - a.z;
                            const f64 d2 = ddx * ddx + ddz * ddz;
                            if (d2 < bestD2) { bestD2 = d2; bestNode = j; }
                        }
                    }
                if (bestNode == 0xFFFFFFFFu) continue;
                const NavNode& b = cd.navNodes[bestNode];
                NavEdge e{};
                e.toNode = bestNode;
                e.laneCount = 1;
                e.direction = 0;
                e.surface = 0;
                e.speedLimitKph = 20.0f;
                e.length = static_cast<f32>(std::sqrt(std::max(0.25, bestD2)));
                e.flags = NavFlagDrivable;
                e.roadClass = 7;                                    // residential link
                e.bezier[0] = a.x; e.bezier[1] = a.y; e.bezier[2] = a.z;
                e.bezier[3] = b.x; e.bezier[4] = b.y; e.bezier[5] = b.z;
                pend.push_back(PendingEdge{i, e});
                ++outDeg[i];
                ++inDeg[bestNode];
                ++links;
                ctx.stats.roadLengthMetres += static_cast<u64>(e.length);
            }
            if (links) MLOG_INFO("nav: %u dangling ends joined in one cell", links);
        }

        // --- 3. Contiguous out-edge runs ------------------------------------
        const u32 nodeCount = static_cast<u32>(cd.navNodes.size());
        std::vector<u32> counts(nodeCount, 0);
        for (const PendingEdge& p : pend) counts[p.from]++;
        u32 cursor = 0;
        for (u32 i = 0; i < nodeCount; ++i) {
            cd.navNodes[i].firstEdge = cursor;
            cd.navNodes[i].edgeCount = static_cast<u16>(std::min<u32>(counts[i], 0xFFFFu));
            cursor += counts[i];
        }
        std::vector<u32> fill(nodeCount, 0);
        for (u32 i = 0; i < nodeCount; ++i) fill[i] = cd.navNodes[i].firstEdge;
        cd.navEdges.resize(pend.size());
        for (const PendingEdge& p : pend) cd.navEdges[fill[p.from]++] = p.e;

        // --- 3. Signals ------------------------------------------------------
        // Approaches, grouped by their heading.  Two phases, 90-second cycle:
        // the dominant axis runs for 42 s from t=0, the crossing street for 42 s
        // from t=45, and the runtime's 3 s amber plus 1.5 s all-red fill the
        // gaps.  A junction earns a signal by having at least two approaches on
        // a secondary-or-better road (the OSM enum runs most-important-first).
        std::vector<std::vector<u32>> incoming(nodeCount);
        for (u32 i = 0; i < cd.navEdges.size(); ++i) {
            const u32 to = cd.navEdges[i].toNode;
            if (to < nodeCount) incoming[to].push_back(i);
        }
        for (u32 n = 0; n < nodeCount; ++n) {
            const u32 outDeg = cd.navNodes[n].edgeCount;
            u32 degree = outDeg + static_cast<u32>(incoming[n].size());
            maxDegree = std::max(maxDegree, degree);
            if (degree < 3) {
                if (outDeg == 0 && !incoming[n].empty()) ++deadEnds;
                continue;
            }
            ++junctions3;
            u32 major = 0, tertiary = 0, wide = 0;
            Vec2d axis{0, 0};
            // Arms, not edges: every incident segment counts once, whether the
            // source stored it as an approach to this node or as a departure from
            // it.  A road that only passes through contributes two arms on one
            // alignment, which the axis test below recognises.
            auto armOf = [&](const NavEdge& ne) {
                const u32 cls = ne.roadClass;
                if (cls >= 1 && cls <= 4) ++major;            // motorway .. secondary
                else if (cls == 5 || cls == 6) ++tertiary;    // tertiary, unclassified
                if (ne.laneCount >= 4) ++wide;
                if (axis.lengthSq() == 0.0 && cls >= 1 && cls <= 6) {
                    const Vec2d d{ne.bezier[3] - ne.bezier[0], ne.bezier[5] - ne.bezier[2]};
                    if (d.lengthSq() > 1e-9) axis = d.normalized();
                }
            };
            for (u32 e : incoming[n]) armOf(cd.navEdges[e]);
            for (u32 k = 0; k < outDeg; ++k) {
                const u32 e = cd.navNodes[n].firstEdge + k;
                if (e < cd.navEdges.size()) armOf(cd.navEdges[e]);
            }
            // Every junction is classified by the shape that warrants its
            // signal, so the numbers below are a decision and not a guess.
            const navsignal::JunctionShape shape{major, tertiary, wide, degree};
            const navsignal::Warrant cause = navsignal::classifyJunction(shape);
            if (cause == navsignal::Warrant::None) { ++warrantCause[WC_None]; continue; }
            ++warrantCause[static_cast<int>(cause)];
            if (axis.lengthSq() == 0.0) { ++warrantCause[WC_NoAxis]; continue; }
            ++junctionsSignalled;
            // One phase per arm, because the runtime turns every two-way segment
            // into both approaches: a phase on an outgoing segment is the phase
            // its opposing approach obeys.
            for (u32 e : incoming[n]) {
                const NavEdge& ne = cd.navEdges[e];
                const Vec2d dir{ne.bezier[3] - ne.bezier[0], ne.bezier[5] - ne.bezier[2]};
                if (dir.lengthSq() < 1e-9) continue;
                const Vec2d u = dir.normalized();
                const bool along = navsignal::armRunsAlong(u, axis);
                NavSignalPhase p{};
                p.edgeIndex = e;
                p.startGreenDeciseconds = static_cast<u16>(
                    along ? 0 : navsignal::kCrossOffsetDeciseconds);
                p.greenDeciseconds = static_cast<u16>(navsignal::kGreenDeciseconds);
                cd.signals.push_back(p);
                ++totalSignals;
            }
            for (u32 k = 0; k < outDeg; ++k) {
                const u32 e = cd.navNodes[n].firstEdge + k;
                if (e >= cd.navEdges.size()) continue;
                const NavEdge& ne = cd.navEdges[e];
                const Vec2d dir{ne.bezier[3] - ne.bezier[0], ne.bezier[5] - ne.bezier[2]};
                if (dir.lengthSq() < 1e-9) continue;
                const bool along = navsignal::armRunsAlong(dir.normalized(), axis);
                NavSignalPhase p{};
                p.edgeIndex = e;
                p.startGreenDeciseconds = static_cast<u16>(
                    along ? 0 : navsignal::kCrossOffsetDeciseconds);
                p.greenDeciseconds = static_cast<u16>(navsignal::kGreenDeciseconds);
                cd.signals.push_back(p);
                ++totalSignals;
            }
        }

        totalNodes += nodeCount;
        totalEdges += cd.navEdges.size();
    }

    ctx.stats.navNodes = totalNodes;
    ctx.stats.navEdges = totalEdges;
    ctx.stats.navSignals = totalSignals;
    MLOG_INFO("nav: signal warrant — two classified %llu, classified+wide %llu, three collectors %llu, two wide %llu, no warrant %llu, no axis %llu",
              (unsigned long long)warrantCause[WC_TwoClassified],
              (unsigned long long)warrantCause[WC_ClassifiedWideJunction],
              (unsigned long long)warrantCause[WC_ThreeCollectors],
              (unsigned long long)warrantCause[WC_TwoWide],
              (unsigned long long)warrantCause[WC_None],
              (unsigned long long)warrantCause[WC_NoAxis]);
    MLOG_INFO("nav: %llu nodes, %llu edges, %llu junctions (3+), %llu signalled, %llu signals, %llu dead ends, max degree %u",
              (unsigned long long)totalNodes, (unsigned long long)totalEdges,
              (unsigned long long)junctions3, (unsigned long long)junctionsSignalled,
              (unsigned long long)totalSignals, (unsigned long long)deadEnds, maxDegree);
}

}  // namespace

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

    planarizeCrossings(ctx, ways);
    buildNavGraph(ctx, ways);

    MLOG_INFO("roads: %llu ways, %.1f km centreline, %llu junctions, %llu triangles",
              (unsigned long long)ctx.stats.roadWays,
              ctx.stats.roadLengthMetres / 1000.0,
              (unsigned long long)ctx.stats.junctions,
              (unsigned long long)ctx.stats.roadTriangles);
    ctx.qa.add("roads.wayCount", static_cast<f64>(ctx.stats.roadWays));
    ctx.qa.add("roads.centrelineKilometres", ctx.stats.roadLengthMetres / 1000.0);
    ctx.qa.add("roads.junctionCount", static_cast<f64>(ctx.stats.junctions));
    ctx.qa.add("roads.triangles", static_cast<f64>(ctx.stats.roadTriangles));
    ctx.qa.add("roads.navNodes", static_cast<f64>(ctx.stats.navNodes));
    ctx.qa.add("roads.navEdges", static_cast<f64>(ctx.stats.navEdges));
    ctx.qa.add("roads.navSignals", static_cast<f64>(ctx.stats.navSignals));
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
