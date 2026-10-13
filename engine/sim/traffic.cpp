// =============================================================================
//  MUMBAI 1:1 EXACT — engine/sim/traffic.cpp
//
//  Conventions used throughout:
//    * `s` is the position of the vehicle's FRONT bumper, measured from the
//      start of its edge; a vehicle leaves the edge when s >= edge.length.
//    * `gap` is therefore (leader.s - leader.length) - s, the distance from this
//      front bumper to the leader's rear bumper.  A signal or a junction that a
//      vehicle must hold for is presented as a stopped leader at that point, so
//      one longitudinal law covers car-following, stop lines and junction
//      blocking.
//    * `lateral` is metres to the right of the edge centreline.  Anything that
//      holds a lane sits at (lane - (lanes-1)/2) * 3.5 m and eases between lane
//      centres; a two-wheeler holds no lane and is steered by the Boids field,
//      which is the whole point of the two-wheeler model.
//    * Nothing here allocates per tick, and nothing reads the clock: identical
//      input gives bit-identical output, which the validation suite checks.
// =============================================================================
#include "engine/sim/traffic.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace mumbai::sim {
namespace {

constexpr f64 kLaneWidth = 3.5;            // m, centre to centre
constexpr f64 kLaneChangeInterval = 0.4;   // s between MOBIL evaluations per vehicle
constexpr f64 kMobilPoliteness = 0.2;      // MOBIL p: how much the others count
constexpr f64 kMobilThreshold = 0.2;       // m/s^2 of incentive before changing
constexpr f64 kMobilSafeBrake = -3.0;      // m/s^2 the new follower must stay above
constexpr f64 kFilterClearance = 0.9;      // m, clearance to a flank
constexpr f64 kStopLineMargin = 0.5;       // m, in front of the stop line
constexpr f64 kVisualRange = 40.0;         // m, how far a two-wheeler looks ahead
constexpr u32 kBucketsPerEdge = 16;        // 16 x 20 m of spatial hash for Boids
// The population device.  kTrailGrace is how far clear of the disc a vehicle may
// get before it is called back; kRecyclePerTick bounds the work one tick can do,
// and a failed placement ends the sweep for that tick: a full device will refuse
// the rest of the queue too, and asking is the most expensive thing a tick does.
constexpr f64 kTrailGrace = 120.0;
constexpr u32 kRecyclePerTick = 8;
constexpr f64 kLateralKp = 3.0;            // lane-keeping spring
constexpr f64 kLateralKd = 2.0 * 1.7320508075688772;   // critically damped

// name, length, mass, width, comfort accel, comfort brake, max accel, max brake,
// top speed, two-wheeler, hails, can tip, tip lateral g
const ArchetypeSpec kSpecs[] = {
    {"motorcycle",         1.95,   135.0,  0.75, 0.05, 0.80, 3.0, 4.0, 80.0, true,  false, false, 0.0},
    {"auto-rickshaw",      2.60,   400.0,  1.40, 0.20, 0.70, 1.5, 3.0, 55.0, false, false, true,  0.36},
    {"kaali-peeli taxi",   3.90,  1150.0,  1.65, 0.40, 0.60, 2.5, 3.5, 85.0, false, true,  false, 0.0},
    {"car",                4.40,  1400.0,  1.75, 0.45, 0.55, 2.6, 3.6, 85.0, false, false, false, 0.0},
    {"BEST double-decker", 11.90, 16000.0, 2.55, 0.90, 0.30, 1.2, 2.0, 60.0, false, false, false, 0.0},
};
static_assert(sizeof(kSpecs) / sizeof(kSpecs[0]) == static_cast<size_t>(Archetype::Count),
              "one archetype spec per enum value");

f64 laneOffset(u16 lane, u16 lanes) {
    return (static_cast<f64>(lane) - (static_cast<f64>(lanes) - 1.0) * 0.5) * kLaneWidth;
}

f64 idmAccel(f64 v, f64 v0, f64 gap, f64 leaderV, f64 a, f64 b, f64 T, f64 s0) {
    const f64 dv = v - leaderV;
    const f64 sStar = s0 + std::max(0.0, v * T + v * dv / (2.0 * std::sqrt(std::max(1e-6, a * b))));
    const f64 free = 1.0 - std::pow(v / std::max(2.0, v0), 4.0);
    const f64 inter = (gap < 1e8) ? std::pow(sStar / std::max(0.1, gap), 2.0) : 0.0;
    return a * (free - inter);
}

}  // namespace

const ArchetypeSpec& archetypeSpec(Archetype a) {
    const size_t i = std::min(static_cast<size_t>(a), static_cast<size_t>(Archetype::Count) - 1);
    return kSpecs[i];
}

SignalState signalStateAt(const SignalPhase& p, f64 t) {
    const f64 cycle = SignalPhase::kCycleSeconds;
    f64 local = std::fmod(t - p.greenStart, cycle);
    if (local < 0.0) local += cycle;
    if (local < p.greenSeconds) return SignalState::Green;
    if (local < p.greenSeconds + SignalPhase::kAmberSeconds) return SignalState::Amber;
    return SignalState::Red;
}

// -----------------------------------------------------------------------------
//  Construction: assemble the graph from the world, then populate it
// -----------------------------------------------------------------------------
Traffic::Traffic(const std::vector<std::string>& chunkFiles,
                 const std::vector<Vec2d>& route,
                 const Vec2d& denseAnchor, u64 fleet, u64 seed)
    : m_denseAnchor(denseAnchor), m_rngState(seed ? seed : 0x5EED1234ull) {
    (void)route;
    std::unordered_map<u64, u32> nodeMap;
    for (const std::string& file : chunkFiles) {
        ChunkReader r;
        std::string err;
        if (!r.open(file, &err)) continue;
        const ChunkReader::NavGraphView graph = r.navGraph();
        if (!graph.valid()) continue;
        const u32 nodeCount = graph.header->nodeCount;
        const NavNode* nodes = graph.nodes;
        const NavEdge* edges = graph.edges;
        const f64 ox = static_cast<f64>(r.header().cellX) * MACRO_CELL_SIZE;
        const f64 oz = static_cast<f64>(r.header().cellZ) * MACRO_CELL_SIZE;
        const u64 cellKey = (static_cast<u64>(r.header().cellX + 1) << 16) ^
                            static_cast<u64>(r.header().cellZ + 1);
        // Local edge index -> global edge index, so a signal phase can name the
        // edge it governs without re-deriving topology.
        std::vector<u32> localToGlobal(graph.header->edgeCount, kNoSignal);

        // Nodes are jointed on world position, and the two kinds of joint are
        // counted apart: a repeat inside one file is a duplicate the producer
        // left, a joint onto a node another file created is the chunk seam.
        const u32 nodesBefore = static_cast<u32>(m_nodes.size());
        for (u32 i = 0; i < nodeCount; ++i) {
            const Vec2d p{ox + nodes[i].x, oz + nodes[i].z};
            bool created = false;
            const u32 gi = nodeAt(p, &created);
            nodeMap[(cellKey << 32) | i] = gi;
            if (created) continue;
            if (gi < nodesBefore) ++m_nodesMergedAcrossChunks;
            else ++m_nodesMerged;
        }
        for (u32 i = 0; i < nodeCount; ++i) {
            const NavNode& n = nodes[i];
            const u32 from = nodeMap[(cellKey << 32) | i];
            for (u32 k = 0; k < n.edgeCount; ++k) {
                if (n.firstEdge + k >= graph.header->edgeCount) break;
                const NavEdge& e2 = edges[n.firstEdge + k];
                if (e2.toNode >= nodeCount) continue;
                auto it = nodeMap.find((cellKey << 32) | e2.toNode);
                if (it == nodeMap.end()) continue;
                SimEdge se;
                se.from = from;
                se.to = it->second;
                se.length = e2.length > 0.5f ? e2.length : 10.0f;
                se.limit = e2.speedLimitKph > 1.0f ? e2.speedLimitKph : 25.0f;
                se.lanes = std::max<u16>(1, e2.laneCount);
                se.cls = static_cast<u8>(e2.roadClass);
                se.a = m_nodes[se.from].p;
                se.b = m_nodes[se.to].p;
                const Vec2d d = se.b - se.a;
                if (d.lengthSq() < 0.01) continue;
                se.dir = d.normalized();
                se.normal = Vec2d{-se.dir.y, se.dir.x};
                se.laneVehicles.assign(se.lanes, {});
                const u32 fwd = static_cast<u32>(m_edges.size());
                const bool bothWays = e2.direction != 1;
                localToGlobal[n.firstEdge + k] = fwd;
                se.reverse = bothWays ? fwd + 1u : kNoEdge;
                m_nodes[from].out.push_back(fwd);
                m_edges.push_back(std::move(se));
                if (bothWays) {
                    // The opposing approach of a two-way segment: same surface,
                    // same lanes, reversed geometry.
                    SimEdge rv = m_edges[fwd];
                    rv.from = m_edges[fwd].to;
                    rv.to = from;
                    rv.a = m_edges[fwd].b;
                    rv.b = m_edges[fwd].a;
                    rv.dir = Vec2d{-m_edges[fwd].dir.x, -m_edges[fwd].dir.y};
                    rv.normal = Vec2d{-m_edges[fwd].normal.x, -m_edges[fwd].normal.y};
                    rv.laneVehicles.assign(rv.lanes, {});
                    rv.reverse = fwd;
                    const u32 rev = static_cast<u32>(m_edges.size());
                    m_nodes[rv.from].out.push_back(rev);
                    m_edges.push_back(std::move(rv));
                }
            }
        }

        // Signal phases: the world has carried a green window per edge since the
        // format was defined.  A phase governs the approach along its edge to the
        // edge's far node; where the format could only name one direction of a
        // two-way segment, the runtime completes the other approach afterwards
        // (see deriveSignals).
        for (u32 s = 0; s < graph.header->signalCount; ++s) {
            const NavSignalPhase& sp = graph.signals[s];
            if (sp.edgeIndex >= graph.header->edgeCount) continue;
            const u32 gi = localToGlobal[sp.edgeIndex];
            if (gi == kNoSignal) continue;
            SignalPhase p;
            p.edge = gi;
            p.greenStart = static_cast<f64>(sp.startGreenDeciseconds) * 0.1;
            p.greenSeconds = static_cast<f64>(sp.greenDeciseconds) * 0.1;
            if (p.greenSeconds <= 0.0) continue;
            m_signals.push_back(p);
        }
    }

    m_signalOfEdge.assign(m_edges.size(), kNoSignal);
    for (u32 s = 0; s < m_signals.size(); ++s) m_signalOfEdge[m_signals[s].edge] = s;
    deriveSignals();
    m_edgeOrder.assign(m_edges.size(), {});
    m_buckets.assign(m_edges.size() * kBucketsPerEdge, {});

    if (m_edges.empty() || !fleet) return;

    setDenseAnchor(m_denseAnchor, m_denseRadius);

    // Fleet mix: two-wheeler country first, then autos, taxis, cars, and a few
    // BEST double-deckers.  A signal at Andheri looks like these proportions.
    m_vehicles.reserve(fleet);
    for (u64 i = 0; i < fleet; ++i) {
        const bool inDense = (i % 10) != 0;
        const u32 k = rng(100);
        Archetype kind = Archetype::Car;
        if (k < 46) kind = Archetype::Motorcycle;
        else if (k < 60) kind = Archetype::AutoRickshaw;
        else if (k < 74) kind = Archetype::Taxi;
        else if (k < 93) kind = Archetype::Car;
        else kind = Archetype::Bus;
        const ArchetypeSpec& spec = archetypeSpec(kind);
        for (int attempt = 0; attempt < 24; ++attempt) {
            const u32 e = inDense ? m_denseEdges[rng(static_cast<u32>(m_denseEdges.size()))]
                                  : rng(static_cast<u32>(m_edges.size()));
            const SimEdge& edge = m_edges[e];
            const f64 spare = edge.length - spec.length - 3.0;
            if (spare < 0.5) continue;
            const u16 lane = static_cast<u16>(rng(edge.lanes));
            bool placed = false;
            for (f64 cand = kLaneWidth * 0.5; cand <= spare && !placed; cand += kLaneWidth) {
                bool clear = true;
                for (u32 other : edge.laneVehicles[lane])
                    if (std::fabs(m_vehicles[other].s - cand) < spec.length + 3.0) { clear = false; break; }
                if (!clear) continue;
                spawn(kind, e, lane, cand, spec.topSpeedKph / 3.6 * 0.75 * 0.35);
                placed = true;
            }
            if (placed) break;
        }
    }
    m_spawned = m_vehicles.size();
}

u32 Traffic::spawn(Archetype kind, u32 edge, u16 lane, f64 s, f64 v) {
    const ArchetypeSpec& spec = archetypeSpec(kind);
    Vehicle veh;
    veh.kind = kind;
    veh.edge = std::min<u32>(edge, static_cast<u32>(m_edges.size() ? m_edges.size() - 1 : 0));
    if (m_edges.empty()) return 0;
    SimEdge& e = m_edges[veh.edge];
    veh.lane = std::min<u16>(lane, static_cast<u16>(e.lanes - 1));
    veh.length = spec.length;
    veh.a = spec.maxAccel * spec.comfortAccel;
    veh.b = spec.maxBrake * spec.comfortBrake;
    veh.T = spec.twoWheeler ? 0.9 : (kind == Archetype::Bus ? 1.8 : 1.2);
    veh.s0 = spec.twoWheeler ? 1.2 : 2.2;
    veh.v0 = std::min(spec.topSpeedKph / 3.6 * 0.75, e.limit / 3.6 * 1.05);
    veh.lateral = laneOffset(veh.lane, e.lanes);
    if (spec.twoWheeler) {
        // Never on a lane centre: scattered across the carriageway, which is
        // where a two-wheeler actually rides.
        const f64 half = e.lanes * kLaneWidth * 0.5 - 0.5;
        veh.lateral = std::clamp(veh.lateral + (rng01() - 0.5) * 2.4, -half, half);
    }
    if (spec.canHail) veh.hailTimer = rng01() * 600.0;
    veh.s = std::clamp(s, 0.0, std::max(0.0, e.length - veh.length - 0.05));
    // Never create a vehicle inside another one: a spawn point that is occupied
    // is refused, loudly, rather than silently overlapped and then pushed apart
    // by the next tick's corrections.
    for (u32 other : e.laneVehicles[veh.lane]) {
        const Vehicle& o = m_vehicles[other];
        const bool overlaps = (veh.s < o.s + o.length + kMinGap) && (o.s < veh.s + veh.length + kMinGap);
        if (overlaps) return kNoVehicle;
    }
    veh.v = std::clamp(v, 0.0, veh.v0 * 1.3);
    m_vehicles.push_back(veh);
    const u32 idx = static_cast<u32>(m_vehicles.size() - 1);
    std::vector<u32>& list = e.laneVehicles[veh.lane];
    list.insert(std::lower_bound(list.begin(), list.end(), idx,
                                 [this](u32 a, u32 b) { return m_vehicles[a].s < m_vehicles[b].s; }),
                idx);
    return idx;
}

void Traffic::insertSorted(std::vector<u32>& list, u32 idx) {
    list.insert(std::lower_bound(list.begin(), list.end(), idx,
                                 [this](u32 a, u32 b) { return m_vehicles[a].s < m_vehicles[b].s; }),
                idx);
}

void Traffic::placeHeld(Vehicle& veh, const SimEdge& e, u16 lane, f64 limit) {
    // A vehicle held at the end of its edge — because the junction will not take
    // it, or because the road simply ends — must still be behind nothing and in
    // front of everything.  Its own follower is the constraint: holding it at the
    // edge end is only safe if the vehicle behind is far enough back, and when it
    // is not, the held vehicle takes the small forward overshoot instead.  (An
    // earlier version simply placed it at `limit - 0.05`, which drove it into its
    // own follower whenever the queue behind was tight.)
    // Two positions, and the vehicle takes the forward-most: just short of the
    // edge end (where a hold wants it, so that it is no longer an arrival and does
    // not re-run the junction every tick), and clear of the body behind it.  The
    // floor term uses MY length, not the follower's — the gap is measured from the
    // rear face of the vehicle being placed — and it is what keeps the pull-back
    // from being a push-back into the queue, which was the last sub-floor pair the
    // diagnostic found.
    f64 want = limit - 0.05;
    for (u32 other : e.laneVehicles[lane]) {
        const Vehicle& o = m_vehicles[other];
        if (o.s >= veh.s) continue;
        want = std::max(want, o.s + veh.length + kMinGap + kEntryMargin);
    }
    // A body may stand in the junction it is waiting for, but never further than
    // its own length, or a long hold would walk into the crossing street.  When
    // the cap binds the vehicle is stopped outright and the event is counted: the
    // queue behind it gives way instead (its correction runs against this fixed
    // position on the next tick), and a hold that had to stop is a number the
    // validation reports rather than an error it hides.
    const f64 cap = limit + veh.length;
    if (want > cap) {
        ++m_holdCaps;
        veh.v = 0.0;
    }
    veh.s = std::min(want, cap);
}

bool Traffic::laneEntryClear(const SimEdge& e, u16 lane, f64 s, f64 length, f64 width) const {
    // Space on a carriageway is a PHYSICAL question, not a bookkeeping one: the
    // check covers every vehicle on the edge whose lateral footprint overlaps
    // where this one is going, because a two-wheeler that has filtered across two
    // lanes is still registered in the lane it entered by, and a vehicle may join
    // at `s` only if its front bumper would leave the vehicle ahead a gap of at
    // least the floor and its rear bumper would leave the vehicle behind the same.
    // (Demanding a fixed absolute clearance instead throttled every junction to
    // one vehicle per few seconds; checking only the target lane let a car be
    // created inside a machine that had drifted into its path.)
    const f64 myLateral = laneOffset(lane, e.lanes);
    for (u16 l = 0; l < e.lanes; ++l) {
        for (u32 other : e.laneVehicles[l]) {
            const Vehicle& o = m_vehicles[other];
            // Lateral footprint: bodies that cannot touch are not each other's
            // problem, however their bookkeeping lanes are arranged.
            const f64 lateralClear =
                0.5 * (width + archetypeSpec(o.kind).width) + 0.4;
            if (std::fabs(o.lateral - myLateral) > lateralClear) continue;
            if (o.s > s) {
                if (o.s - o.length - s < kMinGap + kEntryMargin) return false;
                // And it must have cleared the box: following a long vehicle onto
                // an edge while its tail still lies across the junction is how a
                // body ends up straddling a boundary.
                if (o.s < o.length) return false;
            } else {
                // A body behind: the entrant's rear face must leave it the floor.
                // Where that body could be pushed back TO is not the question — a
                // queue spills across the junction without limit — so the distance
                // is measured to where it actually is.
                if (s - length - o.s < kMinGap + kEntryMargin) return false;
            }
        }
    }
    return true;
}

u32 Traffic::rng(u32 bound) {
    m_rngState += 0x9E3779B97F4A7C15ull;
    u64 z = m_rngState;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return static_cast<u32>((z ^ (z >> 31)) % (bound ? bound : 1));
}

f64 Traffic::rng01() { return static_cast<f64>(rng(1u << 20)) / static_cast<f64>(1u << 20); }

Vec2d Traffic::positionOf(const Vehicle& v) const {
    const SimEdge& e = m_edges[v.edge];
    return e.a + e.dir * v.s + e.normal * v.lateral;
}

// -----------------------------------------------------------------------------
//  Leaders, the two-wheeler field, and signal heads
// -----------------------------------------------------------------------------
void Traffic::leadersAndSignals() {
    // Lane leaders.  Two-wheelers are deliberately excluded: they hold no lane,
    // and their longitudinal reference is found by the lateral-aware search
    // below, which sees the whole carriageway.
    for (u32 ei : m_activeEdges) {
        SimEdge& e = m_edges[ei];
        for (u16 l = 0; l < e.lanes; ++l) {
            const std::vector<u32>& list = e.laneVehicles[l];
            for (size_t k = 0; k < list.size(); ++k) {
                Vehicle& veh = m_vehicles[list[k]];
                if (archetypeSpec(veh.kind).twoWheeler) continue;
                veh.obstruction = 1e18;
                veh.leaderV = veh.v;
                veh.atRedLine = false;
                // The first vehicle ahead that is actually IN THE WAY: lane
                // registration is bookkeeping, lateral footprint is physics.  A
                // vehicle halfway through a lane change is still out there in the
                // lane it is leaving.
                for (size_t j = k + 1; j < list.size(); ++j) {
                    const Vehicle& lead = m_vehicles[list[j]];
                    if (std::fabs(lead.lateral - veh.lateral) >
                        0.5 * (archetypeSpec(veh.kind).width + archetypeSpec(lead.kind).width))
                        continue;
                    veh.obstruction = lead.s - lead.length;
                    veh.leaderV = lead.v;
                    break;
                }
                veh.gap = std::min(1e9, veh.obstruction - veh.s);
            }
        }
    }

    // Order every edge's vehicles by s.  The lane lists are each sorted, so the
    // merged order is a merge of sorted runs; a sort of the concatenation is
    // cheap at this population and keeps the code obvious.
    for (u32 ei : m_activeEdges) {
        SimEdge& e = m_edges[ei];
        // A two-wheeler that filters past a lane holder overtakes it inside the
        // same lane list.  Repair the order when that happens: every correction
        // below assumes the list runs front to back.
        for (u16 l = 0; l < e.lanes; ++l) {
            std::vector<u32>& list = e.laneVehicles[l];
            bool ordered = true;
            for (size_t k = 1; k < list.size() && ordered; ++k)
                if (m_vehicles[list[k]].s < m_vehicles[list[k - 1]].s) ordered = false;
            if (!ordered)
                std::sort(list.begin(), list.end(),
                          [this](u32 a, u32 b) { return m_vehicles[a].s < m_vehicles[b].s; });
        }
        std::vector<u32>& order = m_edgeOrder[static_cast<u32>(&e - m_edges.data())];
        order.clear();
        for (u16 l = 0; l < e.lanes; ++l)
            order.insert(order.end(), e.laneVehicles[l].begin(), e.laneVehicles[l].end());
        std::sort(order.begin(), order.end(),
                  [this](u32 a, u32 b) { return m_vehicles[a].s < m_vehicles[b].s; });
    }

    // A two-wheeler follows the nearest vehicle ahead whose body overlaps its own
    // line, wherever on the carriageway that is.  This is what lets a bike filter
    // past a bus: the bus is not in its path, so it is not its leader.
    for (u32 i = 0; i < m_vehicles.size(); ++i) {
        Vehicle& veh = m_vehicles[i];
        if (!archetypeSpec(veh.kind).twoWheeler) continue;
        const std::vector<u32>& order = m_edgeOrder[veh.edge];
        veh.obstruction = 1e18;          // found afresh every tick
        veh.atRedLine = false;
        // Scan forward along the road, and take the first body whose lateral
        // footprint overlaps this rider's line.  A machine riding clear of a bus
        // is not following it; a machine riding into the bus's flank is, and the
        // obstruction is the bus's real rear bumper, not a nominal one.
        f64 best = 1e9, bestV = veh.v, bestLen = 0.0;
        bool found = false;
        for (size_t k = 0; k < order.size() && !found; ++k) {
            const u32 j = order[k];
            if (j == i) continue;
            const Vehicle& other = m_vehicles[j];
            if (other.s <= veh.s) continue;                  // behind
            if (other.s - veh.s > kVisualRange) break;       // sorted: nothing nearer
            const f64 lateralClear = kFilterClearance + archetypeSpec(other.kind).width * 0.5;
            if (std::fabs(other.lateral - veh.lateral) > lateralClear) continue;
            best = other.s;
            bestV = other.v;
            bestLen = other.length;
            found = true;
        }
        if (found) {
            const f64 rear = best - bestLen;
            if (rear - kMinGap < veh.s || rear < 1e17) {
                veh.obstruction = rear;
                veh.leaderV = bestV;
            }
        }
        veh.gap = std::min(1e9, veh.obstruction - veh.s);
    }

    // Signal heads.  A phase that is not green presents a stopped leader at the
    // stop line, so vehicles queue, creep and hold — which is what a junction is.
    // Amber is stop-if-you-can.
    for (const SignalPhase& p : m_signals) {
        const SignalState st = signalStateAt(p, m_time);
        if (st == SignalState::Green) continue;
        const SimEdge& e = m_edges[p.edge];
        const bool amber = (st == SignalState::Amber);
        for (u16 l = 0; l < e.lanes; ++l) {
            for (u32 vi : e.laneVehicles[l]) {
                Vehicle& veh = m_vehicles[vi];
                const f64 toLine = e.length - kStopLineMargin - veh.s;
                if (toLine <= 0.0) continue;                 // past the line: committed
                if (amber && veh.v > 4.0 && toLine > veh.v * 1.2 + 2.0) continue;
                if (toLine < veh.obstruction - veh.s) {
                    veh.obstruction = e.length - kStopLineMargin;
                    veh.gap = std::max(0.0, toLine);
                    veh.leaderV = 0.0;
                    // Held by the aspect, not by another vehicle: a queue behind a
                    // queue is a junction problem, a queue behind a red light is a
                    // signal doing its job.  The two are counted apart.
                    veh.atRedLine = toLine <= kStopLineHold;
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
//  Longitudinal model
// -----------------------------------------------------------------------------
void Traffic::longitudinal(f64 dt) {
    // 1. Integrate.  IDM is asymptotically safe, but a single step can overshoot
    //    while a platoon compresses, so the integration is followed by exact
    //    corrections rather than by trusting the ODE.
    for (Vehicle& veh : m_vehicles) {
        if (veh.hailing) {
            // A taxi that has been hailed stops at the kerb, comfortably, and
            // the queue behind it forms for real.
            veh.v = std::max(0.0, veh.v - veh.b * dt);
            veh.s += veh.v * dt;
            continue;
        }
        const f64 v0 = std::max(2.0, veh.v0);
        const f64 acc = idmAccel(veh.v, v0, veh.gap, veh.leaderV, veh.a, veh.b, veh.T, veh.s0);
        veh.v = std::clamp(veh.v + acc * dt, 0.0, v0 * 1.3);
        veh.s += veh.v * dt;
    }

}

// -----------------------------------------------------------------------------
//  Corrections — the exact pass that holds the floor
// -----------------------------------------------------------------------------
// Run once per tick, AFTER everything that can move a body: the integrator, the
// lane changes, the Boids field and the lateral tracker.  It cannot run earlier,
// because the floor is a property of the lateral footprint and a two-wheeler that
// slides under the vehicle in front of it would otherwise leave a pair closer than
// the floor with the sweep already behind it.  Nothing moves a body's s after
// this: the only later writes of s are admissions, and every admission tests the
// floor (laneEntryClear) before it takes the space.
void Traffic::corrections() {
    // 1. One descending-s sweep per edge, front of the queue to back.  Each
    //    vehicle's constraint is the tighter of its obstruction (a stop line, or
    //    the body a two-wheeler is riding up behind) and the real rear of the
    //    vehicle listed in front of it in the lane it is registered in.  Because
    //    every leader is settled before the vehicle behind it is clamped, the
    //    floor holds after ONE pass — two passes in sequence (lane discipline,
    //    then obstruction) let the second undo the first: a motorcycle that was
    //    pushed back by its own leader after the car behind it had already been
    //    corrected left a 0.29 m gap where the floor asks for 0.5 m.
    for (u32 ei : m_activeEdges) {
        const SimEdge& e = m_edges[ei];
        for (u16 l = 0; l < e.lanes; ++l) {
            const std::vector<u32>& list = e.laneVehicles[l];
            for (size_t k = list.size(); k-- > 0;) {
                Vehicle& veh = m_vehicles[list[k]];
                f64 constraint = veh.obstruction;
                f64 leaderV = veh.leaderV;
                if (k + 1 < list.size()) {
                    const Vehicle& lead = m_vehicles[list[k + 1]];
                    const f64 lateralClear = 0.5 * (archetypeSpec(veh.kind).width +
                                                    archetypeSpec(lead.kind).width);
                    if (std::fabs(lead.lateral - veh.lateral) <= lateralClear) {
                        const f64 rear = lead.s - lead.length;
                        if (rear < constraint) {
                            constraint = rear;
                            leaderV = lead.v;
                        }
                    }
                }
                if (constraint >= 1e17) continue;
                const f64 target = constraint - kMinGap;
                if (veh.s > target) {
                    // A queue that runs out of edge spills BACK over the junction
                    // behind it, as far as the queue reaches: s goes negative and
                    // the body is drawn on the edge behind, which is where it truly
                    // is.  There is no floor on that spill, and there must not be:
                    // a floor is a position every pushed-back body can occupy, so a
                    // long queue stacks on it — clamping at zero put two motorcycles
                    // at the same s, and half a metre of tolerance only moved the
                    // stack a metre.  Nothing else can clamp this vehicle either:
                    // the sweep runs front to back, so each body stops behind the
                    // one already corrected and the order is preserved by
                    // construction.
                    veh.s = target;
                    veh.v = std::min(veh.v, leaderV);
                    ++m_hardBrakes;
                }
            }
        }
    }

    // 2. Report the honest number: the smallest longitudinal gap between two
    //    bodies whose lateral footprints overlap, measured on the settled state.
    //    A gap measured before the corrections is not the gap the world has, and
    //    a pair riding abreast is not a pair following.
    for (Vehicle& veh : m_vehicles) veh.gap = std::min(1e9, veh.obstruction - veh.s);
    m_minGap = 1e9;
    for (u32 ei : m_activeEdges) {
        const SimEdge& e = m_edges[ei];
        for (u16 l = 0; l < e.lanes; ++l) {
            const std::vector<u32>& list = e.laneVehicles[l];
            for (size_t k = 1; k < list.size(); ++k) {
                const Vehicle& ahead = m_vehicles[list[k]];
                const Vehicle& behind = m_vehicles[list[k - 1]];
                const f64 lateralClear = 0.5 * (archetypeSpec(ahead.kind).width +
                                                archetypeSpec(behind.kind).width);
                if (std::fabs(ahead.lateral - behind.lateral) > lateralClear) continue;
                m_minGap = std::min(m_minGap, ahead.s - ahead.length - behind.s);
            }
        }
    }
    if (m_minGap > 1e8) m_minGap = 1e9;      // nobody is following anybody
}

// -----------------------------------------------------------------------------
//  The active set
// -----------------------------------------------------------------------------
// Which edges hold a vehicle this tick.  Every sweep that used to walk the whole
// graph walks this instead; it is rebuilt from the fleet (a few hundred entries
// against ~170k edges) and sorted so the sweeps stay deterministic.
void Traffic::collectActiveEdges() {
    m_activeEdges.clear();
    m_activeEdges.reserve(m_vehicles.size());
    for (const Vehicle& veh : m_vehicles) m_activeEdges.push_back(veh.edge);
    std::sort(m_activeEdges.begin(), m_activeEdges.end());
    m_activeEdges.erase(std::unique(m_activeEdges.begin(), m_activeEdges.end()),
                        m_activeEdges.end());
}

// -----------------------------------------------------------------------------
//  MOBIL lane changes (everything that holds a lane)
// -----------------------------------------------------------------------------
void Traffic::laneChanges(f64 dt) {
    m_laneAccum += dt;
    if (m_laneAccum < kLaneChangeInterval) return;
    m_laneAccum = 0.0;
    ++m_laneTick;

    for (Vehicle& veh : m_vehicles) {
        veh.changedLaneThisStep = false;
        if (archetypeSpec(veh.kind).twoWheeler) continue;    // two-wheelers do not use lanes
        SimEdge& e = m_edges[veh.edge];
        if (e.lanes < 2) continue;
        // One vehicle in four evaluates per interval, staggered deterministically.
        // (The rotation must be a tick counter, not a function of the clock: a
        // phase derived from m_time advances by exactly one interval's worth per
        // interval and is therefore constant for each vehicle forever.)
        if ((static_cast<u32>(&veh - m_vehicles.data()) + static_cast<u32>(m_laneTick)) % 4u) continue;

        const u32 self = static_cast<u32>(&veh - m_vehicles.data());
        const f64 aNow = idmAccel(veh.v, std::max(2.0, veh.v0), veh.gap, veh.leaderV,
                                  veh.a, veh.b, veh.T, veh.s0);
        for (int side = 0; side < 2 && !veh.changedLaneThisStep; ++side) {
            const i32 target = static_cast<i32>(veh.lane) + (side == 0 ? -1 : 1);
            if (target < 0 || target >= static_cast<i32>(e.lanes)) continue;

            f64 gapLead = 1e9, leadV = veh.v, gapFollow = 1e9;
            u32 followIdx = 0xFFFFFFFFu;
            for (u32 other : e.laneVehicles[static_cast<u16>(target)]) {
                const Vehicle& o = m_vehicles[other];
                if (o.s > veh.s) {
                    const f64 g = o.s - o.length - veh.s;
                    if (g < gapLead) { gapLead = g; leadV = o.v; }
                } else {
                    const f64 g = veh.s - veh.length - o.s;
                    if (g < gapFollow) { gapFollow = g; followIdx = other; }
                }
            }
            if (gapLead < kMinGap || gapFollow < kMinGap) continue;   // no room at all

            const f64 aAfter = idmAccel(veh.v, std::max(2.0, veh.v0), gapLead, leadV,
                                        veh.a, veh.b, veh.T, veh.s0);
            f64 followBefore = 0.0, followAfter = 0.0;
            if (followIdx != 0xFFFFFFFFu) {
                const Vehicle& f = m_vehicles[followIdx];
                followBefore = idmAccel(f.v, std::max(2.0, f.v0), gapFollow, veh.v,
                                        f.a, f.b, f.T, f.s0);
                const f64 newGap = std::max(0.05, veh.s - veh.length - f.s);
                followAfter = idmAccel(f.v, std::max(2.0, f.v0), newGap, veh.v,
                                       f.a, f.b, f.T, f.s0);
            }
            // MOBIL: my gain plus the politeness-weighted pain of whoever I cut
            // in front of.
            const f64 incentive = (aAfter - aNow) + kMobilPoliteness * (followAfter - followBefore);
            const bool safe = (followIdx == 0xFFFFFFFFu) || followAfter > kMobilSafeBrake;
            if (safe && incentive > kMobilThreshold) {
                e.laneVehicles[veh.lane].erase(
                    std::remove(e.laneVehicles[veh.lane].begin(), e.laneVehicles[veh.lane].end(), self),
                    e.laneVehicles[veh.lane].end());
                veh.lane = static_cast<u16>(target);
                // The lateral offset eases across in `lateralDynamics`, rate-limited
                // by the vehicle's rollover geometry.
                veh.lateralCapped = false;
                e.laneVehicles[veh.lane].push_back(self);
                std::sort(e.laneVehicles[veh.lane].begin(), e.laneVehicles[veh.lane].end(),
                          [this](u32 a, u32 b) { return m_vehicles[a].s < m_vehicles[b].s; });
                // Corrections go through the target-lane leader next tick.
                ++m_laneChanges;
                veh.changedLaneThisStep = true;
            } else {
                ++m_laneChangeRefusals;
            }
        }
    }
}

// -----------------------------------------------------------------------------
//  Lateral dynamics: one integrator for everybody's line
// -----------------------------------------------------------------------------
void Traffic::lateralDynamics(f64 dt) {
    for (Vehicle& veh : m_vehicles) {
        const ArchetypeSpec& spec = archetypeSpec(veh.kind);
        const SimEdge& e = m_edges[veh.edge];
        const f64 leanCap = m_boids.lateralMaxG * 9.81;
        const f64 cap = spec.canTip ? std::min(leanCap, spec.tipLateralG * 9.81) : leanCap;
        f64 a = 0.0;
        if (spec.twoWheeler) {
            // The Boids field commands a lateral acceleration every 200 ms;
            // between updates the lean is simply held, as a rider does.
            a = veh.latAccelCommand;
            veh.lateralVel *= 0.9;                       // settling
            veh.lateralVel += a * dt;
        } else {
            const f64 target = laneOffset(veh.lane, e.lanes);
            const f64 err = target - veh.lateral;
            a = kLateralKp * err - kLateralKd * veh.lateralVel;
            if (std::fabs(err) > 0.05) {
                if (std::fabs(a) > cap && !veh.lateralCapped) {
                    // A vehicle that would have to lean past its own rollover
                    // limit to complete the manoeuvre takes the manoeuvre slowly.
                    veh.lateralCapped = true;
                    ++m_tipLimits;
                }
            }
            veh.lateralVel += std::clamp(a, -cap, cap) * dt;
        }
        const f64 aEff = std::clamp(a, -cap, cap);
        m_maxLateralG = std::max(m_maxLateralG, std::fabs(aEff) / 9.81);
        veh.lateral += veh.lateralVel * dt;
        if (!spec.twoWheeler && std::fabs(veh.latAccelCommand) > 0.0) veh.latAccelCommand = 0.0;
        const f64 half = e.lanes * kLaneWidth * 0.5 - 0.2;
        veh.lateral = std::clamp(veh.lateral, -half, half);
    }
}

// -----------------------------------------------------------------------------
//  Two-wheeler flow field (Boids: separation strong, alignment present,
//  cohesion ~ 0) on a 200 ms update with a 20 m broadphase
// -----------------------------------------------------------------------------
void Traffic::twoWheelerField(f64 dt) {
    m_boidsAccum += dt;
    if (m_boidsAccum < m_boids.updateSeconds) return;
    m_boidsAccum = 0.0;

    // Broadphase: 16 x 20 m buckets per edge, reused every update so that the
    // field never allocates while the city is running.
    for (std::vector<u32>& b : m_buckets) b.clear();
    for (u32 i = 0; i < m_vehicles.size(); ++i) {
        const Vehicle& v = m_vehicles[i];
        const u32 cell = static_cast<u32>(std::max<f64>(0.0, v.s) / m_boids.broadphaseMetres);
        m_buckets[v.edge * kBucketsPerEdge + std::min(cell, kBucketsPerEdge - 1)].push_back(i);
    }

    for (u32 i = 0; i < m_vehicles.size(); ++i) {
        Vehicle& veh = m_vehicles[i];
        const ArchetypeSpec& spec = archetypeSpec(veh.kind);
        if (!spec.twoWheeler) continue;
        const SimEdge& e = m_edges[veh.edge];
        const u32 home = std::min(static_cast<u32>(std::max<f64>(0.0, veh.s) / m_boids.broadphaseMetres),
                                  kBucketsPerEdge - 1);

        f64 sepLateral = 0.0, align = 0.0, cohesion = 0.0;
        u32 n = 0;
        for (i32 c = -1; c <= 1; ++c) {
            const i64 cell = static_cast<i64>(home) + c;
            if (cell < 0 || cell >= static_cast<i64>(kBucketsPerEdge)) continue;
            for (u32 j : m_buckets[veh.edge * kBucketsPerEdge + static_cast<u32>(cell)]) {
                if (j == i) continue;
                const Vehicle& other = m_vehicles[j];
                const f64 ds = other.s - veh.s;
                const f64 dl = other.lateral - veh.lateral;
                const f64 d2 = ds * ds + dl * dl;
                if (d2 > m_boids.broadphaseMetres * m_boids.broadphaseMetres) continue;
                // Separation is about bodies riding side by side: it applies
                // when the two machines overlap lengthwise, and it pushes them
                // apart laterally.  A machine 40 m up the road is not a
                // separation problem, however close its lane is.
                const f64 lonReach = 0.5 * (veh.length + other.length) + 1.5;
                if (std::fabs(ds) < lonReach) {
                    const f64 want = 0.5 * (spec.width + archetypeSpec(other.kind).width) + 0.9;
                    if (std::fabs(dl) < want) {
                        sepLateral += -((dl >= 0.0) ? 1.0 : -1.0) * ((want - std::fabs(dl)) / want);
                        ++m_separationEvents;
                    }
                }
                align += other.v;
                cohesion += dl;
                ++n;
            }
        }

        const f64 limit = std::max(4.0, static_cast<f64>(e.limit) / 3.6);
        f64 steer = 0.0;
        if (n > 0) {
            steer = m_boids.separation * (sepLateral / static_cast<f64>(n)) +
                    m_boids.alignment * ((align / static_cast<f64>(n)) - veh.v) / limit * 0.5 +
                    m_boids.cohesion * (cohesion / static_cast<f64>(n));
        }
        // Hold them inside the carriageway: a two-wheeler may use the whole
        // width of the road, and no more.
        const f64 half = e.lanes * kLaneWidth * 0.5 - 0.7;
        if (std::fabs(veh.lateral) > half)
            steer += (veh.lateral > 0.0 ? -1.0 : 1.0) * m_boids.boundary * 0.6;
        // A derivative term: without it the field oscillates around its wanted
        // separation instead of settling on a line.
        veh.latAccelCommand = steer * 6.0 - veh.lateralVel * 0.6;
        ++m_boidsUpdates;
    }
}

// -----------------------------------------------------------------------------
//  Fares: a kaali-peeli stops when hailed
// -----------------------------------------------------------------------------
void Traffic::faresAndQueues(f64 dt) {
    for (Vehicle& veh : m_vehicles) {
        const ArchetypeSpec& spec = archetypeSpec(veh.kind);
        if (!spec.canHail) continue;
        const SimEdge& e = m_edges[veh.edge];
        if (veh.hailing) {
            veh.hailTimer -= dt;
            // Pulled in to the kerb.  Mumbai drives on the left, so the kerb is
            // to the left of the direction of travel.
            veh.lateral = std::max(-(e.lanes * kLaneWidth * 0.5 - 0.6),
                                   laneOffset(veh.lane, e.lanes) - 0.8);
            if (veh.hailTimer <= 0.0) {
                veh.hailing = false;
                veh.hailTimer = 60.0 + rng01() * 540.0;    // cool-down
            }
            continue;
        }
        if (veh.hailTimer > 0.0) { veh.hailTimer -= dt; continue; }
        // On average one hail per taxi per ten simulated minutes: 1/600 per
        // second, drawn per tick at dt/600.
        if (rng(1000000u) < static_cast<u32>(dt * 1000000.0 / 600.0)) {
            veh.hailing = true;
            veh.hailTimer = 6.0 + rng01() * 8.0;
            ++m_hailStops;
        }
    }
}

// -----------------------------------------------------------------------------
//  Leave edges, queue at junctions, and re-enter
// -----------------------------------------------------------------------------
void Traffic::advanceEdges() {
    // Phase A: take out everything that finished its edge.  Erasing from the lane
    // lists only after the vehicle is recorded keeps the lists consistent while
    // they are being walked.
    m_arrived.clear();
    for (u32 edgeIndex : m_activeEdges) {
        SimEdge& e = m_edges[edgeIndex];
        for (u16 l = 0; l < e.lanes; ++l) {
            std::vector<u32>& list = e.laneVehicles[l];
            for (size_t k = 0; k < list.size();) {
                if (m_vehicles[list[k]].s < e.length) { ++k; continue; }
                m_arrived.push_back(Arrival{list[k], edgeIndex, l});
                list.erase(list.begin() + static_cast<long>(k));
            }
        }
    }

    // Phase B: send each arrival somewhere.  A junction that cannot accept it
    // holds it at the end of its edge — which is what a blocked junction does to
    // a city.
    for (const Arrival& a : m_arrived) {
        Vehicle& veh = m_vehicles[a.idx];
        const SimEdge& from = m_edges[a.edge];
        SimEdge& src = m_edges[a.edge];
        ++m_edgeCompletions;
        if (m_signalOfEdge[a.edge] != kNoSignal) ++m_signalCrossings;
        const SimNode& node = m_nodes[from.to];

        if (node.out.empty()) {
            // A dead end: a cul-de-sac, or the end of a way whose continuation the
            // source never joined to it.  Hold at the end of the edge, unless the
            // far end of the same edge is clear, in which case the arrival
            // re-enters there — the corridor keeps circulating instead of
            // accumulating, which is a population device and is documented as one
            // (see DOCS/D-029).  A held vehicle is a wall, and walls queue.
            bool clear = true;
            for (u32 other : src.laneVehicles[a.lane])
                if (m_vehicles[other].s < veh.length + 3.0) { clear = false; break; }
            if (clear) {
                veh.s = kMinGap;
                veh.v = std::min(veh.v, 6.0);
                veh.obstruction = 1e18;
                veh.lateral = laneOffset(veh.lane, src.lanes);
                if (archetypeSpec(veh.kind).twoWheeler) veh.lateral += (rng01() - 0.5) * 2.0;
                insertSorted(src.laneVehicles[a.lane], a.idx);
            } else {
                placeHeld(veh, src, a.lane, from.length);
                ++m_deadEndHolds;
                insertSorted(src.laneVehicles[a.lane], a.idx);
            }
            continue;
        }

        // Demand-fixed population: a vehicle that leaves the dense corridor is
        // replaced by an arrival at its far end, which holds the fleet without
        // inventing an origin-destination matrix.
        if (!m_denseEdges.empty() &&
            (node.p - m_denseAnchor).lengthSq() > m_denseRadius * m_denseRadius) {
            // A departure that is beyond the disc is traffic that has left the
            // camera's neighbourhood: it is replaced by an arrival inside it.  A
            // failure is counted — a population device that cannot place a
            // departure has to say so rather than quietly change its scenario.
            if (recycle(a.idx, false)) continue;
            ++m_recycleFailures;
        }

        // Junction choice: round-robin over the arms, refusing the reversal of
        // the edge just travelled while any other arm exists — a U-turn in the
        // middle of a block is not something Mumbai traffic does.  At a dead end
        // the reversal is the only exit there is, and there it is taken.
        const u32 rev = m_edges[a.edge].reverse;
        const size_t outs = node.out.size();
        u32 next = node.out[m_turn % outs];
        for (size_t k = 0; k < outs; ++k) {
            const u32 cand = node.out[(m_turn + k) % outs];
            if (cand != rev || outs == 1) {
                next = cand;
                if (cand == rev) ++m_deadEndUTurns;
                m_turn += k + 1;
                break;
            }
        }
        SimEdge& ne = m_edges[next];
        const u16 lane = static_cast<u16>(std::min<u32>(a.lane, ne.lanes - 1));
        const f64 entry = std::max(0.0, veh.s - from.length);
        if (!laneEntryClear(ne, lane, entry, veh.length, archetypeSpec(veh.kind).width)) {
            placeHeld(veh, src, a.lane, from.length);  // hold at the junction
            veh.v = std::min(veh.v, 1.0);
            ++m_junctionHolds;
            insertSorted(src.laneVehicles[a.lane], a.idx);
            continue;
        }
        veh.edge = next;
        veh.lane = lane;
        veh.lateral = laneOffset(lane, ne.lanes);
        veh.lateralVel = 0.0;
        veh.latAccelCommand = 0.0;
        if (archetypeSpec(veh.kind).twoWheeler) {
            const f64 half = ne.lanes * kLaneWidth * 0.5 - 0.5;
            veh.lateral = std::clamp(veh.lateral + (rng01() - 0.5) * 2.0, -half, half);
        }
        veh.s = entry;
        insertSorted(ne.laneVehicles[lane], a.idx);
    }
}

// -----------------------------------------------------------------------------
//  Nodes, jointed by position
// -----------------------------------------------------------------------------
// A junction whose arms lie in two neighbouring chunks is two nodes in two
// files: the chunk format names nodes by an index local to the file, so no
// record can point across a boundary.  Merging on world position closes that
// seam — the crossing points are identical to well inside the millimetre the
// local float coordinates can express at this extent — and the probe of the
// neighbouring quanta absorbs a point that lands exactly on a quantum edge.
u32 Traffic::nodeAt(const Vec2d& p, bool* created) {
    if (created) *created = false;
    constexpr f64 kQuantum = 0.10;      // m
    const auto keyOf = [](i64 x, i64 z) {
        return (static_cast<u64>(static_cast<u32>(static_cast<i32>(x))) << 32) |
               static_cast<u32>(static_cast<i32>(z));
    };
    const i64 x = static_cast<i64>(std::llround(p.x / kQuantum));
    const i64 z = static_cast<i64>(std::llround(p.y / kQuantum));
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
            const auto it = m_nodeAt.find(keyOf(x + dx, z + dz));
            if (it != m_nodeAt.end()) return it->second;
        }
    }
    const u32 idx = static_cast<u32>(m_nodes.size());
    m_nodes.push_back(SimNode{p, {}});
    m_nodeAt.emplace(keyOf(x, z), idx);
    if (created) *created = true;
    return idx;
}

// -----------------------------------------------------------------------------
//  Signals the chunk format could not express
// -----------------------------------------------------------------------------
// Two things are missing from a baked phase set, and neither is the pipeline's
// fault: (1) the opposing approach of a two-way segment, which the format stores
// as a direction flag rather than as a second record; (2) a junction whose arms
// live in two chunks, which the pipeline sees as two separate nodes and therefore
// never classifies as a junction at all.  This pass supplies both, using the same
// warrant and the same plan constants the pipeline uses (engine/world/
// signal_warrant.h), so the two cannot drift apart.
void Traffic::addDerivedPhase(u32 edge, const Vec2d& axis) {
    SignalPhase p;
    p.edge = edge;
    p.greenStart = navsignal::armRunsAlong(m_edges[edge].dir, axis)
                       ? 0.0
                       : 0.1 * static_cast<f64>(navsignal::kCrossOffsetDeciseconds);
    p.greenSeconds = 0.1 * static_cast<f64>(navsignal::kGreenDeciseconds);
    m_signals.push_back(p);
    m_signalOfEdge[edge] = static_cast<u32>(m_signals.size() - 1);
    ++m_derivedSignals;
}

void Traffic::deriveSignals() {
    if (m_edges.empty() || m_nodes.empty()) return;
    std::vector<std::vector<u32>> incoming(m_nodes.size());
    for (u32 i = 0; i < m_edges.size(); ++i) {
        const u32 to = m_edges[i].to;
        if (to < incoming.size()) incoming[to].push_back(i);
    }
    for (u32 n = 0; n < m_nodes.size(); ++n) {
        const std::vector<u32>& arms = incoming[n];
        // One approach per arm.  Fewer than three approaches is not a junction:
        // a road that passes through has two, one in each direction.
        if (arms.size() < 3) continue;
        u32 baked = 0, missing = 0;
        for (u32 e : arms) (m_signalOfEdge[e] == kNoSignal ? ++missing : ++baked);
        if (baked != 0) {
            if (missing == 0) continue;
            // The pipeline decided this junction.  Complete the plan for the arms
            // the format could not name, taking the axis from the arms it did.
            Vec2d axis{0, 0};
            for (u32 e : arms) {
                const u32 si = m_signalOfEdge[e];
                if (si == kNoSignal) continue;
                if (m_signals[si].greenStart < 0.5) { axis = m_edges[e].dir; break; }
            }
            if (axis.lengthSq() < 1e-9) axis = m_edges[arms[0]].dir;
            for (u32 e : arms)
                if (m_signalOfEdge[e] == kNoSignal) addDerivedPhase(e, axis);
            continue;
        }
        navsignal::JunctionShape shape;
        shape.degree = static_cast<u32>(arms.size());
        Vec2d axis{0, 0};
        for (u32 e : arms) {
            const SimEdge& se = m_edges[e];
            if (se.cls >= 1 && se.cls <= 4) ++shape.major;
            else if (se.cls == 5 || se.cls == 6) ++shape.tertiary;
            if (se.lanes >= 4) ++shape.wide;
            if (axis.lengthSq() == 0.0 && se.cls >= 1 && se.cls <= 6 && se.dir.lengthSq() > 1e-9)
                axis = se.dir;
        }
        if (!navsignal::signalWarranted(shape) || axis.lengthSq() < 1e-9) continue;
        for (u32 e : arms) addDerivedPhase(e, axis);
    }
}

void Traffic::removeFromLane(u32 edge, u16 lane, u32 idx) {
    std::vector<u32>& list = m_edges[edge].laneVehicles[lane];
    for (size_t k = 0; k < list.size(); ++k) {
        if (list[k] != idx) continue;
        list.erase(list.begin() + static_cast<long>(k));
        return;
    }
}

bool Traffic::recycle(u32 idx, bool preferAhead) {
    if (m_denseEdges.empty()) return false;
    Vehicle& veh = m_vehicles[idx];
    const std::vector<u32>* rings[2] = {&m_denseEdges, &m_denseRing};
    for (int ring = 0; ring < 2; ++ring) {
        const std::vector<u32>& cands = *rings[ring];
        if (cands.empty()) continue;
        const size_t tries = std::min<size_t>(ring == 0 ? 256 : 48, cands.size());
        const f64 lim = ring == 0 ? m_denseRadius : m_denseRadius * 4.0;
        // Two passes when the caller wants the arrival put ahead of the camera:
        // behind is where the disc is sweeping traffic out of, so a vehicle placed
        // there is swept out again immediately.  The second pass relaxes that, and
        // exists so that a disc with no road ahead of the camera still fills.
        const int passes = (preferAhead && m_denseHasDir) ? 2 : 1;
        for (int pass = 0; pass < passes; ++pass) {
            for (size_t t = 0; t < tries; ++t) {
                const size_t pick = static_cast<size_t>(m_turn) + t;
                const u32 reentry = cands[pick % cands.size()];
                SimEdge& re = m_edges[reentry];
                const f64 spare = re.length - veh.length - 3.0;
                if (spare < 0.5) continue;
                const u16 lane = static_cast<u16>(pick % re.lanes);
                // Entrances are spread along the edge instead of stacked at its
                // mouth: a long bridge deck holds a whole population if they are
                // not all asked to stand in the same sixteen metres.
                const f64 cand = kLaneWidth * 0.5 + static_cast<f64>(pick % 48) * 8.0;
                if (cand > spare) continue;
                // The point, not the edge, has to lie inside the disc.  An edge
                // whose midpoint is a kilometre away can still cross the disc, and
                // a long edge that begins inside it can leave: the criterion counts
                // vehicles around the camera, not edges.
                const Vec2d entry = re.a + re.dir * cand;
                const Vec2d off = entry - m_denseAnchor;
                if (off.lengthSq() > lim * lim) continue;
                if (pass == 0 &&
                    off.x * m_denseDir.x + off.y * m_denseDir.y < -0.25 * m_denseRadius)
                    continue;
                if (!laneEntryClear(re, lane, cand, veh.length,
                                    archetypeSpec(veh.kind).width))
                    continue;
                veh.edge = reentry;
                veh.lane = lane;
                veh.lateral = laneOffset(lane, re.lanes);
                veh.lateralVel = 0.0;
                veh.latAccelCommand = 0.0;
                veh.s = cand;
                veh.v = std::min(veh.v, 8.0);
                veh.obstruction = 1e18;
                veh.atRedLine = false;
                veh.redStopped = false;
                veh.changedLaneThisStep = false;
                insertSorted(re.laneVehicles[lane], idx);
                m_turn += t + 1;
                return true;
            }
        }
    }
    return false;
}

void Traffic::maintainDensePopulation(f64 dt) {
    if (m_denseEdges.empty()) return;
    // The camera drives at ~86 km/h and the fleet at city speed, so traffic is
    // continuously swept out of the disc from behind.  A vehicle is called back as
    // soon as it is clear of the disc rather than when it happens to finish an
    // edge — which is the difference between a population that follows the camera
    // and one that trails it.
    const f64 lim = m_denseRadius + kTrailGrace;
    u32 budget = kRecyclePerTick;   // a tick's worth of teleports, not the whole queue
    for (u32 i = 0; i < m_vehicles.size() && budget; ++i) {
        Vehicle& veh = m_vehicles[i];
        if (veh.recycleIn > 0.0) { veh.recycleIn -= dt; continue; }
        if ((positionOf(veh) - m_denseAnchor).lengthSq() <= lim * lim) continue;
        --budget;
        const u32 oldEdge = veh.edge;
        const u16 oldLane = veh.lane;
        // Leave the lane before asking for a new one.  A vehicle that is moved
        // onto the edge it already occupies — a long edge that crosses the disc —
        // must not end up listed twice, and a vehicle that cannot be placed at all
        // must go back exactly where it was.
        removeFromLane(oldEdge, oldLane, i);
        if (!recycle(i, true)) {
            // The device is full — every candidate in the disc and in the ring
            // around it refused the vehicle.  Asking again for the rest of the
            // queue this tick cannot succeed either, and is the most expensive
            // thing the tick can do, so the sweep stops here and the queue waits.
            ++m_recycleFailures;
            veh.recycleIn = 1.0;
            insertSorted(m_edges[oldEdge].laneVehicles[oldLane], i);
            break;
        }
    }
}

void Traffic::setDenseAnchor(const Vec2d& anchor, f64 radius) {
    // The camera's own direction: what "ahead" means when a recycled vehicle is
    // placed, and the only sense in which the population device has a flow.
    const Vec2d moved = anchor - m_denseLast;
    if (moved.lengthSq() > 25.0) {
        const Vec2d d = moved.normalized();
        if (!m_denseHasDir || m_denseDir.x * d.x + m_denseDir.y * d.y < 0.0) {
            m_denseDir = d;
            m_denseHasDir = true;
        } else {
            const Vec2d blended{m_denseDir.x * 0.7 + d.x * 0.3, m_denseDir.y * 0.7 + d.y * 0.3};
            if (blended.lengthSq() > 1e-9) m_denseDir = blended.normalized();
        }
        m_denseLast = anchor;
    }
    m_denseAnchor = anchor;
    m_denseRadius = radius;
    m_denseEdges.clear();
    m_denseRing.clear();
    const f64 ring = radius * 4.0;
    const f64 inner2 = radius * radius, ring2 = ring * ring;
    for (u32 i = 0; i < m_edges.size(); ++i) {
        const Vec2d mid = (m_edges[i].a + m_edges[i].b) * 0.5;
        const f64 d2 = (mid - anchor).lengthSq();
        if (d2 <= inner2) m_denseEdges.push_back(i);
        else if (d2 <= ring2) m_denseRing.push_back(i);
    }
    // A disc that contains no road cannot hold a population: over open water it
    // holds none, and on a bridge deck a handful of long edges.  When there is
    // nothing inside it, the ring stands in for it; when there is nothing at all,
    // the whole loaded corridor does, so that a departure is never lost.
    if (m_denseEdges.empty()) m_denseEdges = m_denseRing;
    if (m_denseEdges.empty())
        for (u32 i = 0; i < m_edges.size(); ++i) m_denseEdges.push_back(i);
}

// -----------------------------------------------------------------------------
//  Public step
// -----------------------------------------------------------------------------
void Traffic::step(f64 dt) {
    collectActiveEdges();
    leadersAndSignals();
    longitudinal(dt);
    laneChanges(dt);
    twoWheelerField(dt);
    lateralDynamics(dt);
    // Kerb pull-ins and hail timers last, because a taxi pulling to the kerb
    // writes the same lateral the tracker just settled — and then the corrections
    // run over whatever line everybody ended up taking.
    faresAndQueues(dt);
    corrections();
    // A stop at the line is a state change, counted once: the tick the vehicle
    // actually comes to rest while the aspect is against it.
    for (Vehicle& veh : m_vehicles) {
        if (veh.atRedLine && veh.v < 0.35) {
            if (!veh.redStopped) { ++m_signalStops; veh.redStopped = true; }
        } else if (!veh.atRedLine) {
            veh.redStopped = false;
        }
    }
    advanceEdges();
    maintainDensePopulation(dt);
    m_time += dt;
}

void Traffic::audit(f64 minX, f64 maxX, f64 minZ, f64 maxZ) {
    // Structural invariants, checked against the world rather than trusted: no
    // two vehicles in one lane may sit closer than the floor, and every vehicle
    // must be inside the world's extent.
    for (const SimEdge& e : m_edges) {
        for (u16 l = 0; l < e.lanes; ++l) {
            const std::vector<u32>& list = e.laneVehicles[l];
            for (size_t k = 1; k < list.size(); ++k) {
                const Vehicle& behind = m_vehicles[list[k - 1]];
                const Vehicle& ahead = m_vehicles[list[k]];
                // Lane registration is bookkeeping, lateral footprint is physics —
                // the same rule the model itself uses.  Two riders abreast are not
                // in each other's way however the list happens to be ordered; two
                // bodies whose footprints overlap are, and then there is no gap to
                // speak of.
                if (std::fabs(ahead.lateral - behind.lateral) >
                    0.5 * (archetypeSpec(ahead.kind).width + archetypeSpec(behind.kind).width))
                    continue;
                if (ahead.s < behind.s) {
                    ++m_laneViolations;
#if defined(MUMBAI_TRAFFIC_DIAG)
                    std::fprintf(stderr,
                                 "  order: edge %zu lane %u  idx %u s %.3f (%s len %.2f lat %.3f) "
                                 "listed before idx %u s %.3f (%s len %.2f lat %.3f)  t %.3f\n",
                                 static_cast<size_t>(&e - m_edges.data()), l, list[k], ahead.s,
                                 archetypeSpec(ahead.kind).name, ahead.length, ahead.lateral,
                                 list[k - 1], behind.s, archetypeSpec(behind.kind).name,
                                 behind.length, behind.lateral, m_time);
#endif
                    continue;
                }
                if (ahead.s - ahead.length - behind.s < kMinGap - 1e-6) {
                    ++m_laneViolations;
#if defined(MUMBAI_TRAFFIC_DIAG)
                    std::fprintf(stderr,
                                 "  gap:   edge %zu lane %u  idx %u %s s %.3f len %.2f lat %.3f "
                                 "(front %.3f)  ahead of idx %u %s s %.3f lat %.3f  gap %.4f  t %.3f\n",
                                 static_cast<size_t>(&e - m_edges.data()), l, list[k],
                                 archetypeSpec(ahead.kind).name, ahead.s, ahead.length, ahead.lateral,
                                 ahead.s - ahead.length, list[k - 1], archetypeSpec(behind.kind).name,
                                 behind.s, behind.lateral, ahead.s - ahead.length - behind.s, m_time);
#endif
                }
            }
        }
    }
    for (const Vehicle& veh : m_vehicles) {
        if (!std::isfinite(veh.s) || !std::isfinite(veh.v) || !std::isfinite(veh.lateral)) {
            ++m_badState;
            continue;
        }
        const ArchetypeSpec& spec = archetypeSpec(veh.kind);
        if (veh.v < -0.01 || veh.v > spec.topSpeedKph / 3.6 * 1.4 + 0.5) { ++m_badSpeed; continue; }
        const Vec2d p = positionOf(veh);
        if (p.x < minX - 1 || p.x > maxX + 1 || p.y < minZ - 1 || p.y > maxZ + 1) ++m_badPosition;
    }
}

TrafficMetrics Traffic::metrics(const Vec2d& camera, f64 radius) const {
    TrafficMetrics m;
    m.vehicles = m_vehicles.size();
    m.edges = m_edges.size();
    m.nodes = m_nodes.size();
    m.signals = m_signals.size();
    for (const SimEdge& e : m_edges) m.lanesTotal += e.lanes;
    f64 sum = 0.0;
    size_t crawl = 0;
    const f64 r2 = radius * radius;
    for (const Vehicle& v : m_vehicles) {
        sum += v.v;
        if (v.v < 1.4) ++crawl;
        ++m.archetypeCount[static_cast<int>(v.kind)];
        if ((positionOf(v) - camera).lengthSq() <= r2) ++m.nearCamera;
        if (!m_denseEdges.empty() &&
            (positionOf(v) - m_denseAnchor).lengthSq() >
                (m_denseRadius + kTrailGrace) * (m_denseRadius + kTrailGrace))
            ++m.stranded;
    }
    m.meanSpeedKph = m_vehicles.empty() ? 0.0 : sum / m_vehicles.size() * 3.6;
    m.crawlShare = m_vehicles.empty() ? 0.0 : static_cast<f64>(crawl) / m_vehicles.size();
    m.minGap = m_minGap;
    m.deadEndHolds = m_deadEndHolds;
    m.laneChanges = m_laneChanges;
    m.laneChangeRefusals = m_laneChangeRefusals;
    m.edgeCompletions = m_edgeCompletions;
    m.signalStops = m_signalStops;
    m.junctionHolds = m_junctionHolds;
    m.holdCaps = m_holdCaps;
    m.signalCrossings = m_signalCrossings;
    m.recycleFailures = m_recycleFailures;
    m.boidsUpdates = m_boidsUpdates;
    m.separationEvents = m_separationEvents;
    m.tipLimits = m_tipLimits;
    m.hailStops = m_hailStops;
    m.hardBrakes = m_hardBrakes;
    m.badState = m_badState;
    m.badSpeed = m_badSpeed;
    m.badPosition = m_badPosition;
    m.laneViolations = m_laneViolations;
    m.maxLateralG = m_maxLateralG;
    m.nodesMerged = m_nodesMerged + m_nodesMergedAcrossChunks;
    m.nodesMergedAcrossChunks = m_nodesMergedAcrossChunks;
    m.deadEndUTurns = m_deadEndUTurns;
    m.derivedSignals = static_cast<size_t>(m_derivedSignals);
    m.reverseEdges = 0;
    for (u32 i = 0; i < m_edges.size(); ++i)
        if (m_edges[i].reverse != kNoEdge && m_edges[i].reverse > i) ++m.reverseEdges;
    return m;
}

}  // namespace mumbai::sim
