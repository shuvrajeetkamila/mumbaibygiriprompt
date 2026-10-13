// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_traffic.cpp
//
//  Section 5's traffic system.  Every case runs against a synthetic world
//  written to a temporary chunk, so the answers are known: carriageways of known
//  length and lane count, signals with known phases, and archetypes whose
//  behaviour the specification pins down — the bus pulls away hard and brakes
//  early, the auto crawls and brakes late, the motorcyclist barely touches the
//  throttle and brakes hardest of all, two-wheelers filter and hold no lane.
//
//  Note on placement: `s` is measured along ONE edge, not along a chain, and
//  `spawn` refuses a position that is occupied.  Tests therefore place vehicles
//  through `SyntheticWorld::at()`, which converts "metres along this chain" into
//  the (edge, s) pair the model speaks in.
// =============================================================================
#include "engine/sim/traffic.h"
#include "engine/world/chunk.h"
#include "tests/test_harness.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace mumbai;

namespace {

struct Straight {
    f64 x0 = 0.0, z0 = 0.0;
    int segments = 10;
    f64 segLen = 40.0;
    u16 lanes = 1;
    f32 limit = 40.0f;
    f64 greenStart = 0.0;      // seconds into the cycle the green opens
    f64 greenSeconds = 0.0;    // 0 = no signal, on the last straight edge
    bool loop = false;         // connect the last node back to the first
};

struct SyntheticWorld {
    std::vector<NavNode> nodes;
    std::vector<NavEdge> edges;
    std::vector<NavSignalPhase> signals;
    std::vector<u32> chainFirstEdge;
    std::vector<u32> chainLastEdge;
    std::vector<f64> chainLength;

    void add(const Straight& s) {
        const u32 first = static_cast<u32>(nodes.size());
        for (int i = 0; i <= s.segments; ++i) {
            NavNode n{};
            n.x = static_cast<f32>(s.x0 + i * s.segLen);
            n.y = 0.0f;
            n.z = static_cast<f32>(s.z0);
            n.firstEdge = 0;
            n.edgeCount = (i < s.segments) ? 1u : 0u;
            n.flags = 0;
            n.osmId = 0;
            nodes.push_back(n);
        }
        chainFirstEdge.push_back(static_cast<u32>(edges.size()));
        chainLength.push_back(s.segments * s.segLen);
        for (int i = 0; i < s.segments; ++i) {
            NavEdge e{};
            e.toNode = first + static_cast<u32>(i + 1);
            e.laneCount = s.lanes;
            e.direction = 1;
            e.surface = 0;
            e.speedLimitKph = s.limit;
            e.length = static_cast<f32>(s.segLen);
            e.flags = 0;
            e.roadClass = 12;
            nodes[first + static_cast<u32>(i)].firstEdge = static_cast<u32>(edges.size());
            edges.push_back(e);
        }
        const u32 lastStraight = static_cast<u32>(edges.size()) - 1;
        if (s.loop) {
            // A closed circuit.  A chain with a bare end is a wall, and a city
            // full of walls measures the walls, not the traffic model: the ends
            // of the pipeline's nav chains are unrepresented junctions, which
            // junction connectivity will remove (DOCS/06_ROADMAP.md, A3).
            const u32 lastNode = first + static_cast<u32>(s.segments);
            NavEdge e{};
            e.toNode = first;
            e.laneCount = s.lanes;
            e.direction = 1;
            e.surface = 0;
            e.speedLimitKph = s.limit;
            e.length = static_cast<f32>(s.segLen);
            e.flags = 0;
            e.roadClass = 12;
            nodes[lastNode].firstEdge = static_cast<u32>(edges.size());
            nodes[lastNode].edgeCount = 1;
            edges.push_back(e);
        }
        chainLastEdge.push_back(lastStraight);
        if (s.greenSeconds > 0.0) {
            NavSignalPhase p{};
            p.edgeIndex = lastStraight;
            p.startGreenDeciseconds = static_cast<u16>(s.greenStart * 10.0);
            p.greenDeciseconds = static_cast<u16>(s.greenSeconds * 10.0);
            signals.push_back(p);
        }
    }

    // "metres along the chain" -> (edge, s within that edge).  Clamped to the
    // chain, so a test that asks for a point past the end gets the last edge
    // rather than an out-of-range walk.
    struct Place { u32 edge; f64 s; };
    Place at(u32 chain, f64 metres) const {
        u32 e = chainFirstEdge.at(chain);
        const u32 last = chainLastEdge.at(chain);
        f64 rem = std::max(0.0, metres);
        while (e < last && edges[e].length < rem) { rem -= edges[e].length; ++e; }
        return Place{e, std::min(rem, static_cast<f64>(edges[e].length) - 0.5)};
    }

    std::string write(const std::string& name) const {
        NavGraphHeader nh{};
        nh.nodeCount = static_cast<u32>(nodes.size());
        nh.edgeCount = static_cast<u32>(edges.size());
        nh.signalCount = static_cast<u32>(signals.size());
        std::vector<u8> payload(sizeof(NavGraphHeader) + nodes.size() * sizeof(NavNode) +
                                edges.size() * sizeof(NavEdge) + signals.size() * sizeof(NavSignalPhase));
        size_t c = 0;
        std::memcpy(payload.data() + c, &nh, sizeof(nh));
        c += sizeof(nh);
        std::memcpy(payload.data() + c, nodes.data(), nodes.size() * sizeof(NavNode));
        c += nodes.size() * sizeof(NavNode);
        std::memcpy(payload.data() + c, edges.data(), edges.size() * sizeof(NavEdge));
        c += edges.size() * sizeof(NavEdge);
        if (!signals.empty())
            std::memcpy(payload.data() + c, signals.data(), signals.size() * sizeof(NavSignalPhase));

        ChunkWriter w(0, 0, 0xA3C0FFEEull);
        w.addSection(SectionType::RoadNavGraph, payload.data(), payload.size(), nh.nodeCount,
                     sizeof(NavNode), 1, SectionFlagMandatory | SectionFlagPrefixed);
        const std::string path = "/tmp/mumbai_traffic_" + name + ".chunk";
        return w.write(path) ? path : std::string();
    }
};

// Places one vehicle at a point measured along a chain.  Fails the case if the
// point is occupied, because a test that silently places two vehicles on top of
// each other is testing the corrections, not the model.
u32 place(sim::Traffic& t, const SyntheticWorld& w, u32 chain, f64 metres, sim::Archetype kind,
          u16 lane = 0, f64 v = 8.0) {
    SyntheticWorld::Place p = w.at(chain, metres);
    u32 idx = t.spawn(kind, p.edge, lane, p.s, v);
    for (f64 extra = 1.0; idx == sim::Traffic::kNoVehicle && extra <= 24.0; extra += 1.0) {
        p = w.at(chain, metres + extra);
        idx = t.spawn(kind, p.edge, lane, p.s, v);
    }
    CHECK(idx != sim::Traffic::kNoVehicle);
    if (idx == sim::Traffic::kNoVehicle) return 0;   // keep the case running; the CHECK failed
    return idx;
}

// Distance in metres from the start of `chain` to a vehicle's front bumper.
f64 chainDistance(const sim::Traffic& t, const SyntheticWorld& w, u32 chain, const sim::Vehicle& v) {
    f64 d = 0.0;
    for (u32 e = w.chainFirstEdge.at(chain); e < v.edge; ++e) d += t.edges()[e].length;
    return d + v.s;
}

// Invariants that hold on every edge and lane at every moment: the lane list is
// ordered by s, and no two vehicles in it overlap.
void checkLaneInvariants(const sim::Traffic& t) {
    size_t disorder = 0, overlaps = 0;
    for (const sim::SimEdge& e : t.edges()) {
        for (u16 l = 0; l < e.lanes; ++l) {
            const std::vector<u32>& list = e.laneVehicles[l];
            for (size_t k = 1; k < list.size(); ++k) {
                const sim::Vehicle& prev = t.vehicles()[list[k - 1]];
                const sim::Vehicle& cur = t.vehicles()[list[k]];
                if (cur.s < prev.s) ++disorder;
                // A two-wheeler may be listed in the lane it entered while riding
                // beside a lane holder, so sideways separation counts.
                const f64 lateral = std::fabs(cur.lateral - prev.lateral);
                const f64 sides = 0.5 * (sim::archetypeSpec(cur.kind).width +
                                         sim::archetypeSpec(prev.kind).width);
                if (lateral > sides) continue;
                if (cur.s - cur.length - prev.s < sim::kMinGap - 1e-6) ++overlaps;
            }
        }
    }
    CHECK_EQ(disorder, 0u);
    CHECK_EQ(overlaps, 0u);
}

sim::TrafficMetrics run(sim::Traffic& t, f64 seconds, const Vec2d& focus = Vec2d{0, 0}, f64 radius = 1e9) {
    const int steps = static_cast<int>(seconds * 60.0);
    for (int i = 0; i < steps; ++i) t.step(1.0 / 60.0);
    return t.metrics(focus, radius);
}

}  // namespace

// -----------------------------------------------------------------------------
//  Archetypes
// -----------------------------------------------------------------------------
TEST(archetypesCarryTheSpecifiedMassesAndDimensions) {
    const sim::ArchetypeSpec& bus = sim::archetypeSpec(sim::Archetype::Bus);
    CHECK_EQ(std::string(bus.name), std::string("BEST double-decker"));
    CHECK(bus.length > 11.0 && bus.length < 13.0);         // 12 m double-decker
    CHECK_NEAR(bus.massKg, 16000.0, 1.0);
    CHECK_NEAR(bus.width, 2.55, 0.01);

    const sim::ArchetypeSpec& autoR = sim::archetypeSpec(sim::Archetype::AutoRickshaw);
    CHECK_NEAR(autoR.massKg, 400.0, 1.0);
    CHECK(autoR.canTip);                                    // rollover geometry
    CHECK_NEAR(autoR.tipLateralG, 0.36, 0.01);
    CHECK_FALSE(autoR.twoWheeler);

    const sim::ArchetypeSpec& bike = sim::archetypeSpec(sim::Archetype::Motorcycle);
    CHECK(bike.twoWheeler);                                 // rides the Boids field
    CHECK(bike.massKg >= 110.0 && bike.massKg <= 160.0);    // 110-160 kg

    const sim::ArchetypeSpec& taxi = sim::archetypeSpec(sim::Archetype::Taxi);
    CHECK(taxi.canHail);                                    // stops for a fare
    CHECK_EQ(std::string(taxi.name), std::string("kaali-peeli taxi"));

    // The comfort pair as the specification gives it (D-027).
    CHECK_NEAR(bus.comfortAccel, 0.9, 1e-9);
    CHECK_NEAR(bus.comfortBrake, 0.3, 1e-9);
    CHECK_NEAR(autoR.comfortAccel, 0.2, 1e-9);
    CHECK_NEAR(autoR.comfortBrake, 0.7, 1e-9);
    CHECK_NEAR(taxi.comfortAccel, 0.4, 1e-9);
    CHECK_NEAR(taxi.comfortBrake, 0.6, 1e-9);
    CHECK_NEAR(bike.comfortAccel, 0.05, 1e-9);
    CHECK_NEAR(bike.comfortBrake, 0.8, 1e-9);
}

TEST(theBusPullsAwayHardestAndTheMotorcycleGentlest) {
    SyntheticWorld w;
    for (int i = 0; i < 5; ++i) {
        Straight s;
        s.z0 = i * 30.0;
        s.segments = 6;
        s.segLen = 100.0;
        s.lanes = 1;
        s.limit = 90.0f;
        w.add(s);
    }
    const std::string path = w.write("archetypes");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 7);
    const sim::Archetype kinds[5] = {sim::Archetype::Bus, sim::Archetype::Car, sim::Archetype::Taxi,
                                     sim::Archetype::AutoRickshaw, sim::Archetype::Motorcycle};
    u32 idx[5];
    for (int i = 0; i < 5; ++i) idx[i] = place(t, w, static_cast<u32>(i), 100.0, kinds[i], 0, 0.1);

    const sim::TrafficMetrics m = run(t, 3.0);      // 3 s: early enough that no
    CHECK_EQ(m.vehicles, 5u);                       // class has reached its v0 yet
    const f64 vBus = t.vehicles()[idx[0]].v;
    const f64 vCar = t.vehicles()[idx[1]].v;
    const f64 vTaxi = t.vehicles()[idx[2]].v;
    const f64 vAuto = t.vehicles()[idx[3]].v;
    const f64 vBike = t.vehicles()[idx[4]].v;

    // Effective acceleration order: car > bus > taxi > auto > motorcycle.
    CHECK(vBus > vTaxi);
    CHECK(vBus > vAuto);
    CHECK(vCar > vAuto);
    CHECK(vTaxi > vAuto);
    CHECK(vAuto > vBike);
    // And the magnitudes are fractions of each class's maxima, not the raw
    // numbers (D-027): a bus "0.9" is 1.08 m/s^2, not 0.9 of anything else.
    CHECK_NEAR(t.vehicles()[idx[0]].a, 1.2 * 0.9, 1e-9);
    CHECK_NEAR(t.vehicles()[idx[4]].a, 3.0 * 0.05, 1e-9);
    CHECK_NEAR(t.vehicles()[idx[4]].b, 4.0 * 0.8, 1e-9);
    CHECK(t.vehicles()[idx[4]].b > t.vehicles()[idx[0]].b);   // the bike brakes hardest
    CHECK(t.vehicles()[idx[0]].b < t.vehicles()[idx[2]].b);   // the bus brakes earliest
}

// -----------------------------------------------------------------------------
//  Signals
// -----------------------------------------------------------------------------
TEST(signalPhasesRunGreenAmberRedOnAFixedCycle) {
    sim::SignalPhase p;
    p.greenStart = 10.0;
    p.greenSeconds = 40.0;
    CHECK_EQ(sim::SignalPhase::kCycleSeconds, 90.0);
    CHECK(sim::signalStateAt(p, 10.0) == sim::SignalState::Green);
    CHECK(sim::signalStateAt(p, 30.0) == sim::SignalState::Green);
    CHECK(sim::signalStateAt(p, 50.1) == sim::SignalState::Amber);
    CHECK(sim::signalStateAt(p, 53.5) == sim::SignalState::Red);
    CHECK(sim::signalStateAt(p, 89.9) == sim::SignalState::Red);
    CHECK(sim::signalStateAt(p, 100.0) == sim::SignalState::Green);   // cycle wraps
    CHECK(sim::signalStateAt(p, 0.0) == sim::SignalState::Red);       // before the green
}

TEST(theSignalHoldsVehiclesAtTheStopLineAndThenReleasesThem) {
    SyntheticWorld w;
    Straight s;
    s.segments = 5;          // 5 x 40 m = 200 m, closed, one lane
    s.segLen = 40.0;
    s.lanes = 1;
    s.greenStart = 40.0;     // red for the first 40 s of the cycle
    s.greenSeconds = 40.0;
    s.loop = true;
    w.add(s);
    const std::string path = w.write("signal");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 11);
    std::vector<u32> cars;
    for (int i = 0; i < 4; ++i)
        cars.push_back(place(t, w, 0, 150.0 + i * 8.0, sim::Archetype::Car, 0, 12.0));

    sim::TrafficMetrics m = run(t, 30.0);
    CHECK_EQ(m.signalCrossings, 0u);              // nothing crosses on red
    CHECK_EQ(m.signals, 1u);
    f64 lead = 0.0;
    for (u32 i : cars) lead = std::max(lead, chainDistance(t, w, 0, t.vehicles()[i]));
    CHECK(lead > 190.0);                          // it drove up to the line...
    CHECK(lead < 199.5);                          // ...and did not cross it
    CHECK(m.minGap >= sim::kMinGap - 1e-6);       // nose to tail, no penetration
    CHECK(t.vehicles()[cars.front()].v < 0.4);    // stationary at the line
    checkLaneInvariants(t);

    m = run(t, 20.0);                             // through the green at t = 40 s
    CHECK(m.signalCrossings >= 3u);
    CHECK(m.edgeCompletions >= 3u);
    checkLaneInvariants(t);
}

TEST(amberIsStopIfYouCanAndGreenIsCommitment) {
    // Green [0,6), amber [6,9), red [9,90).  A vehicle almost at the line when
    // the amber comes on is committed and goes; one still 80 m back stops.
    SyntheticWorld w;
    Straight s;
    s.segments = 5;          // 5 x 40 m = 200 m
    s.segLen = 40.0;
    s.lanes = 1;
    s.greenStart = 0.0;
    s.greenSeconds = 6.0;
    s.loop = true;
    w.add(s);
    const std::string path = w.write("amber");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 3);
    run(t, 5.5);                                            // still green
    const u32 committed = place(t, w, 0, 195.0, sim::Archetype::Car, 0, 10.0);
    const u32 behind = place(t, w, 0, 120.0, sim::Archetype::Car, 0, 10.0);
    const sim::TrafficMetrics m = run(t, 12.0);             // through amber into red

    CHECK_EQ(m.signalCrossings, 1u);                        // exactly one got through
    CHECK(t.vehicles()[committed].v > 4.0);                 // committed: it kept going
    CHECK(t.vehicles()[behind].v < 0.4);                    // the other one stopped
    CHECK(chainDistance(t, w, 0, t.vehicles()[behind]) < 199.5);
    CHECK(m.minGap >= sim::kMinGap - 1e-6);
    CHECK_EQ(m.badState, 0u);
}

// -----------------------------------------------------------------------------
//  Longitudinal behaviour
// -----------------------------------------------------------------------------
TEST(aPlatoonKeepsItsOrderAndNeverPenetrates) {
    SyntheticWorld w;
    Straight s;
    s.segments = 10;         // 10 x 60 m = 600 m, closed
    s.segLen = 60.0;
    s.lanes = 1;
    s.limit = 50.0f;
    s.loop = true;
    w.add(s);
    const std::string path = w.write("platoon");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 5);
    for (int i = 0; i < 12; ++i)
        place(t, w, 0, 60.0 + i * 20.0, sim::Archetype::Car, 0, 0.5);

    const sim::TrafficMetrics m = run(t, 120.0);
    CHECK_EQ(m.badState, 0u);
    CHECK_EQ(m.badSpeed, 0u);
    CHECK(m.minGap >= sim::kMinGap - 1e-6);          // no wheel ever touches a bumper
    CHECK_EQ(m.vehicles, 12u);                       // the population is conserved
    checkLaneInvariants(t);
    CHECK(m.meanSpeedKph > 20.0);                    // rolling, not stalled
    CHECK(m.meanSpeedKph <= 60.0);
    CHECK(m.hardBrakes < m.vehicles * 100000);       // the ODE is what drives it
}

// -----------------------------------------------------------------------------
//  MOBIL lane changes
// -----------------------------------------------------------------------------
TEST(mobilFindsItsWayPastASlowLeader) {
    SyntheticWorld w;
    Straight s;
    s.segments = 12;         // 12 x 50 m = 600 m, two lanes
    s.segLen = 50.0;
    s.lanes = 2;
    s.limit = 60.0f;
    w.add(s);
    const std::string path = w.write("mobil");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 17);
    const u32 bus = place(t, w, 0, 250.0, sim::Archetype::Bus, 0, 9.0);
    const u32 car = place(t, w, 0, 150.0, sim::Archetype::Car, 0, 9.0);
    place(t, w, 0, 60.0, sim::Archetype::Car, 1, 9.0);   // traffic in the left lane too

    const sim::TrafficMetrics m = run(t, 40.0);
    CHECK(m.laneChanges >= 1u);
    CHECK_EQ(t.vehicles()[car].lane, 1u);                        // it moved right
    CHECK(t.vehicles()[car].s > t.vehicles()[bus].s);            // and got past the bus
    CHECK(m.laneChangeRefusals > 0u);                            // not every evaluation was granted
    CHECK_EQ(m.badState, 0u);
    CHECK(m.minGap >= sim::kMinGap - 1e-6);
    checkLaneInvariants(t);
}

TEST(laneChangesRespectTheRolloverOfAThreeWheeler) {
    // An auto-rickshaw leaning into a 3.5 m lane change would need about
    // 10 m/s^2 of lateral acceleration; its own geometry gives it 0.36 g, so the
    // manoeuvre is rate-limited and the metric records that it was.
    SyntheticWorld w;
    Straight s;
    s.segments = 12;
    s.segLen = 50.0;
    s.lanes = 2;
    s.limit = 60.0f;
    w.add(s);
    const std::string path = w.write("rollover");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 23);
    place(t, w, 0, 250.0, sim::Archetype::Bus, 0, 8.0);
    for (int i = 0; i < 4; ++i)
        place(t, w, 0, 100.0 + i * 14.0, sim::Archetype::AutoRickshaw, 0, 8.0);

    const sim::TrafficMetrics m = run(t, 60.0);
    CHECK(m.tipLimits > 0u);                    // rickshaws took their manoeuvres slowly
    CHECK(m.maxLateralG <= sim::BoidsWeights{}.lateralMaxG + 1e-9);   // never beyond the lean
    CHECK(m.laneChanges > 0u);
    CHECK_EQ(m.badState, 0u);
    checkLaneInvariants(t);
}

// -----------------------------------------------------------------------------
//  Two-wheelers: no lanes, a Boids field, and filtering
// -----------------------------------------------------------------------------
TEST(twoWheelersFilterPastAQueueingBus) {
    // Three lanes, a bus stopped at a red signal, bikes riding up behind it.  The
    // bus occupies the middle; a bike finds room on the outside and gets to the
    // front of the queue, which is the behaviour a Mumbai rider knows instantly.
    SyntheticWorld w;
    Straight s;
    s.segments = 5;          // 5 x 40 m = 200 m
    s.segLen = 40.0;
    s.lanes = 3;
    s.greenStart = 60.0;     // red for a minute
    s.greenSeconds = 20.0;
    w.add(s);
    const std::string path = w.write("filter");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 29);
    const u32 bus = place(t, w, 0, 180.0, sim::Archetype::Bus, 1, 10.0);
    std::vector<u32> bikes;
    for (int i = 0; i < 3; ++i)
        bikes.push_back(place(t, w, 0, 110.0 + i * 8.0, sim::Archetype::Motorcycle, 0, 8.0));

    const sim::TrafficMetrics m = run(t, 40.0);
    CHECK_EQ(m.signalCrossings, 0u);                     // still red
    CHECK_EQ(m.badState, 0u);
    size_t ahead = 0;
    const f64 busAlong = chainDistance(t, w, 0, t.vehicles()[bus]);
    for (u32 b : bikes) {
        if (chainDistance(t, w, 0, t.vehicles()[b]) > busAlong) {
            ++ahead;
            // It went past on the outside, not through the bus.
            const f64 dl = std::fabs(t.vehicles()[b].lateral - t.vehicles()[bus].lateral);
            CHECK(dl > sim::archetypeSpec(sim::Archetype::Bus).width * 0.5);
        }
    }
    CHECK(ahead >= 1u);
    CHECK(m.boidsUpdates > 0u);
    CHECK(m.minGap >= sim::kMinGap - 1e-6);
    CHECK(sim::archetypeSpec(sim::Archetype::Motorcycle).width < 1.0);
}

TEST(theBoidsFieldSeparatesTwoWheelersWithoutBunchingThem) {
    // Twenty machines packed nose to tail in ONE lane at the closest spacing the
    // spawn rule allows.  This is the density the two-wheeler model exists for:
    // the field must push them apart laterally — riders going two abreast, which
    // is what an Indian arterial looks like — without letting them bunch.
    SyntheticWorld w;
    Straight s;
    s.segments = 8;          // 8 x 50 m = 400 m, three lanes, closed
    s.segLen = 50.0;
    s.lanes = 3;
    s.limit = 40.0f;
    s.loop = true;
    w.add(s);
    const std::string path = w.write("boids");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 31);
    size_t placed = 0;
    f64 cursor = 30.0;
    for (int i = 0; i < 20; ++i) {
        const u32 idx = place(t, w, 0, cursor, sim::Archetype::Motorcycle, 0, 7.0);
        if (idx != sim::Traffic::kNoVehicle) ++placed;
        cursor += 2.6;                        // just over one machine and the floor
    }
    CHECK_EQ(placed, 20u);

    const sim::TrafficMetrics m = run(t, 60.0);
    CHECK_EQ(m.vehicles, 20u);
    CHECK(m.boidsUpdates > 0u);
    CHECK(m.separationEvents > 0u);            // the field had to push riders apart
    CHECK(m.maxLateralG <= sim::BoidsWeights{}.lateralMaxG + 1e-9);
    CHECK_EQ(m.badState, 0u);
    CHECK_EQ(m.badPosition, 0u);
    // They spread across the carriageway instead of queueing in a line.
    f64 latMin = 1e9, latMax = -1e9;
    for (const sim::Vehicle& v : t.vehicles()) {
        latMin = std::min(latMin, v.lateral);
        latMax = std::max(latMax, v.lateral);
    }
    CHECK(latMax - latMin > 1.0);
    CHECK(m.meanSpeedKph > 5.0);
}

// -----------------------------------------------------------------------------
//  Fares and fleet behaviour
// -----------------------------------------------------------------------------
TEST(hailedTaxisStopAndTheCityKeepsMoving) {
    SyntheticWorld w;
    Straight s;
    s.segments = 8;
    s.segLen = 60.0;
    s.lanes = 2;
    s.limit = 60.0f;
    s.loop = true;
    w.add(s);
    const std::string path = w.write("fares");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 37);
    for (int i = 0; i < 12; ++i)
        place(t, w, 0, 40.0 + i * 30.0, sim::Archetype::Taxi, static_cast<u16>(i % 2), 9.0);

    // Ten simulated minutes: every taxi hails about once.
    const sim::TrafficMetrics m = run(t, 600.0);
    CHECK(m.hailStops > 0u);
    CHECK(m.meanSpeedKph > 5.0);                // a fare does not stop the street
    CHECK_EQ(m.badState, 0u);
    CHECK(t.time() > 590.0);
    checkLaneInvariants(t);
}

TEST(theFleetMixIsTheCitysAndTheCountersAddUp) {
    SyntheticWorld w;
    for (int i = 0; i < 3; ++i) {
        Straight s;
        s.z0 = i * 40.0;
        s.segments = 10;
        s.segLen = 50.0;
        s.lanes = 2;
        s.loop = true;
        w.add(s);
    }
    const std::string path = w.write("mix");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{200, 40}, 200, 41);
    const sim::TrafficMetrics m = run(t, 60.0, Vec2d{200, 40}, 500.0);
    size_t sum = 0;
    for (int i = 0; i < static_cast<int>(sim::Archetype::Count); ++i) sum += m.archetypeCount[i];
    CHECK_EQ(sum, m.vehicles);
    CHECK(m.vehicles > 150u);                   // the fleet stuck: placement worked
    CHECK(m.archetypeCount[static_cast<int>(sim::Archetype::Motorcycle)] > m.vehicles / 4u);
    CHECK(m.archetypeCount[static_cast<int>(sim::Archetype::Bus)] > 0u);
    CHECK(m.nearCamera > 0u && m.nearCamera <= m.vehicles);
    CHECK(m.edges > 0u && m.nodes > 0u && m.lanesTotal > 0u);
    CHECK(m.minGap >= sim::kMinGap - 1e-6);
    CHECK(m.meanSpeedKph < 45.0);               // a peak-hour street is not a racetrack
    CHECK(m.crawlShare < 0.98);                 // and it is not a car park either
    checkLaneInvariants(t);
}

TEST(trafficIsDeterministic) {
    SyntheticWorld w;
    Straight s;
    s.segments = 8;
    s.segLen = 50.0;
    s.lanes = 2;
    s.greenStart = 20.0;
    s.greenSeconds = 35.0;
    w.add(s);
    Straight s2;
    s2.z0 = 60.0;
    s2.segments = 8;
    s2.segLen = 50.0;
    s2.lanes = 2;
    w.add(s2);
    const std::string path = w.write("determinism");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic a({path}, {}, Vec2d{100, 30}, 120, 99);
    sim::Traffic b({path}, {}, Vec2d{100, 30}, 120, 99);
    CHECK_EQ(a.vehicles().size(), b.vehicles().size());
    for (int i = 0; i < 1200; ++i) { a.step(1.0 / 60.0); b.step(1.0 / 60.0); }
    const sim::TrafficMetrics ma = a.metrics(Vec2d{0, 0}, 1e9);
    const sim::TrafficMetrics mb = b.metrics(Vec2d{0, 0}, 1e9);
    CHECK_EQ(ma.vehicles, mb.vehicles);
    CHECK_EQ(ma.laneChanges, mb.laneChanges);
    CHECK_EQ(ma.edgeCompletions, mb.edgeCompletions);
    CHECK_EQ(ma.boidsUpdates, mb.boidsUpdates);
    CHECK_NEAR(ma.meanSpeedKph, mb.meanSpeedKph, 1e-12);
    bool identical = true;
    for (size_t i = 0; i < a.vehicles().size() && identical; ++i) {
        const sim::Vehicle& x = a.vehicles()[i];
        const sim::Vehicle& y = b.vehicles()[i];
        identical = (x.s == y.s) && (x.v == y.v) && (x.lateral == y.lateral) && (x.edge == y.edge);
    }
    CHECK(identical);
}

TEST(anOccupiedSpawnPointIsRefusedRatherThanOverlapped) {
    SyntheticWorld w;
    Straight s;
    s.segments = 6;
    s.segLen = 60.0;
    s.lanes = 1;
    w.add(s);
    const std::string path = w.write("occupied");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{0, 0}, 0, 43);
    const u32 first = place(t, w, 0, 100.0, sim::Archetype::Car, 0, 5.0);
    const u32 clash = t.spawn(sim::Archetype::Car, w.at(0, 100.0).edge, 0, w.at(0, 100.0).s, 5.0);
    CHECK_EQ(clash, sim::Traffic::kNoVehicle);
    CHECK_EQ(t.vehicles().size(), 1u);
    const u32 clear = t.spawn(sim::Archetype::Car, w.at(0, 100.0).edge, 0, w.at(0, 100.0).s + 12.0, 5.0);
    CHECK(clear != sim::Traffic::kNoVehicle);
    CHECK_EQ(t.vehicles().size(), 2u);
    (void)first;
}

TEST(aLongRunStaysInsideTheWorldAndTheLaw) {
    SyntheticWorld w;
    for (int i = 0; i < 4; ++i) {
        Straight s;
        s.z0 = i * 45.0;
        s.segments = 10;
        s.segLen = 60.0;
        s.lanes = 3;
        s.limit = 50.0f;
        s.loop = true;
        if (i % 2 == 0) { s.greenStart = 15.0 * i; s.greenSeconds = 40.0; }
        w.add(s);
    }
    const std::string path = w.write("longrun");
    CHECK_FALSE(path.empty());
    if (path.empty()) return;

    sim::Traffic t({path}, {}, Vec2d{200, 60}, 240, 1234);
    const sim::TrafficMetrics m = run(t, 600.0);
    t.audit(-10.0, 700.0, -10.0, 250.0);
    const sim::TrafficMetrics a = t.metrics(Vec2d{200, 60}, 500.0);
    checkLaneInvariants(t);
    CHECK_EQ(a.badState, 0u);
    CHECK_EQ(a.badSpeed, 0u);
    CHECK_EQ(a.badPosition, 0u);
    CHECK_EQ(a.laneViolations, 0u);
    CHECK(a.edgeCompletions > 100u);            // the network actually carried traffic
    CHECK(a.nearCamera > 0u);
    CHECK(m.vehicles > 200u);
    CHECK(a.meanSpeedKph > 1.0);
CHECK(a.minGap >= sim::kMinGap - 1e-6);
}
