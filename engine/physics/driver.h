// =============================================================================
//  Mumbai 1:1 EXACT — engine/physics/driver.h
//
//  The driver: Section 5's traffic model and Section 4's physics, sharing one
//  vehicle (D-034).
//
//  Until this file existed, the project had two fleets that never met.  The
//  traffic layer (`engine/sim`) decided where every vehicle goes — IDM, MOBIL,
//  the two-wheeler Boids field, signals, junction holds — and moved points along
//  a graph.  The physics layer (`engine/physics`) built real bodies with wheels,
//  suspension and lean springs, and the harness drove four of them by hand along
//  the Sea Link.  Neither could answer the question the specification actually
//  asks, which is what a Mumbai traffic jam *does* when it is made of matter.
//
//  The split here is the one every game engine converges on, and it is chosen
//  for a reason rather than by habit:
//
//    * the model owns **perception and intent** — who is ahead, what the signal
//      says, how fast IDM wants to go, whether to change lanes;
//    * the body owns **position and heading** — it accelerates, brakes, steers,
//      leans, climbs kerbs, and is the authority on where the vehicle went;
//    * every tick the body's actual position is projected back into the model
//      (`sim::Traffic::adoptPosition`), so the model's next decision is made
//      from where the vehicle really is, not from where the model hoped it
//      would be.
//
//  That last line is what makes this a simulation rather than an animation.  A
//  follower that is driven by the body's real position cannot be fed a fiction:
//  if the physics brakes harder than IDM predicted, the model sees the shorter
//  gap and the vehicle behind reacts to the *actual* gap.  The traffic layer's
//  behaviours are unchanged — the same 58-case suite still passes with promoted
//  and unpromoted vehicles mixed in one lane — and no vehicle is special-cased.
//
//  A driver is deliberately *not* an autopilot: it holds a lane (pure pursuit at
//  a fixed lookahead), it tracks the model's speed command, and it asks for the
//  steering and throttle the model decided.  Where it cannot — a kerb, a puddle,
//  a bus in the way, a wet hill — the body goes where physics takes it, the model
//  adopts that, and the fact is counted in `DriverMetrics` rather than smoothed
//  away.  A vehicle that ends up somewhere the model cannot explain is handed
//  back to the model and reported.
// =============================================================================
#pragma once

#include "engine/physics/vehicle.h"
#include "engine/physics/world.h"
#include "engine/sim/traffic.h"

#include <vector>

namespace mumbai::phys {

// How a driver behaves.  Every one of these is a knob with a physical meaning,
// and none of them is a fudge factor for a bug: the lookahead is where a driver
// looks, the gains are how hard they correct, and the corridor is how far off the
// lane a vehicle may drift before the model is told it is no longer on its road.
struct DriverSettings {
    // A sample counts as "off the road" this far from the lane centre; and a
    // vehicle counts as moving above this speed, which is what makes "wheels on
    // the road" a statement about driving rather than about standing still.
    f64 offRoadMetres = 2.0;
    f64 movingSpeed = 1.0;        // m/s

    // Which vehicles to take over.  The fleet is the `count` vehicles nearest
    // `centre` that the model has room to hand over (deterministic: ties are
    // broken by the model's own vehicle index).
    size_t count = 24;
    f64 attachRadius = 220.0;
    bool includeTwoWheelers = true;
    // Pure pursuit: aim at the lane centreline this far ahead.
    f64 lookaheadBase = 6.0;      // metres of line to aim at when stopped
    f64 lookaheadPerSpeed = 0.55; // ...plus this much per m/s of speed
    f64 lookaheadMax = 24.0;
    f64 steerGain = 1.8;          // rad of heading error -> full lock
    f64 lateralGain = 0.10;       // m of lane error -> rad of extra lock
    f64 steerSpeedDamping = 0.06; // ...both relaxed as speed rises (per m/s)
    // Speed control: throttle and brake as functions of the error between what
    // the model asked for and what the body is doing.
    f64 speedDeadband = 0.6;      // m/s where neither pedal is applied
    f64 throttleGain = 0.35;      // per m/s of error
    f64 throttleFloor = 0.12;     // keeps a moving vehicle rolling
    f64 brakeGain = 0.30;         // per m/s of error
    f64 brakeFloor = 0.08;
    f64 maxBrake = 1.0;
    // Hand back a vehicle whose body has left the corridor the model believes it
    // is on: a crash, a kerb climb, a fall.  The model must not be taught that
    // its road goes there.
    f64 corridorMargin = 6.0;     // m beyond the carriageway half-width
    f64 corridorShoulder = 1.0;   // ...and around a single lane (the driven corridor)
    f64 wrongWaySeconds = 0.5;    // s facing away from its own edge before handback
    // Seconds a body may sit outside the model's own edge (behind its start or past
    // its end) before the model takes it back.  Half a second is long enough to
    // cross a junction knot and short enough that a body is never driven far on a
    // line the model does not have (D-034).
    f64 offFrameSeconds = 0.5;
    // How many of the eight spawn-clearance rays may be blocked before a vehicle
    // is refused.  Zero would refuse a vehicle whose lane is beside a wall, which
    // is most of Mumbai; eight would place a car inside a building.
    int maxBlockedRays = 3;
    // How far the collision layer's surface may disagree with the model's own
    // road height before the vehicle is refused at handover.  The two layers are
    // built from different data — the graph from the DEM and OSM way flags, the
    // collision from the world's own bake — and a disagreement larger than a
    // road's thickness means the model is describing a place the collision does
    // not have (a tunnel that was never built into the geometry, a lane over
    // water).  Such a vehicle is refused, and the disagreement is counted.
    f64 heightTolerance = 1.0;
    // Once a vehicle is on the road, the *relationship* is what matters: the
    // model's height is a straight line between two node elevations, and a real
    // road curves vertically (a ramp, a crest, a bridge approach), so the two
    // layers legitimately differ by a metre along one edge while agreeing about
    // where the road is.  What must not happen is the difference *changing* — that
    // is the body climbing a kerb or falling through a hole.  So the driver
    // remembers the offset measured at handover and faults on its change.
    f64 heightDriftTolerance = 0.6;
    // The largest step in the surface under a vehicle's own footprint that it will
    // be placed across: a kerb (0.15 m) is a step, a dropped kerb is not, and
    // anything above this is a wall or a plinth the model has no idea about.
    f64 footprintStepTolerance = 0.35;
    // A driver looks ahead.  Every `lookAheadSeconds` the road `lookAheadMetres`
    // in front of a driven vehicle is validated the same way its spawn position
    // was, and a vehicle whose road ahead the collision does not have is handed
    // back to the model *before* it drives into it.  Without this the section
    // measured bodies falling into holes and climbing walls and called it
    // penetration (D-034); with it, a place the two layers disagree about simply
    // does not get a body driven into it.
    f64 lookAheadSeconds = 0.5;
    // How far a driven vehicle's path is validated ahead of it, and how much
    // overlap is tolerated while doing it.  A centimetre or two is a kerb a tyre
    // rides over; half a metre is a wall.
    f64 pathProbeMetres = 12.0;
    f64 probePenetrationTolerance = 0.25;   // overlap tolerated by the look-ahead
    // Overlap tolerated where a body is *created*: the same authored-clearance
    // rule the props obey (nothing is authored inside anything else).  A vehicle
    // placed with a quarter of a metre of itself inside a wall is ejected by the
    // solver at metres per second, which the model then sees as a body that
    // refused to obey (D-034).
    f64 spawnOverlapTolerance = 0.05;
    f64 lookAheadMetres = 18.0;
    // Ticks of agreement violation tolerated before the model takes a vehicle
    // back.  A single tick of disagreement is a bump; six is a road that is not
    // there.
    int heightGraceTicks = 6;
    // Diagnostic: print each handback with its reason, and each vehicle that
    // drifts further than `traceOffM` from its lane, once per vehicle.
    bool trace = false;
    // Per-ray diagnostics for the survey's no-surface samples: what each ground
    // ray started at, what it hit and where.  The D-036 counters disagreed with
    // each other — an over-ray finding a surface 0.2-0.7 m above the model's road
    // while the ground rays reported nothing in the band — and that is a question
    // only a per-ray dump can answer (D-037).
    bool surveyTrace = false;
    f64 traceOffM = 1.5;
};

struct DrivenVehicle {
    u32 simIndex = 0;
    Vehicle body;
    // Diagnostics for this vehicle, kept per vehicle so a failure names one.
    f64 worstLateralOff = 0.0;
    f64 alongErr = 0.0;         // m the body is ahead of the model's s
    f64 sideErr = 0.0;          // m the body is beside the model's lane line
    f64 worstAlongErr = 0.0, worstSideErr = 0.0;    // m, distance from the lane centreline
    f64 worstSpeedError = 0.0;    // m/s, |body speed - the model's command|
    f64 lateralOffSum = 0.0;      // m, summed over ticks
    f64 wheelContactSum = 0.0;    // wheel contacts per tick, summed
    f64 wheelCountSum = 0.0;      // wheels per tick, summed (the denominator)
    u64 airborneSamples = 0;      // ticks with no wheel touching anything
    u32 wheelTicks[4] = {0, 0, 0, 0};   // contacts per wheel index, over the run
    // The speed the *driver* is judged on: the samples where the vehicle is on the
    // road the model put it on (its own lane, and the two layers agreeing about the
    // height).  A sample where the model asked a vehicle to hold 40 km/h on a road
    // the collision does not have is reported as an off-road or handback event, not
    // as a speed-tracking error (D-034).
    u64 onRoadSamples = 0;
    f64 onRoadSpeedErrorSum = 0.0;
    f64 onRoadSpeedErrorWorst = 0.0;
    u64 movingSamples = 0;        // ticks with the vehicle actually moving
    f64 movingContactSum = 0.0;   // wheel contacts while moving, summed
    f64 movingSlotSum = 0.0;      // wheels while moving, summed (the denominator)
    f64 offSumSquared = 0.0;      // m^2, for the RMS lane error
    f64 sideAbsSum = 0.0;         // m, |side| summed (the driving error)
    f64 alongAbsSum = 0.0;        // m, |along| summed (the model's bookkeeping error)
    u64 sideOffSamples = 0;       // ticks with |side| > `offRoadMetres`
    f64 wrongWaySeconds = 0.0;    // seconds facing away from the vehicle's own edge
    f64 offFrameSeconds = 0.0;    // seconds outside the model's own edge
    f64 corridorM = 0.0;          // the corridor it was held to this tick
    f64 worstOutsideM = 0.0;      // worst distance outside it, either end
    // Where the worst speed error happened, and what the two numbers were: a
    // worst-case number with no place and no pair (command, actual) is not a
    // diagnosis (D-034).
    u64 worstSpeedAtTick = 0;
    f64 worstCommandMs = 0.0, worstActualMs = 0.0;
    Vec2d worstSpeedAt{};
    u64 movingAirborne = 0;
    u64 offRoadSamples = 0;        // ticks more than `offRoadMetres` from the lane
    f64 surfaceOffset = 0.0;      // collision surface - model height, at handover
    f64 heightErrorSum = 0.0;     // m, change in that offset since handover
    f64 heightErrorWorst = 0.0;
    int heightGrace = 0;          // consecutive ticks of drift
    f64 speedErrorSum = 0.0;      // m/s, summed over ticks
    f64 speedSum = 0.0;           // m/s, summed over ticks
    u64 speedSamples = 0;
    u64 wheelsDownSamples = 0, samples = 0;
    bool handedBack = false;
    const char* handbackReason = "";
    // Where the body was created and what the collision said under it: kept so
    // that a vehicle whose run goes wrong can be reported with the place it
    // started rather than only the place it ended.
    Vec2d spawn{0, 0};
    f64 spawnY = 0.0;
};

struct DriverMetrics {
    size_t promoted = 0;
    size_t handedBack = 0;
    u64 ticks = 0;
    f64 worstLateralOffM = 0.0;
    f64 meanLateralOffM = 0.0;
    f64 meanSpeedErrorMs = 0.0;
    f64 worstSpeedErrorMs = 0.0;
    f64 wheelsDownFraction = 0.0;
    f64 movingWheelFraction = 0.0;   // wheels down while actually moving
    f64 meanSideErrorM = 0.0;        // mean |lateral| error to the vehicle's own lane
    f64 worstSideErrorM = 0.0;       // worst |lateral| error to its own lane
    f64 meanAlongErrorM = 0.0;       // mean |along| error (the model's s vs the body)
    f64 worstAlongErrorM = 0.0;
    f64 sideOffShare = 0.0;          // share of samples more than `offRoadMetres` beside
    f64 laneErrorRmsM = 0.0;         // RMS of the whole lane error, metres
    f64 onRoadMeanSpeedErrorMs = 0.0;   // speed error over samples on its own road
    f64 onRoadWorstSpeedErrorMs = 0.0;
    u64 onRoadSamples = 0;
    f64 offRoadShare = 0.0;          // samples further than `offRoadMetres` off lane
    u64 movingSamples = 0;
    f64 meanSpeedKph = 0.0;
    f64 deepestPenetrationM = 0.0;
    // Split by what the driven vehicle was in contact with.  The city's geometry is
    // the model's to respect (a building, a kerb, the deck); another body is a
    // contact between two solver objects, and Jolt resolves that with its own
    // allowance.  One number cannot say which of the two happened, and the first
    // version of this accounting read the static depth for both (D-034).
    f64 deepestDrivenStaticM = 0.0;
    // Split by speed, because they are different events (D-037): a body *at rest*
    // inside the city is a placement defect at any depth, a body *moving* against
    // a vertical face is Jolt's linear cast for one step.
    f64 deepestDrivenStaticAtRestM = 0.0;
    f64 deepestDrivenStaticMovingM = 0.0;
    f64 worstMovingContactSpeedMs = 0.0;
    // Carried, not touched: the longest run of consecutive ticks in which a driven
    // vehicle was more than the budget inside the city, and the deepest depth seen
    // during a run long enough that the solver was not resolving it (D-037).
    u64 longestStaticRunTicks = 0;
    f64 deepestSustainedStaticM = 0.0;
    Vec3 sustainedStaticAt{};
    f64 deepestDrivenDynamicM = 0.0;
    u64 modelVehiclesPromoted = 0;
    u64 rejectedSpawns = 0;       // vehicles the model offered and the city refused
    u64 rejectedOnTraffic = 0;    // refused because a vehicle was already there
    u64 rejectedOnCity = 0;       // refused because the city's own shape is under it
    u64 handedBackHeight = 0;     // handed back because the layers disagreed
    u64 handedBackAhead = 0;      // handed back because the road ahead is not there
    u64 handedBackOffRoad = 0;    // handed back because the body left the corridor
    u64 handedBackWrongWay = 0;   // handed back because the model turned it around
    u64 handedBackOffFrame = 0;   // handed back because the body left the model's edge
    u64 airborneSamples = 0;      // ticks with no wheel in contact at all
    f64 worstHeightErrorM = 0.0;  // worst drift between the two layers
    f64 deepestDrivenM = 0.0;     // deepest penetration of a vehicle still driven
    f64 meanHeightErrorM = 0.0;
};

class PhysicalFleet {
public:
    // Takes over up to `settings.count` vehicles near `centre`.  Returns how many
    // were promoted.  Bodies are placed on the model's own road height (the nav
    // graph's node elevations), refined by one short downward cast so that a
    // bridge deck, a kerb or a ramp is found where it really is.
    size_t attach(World& world, sim::Traffic& traffic, const Vec2d& centre,
                  const DriverSettings& settings = DriverSettings{});

    // Drives every promoted vehicle from the model's decision, steps the world
    // once, and reports every body's position back to the model.  This is the
    // whole control loop, and it must be called with the same `dt` the model is
    // stepped with.
    void step(World& world, sim::Traffic& traffic, f64 dt);

    // Hands every vehicle back to the model and destroys the bodies.
    void detach(World& world, sim::Traffic& traffic);

    const std::vector<DrivenVehicle>& vehicles() const { return m_driven; }
    // Prints one line per vehicle: where it ended up, how far off its lane it
    // drifted and how often its wheels were on the road.  A section-level average
    // hides the one vehicle that spent the run in a wall.
    void report() const;
    const DriverMetrics& metrics() const { return m_metrics; }
    size_t size() const { return m_driven.size(); }

    // One surveyed point's story, kept for the diagnostic list.
    struct BlockedSample {
        f64 model = 0.0, ground = 0.0, probeY = 0.0, depth = 0.0;
        Vec2d at{};
        u16 lanes = 1;
        u8 cls = 0;
        std::string what;      // what the box is inside, in words
    };
    // The result of surveying the model's whole road network against the
    // collision layer: the one measurement that says whether the two layers
    // describe the same city.  A driven vehicle can only ever sample the few
    // hundred metres it happens to drive; this samples every road the model has.
    struct LaneSurvey {
        u64 samples = 0;           // lane points probed
        u64 withinQuarter = 0;     // collision surface within 0.25 m of the model's
        u64 withinHalf = 0;
        u64 withinTwo = 0;
        u64 blocked = 0;           // a vehicle could not be placed there at all
        u64 blockedHeightAgrees = 0;   // ...where the two layers *do* agree on height
        u64 blockedHeightDiffers = 0;  // ...where they do not
        u64 noSurface = 0;         // the collision has no surface anywhere near
        u64 structureOnly = 0;     // ...of which only structures outside the band
        u64 underStructure = 0;    // ...of which the road is under something above it
        u64 noCollisionAtAll = 0;  // ...of which nothing is there at all
        f64 minUnderStructureM = 1e30;   // the lowest such structure's clearance
        // What is above a lane point with nothing in the band, in five bins: a
        // clearance of ~0 means the model's road is *inside* geometry, 1.5-8 m is
        // a surface above it (a deck, or ground the grading put above the lane),
        // and 8 m+ is a road the collision layer simply does not have (D-036).
        u64 overBins[5] = {0, 0, 0, 0, 0};
        // The *signed* disagreement, ground minus model, in 0.25 m bins from
        // -3 m to +3 m with the two overflows.  A magnitude says the two layers
        // disagree; the sign says which of them is the authority to make the
        // other follow (D-037, item 4b).
        u64 signedBins[26] = {};
        f64 signedSum = 0.0;
        // The positive tail (the collision ground more than 0.25 m *above* the
        // model's road, which is every bin that matters), split by the layer the
        // hit belongs to: a terrain heightfield, a road/structure mesh, or a built
        // compound (D-037).
        u64 tailAboveHeightfield = 0, tailAboveMesh = 0, tailAboveBuilt = 0;
        u64 diagnosed = 0;         // samples whose rays were dumped (surveyTrace)
        std::vector<BlockedSample> examples;
        u64 stepped = 0;           // more than `footprintStepTolerance` of step
        f64 worstErrorM = 0.0;
        Vec2d worstAt{};
        u64 worstBody = 0;
        f64 worstDepthM = 0.0;
        Vec2d blockedAt{};
        u64 blockedBody = 0;
        f64 blockedDepthM = 0.0;
        std::string blockedWhat;
        u64 worstEdge = 0;
        // --- elevation, measured apart from the surface (D-036) ---------------
        // An elevated road's model height is its *deck*, and the ground under it
        // is tens of metres lower.  Comparing the two and calling the difference
        // a disagreement is a category error in the instrument: it says nothing
        // about whether the deck the vehicle drives on is where the model thinks
        // it is.  The survey therefore reports bridge edges apart from surface
        // streets, and counts a bridge sample with nothing near the model's
        // height as `noDeckSurface` rather than as a 30 m error.
        u64 bridgeSamples = 0;
        u64 bridgeWithinQuarter = 0, bridgeWithinHalf = 0, bridgeWithinTwo = 0;
        u64 bridgeNoDeck = 0;
        f64 worstBridgeErrorM = 0.0;
        Vec2d worstBridgeAt{};
        // --- elevated roads, classified by measurement (D-036) ----------------
        // The OSM bridge flag covers 133 of 854,696 samples — the authored
        // corridors and the Sea Link are elevated roads that carry no such tag, so
        // a flag cannot be the classifier.  The measurement can: a lane point whose
        // nearest collision surface is more than 3 m *below* the model's road is an
        // elevated road, and the question for it is not "does the ground agree"
        // but "is there a deck at the model's height".  Street-level samples keep
        // the plain comparison and the headline.
        u64 elevatedSamples = 0, elevatedWithDeck = 0;
        // The other direction: a collision surface well *above* the model's road.
        // That is a deck or a structure the collision layer has and the nav graph
        // does not — the Sea Link's own corridor, where the model's edge runs at
        // ground level under the deck it is supposed to be on (D-036).
        u64 collisionAbove = 0;
        f64 worstCollisionAboveM = 0.0;
        Vec2d worstCollisionAboveAt{};
        f64 worstElevatedDropM = 0.0;
        Vec2d worstElevatedAt{};
        // --- headroom ---------------------------------------------------------
        // Straight up from the lane centre: the first surface above the road.  A
        // box that is "1.88 m inside a mesh" in a street with 2.05 m of clearance
        // is a vehicle that does not fit under the structure, and saying so is
        // the difference between a finding and a mystery.  `headroomUnder 2.6 m`
        // is the count of lane points a car (rest + roof + a margin) cannot pass.
        u64 headroomSamples = 0, headroomUnderCar = 0, headroomUnderAny = 0;
        f64 minHeadroomM = 1e30;
        Vec2d minHeadroomAt{};
        u64 minHeadroomBody = 0;
        f64 minHeadroomAtM = 0.0;
        u64 headroomHeightAgrees = 0;   // the blocked sample's ground matched the model
    };
    // Every `spacing` metres along every edge, at the centre of a lane, ask the
    // collision layer where the road is.  Costs a ray cast and a box overlap per
    // sample; on this city's 8,394 km of centreline that is minutes, so it is a
    // diagnostic (see `--survey`), not something a run does by itself.
    LaneSurvey surveyLanes(World& world, sim::Traffic& traffic, f64 spacing,
                           const DriverSettings& settings = DriverSettings{});
    struct ProbeReport {
        f64 depth = 0.0;
        u32 body = 0xFFFFFFFFu;
        Vec3 at{};
    };
    // Whether a vehicle's own chassis box, at this position and heading, is clear
    // of the city.  This is the real question — a vehicle does not fit through a
    // gap because eight rays missed it, it fits because the box it *is* fits —
    // and it catches what a horizontal ray cannot: a kerb it would have to climb,
    // a balcony over its roof, a lane that narrows to a metre.
    bool boxFits(World& world, const Vec2d& p, f64 y, f64 yaw, sim::Archetype kind,
                 f64 penetrationTolerance, ProbeReport* reportOut);
    // The same test for a vehicle in motion: its box is sampled at four points
    // along the path it is about to take.
    bool pathFits(World& world, sim::Traffic& traffic, u32 index, f64 metres,
                  ProbeReport* reportOut);

private:
    // Whether a vehicle of this archetype, placed at this pose, overlaps the
    // static city.  This is the *authored* clearance check: the project's rule is
    // that nothing may be authored inside anything else, and a body created with a
    // metre of itself inside a wall is ejected by the solver at tens of metres a
    // second — which is what the first run of this section measured (a body at
    // -9.18 m/s while the model asked it to stand still, D-034).
    f64 staticOverlapAt(World& world, sim::Archetype kind, const Vec2d& p, f64 y, f64 yaw);
    // The *whole* vehicle, not only its chassis box: everything the body has
    // between the ground and its roof, which is the chassis box plus the wheels
    // and the space under the floor.  A chassis box starts 0.48 m above the road
    // on a double-decker and a kerb is 0.15 m tall, so a gate that asks only
    // about the chassis authorises a vehicle whose *wheels* are inside the kerb:
    // measured, that is the 0.0370 m that survived every change to how the
    // vehicle is driven (D-034).
    f64 wholeVehicleOverlapAt(World& world, sim::Archetype kind, const Vec2d& p, f64 groundY,
                              f64 yaw);
    // The same probe against *every* body, so a spawn is never authored on top of a
    // vehicle the fleet has already placed (D-034).
    f64 anyOverlapAt(World& world, sim::Archetype kind, const Vec2d& p, f64 y, f64 yaw);
    // Hands one vehicle back (body destroyed, model resumed) in place.
    void handBack(World& world, sim::Traffic& traffic, size_t i);
    // The surface under (x, z), searched around the model's own idea of the road
    // height.  Returns the model's number when the collision disagrees with it by
    // more than a road's thickness.
    f64 surfaceAt(World& world, f64 x, f64 z, f64 modelHeight);
    // The depth already classified at rest / while moving: a persistent deepest
    // contact is attributed once, where it appeared.
    f64 m_classifiedStaticM = 0.0;
    u32 m_staticRunTicks = 0;
    std::vector<DrivenVehicle> m_driven;
    DriverMetrics m_metrics;
    DriverSettings m_settings;
    f64 m_lookAheadAccum = 0.0;
    // The deepest contact seen so far *in this section*, so that it can be named
    // while the bodies in it are still alive: a body id is only valid until the
    // body is destroyed, and the vehicles of this section are destroyed when the
    // harness detaches the fleet (D-034).
    f64 m_reportedStaticM = 0.0;
    f64 m_reportedDynamicM = 0.0;
};

}  // namespace mumbai::phys
