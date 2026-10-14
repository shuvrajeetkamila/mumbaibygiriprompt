// =============================================================================
//  MUMBAI 1:1 EXACT — engine/sim/traffic.h
//
//  Section 5's traffic system, as a runtime layer rather than a demo.
//
//  The specification asks for a hybrid model:
//    * large vehicles — BEST double-deckers, kaali-peeli taxis, auto-rickshaws —
//      on IDM longitudinal behaviour with MOBIL lane changes;
//    * small vehicles — two-wheelers, 110-160 kg, which in Mumbai ignore lanes
//      entirely — on a Boids flow field with a 200 ms update, a 20 m
//      broadphase, separation strong, alignment present, cohesion ~ 0;
//    * signals with phases, because a junction with no phases is a crossing.
//
//  Everything here is deterministic: one seed, no wall clock, identical output
//  for identical input.  The harness runs it for simulated hours and audits
//  every vehicle every tick, so a model that is merely plausible is not enough.
// =============================================================================
#pragma once
#include "engine/core/geo.h"
#include "engine/core/math.h"
#include "engine/world/chunk.h"
#include "engine/world/signal_warrant.h"

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mumbai::sim {

// -----------------------------------------------------------------------------
//  Vehicle archetypes
// -----------------------------------------------------------------------------
// Masses and dimensions are the specification's numbers.  The pair in brackets
// that the specification gives per archetype is implemented as the *comfort*
// fraction of the class maxima in `archetypeSpec`, not as metres per second
// squared: read literally as m/s² those numbers make a motorcycle take six
// minutes to reach 20 km/h, which no motorcycle does, whereas as fractions they
// reproduce exactly the behaviour the specification describes in words — the
// bus pulls away hard and brakes early, the auto crawls and brakes late, the
// motorcycle barely touches the throttle and brakes hardest of all.  See D-027.
enum class Archetype : u8 {
    Motorcycle = 0,
    AutoRickshaw,
    Taxi,
    Car,
    Bus,
    Count,
};

struct ArchetypeSpec {
    const char* name;
    f64 length;            // m, bumper to bumper
    f64 massKg;
    f64 width;             // m
    f64 comfortAccel;      // fraction of maxAccel the driver actually uses
    f64 comfortBrake;      // fraction of maxBrake the driver actually uses
    f64 maxAccel;          // m/s^2
    f64 maxBrake;          // m/s^2
    f64 topSpeedKph;
    bool twoWheeler;       // rides the Boids field instead of a lane
    bool canHail;          // stops for a fare (kaali-peeli)
    bool canTip;           // rollover geometry (three-wheeler)
    f64 tipLateralG;       // lateral acceleration at which it goes over
};

const ArchetypeSpec& archetypeSpec(Archetype a);

// -----------------------------------------------------------------------------
//  Signal phases
// -----------------------------------------------------------------------------
// A phase governs the vehicles approaching the *end* of one edge.  The world
// carries only a green window inside a fixed cycle (NavSignalPhase); amber and
// the all-red interval are derived here so the behaviour lives in one place.
struct SignalPhase {
    u32 edge = 0;
    f64 greenStart = 0.0;
    f64 greenSeconds = 0.0;
    static constexpr f64 kCycleSeconds = 90.0;
    static constexpr f64 kAmberSeconds = 3.0;
    static constexpr f64 kAllRedSeconds = 1.5;
};

enum class SignalState : u8 { Green = 0, Amber, Red };

// The state of one phase at time `t` seconds into the run.
SignalState signalStateAt(const SignalPhase& p, f64 t);

// -----------------------------------------------------------------------------
//  Boids weights for the two-wheeler field
// -----------------------------------------------------------------------------
struct BoidsWeights {
    f64 separation = 0.8;      // strong, per the specification
    f64 alignment  = 0.25;     // present
    f64 cohesion   = 0.0;      // ~ 0: two-wheelers filter, they do not bunch
    f64 boundary   = 1.2;      // holds them inside the carriageway
    f64 lateralMaxG = 0.45;    // the lean at which the front wheel lets go
    f64 updateSeconds = 0.20;
    f64 broadphaseMetres = 20.0;
};

// -----------------------------------------------------------------------------
//  Vehicle
// -----------------------------------------------------------------------------
struct Vehicle {
    Archetype kind = Archetype::Car;
    u32 edge = 0;
    u16 lane = 0;
    f64 s = 0.0;             // metres along the edge
    f64 v = 0.0;             // m/s
    f64 v0 = 0.0;            // desired speed (m/s)
    f64 lateral = 0.0;       // metres right of the edge centreline
    // Heading, radians, of a vehicle whose motion is external (the physics body's
    // own yaw, `atan2(dx, dz)`).  A driven vehicle's path through a junction is the
    // driver's, not the graph's: the model follows the body (see `advanceEdges`).
    f32 heading = 0.0f;
    bool headingValid = false;
    // The body's own position along the model's edge, *unclamped*: negative behind
    // the edge's start, beyond `length` past its end.  `s` is the model's clamped
    // reading of the same thing; this is what says whether the model's frame still
    // contains the body at all (D-034).
    f64 bodyAlong = 0.0;
    f64 lateralVel = 0.0;    // m/s, the animation of the line the vehicle takes
    f64 latAccelCommand = 0.0;  // m/s^2, set by the Boids field every 200 ms
    f64 a = 1.4, b = 2.0;    // effective IDM parameters (m/s^2)
    f64 T = 1.2, s0 = 2.2;
    f64 length = 4.4;
    // The obstruction the vehicle must not pass: the rear face of whatever is
    // ahead — the leader's rear bumper, or the stop line at a signal.  One
    // scalar covers car-following and signal obedience, and makes the safety
    // projection exact: s is never allowed past obstruction - kMinGap.
    f64 obstruction = 1e18;
    f64 gap = 1e9;           // obstruction - s, recomputed every tick
    f64 leaderV = 0.0;
    f64 hailTimer = 0.0;     // seconds left of a fare stop (or of a cool-down)
    bool hailing = false;
    bool changedLaneThisStep = false;
    bool lateralCapped = false;   // this manoeuvre hit the rollover limit
    bool atRedLine = false;       // held by a red or amber phase this tick
    bool redStopped = false;      // ...and has actually come to rest there
    // A vehicle the population device could not place waits this long before it
    // is tried again: a jammed disc must cost nothing to stay jammed.
    f64 recycleIn = 0.0;
    // --- motion handoff (D-034) ---------------------------------------------
    // A *promoted* vehicle's position and heading are owned by a physics body.
    // The model still perceives (leaders, gaps, signals) and still decides — the
    // two fields below are what it decided — but it does not integrate, because
    // the body is the authority on where the vehicle went.  The body is driven
    // to `commandSpeed`, and the achieved state comes back through
    // `adoptPosition()`, so perception sees what the physics actually did.
    bool external = false;
    f64 commandAccel = 0.0;    // m/s^2, IDM's answer this tick
    f64 commandSpeed = 0.0;    // m/s, where IDM would have put it this tick
    // How long an external vehicle has been held at the end of an edge (a dead
    // end, or a junction that will not take it).  A vehicle whose position is
    // owned by a body cannot be teleported, so a hold that never ends has to end
    // some other way: after `kAnchorHoldSeconds` the model takes the vehicle back
    // outright, and the owner sees `external` cleared and destroys the body.
    f64 heldSeconds = 0.0;
};

// -----------------------------------------------------------------------------
//  Graph
// -----------------------------------------------------------------------------
// The chunk format stores one record per segment with a direction flag: 0 means
// both ways, and no reverse record is written.  The runtime therefore builds the
// reverse edge itself — a road a vehicle can only drive along in one direction is
// not a road — and remembers the pairing so that a U-turn at a mid-block node can
// be refused and the opposing approach can carry the signal phase its own
// direction could not name.
constexpr u32 kNoEdge = 0xFFFFFFFFu;

struct SimEdge {
    u32 from = 0, to = 0;
    u32 reverse = kNoEdge;
    f32 length = 0.0f;
    f32 limit = 30.0f;       // kph
    u16 lanes = 1;
    u8 cls = 0;
    Vec2d a, b;
    Vec2d dir{1, 0};         // unit, a -> b
    Vec2d normal{0, 1};      // unit, right of dir
    // Absolute road-surface height at each end, from the nav graph's own node
    // elevations.  The physics layer needs to know where the road is before it
    // can put a body on it, and this is the model's own knowledge of its road —
    // a probe cast from an assumed height would be a guess about the city.
    f32 ay = 0.0f, by = 0.0f;
    std::vector<std::vector<u32>> laneVehicles;   // sorted by s
};

struct SimNode { Vec2d p; std::vector<u32> out; };

// -----------------------------------------------------------------------------
//  Metrics, published so the harness can assert on them
// -----------------------------------------------------------------------------
struct TrafficMetrics {
    size_t vehicles = 0;
    size_t nearCamera = 0;         // within the audit radius of the given point
    size_t edges = 0;
    size_t nodes = 0;
    size_t lanesTotal = 0;
    size_t signals = 0;
    size_t reverseEdges = 0;       // two-way segments materialised as both edges
    size_t derivedSignals = 0;     // phases the runtime had to derive itself
    size_t stranded = 0;           // vehicles clear of the disc and not yet placed
    f64 meanSpeedKph = 0.0;
    f64 minGap = 1e9;
    f64 crawlShare = 0.0;          // fraction below 1.4 m/s
    u64 laneChanges = 0;
    u64 laneChangeRefusals = 0;
    u64 edgeCompletions = 0;
    u64 deadEndHolds = 0;
    u64 signalStops = 0;           // came to rest at a red or amber line
    u64 junctionHolds = 0;         // turned away because the junction was full
    u64 holdCaps = 0;              // holds stopped at the junction-box limit
    u64 signalCrossings = 0;       // cleared a signalised edge (so: on green)
    u64 recycleFailures = 0;       // departures with nowhere to re-enter
    u64 boidsUpdates = 0;
    u64 separationEvents = 0;
    u64 tipLimits = 0;
    u64 hailStops = 0;
    u64 hardBrakes = 0;
    u64 badState = 0, badSpeed = 0, badPosition = 0;
    u64 nodesMerged = 0;           // nodes jointed to another node by position
    u64 nodesMergedAcrossChunks = 0;   // ...of which the seam between files
    u64 deadEndUTurns = 0;
    u64 graphContradictions = 0;   // bodies whose node had no arm they could take
    u64 laneViolations = 0;        // pairs in one lane closer than the floor
    u64 anchorReleases = 0;        // external vehicles the model took back
    u64 orphanRepairs = 0;         // lane-list entries rebuilt for a driven vehicle
    f64 maxLateralG = 0.0;         // worst lean seen, across all two-wheelers
    u64 archetypeCount[static_cast<int>(Archetype::Count)] = {0, 0, 0, 0, 0};
};

// -----------------------------------------------------------------------------
//  Traffic
// -----------------------------------------------------------------------------
class Traffic {
public:
    // Builds the graph from the world's chunk files and populates a peak-hour
    // fleet: most of it packed into a dense disc around `denseAnchor`, the rest
    // spread along the corridor.
    Traffic(const std::vector<std::string>& chunkFiles,
            const std::vector<Vec2d>& route,
            const Vec2d& denseAnchor,
            u64 fleet,
            u64 seed = 0x5EED1234ull);

    void step(f64 dt);
    void audit(f64 minX, f64 maxX, f64 minZ, f64 maxZ);
    // The audit's findings are counted always and named on request: a violation is a
    // fact about the fleet, and which pair is in it is what says whose motion is
    // wrong (D-034).
    void setAuditTrace(bool on) { m_auditTrace = on; }
    TrafficMetrics metrics(const Vec2d& camera, f64 radius) const;

    // Exposed for tests and for a renderer that wants to draw them.
    const std::vector<Vehicle>& vehicles() const { return m_vehicles; }
    const std::vector<SimEdge>& edges() const { return m_edges; }
    const std::vector<SignalPhase>& signals() const { return m_signals; }
    const BoidsWeights& boids() const { return m_boids; }

    // Simulated seconds since construction.
    f64 time() const { return m_time; }

    // How many edges the dense disc currently offers as re-entry points.  A
    // population device that cannot place a departure has to say so, and this is
    // how the harness sees it.
    size_t denseEdgeCount() const { return m_denseEdges.size(); }
    size_t denseRingEdgeCount() const { return m_denseRing.size(); }

    // Moves the dense disc.  This is the harness's population device, not a
    // traffic behaviour: the specification's fleet is a peak-hour population
    // *around the camera*, and a disc fixed at the construction point empties as
    // the camera drives away from it.  It changes only where a recycled vehicle
    // reappears — the disc is a place, never a behaviour.
    void setDenseAnchor(const Vec2d& anchor, f64 radius = 500.0);

    // --- motion handoff (D-034) ---------------------------------------------
    // Promotes a vehicle's motion to an external body: from this tick the model
    // decides but does not integrate.  Returns false if the index is not a
    // vehicle, or if it is already promoted.
    bool promote(u32 idx);
    // Hands the motion back.  The model resumes from whatever state the body
    // last reported, so a handback is continuous by construction.
    void demote(u32 idx);
    bool isPromoted(u32 idx) const;
    u64 promotions() const { return m_promotions; }
    u64 anchorReleases() const { return m_anchorReleases; }
    u64 orphanRepairs() const { return m_orphanRepairs; }
    u64 demotions() const { return m_demotions; }
    size_t promotedCount() const { return m_promotedNow; }

    // A reading of the loaded graph's own connectivity, for the acceptance run
    // to publish: how many nodes have exactly one out-arm (their own reversal),
    // i.e. places where a body can arrive and the model can only turn it around.
    // A road graph in which a node's only exit is the way back is either a
    // genuine cul-de-sac or a hole; naming the count is what separates the two
    // from a guess (D-034).
    struct GraphAudit {
        u64 nodes = 0, out1 = 0, out2 = 0, out3 = 0, out4plus = 0;
        u64 oneArmIsReversal = 0u;      // the single out-arm goes back the way in
        u64 oneArmHasTwinNearby = 0u;   // ...and another node with arms is within 2 m
    };
    GraphAudit graphAudit() const;

    // The route the model has the vehicle on, as a point in world space `ahead`
    // metres along its lane — what a driver needs to steer at, and the only
    // thing about the graph an external body has to know.
    Vec2d lanePoint(const Vehicle& v, f64 ahead) const;
    // Heading of that lane, radians, in the world's yaw convention
    // (`atan2(dx, dz)`, the one `spawnVehicle` and the renderer share).
    f64 laneHeading(const Vehicle& v) const;

    // The model's road at an arbitrary point on an arbitrary edge — the *lane*
    // centreline of lane `lane` (0-based, out of `e.lanes`) at `s` metres along
    // the edge, and the model's height there.  This is what a survey of the whole
    // city needs and what a per-vehicle query cannot give: the road as the model
    // believes it to be, independent of which vehicle happens to be on it (D-034).
    Vec2d laneCentre(const SimEdge& e, u32 lane, f64 s) const;
    // The lateral offset of a lane's centreline from the edge's centreline, in metres
    // (right positive): the number that says where a lane is, exposed so a driver can
    // reason about the road's own flat zone the way the model does (D-034).
    static f64 laneOffsetFor(u16 lane, u16 lanes);
    // The direction the model has the vehicle travelling along its edge.
    Vec2d edgeDir(const Vehicle& v) const { return m_edges[v.edge].dir; }
    f64 laneHeight(const SimEdge& e, f64 s) const;

    // A body reports where it actually is, in world space.  The model projects
    // the point onto the vehicle's own edge, updates its own `s`/`lateral`/`v`
    // and re-orders its lane list, so perception downstream is exact.
    void adoptPosition(u32 idx, const Vec2d& p, f64 speed, f64 heading = 0.0);

    const Vehicle& vehicleAt(u32 idx) const { return m_vehicles[idx]; }
    size_t vehicleCount() const { return m_vehicles.size(); }
    const SimEdge& edgeOf(const Vehicle& v) const { return m_edges[v.edge]; }

    // Places one vehicle of a given archetype at an exact point on an exact
    // edge.  Used by the harness for scripted scenarios and by the validation
    // suite to build a fleet by hand.  Returns kNoVehicle if the point is
    // occupied: a vehicle is never created inside another one.
    u32 spawn(Archetype kind, u32 edge, u16 lane, f64 s, f64 v);
    static constexpr u32 kNoVehicle = 0xFFFFFFFFu;

    // Position of a vehicle in world space.
    Vec2d positionOf(const Vehicle& v) const;

    // The height of the road surface under a vehicle, interpolated along its own
    // edge from the nav graph's node elevations.  Used to place a physical body
    // on the road (D-034) and by anything that needs the model's idea of where
    // the ground is — the renderer, and the audio layer's distance cues.
    f64 surfaceHeightOf(const Vehicle& v) const;

private:
    void leadersAndSignals();
    void longitudinal(f64 dt);
    void corrections();
    void laneChanges(f64 dt);
    void twoWheelerField(f64 dt);
    void lateralDynamics(f64 dt);
    void faresAndQueues(f64 dt);
    void advanceEdges();
    // Moves one vehicle back into the disc around the camera.  Returns false if
    // the disc (and then the ring around it) has nowhere to put it.  The caller
    // owns the lane lists: the departure path has already unregistered the
    // vehicle, the maintenance path has not.
    bool recycle(u32 idx, bool preferAhead);
    void removeFromLane(u32 edge, u16 lane, u32 idx);
    void maintainDensePopulation(f64 dt);
    void collectActiveEdges();
    void deriveSignals();
    void addDerivedPhase(u32 edge, const Vec2d& axis);
    u32 nodeAt(const Vec2d& p, bool* created = nullptr);
    void insertSorted(std::vector<u32>& list, u32 idx);
    bool laneEntryClear(const SimEdge& e, u16 lane, f64 s, f64 length, f64 width) const;
    void placeHeld(Vehicle& veh, const SimEdge& e, u16 lane, f64 limit);

    std::vector<SimNode> m_nodes;
    // The elevation of each node, kept beside its plan position and merged the
    // same way.  A junction whose arms lie in two chunk files is two node records
    // with the same position *and different heights* (each file sampled the
    // terrain for itself), so merging only the position left the model's road
    // height stepping by up to a metre at a chunk seam — invisible to the model,
    // which only ever used x and z, and immediately visible to a body being driven
    // across it (D-034).
    std::vector<f32> m_nodeY;
    std::vector<SimEdge> m_edges;
    // Nodes are jointed by position across the whole world, not per file: a
    // junction whose arms lie in two neighbouring cells is two nodes in two
    // chunks, and only this map can see it as one junction.
    std::unordered_map<u64, u32> m_nodeAt;
    std::vector<Vehicle> m_vehicles;
    std::vector<SignalPhase> m_signals;
    std::vector<u32> m_signalOfEdge;      // edge -> index into m_signals, or kNoSignal
    BoidsWeights m_boids;

    Vec2d m_denseAnchor{0, 0};
    f64 m_denseRadius = 400.0;
    std::vector<u32> m_denseEdges;   // inside the disc: where a departure reappears
    std::vector<u32> m_denseRing;    // the widened ring of last resort
    Vec2d m_denseDir{1, 0};          // the direction the camera is travelling
    Vec2d m_denseLast{0, 0};         // where it was when that was measured
    bool m_denseHasDir = false;

    f64 m_time = 0.0;
    f64 m_boidsAccum = 0.0;
    f64 m_laneAccum = 0.0;
    u64 m_turn = 0;
    u64 m_spawned = 0;
    u64 m_rngState = 0;
    u64 m_laneChanges = 0, m_laneChangeRefusals = 0;
    u64 m_nodesMerged = 0, m_nodesMergedAcrossChunks = 0;
    u64 m_derivedSignals = 0, m_deadEndUTurns = 0;
    u64 m_signalStops = 0, m_junctionHolds = 0, m_signalCrossings = 0, m_holdCaps = 0;
    u64 m_edgeCompletions = 0, m_deadEndHolds = 0, m_recycleFailures = 0;
    u64 m_graphContradictions = 0;
    u64 m_laneTick = 0;
    u64 m_promotions = 0, m_demotions = 0, m_anchorReleases = 0, m_orphanRepairs = 0;
    size_t m_promotedNow = 0;
    u64 m_boidsUpdates = 0, m_separationEvents = 0;
    u64 m_tipLimits = 0, m_hailStops = 0, m_hardBrakes = 0;
    u64 m_badState = 0, m_badSpeed = 0, m_badPosition = 0, m_laneViolations = 0;
    bool m_auditTrace = false;
    // Scratch reused every tick so that no tick allocates.
    std::vector<std::vector<u32>> m_edgeOrder;    // vehicles per edge, sorted by s
    // The edges that hold a vehicle this tick.  The city graph has ~170k edges
    // and the fleet is a few hundred vehicles, so every per-edge sweep walks
    // this list rather than the graph: the difference is three orders of
    // magnitude of work per tick, and it is what lets the 60-minute validation
    // run the real model on the real network instead of a corridor subset.
    std::vector<u32> m_activeEdges;
    std::vector<std::vector<u32>> m_buckets;      // 16 s-buckets per edge
    struct Arrival { u32 idx; u32 edge; u16 lane; };
    std::vector<Arrival> m_arrived;              // vehicles that finished an edge
    f64 m_minGap = 1e9;
    f64 m_maxLateralG = 0.0;

    static constexpr u32 kNoSignal = 0xFFFFFFFFu;

    // Deterministic stream, independent of the world's RNG so that adding
    // traffic behaviour can never shift the terrain or the fabric.
    u32 rng(u32 bound);
    f64 rng01();
};

// Lane geometry: 3.5 m centre to centre, the width a carriageway is divided
// into.  It lives here rather than in the .cpp because a physical driver has to
// know what "the carriageway" is to drive one (D-034).
inline constexpr f64 kLaneWidth = 3.5;

// How long the model holds an *external* vehicle at the end of an edge before it
// takes the vehicle back.  Mumbai jams last longer than this; a vehicle that
// cannot be placed anywhere on the graph, by contrast, is not in a jam and the
// model should not keep pretending it is (D-034).
inline constexpr f64 kAnchorHoldSeconds = 12.0;

// Minimum bumper-to-bumper separation the safety projection enforces.  A floor,
// not a target: IDM normally holds 2-15 m.
inline constexpr f64 kMinGap = 0.5;
// Admissions — a vehicle joining a lane, whether it drove in, changed into it, or
// was placed there by the population device — are held to the floor plus this
// margin, so that corrections, which hold exactly the floor, always have slack to
// work in and the invariant is never decided by the last bit of a double.
inline constexpr f64 kEntryMargin = 0.02;
// Within this distance of a stop line, a red or amber aspect is what is holding a
// vehicle — as opposed to the tail of a queue that happens to be near the line.
// The two are counted apart, because one is a signal working and the other is a
// junction working, and a validation that conflates them measures neither.
inline constexpr f64 kStopLineHold = 12.0;

// The spill of a queue that runs out of edge is NOT bounded by a floor.  Two
// floors were tried and both stacked bodies: clamping at s >= 0 stacked them at
// the edge mouth, and clamping at s >= -(length + 0.5) stacked them a metre
// later, because a floor is a position every pushed-back vehicle can reach and
// therefore a position they can all reach at once.  The correction clamps to the
// distance from the leader it can see and nothing else, which is monotone down the
// queue and can never put two bodies in the same place.

}  // namespace mumbai::sim
