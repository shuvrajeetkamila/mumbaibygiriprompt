// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/hydro.h
//
//  Monsoon hydrology over the built heightfield.
//
//  WHY THIS EXISTS.  Section 3 of the specification asks for a flood model in
//  which "Hindmata, King's Circle and the Andheri subway flood as an emergent
//  consequence of the DEM" — not from hand-placed triggers.  That rules out
//  painting water depth anywhere; the depth has to fall out of rain, the
//  terrain, the street drains and the tide.  This module is that model.  It is
//  the same code in two places: the pipeline runs it offline for the design
//  storms and bakes the result into the chunks for rendering, and the runtime
//  runs it live against the chunk heightfields for the weather system.
//
//  The chain is the standard one for urban flood analysis:
//
//      heightfield
//          │  priority-flood depression fill   (Barnes et al. 2014)
//          ▼
//      hydrologically connected surface  ──►  D8 flow directions
//          │                                        │
//          │                                        ▼
//          │                                 flow accumulation
//          │                                        │
//          ▼                                        ▼
//      sea / tide outlets                      channel network + watersheds
//          │                                        │
//          └───────────────┬────────────────────────┘
//                          ▼
//                 storm simulation (rain → runoff → routing → drains → outfall)
//
//  DECISIONS TAKEN HERE (see DOCS/02_DECISIONS.md D-026):
//
//   * The sea is an outlet boundary, not a basin.  The previous low-point
//     analysis ranked the open sea floor as the city's largest "drainage
//     basin" (560 km² at −13.5 m).  Here, cells below sea level are outlets
//     that seed the priority flood, and only land can accumulate water.
//   * Outflow through an outfall is tide-locked.  A land cell may discharge
//     into a sea cell only while its water surface stands above the tide; when
//     the tide is higher, the sea backs up into the cell instead.  This is the
//     mechanism behind Mumbai's most famous flooding, and it is why a storm
//     during a spring high tide is dramatically worse than the same storm at
//     low water.
//   * Drain capacity is a property of the street network, not a global
//     constant: municipal storm-water drains run under roads, so removal
//     capacity is rasterised from the road classes, and natural channels carry
//     a Manning-capacity cross-section scaled by their contributing area.
//     The MCGM drain database is not obtainable in this environment, so the
//     per-class design capacities are stated as assumptions in `DrainSpec`
//     rather than presented as surveyed values.
//   * Routing is a linear reservoir per cell with a Chézy-limited time
//     constant.  At 24 m cells an explicit kinematic wave is unconditionally
//     stable this way, conserves mass exactly, and still produces the
//     behaviour that matters: ponding where the terrain has no outlet, and
//     backwater when the outfall is surcharged.
// =============================================================================
#pragma once
#include "pipeline/mgeo/dem.h"

namespace mumbai::hydro {

// -----------------------------------------------------------------------------
//  Hydrological surface
// -----------------------------------------------------------------------------
// Derived once per world build (and once at runtime from the chunk heightfields).
// `flowDir` is a D8 code: 0=E 1=NE 2=N 3=NW 4=W 5=SW 6=S 7=SE, 8 = none (cell is
// an outlet or drains off the domain).
struct Surface {
    u32 width = 0, height = 0;
    f64 originX = 0, originZ = 0, step = 24.0;

    std::vector<f32> ground;      // the DEM as built: water levels and depths
    std::vector<f32> filled;      // depression-filled elevation: flow directions only
    std::vector<u8>  isBoundary;  // domain edge: drains freely, not tide-driven
    std::vector<u8>  flowDir;     // D8 direction (8 = outlet / no downstream)
    std::vector<u32> accum;       // contributing cells, including self
    std::vector<u32> watershed;   // watershed id, 0 = sea/outlet
    std::vector<u8>  isSea;       // open water: sea, creek, harbour
    std::vector<u8>  isShore;     // land cell with at least one sea neighbour
    std::vector<u8>  isChannel;   // contributing area above the channel threshold
    // A shore cell the tide can actually reach: one fronting a creek, backwater
    // or mangrove channel rather than the open coast.  Mumbai's sea face is
    // walled from Colaba to Dahisar, so the Arabian Sea does not spread over the
    // city at high water.  The tide enters through the creek mouths — Mahim Bay
    // and the Mithi, Thane Creek, Malad and Manori creeks — and that is the
    // mechanism that backs water up into Sion, Kurla, Chembur and the Andheri
    // subway.  A model that lets the open sea flood the shore belt gets a city
    // that is under water twice a day, and it hides the real signal behind it.
    std::vector<u8>  isTidalInlet;

    std::vector<u32> order;
    // How many cells were raised by DEM conditioning, for the QA record.
    u64 pitsCarved = 0;
    f64 pitFillMaxM = 0.0;       // priority-flood settling order (outlet = 0)

    size_t cellCount() const { return static_cast<size_t>(width) * height; }
    i64 indexOf(f64 x, f64 z) const {
        const i64 i = static_cast<i64>((x - originX) / step + 0.5);
        const i64 j = static_cast<i64>((z - originZ) / step + 0.5);
        if (i < 0 || j < 0 || i >= static_cast<i64>(width) || j >= static_cast<i64>(height)) return -1;
        return j * static_cast<i64>(width) + i;
    }
    // Nearest-cell sample of any per-cell field, -1 when outside.
    f32 valueAt(const std::vector<f32>& field, f64 x, f64 z) const {
        const i64 k = indexOf(x, z);
        return k < 0 ? 0.0f : field[static_cast<size_t>(k)];
    }
    f64 cellArea() const { return step * step; }
};

struct SurfaceOptions {
    f64 seaLevel = 0.0;
    // --- DEM conditioning ----------------------------------------------------
    // Priority-flood fill, then carve out the pits that needed less than this
    // much fill.  A 32 m sampling of a real city surface is full of closed
    // micro-depressions: the terrain synthesis carries 2.4 m of fractal detail,
    // and sampling that at 32 m leaves thousands of pits of a few decimetres,
    // each of which holds a pond for ever.  Measured on the 26 July 2005
    // analogue: 128 of 149 km² of ground above 10 m was under more than 15 cm of
    // water, spread evenly over the city instead of pooling in the low ground —
    // the signature of a model drowning in its own noise.  Filling shallow pits
    // is standard hydrological conditioning and it is what makes the *authored*
    // depressions (below) the only places that hold water on high ground.
    f64 pitFillDepth = 0.0;        // 0 disables; the city run sets 2.0 m
    // A depression that is known to exist is never conditioned away.  The
    // terrain's authored bowls (ControlPoint::flat) assert themselves here.
    std::function<bool(const Vec2d&)> keepDepression;
    // Height at least this far below the water plane marks open water.  The
    // value is not arbitrary: buildTerrain floors *land* at seaLevel - 0.75 and
    // deepens open water from seaLevel - 1.5 downwards, so 1.1 m is the
    // midpoint of a gap the data already contains.  Classifying by a shallower
    // threshold (0.5 m) swept the whole low-lying west coast — marsh, docks and
    // reclamation, which the DEM floors at -0.75 m — into the sea, and a city
    // whose coast is drawn 1.5 m too high floods nothing.
    f64 seaPickThreshold = 1.1;
    f64 channelAreaKm2 = 0.30;     // contributing area that defines a channel
    bool openBoundaries = true;    // domain edge drains freely (no fake lakes)

    // A distance-field callback is not needed; the classification is purely
    // height-based, which keeps the runtime path identical to the build path.
};

Surface buildSurface(const terrain::Heightfield& hf, const SurfaceOptions& opts);

// -----------------------------------------------------------------------------
//  Storm specification
// -----------------------------------------------------------------------------
// A block hyetograph (mm of rain per hour, one entry per hour) and a tide curve.
// The default severe scenario is the 26 July 2005 event — Mumbai's record storm,
// 944 mm in 24 h with a three-hour burst above 150 mm/h — which is the event the
// city's flood defences are measured against.  The moderate scenario is a
// routine monsoon day that nonetheless closes the Andheri subway.
struct StormSpec {
    const char* name = "moderate";
    std::vector<f64> rainMmPerHour;    // hourly intensities
    f64 tideAmplitude = 2.1;           // metres above MSL, spring range
    f64 tidePhaseHours = 0.0;          // 0 = high water at t=0
    f64 tidePeriodHours = 12.42;       // M2 tidal period
    f64 stepSeconds = 120.0;
    f64 initialLossMm = 5.0;           // interception / depression storage
    // Routing sub-steps per weather step.  The explicit scheme caps each cell's
    // outflow at half of what it holds per step, which at a five-minute step and
    // 32 m cells throttles conveyance to a fraction of what Manning's equation
    // says a hillside or a street should pass.  Water then stands on slopes that
    // would shed it in minutes — the model put half a metre on the ridges in a
    // burst, which is the clearest possible sign of a time-step artefact rather
    // than weather.  Sub-stepping leaves the weather cadence and the bookkeeping
    // alone and only tightens the routing.
    i32 routingSubSteps = 8;
};

StormSpec stormSpecModerate();
StormSpec stormSpecSevere();

// -----------------------------------------------------------------------------
//  Land surface properties fed to the model
// -----------------------------------------------------------------------------
// All rasters are per-surface-cell.  `impervious` is a runoff coefficient in
// [0.05, 1]; `drainCapacity` is the man-made removal rate in metres of water
// column per second (0 = no drain); `channelCapacity` is the natural-channel
// discharge in cubic metres per second that the cell can pass on.
struct LandSpec {
    std::vector<f32> impervious;
    std::vector<f32> drainCapacity;
    std::vector<f32> channelCapacity;
    // Manning roughness of the surface itself.  This is the single most
    // important number in the model: overland flow on a 32 m cell at n = 0.08
    // moves at centimetres per second and cannot carry a city's storm runoff,
    // so a uniform roughness makes every storm flood everything.  Real urban
    // runoff travels in gutters, kerb lines and channels — surfaces with n of
    // 0.012-0.020 — which is why paved cells get the low value here.  Mumbai's
    // streets genuinely are the drainage network.
    std::vector<f32> roughness;
    // Cells that belong to the *engineered* drainage network: streets with
    // gutters and kerbs, and the creeks and nullahs.  These are the paths a
    // city's storm water actually takes, and the model treats their hydraulic
    // gradient differently from open ground (see SimOptions::minDrainSlope).
    std::vector<u8> drainNetwork;
    // Design conveyance of the street drain at this cell, m3/s.  It bounds the
    // part of the flux that the *imposed* gutter gradient drives — a street
    // carries its design flow and no more — while flow driven by a real head of
    // water still follows Manning's equation, because a street with a metre of
    // water on it moves that water whether or not the pipe is surcharged.
    std::vector<f32> designFlow;
    // Where rainfall from a cell actually goes: the first drainage-network cell
    // on its downhill path, or the cell itself when it is already on the
    // network.  Roofs, yards and open ground discharge into the street, and the
    // street carries the water; a model that leaves the rain where it fell has
    // to move it across the surface afterwards, and on flat ground a uniform
    // sheet has no slope left to move it.  Rainfall is therefore booked to the
    // outlet cell, which is mass conserving by construction and is what makes
    // the network, rather than the sheet, the thing that drains the city.
    std::vector<u32> networkOutlet;

    void resize(size_t n, f32 imperviousDefault = 0.45f) {
        impervious.assign(n, imperviousDefault);
        drainCapacity.assign(n, 0.0f);
        channelCapacity.assign(n, 0.0f);
        roughness.assign(n, 0.10f);
        drainNetwork.assign(n, 0);
        designFlow.assign(n, 0.0f);
        networkOutlet.clear();
    }
};

// Design assumptions for the man-made network, in millimetres of water column
// removed per hour.  These stand in for the MCGM storm-water drain database
// (not obtainable here) and are deliberately conservative — a city that
// drained 100 mm/h would not flood, and Mumbai does.
struct DrainSpec {
    // How far a cell's rainfall may travel to reach the gutter: three cells, or
    // about a hundred metres.  Runoff from a roof or a courtyard is in the street
    // drain within that distance; a hillside's rain is not teleported to a road
    // two kilometres downhill, which is what an unbounded search did — and that
    // one line put a metre of water on the ridges during a burst.
    i32 maxOutletWalk = 3;
    f64 pavedRoughness = 0.016;        // asphalt/concrete with gutters
    f64 perviousRoughness = 0.10;      // open ground, gardens, hillside
    f64 channelRoughness = 0.030;      // lined nullah / creek
    f64 motorwayMmPerHour = 60.0;
    f64 arterialMmPerHour = 35.0;
    f64 residentialMmPerHour = 14.0;
    f64 serviceMmPerHour = 8.0;
    f64 naturalChannelDepth = 2.2;     // assumed channel depth for capacity
    f64 naturalChannelManningN = 0.030;
};

// Capacity of a natural channel from its contributing area: a trapezoidal
// cross-section whose width grows with the square root of the catchment, with
// the local bed slope, evaluated with Manning's equation.
f64 channelCapacityFor(u32 contributingCells, f64 cellArea, f64 bedSlope,
                       const DrainSpec& spec);

// -----------------------------------------------------------------------------
//  Result
// -----------------------------------------------------------------------------
struct FloodResult {
    std::vector<f32> maxDepth;        // metres, peak over the whole storm
    std::vector<f32> finalDepth;      // metres at the end of the simulation
    std::vector<f32> timeOfPeakHours; // hours from storm start
    f64 rainfallM3 = 0.0;             // rain that fell on land
    f64 drainedM3 = 0.0;              // removed by the man-made network
    f64 channelledM3 = 0.0;           // removed by natural channels
    f64 outfallM3 = 0.0;              // water that reached the sea or the open edge
    f64 tideExchangeM3 = 0.0;         // signed level-control exchange at the open sea
    f64 tideFloodM3 = 0.0;            // sea water pushed inland over the shoreline
    f64 floodedArea150mmKm2 = 0.0;    // peak depth > 0.15 m: roads impassable
    f64 floodedArea500mmKm2 = 0.0;    // peak depth > 0.50 m: ground floors
    f64 floodedArea1000mmKm2 = 0.0;   // peak depth > 1.00 m: 2005-event depths
    // Water retired below the routing threshold as cells dry out.  This is a
    // numerical closure term, not a physical flux; it is booked here so the
    // mass balance is exact and its size can be audited against rainfall.
    f64 filmLossM3 = 0.0;
    f64 storedM3 = 0.0;               // still on the surface at the end
    f64 peakStorageM3 = 0.0;
    f64 peakTideLevel = 0.0;
    f64 hoursSimulated = 0.0;
    size_t floodedCells = 0;          // cells whose peak depth exceeds 0.05 m
    f64 floodedAreaKm2 = 0.0;

    f32 depthAt(const Surface& s, f64 x, f64 z) const {
        const i64 k = s.indexOf(x, z);
        return k < 0 ? 0.0f : maxDepth[static_cast<size_t>(k)];
    }
};

struct SimOptions {
    // Land at or below this elevation is protected by the shore line rather than
    // open to it.  buildTerrain clamps sub-datum *land* to seaLevel - 0.75 m: the
    // reclamations behind Nariman Point and Cuffe Parade, the dock estates, the
    // BKC marsh.  Those cells are genuinely below the tide, and the tide reaches
    // them only through a breach or a surge, so the shoreline level rule must not
    // drive them.  They flood from rainfall — which is exactly why Hindmata and
    // King's Circle are the city's flood sites, and why a model that lets the
    // tide into them drowns 40 % of the city in a routine monsoon.
    f64 landFloor = -0.75;
    f64 minDepth = 1e-4;              // below this a cell is treated as dry
    f64 velocityCap = 3.0;            // m/s, keeps urban routing subcritical
    // Imposed minimum hydraulic gradient on the drainage network.  A diffusive
    // wave on flat ground stalls: once rain has settled into a near-uniform
    // sheet there is no water-surface slope left to drive it, and the model
    // keeps water that a real city removes.  Mumbai's streets are graded to
    // their gutters — a fall of roughly 1 in 700 is the design standard for a
    // kerb line — so on network cells the head across a face is taken to be at
    // least this gradient, and only ever downhill.  Open ground gets no such
    // help, which is why hillsides, mangroves and the polders still pond.
    f64 minDrainSlope = 0.0015;
    // Crown of the sea wall, metres above datum.  Below this level the open
    // coast is protected and only the creek mouths admit the tide; above it the
    // sea comes over the wall along the whole frontage.  MCGM's sea wall was
    // rebuilt after 2005 to a crest of roughly four metres, which is why the
    // 2005 event is the one that overtopped it and the years since are not.
    f64 seawallCrest = 4.0;
    // Gravity outfall structure: invert height above datum and the design head
    // it discharges against.  A storm drain does not lose capacity as soon as
    // the tide lifts off zero — it flows full bore until the sea reaches its
    // invert, and then backs up over the next metre and a half.  Derating from
    // mean sea level instead is what made a routine monsoon day flood the whole
    // city in the first version of this model.
    f64 outfallInvert = 0.6;
    f64 outfallDesignHead = 1.6;
    f64 roughnessManningsN = 0.08;    // used only when LandSpec::roughness is empty
    bool tideLockOutfalls = true;
};

// Runs the storm.  The mass balance is exact by construction — every transfer
// across the domain boundary is accumulated as it happens, and water is never
// stored in a cell that represents the boundary itself — so callers can assert
//
//      rainfall + tideFlood  ==  drained + outfall + stored
//
// (`tideExchange` is the level-control term that keeps the open sea at the tide
// and is internal to the boundary, so it is not part of the city's balance.)
//
// to f64 rounding.  `tideExchangeM3` is signed: positive when the rising tide
// pushes water into the city, negative when the falling tide takes it back out.
FloodResult simulate(const Surface& surface, const LandSpec& land, const StormSpec& storm,
                     const SimOptions& options = {});

// NOTE on the two elevations.  `filled` exists only to define drainage
// directions: without it, every closed depression is a place where flow cannot
// be routed at all.  Water levels, depths and ponding must always use `ground`
// (the DEM as built) — a pit whose filled level is its 14 m rim still holds
// water from 4 m upwards, and using the filled surface by mistake is what makes
// a flood model drain every puddle it is supposed to predict.

// Convenience for the pipeline: rasterise roads / buildings into a LandSpec.
struct CoverageInput {
    // Any callback returning true means the cell is covered by that feature.
    std::function<bool(const Vec2d&)> isRoad;
    std::function<bool(const Vec2d&)> isBuilding;
    std::function<bool(const Vec2d&)> isGreen;
    // Road class at a point (0 = none), used for the drain capacity map.
    std::function<i32(const Vec2d&)> roadClass;
};
LandSpec rasteriseLand(const Surface& surface, const CoverageInput& input, const DrainSpec& spec);

} // namespace mumbai::hydro
