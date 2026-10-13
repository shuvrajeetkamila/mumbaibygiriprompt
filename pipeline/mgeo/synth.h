// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/synth.h
//
//  Urban fabric synthesis.  The raw OSM extract available offline (see
//  DOCS/D-020) carries the complete tag inventory of Mumbai but only a tenth of
//  the node geometry, so road centrelines and building footprints have to be
//  reconstructed around the data that IS authoritative:
//
//      * the 24 real BMC ward polygons (land/sea edge, ward boundaries),
//      * the real slum-cluster and SRA polygons (informal fabric extent),
//      * authored arterial corridors and railway alignments on real alignments,
//      * published ward-level statistics (density, floor counts, fleet mix).
//
//  The generator produces a street lattice and building parcels *inside the real
//  ward polygons*, oriented along the local arterial bearing and jittered per
//  street so the result reads as a grown Indian city rather than a CAD grid.
//  Every generated element is tagged so the runtime, the QA report and the
//  documentation can state exactly what is surveyed and what is synthesised.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include "pipeline/mgeo/geometry.h"

#include <functional>
#include <vector>

namespace mumbai::synth {

// --- street lattice ----------------------------------------------------------

struct StreetSeg {
    Vec2d a, b;                  // world space, metres
    f64 halfWidth = 3.0;         // half carriageway width
    u8  rank = 1;                // 0 = collector, 1 = local
};

struct FabricOut {
    std::vector<StreetSeg> streets;
    std::vector<geo2d::Ring> footprints;     // world space, closed, CCW
    std::vector<Vec2d> footprintCentre;      // parallel to footprints
};

// Catmull-Rom through the control points, resampled so that no edge exceeds
// `maxEdge`.  Used to turn the authored station/junction knots into the smooth
// alignment a corridor actually follows.
std::vector<Vec2d> smoothThrough(const std::vector<Vec2d>& control, f64 maxEdge);

// Builds the street lattice and the building parcels inside one land polygon.
//
//  * `bearing`   orientation of the local street system (radians, world XZ)
//  * `spacing`   distance between parallel streets
//  * `streetJitter` fraction of `spacing` by which each street wanders
//  * `arterialDist` generated streets closer than this to an arterial chain are
//                dropped, so synthesised fabric never duplicates a real road
//  * `coverage`  fraction of parcels that receive a building (0..1)
//  * `margin`    setback from the street centreline to a footprint edge
//  * `lots`      maximum parcels per lattice cell per axis
//
// `accept` filters candidate footprints (ward membership, water, exclusion
// zones).  Cells whose centre is outside the polygon are skipped entirely.
FabricOut buildUrbanFabric(const geo2d::Polygon& land, f64 bearing, f64 spacing,
                           f64 streetJitter, f64 arterialDist,
                           const std::vector<std::vector<Vec2d>>& arterials,
                           f64 coverage, f64 margin, u32 lots,
                           const std::function<bool(const Vec2d&)>& accept,
                           Rng& rng);

// Deterministic pseudo-random unit value for a lattice line index, so the same
// street wanders identically in every pass that touches it.
f64 latticeJitter(i64 index, u64 seed);

}  // namespace mumbai::synth
