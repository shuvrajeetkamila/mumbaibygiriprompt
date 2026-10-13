// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/signal_warrant.h
//
//  Which junctions deserve a signal, and on what plan.
//
//  This rule lives in one header because two very different pieces of the
//  system have to agree on it: the offline pipeline, which bakes the phases it
//  can express into the chunks, and the runtime, which has to phase the
//  junctions the chunk format *cannot* express — a junction whose arms live in
//  two neighbouring cells is two nodes in two files, and only the runtime, with
//  the merged graph in hand, can see it as one junction.  Two copies of a
//  warrant would be two different cities.
//
//  Counts are of *arms*, not of edges: at a junction, every road that reaches
//  it contributes one arm, whether the source store the segment as incoming or
//  outgoing, and whether it is one-way or two-way.  A road that merely passes
//  through is two arms on one alignment and is not a junction at all — the
//  caller has already refused to classify it.
//
//  The warrant, in the shapes Mumbai actually signals (D-028):
//    * two or more classified arms — an arterial meeting an arterial, or an
//      arterial passing through a four-arm junction.  Every junction on S.V.
//      Road, L.B.S. Marg or the Western Express Highway is signalised, and the
//      junctions that are not are the ones with no fourth arm;
//    * one classified arm at a four-arm junction: a four-arm junction on an
//      arterial is signalised in Mumbai even where the cross street is a lane;
//    * three or more collector-or-better arms (tertiary and above, i.e. three
//      classified approaches of any rank);
//    * two four-lane approaches — a wide junction with no class, which is how a
//      signal appears in the middle of a market road nobody classified.
// =============================================================================
#pragma once
#include "engine/core/core.h"

namespace mumbai {
namespace navsignal {

// -----------------------------------------------------------------------------
//  Plan shape
// -----------------------------------------------------------------------------
// The cycle is 90 s, split into two approach axes 45 s apart.  A phase is green
// plus amber plus all-red, so the green cannot exceed 45 - 3 - 1.5 = 40.5 s: the
// earlier 42 s green and 45 s offset overlapped the crossing phase's green by
// 1.5 s, which is exactly the failure a signal model must not have.  400
// deciseconds of green leaves the half-second of slack the arithmetic wants.
constexpr u32 kCycleDeciseconds = 900;
constexpr u32 kGreenDeciseconds = 400;
constexpr u32 kCrossOffsetDeciseconds = 450;

enum class Warrant : u8 {
    None = 0,
    TwoClassified,             // two or more classified arms
    ClassifiedWideJunction,    // a classified arm at a four-arm junction
    ThreeCollectors,           // three or more classified approaches
    TwoWide,                   // two four-lane approaches
    Count,
};

struct JunctionShape {
    u32 major = 0;       // arms of class motorway .. secondary
    u32 tertiary = 0;    // arms of class tertiary or unclassified
    u32 wide = 0;        // arms with four or more lanes
    u32 degree = 0;      // total arms at the junction
};

inline Warrant classifyJunction(const JunctionShape& s) {
    if (s.major >= 2) return Warrant::TwoClassified;
    if (s.major >= 1 && s.degree >= 4) return Warrant::ClassifiedWideJunction;
    if (s.major + s.tertiary >= 3) return Warrant::ThreeCollectors;
    if (s.wide >= 2) return Warrant::TwoWide;
    return Warrant::None;
}

inline bool signalWarranted(const JunctionShape& s) {
    return classifyJunction(s) != Warrant::None;
}

inline const char* warrantName(Warrant w) {
    switch (w) {
        case Warrant::TwoClassified: return "two-classified";
        case Warrant::ClassifiedWideJunction: return "classified-wide";
        case Warrant::ThreeCollectors: return "three-collectors";
        case Warrant::TwoWide: return "two-wide";
        default: return "none";
    }
}

// Which arms of a junction run along the first axis and which cross it.  A road's
// bearing from the junction is all that decides it: arms within 45° of the
// junction's first classified arm open together, and the rest open half a cycle
// later.
inline bool armRunsAlong(const Vec2d& armDir, const Vec2d& axis) {
    return std::fabs(armDir.x * axis.x + armDir.y * axis.y) >= 0.70710678;
}

}  // namespace navsignal
}  // namespace mumbai
