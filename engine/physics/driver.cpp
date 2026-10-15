// =============================================================================
//  engine/physics/driver.cpp — see driver.h for the design notes.
// =============================================================================
#include "engine/physics/driver.h"

#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace mumbai::phys {

namespace {

constexpr f64 kPi = 3.14159265358979323846;
// Bodies are placed this far above the surface they rest on and settle onto
// their suspension.  Small: the placement is already exact, and a large
// clearance would be a drop test for every vehicle the model hands over.
constexpr f64 kAttachClearance = 0.03;

// Wraps an angle to (-pi, pi]: a heading error of 350 degrees is a 10 degree
// correction to the left, and a driver that cannot see that drives in circles.
f64 wrapPi(f64 a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

// The body's heading, in the world's yaw convention: `atan2(forward.x,
// forward.z)`, which is what `spawnVehicle` takes and what the renderer draws.
f64 headingOf(World& world, const Vehicle& v) {
    const J::Quat q = world.bodies().GetRotation(v.body.id);
    const J::Vec3 fwd = q * J::Vec3::sAxisZ();
    return std::atan2(static_cast<f64>(fwd.GetX()), static_cast<f64>(fwd.GetZ()));
}

}  // namespace

f64 PhysicalFleet::surfaceAt(World& world, f64 x, f64 z, f64 modelHeight) {
    // The model knows where its road is — the nav graph's node elevations,
    // interpolated along the edge — so the search starts there rather than at an
    // assumed datum.  It is not a one-line cast, because a ray that starts inside
    // a collider reports a hit at its own origin, and a building overhanging a
    // lane would then be read as the ground under it (the first version of this
    // placed a taxi on the roof of a Marine Drive block, 4 m up, and the solver
    // reported the overlap as 3.4 m of penetration).
    //
    // Sampling heights around the model's own number and keeping the nearest
    // *genuine* hit is both robust and explainable: a road is within a couple of
    // metres of the elevation the graph gives it, whichever side of the surface
    // the sample happens to fall on.
    f64 best = std::numeric_limits<f64>::quiet_NaN();
    f64 bestDistance = 1e30;
    const f64 drops[5] = {0.4, 1.2, 2.5, 6.0, 12.0};
    for (const f64 drop : drops) {
        const f64 from = modelHeight + drop;
        const f64 h = world.groundHeightStatic(
            Vec3(static_cast<f32>(x), static_cast<f32>(from), static_cast<f32>(z)), drop + 0.7);
        if (!std::isfinite(h)) continue;
        if (from - h < 0.05) continue;               // started inside something
        const f64 distance = std::fabs(h - modelHeight);
        if (distance < bestDistance) { bestDistance = distance; best = h; }
    }
    // A road is within 3 m of the elevation the graph gives it.  Anything further
    // is not this vehicle's road — it is a roof, a deck or the seabed — and the
    // model's own number is a better answer than a surface the model cannot
    // explain.
    if (std::isfinite(best) && bestDistance <= 3.0) return best;
    return modelHeight;
}

namespace {

// Collects every static overlap of the probe box and keeps the deepest.  The
// dynamic bodies are filtered out by the body filter, so a platoon of traffic
// does not read as a wall: this probe is about the *city*.
class DeepestOverlap final : public J::CollideShapeCollector {
public:
    explicit DeepestOverlap(f64 tolerance) : m_tolerance(tolerance) {}
    void AddHit(const J::CollideShapeResult& inResult) override {
        if (inResult.mPenetrationDepth > m_tolerance) {
            m_deepest = std::max(m_deepest, static_cast<f64>(inResult.mPenetrationDepth));
            m_body = inResult.mBodyID2;
            const J::RVec3& at = inResult.mContactPointOn2;
            m_at = Vec3(static_cast<f32>(at.GetX()), static_cast<f32>(at.GetY()),
                        static_cast<f32>(at.GetZ()));
            ForceEarlyOut();
        }
    }
    f64 deepest() const { return m_deepest; }
    bool any() const { return m_deepest > 0.0; }
    u32 body() const { return m_body.GetIndex(); }
    const Vec3& at() const { return m_at; }
private:
    f64 m_tolerance = 0.0;
    f64 m_deepest = 0.0;
    J::BodyID m_body;
    Vec3 m_at{};
};

// One line that says what a body *is*: where it sits and how big it is.  A body
// id alone is not a diagnosis.
std::string describeBody(World& world, J::BodyID bid, const Vec3& atWorld) {
    // The id is taken whole.  A `BodyID` carries a sequence number beside the
    // index, and an id rebuilt from the index alone resolves to nothing at all —
    // which sent this function down its "unresolved" path and reported the terrain
    // for every contact in the city (D-034, measured).
    const J::Body* body = bid.IsInvalid() || bid.GetIndex() >= world.system().GetNumBodies()
                              ? nullptr
                              : world.system().GetBodyLockInterfaceNoLock().TryGetBody(bid);
    if (body == nullptr) {
        // The narrow phase reported an id this system does not resolve (a compound
        // child, or a sequence number that has since been reused).  The contact
        // point is not a guess: find the body whose bounds contain it, smallest
        // first, so the answer is the shape actually touched rather than the
        // terrain every shape sits in.
        const J::Body* best = nullptr;
        f64 bestVolume = 1e30;
        J::BodyIDVector all;
        world.system().GetBodies(all);
        for (const J::BodyID other : all) {
            const J::Body* candidate = world.system().GetBodyLockInterfaceNoLock().TryGetBody(other);
            if (candidate == nullptr || !candidate->IsStatic()) continue;
            const J::AABox& b = candidate->GetWorldSpaceBounds();
            if (atWorld.x < b.mMin.GetX() || atWorld.x > b.mMax.GetX()) continue;
            if (atWorld.y < b.mMin.GetY() || atWorld.y > b.mMax.GetY()) continue;
            if (atWorld.z < b.mMin.GetZ() || atWorld.z > b.mMax.GetZ()) continue;
            const f64 volume = (static_cast<f64>(b.mMax.GetX()) - b.mMin.GetX()) *
                               (static_cast<f64>(b.mMax.GetY()) - b.mMin.GetY()) *
                               (static_cast<f64>(b.mMax.GetZ()) - b.mMin.GetZ());
            if (volume < bestVolume) { bestVolume = volume; best = candidate; }
        }
        if (best == nullptr) return "a body the system does not resolve, and nothing contains the contact";
        body = best;
    }
    // What the body *is*, not only where it is.  A compound's bounding box is a
    // cell wide, so "bounds x -10240.0..-9166.1" says nothing about what a car
    // was blocked by; the shape's own sub-type and its child count do, and they
    // are the difference between a kerb, a deck and a road ribbon (D-035).
    std::string shape;
    if (const J::Shape* sh = body->GetShape()) {
        const char* name = "shape";
        switch (sh->GetSubType()) {
            case J::EShapeSubType::Sphere: name = "sphere"; break;
            case J::EShapeSubType::Box: name = "box"; break;
            case J::EShapeSubType::Triangle: name = "triangle"; break;
            case J::EShapeSubType::Capsule: name = "capsule"; break;
            case J::EShapeSubType::Cylinder: name = "cylinder"; break;
            case J::EShapeSubType::ConvexHull: name = "convex hull"; break;
            case J::EShapeSubType::StaticCompound: name = "static compound"; break;
            case J::EShapeSubType::MutableCompound: name = "mutable compound"; break;
            case J::EShapeSubType::RotatedTranslated: name = "rotated shape"; break;
            case J::EShapeSubType::Scaled: name = "scaled shape"; break;
            case J::EShapeSubType::Mesh: name = "mesh"; break;
            case J::EShapeSubType::HeightField: name = "heightfield"; break;
            default: break;
        }
        char sbuf[96];
        if (sh->GetType() == J::EShapeType::Compound) {
            const J::CompoundShape* cs = static_cast<const J::CompoundShape*>(sh);
            std::snprintf(sbuf, sizeof(sbuf), "%s of %u children", name, cs->GetNumSubShapes());
        } else {
            std::snprintf(sbuf, sizeof(sbuf), "%s", name);
        }
        shape = sbuf;
    }
    const J::RVec3 p = body->GetCenterOfMassPosition();
    const J::AABox& bounds = body->GetWorldSpaceBounds();
    char buf[320];
    std::snprintf(buf, sizeof(buf),
                  "body %u (%s) at (%.1f, %.1f, %.1f), bounds x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f",
                  body->GetID().GetIndex(), shape.c_str(), p.GetX(), p.GetY(), p.GetZ(),
                  static_cast<f64>(bounds.mMin.GetX()),
                  static_cast<f64>(bounds.mMax.GetX()), static_cast<f64>(bounds.mMin.GetY()),
                  static_cast<f64>(bounds.mMax.GetY()), static_cast<f64>(bounds.mMin.GetZ()),
                  static_cast<f64>(bounds.mMax.GetZ()));
    return std::string(buf);
}

// What the box probes are *for*: the built city — a wall, a kerb, a balcony, a
// deck slab, a cable pylon.  The ground is a different question.  It is a
// heightfield and a road mesh, it is answered exactly by the rays that measure
// the surface under the box's own corners (`surfaceAt`, the footprint step), and
// asking Jolt for the penetration of a box *inside* a triangle soup is asking
// the EPA solver for a depth it does not always have: on a degenerate triangle
// it asserts (`JPH_ASSERT(!ioV.IsNearZero())`, EPAPenetrationDepth.h) and the
// process dies of a breakpoint trap — measured twice in one session, RC 105,
// once inside `surveyLanes` and once in the spawn gate (D-034).  Filtering the
// triangle shapes out of the probe removes the failure mode *and* asks a
// cleaner question: the probe answers "is anything built standing here", and the
// ground is measured by the rays that can actually see a kerb.
struct BuiltShapeFilter : public J::ShapeFilter {
    static bool isGround(const J::Shape& s) {
        const J::EShapeSubType t = s.GetSubType();
        return t == J::EShapeSubType::Mesh || t == J::EShapeSubType::HeightField;
    }
    bool ShouldCollide(const J::Shape* inShape2, const J::SubShapeID&) const override {
        return !isGround(*inShape2);
    }
    bool ShouldCollide(const J::Shape* inShape1, const J::SubShapeID&, const J::Shape* inShape2,
                       const J::SubShapeID&) const override {
        return !isGround(*inShape1) && !isGround(*inShape2);
    }
};

// The probe report carries a bare index (`ProbeReport::body`); that id keeps the
// old behaviour, including the unresolved-id fallback above.
std::string describeBody(World& world, u32 id, const Vec3& atWorld) {
    return describeBody(world, J::BodyID(id), atWorld);
}

}  // namespace

f64 PhysicalFleet::staticOverlapAt(World& world, sim::Archetype kind, const Vec2d& p, f64 y,
                                  f64 yaw) {
    const sim::ArchetypeSpec& spec = sim::archetypeSpec(kind);
    const J::BoxShape box(J::Vec3(static_cast<f32>(spec.width * 0.5),
                                  static_cast<f32>(chassisHalfHeight(kind)),
                                  static_cast<f32>(spec.length * 0.5)));
    const J::RMat44 transform = J::RMat44::sRotationTranslation(
        J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)),
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)));
    J::CollideShapeSettings settings;
    settings.mMaxSeparationDistance = 0.0f;
    DeepestOverlap collector(0.0);
    const StaticBodyFilter filter;   // the city only: a platoon is not a wall
    const BuiltShapeFilter shapeFilter;   // the built city only: the ground is a ray's job
    world.system().GetNarrowPhaseQuery().CollideShape(
        &box, J::Vec3::sReplicate(1.0f), transform, settings,
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)), collector,
        {}, {}, filter, shapeFilter);
    return static_cast<f64>(collector.deepest());
}

f64 PhysicalFleet::wholeVehicleOverlapAt(World& world, sim::Archetype kind, const Vec2d& p,
                                             f64 groundY, f64 yaw) {
    const sim::ArchetypeSpec& spec = sim::archetypeSpec(kind);
    const f64 rest = chassisRestHeight(kind);
    const f64 halfHeight = chassisHalfHeight(kind);
    // From the road surface to the chassis roof, at the vehicle's own width and
    // length.  The wheels sit inside the width (measured from `spawnVehicle`'s
    // own wheel placement), so the union is a box and the probe stays the shape
    // it stands in for.
    const f64 top = rest + halfHeight;
    const J::BoxShape box(J::Vec3(static_cast<f32>(spec.width * 0.5),
                                  static_cast<f32>(top * 0.5),
                                  static_cast<f32>(spec.length * 0.5)));
    const J::RMat44 transform = J::RMat44::sRotationTranslation(
        J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)),
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(groundY + top * 0.5),
                 static_cast<f32>(p.y)));
    J::CollideShapeSettings settings;
    settings.mMaxSeparationDistance = 0.0f;
    DeepestOverlap collector(0.0);
    const StaticBodyFilter filter;        // the city only, and only what is built
    const BuiltShapeFilter shapeFilter;
    world.system().GetNarrowPhaseQuery().CollideShape(
        &box, J::Vec3::sReplicate(1.0f), transform, settings,
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(groundY + top * 0.5),
                 static_cast<f32>(p.y)),
        collector, {}, {}, filter, shapeFilter);
    return static_cast<f64>(collector.deepest());
}

f64 PhysicalFleet::anyOverlapAt(World& world, sim::Archetype kind, const Vec2d& p, f64 y,
                                f64 yaw) {
    // Every body, the fleet's own included.  A vehicle authored on top of another
    // vehicle is a solver overlap from its first tick: Jolt pushes two 16 t bodies
    // apart over the next second and the run measures the overlap as penetration,
    // which is what the first 24-vehicle run did (0.1007 m, two double-deckers,
    // D-034).  The city-only probe cannot see that, because the other vehicle is
    // dynamic and was created before this one.
    const sim::ArchetypeSpec& spec = sim::archetypeSpec(kind);
    const J::BoxShape box(J::Vec3(static_cast<f32>(spec.width * 0.5),
                                  static_cast<f32>(chassisHalfHeight(kind)),
                                  static_cast<f32>(spec.length * 0.5)));
    const J::RMat44 transform = J::RMat44::sRotationTranslation(
        J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)),
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)));
    J::CollideShapeSettings settings;
    settings.mMaxSeparationDistance = 0.0f;
    DeepestOverlap collector(0.0);
    const J::BodyFilter filter;      // everything: the city and the fleet
    const BuiltShapeFilter shapeFilter;   // ...minus the ground, which is not a body
    world.system().GetNarrowPhaseQuery().CollideShape(
        &box, J::Vec3::sReplicate(1.0f), transform, settings,
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)), collector,
        {}, {}, filter, shapeFilter);
    return static_cast<f64>(collector.deepest());
}

// Does the vehicle's own chassis box fit here?  The question a ray cannot answer:
// eight horizontal rays at wheel height pass over a kerb, under a balcony and
// beside a wall, and a vehicle that is 2.6 m wide and 4.3 m tall is not a point.
// The box is the chassis box the body is created with, and the shapes it is
// tested against are the built ones (see `BuiltShapeFilter`): the ground under
// the footprint is measured by the corner rays the caller has already cast.
namespace {

// The raw measurement: the nearest genuine collision surface under (x, z),
// searched from successively higher origins.  No model fallback and no tolerance:
// a survey that accepted the model's own number wherever the collision disagreed
// could never report a disagreement larger than the tolerance, which is exactly
// the check-that-cannot-fail this project keeps finding (D-034).
f64 rawGroundAt(World& world, f64 x, f64 z, f64 near, bool* structureOnly = nullptr) {
    // The search starts above the model's road and walks down, and the candidate
    // closest to the model's height wins — which, without a band, means the ray
    // fired from near + 30 m can land on the *top of a wall* and report it as the
    // ground 29.5 m above the lane.  Measured: that is exactly the survey's worst
    // "height disagreement", 63,358 samples of it, and it is the instrument's
    // failure, not the city's (D-036).  A lane's ground is within a metre or so of
    // the model's road; a surface further above that is a structure, and it is
    // the headroom ray's question, not this one.
    constexpr f64 kGroundAboveM = 1.5;
    constexpr f64 kGroundBelowM = 4.0;
    const f64 drops[5] = {1.2, 2.5, 5.0, 12.0, 30.0};
    f64 best = 0.0;
    bool found = false, sawStructure = false;
    for (const f64 drop : drops) {
        const f64 from = near + drop;
        const f64 reach = drop + 4.0;
        const World::RayHit h = world.rayCastStatic(
            Vec3(static_cast<f32>(x), static_cast<f32>(from), static_cast<f32>(z)),
            Vec3(0.0f, -1.0f, 0.0f), reach);
        if (!h.hit) continue;
        const f64 at = from - h.fraction * reach;
        if (from - at < 0.05) continue;          // the ray started inside something
        if (at > near + kGroundAboveM) { sawStructure = true; continue; }
        if (at < near - kGroundBelowM) { sawStructure = true; continue; }
        if (!found || std::fabs(at - near) < std::fabs(best - near)) {
            best = at;
            found = true;
        }
    }
    if (!found && structureOnly) *structureOnly = sawStructure;
    return found ? best : std::numeric_limits<f64>::quiet_NaN();
}

}  // namespace

PhysicalFleet::LaneSurvey PhysicalFleet::surveyLanes(World& world, sim::Traffic& traffic,
                                                     f64 spacing) {
    LaneSurvey out;
    const sim::ArchetypeSpec& spec = sim::archetypeSpec(sim::Archetype::Car);
    const std::vector<sim::SimEdge>& edges = traffic.edges();
    for (size_t ei = 0; ei < edges.size(); ++ei) {
        const sim::SimEdge& e = edges[ei];
        if (e.length <= 1.0f) continue;
        const u32 lanes = e.lanes == 0 ? 1 : e.lanes;
        for (f64 s = spacing * 0.5; s < e.length; s += spacing) {
            const u32 lane = static_cast<u32>(out.samples % lanes);
            const Vec2d at = traffic.laneCentre(e, lane, s);
            const f64 model = traffic.laneHeight(e, s);
            ++out.samples;
            bool structureOnly = false;
            const f64 measured = rawGroundAt(world, at.x, at.y, model, &structureOnly);
            if (std::isnan(measured)) {
                // Nothing in the band.  "Only structures" is a different fact from
                // "nothing at all": a lane point whose only collision surface is a
                // wall 20 m up is a lane the collision layer has no road under.
                if (structureOnly) ++out.structureOnly;
                ++out.noSurface;
                // Nothing in the band under the model's road: either a structure
                // hangs over it (the road is under something) or the collision
                // layer has no road here at all.  Two different facts, two
                // counters, one ray (D-036).
                const World::RayHit over = world.rayCastStatic(
                    Vec3(static_cast<f32>(at.x), static_cast<f32>(model + 0.2),
                         static_cast<f32>(at.y)),
                    Vec3(0.0f, 1.0f, 0.0f), 14.0f);
                if (over.hit) {
                    ++out.underStructure;
                    const f64 clear = over.fraction * 14.0;
                    if (clear < out.minUnderStructureM) out.minUnderStructureM = clear;
                    const u32 bin = clear < 0.5 ? 0u : clear < 1.5 ? 1u : clear < 3.0 ? 2u
                                                 : clear < 8.0 ? 3u : 4u;
                    ++out.overBins[bin];
                } else {
                    ++out.noCollisionAtAll;
                }
                continue;
            }
            const f64 ground = measured;
            const f64 error = std::fabs(ground - model);
            // An elevated road's model height is its *deck*, and the ground under
            // it is tens of metres lower: comparing the two and calling the
            // difference a disagreement is a category error in the instrument.
            // Bridge edges are therefore measured apart, and a bridge sample whose
            // nearest surface is metres below the deck is a deck the collision
            // layer does not have — named, not averaged into a headline (D-036).
            const bool bridge = e.bridge;
            // Where the model's road is metres above every surface under it, the
            // sample is an elevated road: the deck is the assertion, not the bed.
            const f64 drop = model - ground;
            if (drop > 3.0) {
                ++out.elevatedSamples;
                if (drop > out.worstElevatedDropM) {
                    out.worstElevatedDropM = drop;
                    out.worstElevatedAt = at;
                }
                // A deck at the model's height is a surface within a quarter metre
                // of it, found by asking from just above the model's own road.
                const World::RayHit deck = world.rayCastStatic(
                    Vec3(static_cast<f32>(at.x), static_cast<f32>(model + 0.5),
                         static_cast<f32>(at.y)),
                    Vec3(0.0f, -1.0f, 0.0f), 1.0f);
                if (deck.hit) ++out.elevatedWithDeck;
                continue;      // counted; the ground comparison is not about this road
            }
            if (ground - model > 3.0) {
                // The collision is above the model's road here: the surface found
                // is a structure the model does not know it is under.
                ++out.collisionAbove;
                if (ground - model > out.worstCollisionAboveM) {
                    out.worstCollisionAboveM = ground - model;
                    out.worstCollisionAboveAt = at;
                }
                continue;      // likewise not a ground comparison
            }
            if (bridge) {
                ++out.bridgeSamples;
                if (error < 0.25) ++out.bridgeWithinQuarter;
                if (error < 0.5) ++out.bridgeWithinHalf;
                if (error < 2.0) ++out.bridgeWithinTwo;
                if (error > 3.0) {
                    ++out.bridgeNoDeck;
                } else if (error > out.worstBridgeErrorM) {
                    out.worstBridgeErrorM = error;
                    out.worstBridgeAt = at;
                }
            } else {
                if (error < 0.25) ++out.withinQuarter;
                if (error < 0.5) ++out.withinHalf;
                if (error < 2.0) ++out.withinTwo;
                if (error > out.worstErrorM) {
                    out.worstErrorM = error;
                    out.worstAt = at;
                    out.worstEdge = ei;
                }
            }
            // A step under the footprint is a road the model has and the collision
            // does not (or the other way round).
            f64 low = ground, high = ground;
            const f64 halfL = spec.length * 0.5;
            const f64 halfW = spec.width * 0.5;
            const f64 dx[4] = {halfL, -halfL, 0.0, 0.0};
            const f64 dz[4] = {0.0, 0.0, halfW, -halfW};
            for (int k = 0; k < 4; ++k) {
                const f64 h = surfaceAt(world, at.x + dx[k], at.y + dz[k], ground);
                low = std::min(low, h);
                high = std::max(high, h);
            }
            if (high - low > m_settings.footprintStepTolerance) ++out.stepped;
            // And can a vehicle of the smallest class that uses this road be put
            // there at all?
            // Headroom: what is above the lane, straight up from the lane centre.
            // Recorded for every sample — a street with a low bridge over it is a
            // street only a motorcycle uses — and used below to explain a block.
            const World::RayHit up = world.rayCastStatic(
                Vec3(static_cast<f32>(at.x), static_cast<f32>(ground + 0.2),
                     static_cast<f32>(at.y)),
                Vec3(0.0f, 1.0f, 0.0f), 12.0f);
            if (up.hit) {
                const f64 clear = up.fraction * 12.0;
                ++out.headroomSamples;
                if (clear < out.minHeadroomM) {
                    out.minHeadroomM = clear;
                    out.minHeadroomAt = at;
                    out.minHeadroomBody = up.body.GetIndex();
                }
                if (clear < 2.6) ++out.headroomUnderCar;
                if (clear < 2.0) ++out.headroomUnderAny;
            }
            const f64 probeY = ground + chassisRestHeight(sim::Archetype::Car) + kAttachClearance;
            ProbeReport probe;
            if (!boxFits(world, at, probeY, std::atan2(e.dir.x, e.dir.y), sim::Archetype::Car,
                         m_settings.probePenetrationTolerance, &probe)) {
                ++out.blocked;
                if (error < m_settings.heightTolerance) ++out.blockedHeightAgrees;
                else ++out.blockedHeightDiffers;
                // A block with less than a car's height of clearance is a vehicle
                // that does not fit under the structure, which is a fact about the
                // city; one without is a shape in the way, which is a fact about
                // the collision (D-036).
                if (up.hit && up.fraction * 12.0 < 2.6) ++out.headroomHeightAgrees;
                if (probe.depth > out.blockedDepthM) {
                    out.blockedDepthM = probe.depth;
                    out.blockedAt = at;
                    out.blockedBody = probe.body;
                    out.blockedWhat = describeBody(world, probe.body, probe.at);
                }
                // Keep a few, spread through the survey, so a run can say *what*
                // kind of blockage dominates rather than only how many.
                const u64 slot = out.samples / 8500;
                if (out.examples.size() < 12 && (out.blocked < 40 || slot >= out.examples.size())) {
                    BlockedSample bs;
                    bs.model = model;
                    bs.ground = ground;
                    bs.probeY = probeY;
                    bs.depth = probe.depth;
                    bs.at = at;
                    bs.lanes = e.lanes;
                    bs.cls = e.cls;
                    bs.what = describeBody(world, probe.body, probe.at);
                    out.examples.push_back(bs);
                }
            }
        }
    }
    return out;
}

bool PhysicalFleet::boxFits(World& world, const Vec2d& p, f64 y, f64 yaw, sim::Archetype kind,
                            f64 penetrationTolerance, ProbeReport* reportOut) {
    const sim::ArchetypeSpec& spec = sim::archetypeSpec(kind);
    // The box is the chassis box itself, from the same source `spawnVehicle` reads:
    // a probe that is not the shape it stands in for answers a different question.
    const J::BoxShape box(J::Vec3(static_cast<f32>(spec.width * 0.5),
                                  static_cast<f32>(chassisHalfHeight(kind)),
                                  static_cast<f32>(spec.length * 0.5)));
    const J::RMat44 transform = J::RMat44::sRotationTranslation(
        J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)),
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)));
    J::CollideShapeSettings settings;
    settings.mMaxSeparationDistance = 0.0f;
    DeepestOverlap collector(penetrationTolerance);
    const StaticBodyFilter bodyFilter;
    const BuiltShapeFilter shapeFilter;
    world.system().GetNarrowPhaseQuery().CollideShape(
        &box, J::Vec3::sReplicate(1.0f), transform, settings,
        J::RVec3(static_cast<f32>(p.x), static_cast<f32>(y), static_cast<f32>(p.y)), collector,
        {}, {}, bodyFilter, shapeFilter);
    if (reportOut) {
        // The collector reports relative to the base offset it was given, which is
        // this box's centre: put the contact back into world coordinates where it
        // can be compared with the city.
        reportOut->depth = collector.deepest();
        reportOut->body = collector.body();
        reportOut->at = Vec3(static_cast<f32>(p.x) + collector.at().x,
                             static_cast<f32>(y) + collector.at().y,
                             static_cast<f32>(p.y) + collector.at().z);
    }
    return !collector.any();
}

// The box along the path the vehicle is about to take: four samples, so a wall
// 6 m ahead is found before the bumper reaches it.
bool PhysicalFleet::pathFits(World& world, sim::Traffic& traffic, u32 index, f64 metres,
                             ProbeReport* reportOut) {
    const sim::Vehicle& sv = traffic.vehicleAt(index);
    const f64 rest = chassisRestHeight(sv.kind);
    ProbeReport worst;
    for (int k = 1; k <= 4; ++k) {
        const f64 ahead = metres * static_cast<f64>(k) / 4.0;
        const Vec2d point = traffic.lanePoint(sv, ahead);
        const f64 modelHeight = traffic.surfaceHeightOf(sv);
        const f64 ground = surfaceAt(world, point.x, point.y, modelHeight);
        const Vec2d from = traffic.lanePoint(sv, 0.0);
        const f64 yaw = std::atan2(point.x - from.x, point.y - from.y);
        ProbeReport probe;
        if (!boxFits(world, point, ground + rest + kAttachClearance, yaw, sv.kind,
                     m_settings.probePenetrationTolerance, &probe) &&
            probe.depth > worst.depth) {
            worst = probe;
        }
    }
    if (reportOut) *reportOut = worst;
    return worst.depth == 0.0;
}

size_t PhysicalFleet::attach(World& world, sim::Traffic& traffic, const Vec2d& centre,
                             const DriverSettings& settings) {
    m_settings = settings;
    m_driven.clear();
    m_metrics = DriverMetrics{};

    // Candidates: the vehicles nearest the given centre, two-wheelers included
    // or not by setting.  Ordered by distance and then by index so the choice is
    // deterministic and the same run twice takes over the same vehicles.
    struct Candidate { f64 d2; u32 idx; };
    std::vector<Candidate> candidates;
    const size_t n = traffic.vehicleCount();
    candidates.reserve(n);
    for (u32 i = 0; i < n; ++i) {
        const sim::Vehicle& v = traffic.vehicleAt(i);
        if (!settings.includeTwoWheelers && sim::archetypeSpec(v.kind).twoWheeler) continue;
        // A vehicle in the middle of a hail stop is being animated to the kerb;
        // taking it over mid-manoeuvre would be a fight between two controllers.
        if (v.hailing) continue;
        const Vec2d p = traffic.positionOf(v);
        candidates.push_back(Candidate{(p - centre).lengthSq(), i});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.d2 != b.d2) return a.d2 < b.d2;
        return a.idx < b.idx;
    });

    const f64 radiusSq = settings.attachRadius * settings.attachRadius;
    for (const Candidate& c : candidates) {
        if (m_driven.size() >= settings.count) break;
        if (c.d2 > radiusSq) break;
        if (traffic.isPromoted(c.idx)) continue;
        const sim::Vehicle& sv = traffic.vehicleAt(c.idx);
        const Vec2d lane = traffic.lanePoint(sv, 0.0);
        const f64 modelHeight = traffic.surfaceHeightOf(sv);
        const f64 ground = surfaceAt(world, lane.x, lane.y, modelHeight);
        if (std::fabs(ground - modelHeight) > settings.heightTolerance) {
            ++m_metrics.rejectedSpawns;
            ++m_metrics.handedBackHeight;
            if (settings.trace)
                std::printf("      refused veh %2u %-16s at (%8.1f,%8.1f): the collision surface "
                            "(%.2f) and the model's road (%.2f) are %.2f m apart\n", c.idx,
                            sim::archetypeSpec(sv.kind).name, lane.x, lane.y, ground, modelHeight,
                            std::fabs(ground - modelHeight));
            continue;
        }
        const f64 rest = chassisRestHeight(sv.kind);
        const f64 y = ground + rest + kAttachClearance;
        {
            const f64 yaw = traffic.laneHeading(sv);
            ProbeReport probe;
            // Two gates, because they answer two different questions: the city-only
            // box probe says the city will hold this vehicle, and the all-bodies
            // probe says no vehicle already placed is standing where this one goes.
            // Authored on top of another vehicle, a body starts life overlapped and
            // the overlap is measured as penetration for as long as Jolt takes to
            // push it out — two 16 t double-deckers, 0.1007 m, D-034.
            // Everything from the road up: the wheels and the space under the
            // floor are as much a part of the vehicle as its roof.
            const f64 underbody = wholeVehicleOverlapAt(world, sv.kind, lane, ground, yaw);
            if (underbody > settings.spawnOverlapTolerance) {
                ++m_metrics.rejectedSpawns;
                ++m_metrics.rejectedOnCity;
                if (settings.trace)
                    std::printf("      refused veh %2u %-16s at (%8.1f,%8.1f): its wheels or "
                                "floor are %.2f m inside the city\n",
                                c.idx, sim::archetypeSpec(sv.kind).name, lane.x, lane.y, underbody);
                continue;
            }
            const f64 fleetOverlap = anyOverlapAt(world, sv.kind, lane, y, yaw);
            if (fleetOverlap > settings.spawnOverlapTolerance) {
                ++m_metrics.rejectedSpawns;
                ++m_metrics.rejectedOnTraffic;
                if (settings.trace)
                    std::printf("      refused veh %2u %-16s at (%8.1f,%8.1f): its own body is "
                                "%.2f m inside another body already placed\n",
                                c.idx, sim::archetypeSpec(sv.kind).name, lane.x, lane.y,
                                fleetOverlap);
                continue;
            }
            if (!boxFits(world, lane, y, yaw, sv.kind, settings.spawnOverlapTolerance, &probe)) {
                ++m_metrics.rejectedSpawns;
                if (settings.trace)
                    std::printf("      refused veh %2u %-16s at (%8.1f,%8.1f): its own body is "
                                "%.2f m inside the city — %s\n",
                                c.idx, sim::archetypeSpec(sv.kind).name, lane.x, lane.y,
                                probe.depth, describeBody(world, probe.body, probe.at).c_str());
                continue;
            }
        }
        // A step under the footprint is not something a car can be placed on: a
        // kerb, a plinth or a median edge reads as a road at the lane centre and
        // as a wall at the vehicle's own corner.  Measured, not assumed.
        {
            const f64 halfL = sim::archetypeSpec(sv.kind).length * 0.5;
            const f64 halfW = sim::archetypeSpec(sv.kind).width * 0.5;
            f64 low = ground, high = ground;
            const f64 dx[4] = {halfL, -halfL, 0.0, 0.0};
            const f64 dz[4] = {0.0, 0.0, halfW, -halfW};
            for (int k = 0; k < 4; ++k) {
                const f64 h = surfaceAt(world, lane.x + dx[k], lane.y + dz[k], ground);
                low = std::min(low, h);
                high = std::max(high, h);
            }
            if (high - low > settings.footprintStepTolerance) {
                ++m_metrics.rejectedSpawns;
                if (settings.trace)
                    std::printf("      refused veh %2u %-16s at (%8.1f,%8.1f): %.2f m of step "
                                "under its own footprint\n", c.idx,
                                sim::archetypeSpec(sv.kind).name, lane.x, lane.y, high - low);
                continue;
            }
        }
        // The heading is the lane's, not the model's `s`-derivative: a vehicle
        // entering a curve is put on the curve, not aimed at where the curve was.
        const f64 ahead = std::min(4.0, static_cast<f64>(traffic.edgeOf(sv).length) - sv.s);
        const Vec2d aim = traffic.lanePoint(sv, std::max(1.0, ahead));
        const f64 yaw = std::atan2(aim.x - lane.x, aim.y - lane.y);
        Vehicle body = spawnVehicle(world, sv.kind,
                                    Vec3(static_cast<f32>(lane.x), static_cast<f32>(y),
                                         static_cast<f32>(lane.y)),
                                    yaw);
        if (!body.valid()) continue;
        // Start it at the speed the model already believes it has: a vehicle that
        // is handed over at rest would be a phantom braking event for everyone
        // behind it.
        world.bodies().SetLinearVelocity(
            body.body.id,
            J::Vec3(static_cast<f32>(std::sin(yaw) * sv.v), 0.0f,
                    static_cast<f32>(std::cos(yaw) * sv.v)));
        if (!traffic.promote(c.idx)) {
            destroyVehicle(world, body);
            continue;
        }
        DrivenVehicle d;
        d.simIndex = c.idx;
        d.body = body;
        d.spawn = lane;
        d.spawnY = y;
        d.surfaceOffset = ground - modelHeight;
        if (m_settings.trace) {
            // Free space at the spawn: eight horizontal rays at wheel height.  A
            // vehicle can be placed perfectly on the road and still be inside a
            // building, and this is the measurement that says so.
            int blocked = 0;
            f64 nearest = 1e9;
            for (int k = 0; k < 8; ++k) {
                const f64 a = k * (2.0 * kPi / 8.0);
                const World::RayHit h = world.rayCastStatic(
                    Vec3(static_cast<f32>(lane.x), static_cast<f32>(y), static_cast<f32>(lane.y)),
                    Vec3(static_cast<f32>(std::sin(a)), 0.0f, static_cast<f32>(std::cos(a))), 3.0);
                if (h.hit) {
                    ++blocked;
                    nearest = std::min(nearest, h.fraction * 3.0);
                }
            }
            std::printf("      spawn  veh %2u %-16s lanes %u at (%8.1f,%7.2f,%8.1f) ground %.2f "
                        "model %.2f  blocked %d/8  nearest %.2f m\n", c.idx,
                        sim::archetypeSpec(sv.kind).name, traffic.edgeOf(sv).lanes, lane.x, y,
                        lane.y, ground, modelHeight, blocked, nearest < 1e9 ? nearest : -1.0);
        }
        m_driven.push_back(d);
    }
    m_metrics.promoted = m_driven.size();
    m_metrics.modelVehiclesPromoted = traffic.promotedCount();
    return m_driven.size();
}

void PhysicalFleet::step(World& world, sim::Traffic& traffic, f64 dt) {
    ++m_metrics.ticks;

    // 0. Look ahead: the road in front of a driven vehicle is validated before it
    //    is driven onto.  A vehicle whose road ahead does not exist in the
    //    collision is handed back now, while it is still on a road the two layers
    //    agree about, rather than after it has driven into a wall.
    m_lookAheadAccum += dt;
    const bool lookNow = m_lookAheadAccum >= m_settings.lookAheadSeconds;
    if (lookNow) m_lookAheadAccum = 0.0;

    // 1. Control: ask each body for what the model decided this tick.
    for (size_t i = 0; i < m_driven.size(); ++i) {
        DrivenVehicle& d = m_driven[i];
        if (d.handedBack) continue;
        const sim::Vehicle& sv = traffic.vehicleAt(d.simIndex);
        if (!sv.external) { d.handedBack = true; continue; }

        if (lookNow) {
            ProbeReport probe;
            const bool missing =
                !pathFits(world, traffic, d.simIndex, m_settings.pathProbeMetres, &probe);
            if (missing && m_settings.trace)
                std::printf("      ahead:   veh %2u its own body is %.2f m inside the city within "
                            "%.0f m of road ahead — %s\n", d.simIndex, probe.depth,
                            m_settings.pathProbeMetres,
                            describeBody(world, probe.body, probe.at).c_str());
            if (missing) {
                ++m_metrics.handedBackAhead;
                d.handbackReason = "the road ahead does not exist in the collision";
                handBack(world, traffic, i);
                ++i;
                continue;
            }
        }
        const Vec3 p = world.position(d.body.body);
        const f64 have = headingOf(world, d.body);
        // Pure pursuit at the lookahead: aim at the lane centreline ahead of the
        // vehicle, not at a point behind it — the classic failure of aiming at the
        // nearest point on a straight line is that the aim point overtakes the
        // vehicle and the steering oscillates.
        // The lookahead grows with speed, the way a driver's eyes do: a fixed 9 m
        // is a third of a second at 100 km/h and the controller then fights every
        // curve (measured: mean lane error 1.9 m and one vehicle 9.5 m wide of its
        // lane in an 8 s run, D-034).
        const f64 speedNow = forwardSpeed(world, d.body);
        const f64 lookahead =
            std::clamp(m_settings.lookaheadBase + m_settings.lookaheadPerSpeed * speedNow,
                       m_settings.lookaheadBase, m_settings.lookaheadMax);
        //
        // Does the model's frame still contain the body at all?  A body past the
        // end of its edge (the graph had no arm it could take) or behind the edge's
        // start has no lane the model can name, and asking a driver to hold one
        // means handing it an aim point behind its own nose: full lock, at speed, by
        // construction.  Measured, that is where the airborne wheels and the kerb
        // penetration of this check came from (D-034).  The driver holds the body's
        // own heading instead, and the report pass hands the vehicle back.
        const sim::SimEdge& me = traffic.edgeOf(sv);
        const bool offFrame = sv.bodyAlong < -1.0 ||
                              sv.bodyAlong > static_cast<f64>(me.length) + 1.0;
        const Vec2d aim =
            offFrame ? Vec2d{static_cast<f64>(p.x) + std::sin(have) * lookahead,
                             static_cast<f64>(p.z) + std::cos(have) * lookahead}
                     : traffic.lanePoint(sv, lookahead);
        const f64 want = std::atan2(aim.x - static_cast<f64>(p.x), aim.y - static_cast<f64>(p.z));
        const f64 err = wrapPi(want - have);
        // `right > 0` turns clockwise in this world frame (D-033, measured), and a
        // positive heading error is a vehicle that has to turn clockwise to face
        // its aim point: the sign follows from the convention, it is not a guess.
        // The lateral part of the error: how far the body is from the lane line it
        // is standing on, measured along the lane's own right axis.  A controller
        // that only chases a heading cannot tell a metre-wide lane error from a
        // one-degree heading error — and it is the metre-wide error a driver has to
        // remove, which is why the model's own lane offset is the primary check
        // (D-034).
        const Vec2d here = traffic.lanePoint(sv, 0.0);
        const f64 h = traffic.laneHeading(sv);
        const f64 lateral = offFrame ? 0.0
                                     : (here.x - static_cast<f64>(p.x)) * std::cos(h) -
                                           (here.y - static_cast<f64>(p.z)) * std::sin(h);
        // `right` is the full-lock fraction: heading gain plus lateral gain, then a
        // speed term that stops a fast vehicle from sawing at the wheel.
        const f64 steerCmd = (m_settings.steerGain * err + m_settings.lateralGain * lateral) /
                             (1.0 + m_settings.steerSpeedDamping * std::fabs(speedNow));
        const f32 right = static_cast<f32>(std::clamp(steerCmd, -1.0, 1.0));

        const f64 speed = speedNow;
        const f64 error = sv.commandSpeed - speed;
        f32 forward = 0.0f, brake = 0.0f;
        if (error > m_settings.speedDeadband) {
            forward = static_cast<f32>(std::min(1.0, m_settings.throttleFloor +
                                                          m_settings.throttleGain * error));
        } else if (error < -m_settings.speedDeadband) {
            brake = static_cast<f32>(std::min(m_settings.maxBrake,
                                              m_settings.brakeFloor + m_settings.brakeGain * (-error)));
        } else {
            // Inside the deadband: keep the throttle barely open, so that rolling
            // resistance does not turn a cruise into a coast.  A zero here makes
            // every vehicle in the fleet decay to a crawl over a long run.
            forward = static_cast<f32>(m_settings.throttleFloor * 0.5);
        }
        // A driver who has the wheel at full lock is not asking for more speed:
        // without this the bus tries to turn around at 35 km/h in a 7 m street and
        // ends up inside a building (measured: one BEST double-decker at 3.4 m of
        // penetration, D-034).
        if (std::fabs(static_cast<f64>(right)) > 0.6 && forward > 0.0f)
            forward = static_cast<f32>(forward * (1.0 - 0.6 * (std::fabs(static_cast<f64>(right)) - 0.6) / 0.4));
        driveVehicle(d.body, forward, right, brake);

        // The error the model asked for, not the error the physics achieved: this
        // is what `worstSpeedError` is (it was accumulated nowhere, which made half
        // of the speed check unable to fail — D-034).
        d.speedErrorSum += std::fabs(error);
        if (std::fabs(error) > d.worstSpeedError) {
            d.worstSpeedError = std::fabs(error);
            d.worstSpeedAtTick = m_metrics.ticks;
            d.worstCommandMs = sv.commandSpeed;
            d.worstActualMs = speed;
            d.worstSpeedAt = Vec2d(static_cast<f64>(p.x), static_cast<f64>(p.z));
        }
        ++d.speedSamples;
        d.speedSum += speed;
        // Wheels on the ground, per wheel: a three-wheeled auto-rickshaw is not
        // judged by a car's four, and "2 of 3" is a different fact from "0 of 4".
        const int contacts = wheelsOnGround(d.body);
        const int total = wheelCount(d.body);
        for (int w = 0; w < total && w < 4; ++w) {
            const J::Wheel* wheel = d.body.constraint->GetWheel(static_cast<J::uint>(w));
            if (wheel != nullptr && wheel->HasContact()) ++d.wheelTicks[w];
        }
        d.wheelContactSum += contacts;
        d.wheelCountSum += total;
        if (contacts == 0) ++d.airborneSamples;
        if (speed > m_settings.movingSpeed) {
            ++d.movingSamples;
            d.movingContactSum += contacts;
            d.movingSlotSum += total;
            if (contacts == 0) ++d.movingAirborne;
        }
        ++d.samples;
    }

    // 2. Simulate.  The vehicle controllers were set in the control pass, so the
    //    step integrates exactly the inputs the model asked for.
    world.step(static_cast<f32>(dt));

    // 3. Report back: the body's position is the truth the model reasons from
    //    next tick.  Doing this after the step (rather than before) means the
    //    model sees the state the renderer is about to draw.
    for (size_t i = 0; i < m_driven.size();) {
        DrivenVehicle& d = m_driven[i];
        if (d.handedBack) { ++i; continue; }
        const Vec3 p = world.position(d.body.body);
        const f64 speed = forwardSpeed(world, d.body);
        traffic.adoptPosition(d.simIndex, Vec2d(static_cast<f64>(p.x), static_cast<f64>(p.z)),
                              speed, headingOf(world, d.body));
        const sim::Vehicle& sv = traffic.vehicleAt(d.simIndex);

        // Distance from the lane centreline: the same pure-pursuit aim point at
        // zero lookahead is the lane's own position beside the vehicle.
        const Vec2d lane = traffic.lanePoint(sv, 0.0);
        const Vec2d delta = Vec2d(static_cast<f64>(p.x), static_cast<f64>(p.z)) - lane;
        const f64 off = delta.length();
        // Which way the error points: along the lane (the model's s is not where the
        // body is — a bookkeeping problem) or across it (the driver is off the road
        // — a driving problem).  They need opposite fixes, so they are counted
        // separately (D-034).
        const Vec2d edir = traffic.edgeDir(sv);
        const f64 have = headingOf(world, d.body);
        d.alongErr = delta.x * edir.x + delta.y * edir.y;
        d.sideErr = delta.x * edir.y - delta.y * edir.x;
        if (std::fabs(d.alongErr) > std::fabs(d.worstAlongErr)) d.worstAlongErr = d.alongErr;
        if (std::fabs(d.sideErr) > std::fabs(d.worstSideErr)) d.worstSideErr = d.sideErr;
        d.sideAbsSum += std::fabs(d.sideErr);
        d.alongAbsSum += std::fabs(d.alongErr);
        if (std::fabs(d.sideErr) > m_settings.offRoadMetres) ++d.sideOffSamples;
        // Facing away from the edge the model has it on: the model has turned the
        // vehicle around (a U-turn at a dead end, a junction choice the body has not
        // made) and the body is still travelling the other way.  A body held like
        // that is not being driven by this controller and the model can have it back
        // (D-034).
        // The heading's own forward vector is (sin h, cos h) in this world frame —
        // `yaw = atan2(dx, dz)` is the project's convention — so the test is a dot
        // product with that vector and not with the angle's cosines (a first version
        // got this wrong and handed back five of six healthy vehicles, D-034).
        const f64 facing = std::sin(have) * edir.x + std::cos(have) * edir.y;
        if (facing < -0.2) d.wrongWaySeconds += dt;
        else d.wrongWaySeconds = 0.0;
        if (m_settings.trace && off > d.worstLateralOff && off > m_settings.traceOffM) {
            const sim::SimEdge& te = traffic.edgeOf(sv);
            // The lateral frame of this disagreement, in absolute numbers: the
            // lane centre the model asked for, where the body actually is, and the
            // two headings.  `lat` alone cannot say which of the two is displaced.
            const Vec2d lpt = traffic.lanePoint(sv, 0.0);
            const f64 laneOffLat = (lpt.x - te.a.x) * te.normal.x + (lpt.y - te.a.y) * te.normal.y;
            const f64 bodyLat = (static_cast<f64>(p.x) - te.a.x) * te.normal.x +
                                (static_cast<f64>(p.z) - te.a.y) * te.normal.y;
            const f64 laneHdg = traffic.laneHeading(sv);
            const f64 yawErr = wrapPi(laneHdg - have);
            std::printf("      drift  veh %2u %-16s lane %u/%u s %6.1f/%-6.1f lat %+6.2f  body "
                        "(%8.1f,%7.2f,%8.1f)  off %5.2f m (along %+5.2f, side %+5.2f)  "
                        "laneLat %+5.2f bodyLat %+5.2f  yaw %+6.1f laneHdg %+6.1f "
                        "(err %+5.1f)  modelY %7.2f  v %5.1f km/h\n",
                        d.simIndex, sim::archetypeSpec(sv.kind).name, sv.lane, te.lanes, sv.s,
                        static_cast<f64>(te.length), sv.lateral, static_cast<f64>(p.x),
                        static_cast<f64>(p.y), static_cast<f64>(p.z), off, d.alongErr,
                        d.sideErr, laneOffLat, bodyLat, have * 57.29577951308232,
                        laneHdg * 57.29577951308232, yawErr * 57.29577951308232,
                        traffic.surfaceHeightOf(sv), speed * 3.6);
        }
        d.worstLateralOff = std::max(d.worstLateralOff, off);
        d.lateralOffSum += off;
        if (off > m_settings.offRoadMetres) ++d.offRoadSamples;

        // Where the collision layer and the model disagree about the height of
        // the road, the body is on one surface and the model believes another.
        // This is a data disagreement, not a driving failure, and it is reported
        // as one.
        const f64 bodySurface = static_cast<f64>(p.y) - chassisRestHeight(sv.kind);
        const f64 offsetNow = bodySurface - traffic.surfaceHeightOf(sv);
        const f64 heightError = std::fabs(offsetNow - d.surfaceOffset);
        d.heightErrorSum += heightError;
        d.heightErrorWorst = std::max(d.heightErrorWorst, heightError);
        // On the road the model put it on: its own lane, and the two layers
        // agreeing about where the road is.
        if (std::fabs(d.sideErr) <= m_settings.offRoadMetres &&
            heightError <= m_settings.heightTolerance && speed > m_settings.movingSpeed) {
            ++d.onRoadSamples;
            const f64 onRoadError = std::fabs(sv.commandSpeed - speed);
            d.onRoadSpeedErrorSum += onRoadError;
            d.onRoadSpeedErrorWorst = std::max(d.onRoadSpeedErrorWorst, onRoadError);
        }
        if (heightError > m_settings.heightDriftTolerance) ++d.heightGrace;
        else d.heightGrace = 0;
        // Outside the model's own edge, measured at the body: while it is there the
        // model's lane cannot describe it, and the clock that hands it back runs.
        const sim::SimEdge& oe = traffic.edgeOf(sv);
        const f64 outsideM =
            sv.bodyAlong < 0.0
                ? -sv.bodyAlong
                : (sv.bodyAlong > static_cast<f64>(oe.length)
                       ? sv.bodyAlong - static_cast<f64>(oe.length)
                       : 0.0);
        if (outsideM > 1.0) {
            d.offFrameSeconds += dt;
            d.worstOutsideM = std::max(d.worstOutsideM, outsideM);
        } else {
            d.offFrameSeconds = 0.0;
        }

        // A vehicle the model can no longer explain is handed back, and the event
        // is counted.  This is not a recovery path for a bug: falling off a bridge,
        // climbing a kerb into a compound or being carried by a bus are all
        // legitimate outcomes of driving a city, and the honest thing to do with
        // one is to stop pretending the road model describes it.
        const sim::SimEdge& e = traffic.edgeOf(sv);
        // The lane plus a shoulder is the corridor: a vehicle more than that from
        // the centre of its own lane is not in its lane, and the model takes it
        // back.  The first version of this used the whole carriageway plus six
        // metres, which let a body sit four metres off its line and still be
        // "driven" (D-034).
        // Two rules, whichever is tighter.  The lane plus a shoulder says a vehicle
        // more than that from the centre of its own lane is not in its lane.  The
        // graded flat zone says a body whose box overhangs it has its own side in
        // the bank: the bake flattens the ground for `lanes * 3.5/2 + 1.5` m either
        // side of the centreline and blends back to the hill over the next 10 m
        // (`gradeRoadsIntoGrid`), so a 12.8 m double-decker allowed 4.5 m of lane
        // error on a one-lane approach drove its side 0.0401 m into the hillside
        // (D-034, measured).
        const f64 laneCorridor = 0.5 * static_cast<f64>(e.lanes) * sim::kLaneWidth +
                                 m_settings.corridorShoulder;
        const f64 flat = 0.5 * static_cast<f64>(e.lanes) * sim::kLaneWidth + 1.5;
        const f64 laneLat = std::fabs(sim::Traffic::laneOffsetFor(sv.lane, e.lanes));
        const f64 flatCorridor =
            std::max(0.0, flat - laneLat - sim::archetypeSpec(sv.kind).width * 0.5);
        const f64 corridor = std::min(laneCorridor, flatCorridor);
        d.corridorM = corridor;
        if (d.offFrameSeconds > m_settings.offFrameSeconds) {
            ++m_metrics.handedBackOffFrame;
            d.handbackReason = "the body left the edge the model put it on";
            if (m_settings.trace)
                std::printf("      handback: veh %u %-16s %+.2f m outside its edge (%.1f m of "
                            "%.1f) for %.1f s\n",
                            d.simIndex, sim::archetypeSpec(sv.kind).name,
                            sv.bodyAlong < 0.0 ? -d.worstOutsideM : d.worstOutsideM,
                            sv.bodyAlong, static_cast<f64>(e.length), d.offFrameSeconds);
            handBack(world, traffic, i);
            ++i;
            continue;
        }
        if (d.wrongWaySeconds > m_settings.wrongWaySeconds) {
            ++m_metrics.handedBackWrongWay;
            d.handbackReason = "the model turned it around and the body did not follow";
            if (m_settings.trace)
                std::printf("      handback: veh %u %-16s facing %+.2f against its edge for "
                            "%.1f s\n",
                            d.simIndex, sim::archetypeSpec(sv.kind).name, facing, d.wrongWaySeconds);
            handBack(world, traffic, i);
            ++i;
            continue;
        }
        if (off > corridor || d.heightGrace > m_settings.heightGraceTicks) {
            if (m_settings.trace)
                std::printf("      handback: veh %u %-16s off %.2f m (corridor %.2f), layers "
                            "drifted %+.2f m (body surface %.2f, model road %.2f, offset at "
                            "handover %+.2f)\n", d.simIndex,
                            sim::archetypeSpec(sv.kind).name, off, corridor, heightError, bodySurface,
                            traffic.surfaceHeightOf(sv), d.surfaceOffset);
            if (d.heightGrace > m_settings.heightGraceTicks) {
                ++m_metrics.handedBackHeight;
                d.handbackReason = "the layers disagreed about the road";
            } else {
                ++m_metrics.handedBackOffRoad;
                d.handbackReason = "left the corridor";
            }
            handBack(world, traffic, i);
            ++i;
            continue;
        }
        ++i;
    }

    // 4. Metrics, taken from the per-vehicle diagnostics so a vehicle that has
    //    been handed back still contributes the numbers it earned.
    //    The penetration this section owns is the penetration of a vehicle that is
    //    *still being driven*: a body that has left its road and been handed back
    //    is the model's vehicle again, and its last moments are reported as a
    //    handback (with the reason) rather than as a physics defect.  The deepest
    //    contact over everything the section touched is kept beside it, so nothing
    //    is hidden either way.
    m_metrics.deepestPenetrationM =
        std::max({m_metrics.deepestPenetrationM, world.penetration().deepestStatic,
                  world.penetration().deepestDynamic});
    // Attribution: a contact counts as driven if either side of it is a vehicle
    // that is still being driven.  A body against the city and a body against
    // another body are counted apart, because they are different failures: the
    // first is a line the model should not have taken, the second is two solver
    // objects resolving a contact (D-034).
    const auto drivenName = [this](const J::BodyID& id) -> std::string {
        if (id.IsInvalid()) return {};
        for (const DrivenVehicle& d : m_driven) {
            if (d.handedBack) continue;
            if (d.body.body.id == id) {
                char buf[192];
                std::snprintf(buf, sizeof(buf),
                              "veh %u %s (lane error now %.2f m, worst %.2f m, corridor %.2f m, "
                              "%llu off-road samples)",
                              d.simIndex, sim::archetypeSpec(d.body.kind).name, d.sideErr,
                              d.worstLateralOff, d.corridorM,
                              static_cast<unsigned long long>(d.offRoadSamples));
                return std::string(buf);
            }
        }
        return {};
    };
    const auto liveDriven = [&drivenName](const J::BodyID& id) { return !drivenName(id).empty(); };
    if (liveDriven(world.penetration().deepestStaticBody) ||
        liveDriven(world.penetration().deepestStaticPartner))
        m_metrics.deepestDrivenStaticM =
            std::max(m_metrics.deepestDrivenStaticM, world.penetration().deepestStatic);
    if (liveDriven(world.penetration().deepestDynamicBody) ||
        liveDriven(world.penetration().deepestDynamicPartner))
        m_metrics.deepestDrivenDynamicM =
            std::max(m_metrics.deepestDrivenDynamicM, world.penetration().deepestDynamic);
    m_metrics.deepestDrivenM =
        std::max(m_metrics.deepestDrivenStaticM, m_metrics.deepestDrivenDynamicM);
    if (m_settings.trace) {
        // Named as it deepens, not after the fact: the fleet is detached before the
        // section's report, and a BodyID means nothing afterwards (D-034).
        const auto raw = [&world](const J::BodyID& id) -> std::string {
            if (id.IsInvalid()) return "invalid";
            if (id.GetIndex() >= world.system().GetNumBodies())
                return "out-of-range " + std::to_string(id.GetIndex());
            const J::Body* b = world.system().GetBodyLockInterfaceNoLock().TryGetBody(id);
            if (b == nullptr) return "unresolved " + std::to_string(id.GetIndex());
            const char* kind = b->IsDynamic() ? "dynamic" : (b->IsKinematic() ? "kinematic" : "static");
            return std::string(kind) + " " + std::to_string(id.GetIndex());
        };
        const PenetrationStats& ps = world.penetration();
        if (ps.deepestStatic > m_reportedStaticM + 1e-9) {
            m_reportedStaticM = ps.deepestStatic;
            std::printf("      contact t=%.1f s  body vs city %.4f m at (%.1f, %.1f, %.1f): %s vs %s\n",
                        m_metrics.ticks / 60.0, ps.deepestStatic, ps.deepestStaticAt.x,
                        ps.deepestStaticAt.y, ps.deepestStaticAt.z,
                        ps.deepestStaticBody.IsInvalid()
                            ? "a body the system no longer resolves"
                            : describeBody(world, ps.deepestStaticBody, ps.deepestStaticAt).c_str(),
                        raw(ps.deepestStaticPartner).c_str());
            if (!drivenName(ps.deepestStaticBody).empty())
                std::printf("        in that contact: %s\n",
                            drivenName(ps.deepestStaticBody).c_str());
        }
        if (ps.deepestDynamic > m_reportedDynamicM + 1e-9) {
            m_reportedDynamicM = ps.deepestDynamic;
            std::printf("      contact t=%.1f s  body vs body %.4f m at (%.1f, %.1f, %.1f): %s vs %s"
                        "   (%zu bodies; %s vs %s)\n",
                        m_metrics.ticks / 60.0, ps.deepestDynamic, ps.deepestDynamicAt.x,
                        ps.deepestDynamicAt.y, ps.deepestDynamicAt.z,
                        ps.deepestDynamicBody.IsInvalid()
                            ? "an invalid id"
                            : describeBody(world, ps.deepestDynamicBody, ps.deepestDynamicAt).c_str(),
                        ps.deepestDynamicPartner.IsInvalid()
                            ? "an invalid id"
                            : describeBody(world, ps.deepestDynamicPartner, ps.deepestDynamicAt).c_str(),
                        static_cast<size_t>(world.system().GetNumBodies()),
                        raw(ps.deepestDynamicBody).c_str(), raw(ps.deepestDynamicPartner).c_str());
            const std::string a = drivenName(ps.deepestDynamicBody);
            const std::string b = drivenName(ps.deepestDynamicPartner);
            if (!a.empty() || !b.empty())
                std::printf("        in that contact: %s | %s\n",
                            a.empty() ? "not a driven vehicle" : a.c_str(),
                            b.empty() ? "not a driven vehicle" : b.c_str());
        }
    }
    f64 worstOff = 0.0, worstErr = 0.0, offSum = 0.0, speedErrSum = 0.0, speedSum = 0.0;
    f64 contacts = 0.0, wheelSlots = 0.0, heightSum = 0.0, heightWorst = 0.0;
    f64 movingContacts = 0.0, movingSlots = 0.0;
    u64 airborne = 0, total = 0, moving = 0, offRoad = 0, movingAir = 0, handedBackCount = 0;
    u64 sideOff = 0, onRoadSamples = 0;
    f64 worstSide = 0.0, worstAlong = 0.0;
    f64 onRoadErrSum = 0.0, onRoadWorst = 0.0;
    f64 offSquares = 0.0, sideAbs = 0.0, alongAbs = 0.0;
    size_t live = 0;
    for (const DrivenVehicle& d : m_driven) {
        worstOff = std::max(worstOff, d.worstLateralOff);
        worstErr = std::max(worstErr, d.worstSpeedError);
        offSum += d.lateralOffSum;

        speedErrSum += d.speedErrorSum;
        speedSum += d.speedSum;
        contacts += d.wheelContactSum;
        wheelSlots += d.wheelCountSum;
        movingContacts += d.movingContactSum;
        movingSlots += d.movingSlotSum;
        moving += d.movingSamples;
        movingAir += d.movingAirborne;
        offRoad += d.offRoadSamples;
        sideOff += d.sideOffSamples;
        onRoadErrSum += d.onRoadSpeedErrorSum;
        onRoadSamples += d.onRoadSamples;
        onRoadWorst = std::max(onRoadWorst, d.onRoadSpeedErrorWorst);
        sideAbs += d.sideAbsSum;
        worstSide = std::max(worstSide, std::fabs(d.worstSideErr));
        worstAlong = std::max(worstAlong, std::fabs(d.worstAlongErr));
        alongAbs += d.alongAbsSum;
        offSquares += d.offSumSquared;
        if (d.handedBack) ++handedBackCount;
        heightSum += d.heightErrorSum;
        heightWorst = std::max(heightWorst, d.heightErrorWorst);
        airborne += d.airborneSamples;
        total += d.samples;
        if (!d.handedBack) ++live;
    }
    if (total) {
        m_metrics.meanLateralOffM = offSum / static_cast<f64>(total);
        m_metrics.wheelsDownFraction = wheelSlots > 0.0 ? contacts / wheelSlots : 0.0;
        m_metrics.meanSpeedErrorMs = speedErrSum / static_cast<f64>(total);
        m_metrics.meanSpeedKph = speedSum / static_cast<f64>(total) * 3.6;
        m_metrics.meanHeightErrorM = heightSum / static_cast<f64>(total);
        m_metrics.airborneSamples = airborne;
        m_metrics.offRoadShare = static_cast<f64>(offRoad) / static_cast<f64>(total);
        m_metrics.movingSamples = moving;
        m_metrics.movingWheelFraction = movingSlots > 0.0 ? movingContacts / movingSlots : 0.0;
        // The lane error, split into its two causes.  `side` is the driver's own
        // error to the lane line; `along` is how far the model's `s` is from where
        // the body actually is, which no driver can fix (D-034).
        m_metrics.meanSideErrorM = sideAbs / static_cast<f64>(total);
        m_metrics.worstSideErrorM = worstSide;
        m_metrics.worstAlongErrorM = worstAlong;
        m_metrics.meanAlongErrorM = alongAbs / static_cast<f64>(total);
        m_metrics.sideOffShare = static_cast<f64>(sideOff) / static_cast<f64>(total);
        m_metrics.laneErrorRmsM = std::sqrt(offSquares / static_cast<f64>(total));
        m_metrics.onRoadSamples = onRoadSamples;
        m_metrics.onRoadWorstSpeedErrorMs = onRoadWorst;
        m_metrics.onRoadMeanSpeedErrorMs =
            onRoadSamples > 0 ? onRoadErrSum / static_cast<f64>(onRoadSamples) : 0.0;
    }
    m_metrics.worstLateralOffM = worstOff;
    m_metrics.worstSpeedErrorMs = worstErr;
    m_metrics.worstHeightErrorM = heightWorst;
    m_metrics.promoted = live;
    m_metrics.modelVehiclesPromoted = traffic.promotedCount();
}

void PhysicalFleet::handBack(World& world, sim::Traffic& traffic, size_t i) {
    DrivenVehicle& d = m_driven[i];
    if (d.handedBack) return;
    // The model is given the body's last known state (already adopted this tick),
    // so the handback is continuous: the vehicle keeps its position, speed and
    // heading, and the model resumes integrating from there.
    destroyVehicle(world, d.body);
    traffic.demote(d.simIndex);
    d.handedBack = true;
    ++m_metrics.handedBack;
}

void PhysicalFleet::report() const {
    for (const DrivenVehicle& d : m_driven) {
        const f64 meanOff = d.samples ? d.lateralOffSum / static_cast<f64>(d.samples) : 0.0;
        const f64 meanSpeed = d.samples ? d.speedSum / static_cast<f64>(d.samples) * 3.6 : 0.0;
        const f64 down = d.wheelCountSum > 0.0 ? d.wheelContactSum / d.wheelCountSum * 100.0 : 0.0;
        const f64 hErr = d.samples ? d.heightErrorSum / static_cast<f64>(d.samples) : 0.0;
        const f64 movingDown = d.movingSlotSum > 0.0
                                   ? d.movingContactSum / d.movingSlotSum * 100.0 : 0.0;
        const f64 offShare = d.samples
                                 ? static_cast<f64>(d.offRoadSamples) / d.samples * 100.0 : 0.0;
        std::printf("      veh %2u  lane %5.2f m mean / %5.2f worst (%4.1f %% beyond %.0f m)   "
                    "speed %5.1f km/h   wheels %5.1f %% down (%5.1f %% moving)   layers %4.2f m   %s\n",
                    d.simIndex, meanOff, d.worstLateralOff, offShare, m_settings.offRoadMetres,
                    meanSpeed, down, movingDown, hErr,
                    d.handedBack ? d.handbackReason : "still driven");
        // Split the worst lane error: a body *ahead of* the model's s is a
        // bookkeeping fault, a body *beside* its own lane line is a driving fault,
        // and the two need opposite fixes (D-034).
        if (d.samples > 0)
            std::printf("              worst along %+5.2f m, side %+5.2f m\n", d.worstAlongErr,
                        d.worstSideErr);
        if (d.movingSlotSum > 0.0)
            std::printf("              moving: %5.1f %% of wheel contacts down over %llu moving "
                        "ticks, %llu of them airborne\n",
                        d.movingContactSum / d.movingSlotSum * 100.0,
                        static_cast<unsigned long long>(d.movingSamples),
                        static_cast<unsigned long long>(d.movingAirborne));
        if (d.samples > 0)
            std::printf("              worst along %+5.2f m (bookkeeping), side %+5.2f m "
                        "(driving)\n", d.worstAlongErr, d.worstSideErr);
        if (d.worstSpeedError > 0.5)
            std::printf("              worst speed error %5.2f m/s at tick %llu near (%.0f, %.0f): "
                        "the model asked %5.2f m/s, the body had %5.2f m/s\n",
                        d.worstSpeedError, static_cast<unsigned long long>(d.worstSpeedAtTick),
                        d.worstSpeedAt.x, d.worstSpeedAt.y, d.worstCommandMs, d.worstActualMs);
        std::printf("              wheels: %u %u %u %u contacts (of %llu ticks), %llu airborne\n",
                    d.wheelTicks[0], d.wheelTicks[1], d.wheelTicks[2], d.wheelTicks[3],
                    static_cast<unsigned long long>(d.samples),
                    static_cast<unsigned long long>(d.airborneSamples));
    }
}

void PhysicalFleet::detach(World& world, sim::Traffic& traffic) {
    for (size_t i = 0; i < m_driven.size(); ++i) handBack(world, traffic, i);
    m_driven.clear();
    m_metrics.promoted = 0;
}

}  // namespace mumbai::phys
