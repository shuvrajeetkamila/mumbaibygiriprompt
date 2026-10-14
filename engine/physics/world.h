// =============================================================================
//  Mumbai 1:1 EXACT — engine/physics/world.h
//
//  The physics world: Jolt, configured for this project, wrapped so that the
//  rest of the engine never includes a Jolt header directly.
//
//  Three things here are specific to Mumbai rather than to a generic integration:
//
//  * **The world is big and the frame is not.**  Section 4's floating origin
//    moves the *render* frame in 1024 m macro cells.  Physics bodies live in the
//    same frame as the renderer, so when the origin shifts every body moves with
//    it — atomically, between the physics tick and the render submit — and the
//    shift is measured (`rebase`) rather than assumed.  Jolt is built with f32
//    positions (JPH_DOUBLE_PRECISION off, D-033): with a 1024 m macro cell the
//    worst-case position magnitude in a cell is ~1.5 km, where an f32 ulp is
//    1.2e-4 m, well inside the millimetre the acceptance criteria ask for — and
//    f64 positions would cost the broadphase and every solver operation double
//    the bandwidth for no gain this side of the shift.
//
//  * **The collision geometry is the pipeline's.**  There is no second authoring
//    of collision: `bake.h` reads the CollisionBox records the world build wrote
//    (buildings, walls, hero meshes, the Sea Link deck and its cable capsules)
//    plus the chunk's own terrain and road meshes.
//
//  * **Penetration is measured, not assumed.**  A contact listener records the
//    deepest interpenetration seen in a run, because "no collision penetration"
//    is an acceptance criterion and a claim about it is worth exactly as much as
//    the instrument behind it.
// =============================================================================
#pragma once

#include "engine/core/core.h"
#include "engine/core/math.h"
#include "engine/physics/jolt.h"
#include "engine/world/chunk.h"   // CollisionBox: the world's collider records

#include <atomic>
#include <string>
#include <vector>

namespace mumbai::phys {

// Deepest penetration seen since the last reset, in metres, per contact kind.
struct PenetrationStats {
    f64 deepestStatic = 0.0;    // dynamic body vs world geometry
    f64 deepestDynamic = 0.0;   // dynamic vs dynamic
    u64 contacts = 0;
    u64 manifolds = 0;
    void reset() { deepestStatic = deepestDynamic = 0.0; contacts = manifolds = 0; }
};

// A body the simulation owns, with the label it was created for.
struct BodyHandle {
    JPH::BodyID id;
    ObjectLayerId layer = LayerStatic;
    bool valid() const { return !id.IsInvalid(); }
};

// How a dynamic body is swept through the world between ticks.  A discrete body
// moves in one jump per tick and is resolved after the fact, so at 20 m/s it
// spends one tick up to 0.35 m inside whatever it just hit — which for a 1:1 city
// is the difference between a crate landing on the Sea Link deck and a crate
// landing in the sea.  A linear-cast body sweeps, so the contact is found before
// the overlap exists.  Props and vehicles in this project are linear-cast (A2).
enum class MotionQuality {
    Discrete,
    LinearCast,
};

class World {
public:
    World();
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // Initialises Jolt.  `maxBodies` sizes the body manager (the world build
    // holds one body per compound group of a cell's collision boxes plus the
    // vehicles); `tempPoolBytes` is the per-step scratch pool.
    bool init(u32 maxBodies, u32 tempPoolBytes = 32u << 20, int jobThreads = 1);
    void shutdown();
    bool ready() const { return m_ready; }

    // --- geometry -----------------------------------------------------------
    // A static body holding a compound of boxes and capsules.  Returns an
    // invalid handle for an empty group.  `boxes` are given in the *world*
    // frame already (the caller applies the chunk origin).
    BodyHandle addStaticCompound(const std::vector<CollisionBox>& boxes);

    // Convenience wrappers, mostly for the harness and tests.
    BodyHandle addStaticBox(const Vec3& center, const Vec3& halfExtents, f64 yawRadians = 0.0);
    BodyHandle addStaticCapsule(const Vec3& a, const Vec3& b, f64 radius);

    // A dynamic box body.  `centerOfMassOffset` shifts the centre of mass below
    // the geometric centre, which is how a real vehicle's mass sits.
    BodyHandle addDynamicBox(const Vec3& center, const Vec3& halfExtents, f64 yawRadians,
                             f64 massKg, const Vec3& centerOfMassOffset = {0, 0, 0},
                             ObjectLayerId layer = LayerVehicle,
                             MotionQuality quality = MotionQuality::LinearCast);

    // --- simulation ---------------------------------------------------------
    // One fixed step.  The caller owns the time base (the runtime runs 60 Hz).
    void step(f32 dt, u32 collisionSteps = 1);

    // --- floating origin ----------------------------------------------------
    // Moves every body by `delta` in one pass.  Returns the number of bodies
    // moved and the wall time in milliseconds.  This is Section 4's "bulk
    // transform update": it must happen between the physics tick and the render
    // submit, and it is called only when the origin actually crosses a macro
    // cell, so the cost is amortised over ~40 s of driving at 24 m/s.
    struct RebaseResult {
        u64 bodies = 0;
        f64 milliseconds = 0.0;         // the shift itself, nothing else
        f64 worstPositionError = 0.0;   // metres: measured by reading every body
                                        // back, and deliberately measured *after*
                                        // the timer stops, because the criterion
                                        // is the duration of the shift
    };
    RebaseResult rebase(const Vec3& delta);

    // --- measurement --------------------------------------------------------
    PenetrationStats& penetration() { return m_penetration; }
    const PenetrationStats& penetration() const { return m_penetration; }

    u32 bodyCount() const;
    u32 activeBodyCount() const;
    J::PhysicsSystem& system() { return *m_system; }

    // Creates a body without adding it, so a constraint (a vehicle) can be built
    // against it before it enters the simulation.  The caller owns the result and
    // must add it, or destroy it.
    J::Body* createBody(const J::BodyCreationSettings& settings) {
        return m_system->GetBodyInterface().CreateBody(settings);
    }
    J::BodyInterface& bodies() { return m_system->GetBodyInterface(); }
    J::BodyInterface& bodiesNoLock() { return m_system->GetBodyInterfaceNoLock(); }

    // A ray cast against the static world: how the harness finds the Sea Link
    // deck (and how the runtime will find the ground under a wheel that is not
    // touching anything).  `direction` need not be unit length; `maxDistance`
    // is in metres along it.
    struct RayHit {
        bool hit = false;
        Vec3 point{};
        f64 fraction = 1.0;      // of maxDistance
        J::BodyID body;
        ObjectLayerId layer = LayerStatic;
    };
    RayHit rayCast(const Vec3& from, const Vec3& direction, f64 maxDistance) const;
    // The same cast, but dynamic bodies are invisible to it (see StaticBodyFilter).
    RayHit rayCastStatic(const Vec3& from, const Vec3& direction, f64 maxDistance) const;
    // Convenience: straight down from `from`, returning the surface height under
    // it, or `NAN` when nothing is below.
    f64 groundHeight(const Vec3& from, f64 maxDrop = 200.0) const;
    f64 groundHeightStatic(const Vec3& from, f64 maxDrop = 200.0) const;

    // Position of a body's centre of mass, world frame.
    Vec3 position(const BodyHandle& h) const;
    Vec3 linearVelocity(const BodyHandle& h) const;
    void setPosition(const BodyHandle& h, const Vec3& p);
    void remove(const BodyHandle& h);

    // Diagnostic string for the harness's report.
    std::string versionString() const;

private:
    JPH::TempAllocatorImpl* m_temp = nullptr;
    JPH::JobSystemThreadPool* m_jobs = nullptr;
    JPH::PhysicsSystem* m_system = nullptr;
    // Scratch for `rebase`: the target position of every body, kept across calls
    // so a cell crossing allocates nothing.
    std::vector<Vec3> m_rebaseTargets;
    BroadPhaseLayerInterfaceImpl m_bpFilter;
    ObjectVsBroadPhaseLayerFilterImpl m_ovbp;
    ObjectLayerPairFilterImpl m_ovl;
    PenetrationStats m_penetration;
    u32 m_maxBodies = 0;
    bool m_ready = false;
};

// The contact listener that measures penetration.  Exposed so the harness can
// reset the statistics mid-run.
class PenetrationListener final : public JPH::ContactListener {
public:
    explicit PenetrationListener(PenetrationStats* stats) : m_stats(stats) {}
    void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2,
                        const JPH::ContactManifold& inManifold,
                        JPH::ContactSettings& ioSettings) override;
    void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2,
                            const JPH::ContactManifold& inManifold,
                            JPH::ContactSettings& ioSettings) override;

private:
    void record(const JPH::Body& b1, const JPH::Body& b2, const JPH::ContactManifold& m);
    PenetrationStats* m_stats;
};

}  // namespace mumbai::phys
