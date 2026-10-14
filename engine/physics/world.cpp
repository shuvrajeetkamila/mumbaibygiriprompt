// =============================================================================
//  engine/physics/world.cpp — see world.h for the design notes.
// =============================================================================
#include "engine/physics/world.h"

#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>

#include <chrono>
#include <cstring>
#include <limits>

namespace mumbai::phys {

// Jolt's types are written out in full here rather than pulled in with a
// using-directive: `Vec3`, `Quat` and `Body` exist in both engines, and an
// unqualified name that resolves differently depending on which headers a
// translation unit included is a bug waiting for a rebuild.
namespace {

// Jolt's global state is process-wide (the factory and the type register), so
// the world counts its instances and tears the global state down with the last
// one.  Two worlds in one process is what the harness does when it checks
// determinism, and a double RegisterTypes() is an assert in a debug Jolt build.
int g_worldInstances = 0;

J::Quat quatFromYaw(f64 yaw) { return J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)); }

// Jolt to engine vectors and back.  Written out rather than added as a
// converting constructor on mumbai::Vec3, which every other subsystem would
// then accidentally inherit.
template <typename V>
Vec3 fromJolt(const V& v) {
    // In a single-precision build JPH::RVec3 *is* JPH::Vec3, so one template
    // covers both; in a double-precision build it is a DVec3 and the cast is
    // what keeps the engine's float world honest.
    return Vec3(static_cast<f32>(v.GetX()), static_cast<f32>(v.GetY()), static_cast<f32>(v.GetZ()));
}

// The axis (local +Y) that a capsule written by the pipeline points along,
// reconstructed from its quaternion.
[[maybe_unused]] J::Vec3 capsuleAxis(const CollisionBox& b) {
    const J::Quat q(b.qx, b.qy, b.qz, b.qw);
    return q.RotateAxisY();
}

}  // namespace

// -----------------------------------------------------------------------------
//  Contact listener
// -----------------------------------------------------------------------------
// Jolt states the overlap directly: `mPenetrationDepth` is "how far to move
// shape 2 to resolve the collision", and it is negative for a speculative
// contact (a contact generated before the bodies actually touch, which is how a
// fast body is stopped from tunnelling).  So the measurement is not a
// reconstruction from contact points — it is the solver's own number, and the
// deepest one over a run is the honest answer to "was anything ever inside
// anything else, and by how much".
void PenetrationListener::record(const J::Body& b1, const J::Body& b2,
                                 const J::ContactManifold& m) {
    ++m_stats->manifolds;
    m_stats->contacts += m.mRelativeContactPointsOn1.size();
    const f64 penetration = static_cast<f64>(m.mPenetrationDepth);
    if (penetration <= 0.0) return;
    const bool dynamicPair = b1.IsDynamic() && b2.IsDynamic();
    f64& slot = dynamicPair ? m_stats->deepestDynamic : m_stats->deepestStatic;
    if (penetration > slot) slot = penetration;
}

void PenetrationListener::OnContactAdded(const J::Body& b1, const J::Body& b2,
                                         const J::ContactManifold& m, J::ContactSettings&) {
    record(b1, b2, m);
}
void PenetrationListener::OnContactPersisted(const J::Body& b1, const J::Body& b2,
                                             const J::ContactManifold& m, J::ContactSettings&) {
    record(b1, b2, m);
}

// -----------------------------------------------------------------------------
//  World
// -----------------------------------------------------------------------------
World::World() = default;

World::~World() { shutdown(); }

bool World::init(u32 maxBodies, u32 tempPoolBytes, int jobThreads) {
    if (m_ready) return true;
    if (g_worldInstances++ == 0) {
        J::RegisterDefaultAllocator();
        J::Factory::sInstance = new J::Factory();
        J::RegisterTypes();
    }
    m_maxBodies = maxBodies;
    m_temp = new J::TempAllocatorImpl(tempPoolBytes);
    m_jobs = new J::JobSystemThreadPool(J::cMaxPhysicsJobs, J::cMaxPhysicsBarriers, jobThreads);

    m_system = new J::PhysicsSystem();
    // Body capacity, and the same number for the mutex pool (a smaller mutex
    // pool serialises unrelated bodies and shows up as solver tail latency).
    // 0 buckets asks Jolt to size the broadphase from the body count.
    m_system->Init(maxBodies, 0, maxBodies, 1024, m_bpFilter, m_ovbp, m_ovl);
    // The default penetration slop is 20 mm: a "resting" body is allowed to sit
    // 20 mm inside the surface it rests on, which is 20x the accuracy this
    // project's acceptance criteria ask for.  5 mm keeps the solver stable
    // (Jolt's own guidance is 1-5 cm for stability) while making the measured
    // penetration meaningful.
    J::PhysicsSettings settings = m_system->GetPhysicsSettings();
    settings.mPenetrationSlop = 0.005f;
    settings.mSpeculativeContactDistance = 0.02f;   // default, kept explicit
    // Jolt only sweeps a linear-cast body when it moves further than
    // `mLinearCastThreshold` * the body's inner radius in one tick.  The default
    // of 0.75 is too late for this project: the A2 harness dropped a 300 kg crate
    // onto the Sea Link deck from 22 m (19.5 m/s, 0.33 m per tick, inner radius
    // 0.8 m, threshold 0.6 m) and Jolt never cast at all, leaving a measured
    // 0.21 m interpenetration for one tick.  At 0.25 a vehicle sweeps from about
    // 50 km/h upward and a crate from about 9 m/s — the speeds at which a body
    // would otherwise pass into the deck, a kerb or a wall by a visible margin.
    settings.mLinearCastThreshold = 0.25f;
    m_system->SetPhysicsSettings(settings);

    m_system->SetContactListener(new PenetrationListener(&m_penetration));
    m_ready = true;
    return true;
}

void World::shutdown() {
    if (!m_ready) return;
    delete m_system; m_system = nullptr;
    delete m_jobs; m_jobs = nullptr;
    delete m_temp; m_temp = nullptr;
    m_ready = false;
    if (--g_worldInstances == 0) {
        J::UnregisterTypes();
        delete J::Factory::sInstance;
        J::Factory::sInstance = nullptr;
    }
}

BodyHandle World::addStaticCompound(const std::vector<CollisionBox>& boxes) {
    BodyHandle h;
    if (!m_ready || boxes.empty()) return h;
    J::StaticCompoundShapeSettings compound;
    for (const CollisionBox& b : boxes) {
        const bool capsule = (b.flags & 8u) != 0u;
        if (capsule) {
            // A capsule's half height excludes the hemispherical caps: the
            // pipeline writes the full half length in hy and the radius in hx.
            const f32 radius = std::max(0.05f, b.hx);
            const f32 half = std::max(0.0f, b.hy - radius);
            const J::Quat q(b.qx, b.qy, b.qz, b.qw);
            compound.AddShape(J::Vec3(b.cx, b.cy, b.cz), q, new J::CapsuleShape(half, radius));
        } else {
            const f32 hx = std::max(0.02f, b.hx);
            const f32 hy = std::max(0.02f, b.hy);
            const f32 hz = std::max(0.02f, b.hz);
            const J::Quat q(b.qx, b.qy, b.qz, b.qw);
            compound.AddShape(J::Vec3(b.cx, b.cy, b.cz), q, new J::BoxShape(J::Vec3(hx, hy, hz)));
        }
    }
    J::ShapeSettings::ShapeResult result = compound.Create();
    if (result.HasError()) {
        MLOG_WARN("physics: compound shape rejected: %s", result.GetError().c_str());
        return h;
    }
    J::BodyCreationSettings bcs(result.Get(), J::RVec3::sZero(), J::Quat::sIdentity(),
                                J::EMotionType::Static, LayerStatic);
    h.id = m_system->GetBodyInterface().CreateAndAddBody(bcs, J::EActivation::DontActivate);
    h.layer = LayerStatic;
    return h;
}

BodyHandle World::addStaticBox(const Vec3& center, const Vec3& halfExtents, f64 yawRadians) {
    BodyHandle h;
    if (!m_ready) return h;
    J::BodyCreationSettings bcs(new J::BoxShape(J::Vec3(halfExtents.x, halfExtents.y, halfExtents.z)),
                                J::RVec3(center.x, center.y, center.z), quatFromYaw(yawRadians),
                                J::EMotionType::Static, LayerStatic);
    h.id = m_system->GetBodyInterface().CreateAndAddBody(bcs, J::EActivation::DontActivate);
    return h;
}

BodyHandle World::addStaticCapsule(const Vec3& a, const Vec3& b, f64 radius) {
    BodyHandle h;
    if (!m_ready) return h;
    const Vec3 d = b - a;
    const f32 len = static_cast<f32>(d.length());
    if (len < 1e-4f) return h;
    const Vec3 mid = (a + b) * 0.5;
    const Vec3 axis = d * (1.0 / static_cast<f64>(len));
    // Quaternion taking local +Y onto the capsule's axis (shortest arc).
    const f32 w = std::sqrt(std::max(0.0f, (1.0f + static_cast<f32>(axis.y)) * 0.5f));
    const f32 s = std::sqrt(std::max(0.0f, (1.0f - static_cast<f32>(axis.y)) * 0.5f));
    const f64 cross = std::sqrt(axis.z * axis.z + axis.x * axis.x);
    const J::Quat q = cross > 1e-9
        ? J::Quat(static_cast<f32>(axis.z / cross) * s, 0.0f,
                  static_cast<f32>(-axis.x / cross) * s, w)
        : J::Quat::sIdentity();
    J::BodyCreationSettings bcs(
        new J::CapsuleShape(std::max(0.0f, len * 0.5f - static_cast<f32>(radius)),
                            static_cast<f32>(radius)),
        J::RVec3(mid.x, mid.y, mid.z), q, J::EMotionType::Static, LayerStatic);
    h.id = m_system->GetBodyInterface().CreateAndAddBody(bcs, J::EActivation::DontActivate);
    return h;
}

BodyHandle World::addDynamicBox(const Vec3& center, const Vec3& halfExtents, f64 yawRadians,
                                f64 massKg, const Vec3& centerOfMassOffset, ObjectLayerId layer,
                                MotionQuality quality) {
    BodyHandle h;
    if (!m_ready) return h;
    J::RefConst<J::Shape> shape =
        new J::BoxShape(J::Vec3(halfExtents.x, halfExtents.y, halfExtents.z));
    if (centerOfMassOffset.lengthSq() > 0.0)
        shape = new J::OffsetCenterOfMassShape(
            shape, J::Vec3(centerOfMassOffset.x, centerOfMassOffset.y, centerOfMassOffset.z));
    J::BodyCreationSettings bcs(shape, J::RVec3(center.x, center.y, center.z), quatFromYaw(yawRadians),
                                J::EMotionType::Dynamic, layer);
    bcs.mOverrideMassProperties = J::EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = static_cast<f32>(massKg);
    bcs.mFriction = 0.9f;
    bcs.mRestitution = 0.05f;
    bcs.mLinearDamping = 0.05f;
    bcs.mMotionQuality = quality == MotionQuality::LinearCast ? J::EMotionQuality::LinearCast
                                                             : J::EMotionQuality::Discrete;
    h.id = m_system->GetBodyInterface().CreateAndAddBody(bcs, J::EActivation::Activate);
    h.layer = layer;
    return h;
}

void World::step(f32 dt, u32 collisionSteps) {
    if (!m_ready) return;
    m_system->Update(dt, static_cast<int>(collisionSteps), m_temp, m_jobs);
}

World::RebaseResult World::rebase(const Vec3& delta) {
    RebaseResult r;
    if (!m_ready || (delta.x == 0.0f && delta.y == 0.0f && delta.z == 0.0f)) return r;
    const auto t0 = std::chrono::steady_clock::now();
    J::BodyInterface& bi = m_system->GetBodyInterface();
    J::BodyIDVector ids;
    m_system->GetBodies(ids);
    m_rebaseTargets.resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
        const Vec3 target = fromJolt(bi.GetPosition(ids[i])) - delta;
        m_rebaseTargets[i] = target;
        // DontActivate: a shift is not a disturbance.  Waking 600 bodies on a
        // cell boundary would turn a bookkeeping operation into a simulation
        // event, and the solver would report it as an impulse.
        bi.SetPosition(ids[i], J::RVec3(target.x, target.y, target.z), J::EActivation::DontActivate);
    }
    const auto t1 = std::chrono::steady_clock::now();
    r.bodies = ids.size();
    r.milliseconds = std::chrono::duration<f64, std::milli>(t1 - t0).count();
    // Fidelity, measured by reading every body back.  It is outside the timer on
    // purpose: the criterion is the duration of the shift, and a check that costs
    // more than the thing it checks should not be paid for inside it.
    for (size_t i = 0; i < ids.size(); ++i) {
        const Vec3 after = fromJolt(bi.GetPosition(ids[i]));
        const Vec3 want = m_rebaseTargets[i];
        r.worstPositionError = std::max(
            r.worstPositionError,
            std::max(std::fabs(static_cast<f64>(after.x) - want.x),
                     std::max(std::fabs(static_cast<f64>(after.y) - want.y),
                              std::fabs(static_cast<f64>(after.z) - want.z))));
    }
    return r;
}

u32 World::bodyCount() const { return m_ready ? m_system->GetNumBodies() : 0; }

u32 World::activeBodyCount() const {
    return m_ready ? m_system->GetNumActiveBodies(J::EBodyType::RigidBody) : 0;
}

World::RayHit World::rayCast(const Vec3& from, const Vec3& direction, f64 maxDistance) const {
    RayHit out;
    if (!m_ready || maxDistance <= 0.0) return out;
    const f32 len = static_cast<f32>(std::sqrt(static_cast<f64>(direction.x) * direction.x +
                                               static_cast<f64>(direction.y) * direction.y +
                                               static_cast<f64>(direction.z) * direction.z));
    if (len < 1e-6f) return out;
    const J::Vec3 d(direction.x, direction.y, direction.z);
    const J::RRayCast ray(J::RVec3(from.x, from.y, from.z),
                          d * (static_cast<f32>(maxDistance) / len));
    J::RayCastResult result;
    // Default (empty) filters: a probe ray sees every layer.  Only the static
    // world is ever in the way here — the vehicles are dynamic bodies and a
    // probe wants the deck, not the wheel that is standing on it.
    if (!m_system->GetNarrowPhaseQuery().CastRay(ray, result)) return out;
    out.hit = true;
    out.fraction = static_cast<f64>(result.mFraction);
    const J::RVec3 p = ray.GetPointOnRay(result.mFraction);
    out.point = Vec3(static_cast<f32>(p.GetX()), static_cast<f32>(p.GetY()), static_cast<f32>(p.GetZ()));
    out.body = result.mBodyID;
    return out;
}

World::RayHit World::rayCastStatic(const Vec3& from, const Vec3& direction, f64 maxDistance) const {
    RayHit out;
    if (!m_ready || maxDistance <= 0.0) return out;
    const f32 len = static_cast<f32>(std::sqrt(static_cast<f64>(direction.x) * direction.x +
                                               static_cast<f64>(direction.y) * direction.y +
                                               static_cast<f64>(direction.z) * direction.z));
    if (len < 1e-6f) return out;
    const J::Vec3 d(direction.x, direction.y, direction.z);
    const J::RRayCast ray(J::RVec3(from.x, from.y, from.z),
                          d * (static_cast<f32>(maxDistance) / len));
    J::RayCastResult result;
    static const StaticBodyFilter filter;
    if (!m_system->GetNarrowPhaseQuery().CastRay(ray, result, J::BroadPhaseLayerFilter(),
                                                 J::ObjectLayerFilter(), filter))
        return out;
    out.hit = true;
    out.fraction = static_cast<f64>(result.mFraction);
    const J::RVec3 p = ray.GetPointOnRay(result.mFraction);
    out.point = Vec3(static_cast<f32>(p.GetX()), static_cast<f32>(p.GetY()), static_cast<f32>(p.GetZ()));
    out.body = result.mBodyID;
    return out;
}

f64 World::groundHeight(const Vec3& from, f64 maxDrop) const {
    const RayHit h = rayCast(from, Vec3(0.0f, -1.0f, 0.0f), maxDrop);
    return h.hit ? static_cast<f64>(h.point.y) : std::numeric_limits<f64>::quiet_NaN();
}

f64 World::groundHeightStatic(const Vec3& from, f64 maxDrop) const {
    const RayHit h = rayCastStatic(from, Vec3(0.0f, -1.0f, 0.0f), maxDrop);
    return h.hit ? static_cast<f64>(h.point.y) : std::numeric_limits<f64>::quiet_NaN();
}

Vec3 World::position(const BodyHandle& h) const {
    if (!m_ready || !h.valid()) return {0, 0, 0};
    return fromJolt(m_system->GetBodyInterface().GetPosition(h.id));
}

Vec3 World::linearVelocity(const BodyHandle& h) const {
    if (!m_ready || !h.valid()) return {0, 0, 0};
    return fromJolt(m_system->GetBodyInterface().GetLinearVelocity(h.id));
}

void World::setPosition(const BodyHandle& h, const Vec3& p) {
    if (!m_ready || !h.valid()) return;
    m_system->GetBodyInterface().SetPosition(h.id, J::RVec3(p.x, p.y, p.z), J::EActivation::Activate);
}

void World::remove(const BodyHandle& h) {
    if (!m_ready || !h.valid()) return;
    m_system->GetBodyInterface().RemoveBody(h.id);
    m_system->GetBodyInterface().DestroyBody(h.id);
}

std::string World::versionString() const {
    // The version *and* the solver configuration, because the second is part of
    // every number the harness prints: a claim about penetration is a claim about
    // a slop, and a claim about an impact is a claim about a cast threshold.
    char buf[224];
    const J::PhysicsSettings s = m_system->GetPhysicsSettings();
    std::snprintf(buf, sizeof(buf),
                  "Jolt %d.%d.%d (solver %.0f velocity / %.0f position steps, slop %.0f mm, "
                  "linear cast at %.2f r, speculative %.0f mm)",
                  JPH_VERSION_MAJOR, JPH_VERSION_MINOR, JPH_VERSION_PATCH,
                  static_cast<f64>(s.mNumVelocitySteps), static_cast<f64>(s.mNumPositionSteps),
                  static_cast<f64>(s.mPenetrationSlop) * 1000.0,
                  static_cast<f64>(s.mLinearCastThreshold),
                  static_cast<f64>(s.mSpeculativeContactDistance) * 1000.0);
    return buf;
}

}  // namespace mumbai::phys
