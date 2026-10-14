// =============================================================================
//  engine/physics/vehicle.cpp — see vehicle.h for the design notes.
//
//  The numbers are the ones the city drives by.  Every dimension and mass comes
//  from `archetypeSpec()` in engine/sim/traffic.h rather than from a second
//  table here: the traffic layer's fleet mix and the physical layer's chassis
//  are the same vehicles, and a project where a "BEST bus" is 12 m long in one
//  layer and 9 m in the other is a project with two buses.
// =============================================================================
#include "engine/physics/vehicle.h"

#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>

namespace mumbai::phys {

namespace {

// A wheel's suspension and tyre, scaled to the vehicle.  `caster` tilts the
// suspension axis back, which is what makes a two-wheeler stable; a car's strut
// is near-vertical, so the angle is zero.
void configureWheel(J::WheelSettingsWV* w, f32 radius, f32 width, const J::Vec3& position,
                    f32 maxSteer, f32 suspensionMin, f32 suspensionMax, f32 frequency,
                    f32 brakeTorque, f32 casterDegrees) {
    w->mPosition = position;
    w->mRadius = radius;
    w->mWidth = width;
    w->mMaxSteerAngle = maxSteer;
    w->mSuspensionMinLength = suspensionMin;
    w->mSuspensionMaxLength = suspensionMax;
    w->mSuspensionSpring.mFrequency = frequency;
    w->mSuspensionSpring.mDamping = 0.5f;
    w->mMaxBrakeTorque = brakeTorque;
    w->mMaxHandBrakeTorque = 0.0f;
    // Tyre curves: peak grip at zero slip, falling as the wheel spins or slides.
    // The x axis is the slip ratio for the longitudinal curve and the slip angle
    // in degrees for the lateral one.
    w->mLongitudinalFriction.Clear();
    w->mLongitudinalFriction.AddPoint(-1.0f, 1.2f);
    w->mLongitudinalFriction.AddPoint(0.0f, 3.0f);
    w->mLongitudinalFriction.AddPoint(1.0f, 1.0f);
    w->mLateralFriction.Clear();
    w->mLateralFriction.AddPoint(-10.0f, 3.0f);
    w->mLateralFriction.AddPoint(0.0f, 3.0f);
    w->mLateralFriction.AddPoint(10.0f, 1.2f);
    if (casterDegrees > 0.0f) {
        const f32 c = std::tan(J::DegreesToRadians(casterDegrees));
        w->mSuspensionDirection = J::Vec3(0, -1, c).Normalized();
        w->mSteeringAxis = -w->mSuspensionDirection;
    }
}

}  // namespace

Vehicle spawnVehicle(World& world, sim::Archetype kind, const Vec3& position, f64 yawRadians) {
    Vehicle v;
    const sim::ArchetypeSpec& spec = sim::archetypeSpec(kind);
    v.kind = kind;
    v.massKg = spec.massKg;
    v.halfLength = spec.length * 0.5;
    v.halfWidth = spec.width * 0.5;

    const bool twoWheeler = spec.twoWheeler;
    const f64 wheelRadius = twoWheeler ? 0.31
                          : kind == sim::Archetype::AutoRickshaw ? 0.24
                                                            : 0.34;
    v.wheelRadius = wheelRadius;
    // Body height is the *chassis* box the wheels hang from: the floor, the
    // suspension and the running gear.  A BEST double-decker is 4.3 m tall and
    // its mass is in the chassis, not the roof — which is exactly why its centre
    // of mass sits below its geometric centre and why it carries anti-roll bars.
    const f64 bodyHeight = twoWheeler ? 0.62
                          : kind == sim::Archetype::AutoRickshaw ? 1.20
                          : kind == sim::Archetype::Bus ? 4.30
                                                        : 1.45;
    v.halfHeight = bodyHeight * 0.5;
    // A two-wheeler's wheels are at its ends; a four-wheeler's are at 60 % of
    // its length, which is where real vehicles put them.
    v.wheelbase = twoWheeler ? spec.length - 2.0 * wheelRadius : spec.length * 0.60;
    const f64 track = twoWheeler ? 0.0 : spec.width * 0.80;

    // Chassis: a box with its centre of mass dropped below the geometric centre.
    // A vehicle whose centre of mass sits at its geometric centre tips over at
    // the first roundabout — the failure the specification's rollover note is
    // about — and a 16 t double-decker is the worst case in the fleet.
    const f64 comDrop = kind == sim::Archetype::Bus ? 1.00
                      : twoWheeler ? 0.22
                      : kind == sim::Archetype::AutoRickshaw ? 0.20
                                                            : 0.28;
    J::RefConst<J::Shape> chassis = new J::BoxShape(J::Vec3(static_cast<f32>(v.halfWidth),
                                                            static_cast<f32>(v.halfHeight),
                                                            static_cast<f32>(v.halfLength)));
    J::RefConst<J::Shape> com = new J::OffsetCenterOfMassShape(
        chassis, J::Vec3(0.0f, static_cast<f32>(-comDrop), 0.0f));

    J::BodyCreationSettings bcs(com, J::RVec3(position.x, position.y, position.z),
                                J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yawRadians)),
                                J::EMotionType::Dynamic, LayerVehicle);
    bcs.mOverrideMassProperties = J::EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = static_cast<f32>(spec.massKg);
    bcs.mFriction = 0.9f;
    bcs.mRestitution = 0.02f;
    bcs.mAllowSleeping = true;      // an idle vehicle must not cost solver time
    // Linear cast: a vehicle at 100 km/h moves 0.46 m per tick, and the Sea
    // Link's deck slab is 0.68 m thick — a discrete body can be inside the deck
    // for a tick before it is pushed back out, which the A2 harness measured as
    // 0.09 m of penetration.  The sweep costs a fraction of a narrow-phase query
    // and only when the body is actually moving fast.
    bcs.mMotionQuality = J::EMotionQuality::LinearCast;
    J::Body* body = world.createBody(bcs);
    if (body == nullptr) return v;
    world.bodies().AddBody(body->GetID(), J::EActivation::Activate);
    v.body = BodyHandle{body->GetID(), LayerVehicle};

    // --- constraint and wheels -------------------------------------------------
    J::VehicleConstraintSettings settings;
    settings.mUp = J::Vec3::sAxisY();
    settings.mForward = J::Vec3::sAxisZ();
    settings.mMaxPitchRollAngle = J::DegreesToRadians(kind == sim::Archetype::Bus ? 55.0f : 60.0f);
    const f32 axleY = static_cast<f32>(-v.halfHeight + wheelRadius * 0.35);

    std::vector<J::WheelSettingsWV*> wheels;
    if (twoWheeler) {
        J::WheelSettingsWV* front = new J::WheelSettingsWV;
        configureWheel(front, static_cast<f32>(wheelRadius), 0.06f,
                       J::Vec3(0.0f, axleY, static_cast<f32>(v.wheelbase * 0.5)),
                       J::DegreesToRadians(30.0f), 0.30f, 0.50f, 1.5f, 500.0f, 30.0f);
        J::WheelSettingsWV* back = new J::WheelSettingsWV;
        configureWheel(back, static_cast<f32>(wheelRadius), 0.07f,
                       J::Vec3(0.0f, axleY, static_cast<f32>(-v.wheelbase * 0.5)),
                       0.0f, 0.30f, 0.50f, 2.0f, 250.0f, 0.0f);
        settings.mWheels = { front, back };
        wheels = { front, back };

        J::MotorcycleControllerSettings* c = new J::MotorcycleControllerSettings;
        c->mMaxLeanAngle = J::DegreesToRadians(45.0f);
        c->mLeanSpringConstant = 5000.0f;
        c->mLeanSpringDamping = 1000.0f;
        c->mLeanSmoothingFactor = 0.8f;
        c->mEngine.mMaxTorque = kind == sim::Archetype::Motorcycle ? 150.0f : 60.0f;
        c->mEngine.mMinRPM = 1000.0f;
        c->mEngine.mMaxRPM = 10000.0f;
        c->mTransmission.mShiftDownRPM = 2000.0f;
        c->mTransmission.mShiftUpRPM = 8000.0f;
        c->mTransmission.mGearRatios = { 2.27f, 1.63f, 1.30f, 1.09f, 0.96f, 0.88f };
        c->mTransmission.mReverseGearRatios = { -4.0f };
        c->mTransmission.mClutchStrength = 2.0f;
        c->mDifferentials.resize(1);
        c->mDifferentials[0].mLeftWheel = -1;
        c->mDifferentials[0].mRightWheel = 1;
        c->mDifferentials[0].mDifferentialRatio = 1.93f * 40.0f / 16.0f;
        settings.mController = c;
    } else {
        auto make = [&](f32 z, f32 x, f32 steer) {
            J::WheelSettingsWV* w = new J::WheelSettingsWV;
            configureWheel(w, static_cast<f32>(wheelRadius),
                           kind == sim::Archetype::Bus ? 0.28f : 0.20f,
                           J::Vec3(x, axleY, z), steer, 0.20f, 0.45f,
                           kind == sim::Archetype::Bus ? 2.2f : 2.6f,
                           kind == sim::Archetype::Bus ? 3000.0f : 900.0f, 0.0f);
            return w;
        };
        const f32 front = static_cast<f32>(v.wheelbase * 0.5);
        const f32 rear = static_cast<f32>(-v.wheelbase * 0.5);
        if (kind != sim::Archetype::AutoRickshaw) {
            J::WheelSettingsWV* fl = make(front, static_cast<f32>(-track * 0.5), J::DegreesToRadians(32.0f));
            J::WheelSettingsWV* fr = make(front, static_cast<f32>(track * 0.5), J::DegreesToRadians(32.0f));
            J::WheelSettingsWV* rl = make(rear, static_cast<f32>(-track * 0.5), 0.0f);
            J::WheelSettingsWV* rr = make(rear, static_cast<f32>(track * 0.5), 0.0f);
            settings.mWheels = { fl, fr, rl, rr };
            wheels = { fl, fr, rl, rr };
        } else {
            // An auto-rickshaw: one front wheel, two at the back.  This is the
            // rollover geometry the specification calls out by name.
            J::WheelSettingsWV* f = make(front, 0.0f, J::DegreesToRadians(32.0f));
            J::WheelSettingsWV* l = make(rear, static_cast<f32>(-track * 0.5), 0.0f);
            J::WheelSettingsWV* r = make(rear, static_cast<f32>(track * 0.5), 0.0f);
            settings.mWheels = { f, l, r };
            wheels = { f, l, r };
        }

        J::WheeledVehicleControllerSettings* c = new J::WheeledVehicleControllerSettings;
        c->mEngine.mMaxTorque = kind == sim::Archetype::Bus ? 2500.0f
                              : kind == sim::Archetype::AutoRickshaw ? 120.0f
                                                                : 500.0f;
        // A bus engine redlines at 3000 rpm; everything else here at 7000.  The
        // gearbox must be scaled to the engine: Jolt asserts that an automatic
        // box shifts up below the engine's maximum rpm, and the A2 harness
        // aborted (SIGTRAP) on the first double-decker because the shared 4000
        // rpm shift point sat above a 3000 rpm redline.  Shift up at 90 % of the
        // redline and back down at half of that, per class.
        const f32 engineMaxRpm = kind == sim::Archetype::Bus ? 3000.0f : 7000.0f;
        c->mEngine.mMinRPM = 1000.0f;
        c->mEngine.mMaxRPM = engineMaxRpm;
        c->mTransmission.mShiftUpRPM = engineMaxRpm * 0.90f;
        c->mTransmission.mShiftDownRPM = engineMaxRpm * 0.45f;
        c->mTransmission.mGearRatios = { 3.0f, 2.1f, 1.5f, 1.1f, 0.85f };
        c->mTransmission.mReverseGearRatios = { -3.5f };
        c->mTransmission.mClutchStrength = 10.0f;
        c->mDifferentials.resize(1);
        c->mDifferentials[0].mLeftWheel = 0;
        c->mDifferentials[0].mRightWheel = 1;
        c->mDifferentials[0].mDifferentialRatio = 3.4f;
        settings.mController = c;
    }

    J::VehicleConstraint* constraint = new J::VehicleConstraint(*body, settings);
    // The wheel's collision test decides how a wheel meets the world.  The
    // cylinder tester follows the wheel's own profile, which is what makes a
    // kerb a step and not a wall; the ray tester would let a wheel sink through
    // a thin deck slab.
    constraint->SetVehicleCollisionTester(
        new J::VehicleCollisionTesterCastCylinder(LayerVehicle, 1.0f));
    // Jolt's constraint list is an `Array<Ref<Constraint>>`: adding hands the
    // engine the *only* reference, and removing it releases that reference and
    // deletes the constraint.  This project takes a reference of its own so the
    // vehicle wrapper can keep a raw pointer and release it on destroy — without
    // this, `RemoveConstraint` frees the object and the wrapper's `delete` is a
    // double free (found by the A2 harness, which crashed on the first
    // destroyed vehicle).
    world.system().AddConstraint(constraint);
    world.system().AddStepListener(constraint);
    constraint->AddRef();
    v.constraint = constraint;
    v.controller = static_cast<J::WheeledVehicleController*>(constraint->GetController());
    // The build has no RTTI (Jolt is compiled with -fno-rtti and so is this
    // translation unit), and it does not need any: which controller was
    // installed is exactly `twoWheeler`.
    v.motorcycle = twoWheeler ? static_cast<J::MotorcycleController*>(v.controller) : nullptr;
    // A bus leans; anti-roll bars are what stop a 16-tonne double-decker
    // rolling over in a lane change at 60 km/h.
    if (kind == sim::Archetype::Bus && wheels.size() >= 4) {
        J::VehicleAntiRollBar bar;
        bar.mLeftWheel = 0; bar.mRightWheel = 1; bar.mStiffness = 8000.0f;
        constraint->GetAntiRollBars().push_back(bar);
        bar.mLeftWheel = 2; bar.mRightWheel = 3; bar.mStiffness = 8000.0f;
        constraint->GetAntiRollBars().push_back(bar);
    }
    return v;
}

void destroyVehicle(World& world, Vehicle& v) {
    if (v.constraint != nullptr) {
        // The order matters: stop being a step listener, leave the constraint
        // list (which drops the engine's reference), then drop ours.
        world.system().RemoveStepListener(v.constraint);
        world.system().RemoveConstraint(v.constraint);
        v.constraint->Release();
        v.constraint = nullptr;
    }
    v.controller = nullptr;
    v.motorcycle = nullptr;
    if (v.body.valid()) {
        world.bodies().RemoveBody(v.body.id);
        world.bodies().DestroyBody(v.body.id);
        v.body = BodyHandle{};
    }
}

void driveVehicle(Vehicle& v, f32 forward, f32 right, f32 brake, f32 handBrake) {
    if (v.controller == nullptr) return;
    // Jolt's `SetDriverInput(inRight)` is left-positive in 5.6.0 — measured, not
    // read off the parameter name: `WheeledVehicleController::Drive` applies
    // `SetSteerAngle(-mRightInput * max)` and the motorcycle controller applies
    // `steer_sign = -Sign(mRightInput)`, and a positive wheel steer angle about
    // the wheel's +Y axis turns the wheel right in this world frame (x east,
    // y up, z north).  Two rides with a constant input (+0.35 and -0.35) on the
    // Sea Link deck are the evidence: the first turned left, the second right.
    // Callers of this function get the navigational sense — `right > 0` turns
    // the vehicle clockwise — because that is the sense the traffic layer, the
    // driver model and the acceptance harness all reason in.
    v.controller->SetDriverInput(forward, -right, brake, handBrake);
}

f64 forwardSpeed(World& world, const Vehicle& v) {
    if (!v.body.valid()) return 0.0;
    const Vec3 velocity = world.linearVelocity(v.body);
    const J::Quat q = world.bodies().GetRotation(v.body.id);
    const J::Vec3 forward = q.RotateAxisZ();
    return static_cast<f64>(velocity.x * forward.GetX() + velocity.y * forward.GetY() +
                            velocity.z * forward.GetZ());
}

f64 leanAngleDegrees(World& world, const Vehicle& v) {
    if (!v.body.valid() || v.constraint == nullptr) return 0.0;
    // Roll of the chassis about its own forward axis: the number a motorcycle's
    // rollover criterion is written in.
    const J::Quat q = world.bodies().GetRotation(v.body.id);
    const J::Vec3 vehicleUp = q.RotateAxisY();
    const J::Vec3 worldUp(0.0f, 1.0f, 0.0f);
    const f32 cosAngle = std::clamp(vehicleUp.Dot(worldUp), -1.0f, 1.0f);
    return static_cast<f64>(std::acos(cosAngle)) * 180.0 / 3.14159265358979323846;
}

int wheelsOnGround(const Vehicle& v) {
    if (v.constraint == nullptr) return 0;
    int n = 0;
    for (JPH::uint i = 0; i < v.constraint->GetWheels().size(); ++i)
        if (v.constraint->GetWheel(i)->HasContact()) ++n;
    return n;
}

}  // namespace mumbai::phys
