// =============================================================================
//  Mumbai 1:1 EXACT — engine/physics/vehicle.h
//
//  Physical vehicles: the same five archetypes the traffic layer routes around
//  the city (engine/sim/traffic.h is the single source of their published
//  dimensions and masses), given bodies, wheels, suspension and a controller,
//  so a motorcycle that clips a kerb at 40 km/h behaves like a motorcycle.
//
//  The two controllers are Jolt's, and they differ in kind, not in tuning:
//    * `WheeledVehicleController` — cars, taxis, autos and buses.  Four wheels
//      (or three, for the auto), a differential, anti-roll bars on the bus.
//    * `MotorcycleController` — two-wheelers, with a lean spring that holds the
//      bike up and a lean steering limit that stops the rider asking for a turn
//      the bike would fall into.  That is the specification's "rollover
//      geometry" made physical rather than scripted.
// =============================================================================
#pragma once

#include "engine/core/core.h"
#include "engine/physics/world.h"
#include "engine/sim/traffic.h"

#include <string>
#include <vector>

namespace mumbai::phys {

// One physical vehicle: a chassis body, a constraint, a controller.
struct Vehicle {
    BodyHandle body;
    JPH::VehicleConstraint* constraint = nullptr;
    JPH::WheeledVehicleController* controller = nullptr;
    JPH::MotorcycleController* motorcycle = nullptr;   // null unless two-wheeled
    sim::Archetype kind = sim::Archetype::Car;
    f64 wheelRadius = 0.3;
    f64 wheelbase = 2.4;
    f64 massKg = 0;
    f64 halfWidth = 0;
    f64 halfLength = 0;
    f64 halfHeight = 0;
    bool valid() const { return body.valid() && constraint != nullptr; }
};

// Creates a physical vehicle at `position` facing `yawRadians`, using the
// archetype's published length, width and mass (traffic.h).  The chassis is a
// box with its centre of mass dropped below the geometric centre — a bus whose
// mass sits at roof height tips over at the first roundabout, which is the
// failure the specification's rollover note is about.
Vehicle spawnVehicle(World& world, sim::Archetype kind, const Vec3& position, f64 yawRadians);

// Removes the vehicle: step listener, constraint, then body, in that order.
void destroyVehicle(World& world, Vehicle& v);

// Driver input in the controller's own terms: forward/right in [-1, 1], brake
// and handbrake in [0, 1].
void driveVehicle(Vehicle& v, f32 forward, f32 right, f32 brake, f32 handBrake = 0.0f);

// Speed along the vehicle's forward axis, m/s.
f64 forwardSpeed(World& world, const Vehicle& v);

// Lean angle from vertical, degrees — the number a motorcycle's rollover
// criterion is written in.
f64 leanAngleDegrees(World& world, const Vehicle& v);

// How many of the vehicle's wheels are touching something.
int wheelsOnGround(const Vehicle& v);

}  // namespace mumbai::phys
