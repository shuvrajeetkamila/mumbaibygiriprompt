// =============================================================================
//  Mumbai 1:1 EXACT — engine/physics/jolt.h
//
//  One include for the vendored physics engine, and the layer vocabulary the
//  world's collision geometry is registered into.
//
//  Jolt is compiled with a set of instruction-set and feature defines that must
//  match in every translation unit that includes its headers (Jolt's own CMake
//  says so: "a mismatch causes link errors").  The Makefile carries those defines
//  for both the vendored objects and engine/physics, and this file asserts the
//  contract instead of hoping: if the flags ever diverge, the build stops here
//  with a readable message rather than at a link error three minutes later.
// =============================================================================
#pragma once

// Which defines "the same" means depends on the architecture, and the guard has
// to say so: requiring JPH_USE_SSE4_1 on an arm64 device made the Android build
// fail with this very message, which is the guard doing its job on a contract
// that was only ever written for x86.  Jolt's own Core.h defines JPH_USE_NEON
// for aarch64 and nothing for x86 (the SSE defines come from the build), so:
#if defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
  #if !defined(JPH_USE_NEON)
  #error "engine/physics must be compiled with JPH_USE_NEON on ARM, the define the vendored Jolt objects use (see MUMBAI_JOLT_DEFINES in android/CMakeLists.txt)."
  #endif
#elif defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
  #if !defined(JPH_USE_SSE4_1) || !defined(JPH_USE_SSE4_2)
  #error "engine/physics must be compiled with the same instruction-set defines as the vendored Jolt objects (see JOLT_FLAGS in the Makefile)."
  #endif
#else
  #error "unknown CPU architecture: Jolt's instruction-set defines have not been decided for it."
#endif

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/RegisterTypes.h>

namespace mumbai::phys {

// Jolt's names always carry this alias in engine/physics.  Jolt and the engine
// both define Vec3, Quat and Body, and a using-directive makes every one of
// those a coin toss decided by include order; an alias makes the physics code
// say which engine's vector it means.
namespace J = ::JPH;

// Object layers.  Static world geometry never moves; vehicles are the bodies the
// runtime simulates; props are street furniture that a vehicle may knock over.
enum ObjectLayerId : JPH::ObjectLayer {
    LayerStatic  = 0,
    LayerVehicle = 1,
    LayerProp    = 2,
    LayerCount   = 3,
};

enum BroadPhaseLayerId : JPH::BroadPhaseLayer::Type {
    BPLayerStatic  = 0,
    BPLayerDynamic = 1,
    BPLayerCount   = 2,
};

// A filter that admits only static bodies: the semantic of a probe ray.  A
// probe wants the ground, not the wheel that is standing on it, and the wheel's
// own body is a dynamic body the probe would otherwise hit first.
class StaticBodyFilter final : public JPH::BodyFilter {
public:
    bool ShouldCollideLocked(const JPH::Body& inBody) const override { return inBody.IsStatic(); }
};

// Collision filters.  Static geometry does not collide with itself (the world's
// boxes are placed to touch), everything else collides with everything.
class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BPLayerCount; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        return JPH::BroadPhaseLayer(inLayer == LayerStatic ? BPLayerStatic : BPLayerDynamic);
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        return inLayer == JPH::BroadPhaseLayer(BPLayerStatic) ? "static" : "dynamic";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer, JPH::BroadPhaseLayer inBPLayer) const override {
        return !(inLayer == LayerStatic && inBPLayer == JPH::BroadPhaseLayer(BPLayerStatic));
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        return !(a == LayerStatic && b == LayerStatic);
    }
};

}  // namespace mumbai::phys
