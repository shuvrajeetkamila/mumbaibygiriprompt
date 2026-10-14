# =============================================================================
#  third_party/jolt_sources.mk
#
#  The hand-written build of Jolt Physics (D-033).  Jolt ships CMake; this
#  environment has no cmake, so the library's own file list (upstream
#  Jolt/Jolt.cmake) is transcribed here verbatim, minus the parts this project
#  does not build.  Everything removed is listed with the reason, and nothing
#  else was added:
#
#    Compute/          DX12, Vulkan and Metal compute backends, plus the CPU
#                      compute fallback.  They need platform SDKs (d3d12.h,
#                      vulkan.h, Metal) and they exist for GPU soft bodies,
#                      which no system in this project uses.  JPH_USE_*_COMPUTE
#                      stays undefined, so no code path reaches for them.
#    Renderer/         The debug renderer's translation units.  Its *headers*
#                      stay (Body.cpp includes DebugRenderer.h inside
#                      #ifdef JPH_DEBUG_RENDERER); nothing is compiled and no
#                      symbol is referenced, because JPH_DEBUG_RENDERER is off.
#    Shaders/          Compute shader sources — only used by Compute/.
#    ObjectStream/     The serialisation format.  Our world format is
#                      .mumbai_chunk; JPH_OBJECT_STREAM is undefined and the
#                      stage (re)builds from the pipeline every time.
#
#  Build configuration, mirroring the upstream "Distribution" preset:
#    JPH_ENABLE_ASSERTS      off — the shipping configuration; the harness
#                            validates outcomes, not internal invariants.
#    JPH_PROFILE_ENABLED     off — a profiler inside the physics step would
#                            distort the timings this harness exists to take.
#    JPH_DEBUG_RENDERER      off.
#    JPH_OBJECT_STREAM       off.
#    JPH_USE_SSE4_1/4_2      on  — the x86-64 baseline (2009+); AVX2 is
#    JPH_USE_LZCNT/TZCNT     on    deliberately *not* used, so a binary built
#                            here runs on any x86-64 host, and the Android
#                            target gets NEON through the same headers.
#    JPH_USE_CPU_COMPUTE     off.
# =============================================================================

JOLT_DIR := third_party/Jolt

JOLT_SRC := \
	Jolt/AABBTree/AABBTreeBuilder.cpp \
	Jolt/Core/Color.cpp \
	Jolt/Core/Factory.cpp \
	Jolt/Core/IssueReporting.cpp \
	Jolt/Core/JobSystemSingleThreaded.cpp \
	Jolt/Core/JobSystemThreadPool.cpp \
	Jolt/Core/JobSystemWithBarrier.cpp \
	Jolt/Core/LinearCurve.cpp \
	Jolt/Core/Memory.cpp \
	Jolt/Core/Profiler.cpp \
	Jolt/Core/RTTI.cpp \
	Jolt/Core/Semaphore.cpp \
	Jolt/Core/StringTools.cpp \
	Jolt/Core/TickCounter.cpp \
	Jolt/Geometry/ConvexHullBuilder.cpp \
	Jolt/Geometry/ConvexHullBuilder2D.cpp \
	Jolt/Geometry/Indexify.cpp \
	Jolt/Geometry/OrientedBox.cpp \
	Jolt/Math/Vec3.cpp \
	Jolt/ObjectStream/SerializableObject.cpp \
	Jolt/Physics/Body/Body.cpp \
	Jolt/Physics/Body/BodyCreationSettings.cpp \
	Jolt/Physics/Body/BodyInterface.cpp \
	Jolt/Physics/Body/BodyManager.cpp \
	Jolt/Physics/Body/MassProperties.cpp \
	Jolt/Physics/Body/MotionProperties.cpp \
	Jolt/Physics/Character/Character.cpp \
	Jolt/Physics/Character/CharacterBase.cpp \
	Jolt/Physics/Character/CharacterVirtual.cpp \
	Jolt/Physics/Collision/BroadPhase/BroadPhase.cpp \
	Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.cpp \
	Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.cpp \
	Jolt/Physics/Collision/BroadPhase/QuadTree.cpp \
	Jolt/Physics/Collision/CastConvexVsTriangles.cpp \
	Jolt/Physics/Collision/CastSphereVsTriangles.cpp \
	Jolt/Physics/Collision/CollideConvexVsTriangles.cpp \
	Jolt/Physics/Collision/CollideSphereVsTriangles.cpp \
	Jolt/Physics/Collision/CollisionDispatch.cpp \
	Jolt/Physics/Collision/CollisionGroup.cpp \
	Jolt/Physics/Collision/EstimateCollisionResponse.cpp \
	Jolt/Physics/Collision/GroupFilter.cpp \
	Jolt/Physics/Collision/GroupFilterTable.cpp \
	Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp \
	Jolt/Physics/Collision/NarrowPhaseQuery.cpp \
	Jolt/Physics/Collision/NarrowPhaseStats.cpp \
	Jolt/Physics/Collision/PhysicsMaterial.cpp \
	Jolt/Physics/Collision/PhysicsMaterialSimple.cpp \
	Jolt/Physics/Collision/Shape/BoxShape.cpp \
	Jolt/Physics/Collision/Shape/CapsuleShape.cpp \
	Jolt/Physics/Collision/Shape/CompoundShape.cpp \
	Jolt/Physics/Collision/Shape/ConvexHullShape.cpp \
	Jolt/Physics/Collision/Shape/ConvexShape.cpp \
	Jolt/Physics/Collision/Shape/CylinderShape.cpp \
	Jolt/Physics/Collision/Shape/DecoratedShape.cpp \
	Jolt/Physics/Collision/Shape/EmptyShape.cpp \
	Jolt/Physics/Collision/Shape/HeightFieldShape.cpp \
	Jolt/Physics/Collision/Shape/MeshShape.cpp \
	Jolt/Physics/Collision/Shape/MutableCompoundShape.cpp \
	Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.cpp \
	Jolt/Physics/Collision/Shape/PlaneShape.cpp \
	Jolt/Physics/Collision/Shape/RotatedTranslatedShape.cpp \
	Jolt/Physics/Collision/Shape/ScaledShape.cpp \
	Jolt/Physics/Collision/Shape/Shape.cpp \
	Jolt/Physics/Collision/Shape/SphereShape.cpp \
	Jolt/Physics/Collision/Shape/StaticCompoundShape.cpp \
	Jolt/Physics/Collision/Shape/TaperedCapsuleShape.cpp \
	Jolt/Physics/Collision/Shape/TaperedCylinderShape.cpp \
	Jolt/Physics/Collision/Shape/TriangleShape.cpp \
	Jolt/Physics/Collision/TransformedShape.cpp \
	Jolt/Physics/Constraints/ConeConstraint.cpp \
	Jolt/Physics/Constraints/Constraint.cpp \
	Jolt/Physics/Constraints/ConstraintManager.cpp \
	Jolt/Physics/Constraints/ContactConstraintManager.cpp \
	Jolt/Physics/Constraints/DistanceConstraint.cpp \
	Jolt/Physics/Constraints/FixedConstraint.cpp \
	Jolt/Physics/Constraints/GearConstraint.cpp \
	Jolt/Physics/Constraints/HingeConstraint.cpp \
	Jolt/Physics/Constraints/MotorSettings.cpp \
	Jolt/Physics/Constraints/PathConstraint.cpp \
	Jolt/Physics/Constraints/PathConstraintPath.cpp \
	Jolt/Physics/Constraints/PathConstraintPathHermite.cpp \
	Jolt/Physics/Constraints/PointConstraint.cpp \
	Jolt/Physics/Constraints/PulleyConstraint.cpp \
	Jolt/Physics/Constraints/RackAndPinionConstraint.cpp \
	Jolt/Physics/Constraints/SixDOFConstraint.cpp \
	Jolt/Physics/Constraints/SliderConstraint.cpp \
	Jolt/Physics/Constraints/SpringSettings.cpp \
	Jolt/Physics/Constraints/SwingTwistConstraint.cpp \
	Jolt/Physics/Constraints/TwoBodyConstraint.cpp \
	Jolt/Physics/DeterminismLog.cpp \
	Jolt/Physics/IslandBuilder.cpp \
	Jolt/Physics/LargeIslandSplitter.cpp \
	Jolt/Physics/PhysicsScene.cpp \
	Jolt/Physics/PhysicsSystem.cpp \
	Jolt/Physics/PhysicsUpdateContext.cpp \
	Jolt/Physics/Ragdoll/Ragdoll.cpp \
	Jolt/Physics/SoftBody/SoftBodyCreationSettings.cpp \
	Jolt/Physics/SoftBody/SoftBodyMotionProperties.cpp \
	Jolt/Physics/SoftBody/SoftBodyShape.cpp \
	Jolt/Physics/SoftBody/SoftBodySharedSettings.cpp \
	Jolt/Physics/StateRecorderImpl.cpp \
	Jolt/Physics/Vehicle/MotorcycleController.cpp \
	Jolt/Physics/Vehicle/TrackedVehicleController.cpp \
	Jolt/Physics/Vehicle/VehicleAntiRollBar.cpp \
	Jolt/Physics/Vehicle/VehicleCollisionTester.cpp \
	Jolt/Physics/Vehicle/VehicleConstraint.cpp \
	Jolt/Physics/Vehicle/VehicleController.cpp \
	Jolt/Physics/Vehicle/VehicleDifferential.cpp \
	Jolt/Physics/Vehicle/VehicleEngine.cpp \
	Jolt/Physics/Vehicle/VehicleTrack.cpp \
	Jolt/Physics/Vehicle/VehicleTransmission.cpp \
	Jolt/Physics/Vehicle/Wheel.cpp \
	Jolt/Physics/Vehicle/WheeledVehicleController.cpp \
	Jolt/RegisterTypes.cpp \
	Jolt/Skeleton/SkeletalAnimation.cpp \
	Jolt/Skeleton/Skeleton.cpp \
	Jolt/Skeleton/SkeletonMapper.cpp \
	Jolt/Skeleton/SkeletonPose.cpp \
	Jolt/TriangleSplitter/TriangleSplitter.cpp \
	Jolt/TriangleSplitter/TriangleSplitterBinning.cpp \
	Jolt/TriangleSplitter/TriangleSplitterMean.cpp 

