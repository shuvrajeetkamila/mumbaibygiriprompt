# third_party/Jolt — vendored physics engine

Jolt Physics, by Jorrit Rouwe, MIT licensed (`LICENSE` in this directory).

**Provenance.** `jrouwe/JoltPhysics`, tag `v5.6.0` (released 2026-07-11),
GitHub source tarball
`https://codeload.github.com/jrouwe/JoltPhysics/tar.gz/refs/tags/v5.6.0`,
sha256 `6e069ee0172478cc78182047aac87e5310ba14a67a53348ae14cc37801fd3f8e`
(19,390,751 bytes).

**What is vendored.** The engine sources under upstream `Jolt/` — 4.9 MB, 429
files. Not vendored: `Samples/`, `UnitTests/`, `PerformanceTest/`, `HelloWorld/`,
`JoltViewer/`, `TestFramework/`, `Assets/`, `Docs/`, `Build/` and the upstream
`Jolt/Jolt.cmake`. Nothing outside `Jolt/` is needed to build the library, and
the samples' 34 MB of assets are not part of the physics engine.

Within `Jolt/`, these are dropped, with the reasons written out in
`third_party/jolt_sources.mk`, which is the hand-written build's file list:

  * `Compute/` and `Shaders/` — the GPU soft-body/water compute path. It needs a
    compute backend this project does not link, and nothing in the rigid-body,
    vehicle or height-field paths reaches it.
  * `Renderer/*.cpp` — debug rendering, which needs `DebugRenderer`. The headers
    are kept: `Body.cpp` includes one inside an `#ifdef` that is off.
  * `ObjectStream/*.cpp` — the binary serialiser. `JPH_OBJECT_STREAM` is
    undefined, which is what makes the rendering and hair RTTI descriptors
    unnecessary; `SerializableObject.cpp` stays because the RTTI helpers need
    `GetRTTIOfType(SerializableObject *)`.
  * `Physics/Hair/*.cpp` — five files whose only reason to exist is the compute
    path dropped above (`HairSettings.h` includes `Jolt/Compute/ComputeBuffer.h`),
    and whose registration tables name hair types that `RegisterTypes.cpp` no
    longer registers without the object stream.

**Why vendored rather than fetched.** The repository's other large inputs
(`data/raw/`) are fetched and hash-pinned because they are survey data that
changes upstream. A physics engine is a *dependency*: it participates in the
ABI of the binary that ships, its version is part of the build's contract, and a
build that could silently pick up a different Jolt (or fail on a machine with no
network) is not reproducible. Vendoring also lets the build drop the parts we do
not compile, which no upstream binary distribution would do.

**Why not CMake.** Jolt ships `Build/CMakeLists.txt` and `Jolt/Jolt.cmake`. This
environment has no cmake, and adding one would mean adding a build system to a
repository whose whole build is a hand-written `Makefile`. The transcription is
mechanical and auditable: `third_party/jolt_sources.mk` lists the source files,
and the compile flags are Jolt's own "Distribution" preset (see that file).

**Upgrading.** Re-download the tag, verify the sha256, re-run the transcription
of `Jolt/Jolt.cmake` into `third_party/jolt_sources.mk`, and re-run
`make all-checks` — the physics harness (`build/bin/mumbai_phys`) exercises the
paths this project uses: box compounds from the world's baked collision
geometry, capsule colliders on the Sea Link stays, `WheeledVehicleController`
and `MotorcycleController` vehicle dynamics, and bulk body transforms across a
floating-origin shift.
