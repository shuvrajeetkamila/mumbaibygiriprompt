# 04 — Runtime: precision, streaming, traffic, and the device target

This is the contract between the built world and the systems that will render
it. The parts marked **implemented** are real code with tests in this
repository; the parts marked **specified** are the device-side layers the
specification describes for Android, written down here so the interfaces they
need are pinned.

## Precision policy (implemented)

Three tiers (D-004):

```
pipeline / any world query   f64                     ±50 km, 1 mm
runtime position             GlobalPosition           unbounded
                             i64 cellX, i64 cellZ, f32 localOffset ∈ [0,1024)
render space                 f32 vec3, camera-relative ±4096 m
```

`GlobalPosition::normalize()` carries the local offset into the cell indices
with `floor` in `f64`, so repeated shifts cannot drift; `toRenderSpace(camera
CellX, cameraCellZ)` computes the difference in `f64` and narrows once, so the
error is a single f32 rounding of a small number rather than an accumulation.

## Floating origin (implemented)

`FloatingOriginManager::update(cameraPosition)` returns the shift to apply;
`applyShift()` is static and exact. The runtime order per frame is:

```
physics step  →  origin.update(camera)  →  shift physics world (bulk, one transform)
             →  origin shifted in the UBO only  →  render submit
```

The UBO carries the offset, so no vertex data is touched and no per-vertex
translation is needed. `mumbai_sim` measures, for every crossing, both the cost
and the invariant: a probe vertex 250 m east / 120 m north of the camera must
hold its camera-relative position across the shift. Over a 60-minute run at
86 km/h it did so 105 times with a worst camera-relative error of
`0.00e+00 m` and a worst shift cost of `0.0003 ms`.

## Streaming (implemented)

`engine/world/chunk.h` is the runtime's whole view of the world: open a file
(mmap), verify section hashes, then read typed views in place. Residency rule:
LOD0 inside 500 m, reduced LODs to 2 km, impostors beyond (D-011).

`mumbai_sim` runs the rule with a 1536 m load radius and a 2560 m eviction
radius and re-verifies every hash on load: 256 loads, 364 unloads, 0 hash
failures, ~11 chunks / ~10 MiB resident under a camera moving at 24 m/s.

What is *not* implemented here is the job system that keeps this off the
critical path on a device. The numbers above show the work per frame is
bounded (a handful of file opens amortised over 30 frames), but the real
runtime needs the loads on a worker pool with a budget, which is why the
harness is structured as "one maintain() call per N frames" rather than as a
blocking load.

## Material textures (implemented, D-031)

`engine/world/texture.{h,cpp}` is the runtime reader for `.mumbaitex`: mmap,
validate (magic, version, map type, layer table, mip arithmetic, per-layer spans,
8-byte alignment) and, when the caller asks for it, re-hash every layer. The mip
chains are baked offline, so `open()` allocates nothing and the GPU gets a
pointer into the mapping per layer; `mip(i, level)` and `layerMean(i, rgb,
linear)` are views for the streaming and QA paths.

The renderer's contract, once A1 exists: one 4-channel sRGB base-colour array,
one 3-channel normal array and one 3-channel ORM array; a `MaterialDef`'s
`textureSetIndex` is the layer (‑1 untextured); UVs are metres divided by the
material's `tileMetres`, so texel density is physical; and the surface multiplies
the sampled tile by `1/tileMean` so the texture adds detail without changing the
calibration the rest of the shading was verified with.

## Nav graph (implemented)

Every chunk carries `RoadNavGraph`: nodes (position, first edge, edge count,
flags, source OSM id) and edges (target node, lane count, direction, surface,
speed limit, length, cubic control points, flags, road class), plus signal
phases. The lane vectors and signal timings the specification asks for are
carried on the edge/phase records; consumption by the traffic layer is the next
increment.

## Traffic (implemented)

`engine/sim/` is the shipped layer, and `mumbai_sim` drives it — there is no
second model in the harness (D-030). One tick is:

1. **`collectActiveEdges()`** — the ~572 edges that hold a vehicle, against the
   ~86,000 the loaded corridor carries. Every per-edge sweep below walks this
   list; only `audit()` and `metrics()` walk the graph, and they run every 10
   seconds from the harness, outside the frame budget.
2. **`leadersAndSignals()`** — one obstruction per vehicle: the rear face of the
   nearest body that overlaps its path laterally (a two-wheeler scans the whole
   carriageway, which is what lets it filter past a bus), or the stop line when
   the aspect is red or amber.
3. **`longitudinal()`** — IDM with per-class parameters from `archetypeSpec`.
4. **`laneChanges()`** — MOBIL (politeness 0.2, safe brake −3.0 m/s², threshold
   0.2 m/s²) on a four-phase stagger, for everything that holds a lane.
5. **`twoWheelerField()`** — Boids every 200 ms on a 20 m broadphase: separation
   strong (gated on longitudinal overlap, so it moves bodies apart rather than
   queuing them), alignment present, cohesion ≈ 0.
6. **`lateralDynamics()`** — one integrator for everybody's line, clamped by the
   archetype's rollover limit (an auto-rickshaw tips at 0.36 g).
7. **`faresAndQueues()`** — hail timers and the kerb pull-in.
8. **`corrections()`** — the exact pass that holds the 0.5 m floor, front of the
   queue to back, after everything that can move a body (D-030).
9. **`advanceEdges()`** — leave the edge, choose an arm (never the reversal of
   the one just travelled while another exists), enter only if
   `laneEntryClear`, otherwise hold at the junction; then the population device.
10. **`maintainDensePopulation()`** — the harness's scenario device (D-030).

Measured on the 60-minute validation run (D-030, `DOCS/05_VALIDATION.md`),
over the real 119-chunk city graph: 123,552,000 vehicle-steps, 572 vehicles
live, mean 28.2 km/h, minimum gap 2.55 m, 0 penetrations and 0 lane-discipline
events in 216,000 audits, median 451 vehicles within 500 m of the camera (peak
547), solver + origin p99 4.38 ms with chunk I/O timed separately.

## Viewer (implemented)

`viewer/index.html` is a single-file three.js r169 application: terrain
(wireframe option), building instances, the street graph, the five hero meshes,
sun/time presets, layer toggles and presets 1–5. It reads
`viewer/data/mumbai.json` for offsets and `viewer/data/mumbai.bin` for the
payload, and it renders material id 0 as neutral wet concrete.  Since D-031 it
also fetches the base-colour atlas and applies it with a box-projected patch on
the standard material: the projection plane is chosen in the vertex shader from
the object-space normal, the atlas tile and the tile's mean albedo arrive as
per-vertex (and per-instance) attributes, and the sampled tile is divided by that
mean, so a tinted facade keeps its tint and gains grain.  `make all-checks` runs
`tests/verify_viewer_shaders.mjs`, which applies the same patch to the vendored
three.js and checks it still lands.

Serve it with `make serve` (an HTTP server is required: a `file://` page cannot
fetch the payload, and the sandboxed preview iframe cannot either).

## The device target (specified)

The specification's Android/Vulkan target maps onto the interfaces above as
follows. None of this runs in the build environment (D-001).

| system | specification | interface it consumes | status here |
|---|---|---|---|
| Renderer | Vulkan 1.1, depth prepass + forward, clustered lights, WBOIT, TAA, filmic tonemap, compute post | `TerrainMesh`, `BuildingMesh`, `WaterMesh`, `MaterialTable` per chunk | specified; the chunk data it needs is built and verified |
| LOD / impostors | <100 m hero, 100–500 m simplified, 500 m–2 km blockout, >2 km impostors | LOD0/1/2 meshes + `BuildingImpostor` | LOD chain built; impostor quads emitted |
| Weather | monsoon 2,400 mm/yr, >100 mm/h, GPU rain, screen-space wetness driving roughness→0 and albedo −30 % | material wetness response; terrain drainage basins | water material and 19 drainage basins exist; the weather system is not built |
| Ocean | 4–8 Gerstner waves (80–150 m swell, 0.5–2 m amplitude; monsoon 3–5 m), PBR + SSS, sun glitter, shoreline foam from depth-buffer intersection | `WaterMesh` displacement grid at sea level | mesh exists (31,180 quads); shader not built |
| Physics | Jolt, bulk transform on origin shift, baked collision geometry | `TerrainCollision`, `BuildingCollision` (OBB list) | collision geometry baked per chunk |
| ECS | flecs, vehicle archetypes, lane/signal behaviour | `RoadNavGraph` (+ signals) | graph built; ECS not built |
| Dynamic resolution | 720p–1080p, 30 FPS worst case on Adreno 730 / Mali-G710 class | — | not measurable here |

## What a device build still needs

1. a Vulkan renderer consuming the chunk sections in place;
2. Jolt wired to `BuildingCollision`/`TerrainCollision`;
3. the weather/ocean shaders and the flood model (with the drainage-sentinel
   filter noted in `02_DECISIONS.md`);
4. MOBIL + Boids on top of the existing nav graph;
5. the hero detail budgets the specification names (D-032; 193,132 triangles of a
   8,100,000 budget) — the assets
   are correct massings at a fraction of the requested triangle counts.
