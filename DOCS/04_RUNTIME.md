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

## Nav graph (implemented)

Every chunk carries `RoadNavGraph`: nodes (position, first edge, edge count,
flags, source OSM id) and edges (target node, lane count, direction, surface,
speed limit, length, cubic control points, flags, road class), plus signal
phases. The lane vectors and signal timings the specification asks for are
carried on the edge/phase records; consumption by the traffic layer is the next
increment.

## Traffic (implemented, one increment at a time)

`mumbai_sim` implements the longitudinal layer over the real graph:

* **IDM** with per-class parameters — two-wheeler 1.9 m / 12 m/s, taxi 3.6 m,
  car 4.6 m, BEST bus 12 m / 16 t class with the longest headway;
* a **hard safety projection** at a 0.5 m gap floor, applied behind the leader
  (never by pushing a follower forward), so contact is impossible by
  construction rather than by tuning (D-015);
* an **inflow boundary** holding the peak-hour population in the busy corridor;
* **junction admission** so a vehicle only enters an edge with room, which
  produces queueing rather than teleportation.

Measured on a 60-minute run: 131,112,000 vehicle-steps, 607 vehicles live,
mean speed 5.1 km/h with 72 % at a crawl (an actual peak hour), minimum gap
0.50 m held, 545 vehicles within 500 m of the camera, 0 non-finite states,
0 out-of-envelope speeds, 0 out-of-extent positions.

**Specified, not yet built:** MOBIL lane changes for cars and buses, and the
Boids flow field for two-wheelers (200 ms updates, 20 m broadphase, strong
separation, alignment, weak cohesion). Both need the graph, streaming and
origin systems above and nothing else; they are the next increment of the
traffic layer, and the vehicle data model in the harness maps one-to-one onto
the flecs archetypes in the specification.

## Viewer (implemented)

`viewer/index.html` is a single-file three.js r169 application: terrain
(wireframe option), building instances, the street graph, the five hero meshes,
sun/time presets, layer toggles and presets 1–5. It reads
`viewer/data/mumbai.json` for offsets and `viewer/data/mumbai.bin` for the
payload, and it renders material id 0 as neutral wet concrete.

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
5. the hero detail budgets the specification names (D-022) — the current assets
   are correct massings at a fraction of the requested triangle counts.
