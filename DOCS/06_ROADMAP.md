# 06 — What is left to build

A gap register, not a wish list.  Everything here is either specified by the
master prompt (sections 0–7) and not yet built, built but below the
specification's bar, or blocked by data or by the build environment.  Each entry
says which of those it is, because the difference decides what can be done next.

Status line: the **geospatial pipeline, the world format and the CPU runtime
layer are built and verified** (D-026, `05_VALIDATION.md`).  Nothing that needs a
GPU, a device, or the Android toolchain exists yet.

## A. Specified, not built at all

| # | system | specification | what exists | buildable here? |
|---|---|---|---|---|
| A1 | **Renderer (S6)** | Vulkan 1.1: depth prepass + forward, clustered lights, WBOIT, TAA, filmic tonemap, compute post, dynamic resolution 720p–1080p | the chunk sections a renderer consumes are built and hash-verified; the three.js preview is a *viewer*, not this | no — no Vulkan headers, no ICD, no GPU |
| A2 | **Physics (S4/S5)** | Jolt: vehicle dynamics, motorcycle handling, bulk transform update on origin shift, capsule colliders on Sea Link stays | collision geometry baked per chunk (164,104 boxes), collision capsules on the hero meshes, the shift itself is validated at 0.00e+00 m | partly — Jolt is fetchable from GitHub, but it ships CMake and this environment has no cmake; it would need a hand-written build |
| A3 | **ECS + full traffic (S5)** | five vehicle archetypes with the named masses/dimensions, IDM + MOBIL for large vehicles, Boids flow field for two-wheelers, signal phases from the world | **built** (`engine/sim/`): five archetypes (D-027), IDM + MOBIL, two-wheeler Boids field at 200 ms / 20 m, signals with amber and all-red, 15 cases in `tests/test_traffic.cpp`.  Remaining: the 60-minute *validation* still runs the old single-lane model in `engine/app/sim_main.cpp` rather than this layer, and the city graph's own fleet is not yet driven end to end | partly — folding the validation onto `engine/sim/` is the next piece of work |
| A4 | **GPU weather (S3)** | 2,400 mm/yr monsoon, GPU rain particles, screen-space wetness driving roughness→0 and albedo −30 %, normal flatten | the CPU hydrology is built and verified (D-026); the wetness *response* is a material parameter with nothing driving it | no — shader work, unverifiable without a GPU |
| A5 | **Ocean (S3)** | 4–8 Gerstner waves (80–150 m swell, 0.5–2 m; monsoon 3–5 m), PBR + subsurface scattering, sun glitter, depth-buffer foam and runup | the sea/creek mesh exists (31,180 quads) | no — same reason |
| A6 | **Sky / scattering (S3)** | Rayleigh + Mie with time-of-day, humidity and pollution | the preview has a skydome approximation | no |
| A7 | **Android app** | native app, Vulkan surface, JNI, input, audio, installable package ≤160 GB | nothing | **no** — no JDK, Gradle, Android SDK or NDK; `dl.google.com`, `repo1.maven.org`, `services.gradle.org` are blocked |
| A8 | **Texture / material content** | per-surface PBR materials, baked AO, material masks at production density | 116 material definitions, vertex colours, baked AO; no textures, no atlas, no UVs in the payload | **yes** — procedural texture generation writes PNGs, no GPU needed |
| A9 | **Audio** | not specified in detail, but a client engine needs it | nothing | yes (CPU) |

## B. Built, but below the specification's bar

| # | item | specification | now | notes |
|---|---|---|---|---|
| B1 | Hero assets | Gateway ~500k tris, Taj ~800k, CSMT ~1.2M, Sea Link ~300k/pylon, Marine Drive ~50k/building | 1,750 / 4,500 / 1,762 / 6,324 / 400 — 14,736 total (D-022) | correct massings and published dimensions; the detail budgets are ~2 % |
| B2 | Building count | 200k+ | 164,057, of which 148 surveyed and the rest generated inside real parcels and slum clusters | the extract carries 16,408 building ways; the rest needs parcel data |
| B3 | Landmark recognisability at 100 m unlabelled | judged by a Mumbai resident | massings are right; facades, cornices, windows and signage are not modelled | follows B1 |
| B4 | 34,000 km of roadway | 34,000 km with 14 rail lines, 6 highway corridors | 8,189 km OSM centreline + authored corridors; 29 rail ways | the extract is a truncated 2015 sample (D-020) |

## C. Data-limited — code is ready, the inputs are not reachable

| # | item | why | unblocked by |
|---|---|---|---|
| C1 | Terrain within 1 m of Cartosat | synthesised from 43 published control points (D-006) | any real DEM: `--dem <file.geotiff\|.asc>` |
| C2 | Flood depth calibration | 32 m diffusive sheet, no gauge records (D-026) | a real DEM plus MCGM flood records |
| C3 | Real building footprints / LiDAR massing | extract is partial | MCGM building footprints or LiDAR |
| C4 | Landuse at city scale | 290 landuse polygons in the extract, 17 survived classification | full landuse layer |

## D. Verifiable only on a device

| # | criterion | why it cannot be checked here |
|---|---|---|
| D1 | 30 FPS worst case on Adreno 730 / Mali-G710 | no GPU; `mumbai_sim` measures CPU cost only (p99 0.18 ms) |
| D2 | zero rendering artifacts, TAA/WBOIT behaviour | needs the renderer of A1 |
| D3 | ≤160 GB installed | the current tier is 686 MB; the figure only becomes meaningful once B1 and A8 are at production density |
| D4 | the visual standard — motorcycle on the coastal highway at golden hour, wet asphalt reflecting the sky | the preview approximates it in a browser; the specification's answer is the renderer of A1 |

## The order I would build them in

1. **A3 — traffic completion.** Done except for its last piece: the 60-minute
   validation in `engine/app/sim_main.cpp` still runs the old single-lane IDM,
   so the layer that fifteen test cases exercise is not the layer the long run
   measures.  Folding the validation onto `engine/sim/traffic.cpp` (and giving
   the runtime the city's own nav graph rather than a hand-built corridor) is
   what closes Section 5.
2. **A8 — texture and material content.** Procedural PBR maps and an atlas move
   the visual standard more than anything else that can be done without a GPU,
   and they are what makes the 160 GB figure real rather than nominal.
3. **B1/B3 — hero detail.** Window grids, cornices, cable hardware and signage
   along the published geometry.  Large, mechanical, and reviewable in the
   preview.
4. **A2 — Jolt, vendored**, with a hand-written build file, then a headless
   physics harness (drop a motorcycle on the Sea Link, measure penetration) so
   the integration is validated the way the traffic layer is.
5. **A1 — the renderer**, written against the chunk format, with the shaders of
   A4–A6.  It cannot be run here; it can be written, and its data contract is
   already proven by the runtime reader.

A7 is out of reach in this environment at any priority, and nothing in this
repository should imply otherwise.

## Divergences from the specification, for the record

* the pipeline is C++20, not Rust (D-002);
* no third-party C++ dependencies are used, so Recast/Detour, flecs, Jolt and
  `osmpbf` are replaced by in-house equivalents where they exist (D-014);
* this environment's deliverable is the pipeline plus a browser preview rather
  than an installed application (D-001).
