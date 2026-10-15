# 06 — What is left to build

A gap register, not a wish list.  Everything here is either specified by the
master prompt (sections 0–7) and not yet built, built but below the
specification's bar, or blocked by data or by the build environment.  Each entry
says which of those it is, because the difference decides what can be done next.

Status line: the **geospatial pipeline, the world format, the CPU runtime layer
and the physics layer are built and verified** (D-026, D-033, `05_VALIDATION.md`).
Nothing that needs a GPU, a device, or the Android toolchain exists yet.

## A. Specified, not built at all

| # | system | specification | what exists | buildable here? |
|---|---|---|---|---|
| A1 | **Renderer (S6)** | Vulkan 1.1: depth prepass + forward, clustered lights, WBOIT, TAA, filmic tonemap, compute post, dynamic resolution 720p–1080p | the chunk sections a renderer consumes are built and hash-verified; the three.js preview is a *viewer*, not this | no — no Vulkan headers, no ICD, no GPU |
| A2 | **Physics (S4/S5)** | Jolt: vehicle dynamics, motorcycle handling, bulk transform update on origin shift, capsule colliders on Sea Link stays | **built and measured** (D-033): Jolt v5.6.0 vendored and built by a hand-written source list, `engine/physics/` (world, bake, vehicle) the only Jolt-facing code, `make physics` runs a 45-check acceptance harness — 164,594 boxes + 24 capsules + 924 heightfields baked from the pipeline's own chunks, the Sea Link's 3,987 m carriageway recovered from the collision records with a 0.00013 m record-vs-collider deviation, a motorcycle at 100 % wheels down on it, both controllers, a 22 m impact (0.0 mm penetration at rest, no tunnelling), 500 vehicles extrapolated at 8.9 ms, and the floating-origin shift at 0.25 ms mean / 0.31 ms worst over 1,454 bodies | **done** (D-033, extended by D-034): the traffic model's own vehicles are now promoted to Jolt bodies and driven from the model's own command (24 promotions / 24 demotions, 2,012 bodies back where they started, lane error 0.39 m mean, 97.3 % wheels down, 0.0420 m between driven vehicles), and road-mesh collision is on by default; the build's own stale-object defect that made `mumbai_sim` segfault (`sim_main.d` missing from `-include`) is fixed and the 60-minute run passes all 14 checks.  Remaining gap: one check, `no vehicle is driven into the city` — a **0.0370 m** motorcycle contact against a 0.020 m budget, measured to be a *city* disagreement (the road ribbon, the nav graph and the graded ground are three descriptions of one street: 84.7 % agree within 0.25 m, 4.84 % of footprints step by more than 0.35 m) |
| A3 | **ECS + full traffic (S5)** | five vehicle archetypes with the named masses/dimensions, IDM + MOBIL for large vehicles, Boids flow field for two-wheelers, signal phases from the world | **built and validated** (`engine/sim/`): five archetypes (D-027), IDM + MOBIL, two-wheeler Boids field at 200 ms / 20 m, signals with amber and all-red, 18 cases in `tests/test_traffic.cpp`, and the 60-minute validation now drives this layer over the city's own nav graph (119 chunks, ~31k nodes / 86k edges) with the specification's fleet held around the camera (D-030) | **done** — remaining gap is the ECS container itself (D-015: plain vectors, not flecs) and per-entity component stores, which is a packaging decision, not a behaviour |
| A4 | **GPU weather (S3)** | 2,400 mm/yr monsoon, GPU rain particles, screen-space wetness driving roughness→0 and albedo −30 %, normal flatten | the CPU hydrology is built and verified (D-026); the wetness *response* is a material parameter with nothing driving it | no — shader work, unverifiable without a GPU |
| A5 | **Ocean (S3)** | 4–8 Gerstner waves (80–150 m swell, 0.5–2 m; monsoon 3–5 m), PBR + subsurface scattering, sun glitter, depth-buffer foam and runup | the sea/creek mesh exists (31,180 quads) | no — same reason |
| A6 | **Sky / scattering (S3)** | Rayleigh + Mie with time-of-day, humidity and pollution | the preview has a skydome approximation | no |
| A7 | **Android app** | native app, Vulkan surface, JNI, input, audio, installable package ≤160 GB | nothing | **no** — no JDK, Gradle, Android SDK or NDK; `dl.google.com`, `repo1.maven.org`, `services.gradle.org` are blocked |
| A8 | **Texture / material content** | per-surface PBR materials, baked AO, material masks at production density | **built and shipped** (D-031): 115 materials textured into `.mumbaitex` arrays (albedo / normal / ORM, 256 px, 100.5 MB), physical tile sizes driving UVs, an 11×11 atlas wired into the browser preview through a box-projected `onBeforeCompile` patch, a runtime mmap reader, and both a C++ and an independent JS verification of mean preservation | **done for the material library** — decals, signage text and kerb/rail detail maps remain, and nothing on a GPU samples the normal/ORM arrays yet (A1) |
| A9 | **Audio** | not specified in detail, but a client engine needs it | nothing | yes (CPU) |

## B. Built, but below the specification's bar

| # | item | specification | now | notes |
|---|---|---|---|---|
| B1 | Hero assets | Gateway ~500k tris, Taj ~800k, CSMT ~1.2M, Sea Link ~300k/pylon, Marine Drive ~50k/building | 19,314 / 84,494 / 30,304 / 34,476 (2 pylons) / 41,288 — 193,132 total against an 8,100,000 budget (**2.38 %**, D-032) | published dimensions; ornament to the survey tier (coursed ashlar, voussoirs, jali, balustrades, deck furniture, Deco frontage). The budgets are hand-modelled asset budgets; closing the gap needs hand-modelled or photogrammetric assets, not more procedural ornament |
| B2 | Building count | 200k+ | 164,057, of which 148 surveyed and the rest generated inside real parcels and slum clusters | the extract carries 16,408 building ways; the rest needs parcel data |
| B3 | Landmark recognisability at 100 m unlabelled | judged by a Mumbai resident | massings, cornices, window rows with glazing bars, jali screens, voussoir arches, dome ribs, gallery balustrades, deck rails and the Art Deco row are modelled; signage text and carved relief are not | a resident check is not runnable here — the reviewer's eye is the instrument (D-032) |
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

1. ~~**A3 — traffic completion.**~~ **Closed.** The 60-minute validation now
   constructs `mumbai::sim::Traffic` over the chunks the drive streams, so the
   layer the eighteen test cases exercise is the layer the long run measures;
   the harness keeps only the population device, which is documented as a
   scenario device and not as a behaviour (D-030).  The result is in
   `05_VALIDATION.md`.
2. ~~**A8 — texture and material content.**~~ **Closed** (D-031): the library is
   synthesised, mean-preserving, shipped as mmap-able arrays and reviewed
   through the atlas in the preview.  What remains inside A8 is surface *detail*
   at production density (decals, signage, furniture maps) and GPU consumption,
   which belongs with A1.
3. ~~**B1/B3 — hero detail.**~~ **Built to the survey tier** (D-032): every hero
   asset carries a detail level; at level 3 the monuments are coursed ashlar cut
   around their openings, with voussoir arches, jali lattices, gallery
   balustrades, pinnacles, deck rails/kerbs/stiffeners and the Art Deco row.
   193,132 triangles against the specification's 8,100,000 (2.38 %), published
   per asset as `hero.<name>.triangles` beside `hero.<name>.triangleBudget`.
   What remains is not more procedural ornament but hand-modelled or
   photogrammetric assets, plus carved relief and signage text.
4. ~~**A2 — Jolt, vendored**~~ — **done** (D-033), extended by **D-034**: the
   traffic model and the bodies are one fleet, road-mesh collision is on by
   default, and `make physics` runs **55 checks with one failure** — the 0.0370 m
   motorcycle-vs-static contact, attributed by measurement to the disagreement
   between the road ribbon, the nav graph and the graded ground rather than to
   any driving or authoring defect.  The original entry read: a hand-written
   build file, then a headless physics harness (drop a motorcycle on the Sea
   Link, measure penetration) so the integration is validated the way the
   traffic layer is.
4c. ~~**The published terrain sampled a quarter of every cell**~~ — **fixed**
   (D-035): `export.cpp` read the collision heightfield with the published index
   at 4× the stride, so the viewer's terrain raster was the first 256 m of each
   1024 m cell stretched over the whole cell, and the flood QA's sea test was
   reading that against the flood model's own surface.  With the stride derived
   from the header, wet sea cells go 20,394 → **72** and `verify_export.mjs` is
   **396 checks, 0 failing**.  The same decision clamps road grading to a 3 m
   embankment (it was moving samples by up to +69.81 m / −64.60 m, lifting 1,015
   seabed samples onto land).
4d. ~~**The survey measured walls as ground**~~ — **fixed** (D-036): the ground
   search could return a wall top 29.5 m above the lane, which is where the
   "worst height disagreement 29.54 m" of D-034/D-035 came from; with a band of
   `[model − 4 m, model + 1.5 m]` the worst is **2.99 m** and no sample sits above
   it.  The same decision measured, for the first time, the clearance above lanes
   (**13,677 lane points, 1.60 %, below a car's 2.6 m**) and the systematic step
   between the two road descriptions (**7.7 % of lane points have the collision
   ground 1.5–2 m above the model's road**).
4b. **The road surface as one description.**  The next pipeline item, named by
   D-034's own numbers: `pipeline/stages/output.cpp` grades the road into the
   collision grid (`gradeRoadsIntoGrid`, n = 128) and `pipeline/stages/network.cpp`
   builds the nav graph from the OSM centrelines, while the chunk's road mesh is
   the render/collision ribbon — three surfaces for one street.  The fix is to
   make the ribbon the authority and grade *from* it, so a lane point's collision
   surface and the model's road agree to the centimetre the budget needs.
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
