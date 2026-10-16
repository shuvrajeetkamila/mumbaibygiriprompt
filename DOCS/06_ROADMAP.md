# 06 — What is left to build

A gap register, not a wish list.  Everything here is either specified by the
master prompt (sections 0–7) and not yet built, built but below the
specification's bar, or blocked by data or by the build environment.  Each entry
says which of those it is, because the difference decides what can be done next.

Shipped data (2026-10-16): **the whole city is published.**  All 924 cells of the
pipeline's own output travel as `mumbai-world-<manifest sha256:16>.tar` — an
uncompressed tar whose *members* are gzipped, so a paused download resumes at a file
boundary and the transfer is 370 MiB instead of 1.37 GiB (D-050).  Installed by the
client's own downloader over the real archive: 740 steps, 933 files, `diff -r`
against `data/processed` clean; killed at 41 %, the cold start asked for byte
157,267,968 and pulled only the remaining 230,387,712 (D-050).  A world is byte-
reproducible again — the same manifest sha256 from two consecutive builds — which is
what lets the publish step skip an upload it has already made (D-051).

Status line: the **geospatial pipeline, the world format, the CPU runtime layer
and the physics layer are built and verified** (D-026, D-033, `05_VALIDATION.md`).
The renderer, the Android client and the city download are written and verified as
far as a machine with no GPU and no Android toolchain can verify them (D-043,
D-047, D-048), and the frame budget has a policy of its own (D-052): they compile, they check, and their logic is unit-tested — and no
pixel has been rasterised here, no APK has been built here, and the one Java file
in the app has never been compiled here at all (there is no JDK).  CI builds the
APKs; a phone is the only thing that can run them.

## A. Specified, not built at all

| # | system | specification | what exists | buildable here? |
|---|---|---|---|---|
| A1 | **Renderer (S6)** | Vulkan 1.1: depth prepass + forward, clustered lights, WBOIT, TAA, filmic tonemap, compute post, dynamic resolution 720p–1080p | **written, compiled and CPU-verified** (D-043): device/swapchain/passes/pipelines/descriptors/arenas/streaming/texture upload/frame loop, nine GLSL modules, vendored Vulkan headers, `mumbai_render --check` verifying all 924 chunks through the renderer's own reader (10.3 M LOD0 triangles); **no pixel has ever been rasterised** — clustered lights, WBOIT and TAA are specified and not written (D-044 fixed a mirrored camera found on the way).  Dynamic resolution (S6's 720p–1080p window) was a mechanism with nothing driving it until D-052 gave it a policy of its own, unit-tested against five simulated devices: it settles at or under the target frame time rather than ten per cent above it, and the client applies the scale, skipping the first interval after any swapchain rebuild | **partly** — compiles and checks here; running it needs a GPU |
| A2 | **Physics (S4/S5)** | Jolt: vehicle dynamics, motorcycle handling, bulk transform update on origin shift, capsule colliders on Sea Link stays | **built and measured** (D-033): Jolt v5.6.0 vendored and built by a hand-written source list, `engine/physics/` (world, bake, vehicle) the only Jolt-facing code, `make physics` runs a 45-check acceptance harness — 164,594 boxes + 24 capsules + 924 heightfields baked from the pipeline's own chunks, the Sea Link's 3,987 m carriageway recovered from the collision records with a 0.00013 m record-vs-collider deviation, a motorcycle at 100 % wheels down on it, both controllers, a 22 m impact (0.0 mm penetration at rest, no tunnelling), 500 vehicles extrapolated at 8.9 ms, and the floating-origin shift at 0.25 ms mean / 0.31 ms worst over 1,454 bodies | **done** (D-033, extended by D-034): the traffic model's own vehicles are now promoted to Jolt bodies and driven from the model's own command (24 promotions / 24 demotions, 2,012 bodies back where they started, lane error 0.39 m mean, 97.3 % wheels down, 0.0420 m between driven vehicles), and road-mesh collision is on by default; the build's own stale-object defect that made `mumbai_sim` segfault (`sim_main.d` missing from `-include`) is fixed and the 60-minute run passes all 14 checks.  **`make physics` is 56 checks / 0 failed** (D-037) — the first green physics acceptance: the last red check was two events wearing one number, now split into *carried inside the city* (strict 0.020 m, measured 0.0000 m sustained, longest run 1 tick) and *passing through it* (Jolt's linear cast, 0.30 m, measured 0.0370 m).  Remaining accuracy gap, now named rather than tolerated: the road ribbon, the nav graph and the graded ground are three descriptions of one street (91.4 % agree within 0.25 m, worst 3.00 m) — item 4b |
| A3 | **ECS + full traffic (S5)** | five vehicle archetypes with the named masses/dimensions, IDM + MOBIL for large vehicles, Boids flow field for two-wheelers, signal phases from the world | **built and validated** (`engine/sim/`): five archetypes (D-027), IDM + MOBIL, two-wheeler Boids field at 200 ms / 20 m, signals with amber and all-red, 18 cases in `tests/test_traffic.cpp`, and the 60-minute validation now drives this layer over the city's own nav graph (119 chunks, ~31k nodes / 86k edges) with the specification's fleet held around the camera (D-030) | **done** — remaining gap is the ECS container itself (D-015: plain vectors, not flecs) and per-entity component stores, which is a packaging decision, not a behaviour |
| A4 | **GPU weather (S3)** | 2,400 mm/yr monsoon, GPU rain particles, screen-space wetness driving roughness→0 and albedo −30 %, normal flatten | the CPU hydrology is built and verified (D-026); the wetness *response* is a material parameter with nothing driving it | no — shader work, unverifiable without a GPU |
| A5 | **Ocean (S3)** | 4–8 Gerstner waves (80–150 m swell, 0.5–2 m; monsoon 3–5 m), PBR + subsurface scattering, sun glitter, depth-buffer foam and runup | the sea/creek mesh exists (31,180 quads) | no — same reason |
| A6 | **Sky / scattering (S3)** | Rayleigh + Mie with time-of-day, humidity and pollution | the preview has a skydome approximation | no |
| A7 | **Android app** | native app, Vulkan surface, JNI, input, audio, installable package ≤160 GB | **the client shows the city** (D-047, on top of the first APK of D-045): the renderer starts with no world and generates its own atlas instead of failing; an on-screen panel (5×7 font, its own pipeline) reports fps, chunks, position and errors; touch flies the camera (left half moves, right half looks, two buttons climb and descend); the APK carries a **96.3 MiB starter slice of South Bombay** (25 cells plus the albedo array) which the client installs out of its own package on first run, a few megabytes per frame; and it streams the nearest cells from the world manifest (3 km radius, 3 cells per 500 ms). CI builds the world from the raw sources, packs the slice, packs and verifies the whole city (**934 files, 370 MiB**, D-050), builds both APKs, verifies that the APK contains the engine, **11 shader modules** and the slice's chunks, and publishes the city as the `world-data` release asset — skipping the upload when the world's manifest sha256 says it is already there (D-051).  The release page's own notes are rewritten on every publish, from the build's own numbers | **partly** — everything above is measured here except the frame itself; **the city download is built and rehearsed on the real archive** (D-048, D-050): a streamed, checksum-verified tar whose gzipped members are inflated as they are extracted, resumed at a file boundary across a kill, bounded to 512 KiB a frame, gated behind a `GET CITY` button, with HTTPS in one Java class and the JNI bridge compiled against host shims; the APK is 97.7 MiB and the city adds 370 MiB of download and 1.37 GiB on disk, both inside the specification's 160 GB; walking needs a terrain query the client does not have, and the renderer still lacks clustered lights, WBOIT and TAA |
| A8 | **Texture / material content** | per-surface PBR materials, baked AO, material masks at production density | **built and shipped** (D-031): 117 materials textured into `.mumbaitex` arrays (albedo / normal / ORM, 256 px, 100.5 MB), physical tile sizes driving UVs, an 11×11 atlas wired into the browser preview through a box-projected `onBeforeCompile` patch, a runtime mmap reader, and both a C++ and an independent JS verification of mean preservation, and (D-041) a counted repair that guarantees no shipped albedo is a constant, with the test naming any flat layer instead of counting against a bound | **done for the material library** — decals, signage text and kerb/rail detail maps remain, and nothing on a GPU samples the normal/ORM arrays yet (A1) |
| A9 | **Audio** | not specified in detail, but a client engine needs it | **built and shipped** (D-042): `engine/audio` synthesises every voice from the world — exact Doppler, ISO 9613-1 air absorption, ISO 11819-1 tyre laws, chunk-derived occlusion, ground-reflection comb, rain, surf from the dispersion relation, congestion-triggered horns — and `mumbai_audio` renders the west-coast drive from the published chunks into `build/audio/mumbai_drive.wav` (**-21.7 LUFS, 0 clipped samples**) plus a 30 s excerpt the browser preview plays | **done for the mix and the weather layers** — the vehicle voices, the beds and the metering are in; footstep/footfall crowds, sirens and speech are not, and nothing on a GPU is sampled yet (A1 still has to drive it) |

## B. Built, but below the specification's bar

| # | item | specification | now | notes |
|---|---|---|---|---|
| B1 | Hero assets | Gateway ~500k tris, Taj ~800k, CSMT ~1.2M, Sea Link ~300k/pylon, Marine Drive ~50k/building | 19,314 / 84,494 / 30,304 / 34,476 (2 pylons) / 41,288 — 193,132 total against an 8,100,000 budget (**2.38 %**, D-032) | published dimensions; ornament to the survey tier (coursed ashlar, voussoirs, jali, balustrades, deck furniture, Deco frontage). The budgets are hand-modelled asset budgets; closing the gap needs hand-modelled or photogrammetric assets, not more procedural ornament |
| B2 | Building count | 200k+ | **351,051** — 350,903 generated inside real parcels and slum clusters plus 148 surveyed; 6,951,130 LOD0 triangles, 351,069 collision boxes (**target met**, D-041) | met by six per-parcel density profiles (colonial / BKC / suburban / industrial / hillside / informal) with 72,401 parcels inset clear of the carriageway and 28,709 rejected as too small; every one counted in `world_manifest.json`, so the count and D-037's standard both hold |
| B3 | Landmark recognisability at 100 m unlabelled | judged by a Mumbai resident | massings, cornices, window rows with glazing bars, jali screens, voussoir arches, dome ribs, gallery balustrades, deck rails and the Art Deco row are modelled; signage text and carved relief are not | a resident check is not runnable here — the reviewer's eye is the instrument (D-032) |
| B4 | 34,000 km of roadway | 34,000 km with 14 rail lines, 6 highway corridors | **14,147.2 km** centreline (264,882 ways, 175,631 junctions, 1,922,354 triangles) — 8,393.8 km before D-041; 29 rail ways | 42 % of the target.  The ceiling is the data, not the code: the extract is a truncated 2015 sample (D-020) carrying 8,189 km of OSM centreline, and the authored corridors add the six highway links.  Closing the rest needs an untruncated extract |

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
4e. ~~**Streets with buildings standing on them**~~ — **fixed** (D-037): 14.89 % of
   the nav graph's nodes sat inside a building box, because the ward-lattice parcel
   generator does not know where the road network runs and `emitSyntheticFabric`
   built on every parcel with area.  Parcels are now inset (1–3 m) to clear a
   carriageway's own half width and dropped only when they cannot fit: **14.89 % →
   0.01 %**, blocked-for-a-car lane points **13,005 → 59**, lane points with no
   surface in the band **70,296 → 4,980**, agreement within 0.25 m **84.8 % →
   91.4 %** and within 2 m **91.7 % → 99.4 %**.  The residue is real geometry: 72 of
   the 148 surveyed footprints are large irregular polygons whose single fitted box
   reaches a lane; they are counted (`buildings.surveyedOnRoad`) and never moved,
   and the fleet's spawn gate keeps a car out of them.
4f. ~~**The physics acceptance's last red check**~~ — **fixed** (D-037): the
   0.0370 m "vehicle driven into the city" was a single-tick contact against the
   *face* of the raised edge beside the vehicle's own road (contact point 0.0001 m
   below the surface above it; 2.2157 m of surface spread inside the body's own
   bounds), measured by a new per-tick depth: **longest run over the budget 1 tick,
   sustained depth 0.0000 m**.  The check is now the two things it was about —
   *carried* inside the city (strict, 0.020 m, measured 0.0000 m) and *passing
   through* it (Jolt's linear-cast allowance, 0.30 m, measured 0.0370 m) — and
   `make physics` is **56 checks / 0 failed**, the first green acceptance.
4g. ~~**The three descriptions of one street**~~ — **answered** (D-038): the
   signed distribution said every disagreement beyond a quarter of a metre put the
   *ground above* the model's road, and the layer attribution said 64,698 of 68,607
   of those hits were a **road mesh** — the survey's ray was stopping at the upper of
   two overlapping ribbons at a junction.  `rawGroundAt` now peels down through the
   surfaces in the band and keeps the one a wheel rests on: agreement within 0.25 m
   **91.3 % → 99.3 %**, mean signed error **+0.060 → +0.007 m**, the whole tail
   68,607 → **906 samples**.  The ribbon and the graph are now one surface by
   construction (`drapeRoadHeights`, sampled by position).
4i. ~~**The model's gap floor after a handback**~~ — **fixed** (D-040): a pair whose
   member had its position authored by physics within 2 s is excused *and counted*
   (`laneViolationsExempted`, printed beside the violations), and the excuse expires.
   Both halves — the model-owned overlap still failing, the handback overlap excused
   and then strict again 2 s later — are unit tests (59 cases / 1546 checks / 0).
4h. ~~**The ceilings above lanes**~~ — **closed** (D-040 partially, D-041 fully).  D-038
   named the population and D-040 fixed two causes: the patch was laid on the raw DEM
   instead of on the shared drape, and the drape became one function of position
   (`roadDrapeAt`).  D-041 found the last and largest one: **the patch's apex was built
   in the wrong coordinate space** (`Vec3{0, y, 0}` = the cell's south-west corner, while
   every rim vertex subtracts the cell origin), so each junction patch was a fan up to a
   kilometre long lying across the cell at the junction's height.  Measured on the same
   instrument at the standard 20 m spacing: ceilings 0.5-2.6 m above a lane
   **29,025 in 485 cells → 5,411 in 305 cells**, road/structure-mesh attribution
   **74,605 → 2,919**, lane-vs-collision agreement within 0.25 m **99.3 % → 99.6 %**.  A
   10x finer probe (2 m, 176,392 lane points) reads **99.7 %** within 0.25 m, and of the
   57,860 in the band, 134,928 are a terrain heightfield and 41,065 a road mesh: what
   remains is terrain standing above a road that runs through a cut, which is what a road
   in a cut is, not a slab over the street.
4b. ~~**The road surface as one description**~~ — **closed** (D-038, instrumented by D-040
   and D-041).  There were three descriptions of one street: `gradeRoadsIntoGrid`
   (`pipeline/stages/output.cpp`) grades the road into the collision grid, the nav graph
   is built from the OSM centrelines (`pipeline/stages/network.cpp`), and the chunk's road
   mesh is the render/collision ribbon.  D-038 made the ribbon the authority and graded
   *from* it through one shared drape field, so all three sample one function of position:
   lane sample vs collision surface agreement **91.3 % → 99.3 %** within 0.25 m, mean
   signed error **+0.060 → +0.007 m**, and **99.6 %** after D-041's junction fix.
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
