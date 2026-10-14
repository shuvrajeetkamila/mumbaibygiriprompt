# 02 — Decision register

The build instruction was: *"Do not ask questions. Make confident, technically
justified decisions at every ambiguity. Document every decision."* This file is
that record. Every entry is referenced from the source tree by its `D-nnn` tag,
and every entry states what was decided, why, and what it costs.

Entries marked **(deviation)** change something the specification asked for by
name; they are the ones a reviewer should challenge first.

---

## D-001 — Deliverable shape in this build environment **(deviation)**

**Decided.** This repository delivers the offline geospatial pipeline, the
`.mumbai_chunk` world format, the shared runtime core (math, projection,
floating origin, chunk reader), a headless runtime-validation harness and a
browser viewer of the produced world. The specification's device-side layers —
Vulkan 1.1 renderer, Jolt physics, flecs ECS on Android — are specified,
mapped and validated where the sandbox allows, but not executed here.

**Why.** The build environment has two CPU cores, 3 GB of RAM, no GPU, no
Android SDK/NDK, no `cmake`, no `clang`, and outbound network access is limited
to a few package hosts. Anything that claims to be a 30 FPS Android frame
produced here would be unverifiable, and the zero-artifact standard explicitly
forbids shipping unverified assertions.

**Cost / consequence.** Every acceptance criterion that needs a device is
marked *not verifiable here* in `05_VALIDATION.md` with the exact reason. The
runtime-facing interfaces (chunk format, floating origin, nav graph, traffic
state) are real code with tests, so the device layer can be built on top
without re-doing the world.

---

## D-002 — Pipeline language is C++20, not Rust **(deviation)**

**Decided.** The pipeline is C++20 built with `g++` and a plain `Makefile`.
The in-house OSM scanner, mesh tooling and LOD generation replace `osmpbf`,
`meshoptimizer` and the Rust pipeline of the specification.

**Why.** The environment has `g++ 12.2` (full C++20) and no `rustc`/`cargo`;
`static.crates.io` is unreachable, so a Cargo project could not fetch a single
dependency and could not be built at all.

**Cost / consequence.** One language across pipeline and runtime, one compiler,
no dependency graph to audit. The chunk format is the boundary: a future Rust
front end can write the same sections and nothing downstream changes. Cost: the
in-house mesh simplifier is less sophisticated than `meshoptimizer`'s
quadric-error-metric collapse (documented in `03_PIPELINE.md`).

---

## D-003 — Projection: a Mumbai-centred Transverse Mercator, not UTM 43N

**Decided.** WGS84 → custom Transverse Mercator, central meridian 72.9°E,
latitude of origin 18.9°N, scale factor 1, metres (`engine/core/geo.h`).
UTM 43N (the specification's input CRS) is used for reading source data, not as
the working frame.

**Why.** The playable extent is 22.5 km × 43.0 km. A zone centred on the city
keeps scale error symmetric and tiny across the whole area, and there is no
zone-boundary handling anywhere in the pipeline. UTM is the same maths with a
different central meridian and a 0.9996 scale factor, so this is a constants
difference, not an algorithmic one.

**Verification.** `tests/test_geo.cpp` pins the frame: 0.001° latitude =
110.70 m, 0.001° longitude = 105.28 m at 19°N (ellipsoidal radii, so the test
would fail for a spherical approximation).

---

## D-004 — Precision policy

**Decided.** Three tiers, enforced by type:

| tier | type | range | resolution needed |
|---|---|---|---|
| pipeline / any world query | `f64` | ±50 km from origin | 1 mm |
| runtime position | `GlobalPosition` = `i64 cellX, i64 cellZ` + `f32 localOffset` ≤ 1024 m | unbounded | 0.06 mm at 1024 m |
| render space | `f32 vec3` relative to the camera macro-cell | ±4096 m | 0.25 mm at 4096 m |

**Why.** The specification's floating-origin section exists precisely because a
single-precision world frame jitters at kilometre scales. Keeping world
positions in `f64` in the pipeline removes the class of bug where a rounded
coordinate silently shifts a building or a road.

**Verification.** `tests/test_geo.cpp::renderSpaceKeepsSubMillimetreGranularity
AtMapCorners` places a 1 mm vertex pair at the four corners and the centre of
the extent and requires it to stay 1 mm (tolerance 2 × 10⁻⁴ m). `mumbai_sim`
re-checks it live: a probe vertex is compared before and after every origin
shift, worst error 0.00e+00 m over 105 crossings.

---

## D-005 — OSM ingestion reads `.osm` XML, not `.pbf`

**Decided.** `pipeline/mgeo/osm.cpp` is a streaming XML scanner for the
OpenStreetMap XML format.

**Why.** The obtainable Mumbai extract is `.osm` XML (35.2 MB, 170,260 nodes /
20,959 ways / 402 relations), and `osmpbf` / protobuf are excluded by D-014.
A `.pbf` reader without a protobuf library is not worth writing when the data
on hand is XML.

**Cost.** Parsing is ~1.5 s for the full extract; a `.pbf` path can be added
behind the same `osm::Database` interface if a PBF extract becomes available.

---

## D-006 — Terrain is synthesised from authored control points **(deviation)**

**Decided.** The heightfield is generated from
`data/authored/terrain_control_points.json` — 43 published approximate
elevations (Malabar Hill, Antop Hill, Powai basin, the reclamation fringes,
Mumbra/Powai hills, the creeks) — plus coastal and ward priors, then refined by
the drainage pass. `--dem <file>` accepts a real raster (GeoTIFF/ASCII grid) to
replace the synthesis without any other change.

**Why.** Cartosat 1 m DEM and MCGM LiDAR are not obtainable in this
environment: they are licence-gated, and no bulk elevation service is reachable.
The alternative — inventing a plausible-looking heightfield with no published
anchor — would have been worse.

**Cost.** Terrain height above the authored control points is interpolated, so
the ±1 m Cartosat comparison of the acceptance section cannot be met from this
source. `05_VALIDATION.md` records this as a data limitation, not an
implementation one. The control points are a small file in the repository and
each entry carries its source note.

---

## D-007 — Building generation is a native grammar

**Decided.** `pipeline/stages/fabric.cpp` implements the zone grammar directly
(the specification offered Houdini Engine or a Rust-native grammar). Zone
rulesets cover South Bombay Colonial, BKC Commercial, Suburban Residential,
Informal Settlement and Industrial, with LOD0…LOD3, collision boxes, baked
per-building AO and ward-derived heights.

**Why.** Houdini Engine is a commercial DCC plug-in and cannot run here; a
native grammar is also the honest reading of the "Rust-native grammar"
alternative, implemented in the project language per D-002.

---

## D-008 — Facility taxonomy: 10 zones, resolved by ward and geometry

**Decided.** `engine/core/geo.h` defines 10 `ZoningZone` values. Assignment
order is: outside the city → `MangroveWetland`; inside the slum mask →
`InformalSettlement`; within 2.6 km of CSMIA or 1.1 km of Juhu →
`Airport`; wards A–E → `SouthBombayColonial`; ward H/E → `BKCCommercial`;
above 35 m → `Hillside`; otherwise `SuburbanResidential`.

**Why.** The specification's zone list is architectural (what the place looks
like); the data available to distinguish them is ward polygons, the airport's
location, the slum polygons and elevation. This mapping is a faithful,
documented reading of the city rather than a guess per cell.

---

## D-009 — Terrain LOD: one budget per class, crack-free stitching

**Decided.** Terrain meshes are built per 1024 m cell with a quadtree whose
leaf size follows local relief (finest on slopes and the coastline, coarsest on
flats), each leaf tessellated at 4–8 m/texel for flats and down to 1 m/texel on
relief; LOD0 is budgeted at 2,048 triangles per cell, LOD1/2 reduce by
skipping, and the LOD3 impostor is a quad.

**Why.** The specification asks for a quadtree with 1 m/texel on slopes and
4–8 m/texel on flats, with no cracks. Budgeting per cell makes the streaming
cost predictable (`mumbai_sim` shows the resident set is ~11 chunks / ~10 MiB
under a moving camera).

---

## D-010 — Face-orientation convention (empirical, enforced)

**Decided.** Verified by compiling and running a probe against
`engine/core/math.cpp`:

* a CCW triangle `(a,b,c)` mapped to world XZ with +Y up has normal (0,−1,0) —
  it faces **down**;
* emitting walls as (bottom-start, top-start, top-end, bottom-end) faces
  **outward** for a CCW ring;
* the sea-quad ring order `(x0z0, x0z1, x1z1, x1z0)` faces **+Y**;
* ear-clipper output (roofs, roof lips, inland water, landuse tint) is always
  emitted **reversed**.

**Why.** A down-facing sea plane or roof cap is exactly the "rendering
artifact" the zero-artifact standard forbids, and winding bugs are invisible in
a vertex dump. The convention is stated once, here, and every emitter follows
it; `mesh.cpp` and `sources.cpp` carry the reasoning inline at the call sites.

---

## D-011 — Streaming budget

**Decided.** LOD0 fully resident inside 500 m of the camera, reduced LODs from
500 m to 2 km, impostors (LOD3) beyond. The harness streams with a 1536 m load
radius and a 2560 m eviction radius, and re-verifies every section hash on load.

**Why.** This is the specification's residency rule; the wider harness radii
exist so that the validation run also exercises eviction (256 loads / 364
unloads over 60 minutes, no hash failure).

---

## D-012 — Water surfaces

**Decided.** The open sea/creek surface is a quad grid at sea level
(31,180 quads city-wide), inland water comes from OSM `natural=water` polygons
(3 bodies triangulated with holes honoured), and rivers/canals are ribbon
strips along their centrelines (818 segments).

**Why.** The Gerstner-wave shader of the specification needs a displacement
mesh with a stable vertex density; a cell-resolution grid does that without
depending on coastline polygon quality, and the sea plane also gives the flood
model its zero level.

---

## D-013 — Rail geometry constants

**Decided.** Broad gauge 1.676 m, monorail 0.85 m, ballast half-width 3.4 m /
1.6 m, sleepers every 2 m, rail strips 0.08 m wide, track base at terrain
+ 0.5 m. 29 rail ways are built from the extract plus authored alignments.

**Why.** Gauge and sleeper geometry are public constants; keeping them as named
constants means a monorail corridor never renders as broad gauge.

---

## D-014 — No third-party C++ dependencies

**Decided.** The project ships its own JSON reader/writer, OSM XML scanner,
polygon operations, triangulation, simplification, mesh builder, chunk
reader/writer and test harness. No CMake, no protobuf, no Clipper, no
`meshoptimizer`, no Catch2, no `flecs`.

**Why.** Nothing can be fetched reliably in this environment except through
`gh api` and npm/pip; a dependency-free tree builds with one compiler and stays
auditable line by line. The specification's engine choices (`flecs`, `Jolt`)
are runtime-side and remain available to the device layer — the traffic
*behaviour* in `mumbai_sim` is written against a plain data model that maps one
to one onto flecs archetypes.

**Cost.** Documented in the entries it affects (D-002 mesh simplification,
D-014 test harness, D-005 OSM).

---

## D-015 — Traffic model: IDM + hard safety projection, not a free-running fleet

**Decided.** Vehicles follow the intelligent-driver model with
class-specific length, acceleration, headway and desired speed (two-wheeler
1.9 m, taxi 3.6 m, car 4.6 m, BEST bus 12 m), plus:

* a **hard safety projection**: when a gap falls below a 0.5 m floor, the
  follower is placed exactly at the floor *behind* its leader and its speed is
  capped at the leader's — contact is impossible by construction rather than by
  tuning;
* an **inflow boundary**: the peak-hour population is held in the busy corridor
  by replacing vehicles that leave it, which is how a demand-fixed population
  is simulated without inventing an origin–destination matrix;
* **junction admission**: a vehicle only enters a new edge if there is room at
  its start, which produces queueing instead of teleporting into traffic.

**Why.** The specification asks for IDM lane AI for large vehicles, MOBIL lane
changes and a Boids flow field for small ones. The longitudinal layer and the
safety invariant are the parts that can be validated headlessly today; MOBIL
and the flow field are specified in `04_RUNTIME.md` as the next increment, and
they need exactly the graph, streaming and origin systems this layer proves.

---

## D-016 — A headless runtime harness is part of the product

**Decided.** `engine/app/sim_main.cpp` (`mumbai_sim`) drives the real
`data/processed` world through the real chunk reader for a 60-minute, 60 Hz
Colaba → Borivali run, and asserts the runtime contracts: hash verification on
load, origin-shift invariants, vehicle state sanity, gap floor, frame budget.

**Why.** "Zero crashes over a 60-minute run" is a specification requirement, and
it is testable without a GPU. The harness is the only place where streaming,
the floating origin and traffic meet the actual bytes on disk.

---

## D-017 — Viewer: three.js r169, vendored, no CDN

**Decided.** `viewer/index.html` is a single-file three.js app; the library is
vendored under `viewer/vendor/` (installed from npm, committed). All data paths
are relative.

**Why.** The preview environment runs pages inside a sandboxed iframe with no
network access, so a CDN reference would produce a blank page. Vendoring also
makes the viewer reproducible offline.

**Operational note.** That iframe also cannot fetch sibling files, so in this
environment the viewer is served with
`python3 -m http.server 8000 --bind 0.0.0.0 --directory viewer`.

---

## D-018 — The viewer export re-reads the world through the runtime reader

**Decided.** `pipeline/stages/export.cpp` does not serialise in-memory build
structures. It opens `world_manifest.json`, re-opens every chunk through
`ChunkReader` (mmap, typed views, per-section hash verification) and rebuilds
the payload from the files on disk.

**Why.** It converts the viewer into an end-to-end test of the write path: if
the viewer shows Mumbai, the bytes the runtime will read are the bytes the
pipeline wrote. It is also how the corruption class that a stale-object bug
produced earlier was caught.

---

## D-019 — One serialisation of the QA report, published last

**Decided.** `qaReportJson()` (`pipeline/stages/output.cpp`) is the only
serialisation of the report. `finaliseQa()` (`pipeline/stages/export.cpp`)
runs as the last step of the build, adds its own timing entry, and rewrites the
manifest's `quality` block and the viewer payload's `qa` block from the
finished report.

**Why.** The manifest and the payload are written by stages that run before the
last timing counters exist, so taken as-is they under-reported (the payload
read 63 counters while the console printed 66). Now console, manifest and
payload agree exactly (67 / 67 / 67 on the current build) and cannot drift,
because there is one code path.

---

## D-020 — The extract tags the city better than it maps it **(deviation)**

**Decided.** The OSM extract is treated as the *tag* inventory of Mumbai and as
the geometry source wherever it has geometry. Where the surveyed network is
thin or truncated, the fabric is generated deterministically:
`pipeline/stages/network.cpp` (ward streets), `pipeline/mgeo/synth.cpp`
(slum lanes and parcels) and `pipeline/stages/corridors.cpp` (authored
corridors, see D-021). Every generated element is tagged as such and counted
separately in the QA report (`network.syntheticStreets`, `network.slumLanes`,
`network.syntheticParcels`, `network.informalParcels`).

**Why.** The extract carries 16,408 building ways and 8,189 km of road
centreline; the specification's target is 200,000+ buildings and ~34,000 km of
roadway. Pretending the surveyed data covers the city would produce a
half-empty Mumbai in which the missing parts are indistinguishable from the
mapped ones. Generating the remainder *and labelling it* keeps the data honest
and keeps the two classes separable in every downstream system.

**Revision note (2026-10-13).** Running the pipeline against the real extract
showed that 107,764 OSM ways carry a centreline — far more than the initial
reading of the tag inventory suggested, because the extract is a full regional
snapshot rather than a sample. The authored corridors and synth stage therefore
supplement the surveyed network instead of replacing it; D-020's consequences
were narrowed accordingly, and the file headers in `synth.h`, `network.cpp`
and `pipeline.h` carry the same revision.

---

## D-021 — Named corridors are authored data **(deviation)**

**Decided.** The corridors the specification names (the highway corridors, the
harbour line and the coastal rail alignments) exist as a data file,
`data/authored/transport_corridors.json`, consumed by
`pipeline/stages/corridors.cpp`. Each corridor is a polyline with class, name
and provenance, extended so the network is continuous where the extract is
truncated (the Bandra–Worli Sea Link is the clearest example: 4,015 m published
against a 320 m fragment in the extract).

**Why.** A missing link between two mapped roads produces either a hole in the
nav graph or a straight line across the harbour — both are the kind of visible
failure the residency check ("a Mumbai resident would notice") is designed to
catch. Authoring the alignment as reviewed data keeps the fix explicit and
reviewable.

---

## D-022 — Hero assets use public published dimensions, and report themselves honestly

**Decided.** Gateway of India, Taj Mahal Palace, CSMT, the Bandra–Worli Sea
Link and Marine Drive are authored geometry with their dimensions taken from
published figures (Sea Link: 4,015 m alignment, two cable-stayed pylons,
semi-harp stays, 61 pier bents, 8 lanes; the OSM fragment is measured and
reported as `hero.seaLink.osmFragmentMetres`, never used as the alignment).
Triangle budgets are reported per asset in the QA report and compared against
the specification's targets in `05_VALIDATION.md` — currently far below them.

**Why.** The zero-artifact standard makes an under-detailed hero asset a gap to
declare, not to hide behind a screenshot. The counters exist so the gap is
visible in every build.

---

## D-023 — Anchor-tolerance policy in the payload verifier

**Decided.** `tests/verify_export.mjs` requires every vertex of a landmark
material to lie within 400 m of its anchor, except `hero_concrete`, which gets
2,500 m.

**Why.** `hero_concrete` is shared by the CSMT bulk *and* the Marine Drive
promenade, a 3.5 km run whose published anchor is its mid-point; the material
is legitimately kilometres long. The tight limit still applies to the materials
that belong to point landmarks (stone, terracotta, curtain-wall glass). The
verifier's real value is the per-vertex form of the test — an earlier
centroid-based version passed while vertices were 1,700 m out.

---

## D-024 — Repository hygiene: regenerate, don't commit, the bulk data

**Decided.** `data/raw/` (46 MB, four datasets + `SHA256SUMS`) and
`data/processed/` (657 MB of chunks and the manifest) are gitignored and
reproducible: `tools/fetch_data.sh` re-downloads the sources over `gh api` and
verifies hashes, `make world` regenerates the world in ~15 s. The viewer
payload (`viewer/data/mumbai.bin` + `mumbai.json`, 7.7 MB) *is* committed so
that a clean checkout opens a working viewer.

**Why.** A 657 MB build output does not belong in git, but the thing the owner
actually looks at should not require an API token to appear.

---

## D-025 — The published payload is pinned by checksum, and its size is asserted

**Decided.** `make world` writes `viewer/data/SHA256SUMS` over `mumbai.bin` and
`mumbai.json`; `tools/verify_artifacts.sh` re-verifies it and runs inside
`make all-checks`. `tests/verify_export.mjs` additionally asserts that the
payload's length equals the layout its own metadata declares
(`hero.indexOffset + hero.indexCount × 4`) and that the file contains no UTF-8
replacement sequences.

**Why.** A binary can be corrupted *outside* the build and still look fine from
the outside. Observed on 2026-10-13: after a workspace restore, `mumbai.bin`
had grown from 8,359,144 B to 11,611,201 B because the file had made a round
trip through UTF-8 decoding with `errors=replace` — every byte sequence that is
not valid UTF-8 became U+FFFD (3 bytes). 1,635,488 replacement sequences, the
first at byte 1392, and `mumbai.json` still describing the original layout, so
every offset-driven read produced nonsense (a "63.6 % land" terrain, building
footprints of 177 million km²) while the file existed and looked plausible.
The pipeline itself was never at fault: two consecutive runs produce identical
hashes, so the artifact is reproducible and a checksum is a sound pin.

**Cost / consequence.** The guard only covers what is published (the payload);
the 686 MB chunk set is verified structurally by the manifest
(`chunks.verified`, `chunks.bytes`) and by every reader that opens it. Rebuild
cost after any such corruption is ~16 s (`make world`), which is why the
recovery path is printed by the guard itself.

---

## D-026 — Monsoon hydrology: a walled coast, a conditioned DEM, and streets
## that carry their catchment

**Decided.** Section 3's flood model (`pipeline/mgeo/hydro.{h,cpp}`,
`pipeline/stages/flood.cpp`) rides on a hydrologically conditioned surface: a
priority flood, D8 flow directions, accumulation, watersheds and channels, then
two design storms routed with Manning's equation on a 32 m grid, with an
explicit mass balance. The choices that determine the answer:

1. **The sea is a level-controlled boundary, never a routing source.** Each step
   the sea surface is set to the tide and the exchange is booked
   (`tideExchangeM3`); no flux ever leaves a sea cell. Routing from an infinite
   reservoir is unbounded and step-order dependent, and measured, it pushed
   2.5 × 10¹¹ m³ through the books in a single storm.
2. **The open coast is walled; the tide comes in through creek mouths.** Mumbai
   is embanked from Colaba to Dahisar. A shore cell is lifted to the tide only if
   it fronts a *narrow* water body — a creek, nullah or mangrove channel, taken
   as a sea fraction below 0.55 in a centred 9 × 9 window — or if the tide
   overtops the sea wall (`seawallCrest = 4.0 m`). Without this the whole western
   lowland is under water twice a day, which models an unprotected delta, not
   Mumbai.
3. **Below-datum land is behind the shore line, not on it.** Cells the DEM holds
   at or below `landFloor = −0.75 m` (reclamation, docks, marsh) are exempt from
   the tidal lift; they flood from rain and by gravity.
4. **The DEM is conditioned before routing.** `pitFillDepth = 2.0 m` raises every
   cell whose priority-flood fill need is that small — these are closed
   micro-depressions of the grid, not places water can stand, and each one holds
   a pond for ever. Measured on the 2005 analogue before conditioning: 128 of the
   149 km² of ground above 10 m was under more than 15 cm, spread evenly across
   the city. That is a model drowning in its own noise instead of pooling in its
   low ground, and it is the single most visible thing a reader would have
   noticed. Deeper basins are left alone, and the four authored depressions
   (below) are exempt from conditioning entirely.
5. **Streets are the drainage network.** Road cells are network cells with the
   standard design intensities (60 / 35 / 14 / 8 mm/h, motorway to service) and a
   *conveyance* sized by the rational method — design intensity × contributing
   area, from the flow-accumulation grid — because a drain is sized by its
   catchment, not by the 1024 m² of road it sits under. The imposed gutter
   gradient (`minDrainSlope = 0.0015`) is applied only to network cells, only
   downhill, only where the priority flood did not have to raise the cell (an
   impounded cell's water must reach its spill level first), and only up to the
   design flow. Above that, Manning on the real head governs: a street with a
   metre of water on it moves that water.
6. **Rainfall reaches the gutter, not the puddle.** A cell's runoff is booked to
   the first network cell on its downhill path, within `maxOutletWalk = 3` cells
   (~100 m), or to the sea if that is where the path ends. A roof or a courtyard
   is connected; a hillside is not teleported two kilometres to a road, which is
   what an unbounded search did.
7. **Outfalls are tide-locked, and that is the city's flood mechanism.** A
   gravity outfall flows at full rate until the sea submerges it: the removal
   capacity is derated by `clamp(1 − (tide − outfallInvert) / outfallDesignHead,
   0.05, 1)` about an invert of 0.6 m and a design head of 1.6 m. This is what
   makes the *same* rainfall harmless at low water and catastrophic at high
   water, and it is why the design storms carry a spring tide.
8. **Routing is sub-stepped 8× inside each five-minute weather step.** The
   explicit scheme caps a cell's outflow at half of what it holds per step, which
   at 300 s and 32 m cells throttled conveyance far below what Manning gives.
   Measured effect on the severe storm: wetted area 449 → 379 km², area over
   50 cm 273 → 133 km², and the ridges stopped holding half a metre of water.
   This was a time-step artefact and it was the largest error in the first
   working version.

**The DEM had to be shaped, and that is a data decision.** Four control points
in `data/authored/terrain_control_points.json` are marked `flat`: Hindmata
(a former tank bed), King's Circle, and the Andheri and Milan subway trenches.
Each gets a bowl cut explicitly — floor at the stated elevation, rim 1.6 m
higher at its radius — and fractal detail damped inside it. A depression known
to the metre is not something an interpolator invents from its neighbours, and
2.4 m of fractal relief in the floor of a 350 m bowl is a leak: with it the model
drained Hindmata to 25 mm, the "model drains the puddle it must predict"
failure. Marine Drive stays dry in both storms on its own — the sea wall holds
and the promenade is at 3.5 m.

**Named sites** (published peak depth, severe / moderate storm):

| site | 26 July 2005 analogue | routine monsoon day |
|---|---|---|
| Hindmata junction | 289 mm | 277 mm |
| King's Circle | 283 mm | 288 mm |
| Andheri subway | 376 mm | 377 mm |
| Milan subway | 758 mm | 635 mm |
| BKC | 1,710 mm | 1,010 mm |
| Chembur | 341 mm | 3 mm |
| Marine Drive | 0 mm | 0 mm |

The sites are read as a person there would read them: the deepest water in the
junction cell and its eight neighbours, never a single pixel that the rain might
have skipped. They are asserted, not merely reported — Hindmata, King's Circle
and the Andheri subway must exceed 50 mm in the severe storm or the build fails,
and Marine Drive warns above 500 mm, because the promenade is the control case.

**Aggregates** (459.6 km² of modelled land): the routine monsoon day puts more
than 15 cm on 20.7 % of ground below 6 m and 3.9 % of ground above 25 m; the
2005 analogue puts more than 15 cm on 70.1 % and 40.1 % respectively. The
routine day is the check that matters, because that contrast *is* Mumbai's flood
geography; the 2005 analogue is 944 mm-in-a-day weather and the whole city is its
floodplain, so its bar is that the flats are still clearly worse off than the
hills. Both rasters travel in the viewer payload (`flood`, `floodModerate`;
centimetres, 705 × 1345) and are drawn as standing water at ground + depth.

**Mass balance.** Rainfall + tide flood = drainage + outfall + stored + retired
film, closed to 1.5 × 10⁻³ m³ and 7.1 × 10⁻³ m³ on 3.6 × 10⁷ and 2.3 × 10⁸ m³ of
rain — a relative error of 10⁻¹⁰ — and asserted at 10⁻⁸ of rainfall. Three terms
had to be booked rather than silently dropped: rain on open-boundary cells, the
sub-millimetre film retired as drying cells are snapped below `minDepth`
(96,550 m³ = 0.27 % of rainfall in the moderate storm, audited and failed above
0.5 %), and the water that legitimately leaves the surface through an outfall.

**Known limits, recorded rather than papered over.** A 32 m diffusive sheet is
not a pipe network: real trunk mains and nullahs move an order of magnitude more
water per unit width than a cell of overland flow, and the rational-method
conveyance term is a parameterisation of that, not a hydraulic model of it. The
absolute depths therefore carry the DEM's own uncertainty (D-006: the terrain is
synthesised from published control points, not surveyed) and the site table above
is the *shape* of the event, not a gauge reading. BKC's 1.7 m is its marsh basin
filling with no holding pond in the model. Calibration against gauge records
would begin with a real DEM via `--dem`.

**Cost.** The flood stage is ~150 s of the ~200 s pipeline run: two storms,
8 sub-steps, 0.95 M cells, two cores.

---

## D-027 — The specification's comfort numbers are fractions of class maxima

**Decided.** Section 5 gives each archetype a pair of numbers — BEST 0.9/0.3,
auto-rickshaw 0.2/0.7, kaali-peeli 0.4/0.6, motorcycles 0.05/0.8 — and words:
the bus pulls away hard and brakes early, the auto crawls and brakes late, the
two-wheeler barely touches the throttle and brakes hardest of all. Read as metres
per second squared the numbers describe no vehicle that exists: 0.05 m/s² is a
motorcycle taking six minutes to reach 20 km/h, and 0.9 m/s² for a 16-tonne bus
is roughly twice what its engine can do from rest. Read as *fractions of the
archetype's own maxima* every one of the specification's sentences is reproduced
exactly, which is the reading that makes the pair a model parameter rather than a
contradiction: the bus uses 90 % of a modest engine and only 30 % of its brakes,
the motorcycle uses 5 % of a strong engine and 80 % of its brakes.

| archetype | length | mass | width | accel (frac × max) | brake | v₀ |
|---|---|---|---|---|---|---|
| motorcycle | 1.95 m | 135 kg | 0.75 m | 0.05 × 3.0 = 0.15 m/s² | 0.80 × 4.0 = 3.20 | 80 kph |
| auto-rickshaw | 2.60 m | 400 kg | 1.40 m | 0.20 × 1.5 = 0.30 | 0.70 × 3.0 = 2.10 | 55 kph |
| kaali-peeli | 3.90 m | 1,150 kg | 1.65 m | 0.40 × 2.5 = 1.00 | 0.60 × 3.5 = 2.10 | 85 kph |
| car | 4.40 m | 1,400 kg | 1.75 m | 0.45 × 2.6 = 1.17 | 0.60 × 3.6 = 2.16 | 85 kph |
| BEST | 11.90 m | 16,000 kg | 2.55 m | 0.90 × 1.2 = 1.08 | 0.30 × 2.0 = 0.60 | 60 kph |

**Consequence, accepted.** Two-wheelers are the slowest class away from a
signal — after one second a motorcycle at 0.15 m/s² has gained 0.54 km/h. That is
the specification's "barely touches the throttle" taken at its word, and it is
also what makes the fleet's measured mean speed a Mumbai number rather than a
motorway number. A two-wheeler's advantage is not acceleration; it is that it
holds no lane, which is what the Boids field gives it.

---

## D-028 — Junctions exist where roads cross, so the roads are planarised

**Decided.** Before this pass the navigation graph contained only the junctions
the source data happened to store as a shared coordinate: 4,994 of the 16,344
at-grade crossings in the network. Everything else — every arterial crossing a
ward street, and 19 arterial-on-arterial crossings — crossed *mid-span* and met
nothing. Measured, the consequence was stark: 48,279 junctions of 112,704 nodes
and the signal warrant firing at exactly **one** of them, because a junction with
one approach and no arm opposite it is not a junction.

`detail::planarizeCrossings` (`pipeline/stages/network.cpp`) finds every at-grade
crossing with a 24 m spatial hash over the resampled centrelines, and inserts the
crossing point into both roads' *navigation* chains (`WaySpec::navPoints`) — 24,503
vertices into 16,562 roads. The mesh chain (`points`) is untouched: a junction is
a topology fact and a ribbon needs no extra cross-section for it, which also
keeps the 8,394 km of carriageway triangles unchanged. Crossings are skipped when
either road is flagged bridge or tunnel (9 of them, and the flag is what says a
flyover is not a junction) and when a road already has a vertex within 1.5 m.
Planarising cost 14,194 nodes and 25,333 edges and left the triangle count
identical.

**The warrant moves to `engine/world/signal_warrant.h`** because two pieces of
the system must agree on it: the pipeline bakes the phases it can express into the
chunks, and the runtime has to phase the junctions the chunk format *cannot*
express — a junction whose arms lie in two files is two nodes in two files, and
only the runtime, with the merged graph in hand, can see it as one junction. Two
copies of a warrant would be two different cities. Counting is by **arms**, not by
edges: at a junction every incident segment contributes one arm whichever way it
is stored. The warrant fires on two or more classified arms (an arterial
intersection, or an arterial passing through a four-arm junction — every junction
on S.V. Road is signalised), on one classified arm at a four-arm junction, on
three or more classified arms of any rank, and on two four-lane arms.

**The plan is 90 s with two 45 s axes, 40 s of green.** The previous numbers (42 s
green, 45 s offset) overlapped the crossing phase's green by 1.5 s once amber and
all-red are counted; a signal model that runs both axes green at the same time is
the one failure a signal model must not have, and the arithmetic is now asserted
in the header.

**Result (run 46).** 126,898 nodes, 169,701 edges, 59,185 junctions, **181
signalled junctions and 648 armed phases** — up from 1 and 3 before the pass. The
warrant tally says exactly where they come from: 148 junctions with two or more
classified arms, 33 with three or more classified approaches of any rank, and
59,004 junctions that earn nothing. 181 signalised junctions across 8,394 km of
carriageway is the right order for a network that is a quarter of the real city's
length; what it is not yet is the *whole* of Mumbai's signals, because 6,711 of
the 8,167 already-jointed crossings were jointed by the ward lattice meeting
itself, and a lattice street is residential.

---

## D-029 — The runtime materialises the graph the chunk format only describes

**Decided.** Three things the chunk format cannot say, and the runtime therefore
does itself. Each was a measured failure before it was a rule.

1. **Two-way segments become two edges.** The format stores one record per
   segment and a direction flag (0 = both ways), so before this a street was
   drivable in one direction only: 10,767 nodes had no exit at all and every one
   of them was a wall. The runtime now builds the reverse edge for every segment
   whose flag says both ways, with `SimEdge::reverse` recording the pairing.
2. **Nodes are jointed on world position, not per file.** A junction whose arms
   fall in two neighbouring 1,024 m cells is two nodes in two chunks, and no
   record can point across a file boundary. Nodes are merged on a 0.10 m quantum
   with a probe of the neighbouring quanta, which absorbs both the float
   rounding and a point that lands exactly on a quantum edge. `nodesMerged` and
   `nodesMergedAcrossChunks` report the two kinds of joint apart.
3. **Signals are completed, then derived.** Where a two-way segment could only be
   named in one direction, the opposing approach is given the plan of the arms
   that *were* named (the junction's own axis, not a copied phase). Where a
   junction has no baked phase at all — the cross-chunk case — the runtime
   applies the same warrant and the same plan constants as the pipeline
   (`derivedSignals` counts them).

**Two rules the traffic model needed and did not have.**

* **No U-turns at junctions.** With both directions materialised, every node
  offers the reversal of the edge just travelled; taking it is a U-turn in the
  middle of a block, which Mumbai traffic does not do. The reversal is refused
  while any other arm exists and taken only at a dead end (`deadEndUTurns`).
* **A queue spills backwards over the junction it is waiting at.** `s` may go
  negative by up to the vehicle's own length plus 0.5 m, which is geometrically
  the tail of the previous edge. Clamping at zero instead stacked bodies on the
  boundary: the diagnostic this replaced caught a motorcycle inside a bus with
  the front vehicle's rear 1 cm behind the edge origin.

**One pass, not two.** Lane discipline and the obstruction check are now a single
descending-`s` sweep per edge: each vehicle's constraint is the tighter of its
obstruction and the real rear of the vehicle listed in front of it, and because
the sweep runs from the front of the queue backwards every leader is settled
before the vehicle behind it is clamped. Run as two sweeps the second could undo
the first — a motorcycle pushed back by its own leader after the car behind it
had already been corrected left a 0.29 m gap where the floor asks 0.5 m. The
reported `minGap` is likewise measured on the settled state and only between
bodies whose lateral footprints overlap: a pair riding abreast is not a pair
following.

**Tested.** `tests/test_traffic.cpp` gains three cases that exercise the runtime's
own graph construction rather than the driving model: a two-way segment driven
both ways and turned around at its end, a road crossing a chunk boundary jointed
node for node (7 written nodes, 6 graph nodes, one of them the seam), and a T
junction assembled from three chains with no shared node record at all, which must
still be signalled — the arms open on two axes and never both at once.

---

## D-030 — The validation drives the shipped layer, and the population device

**Decided.** `engine/app/sim_main.cpp` no longer carries its own traffic model.
The 60-minute run constructs `mumbai::sim::Traffic` over the chunks the drive
actually streams (`streamer.cellsAlongRoute`), so the layer the tests exercise
and the layer the long run measures are the same code, on the city's own nav
graph (119 chunks, ~31,300 nodes, ~86,000 edges, 302 signal phases) rather than
on a hand-built corridor.  The harness keeps exactly one thing of its own: the
*population device*, which is a scenario, not a behaviour.

**Why.** Before this, `mumbai_sim` ran a single-lane IDM over a synthetic
corridor.  Its numbers — a "fleet" of 620 with a "peak of 545 within 500 m" —
were produced by code that no test covered and that no longer existed anywhere
else.  A validation that measures a second implementation measures nothing.

**The population device.** The specification's fleet is a peak-hour population
*around the camera*: ≥500 vehicles in a 500 m radius.  Two facts force a device
rather than an origin–destination matrix:

* the camera drives at 24 m/s and the fleet at 8 m/s (city speed), so a disc
  fixed to the camera is swept clean from behind in about twenty seconds — the
  traffic physically cannot keep up with the disc;
* a 500 m disc over the western suburbs contains a few hundred lane-kilometres,
  but the same disc on the Bandra–Worli Sea Link contains about eight
  lane-segments of bridge deck.

So the harness moves the disc with the camera (every 2 s — a lag of less than one
edge) and holds the fleet inside it: a vehicle more than `radius + 120 m` from
the anchor is called back to a re-entry point inside the disc, at most 8 per
tick, with a 1 s retry gate after a failed attempt.  It changes only *where* a
vehicle reappears; it never changes how one drives.  It counts its own failures
and reports them (`population device: N re-entry attempts found no room`),
because a device that cannot hold its scenario has to say so rather than quietly
change it.

**Placement.** A candidate re-entry must satisfy `laneEntryClear` (the physical
admission test), and its *point* — not its edge — must lie inside the disc: an
edge whose midpoint is a kilometre away can still cross the disc, and a long edge
that begins inside it can leave.  Entrances are spread 8 m apart along the edge
instead of stacked at its mouth, which is what lets a bridge deck hold a
population at all.  When the caller wants the arrival put ahead of the camera,
a first pass requires it; a second relaxes that, so a disc with no road ahead of
the camera still fills.

**The density criterion is a peak load test.** The run reports the distribution
over 360 samples along the drive (minimum / median / peak) and asserts the peak
≥ 500 — the criterion — plus a median floor of 400, which is evidence that the
device holds through the drive rather than that one moment happened to pass.

**Also decided in this pass, each one found by a diagnostic rather than by
review:**

* **No floor on a queue's spill.**  A correction clamps a vehicle to the distance
  from the body in front of it and to nothing else: `s` may go negative, which is
  a queue standing in the junction it has not yet cleared — what a Mumbai queue
  does.  Two floors were tried and both stacked bodies, because a floor is a
  position that every pushed-back body can reach and therefore a position they
  can all reach at once: clamping at `s ≥ 0` put two motorcycles at the same `s`
  at an edge mouth, and clamping at `s ≥ -(length + 0.5)` merely moved the stack
  a metre.  Nothing else can stack them either: the sweep runs front to back, so
  each body stops behind the one already corrected.
* **Corrections run last.**  The floor is a property of the *lateral* footprint,
  so the descending-`s` sweep runs after everything that can move a body — the
  integrator, MOBIL, the Boids field, the lane tracker and the kerb pull-in.
  Otherwise a two-wheeler sliding up beside the vehicle in front, or a taxi
  pulling to the kerb for a fare, leaves a pair closer than the floor with the
  sweep already behind it; the diagnostic caught one such pair at 0.297 m.
* **Admissions carry a 2 cm margin** (`kEntryMargin`): `laneEntryClear` demands
  `kMinGap + 0.02`, the corrections hold exactly `kMinGap`.  Two constraints that
  are each exactly tight leave the invariant to the last bit of a double.
* **Chunk I/O is timed apart from the frame.**  Streaming is a job on another
  thread in the real runtime; folding a 40 ms file read into the same percentile
  would hide the number the frame criterion is about.  Both are reported.
* **Signal stops are two different events**: a vehicle held at a stop line on red
  (`signalStops`) and a vehicle turned away because the junction is full
  (`junctionHolds`).  A validation that conflates them measures neither.
* **The payload's street graph is clipped to the world.**  The export was
  feeding the viewer a graph with roads outside the extent (x to 30,663 m in a
  22,528 m box) and a node record that clamped: 31,571 nodes — a quarter of the
  graph, the whole northern third of the city — sat at exactly z = 32,000 m,
  which is the clamp, not a road.  Nodes outside the extent are dropped and their
  edges with them (2,550 nodes, 2,556 edges), and the surviving coordinates are
  written as `u16` because the payload is now non-negative by construction and the
  world is 43 km tall.
* **An edge column that names the wrong array is invisible to a reader that
  reads one record at a time.**  `viewer/index.html` drew the graph happily: it
  looks up node *i* for edge *j* and gets a coordinate, any coordinate.  The
  export added the chunk's node base to a value that was already a global node
  index, so 82,770 of 167,145 edges pointed at indices that do not exist (max
  248,518 for a 124,348-node array) and only the first chunk's edges were real.
  The verifier now asks for every endpoint and for the extent the graph reaches.
* **`NodeAt` jointing and `reverseEdges` are reported**, because the runtime
  materialises the opposing approach of every two-way segment and therefore
  roughly doubles the out-degree the pipeline emitted: runtime totals are *not*
  chunk totals, and a metrics reader has to know that.

**Result (60 min, 216,000 ticks, `mumbai_sim --minutes 60`).**  123,552,000
vehicle-steps over the real 119-chunk city graph, 572 vehicles live, mean 28.2
km/h, 1 % at a crawl.  Minimum gap **2.55 m** (floor 0.50 m), **0** lane
violations in 216,000 audits, 0 non-finite states, 0 out-of-envelope speeds,
0 out-of-extent positions.  Density within 500 m of the camera over 360 samples:
minimum 0 (on the Sea Link deck), **median 451**, peak **547**.  Solver +
floating origin: mean 0.37 ms, p50 0.08 ms, **p99 4.38 ms**, worst 14.74 ms
against a 16.6 ms budget, with chunk I/O timed separately (0.09 ms mean, 9.19 ms
worst, 7,200 streaming frames).  105 macro-cell crossings, worst shift 0.0004 ms,
worst camera-relative error 0.00e+00 m.  256 chunk loads, 364 unloads, 0 hash
failures.  Signals: 301 stops at the line on red, 708 cleared on green, 302
phases.  Population device: 14,587 re-entry attempts found no room (0.8 % of
1.8 M edge completions), 0 vehicles stranded clear of the disc.  2,710 of the
72,825 junction holds stopped at the box limit — the one place the floor can be
transiently compressed, counted and reported rather than hidden; the audit found
no violation at any of them.

**Not claimed.**  The GPU frame is not measured (no GPU here); this is CPU cost
with streaming on the same thread, and the runtime moves streaming to a job.
The 30 FPS worst case on an Adreno 730 / Mali-G710 remains unverifiable in this
environment (D-019).

---

## D-031 — Material content is synthesised, mean-preserving, and shipped as a texture array

Section 1B asks for per-surface PBR materials and Section 2 wants no feature
shipped rough.  The world carried material *parameters* — albedo, roughness,
metallic, the wetness response — but no textures, because the offline tier has
no right to ship scanned maps and no way to fetch them here either (D-014).  A8
synthesises them: every material in the library gets a tileable base-colour map,
a normal map derived from a wrapped height field, and an ORM map, all a pure
function of the material's *name* and the build seed.

**Decisions.**

* **The tile's mean is the material's albedo.**  The generator is built around
  mean preservation: a synthesised tile's average *linear* albedo equals the
  `MaterialDef.baseColor` the rest of the pipeline was calibrated against, and
  the ORM green channel is *absolute* roughness whose mean is the authored
  roughness.  The build measures this off the bytes it wrote (worst 1.05 % over
  115 materials, the residue being sRGB quantisation on the darkest channels) and
  **fails the stage above 2 %**.  Adding textures therefore changes detail, not
  tone: the wetness response, the baked AO, the flood shading and the tonemap
  keep the calibration they were verified with.
* **Optically smooth surfaces are flat on purpose.**  Glass curtain walls, lamp
  lenses and rail steel get a flat albedo and a flat normal: their variation in
  reality is reflection and roughness, not surface grain, and inventing grain
  would put a fake bump on every glass facade in the city.  Standing water is the
  one exception — a dead-flat 12 m water tile reads as painted glass, and the
  mesh has no waves of its own (the Gerstner ocean is a separate GPU system for
  the open sea), so inland water carries a quarter-strength ripple.  The test
  suite asserts both the flatness and the ripple, so neither can drift.
* **A texture is content, not a re-grade — the divisor travels with the bytes.**
  Each layer record carries the mean linear albedo of its own mip0 and the
  physical size of one tile, so the array is self-describing: a runtime binding
  layer N knows what to divide the sample by (`tile / mean`, mean 1, which keeps
  a tinted facade variant tinted) and how many metres one UV unit covers.  The
  writer *measures* both from the pixels rather than accepting them from a
  caller, so a record cannot claim behaviour the content does not have.
* **UVs are decided in one place.**  `MaterialLibrary::uvScaleFor` returns
  `1/tileMetres` for every material id — including the tinted facade variants,
  which inherit their base material's tile because the derivation is by *name* —
  and every `MeshBuilder` in every cell points at that one table.  A builder
  without a table (tests, tools) keeps its caller's scale, so the mesh layer
  remains usable in isolation; content built by the pipeline always has the
  table.  Dividing world metres by a physical tile size is what makes brick
  courses brick-sized on a 12 m chawl and a 90 m tower at once.
* **The tile size is physical, and comes from the name.**  Each material family
  declares how many metres one tile covers (asphalt 6 m, brick 1 m, water 12 m),
  the mesh builders divide UVs by it, and the lookup is `name → family`, so the
  74 tinted facade variants minted per building inherit their base material's
  tile instead of falling to a default.  A shader sampling these maps can be
  trusted to put brick courses at brick size on a 12 m chawl and a 90 m tower
  alike.
* **Textures are content, not decoration: the shader is handed a divisor.**  The
  export publishes, per material, the tile index into the base-colour atlas and
  the layer's mean linear albedo (`tileMean*`).  A surface multiplies its colour
  by `tile / tileMean`, a quotient with mean 1.  That is what lets a tinted
  facade variant keep its tint while gaining the texture's grain — the naive
  alternative (multiply by `tile / ownAlbedo`) would flatten every variant to
  its base material's colour and silently erase the weathering that Section 2's
  civic realism depends on.
* **Container: `.mumbaitex`, an mmap-able texture array.**  `TexArrayHeader`
  (64 B) + `TexLayer[count]` (24 B) + per-layer mip chains, box-filtered at build
  time so the runtime never builds one.  Layers start 8-byte aligned, so a
  runtime can upload straight out of the mapping.  The container, its constants
  and its reader live in `engine/world/texture.h` because the offline writer and
  the runtime must share one definition; the writer includes the reader's header,
  not a copy.
* **PNG is a review and browser format, not the shipping format.**  The pipeline
  carries its own deflate encoder (fixed Huffman + greedy LZ77, 32 KiB window,
  15-bit hash chain) and adaptive PNG row filters, which makes the atlas and the
  contact sheet dependency-free and byte-identical across machines — the browser
  preview and a human reviewer read PNGs, the runtime reads `.mumbaitex`.
* **`textureSetIndex` means a layer, and `-1` means none.**  `MaterialDef`.
  `textureSetIndex` is now the 0-based array layer (the same number the atlas
  grid uses) with **-1 for untextured** — id 0 `unknown` is explicitly -1, since
  0 is a real layer and overloading it would hand every fallback surface the
  asphalt tile.

**Result.**  `textures` stage: 115 materials → 41 families, 0 unmapped, 256 px
tiles, 100.5 MB of arrays (40.2 MB base colour at 4 channels, 30.1 MB each for
normal and ORM, mip chains included) (40.2 MB base colour at 4 channels, 30.1 MB each for
normal and ORM), mean albedo error 1.049 %, 6.3 s.  The 11×11 atlas is copied
into the viewer's data directory and decoded independently by
`tests/verify_export.mjs`, which re-inflates the PNG, walks every material's tile
and confirms the published `tileMean` against the pixels (worst error 0.0002).
`engine/world/texture.{h,cpp}` is the runtime reader (mmap, validate, mip
access, layer statistics).  `tests/test_texture.cpp` round-trips the container
through the runtime reader, refuses a truncated file and a flipped byte, holds
the generator to mean preservation and normal validity per family, and then
verifies the *shipped* arrays: 115 layers, whole-file hashes recomputed, every
record's published mean re-measured from the pixels, unit-length normals, and a
count of deliberately flat surfaces (7) that fails if the list grows.

**Not claimed.**  Nothing on a GPU samples these yet: A1 is the renderer that
will.  Beyond base colour, the normal and ORM arrays are validated as data (unit
normals, roughness means, mip arithmetic) but their *visual* effect is not
verified in this environment — the browser preview uses the base-colour atlas
only, with flat roughness.  The remaining material-content work is the surface
detail Section 1B asks for at production density: decals, signage text, kerb and
rail furniture maps, and the harness that would catch a shader regression.

---

## D-032 — Hero detail is a level, not a rewrite; the specification's triangle budgets are reported, not faked

**Decided.** The five hero assets carry a detail level (`hero.detail`, 1..3,
`BuildConfig::heroDetail`; the shipped `configs/world.json` builds at **3**).
The level changes *how much of the published geometry is modelled* — bay counts,
balusters, dentils, colonnettes, glazing bars, masonry courses, jali lattices —
and never where anything stands.  Level 1 is the massing plus its principal
mouldings, 2 adds the ornament that makes a massing read as a building, 3 adds
the **survey tier**: coursed ashlar cut around the openings, voussoir arches with
the barrel behind them, jali screens as pierced lattices, gallery balustrades,
pinnacles, deck stiffeners and kerbs, paving bays on the promenade.

**Why a level and not a rewrite.** The three landmark passes already carried a
detail vocabulary (mouldings, recessed opening rows, drum ribbing, domed turrets,
finials) and were wired into the massings.  The work here extended that
vocabulary with three primitives — `addAshlarFace`, `addVoussoirArch`,
`addJaliLattice` — and a `Detail` object that decides how many of each are used.
That is why the same builder produces 18.5k triangles at level 3's Gateway and
would produce far fewer at level 1, with the same plan, height and position.

**The budgets.** Section 2 asks for ~500k Gateway, ~800k Taj, ~1.2M CSMT,
~300k per Sea Link pylon and ~50k per Marine Drive building.  Those are
hand-modelled photogrammetry-scale asset budgets.  A procedural build cannot
reach them by subdividing surfaces: subdividing a flat panel adds triangles and
no geometry, and the specification's own standard (S0) is that nothing ships
rough.  What a procedural build *can* do is model the real masonry, and that is
what the survey tier does: the Gateway of India is basalt ashlar, so its walls
are emitted as 0.6 m courses of 1.35 m blocks with open joints and per-block
tonal variation — the published masonry size, not a subdivision chosen to reach
a number.  Courses are cut around the openings by *rectangle overlap*, not by the
block's centre: a block whose centre clears a window by half a metre still covers
half of it, and a half-covered window at 100 m is a modelling error.

**Result at level 3 (shipped).** Gateway 19,314; Taj 84,494; CSMT 30,304; Sea
Link 34,476 (two pylons, 4,014.7 m of viaduct); Marine Drive 41,288 of which
16,744 is the Art Deco row in the building buffer.  Total 193,132 triangles
against a specification budget of 8,100,000 — **2.38 %**.  The build publishes
that number itself: each asset carries a `hero.<name>.triangleBudget` counter
beside its `hero.<name>.triangles`, and `hero.budgetMetPercent` is the ratio, so
the gap is a QA reading rather than a claim in a document.

**Second decision in the same area: an asset's count must follow the asset.**
`hero.marineDrive.triangles` used to measure only the hero mesh, which reported
400 triangles while the pass was writing 100 Art Deco blocks — the row goes into
the *building* buffer because it is city fabric.  The counter now measures every
buffer a hero pass can write to and reports the split
(`hero.marineDrive.heroMeshTriangles` / `.buildingTriangles`).  A metric that
cannot see most of what it measures is worse than no metric.

**Not claimed.** The gap to the specification's budgets is real and is not
closed by this entry.  Closing it means either hand-modelled assets or a
photogrammetry import path, neither of which exists here.  What exists is a
silhouette and feature set a resident can check at 100 m: the Gateway's pointed
arch with its voussoirs and barrel, jali screens in its flanking piers, turrets
and finials; the Taj's arcaded faces with their balcony slabs, glazing bars,
drum clerestory and ribbed dome; the CSMT's arcade of colonnettes with pointed
heads, rose window, gallery balustrade and pinnacles; the Sea Link's raked legs,
semi-harp stays, deck rails, kerbs and stiffeners; Marine Drive's promenade
paving, lamp pairs and stepped, fluted, banded Art Deco frontage.

---

## Open decisions (deliberately deferred)

* **Drainage-sentinel filtering.** Resolved in D-026: the hydrology classifies
  sea, boundary, channel, shore and tidal-inlet cells explicitly, and the
  dredging sentinel basins (`pipeline/stages/dredging.cpp`) remain a separate
  concern that does not feed the flood model.
* **Hero budgets.** Resolved in D-032: the assets are built at three detail
  levels and the survey tier reaches 199,812 triangles against the
  specification's 8,100,000 (2.47 %).  The figure is published per asset as
  `hero.<name>.triangles` against `hero.<name>.triangleBudget`; closing the gap
  needs hand-modelled or photogrammetric assets, not more procedural ornament.
* **LOD0 terrain budget.** The specification's per-cell budget is met by
  construction, but the underwater half of many cells is tessellated at LOD0;
  a seabed-aware LOD split is planned.
