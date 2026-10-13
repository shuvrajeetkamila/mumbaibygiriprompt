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

## Open decisions (deliberately deferred)

* **Drainage-sentinel filtering.** Resolved in D-026: the hydrology classifies
  sea, boundary, channel, shore and tidal-inlet cells explicitly, and the
  dredging sentinel basins (`pipeline/stages/dredging.cpp`) remain a separate
  concern that does not feed the flood model.
* **Hero budgets.** 500k / 800k / 1.2M / 300k-per-pylon / 50k-per-building are
  the specification's numbers; current assets are three orders of magnitude
  below them (accurate massings, honestly counted).
* **LOD0 terrain budget.** The specification's per-cell budget is met by
  construction, but the underwater half of many cells is tessellated at LOD0;
  a seabed-aware LOD split is planned.
