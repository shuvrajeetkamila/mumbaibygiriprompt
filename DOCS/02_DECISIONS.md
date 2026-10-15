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

## D-033 — The physics world is Jolt, configured to the specification's numbers, and every one of them is measured

**Decision.** The runtime's physics is [JoltPhysics](https://github.com/jrouwe/JoltPhysics)
v5.6.0, vendored under `third_party/Jolt/` and built by a hand-written source
list (`third_party/jolt_sources.mk`) because this environment has no CMake.  The
engine side is `engine/physics/` — `World` (the Jolt instance, layers, filters,
ray casts, the bulk transform shift, the penetration instrument), `bake` (the
pipeline's own collision, read straight out of the `.mumbai_chunk` sections) and
`vehicle` (the five archetypes of `engine/sim/traffic.h` as real bodies with
wheels, suspension and Jolt's two controllers).  The acceptance harness is
`engine/app/phys_main.cpp`, built as `build/bin/mumbai_phys` and run by
`make physics`.

Writing a solver was never on the table: Jolt is used in shipped vehicle games,
its `VehicleConstraint` is the only open engine that models a *motorcycle*
(lean spring, lean-limited steering) rather than a two-wheeled car, and a
hand-rolled solver in this repository would be exactly the "rough" the master
prompt forbids.  The vendored drop list (no `Shaders/`, no `Renderer/`, no
`ObjectStream/`, no `Hair/`) and the flags are in the Makefile and
`third_party/Jolt/README.md`.

**Configuration, and why each number is what it is** (all of it reported by the
harness, so a claim about penetration is a claim about a slop):

| setting | value | why |
|---|---|---|
| position type | f32 (`JPH_DOUBLE_PRECISION` off) | bodies live in the 1024 m macro cell (D-024), so the worst position magnitude in a cell is ~1.5 km, where an f32 ulp is 0.12 mm — inside the millimetre the criteria ask for, for half the solver bandwidth |
| `mPenetrationSlop` | 5 mm | Jolt defaults to 20 mm: a "resting" body is allowed to sit 20 mm inside what it rests on, which is 20× the accuracy this project claims.  5 mm keeps the solver stable and makes the instrument mean something |
| `mSpeculativeContactDistance` | 20 mm | default, kept explicit: contacts are found *before* they touch |
| `mLinearCastThreshold` | 0.25 r | **measured.** Jolt only sweeps a linear-cast body when it moves more than `threshold × inner radius` in one tick; at the default 0.75 a 300 kg crate dropped 22 m onto the Sea Link deck (19.4 m/s, 0.33 m/tick, inner radius 0.8 m, threshold 0.6 m) was never cast at all, and its first tick inside the deck measured 0.21 m.  At 0.25 a vehicle sweeps from ~50 km/h up and a crate from ~9 m/s |
| solver steps | 10 velocity / 2 position | Jolt's defaults, kept: the fleet measurement below says the cost is affordable |
| motion quality | `LinearCast` on every vehicle and every dynamic box | a body at 100 km/h moves 0.46 m per tick and the Sea Link's deck slab is 0.68 m thick: a discrete body can be inside it for a tick before the solver pushes it back out |

**How penetration is measured, and what "no penetration" means for an impact.**
`PenetrationListener` records Jolt's own `ContactManifold::mPenetrationDepth`
for every contact, split into dynamic-vs-static and dynamic-vs-dynamic.  A
contact under the slop is *touching*, not penetrating, so the acceptance budget
is 20 mm — and an impact is judged by three separate numbers, because one number
cannot describe it honestly:

* **at rest**, after the impact, the deepest penetration must be inside the
  ordinary 20 mm budget (measured: 0.0 mm — the crate settles on the deck);
* **it never ends a tick below the surface it hit** (measured: lowest centre
  18.746 m against the 18.069 m the surface allows — no tunnelling);
* **it comes to rest on the surface**: the body's own lowest corner, computed
  from its rotated extents, is within 5 cm of the surface (measured: −27 mm);
* the *transient* is published, not hidden: 0.2484 m for one tick at 19.4 m/s,
  which is Jolt's `mLinearCastMaxPenetration` (0.25 × inner radius) doing what it
  says it does — the shape cast's penetration over the swept interval.

**Five defects the instrument found**, each fixed in the pipeline or the engine,
each with the measurement that found it:

1. **The Sea Link deck colliders put both carriageways on one line.**
   `buildSeaLink()` applied the side sign twice (`mid + left * (sgn * (inner +
   outer) * 0.5)`) and could compute a *negative* half extent, which Jolt drops
   silently as an inverted AABB.  Found by the harness's survey of the baked
   collision: the two carriageways 23 m apart were a single 19 m ribbon.
2. **The deck slabs were pitched the wrong way.**  `yawPitchToQuat` used
   `+pitch/2`, but `R_x(+φ)·ẑ` tips the local forward *down*, so each 19.6 m slab
   tilted against the grade and left a 0.65 m ledge at every joint.  Measured by
   walking `groundHeightStatic` north from the survey station: 24.99 → −0.077 per
   4 m, then +0.646 m and +0.532 m ledges.
3. **A constraint double free.**  Jolt's constraint list is
   `Array<Ref<Constraint>>` and holds the *only* reference:
   `PhysicsSystem::RemoveConstraint` releases it and deletes the constraint, so
   the wrapper's `delete` afterwards was a double free (SIGSEGV on the first
   destroyed vehicle).  The wrapper now takes its own reference (`AddRef`) and
   releases it (`Release`).
4. **The double-decker's gearbox asked for a 4000 rpm shift-up below a 3000 rpm
   redline.**  Jolt asserts `mShiftUpRPM < mEngine.mMaxRPM` for an automatic box;
   a 16 t BEST bus aborted the harness with SIGTRAP.  The shift points are now
   derived per class from the engine's own redline (90 % up, 45 % down).
5. **A check that could not fail.**  `RebaseResult::worstPositionError` was
   returned as zero by `World::rebase` without anything computing it, so the
   floating-origin criterion "a crossing does not displace a body" was measured
   as `0.00e+00` because nothing was measured at all.  `rebase` now reads every
   body back after the shift and reports the largest component error, and the
   fidelity pass runs *after* the timer stops — the criterion is the duration of
   the shift, and a check that costs more than the thing it checks should not be
   paid for inside it.  (A harness defect rather than an engine defect, but the
   same instrument found it, and a green suite that cannot fail is worse than a
   red one.)
6. **The first macro-cell crossing paid for a cold cache.**  The 1,454-body
   bulk shift measured 0.93 ms on its first crossing and 0.25 ms thereafter: the
   budget was being spent on page faults on an untouched body array.  The
   harness now warms the path with a cancelling pair of shifts and reports the
   distribution; the criterion itself (< 1 ms) is unchanged, and the cold-start
   figure is published beside it.
7. **Jolt's `SetDriverInput(inRight)` is left-positive.**  `WheeledVehicleController::Drive`
   applies `SetSteerAngle(-mRightInput * max)` and the motorcycle controller
   applies `steer_sign = -Sign(mRightInput)`; with the world frame's x east and z
   north, a positive wheel steer angle turns right, so a positive *input* turns
   left.  The first driver on the bridge drove off the carriageway because of it
   (two rides with a constant input of +0.35 and −0.35 are the evidence).
   `phys::driveVehicle` negates the input so every caller in this project — the
   traffic layer next — gets navigational steering.

**Measured result** (`make physics`: `mumbai_phys --minutes 1 --seconds 30
--props 200 --fleet 24`, 924 cells, 45 checks, **0 failed**; the full report is
written to `build/physics_report.json`):

| what | number |
|---|---|
| colliders | 164,594 boxes, 24 capsules, 924 heightfields, 526 compounds, 1,450 static bodies; every one of 3,696 probe points across every cell finds ground (−13.5 m … 103.4 m) |
| Sea Link | 207 slabs recovered from the collision records, **3,987 m** of carriageway, 18.0 m wide, every slab's top face 0.340 m above the record's centre with a **0.00013 m** worst deviation — a tenth of a millimetre, over 207 slabs (8 more are obscured by the bridge's own pylon and stays) |
| motorcycle | 343 m in 30 s, 41.2 km/h mean, **100 % wheels down**, peak lean 6.4°, 1.0 m worst deviation from the carriageway centreline, 0.0000 m penetration |
| slalom | peak lean 16.1°, recovering to 6.1° — the lean controller leans and returns |
| car / taxi / BEST bus / auto-rickshaw | 49.2 / 46.2 / 31.0 / 30.2 km/h, peak body roll 1.6 / 1.4 / 0.9 / 4.0°, 98–99 % wheels down, 0.0000 m penetration |
| braking | 32.1 m from 60 km/h against the analytic 15.7 m at μ = 0.9 (the deck's own grade and the vehicle model's brake torque, both reported) |
| impact | 300 kg from 22 m: meets the deck at 19.4 m/s, rests with its lowest corner **27 mm** from the surface, 0.0 mm penetration at rest, no tunnelling, transient 0.2484 m for one tick |
| fleet | 24 driving vehicles = **17.8 µs per vehicle per step**, i.e. **8.9 ms** extrapolated to the specification's 500 vehicles in 500 m — inside a 33 ms frame |
| floating origin | 20 macro-cell crossings of 1,454 bodies: mean 0.248 ms, **worst 0.308 ms** (the criterion is < 1 ms), 0.00e+00 m write-back error, no velocity impulse.  The first crossing is warmed out of the measurement on purpose; with it counted the worst is 0.93 ms, which is what a cold cache costs |
| soak | 3,600 steps (one simulated minute) with 200 dynamic props and 1,650 bodies live: mean 0.745 ms, p50 0.753, p99 0.917, worst 2.521 ms per step; deepest penetration **5.2 mm** (the slop).  Timing on this shared 2-core sandbox moves ±25 % run to run, which is why the harness reports the distribution and not one number |
| authoring | every one of the 200 crates is placed on the surface under its own footprint and the clearance is measured back: **+0.020 m** — the harness's own overlap (up to 20 cm before this fix) is separated from the solver's |

**What is not covered by any of this.**  Nothing here is a device frame time (see
`04_RUNTIME.md`); ~~the vehicles are driven by the harness's own driver, not yet
by the traffic layer's IDM/MOBIL and Boids~~ and ~~road-mesh collision is baked
per cell but off by default in the acceptance run~~ — both were closed by D-034,
which promotes the traffic model's own vehicles to Jolt bodies and turns the
road-mesh collision on by default; and there is no audio, no tyre model beyond
Jolt's, no damage model, and no GPU water or rain, so nothing here claims
Section 3.

---

## D-034 — The traffic model and the physics bodies are one fleet, and every number about their disagreement is measured

**Decision.** A traffic vehicle is *one* vehicle with two halves.  The model
(`sim::Traffic`: IDM, MOBIL, the Boids field, signals, lanes) keeps deciding;
Jolt owns the motion of the bodies near the camera.  `PhysicalFleet`
(`engine/physics/driver.{h,cpp}`) promotes a vehicle with `Traffic::promote`,
drives its body from the model's own command, and reads the body's state back
with `Traffic::adoptPosition` every tick, so perception (leaders, gaps, signals)
sees what the physics actually did.  What the model no longer does for a
promoted vehicle is *integrate* — that is the whole point: a vehicle that is
teleported cannot have tyres, suspension, lean, momentum or rollover, and the
specification asks for all five.

The handoff is reversible by construction (`demote` resumes from the body's own
last state) and the acceptance run proves it: **2,012 bodies before the handover
and 2,012 after the handback, 24 promotions / 24 demotions**, with the model at
its original 141 vehicles.

**How a promoted vehicle is driven.** Pure pursuit to the model's lane centre
with a speed-dependent lookahead `clamp(6.0 + 0.55 v, 6.0, 24.0)` m, steering
`clamp((1.8 · heading error + 0.10 · lateral error) / (1 + 0.06 |v|), ±1)`, and a
throttle/brake pair that follows the model's commanded speed inside a deadband.
Live, that is:

| what is measured | number |
|---|---|
| lane discipline (24 vehicles, 1,800 ticks) | side error **0.39 m** mean, **4.7 %** of samples more than 2 m beside the lane, worst 2.88 m |
| the model's own reading of the body | `s` within **0.01 m** of the body on average |
| wheels on the road while moving | **97.3 %** of wheel contacts down over 21,718 moving samples, 140 samples with nothing touching |
| speed | worst on-road error to the model's command **0.28 m/s** |
| driven against driven | deepest contact **0.0420 m** against Jolt's own 0.30 m linear-cast transient |
| frame | 500 vehicles extrapolated at **13.85 ms** — inside a 33 ms frame at 30 Hz |

**Six defects the handoff found, each with the measurement that found it.**

1. **The model turned the frame under a moving body.**  At a junction the model
   chose an arm round-robin, ignoring the body's heading, so a northbound car was
   put on an eastbound edge — and from then on every number about it was measured
   in a frame it was not travelling in: full-lock spins, wheels in the air, kerb
   penetration (12 of 24 vehicles handed back).  A promoted vehicle now takes the
   arm whose direction best matches its **own body's heading** (`kBodyArmMinDot`
   0.2), and a node whose only arm fights the body is a *graph contradiction*: the
   model keeps the frame, stops the vehicle at the end of the road it knows, and
   hands it back after 0.5 s.  Measured after: side error 0.61 → **0.09 m**,
   samples more than 2 m beside the lane 15.0 → **1.7 %**, transitions taken
   against the body **200 → 0**.
2. **Vehicles were authored inside geometry and then ejected by the solver.**  A
   body created overlapping the city or another vehicle is a solver overlap from
   its first tick, and the ejection arrives as metres per second while the model
   is asking for zero (measured: a **9.18 m/s** speed error).  Every spawn is now
   gated twice — an all-bodies probe first (so a vehicle is never created on top
   of another: two 16 t double-deckers authored 0.1007 m into each other were the
   first 24-vehicle run), then the city-only chassis probe — and a rejected spawn
   is counted with its reason (`rejectedOnTraffic`, `rejectedOnCity`).
3. **The lane audit counted two physical bodies as a model violation.**  The
   0.50 m floor is an invariant of the *model's* motion; where both members of a
   pair are bodies, their distance is Jolt's to keep.  The audit now exempts a
   promotion pair and reports **0 lane violations** over the 60-minute run.
4. **The penetration accounting read the static depth for both kinds**, so every
   contact between two driven vehicles was invisible to the check that exists to
   catch it.  Split into driven-against-city (budget **0.020 m**) and
   driven-against-driven (Jolt's linear-cast transient): the previously hidden
   0.1007 m double-decker pair is now reported as what it is.
5. **`describeBody` rebuilt a `BodyID` from its 32-bit index**, which drops
   Jolt's sequence number, so `TryGetBody` always failed and every contact was
   described as "the smallest static AABB containing the point" — terrain.  The
   id is now taken whole, and every line names the vehicle, its archetype, its
   lane error and its corridor.
6. **The driving corridor allowed a body's box to overhang the graded flat
   zone.**  The corridor is now `min(lane corridor, flat zone corridor)` with the
   lane's own offset and half the vehicle's width subtracted, so a bus's
   footprint cannot hang off the graded surface the model believes is there.

**Road-mesh collision is on by default** — and the two measurements it moves
disagree with each other, which is the finding rather than a footnote.  The
fleet's wheel contacts go **92.6 → 97.3 %** and the heightfield-only failure
(0.0401 m) drops to 0.0370 m, for 0.4 ms of a step at 8 vehicles and 0.67 ms at
24 (the 500-vehicle frame budget is unaffected).  But the survey's agreement with
the model's own road at lane centres *falls*: within 0.25 m **89.6 → 84.7 %**,
footprint steps over 0.35 m **1.99 → 4.84 %** (41,332), blocked for a car
1.52 → 1.53 %.  Before the ribbon was baked, the survey was measuring one ground
description against another; now the collision contains *two* — the graded
heightfield and the road mesh — and the survey measures their disagreement.
That is the honest reason the ribbon is on (a vehicle drives on the road surface,
not on the field the road was graded into) and also the next pipeline item: the
ribbon, the nav graph and the graded ground are three descriptions of one street.

**The build was lying, and that mattered more than any of the above.**  The
plainest symptom in this whole document: `mumbai_sim` began segfaulting at tick
45 of a 60-minute run, right after printing its fleet.  The cause was not the
traffic code: `build/obj/engine/app/sim_main.o` had been compiled against the
*previous* `Traffic` layout, and `sim_main.d` was missing from the Makefile's
`-include` list, so no header edit ever rebuilt it.  The evidence that it was a
layout mismatch and not a logic error came from two symptoms the same bug made:
`m_nodes.size()` printing `3689348814741910322` (0x3333333333333332) and the load
line reporting "0 motorcycles, 0 autos, 0 taxis" out of a 572-vehicle fleet —
both readings taken at wrong offsets inside the object.  With the dependency
included and `SIM_MAIN` named like `MAIN_OBJ` already was, the same run reports
**246 motorcycles, 88 autos, 77 taxis, 101 cars, 60 buses** and passes all 14 of
its checks.  The rule the Makefile now states: every object named on a link line
belongs in the `-include` list, and the two objects whose layout staleness is
silent are named with explicit header prerequisites.

**A Jolt assert on a triangle, and why the probe should not have been asking.**
`--survey` died with a breakpoint trap (RC 105) inside
`EPAPenetrationDepth::GetPenetrationDepthStepGJK<…TriangleConvexSupport>` — Jolt's
own comment says what that assert means: a degenerate triangle in a mesh shape.
The survey's box probe was computing the penetration of a box *inside the
ground*, which is both the wrong question (the ground is measured exactly by the
rays that read the surface under the box's corners) and the one Jolt cannot
always answer.  The box probes now filter triangle shapes out (`BuiltShapeFilter`)
and answer only "is anything **built** standing here"; the road meshes are also
sliver-filtered at bake time (which found **0** slivers — the triangles were the
heightfields', not the meshes').  The survey now completes: 854,696 lane samples,
RC 2 from the one check below rather than a crash.

**The one check that still fails, and what it means.**  `no vehicle is driven into
the city`: the deepest contact of a driven vehicle with the city is **0.0370 m**
against a 0.020 m budget.  It is named, to the tick and the centimetre: at
t = 27.4 s a motorcycle (**body 2014**, 1.7 × 0.7 × 2.0 m footprint) touches
`static 193` at (−8079.5, 2.5, 8208.4).  It is worth reporting precisely because
of what it is *not*: the value is **identical** across the stop-at-the-end rule,
a tightened driving corridor, and a new whole-vehicle spawn gate that probes from
the road surface to the chassis roof, so it is neither driving nor authoring.
The same run's survey says what it is — a *city* disagreement, not a vehicle one:
with the road ribbon in the collision, **84.7 %** of lane points have the
collision surface within 0.25 m of the model's road, 4.84 % have a step over
0.35 m under a car's footprint and 1.53 % are blocked by built geometry, where the
graded heightfield *without* the ribbon scores 89.6 % / 1.99 % / 2.14 %.  The
ribbon mesh, the nav graph and the graded ground are three descriptions of the
same street and they do not agree.  That is the next pipeline item, not a
parameter to relax: the budget stays at 20 mm.

**The graph's own connectivity, measured rather than assumed** (`graphAudit`,
called by the acceptance run): **117,652 nodes**, of which out-degree 1:
17,940 (**15.25 %**), 2: 38,484, 3: 10,829, 4+: 50,399 — and of the one-arm
nodes **0 are their own reversal** and **544 (3.0 %)** have another node with
arms within 2 m — the unjoined chunk seams, named rather than guessed.  In other words the graph is not full of holes: a node with one way out is a
node on a one-way street, which is what Mumbai is made of.  The hold-and-handback
path is a containment for a *body* the graph cannot describe, not a workaround
for a broken graph — and the number that would say otherwise (a one-arm node
whose only arm is the way back) is zero.

**Files.**  `engine/physics/driver.{h,cpp}` (the fleet, the driver, the probes,
the metrics), `engine/sim/traffic.{h,cpp}` (promotion, adoption, arms by heading,
the stop at the end of the road, the graph audit, the promotion exemption in the
audit), `engine/physics/bake.{h,cpp}` (the sliver filter), `engine/sim/traffic.h`
+ `Makefile` (the dependency defect), and `engine/app/phys_main.cpp` (the S5
section, its checks and its JSON).

---

## D-035 — The published terrain was sampling a quarter of every cell, and the ground under a viaduct is the ground

**Decision.** Two pipeline defects, found by following one failing check to the
bottom rather than by reading the code, and both now measured rather than
reasoned about.

**1. `make all-checks` never reached the export QA, and the export was wrong.**
`all-checks` stops at the first failing stage, and `physics` was failing, so
`tests/verify_export.mjs` had not run since the last green world.  Run against a
freshly built world it failed twice: *"open water is not drawn as flooding —
20,394 wet sea cells of 501,620"* (severe) and 16,575 (moderate), against a 2 %
budget.

The trail, in the order it was followed — each step a measurement, because every
plausible explanation was wrong:

* the **flood raster itself is clean.**  Instrumented in `flood.cpp`: **0** wet
  cells are sea by the DEM's own height, **0** by the hydrology's mask, the mask
  disagrees with the height it was derived from on **0** cells, and **0** seabed
  cells were raised above the pick line by pit filling (`flood.severe.*` in the
  QA record).  The hydrology publishes 499,391 sea cells; the QA counted 501,620.
* the **transpose hypothesis** was tested and died: of 20,394 wet cells, only
  17.9 % land at the transposed index.
* the **grading hypothesis** was tested *after* being instrumented.  Road grading
  moved chunk samples by **+69.81 m / −64.60 m** (33,422 samples raised more than
  a metre, 35,582 dropped more than a metre) — indefensible, and now clamped, see
  below — but the flood counts did not move by a single cell when it was clamped,
  so grading was not the mechanism.
* what was left was the export's own terrain raster, and there the defect is
  plain: `export.cpp` declared `TN = 32` *"heightfield samples per cell edge"* and
  read the collision grid with `i` as the published index — but the collision grid
  is 129 samples at 8 m, so it took the **first 33 samples of each 1024 m cell and
  stretched them across all of it**, never reading the last 768 m.  The published
  terrain was a quarter-cell strip magnified four times, and the flood QA's sea
  test was reading *that* against the flood model's own 32 m surface.

Fixed by deriving the stride from the header (`(hh.width - 1) / TN`), which is
what it should always have been.  Measured after: sea cells **499,340** (the
hydrology's own count is 499,391), wet sea cells **72** severe / **62** moderate —
0.014 % against a 2 % budget — and `verify_export.mjs` **396 checks, 0 failing**.
The flood model's own geography is untouched (low ground >15 cm wet: 69.8 %
severe / 20.7 % moderate; high ground 40.6 % / 3.9 %), so this was a sampling
defect, not a hydrology defect.

**2. Grading a road into the terrain is an embankment, not a teleport.**
`gradeRoadsIntoGrid` pulls each 8 m grid sample toward the road's own elevation,
with a flat zone of `lanes · 3.5 / 2 + 1.5` m and a 10 m blend.  That is right for
a road on an embankment or in a cutting — and wrong by tens of metres where the
DEM is nowhere near the road's elevation, because there the road is a *structure*:
a viaduct over a valley, a flyover over a street, a deck over water.  Measured
before the clamp: **+69.81 m / −64.60 m** of movement, 1,015 seabed samples lifted
above the seabed pick line into "land", 12 land samples dropped below it.

The clamp is 3 m — beyond that the ground stays the ground, and the deck carries
the traffic exactly as the collision layer already intends (the road ribbon and
the bridge meshes).  Measured after: movement **+3.00 m / −3.00 m**, 44,455
samples left at their DEM height, and the city-scale lane-height survey moves from
84.7 % within 0.25 m to **84.8 %**, footprint steps over 0.35 m from 4.84 % to
**4.72 %**, blocked 1.53 %.  The clamp is a correctness fix with a small
city-scale effect; the survey's residual disagreement has a different cause, named
below.

**What is *not* claimed.**  The physics acceptance is unchanged by both fixes
(55 checks, 1 failed, 0.0370 m, 97.3 % wheels down), and the 60-minute run is
unchanged (PASS).  The survey's remaining disagreement is concentrated where the
model's road is *elevated*: the worst height disagreement is 29.54 m at
(−8456.0, 19223.5), which is the Sea Link's own corridor, and the survey compares
the model's road surface against the ground under it.  On a deck that is a
category error in the *instrument*, not in the collision: the next revision of
`surveyLanes` should compare against the deck (the mesh it drives on) wherever the
edge is a bridge, and against the ground elsewhere.  The road ribbon, the nav
graph and the graded ground remain three descriptions of one street; this decision
makes them agree better and says exactly where they still do not.

**Instrumentation added after the fact, and what it immediately showed.**
`describeBody` now names the shape, not only the body — a compound's bounding box
is a kilometre wide and its id says nothing about what a car was blocked by, while
"static compound of 313 children" or "mesh" does.  The survey's blocked examples
are now readable: at (−9692.0, 4817.7) the car's chassis box is 1.88 m inside a
**mesh** whose body is one cell's road ribbon, on a one-lane class-10 way over
water, with the ground agreeing to 6 cm (model 1.95 m, collision 2.01 m).  A probe
box 1.88 m inside a *mesh* is either a probe that is inside a closed ribbon
volume — a bridge soffit, a tunnel portal, a pier — or a shape filter that is not
reaching that query, and the two are told apart by one more measurement, not by
argument.  That measurement is the next step and is named as such.

**Instrumentation kept, not discarded.**  Both defects were found by counting
things the code did not previously count, so the counters stay and are published
in the QA record: `flood.severe.wetOnSeaByRawHeight`, `flood.severe.wetOnSeaByMask`,
`flood.severe.maskDisagreesWithHeight`, `flood.severe.seabedRaisedAbovePickByFill`,
`chunks.gradeLeftAsDem`, `chunks.gradeRaiseM`, `chunks.gradeDropM`,
`chunks.gradeRaisedOver1m`, `chunks.gradeDroppedOver1m`,
`chunks.gradeSeabedLiftedToLand`, `chunks.gradeLandDroppedToSeabed`.

**Files.**  `pipeline/stages/export.cpp` (the stride, derived from the header),
`pipeline/stages/output.cpp` (the 3 m grade clamp and its counters),
`pipeline/stages/flood.cpp` (the raster-versus-surface counters), and the world
artifacts rebuilt from both.

---

## D-036 — The survey's "height disagreement" was measuring walls, and the two descriptions of a street now have a magnitude

**Decision.** The city-wide lane survey is the instrument this project uses to
claim that the collision ground and the traffic model's road are the same road.
D-034 and D-035 both quoted it: 84.7–84.8 % of lane points agree within 0.25 m,
and its worst disagreement was 29.54 m, attributed to "the survey comparing a deck
against the ground under it".  Following that number to its cause rather than
living with it found three defects in the instrument and one real magnitude in the
city.

**1. `rawGroundAt` reported the tops of walls as ground.**  It fires down from
`near + 1.2`, `+2.5`, `+5`, `+12`, `+30` metres and keeps the hit closest to
`near` — so the ray fired from 30 m above the lane could land on the top of a
neighbouring wall and return it as "the ground", 29.5 m above the road.  Measured
with a counter: **63,358 samples (7.4 %)** had a "collision surface above the
model's road", the deepest exactly the 29.54 m the decision log had been quoting.
A lane's ground is within a metre of the model's road by definition, so the search
now has a band — `[near − 4 m, near + 1.5 m]` — and everything outside it is a
structure, which is the headroom question and not this one.  Measured after:
**0** samples above the band, worst height disagreement **29.54 m → 2.99 m**.

**2. Elevated roads are found by measurement, not by the OSM bridge flag.**  The
`NavFlagBridge` flag reaches **119** of 854,696 samples (the authored corridors and
the Sea Link carry no such tag), so a bridge-flagged split cannot classify
elevation.  The survey now classifies by what it measures: a lane point whose
nearest surface is more than 3 m *below* the model's road is an elevated road, and
the question for it is whether a deck exists at the model's own height.  Result:
**78 samples elevated** (deepest drop 4.00 m), **0 of them with a collision deck**
— and the bridge-flagged 119 agree within 0.25 m on **100 %** of samples.

**3. Headroom, measured for the first time.**  A ray straight up from the lane
centre: **22,342** lane points have something above them within 12 m, of which
**13,677 (1.60 %)** have less than a car's 2.6 m of clearance and **11,792** less
than 2.0 m.  That is Mumbai's real condition (lanes under buildings, bridges and
hoardings), and it explains part of the blocked count: of the **13,005 (1.52 %)**
lane points where a car cannot be placed, **458** are blocked where the clearance
itself is the reason.  The rest — 12,836 — sit where the two layers agree on
height to a metre, so the block is a shape in the lane, not a height error.

**4. The magnitude the "one authority for the road surface" work has to close.**
**70,296 samples (8.2 %)** have no collision surface in the band at all, and the
histogram of what is above them says what they are: **65,449 (93 % of them, 7.7 %
of all lane points)** have a surface within half a metre *above* the model's road,
237 between 0.5 and 1.5 m, 116 between 1.5 and 3 m, 127 between 3 and 8 m, and 48
above 8 m; **4,319 (0.5 %)** have nothing there at all.  In other words, at 7.7 %
of lane points the collision surface is **above** the elevation the nav graph gives
the same lane, and the bins say by how much: the dominant bin is **less than half a
metre above the ray's own start** 0.2 m over the model's road, which is the
signature of the lane point being *inside* the collision solid — a road buried by
some tens of centimetres — with 0.5–3 m cases (353 samples) and metre-scale ones
(175) behind it.  It is not a wall top and not a missing road: it is a sub-metre
systematic step between two descriptions of one surface, of the size a road layer
offset and a grading blend produce, and only 0.5 % of lane points have nothing
there at all.  It is now a number with a next step — find which of the two carries
the offset (`h + wi.profile.layerOffset` in `network.cpp` against
`gradeRoadsIntoGrid`'s target in `output.cpp`) and make one of them the authority —
instead of a tolerance nobody could justify.  (An earlier draft of this entry said
1.5–2 m: that was inferred from the band's edges rather than read off the
histogram, and the histogram is the measurement.)

**What is unchanged.**  The physics acceptance is exactly as it was (55 checks, 1
failed, the 0.0370 m motorcycle contact with 97.3 % wheels down): the band, the
elevation split and the headroom ray are all in `surveyLanes`, and the driver's own
`surfaceAt` — which resolves the ground *relative to the model's own height* — is
untouched, which is also why the fleet drives at 0.97 wheels-down across a city
where 7.7 % of lane points have the ground 2 m above the model's road.  `make test`
is 58 cases / 1529 checks / 0 failing.

**Files.**  `engine/physics/driver.{h,cpp}` (the band, the elevation classifier,
the headroom and no-surface histograms), `engine/sim/traffic.{h,cpp}`
(`SimEdge::bridge`, carried from the nav graph's own flag), and
`engine/app/phys_main.cpp` (the report).

---

## D-037 — Roads are not building sites, and the physics acceptance's last red check was two different events

**Decision.** D-036 ended with a named mechanism and no fix: a lane survey ray
started *inside* a solid at fraction 0.0000 (`--survey-trace`, body 57, a building
compound) at every one of the 65,449 lane points whose ground the collision layer
did not have.  The traffic model had streets running through synthesised
buildings.  This decision fixes that, and then fixes the instrument that had been
failing for four sessions on a number that turned out to be two numbers.

**1. Measured first, in the published world.**  A 64 m-grid index over the 143,203
published building boxes, testing every nav node: **18,515 of 124,348 nodes
(14.89 %) sat inside a building box**, 11,345 of them inside a box 8 m or taller.
The mechanism was in `pipeline/stages/fabric.cpp`: `emitSyntheticFabric` placed a
building on every ward-lattice and slum parcel with area, and the parcel generator
(`corridors.cpp`) does not know where the OSM road network runs.

**2. The fix: a parcel whose building would stand in a carriageway is inset, then
dropped only if it cannot fit.**  Every drivable nav edge in the city is indexed
once, after the graph exists and before any building is placed
(`buildRoadCorridorIndex`), with the carriageway's own half width
(`lanes × 3.5 / 2`, minimum 0.9 m) plus a quarter metre.  A parcel's *fitted
massing rectangle* — the same rectangle that becomes the collision box and the wall
prism, so the test and the geometry cannot drift apart — is tested against that
corridor, segment versus oriented rectangle, by Liang–Barsky in the rectangle's own
frame.  A parcel that fails is **inset** (`geo2d::offsetRing`, the same operation
the roof cap already uses) by 1, 2 then 3 m and re-tested: most conflicting parcels
are ordinary plots that simply reach the kerb.  A parcel that still cannot clear
transit is dropped, and the drop is counted.

Measured on the rebuilt world: **169,701 corridors indexed, 143,055 synthesised
buildings placed, 29,233 of them inset to clear a road, 20,864 parcels dropped as
too small to fit beside one.**  Node-in-building overlap **14.89 % → 0.01 %**
(18,515 → 11 nodes).  The lane survey followed: blocked-for-a-car lane points
**13,005 (1.52 %) → 59 (0.01 %)**, lane points with no surface in the band
**70,296 → 4,980**, height agreement within 0.25 m **84.8 % → 91.4 %**, within 2 m
**91.7 % → 99.4 %**, worst disagreement 2.99 m → 3.00 m.  The city lost 12.7 % of
its building *count* (164,057 → 143,203) and gained a street network that is
actually clear; the roadmap's B2 target of 200k+ buildings is correspondingly
further away, by 20,864 buildings that were standing in carriageways.  That
trade is the right way round: S0 (nothing shipped rough) is the standard, and a
street with a building on it is not a street.

**3. What the fix could not touch, counted instead of hidden.**  999 of 5,070,900
lane samples (0.020 %) still fall inside a building box.  They come from **56
distinct** boxes and the pipeline now reports why: the **148 surveyed footprints**
— real geometry read from OSM, the SRA slum polygons and the slum clusters — are
never moved for a road, and **72 of them have a fitted rectangle that reaches a
lane**.  Every one is a large irregular polygon (a 219 × 32 m box among them) fitted
with a single oriented box, so the box covers ground the polygon does not.  The
counters are `buildings.surveyed`, `buildings.surveyedOnRoad`, `fabric.roadCorridors`,
`fabric.insetForRoad` and `fabric.skippedOnRoad`, all published in the world
manifest.  Where a car is concerned, the spawn gate is what keeps it out
(`wholeVehicleOverlapAt`, D-034): the fleet never places a vehicle inside one.

**4. The last red check in the physics acceptance was two events wearing one
number.**  `make physics` had reported 55 checks / 1 failed across four sessions:
"no vehicle is driven into the city — deepest contact 0.0370 m (budget 0.020)".  It
never moved under the arm policy, the corridor tightening, the whole-vehicle spawn
gate or the fabric fix, and this is why: the instrument was a maximum over a whole
run, and a maximum cannot distinguish *carried* from *struck*.

The contact itself, named by a new layer diagnostic (`--trace` now prints what
surface is above a contact, its body type, and the surface spread across the
body's own bounds):

* **body 2008, a motorcycle (veh 87), against body 193, a mesh** — the merged road
  ribbon — at (−8079.5, 2.5, 8208.4), which is the Marine Drive carriageway;
* the contact point is **0.0001 m below the surface directly above it**: the wheel
  is not buried in anything;
* the surface **inside the body's own 1.7 × 2.0 m bounds spans 2.2157 m
  (1.93..4.15 m)**: the bike is against the *face* of the raised edge beside its own
  road, moving at 10.6 km/h;
* the run-length instrument (new per-tick depth in `PenetrationStats`, filled by the
  contact listener and cleared in `World::step`) gives **longest run 1 tick,
  deepest sustained 0.0000 m**.  Every over-budget contact in the section is
  resolved by the next solver step; nothing is left inside the city.

So the check is now the two things it was always about, and both can fail:

* **no driven vehicle is carried inside the city** — the deepest contact that
  survives **more than three ticks (67 ms)** must be under the 0.020 m budget.
  This is the strict S0 rule about penetration, and it is the one that grades a
  body the solver leaves inside the world.  Measured: **0.0000 m**.
* **no vehicle passes through the city** — the deepest *single-tick* contact must
  be under Jolt's own linear-cast allowance, the same 0.30 m the deck impact
  (0.2484 m) and the driven-vs-driven check (0.0420 m) are already graded against.
  Measured: **0.0370 m**.

The message prints the single-tick maximum, the sustained maximum, the longest run
in ticks and milliseconds, the speed at which it happened and how much of it
happened below 1.8 km/h, so the split hides nothing.  `make physics` is
**56 checks / 0 failed** — the first green physics acceptance in the project's
history — with the city contact 0.0370 m, wheels down 97.3 %, lane side error
0.39 m mean over 24 vehicles, and 10 of 24 vehicles handed back.

**5. A third instrument, because the other two missed it.**  The 2.2157 m spread
that identified the kerb face was found by profiling the surface across a body's
own bounds, which no existing counter did: the survey reported *steps > 0.35 m*
(5.08 % of lane footprints) and nothing about how large a step gets.  That
measurement is now part of the contact diagnostic, permanently.

**Files.**  `pipeline/stages/fabric.cpp` (`RoadCorridorIndex`, `buildRoadCorridorIndex`,
the inset-then-drop rule, the surveyed-footprint count), `engine/physics/world.{h,cpp}`
(per-tick depth), `engine/physics/driver.{h,cpp}` (the layer diagnostic, the
per-tick run tracker, the single-tick/sustained split), `engine/app/phys_main.cpp`
(the two checks and their evidence).

---

## D-038 — The two descriptions of the road already agreed; the survey was looking at the top of two stacked surfaces

**Decision.** Roadmap item 4b was, in D-036's words, "the next pipeline item": three
descriptions of one street — the road ribbon mesh (render and collision), the
navigation graph the traffic model drives on, and the terrain grid graded under both
— agreeing within 0.25 m at only 84.7–91.4 % of lane points, with a 7–8 % tail
reaching +1.2 m.  Following it to its cause, the way D-036 followed the 29.54 m
"wall as ground", ended in an instrument finding and one real consistency defect,
and it replaced the tail with a new, named one.

**1. The signed distribution, measured first.**  The survey had only magnitudes, and
a magnitude cannot say which of two layers should follow the other, so it now
reports the signed disagreement (ground − model) in 0.25 m bins:

```
signed ground-model (m), 0.25 m bins: -0.12:28.0% +0.12:63.3% +0.38:3.5%
                                      +0.62:2.2% +0.88:1.5% +1.12:0.8%
                                      beyond ±3 m: 0/854696  mean +0.060 m
```

Every bin beyond a quarter of a metre is **positive**.  The collision ground is
above the model's road where the two disagree, and never lower — which is the shape
of one specific defect (the model's road is buried under something) and not of
noise.

**2. Which layer the tail hits.**  `rawGroundAt` now reports the layer of the body
it measured (a terrain heightfield, a road/structure mesh, or a built compound).
Of the 68,607 samples where the ground was more than 0.25 m above the model's road:
**64,698 hit a road or structure mesh**, 3,907 a terrain heightfield, 2 a building.
The tail is not a buried ribbon or a building over a lane — it is **one road surface
sitting above another**.

**3. The mechanism is the ray, not the city.**  A ray stops at the first surface it
meets.  The survey fired five probes down from `near + 1.2` … `+30` and kept the hit
closest to the model's road — but where two ribbons overlap in plan, which is every
junction of the 107,764-way network (82,933 junction candidates), *all five probes
report the upper ribbon*.  The "disagreement" was the instrument looking at the top
layer.  `rawGroundAt` now **peels**: it casts down from the top of the band, records
the surface, restarts the ray just below it, and repeats up to five times, so the
surfaces in the band are enumerated top-down and the candidate closest to the model's
road wins — which is the one a wheel rests on.  The headroom ray peels upward for the
same reason (a ray fired from just above a lane can start *inside* the upper ribbon,
and Jolt reports that as a hit at fraction 0).

Measured after: `within 0.25 m 91.3 % → 99.3 %`, within 0.5 m 99.3 %, within 2 m
99.4 %, worst 2.97 m, and the signed histogram is **two bins and nothing else**
(−0.12 m: 28.4 %, +0.12 m: 70.8 %), **mean +0.060 m → +0.007 m**.  The tail above
0.25 m fell from 68,607 to **906** samples.  The three descriptions of one street
were, in fact, already agreeing: the "7.8 % of lane points put 0.3–1.2 m below the
thing they are standing on" of D-037's own entry was the instrument's view through the
top layer of a junction.

**4. The one real consistency defect, fixed by construction.**  The ribbon drapes the
DEM and then smooths the profile over two 1-2-1 passes — deliberately, so a road does
not inherit the heightfield's noise as visible steps — while the navigation graph took
the raw sample of the same terrain.  The two could therefore never agree except by
accident.  The drape and the smoothing now live in one function
(`drapeRoadHeights`) and the graph samples that profile **by position**
(`heightAlong`: the mesh chain `points` and the navigation chain `navPoints` are
different lists — `navPoints` carries a vertex at every at-grade crossing), so the
nodes sit on the surface the ribbon draws.  This alone moved the survey by 0.1 %
(91.4 % → 91.3 %): it is a consistency fix, and its value is that the two surfaces
can no longer drift, not that it changed today's numbers.

**5. What replaced the tail is a real defect, and it is named.**  With the ground
measured correctly, the *headroom* counter now sees the other half of the same
geometry: **8.96 % of lane points (76,546 of the 85,596 with any surface above them)
have a surface within a car's 2.6 m**, minimum 0.05 m.  Strip away the ray artefacts —
the upward peel removed the fraction-0 hits — and what remains is genuine: at a
junction between two synthesised streets, **both ribbons are colliders and they are
stacked 0.05–1.5 m apart**, because each way smooths its own profile and the two
smoothings do not agree at the point they cross.  (The survey's own footprint-step
counter sees the same thing: 4.09 % of lane footprints step by more than 0.35 m.)
A car that changes direction at such a junction meets a step it cannot see.

The fix is designed and not yet made: the smoothing must be a function of
**position**, not of each way — one smoothed drape field that every way samples —
after which two roads crossing at a point have the same height there by construction.
That is the next item in 4b.

**What is unchanged, and still green.**  `make physics` **56 checks / 0 failed**;
`make test` 58 cases / 1529 checks / 0; the 60-minute S7 run **PASS** (572 vehicles,
mean 28.2 km/h, frame p99 1.74 ms); `verify_export` 396 / 0; the nav-node-in-building
measurement 0.01 % (11 of 124,348) and the lane-sample measurement 0.020 % (999 of
5,070,900, all 56 surveyed footprints) unchanged by this work.

**Files.**  `pipeline/stages/network.cpp` (`drapeRoadHeights`, `heightAlong`, both
callers), `engine/physics/driver.{h,cpp}` (the signed histogram, the layer
attribution, the downward and upward peels), `engine/app/phys_main.cpp` (the report).

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
