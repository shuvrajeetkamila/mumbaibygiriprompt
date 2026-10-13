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

## Open decisions (deliberately deferred)

* **Drainage-sentinel filtering.** The 19 published basins currently include
  open-sea minima (the largest entry is the sea floor at −13.5 m). The flood
  model must exclude cells that are permanently under the tide before it is
  wired to the weather system.
* **Hero budgets.** 500k / 800k / 1.2M / 300k-per-pylon / 50k-per-building are
  the specification's numbers; current assets are three orders of magnitude
  below them (accurate massings, honestly counted).
* **LOD0 terrain budget.** The specification's per-cell budget is met by
  construction, but the underwater half of many cells is tessellated at LOD0;
  a seabed-aware LOD split is planned.
