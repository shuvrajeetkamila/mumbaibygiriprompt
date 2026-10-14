# 05 — Validation

Everything below is reproducible in one command:

```bash
make all-checks
```

which runs the unit suite, the 60-minute runtime validation, the independent
payload verifier, the viewer-shader verifier, the artifact hash check and the
raw-data hash check. The numbers quoted here are from the last full run (the
`all-checks` log of 2026-10-14, after D-032).

## The six verification layers

### 1. Unit suite — `build/bin/mumbai_tests`

58 cases, 1,529 checks, **0 failing**. It brings its own harness (D-014) and
covers the parts a wrong answer stays quiet about:

* the projection frame (0.001° latitude = 110.70 m, longitude = 105.28 m at
  19°N — an ellipsoidal result, so a spherical approximation fails);
* **the acceptance criterion for precision**: a 1 mm vertex pair at the four
  corners and the centre of the 22.5 × 43.0 km extent stays 1 mm apart through
  f32 render space (tolerance 2 × 10⁻⁴ m);
* 20 floating-origin boundary shifts for exactness;
* the chunk contract: every struct size, the round trip, and corruption
  detection (the test flips a byte at `section.offset + section.size/2` — a
  flip inside alignment padding would prove nothing);
* winding, polygon-with-holes behaviour, JSON;
* **the material library as data (D-031)**: the `.mumbaitex` arrays are mapped by
  the *runtime* reader, every layer's content hash is recomputed, a truncated
  copy and a single flipped payload byte are both refused, and each layer's mean
  linear albedo is compared against the albedo the manifest publishes for that
  material (worst 0.57 % over 115 layers) — the mean-preservation rule, held to
  the bytes on disk rather than to the generator's intent. Normal maps are
  checked for unit length and for actually carrying tilt;
* the monsoon hydrology (12 cases): priority flooding puts every interior
  minimum at its spill level; watersheds partition the terrain; a depression
  ponds instead of draining; rainfall + tide flood closes against drainage +
  outfall + stored film to 10⁻⁹; a spring tide is worse than a neap one for the
  same rain; **a slope sheds its rain instead of holding it** (the case that was
  failing when the routing was throttled by its own time step); **a paved street
  carries water that the same ground holds when it is rough** (measured on the
  street row: 0.042 m of flowing water against 0.035 m standing, and 8,571 m³
  left on the shelf against 9,397 m³); and **the sea wall holds the open coast
  while the creek mouths admit the tide** — the boundary condition that keeps the
  model from drowning the city twice a day.

### 2. Runtime validation — `build/bin/mumbai_sim`

A 60-minute, 60 Hz drive from Colaba to Borivali over the *real* world in
`data/processed`, driving the shipped traffic layer (`engine/sim/`, D-030) on the
city's own nav graph — 119 chunks, 31,345 nodes, 85,956 edges, 302 signal phases
— with a 620-vehicle fleet:

| measurement | value |
|---|---|
| simulated time | 60 min, 216,000 ticks |
| vehicle steps | 123,552,000 |
| vehicles live | **572** (620 requested; 48 could not be placed at legal spacing) |
| mean speed | 28.2 km/h (1 % at a crawl) |
| **vehicles within 500 m of the camera** | median **451**, peak **547** over 360 samples |
| minimum gap held | **2.55 m** against a 0.50 m floor; 0 lane violations in 216,000 audits |
| macro-cell crossings | **105** |
| worst shift cost | 0.0003 ms (budget: 1 ms) |
| worst camera-relative error across a shift | **0.00e+00 m** |
| chunk loads / unloads | 256 / 364 |
| hash failures | **0** |
| frame cost (solver + floating origin, CPU) | mean 0.33 ms, p50 0.08 ms, p99 3.64 ms, worst 9.27 ms |
| streaming cost (separate, off-thread in the runtime) | 0.10 ms mean over 7,200 streaming frames, worst 14.09 ms |
| traffic behaviour | 28,393 lane changes, 28,393 rollover-limited manoeuvres, 4,087,290 Boids updates, 94,530 separation events, 18,833 turnarounds |
| signals | 301 stops at a line on red, 708 cleared on green, 72,825 turned away from a full junction, 0 held at a dead end |
| population device | 14,587 re-entry attempts found no room (0.8 % of 1.8 M edge completions), 0 vehicles stranded |
| junction holds stopped at the box limit | 2,710 of 72,825 (the one place the floor can be transiently compressed; no audit found a violation) |
| invalid states | 0 non-finite, 0 out-of-envelope speeds, 0 out-of-extent positions |
| verdict | **PASS** (14 of 14 checks) |

The harness holds the specification's fleet *around the camera* — a scenario
device, not a behaviour — and reports what it cannot do: on the Sea Link deck the
500 m disc contains a handful of long lane-segments and the density falls to zero,
which is why the run reports the distribution and asserts the peak (D-030).

### 3. Independent payload verifier — `tests/verify_export.mjs`

A Node verifier that re-derives everything from the offsets in
`viewer/data/mumbai.json` and never trusts the pipeline's own counters:
**396 checks, 0 failing**.  Since D-031 it also re-inflates the material atlas
(its own PNG decode, `node:zlib` for the stream), walks every textured
material's tile and confirms the `tileMean` the shader divides by against the
pixels it will sample — worst error 0.0002 — and refuses two materials that name
the same tile. Measured: terrain 705 × 1345 at 32 m, 47.2 % land,
y −13.5 … 91.4 m; 164,057 building records; 126.7 km² of footprint; heights
p50/p90/p99 = 12/22/41 m; **124,348 nodes / 167,145 edges** — the street graph
clipped to the extent (2,550 nodes and 2,556 edges that lie outside the box are
dropped), 100 % of the survivors reachable, median edge 29.7 m, longest 169 m,
reaching x 22,527 of 22,528 m and z 43,005 of 43,008 m; hero 386,320 vertices /
**193,330 triangles**; the Sea Link deck 59,792 vertices across a 3.88 km
corridor span; every vertex of a point-landmark material within 400 m of its
anchor (stone 54 m, terracotta 48 m, glass 82 m) and every vertex of a corridor
material (concrete, paving) within 2.5 km of the corridor's published anchor.

The verifier has earned its place three times.  An earlier version added the
extent origin twice and produced nonsense.  It caught the payload/console QA-count
divergence (D-019).  And it caught the payload's street graph being wired to the
wrong array: the export wrote `nodeBase + remap[...]` where `remap` already held
a global index, so every chunk after the first sent its edges to node indices
that do not exist — 82,770 of 167,145, up to 248,518 for a 124,348-node array,
50.5 % of the graph reachable, and none of it visible to anything that only reads
the payload one record at a time, which is what the viewer does.

### 3b. Viewer shader verifier — `tests/verify_viewer_shaders.mjs`

**31 checks, 0 failing.**  The browser preview applies the atlas by patching
three.js's `MeshStandardMaterial` with string substitution — the one viewer
mechanism that fails *silently* when a chunk is renamed, because `String.replace`
on a missing anchor returns the shader unchanged and the city simply renders
without textures.  The verifier reads the injections out of `viewer/index.html`
(read, not duplicated), applies them to the vendored three.js shader, and asserts
that the anchors exist, that `color_fragment` still runs before
`normal_fragment_begin` (the injection reads `vNormal` otherwise, which is
view-space and makes the projection plane depend on the camera), that the brace
balance is unchanged, that every varying is declared on both sides, that the
geometry supplies the attributes the shader reads, and that the payload
publishes `tex` / `tileMetres` / `tileMean` for every textured material.

### 3c. Physics acceptance — `build/bin/mumbai_phys`

**45 checks, 0 failing**, run by `make physics` (the full configuration:
`--minutes 1 --seconds 30 --props 200 --fleet 24`) and part of `all-checks`.
The report is `build/physics_report.json`; every number below is read from it,
and the solver configuration is printed by the harness itself so a claim about
penetration is a claim about a slop:

```
Jolt 5.6.0 (solver 10 velocity / 2 position steps, slop 5 mm, linear cast at 0.25 r, speculative 20 mm)
```

| group | check | measured |
|---|---|---|
| bake | every cell bakes | 924 cells in 0.14 s — 164,594 boxes, 24 capsules, 924 heightfields, 526 compounds, 1,450 static bodies |
| coverage | every probe across the built extent finds ground | 3,696 / 3,696, surface −13.5 … 103.4 m |
| Sea Link | the deck is recovered from the collision records | 207 slabs, 3,987 m, y 8.93 … 25.76 m, 18.0 m wide, heading −154.9° |
| Sea Link | every slab's collider matches its collision record | top face 0.340 m above the record's centre, **worst deviation 0.00013 m** over 207 slabs; 8 slabs obscured by the bridge's own pylon and stays, 0 unprobeable |
| ride | a motorcycle rides the carriageway | 343 m in 30 s, 41.2 km/h mean, **100 % wheels down**, peak lean 6.4°, worst 1.0 m off the chain, 0.0000 m penetration |
| ride | the lean controller leans and returns | slalom peak 16.1° → 6.1° at the end, 210 m covered |
| ride | car / taxi / BEST double-decker / auto-rickshaw | 49.2 / 46.2 / 31.0 / 30.2 km/h, peak roll 1.6 / 1.4 / 0.9 / 4.0°, 98–99 % wheels down, 0.0000 m penetration on all four |
| braking | a car brakes in a plausible distance | 32.1 m from 60 km/h (analytic 15.7 m at μ = 0.9, the gap being the vehicle model's brake torque and the deck's grade) |
| impact | 300 kg dropped 22 m onto the deck | meets it at 19.4 m/s, settles in 2.75 s, **lowest corner 27 mm from the surface**, 0.0 mm penetration at rest, no tunnelling, one-tick transient of 0.2484 m (Jolt's `mLinearCastMaxPenetration`, published) |
| fleet | 500 vehicles fit a 30 Hz frame | 24 bodies at 17.8 µs each = **8.9 ms** extrapolated (budget 33 ms) |
| floating origin | 20 crossings of 1,454 bodies | mean 0.248 ms, **worst 0.308 ms** (criterion < 1 ms), 0.00e+00 m write-back error, no velocity impulse |
| soak | one simulated minute, 200 props | 3,600 steps, mean 0.745 ms, p99 0.917, worst 2.521 ms; deepest penetration **5.2 mm** (the slop) |
| authoring | every prop sits on the surface under its own footprint | 200 crates, worst authored clearance **+0.020 m** — the harness's own overlap (up to 20 cm before the fix) is attributed to the harness, not the solver |

This layer is what caught five defects the world build could not see — both
carriageways of the Sea Link on one line, the slab pitch, a constraint double
free, the double-decker's shift point above its redline, and Jolt's
left-positive steering input (D-033).  It is the first verification layer that
exercises the *engine*, not the pipeline.

### 4. Artifact integrity — `tools/verify_artifacts.sh`

`viewer/data/SHA256SUMS` pins the published payload (the build is
byte-reproducible, so the hashes are stable across rebuilds) and the verifier
fails loudly if the bytes on disk are not the bytes that were published:
`mumbai.bin` sha256 `62793556…` (23,523,726 B), `mumbai.json` sha256 `a88493ab…`
(37,613 B), and `atlas_basecolor.png` sha256 `dbf81ed5…` (1,704,817 B — the
11x11 material atlas the viewer fetches) pinned by the same script.  The texture
arrays in `data/processed/textures/` are *inputs* to the payload, not artifacts
of the browser build, so they are covered by the manifest byte counts the
pipeline reports (`textures.arrayBytes`) and by the layer hashes inside each
`.mumbaitex` header, which the unit suite recomputes.

This layer exists because of a real incident (D-025): a workspace restore
round-tripped `mumbai.bin` through UTF-8 decoding with `errors=replace`, and the
file came back 3.25 MB larger, made of U+FFFD sequences. The structural
verifier caught it (57 of 73 checks failing, terrain "63.6 % land", building
footprints of 177 million km²) and the two checks added since name the failure
directly:

```
FAIL payload length equals the declared layout  — 11611201 on disk vs 8359144 declared
FAIL payload contains no UTF-8 replacement sequences  — first at byte 1392
```

Two minutes of `make world` restored the exact original bytes (verified against
the pinned hashes).

### 5. Source integrity — `tools/fetch_data.sh --verify-only`

46 MB of raw data, four files, SHA-256 verified on every `all-checks` run.

## Acceptance matrix (specification Section 7)

| # | criterion | status | evidence / reason |
|---|---|---|---|
| 1 | terrain within 1 m vertical of Cartosat | **data-limited** | no Cartosat DEM obtainable (D-006); the synthesis is anchored on 43 published control points, `--dem` swaps in a real raster |
| 2 | roads within 2 m horizontal of OSM | met for surveyed ways | centrelines are used verbatim; resampling only inserts vertices. Authored corridors are published alignments (D-021) |
| 3 | landmark detail / material content | partly met | **textures are no longer the gap** (D-031): 115 materials with albedo, normal and ORM maps at a physical tile size, mean-preserving, verified from the shipped bytes; the remaining gap at this row is hero *geometry*, below |
| 3b | landmarks recognisable at 100 m, unlabelled | partly met | published dimensions plus the identifying ornament at the survey tier (D-032): coursed ashlar cut around the openings, voussoir arches with the barrel behind them, jali lattices, gallery balustrades, pinnacles, deck rails/kerbs/stiffeners, the Art Deco row. 193,330 triangles across five assets (Gateway 19,126, Taj 84,556, CSMT 30,628, Sea Link 34,476, Marine Drive 41,288). The specification's budgets (500k / 800k / 1.2M / 300k per pylon / 50k per building) are hand-modelled asset budgets and are **2.39 % met** — published per asset (`hero.<name>.triangles` beside `hero.<name>.triangleBudget`), not concealed |
| 4 | zero vertex jitter at 4 corners + centre | **met** | unit test (1 mm survives, tol 2e-4) and 105 live crossings at 0.00e+00 m |
| 5 | ≥500 vehicles in a 500 m radius at 30 FPS | CPU-side met | 547 peak / 451 median within 500 m of the moving camera over 60 min; solver + origin CPU p99 3.64 ms against a 16.6 ms budget, with chunk I/O timed apart. Jolt separately puts a 24-vehicle physical fleet at 17.8 µs per vehicle per step = **8.9 ms extrapolated to 500** (D-033). The GPU frame is not measurable here |
| 6 | weather and ocean valid | partly met | **monsoon hydrology is built and verified** (D-026): two design storms on a 705 × 1345 hydrology grid, mass balance closed to 10⁻¹⁰ relative, published peak-depth rasters for both storms, and the specification's own sites (Hindmata 274 mm, King's Circle 280 mm, Andheri subway 408 mm in the 2005 analogue; Marine Drive dry in both) flooding as an emergent consequence of the DEM. GPU rain, wetness shading, Gerstner ocean and scattering remain specified-not-built in `04_RUNTIME.md` |
| 7 | 30 FPS worst case (monsoon + traffic + South Bombay) | **not verifiable here** | needs a device; no GPU in the build environment |
| 8 | ≤160 GB installed | met at this tier | 708 MB of chunks for 968.9 km²; scales with hero density and texture budget, both of which are currently far below the specification's |
| 9 | 60-minute continuous run, zero crashes | **met** | `mumbai_sim` driving `engine/sim/` over the city graph, 216,000 ticks, exit 0, 14/14 checks, 0 penetrations, 0 lane violations; and `mumbai_phys` steps a 1,650-body world for a simulated minute with the step at p99 0.917 ms (D-033) |

## What this proves, and what it does not

**Proved.** The monsoon flood model conserves mass exactly and puts water where
the city's low ground is; its named sites flood in the order residents would
name them, without being told where they are. The world format is sound (mmap, typed views, per-section hashes,
corruption detected); the precision design survives the acceptance test for
coordinate stability, including the floating origin under repeated crossings;
the produced world is internally consistent and can be read back independently;
the traffic layer holds a 620-vehicle peak-hour population for an hour without
a single invalid state or interpenetration; the build is reproducible from
hash-verified sources.

**Not proved, and not claimed.** Anything requiring a GPU or a device: frame
rate, rendering artifacts, TAA/WBOIT behaviour, the weather and ocean shaders,
Android installation size, and the actual visual appearance of the world.
Physics *is* now exercised (layer 3c) but only on this machine's CPU, with the
harness's own driver rather than the traffic layer's, and it is not a device
frame time. Those are specified in `04_RUNTIME.md` and gated on
hardware this environment does not have (D-001).

## Known gaps, in priority order

1. **Hero detail** is 2.39 % of the specification's triangle budgets (D-032).
   The assets are modelled to their published dimensions and ornament (coursed
   ashlar cut around the openings, voussoir arches with the barrel behind them,
   jali lattices, gallery balustrades, deck rails/kerbs/stiffeners, the Art Deco
   row), and every build now audits the hero meshes for non-finite coordinates,
   out-of-range indices and zero-area triangles — the last full run reports
   **0 of each** across 193,330 triangles. What a procedural build cannot supply
   is the photogrammetry-scale triangle density the budgets describe; that needs
   hand-modelled or scanned assets, not more ornament.
2. **Weather and ocean** are designed, not built.
3. **The traffic layer is closed** (D-030): the 60-minute run above drives the
   shipped `engine/sim/` over the city's own graph — 28,393 lane changes through
   MOBIL, 4.09 M two-wheeler boids updates, 302 signal phases obeyed.
4. **Terrain accuracy** is bounded by the authored control points until a real
   DEM is supplied (D-006).
5. **Flood depths are uncalibrated and the conveyance is parameterised.** The
   DEM is synthesised from published control points (D-006), so the site depths
   in D-026 are the shape of the event, not a survey, and at 32 m a diffusive
   sheet stands in for a pipe network that in reality moves much more water per
   unit width. What the model does claim, and what the suite checks, is *where*
   and *in what order* Mumbai floods: the low ground floods, the hills do not,
   and the named sites fill and empty in the right sequence. Calibration would
   need gauge records and a real DEM via `--dem`. The GPU weather stack (rain,
   wetness, ocean) is still design.
6. **Building footprint counts (164k)** are dominated by synthetic parcels
   (163,909 synthetic against 148 OSM structures); the surveyed/synthetic tags
   keep them separable, but the real building count for Mumbai is ~200k+ and
   the extract only carries 16,408 building ways.
