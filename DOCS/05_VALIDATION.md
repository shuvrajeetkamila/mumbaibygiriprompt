# 05 — Validation

Everything below is reproducible in one command:

```bash
make all-checks
```

which runs the unit suite, the 60-minute runtime validation, the independent
payload verifier and the raw-data hash check. The numbers quoted here are from
the last full run.

## The five verification layers

### 1. Unit suite — `build/bin/mumbai_tests`

37 cases, 477 checks, **0 failing**. It brings its own harness (D-014) and
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
`data/processed`, with a 620-vehicle fleet:

| measurement | value |
|---|---|
| simulated time | 60 min, 216,000 ticks |
| vehicle steps | 131,112,000 |
| vehicles live | **607** (620 requested; 13 could not be placed at legal spacing) |
| mean speed | 5.1 km/h (72 % at a crawl — a real peak hour) |
| **peak vehicles within 500 m of the camera** | **545** |
| minimum gap held | 0.50 m floor, never violated (26.5 M safety projections) |
| macro-cell crossings | **105** |
| worst shift cost | 0.0002 ms (budget: 1 ms) |
| worst camera-relative error across a shift | **0.00e+00 m** |
| chunk loads / unloads | 256 / 364 |
| hash failures | **0** |
| frame cost (streaming + traffic, CPU) | mean 0.07 ms, p50 0.07 ms, p99 0.12 ms, worst 7.53 ms |
| invalid states | 0 non-finite, 0 out-of-envelope speeds, 0 out-of-extent positions |
| verdict | **PASS** |

### 3. Independent payload verifier — `tests/verify_export.mjs`

A Node verifier that re-derives everything from the offsets in
`viewer/data/mumbai.json` and never trusts the pipeline's own counters:
28 checks, **0 failing**. Measured: terrain 705 × 1345 at 32 m, 47.2 % land,
y −13.5 … 91.4 m; 164,057 building records; 126.7 km² of footprint; heights
p50/p90/p99 = 12/22/41 m; 254,470 nodes / 127,235 edges; hero 29,232 vertices;
sea-link deck span 3.88 km; every vertex of a point-landmark material within
400 m of its anchor (stone 54 m, terracotta 48 m, glass 82 m).

The verifier has already earned its place twice: an earlier version added the
extent origin twice and produced nonsense, and it is the check that caught the
payload/console QA-count divergence (D-019).

### 4. Artifact integrity — `tools/verify_artifacts.sh`

`viewer/data/SHA256SUMS` pins the published payload (the build is
byte-reproducible, so the hashes are stable across rebuilds) and the verifier
fails loudly if the bytes on disk are not the bytes that were published:
`mumbai.bin` sha256 `5a2ec54f…`, `mumbai.json` sha256 `18fae06c…`, 8,359,144 B
and 22,938 B.

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
| 3 | landmarks recognisable at 100 m, unlabelled | partly met | correct massing and, after the detail pass, 14,736 triangles across five assets (Gateway 1,750, Taj 4,500, CSMT 1,762, Sea Link 6,324, Marine Drive 400). The specification's budgets (500k / 800k / 1.2M / 300k per pylon / 50k per building) are unmet — declared, not concealed (D-022) |
| 4 | zero vertex jitter at 4 corners + centre | **met** | unit test (1 mm survives, tol 2e-4) and 105 live crossings at 0.00e+00 m |
| 5 | ≥500 vehicles in a 500 m radius at 30 FPS | CPU-side met | 545 measured; frame CPU p99 0.12 ms. The GPU frame is not measurable here |
| 6 | weather and ocean valid | partly met | **monsoon hydrology is built and verified** (D-026): two design storms on a 705 × 1345 hydrology grid, mass balance closed to 10⁻¹⁰ relative, published peak-depth rasters for both storms, and the specification's own sites (Hindmata 289 mm, King's Circle 283 mm, Andheri subway 376 mm in the 2005 analogue; Marine Drive dry in both) flooding as an emergent consequence of the DEM. GPU rain, wetness shading, Gerstner ocean and scattering remain specified-not-built in `04_RUNTIME.md` |
| 7 | 30 FPS worst case (monsoon + traffic + South Bombay) | **not verifiable here** | needs a device; no GPU in the build environment |
| 8 | ≤160 GB installed | met at this tier | 686 MB of chunks for 968.9 km²; scales with hero density and texture budget, both of which are currently far below the specification's |
| 9 | 60-minute continuous run, zero crashes | **met** | `mumbai_sim`, 216,000 ticks, exit 0 |

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
Jolt physics behaviour, Android installation size, and the actual visual
appearance of the world. Those are specified in `04_RUNTIME.md` and gated on
hardware this environment does not have (D-001).

## Known gaps, in priority order

1. **Hero detail** is ~2 % of the specification's triangle budgets (D-022).
2. **Weather and ocean** are designed, not built.
3. **MOBIL lane changes and the two-wheeler flow field** are specified; the
   longitudinal layer they build on is implemented and validated (D-015).
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
