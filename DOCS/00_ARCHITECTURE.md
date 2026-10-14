# 00 — Architecture

## What this is

A 1:1 geospatial replica of Mumbai: Colaba to Borivali/Dahisar, the Arabian Sea
to Thane Creek. The build in this repository covers the three layers that
everything else depends on:

1. **an offline pipeline** that ingests real source data and produces the world,
2. **a world format** (`.mumbai_chunk`) designed for `mmap`-backed streaming at
   runtime, with per-section integrity hashes,
3. **a runtime core** that reads that format: projection and floating-origin
   maths, the chunk reader, the nav graph, and a traffic layer validated against
   the real world over 60 simulated minutes.

On top of those sit the device-side systems the specification describes
(Vulkan renderer, physics, weather, ocean, ECS gameplay). Those layers are
specified in `04_RUNTIME.md` and mapped to the interfaces built here; they
cannot be executed in this environment (see D-001 in `02_DECISIONS.md`).

## The three layers

```
   data/raw/                      data/authored/                 configs/world.json
   ├── mumbai_region.osm          ├── terrain_control_points.json
   ├── bmc_wards.geojson          └── transport_corridors.json
   ├── slum_clusters.geojson
   └── mumbai_sra_2017.geojson
          │                                │                            │
          └──────────────┬─────────────────┴────────────────────────────┘
                         ▼
        ┌────────────────────────────────────────────────────────────┐
        │  mumbai_pipeline — 13 stages (pipeline/stages/*.cpp)       │
        │                                                            │
        │  loadSources → buildTerrain → buildZoning →                │
        │  emitTerrainMeshes → buildWater → buildRoads →             │
        │  buildRailAndLanduse → buildBuildings → buildProps →       │
        │  buildHeroAssets → writeChunks → writeManifest →           │
        │  exportViewer → finaliseQa                                 │
        └────────────────────────────────────────────────────────────┘
                         │                              │
          .mumbai_chunk  │                              │  viewer payload
          (924 files, 686 MB)                           │  (mumbai.bin/json)
                         ▼                              ▼
        ┌──────────────────────────────┐     ┌────────────────────────┐
        │  runtime core                │     │  viewer/index.html     │
        │  engine/core  (math, geo,    │     │  three.js r169         │
        │   json, floating origin)     │     │  (vendored, no CDN)    │
        │  engine/world (chunk.h/.cpp) │     └────────────────────────┘
        │  engine/app   (mumbai_sim)   │
        └──────────────────────────────┘
```

## Repository map

| path | what lives there |
|---|---|
| `engine/core/` | `core.h/.cpp` (logging, hashing, RNG, timing, files, CLI), `math.h/.cpp` (SIMD-ready vector/matrix/geometry maths, Vulkan clip conventions), `geo.h/.cpp` (projection, extents, macro cells, `GlobalPosition`, floating origin, taxonomies), `json.h/.cpp` (the project's JSON reader/writer) |
| `engine/world/` | `chunk.h/.cpp` — the `.mumbai_chunk` contract: header, 19 section types, writer, reader, typed views, hash verification. `texture.h/.cpp` — `.mumbaitex`, the mmap-able material texture array, shared with the offline writer |
| `engine/app/` | `sim_main.cpp` — `mumbai_sim`, the headless runtime-validation harness |
| `pipeline/mgeo/` | geometry primitives, OSM XML scanner, DEM synthesis, mesh builder, GeoJSON reader, deterministic synthesis helpers |
| `pipeline/stages/` | the 13 build stages, one concern per file |
| `pipeline/pipeline.h` | `BuildConfig`, `MaterialLibrary`, `ZoningField`, `CellData`, `Context`, `QaReport`, stage declarations |
| `configs/world.json` | the canonical build configuration (extent, budgets, LOD radii, toggles) |
| `data/authored/` | hand-authored, source-cited inputs (control points, corridors) |
| `tests/` | the C++ test suite, and `verify_export.mjs` — an independent verifier for the viewer payload |
| `viewer/` | the three.js viewer, its vendored library, and the published payload |
| `DOCS/` | this documentation set |

## Coordinate stack

```
   WGS84 lat/lon (source data, f64)
        │  TransverseMercator:  λ₀ = 72.9°E, φ₀ = 18.9°N, k₀ = 1, WGS84 ellipsoid
        ▼
   world metres, f64, origin at (φ₀, λ₀); x east, z north
        │  MACRO_CELL_SIZE = 1024 m — cell index = floor(world / 1024)
        ▼
   GlobalPosition { i64 cellX, i64 cellZ, f32 localOffset ≤ 1024 m }
        │  toRenderSpace(cameraCell): (cell − cameraCell)·1024 + local, computed in f64
        ▼
   render space, f32, relative to the camera's macro cell (±4096 m at LOD radius)
```

Three precision tiers, why they exist and how they are verified: D-004.
The origin-shift path (`FloatingOriginManager::update`, `applyShift`) runs
between the physics tick and the render submit; `mumbai_sim` measures the cost
and the invariant (no pop) on every crossing.

## World extent and scale

| quantity | value |
|---|---|
| extent | 22,528 m (x) × 43,008 m (z) = **968.9 km²** |
| macro cells | **924** of 1024 m |
| chunks on disk | 924 files, **686,383,728 B** |
| terrain | 705 × 1345 samples at 32 m, elevations −13.5 m … +91.4 m |
| buildings | **164,057** massings, 164,104 collision boxes |
| road network | 107,764 OSM ways, **8,189 km** of centreline, 82,933 junctions |
| nav graph | **254,470 nodes / 127,235 edges** |
| rail | 29 ways |
| instances (props) | 128,220 (tanks, AC units, lamps, trees, two-wheelers) |
| triangles | 7,563,429 across all built meshes |
| hero assets | 5 (Gateway of India, Taj Mahal Palace, CSMT, Bandra–Worli Sea Link, Marine Drive), 15,676 vertices |
| materials | 116 named PBR materials |

## Data flow, in one paragraph

`loadSources` reads the four raw datasets plus the two authored files, builds
the world extent from the ward polygons, and compiles the zoning field.
`buildTerrain` synthesises the heightfield from the authored control points and
refines it against the coastline and the drainage pass. `emitTerrainMeshes`
tessellates it per cell into LOD0/1/2 meshes plus an impostor quad, with
collision geometry, AO and material masks. `buildWater` emits the sea grid,
inland water and river/canal ribbons. `buildRoads` extrudes every classified
way with a per-class cross-section (carriageway, kerbs, sidewalks, medians),
trims junctions, and emits the nav graph. `buildRailAndLanduse` adds ballast,
sleepers and rail steel, and the landuse polygons. `buildBuildings` applies the
zone grammar to every footprint (plus parcels), producing LOD chain, collision
and AO. `buildProps` scatters rooftop tanks, AC units, cable tangles,
streetlamps along the nav graph and trees in green polygons. `buildHeroAssets`
authors the five landmarks. `writeChunks` serialises each cell, hashes every
section and re-opens each file to verify. `writeManifest` writes the machine
readable build record. `exportViewer` re-reads every chunk through the runtime
reader and packs the viewer payload. `finaliseQa` publishes the finished QA
report to the manifest and the payload.

## Acceptance mapping (specification Section 7)

| criterion | where it lives | status |
|---|---|---|
| terrain 1 m vertical vs Cartosat | `dem.cpp` synthesis, control points | **data-limited** — no Cartosat source available (D-006) |
| roads 2 m horizontal vs OSM | `osm.cpp` centrelines are used verbatim; `network.cpp` extrudes them | met for surveyed ways; authored corridors are public alignments |
| landmarks recognisable at 100 m, unlabelled | `hero.cpp` + viewer | **partly met** — massings are correct, detail budgets are far below the specification's |
| zero vertex jitter at 4 corners + centre | `test_geo.cpp`, `mumbai_sim` shift audit | **met** (worst error 0.00e+00 m, 105 crossings) |
| ≥500 vehicles in 500 m, 30 FPS | `mumbai_sim` (545 peak within 500 m; CPU p99 0.14 ms) | **CPU-side met**; GPU/device frame not verifiable here |
| weather / ocean valid | designed in `04_RUNTIME.md`, not implemented | **not started** |
| ≤160 GB installed | 686 MB of chunks for 968.9 km²; full-density estimate in `05_VALIDATION.md` | **met at the current quality tier**, scales with hero detail |
| 60-minute Colaba → Borivali, zero crashes | `mumbai_sim` | **met** |

## How to run it

```bash
make                 # build pipeline, harness and tests
make all-checks      # tests + 60-minute runtime validation + payload verifier + data hashes
make world           # rebuild the world from data/raw into data/processed (~15 s)
make viewer          # verify (or refresh) the vendored three.js
make serve           # serve viewer/ over HTTP for the three.js scene
make verify-artifacts # payload checksums (also runs inside all-checks)
```
