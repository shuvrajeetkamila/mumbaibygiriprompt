# Mumbai 1:1 EXACT

A geospatial replica of Mumbai — Colaba to Borivali/Dahisar, the Arabian Sea to
Thane Creek — built from real source data, stored in a world format designed
for `mmap`-backed streaming, and validated against the acceptance criteria of
the project's master build prompt.

This repository contains the **offline pipeline**, the **world format**, the
**runtime core** and the **validation harness**. The Android/Vulkan layers the
specification targets are specified in `DOCS/04_RUNTIME.md` and mapped onto the
interfaces built here; they cannot be executed in this build environment (no
GPU, no Android toolchain, no graphics dependencies — see decision D-001 in
`DOCS/02_DECISIONS.md`).

## Status

| | |
|---|---|
| world | 968.9 km², 924 chunks, **708 MB** on disk (7.76 M triangles; 164,075 building collision boxes plus the hero and Sea Link colliders the manifest counts under `hero.*`, 164,594 in the physics bake) |
| terrain | 705 × 1345 samples at 32 m, −13.5 m … +91.4 m |
| buildings | **164,057** massings (148 surveyed + synthetic fabric, tagged separately) |
| roads | 107,764 OSM ways, **8,189 km** of centreline, 82,933 junctions |
| nav graph | **254,470 nodes / 127,235 edges** |
| hero assets | Gateway of India, Taj Mahal Palace, CSMT, Bandra–Worli Sea Link, Marine Drive — **193,330 triangles** at the survey tier (D-032; the specification's asset budgets are hand-modelled and are 2.4 % met, published per asset) |
| monsoon flood | two design storms on a 705 × 1345 hydrology grid; mass balance closed to 10⁻¹⁰ relative; Hindmata 274 mm / King's Circle 280 mm / Andheri subway 408 mm in the 2005 analogue, Marine Drive dry in both (D-026) |
| physics | Jolt v5.6.0 over the pipeline's own collision (D-033): **164,594 colliders + 24 capsules + 924 heightfields**, the Sea Link's 3,987 m carriageway recovered from the baked records, a motorcycle at 100 % wheels down on it, 500 vehicles at 9.1 ms/frame, the origin shift at 0.77 ms worst — **44 / 44 acceptance checks** (`make physics`) |
| QA | 196 counters, 0 warnings, 0 failures |
| tests | **58 cases / 1,529 checks, 0 failing**, plus 396 payload checks, 31 viewer-shader checks, the artifact hashes and the raw-data hashes |
| runtime validation | 60-minute Colaba → Borivali drive: PASS (105 origin crossings at 0.00e+00 m error, 0 hash failures, 547 vehicles within 500 m) |

## Quick start

```bash
bash tools/fetch_data.sh     # 46 MB of hash-verified source data
make                         # pipeline, runtime harness, tests
make world                   # build the world into data/processed  (~15 s)
make all-checks              # unit suite + 60-minute runtime run + payload verifier
                             #   + artifact checksums + source-data hashes
make serve                   # open the three.js viewer at http://localhost:8000/
```

## Layout

```
engine/core/     maths, projection, floating origin, JSON, logging, timing
engine/world/    .mumbai_chunk: header, 19 section types, mmap reader, hashes
engine/app/      mumbai_sim — headless runtime validation (streaming + traffic)
pipeline/mgeo/   OSM scanner, DEM synthesis, geometry, mesh builder, GeoJSON
pipeline/stages/ the 13 build stages
tests/           the C++ suite and the independent viewer-payload verifier
viewer/          single-file three.js scene + the published payload
DOCS/            architecture, data sources, decisions, pipeline, runtime, validation
docs/            the master build prompt as received (structured restatement)
```

## Reading order for a reviewer

0. The viewer preview: `make serve`, then open the page (it streams the real
   built Mumbai — terrain, 164,057 buildings, the street graph, five landmarks).
   Press **L** for the monsoon flood layer: the modelled peak water surface of
   the 26 July 2005 analogue and of a routine monsoon day, drawn at the depth the
   hydrology computed for every cell.
1. `DOCS/00_ARCHITECTURE.md` — what was built and how it fits together.
2. `DOCS/02_DECISIONS.md` — every decision, including the four deviations from
   the specification and why they were necessary.
3. `DOCS/05_VALIDATION.md` — what is proven, what is not, and the gap list.
4. `DOCS/01_DATA_SOURCES.md` — exactly which data the world is made of.
5. `DOCS/06_ROADMAP.md` — what is still missing, why, and what can be built
   without a GPU or a device.
