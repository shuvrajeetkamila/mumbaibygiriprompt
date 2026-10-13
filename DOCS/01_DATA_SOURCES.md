# 01 — Data sources, provenance and gaps

Everything the world is built from is either a **raw dataset** in `data/raw/`
(re-fetchable, hash-verified) or **authored data** in `data/authored/` (small,
reviewable, source-cited). Nothing is generated from an unstated assumption.

`bash tools/fetch_data.sh` re-downloads all four raw datasets and verifies
`data/raw/SHA256SUMS`; `--verify-only` checks the local copies without network
access. `make all-checks` runs the verification.

## Raw datasets

| file | bytes | sha256 (prefix) | source | licence |
|---|---|---|---|---|
| `mumbai_region.osm` | 35,312,441 | `54f4f6bc…` | `valmikkpatel/wrangle-osm-data`, `sample.osm` | ODbL (OpenStreetMap) |
| `bmc_wards.geojson` | 994,699 | `f8472efd…` | `datameet/Municipal_Spatial_Data`, `Mumbai/BMC_Wards.geojson` | as published by DataMeet |
| `slum_clusters.geojson` | 3,373,500 | `348851ec…` | same repository, `Mumbai/slumClusters.geojson` | as published by DataMeet |
| `mumbai_sra_2017.geojson` | 7,680,732 | `9d20dd90…` | `jkkersey/Mumbai-Mapping-Data`, `Mumbai_SRA_2017.geojson` | CC |

Fetched with `gh api -H "Accept: application/vnd.github.raw"`; `raw.github
usercontent.com` is unreachable from the build environment, the API is not.

### `mumbai_region.osm` — the tag inventory of the city

* 170,260 nodes, 20,959 ways, 402 relations; coverage 18.4661–19.5000°N,
  72.7240–73.5159°E (114.8 km × 83.2 km — the city plus its region).
* Vintage: the extract's most recent changeset is 32441939 (2015-07-06). It is
  a decade old; new construction after 2015 is absent, and the file is used
  accordingly (as the mapped-and-tagged baseline, not as a current survey).
* 16,408 `building` ways (16,169 tagged `yes`), 3,277 `highway` ways,
  232 `landuse`, 148 `railway`, 91 `natural`, 84 `leisure`, 76 `waterway`,
  401 `oneway`, 132 `lanes`, 62 `surface`, 168 `bridge`, 151 `layer`,
  17,501 closed ways (candidate footprints).
* Place names present: Bandra, Thane, Borivali, Andheri, Juhu, Colaba,
  Dharavi, Worli, Powai. **Not present:** Marine Drive, Gateway of India,
  Chhatrapati Shivaji Maharaj Terminus, the Bandra–Worli Sea Link (only a
  320 m fragment of way 48803910 survives).
* Consequences: the landmark set is authored (D-022), the corridors are
  authored data (D-021), and the fabric is completed by the synthesis stages
  with every generated element tagged as such (D-020).

### `bmc_wards.geojson` — the administrative frame

24 `MultiPolygon` features with properties `{gid, name}`; `name` is the ward
code (A, B, C, D, E, F/N, F/S, G/N, G/S, H/E, H/W, K/E, K/W, L, M/E, M/W,
P/N, P/S, R/C, R/N, R/S, S, T, and the extended wards in the file).

This layer defines the **world extent** (the polygon union, snapped to the
1024 m macro grid: 22,528 m × 43,008 m = 968.9 km², 924 cells) and drives the
zoning field together with elevation and distance-to-airport rules (D-008).

### `slum_clusters.geojson` + `mumbai_sra_2017.geojson` — informal settlement

Both layers hold 2,542 features whose only property is `polyno`; the geometry
*is* the data. They are unioned into a single informal-settlement mask (8 m
raster) that (a) tags zoning as `InformalSettlement`, (b) restricts the
informal grammar in `buildBuildings`, and (c) bounds the slum-lane synthesis.

Accuracy note: these are 2017-era polygons of *clusters*, not building-level
survey. Dharavi's internal fabric is generated inside the mask by the
deterministic synthesis stage and is labelled synthetic in the QA report — the
specification's "dense, organic, labyrinthine — but accurate, not caricature"
is met at the level of the mask plus a documented grammar, not by survey.

## Authored data

| file | contents | why it exists |
|---|---|---|
| `data/authored/terrain_control_points.json` | 43 published approximate elevations (Malabar Hill, Antop Hill, Powai basin, the reclamation fringes, Mumbra hills, creek edges), each with a source note; four entries carry `flat: true` and are shaped as depressions — Hindmata (a former tank bed at 2.2 m), King's Circle (3.0 m) and the Andheri (3.5 m) and Milan (3.2 m) subway trenches | no obtainable DEM (D-006); the depressions are known to the metre and are the sites the specification requires the flood model to reproduce (D-026) |
| `data/authored/transport_corridors.json` | the named highway/rail corridors as polylines with class, name and provenance | truncated or absent in the extract (D-021) |

Both files are small, diffable, and every entry states where its numbers came
from. Where a corridor matches an extract way, the extract wins and the
authored polyline is used only to close the gap.

## Missing sources, stated plainly

| source the specification names | status | effect |
|---|---|---|
| Cartosat-1 1 m DEM | not obtainable (licence-gated; no reachable service) | terrain is synthesised from control points (D-006); the ±1 m Cartosat comparison cannot be met from this source |
| SRTM 30 m gap-fill | not reachable | as above |
| MCGM LiDAR | not obtainable | as above |
| OSM `.pbf` | not available as XML is; `osmpbf` excluded (D-014) | XML scanner (D-005) |
| Aerial/street imagery for hero assets | not reachable | hero assets are authored from published dimensions (D-022) |

Every one of these is a **data** limitation with a defined replacement point:
`--dem <raster>` for terrain, the `osm::Database` interface for OSM, and the
`BuildingProxy`/hero builders for geometry. None of them requires reworking a
downstream system.
