# 03 — The pipeline

`mumbai_pipeline` turns four raw datasets and two authored files into 924
`.mumbai_chunk` files plus a manifest, in about 15 seconds on two cores.

```
mumbai_pipeline --config configs/world.json
mumbai_pipeline --report data/processed/world_manifest.json     # print a finished build
```

## Stages

| # | stage | file | what it does | outputs |
|---|---|---|---|---|
| 1 | `loadSources` | `sources.cpp` | parses the OSM extract, the ward polygons and the two slum layers; builds the extent from the ward union snapped to 1024 m; compiles the zoning raster (100 m) and the 8 m slum mask; loads control points and corridors | `ctx.osm`, `ctx.extent`, `ctx.zoning`, `ctx.slumMask`, `ctx.materials` |
| 2 | `buildTerrain` | `sources.cpp` + `mgeo/dem.cpp` | inverse-distance synthesis from the 43 control points, coastal and ward priors, city-mask reapply, offshore deepening (`seaLevel − 1.5 − clamp(offshore × 0.02, 0, 12)`), then the drainage pass that finds basins | `ctx.terrain` (f32 grid), `ctx.lowPoints` (19 basins) |
| 3 | `buildZoning` | `sources.cpp` | assigns every 100 m cell a `ZoningZone` and audits the result | `ctx.zoning` |
| 4 | `emitTerrainMeshes` | `terrain_mesh.cpp` | tessellates the heightfield per cell into LOD0/1/2 + impostor with the quadtree, bakes AO, writes the material mask, emits collision geometry | `cd.terrain[3]`, `cd.terrainCollision`, `cd.terrainAO`, `cd.materialRegions` |
| 5 | `buildWater` | `sources.cpp` | the sea/creek quad grid at sea level (31,180 quads), inland water from `natural=water` triangulated with holes honoured (3 bodies), river/canal ribbons (818 segments) | `cd.water` |
| 6 | `buildRoads` | `network.cpp` | classifies every highway way, resamples it, profiles it (carriageway, kerbs, sidewalks, median), trims junctions, extrudes the carriageway/kerb/sidewalk/median meshes, and emits the nav graph | `cd.roads`, `cd.navNodes`, `cd.navEdges`, `cd.signals`, `cd.collision` |
| 7 | `buildRailAndLanduse` | `network.cpp` | ballast, sleepers and rail steel for 29 rail ways; landuse polygons and ground tint | `cd.rail`, `cd.landusePolys`, `cd.landuseVerts` |
| 8 | `buildBuildings` | `fabric.cpp` | zone grammar over every footprint: height from tags or ward rule, shops band, walls, roof/parapet, LOD1/2/3, collision box(es), AO | `cd.buildings[4]`, `cd.collision`, `cd.proxies` |
| 9 | `buildProps` | `fabric.cpp` | rooftop tanks and AC units, cable tangles, streetlamps along the nav graph, trees in green polygons, parked two-wheelers | `cd.instances` |
| 10 | `buildHeroAssets` | `hero.cpp` | Gateway of India, Taj Mahal Palace, CSMT, Bandra–Worli Sea Link, Marine Drive | `cd.hero` |
| 10b | `buildFloodModel` | `stages/flood.cpp` + `mgeo/hydro.cpp` | the monsoon flood model (D-026): conditions the DEM, builds the hydrology surface (sea, boundary, shore, tidal inlet, D8, accumulation, watershed, channels), rasterises coverage from the nav graph, building proxies and landuse, then runs the routine and the 2005-analogue design storms and publishes both peak-depth rasters | `ctx.floodSevere`, `ctx.floodModerate`, `flood.*` QA counters |
| 11 | `writeChunks` | `output.cpp` | serialises every cell, hashes every section, re-opens every file and verifies | 924 chunks + QA counters |
| 12 | `writeManifest` | `output.cpp` | the machine-readable build record: extent, projection, cell list, totals, zones, surfaces, drainage basins, QA | `world_manifest.json` |
| 13 | `exportViewer` + `finaliseQa` | `export.cpp` | re-reads every chunk through the runtime reader and packs the viewer payload; then publishes the finished QA report to both JSON files | `viewer/data/mumbai.{bin,json}` |

## `.mumbai_chunk` — the world format

One file per 1024 m macro cell, designed so the runtime can `mmap` it and use
the bytes in place: every section offset is 16-byte aligned, every structure is
`#pragma pack`ed with a `static_assert` on its size, and every section carries
its own FNV-1a 64 hash.

```
ChunkHeader (128 B, magic "MUMBAICH")
    version, headerSize, cellX, cellZ, cellSize, minHeight, maxHeight,
    sectionCount, flags, fileSize, contentHash, sourceHash, buildSchema
SectionEntry[sectionCount] (40 B each)
    type, format, flags, offset, size, elementCount, stride, hash
payload sections — one per entry, 16-byte aligned
```

`SectionType` covers the 19 kinds the runtime consumes: `ChunkInfo`,
`TerrainHeight` (query/flood/placement), `TerrainMesh` (with `MeshEntry` per
LOD, whose submesh and vertex/index ranges are all file-relative offsets),
`TerrainCollision`, `TerrainAO`, `MaterialRegions`, `RoadMesh`,
`RoadNavGraph` (header + nodes + edges + signals), `BuildingMesh`,
`BuildingCollision`, `BuildingImpostor`, `WaterMesh`, `RailMesh`,
`LandusePolys`, `InstanceTable`, `MaterialTable`, `HeroAssets`, `QAReport` and
`BuildingProxy`.

Two format rules exist because breaking them is silent:

* **Prefixed sections.** `RoadNavGraph` starts with a `NavGraphHeader` followed
  by three arrays. `array<T>()` refuses a section flagged
  `SectionFlagPrefixed` — reading a prefixed section as a flat array hands back
  the header as element zero, which cost one out-of-bounds graph walk before
  the flag existed. Use `ChunkReader::navGraph()`, which re-validates every
  offset against the section size.
* **Hash on load.** `verifyHashes()` checks every section; `exportViewer` and
  `mumbai_sim` both call it for every chunk they touch, so a byte that changes
  after the build is caught by the next thing that reads it.

Record sizes (all `static_assert`ed): `Header` 128, `SectionEntry` 40,
`MeshEntry` 40, `MeshVertex` 44, `CollisionBox` 48, `BuildingProxy` 48,
`NavNode` 24, `NavEdge` 56, `InstanceRecord` 52, `MaterialDef` 80,
`CellInfo` 128.

## Geometry conventions

* **Winding** (D-010): walls are emitted `(bottom-start, top-start, top-end,
  bottom-end)`; ear-clipper output is always reversed; sea quads use
  `(x0z0, x0z1, x1z1, x1z0)`. A CCW triangle in world XZ faces *down*.
* **Cell-local output**: every stage writes cell-local metres (world − cell
  origin); the export converts to the extent-relative frame
  (`cellX·1024 + local − extent.min`) exactly once.
* **UV policy**: U follows arc length in world metres, V follows height, so
  texel density is constant regardless of triangle size; uv2 carries a
  world-position macro variation (advanced by `offset × 0.0016` on translate)
  to break up tiling.
* **Polygons**: `triangulate()` takes an outer ring CCW and holes CW,
  `offsetRing(r, +δ)` shrinks a CCW ring, `resampleOpen()` limits spacing, and
  `polygonContains()` respects holes.

## Materials

116 materials are defined in `MaterialLibrary` (declared once, ids assigned in
insertion order, mirrored by the `MaterialId` enum and serialised as
`MaterialDef` 80 B records). Id **0 means "no material"** and the viewer renders
it as neutral wet concrete rather than black — a missing id must never produce
a rendering artifact.

The ordered base set (id = position): `asphalt_smooth`, `asphalt_patched`,
`asphalt_sealink`, `concrete_road`, `paver_block`, `gravel`, `dirt`, `rock`,
`earth`, `sand`, `wetland_mud`, `reclamation_fill`, `grass`, `plaster_painted`,
`stone_ashlar`, `stone_rubble`, `curtain_wall_glass`, `brick_exposed`,
`corrugated_metal`, `concrete_raw`, `kerb_concrete`, `sidewalk_concrete`,
`median_barrier`, `water_surface`, `inland_water`, `rail_ballast`,
`rail_steel`, `rail_sleeper`, `water_tank_plastic`, `ac_unit`,
`metal_railing`, `signage_painted`, `tree_foliage`, `tree_bark`,
`streetlamp_pole`, `streetlamp_glow` (emissive), `hero_stone`,
`hero_terracotta`, `hero_concrete`, `sea_link_pylon`, `sea_link_cable`.

## Roads, rail and the nav graph

* Cross-sections are per class: lane width 3.5 m (motorway/trunk) down to
  2.5 m (track); medians 2.0 m (motorway/trunk) and 1.2 m (primary); sidewalks
  2.5/2.0/1.5 m; kerbs 0.15 m high; the median barrier 0.9 m.
* Resampling caps edge length at `max(4 m, carriageway × 1.5)`; junction
  detection keys endpoints to 0.01 m and trims within
  `max(3 m, carriageway × 0.65)`; junctions get an octagonal fan patch.
* Default speed limits by class (km/h): 80 motorway, 65 trunk, 55 primary,
  45 secondary, 40 tertiary, 30 residential, 15 service, 5 pedestrian, 25
  default; a `maxspeed` tag overrides.
* Nav flags record drivable/pedestrian-only, bridge, tunnel, one-way and
  walkable; edges carry lane count, surface, speed limit, length, road class
  and the cubic control points of the curve they approximate.
* Rail constants: broad gauge 1.676 m, monorail 0.85 m, ballast half-width
  3.4 m / 1.6 m, sleepers every 2 m, rail strips 0.08 m wide, base at
  terrain + 0.5 m (D-013).

## Buildings and props

Heights come from the `height`/`levels` tags when present, otherwise from the
zone rule (colonial 12–22 m, BKC 50–120 m, suburban 15–45 m in the tall wards
and 9–25 m elsewhere, informal 3.2–9.6 m, industrial 6–16 m, hillside 6–14 m,
airport 8–22 m, institutional 10–26 m), snapped to whole storeys where the
grammar calls for it. Facade palettes are per zone (the suburban palette is the
specification's cream / pale blue / mint / salmon / ochre), and material
variants are cached by (base | weathering bucket | tint bucket | zone) so a
ward of 30,000 buildings does not create 30,000 materials.

Each building emits LOD0 (shops band, walls, roof cap or pitched roof, parapet),
LOD1, LOD2 and an impostor quad, one collision box (decomposed into boxes above
2,500 m²), and an AO term baked from its own massing. Props are placed on the
footprints the building stage publishes (`ctx.footprints`), so a tank never
floats off a roof.

## QA

`QaReport` collects named counters, warnings and failures; a failure makes the
process exit non-zero. The last build reports **155 counters, 0 warnings,
0 failures**. Counters are the build's public record: geometry totals,
per-stage timings, per-asset triangle counts, synthetic-versus-surveyed
counts, and the corridor/hero self-descriptions (for example
`hero.seaLink.lengthMetres` = 4,014.697 against
`hero.seaLink.osmFragmentMetres` = 320.464).

The flood stage is the one place that can *fail* the build on geography rather
than on arithmetic: Hindmata, King's Circle and the Andheri subway must each
exceed 50 mm in the severe storm or the stage reports a failure, because a
hydrology model that leaves those three dry is not modelling Mumbai. Marine
Drive is the reverse control — it warns above 500 mm, since the sea wall holds
and the promenade is at 3.5 m (D-026).

Publication is single-path (D-019): `qaReportJson()` is the only serialisation,
and `finaliseQa` rewrites both JSON files at the end of the build so the
console, the manifest and the viewer payload can never disagree.
