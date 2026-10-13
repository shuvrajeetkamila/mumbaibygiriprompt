# Master build prompt — requirements as received

**Provenance.** The project owner's master prompt ("MUMBAI 1:1 EXACT") was
supplied as pasted text in the build conversation; a PDF export of the same
document was referenced but never arrived on disk. This file is a *structured
restatement* of those requirements, written so that the repository carries its
own build contract and so that every acceptance statement in `DOCS/` can be
traced back to a requirement. It is a faithful summary, not a verbatim copy.

Standing directives from the prompt, quoted because they govern how the work
was done:

> "Do not ask questions. Make confident, technically justified decisions at
> every ambiguity. Document every decision."

> "Build to that standard. Begin with the geospatial pipeline. Every subsequent
> system depends on its accuracy."

---

## Section 0 — Mission and standards

An exact 1:1 topographical and architectural replica of Mumbai: Colaba to
Borivali/Dahisar, the Arabian Sea to Thane Creek, delivered as a native
high-fidelity Android application, > 150 GB of installed data, ~600 km²
playable, 200,000+ buildings, ~34,000 km of roadway, 14 rail lines, 6 highway
corridors, 150+ km of coastline.

Visual standard: a motorcycle on the coastal highway at golden hour, wet
asphalt reflecting the sky. Nothing shipped "rough".

Zero-artifact standards: no rendering artifacts, no physics instability, no
floating-point drift, no streaming hitches, no frame drops, no collision
penetration, no navmesh failures, no memory leaks, no crashes. Evaluated as if
by geospatial engineers and Mumbai residents frame by frame — the prompt's own
example: a Bandra–Worli Sea Link ramp radius wrong by more than 2 m is a
failure.

## Section 1 — Geospatial pipeline

1. **DEM ingestion.** Cartosat-1 (1 m) primary, SRTM (30 m) gap-fill, MCGM
   LiDAR where available. Source CRS WGS84/UTM 43N → a Mumbai-centred
   Transverse Mercator. Quadtree variable resolution: 1 m/texel on slopes,
   4–8 m/texel on flats. Meshoptimizer-class LOD generation with crack-free
   stitching. Baked collision geometry, ambient occlusion and material region
   masks (rock, earth, sand, reclamation fill, marsh). Sub-metre accuracy:
   Malabar Hill ~50 m, Pali Hill, the Powai basin, and the Nariman Point /
   Cuffe Parade / BKC reclamations at or below high-tide level.
2. **Roads.** OSM `.pbf` via `osmpbf`; Bézier spline fitting within 0.5 m of
   the source geometry; cross-section extrusion per road class (Worli Sea Link,
   the Western Express Highway's 6 lanes with median, Andheri residential
   streets, pedestrian alleys); PBR material per surface tag; Recast/Detour
   navmesh with lane vectors and traffic-signal phases.
3. **Buildings.** Footprints from OSM plus rule-based heights by ward — C Ward
   12–20 m, K-East 25–45 m, G-North/BKC 50–120 m.

## Section 2 — Assets

1. **Procedural generation** (Houdini Engine or a Rust-native grammar) for five
   zones: South Bombay Colonial (Indo-Saracenic / Victorian Gothic), BKC
   Commercial (curtain-wall glass), Suburban Residential (RC frame, water
   tanks, laundry, AC units), Informal Settlement (Dharavi — dense, organic,
   labyrinthine, *accurate, not caricature*), Industrial (MIDC and the dock
   estates). LOD tiers: < 100 m full detail, 100–500 m simplified, 500 m–2 km
   blockout with baked facades, > 2 km impostors.
2. **Hero landmarks** with triangle budgets: Gateway of India ~500k tris; Taj
   Mahal Palace ~800k (heritage wing + tower); CSMT ~1.2M; the Bandra–Worli Sea
   Link ~300k per pylon with capsule colliders for the cables; Marine Drive's
   Queen's Necklace ~50k per building with clustered forward lighting for 100+
   lamps.

## Section 3 — Weather, atmosphere, ocean

1. Monsoon: 2,400 mm/year, June–September, > 100 mm/hr bursts; GPU rain
   particles; a screen-space wetness buffer driving PBR (roughness → 0, albedo
   −30 %, normal flattening); a heightfield flood model in which Hindmata,
   King's Circle and the Andheri subway flood as an emergent consequence of the
   DEM.
2. Sky: Rayleigh + Mie scattering parameterised by time of day, humidity and
   pollution (monsoon haze vs winter smog).
3. Arabian Sea: 4–8 Gerstner waves (swell 80–150 m wavelength, 0.5–2 m
   amplitude; monsoon 3–5 m), PBR with subsurface scattering, sun glitter, and
   depth-buffer shoreline foam and runup.

## Section 4 — Floating origin

1024 m macro-cells (~600 of them). `GlobalPosition` as `int64 cellX`,
`int64 cellZ` plus a `vec3 localOffset` bounded by the cell; `Normalize()` and
`ToRenderSpace(cameraCell)`. The origin shifts between the physics tick and the
render submission, under 1 ms, with no pop, no impulse and no audio jump: Jolt
bulk transforms for the physics world, a uniform-buffer offset (not per-vertex
work) for rendering, and adjusted particle and audio positions. Verified by 20
consecutive boundary crossings at maximum speed.

## Section 5 — Traffic

flecs ECS. Vehicle components: transform, physics, dimensions, navigation
state, driver behaviour, visual state. Archetypes: BEST double-decker bus
(12 m, 16,000 kg, lane-discipline 0.9 / aggression 0.3); auto-rickshaw (400 kg,
0.2 / 0.7, rollover geometry); kaali-peeli taxi (0.4 / 0.6, stops for hailing
pedestrians); motorcycles (110–160 kg, 0.05 / 0.8). Hybrid control: IDM
longitudinal + MOBIL lane changes for large vehicles; a Boids flow field for
small vehicles (200 ms updates, 20 m broadphase; strong separation, alignment,
cohesion ≈ 0).

## Section 6 — Streaming and rendering

mmap-backed `.mumbai_chunk` streaming: LOD0 resident under 500 m, reduced
500 m–2 km, impostors beyond. 30 FPS on Adreno 730 / Mali-G710 class hardware
with 8 GB RAM, dynamic resolution 720p–1080p, and a ~28 ms GPU budget. Vulkan
1.1: depth prepass + forward pass with clustered lights, WBOIT transparency,
TAA, filmic tonemapping, compute post-processing.

## Section 7 — Acceptance criteria

1. Terrain within 1 m vertically of Cartosat.
2. Roads within 2 m horizontally of OSM.
3. Landmarks recognisable at 100 m with no labels.
4. Zero vertex jitter, verified at the four corners and the centre of the map.
5. At least 500 vehicles in a 500 m radius with no frame impact.
6. Weather and ocean systems valid.
7. 30 FPS worst case: monsoon rain, maximum traffic, dense South Bombay, Marine
   Drive.
8. 160 GB or less installed.
9. A 60-minute continuous Colaba → Borivali session with zero crashes.

Status of every one of these, with evidence: `DOCS/05_VALIDATION.md`.
