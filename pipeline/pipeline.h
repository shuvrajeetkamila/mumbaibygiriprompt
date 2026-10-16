// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/pipeline.h
//
//  The offline world builder.  Stages run in a strict order because each one
//  consumes the previous one's product:
//
//    loadSources -> terrain -> zoning -> water -> roads -> rail/landuse
//                -> buildings -> instances -> chunks -> manifest
//
//  Everything is deterministic: same inputs + same config = byte-identical
//  chunks (verified by re-reading each written chunk and checking its content
//  hash in the same process that wrote it).
// =============================================================================
#pragma once
#include "engine/core/geo.h"
#include "engine/core/json.h"
#include "engine/world/chunk.h"
#include "pipeline/mgeo/osm.h"
#include "pipeline/mgeo/geojson.h"
#include "pipeline/mgeo/geometry.h"
#include "pipeline/mgeo/mesh.h"
#include "pipeline/mgeo/dem.h"
#include "pipeline/mgeo/texture.h"

#include <map>

namespace mumbai::pipeline {

// -----------------------------------------------------------------------------
//  Build configuration
// -----------------------------------------------------------------------------
struct BuildConfig {
    std::string rawDir      = "data/raw";
    std::string authoredDir = "data/authored";
    std::string outDir      = "data/processed";

    // DEM conditioning for the hydrology (D-026): closed depressions needing
    // less than this much fill are fills of the grid, not places water stands.
    f64 pitFillDepth      = 2.0;
    f64 terrainResolution = 24.0;     // metres per heightfield sample
    f64 zoningResolution  = 100.0;    // metres per zoning raster cell
    f64 seaLevel          = 0.0;
    f64 terrainDetail     = 2.4;      // fractal detail amplitude (metres)
    f64 seaLevelLandFloor = 0.15;     // minimum land elevation above sea level
    u64 seed              = 20251013;

    // Road/geometry density controls (quality vs build time).
    bool buildTerrain  = true;
    bool buildFloodModel = true;      // monsoon hydrology: storms + flood raster
    bool buildRoads    = true;
    bool buildBuildings= true;
    bool buildProps    = true;
    bool buildHero     = true;
    bool verifyChunks  = true;        // re-open and hash every chunk after write
    bool buildTextures = true;        // procedural PBR content (A8)
    u32 textureSize    = 256;         // mip0 edge of one material tile, pixels
    // Hero asset detail, 1..3 (B1/B3).  It drives repetition counts and the
    // number of refinement steps in the landmark detail passes — opening
    // mullions, balustrades, arcade columns, dome ribs, deck railings, the Art
    // Deco frontage — not their position or proportion, so a change here is a
    // change in how much of the published geometry is *modelled* rather than in
    // where it stands.  1 = the massing plus its principal mouldings, 2 = the
    // shipping default (glazing bars, balustrades, columns, deck furniture),
    // 3 = the survey-chasing tier that also subdivides curved surfaces finely.
    u32 heroDetail     = 2;

    // Simplification tolerances, all in metres.
    f64 footprintSimplify = 0.35;
    f64 roadSimplify      = 0.5;
    f64 minBuildingArea   = 12.0;

    static BuildConfig fromJson(const Json& j);
};

// -----------------------------------------------------------------------------
//  Material library — the single source of truth for material ids shared by
//  the pipeline, the runtime and the viewer.  Ids are stable and serialised
//  into every chunk's MaterialTable section.
// -----------------------------------------------------------------------------
class MaterialLibrary {
public:
    MaterialLibrary();
    u32 id(const std::string& name) const;      // 0 = unknown
    u32 add(const MaterialDef& def);
    const std::vector<MaterialDef>& all() const { return m_defs; }
    size_t count() const { return m_defs.size(); }

    // Serialises the table as a section payload.
    std::vector<u8> serialize() const;

    // --- texture content (A8) ----------------------------------------------
    // `layer` is the 1-based index into the material's texture array; 0 means
    // the material has no texture and renders from its parameters alone.
    // Records that material `id` is baked into array layer `layer` (0-based, the
    // index the runtime and the atlas grid both use) and how many metres one tile
    // covers.  Called by the texture stage as it appends each layer.
    void setTextureSet(u32 id, u32 layer, f32 tileMetres);
    // 0-based texture-array / atlas-tile index of a material, or -1 when the
    // material is untextured (id 0 "unknown" is).  The sentinel is -1 rather than
    // 0 because layer 0 is a real layer: overloading it would silently give every
    // untextured surface the first material in the library.
    i32 textureLayerIndex(u32 id) const;
    // 1-based layer number, kept for the QA report ("layer 3 of 115").
    u32 textureLayer(u32 id) const;
    // Mean *linear* albedo of a baked layer.  This is the number the shader
    // divides the sampled tile by, which is what makes adding textures a
    // detail change rather than a re-grade: the quotient has mean 1, so a
    // tinted facade variant keeps its tint and still gains the grain.
    void setTextureLayerMean(u32 layer, f32 r, f32 g, f32 b);
    bool textureLayerMean(i32 layer, f32 rgb[3]) const;
    // The physical size of one texture tile in metres.  This is what the mesh
    // layer divides UVs by, so it is derived from the material's *name* and is
    // available for every material — including the facade variants that are
    // minted per building long after the base library was built.
    f32 tileMetres(u32 id) const;
    // Metres per UV unit: one UV unit is one tile.  Falls back to the caller's
    // own scale when the material is unknown, so a mesh built without a library
    // (tests, tools) behaves exactly as it did before textures existed.
    f32 uvScaleFor(u32 id, f32 fallback) const;
    u64 unmappedUvLookups() const { return m_unmapped; }
    // Number of materials that received a texture layer (A8), i.e. the number of
    // tiles in the atlas the viewer reads.
    u32 textureLayerCount() const { return m_textureLayers; }
    void setTextureLayerCount(u32 n) { m_textureLayers = n; }
    u32 textureAtlasTile() const { return m_atlasTile; }
    void setTextureAtlasTile(u32 n) { m_atlasTile = n; }

private:
    std::vector<MaterialDef> m_defs;
    std::unordered_map<std::string, u32> m_lookup;
    std::vector<f32> m_tiles;         // parallel to m_defs
    mutable u64 m_unmapped = 0;
    u32 m_textureLayers = 0;          // how many layers the texture stage wrote
    u32 m_atlasTile = 0;              // atlas tile edge in pixels
    std::vector<f32> m_layerMean;     // 3 floats per baked layer (linear RGB)
};

// -----------------------------------------------------------------------------
//  Zoning raster: the per-metre classification that drives procedural
//  generation rules (Section 2A) and traffic behaviour.
// -----------------------------------------------------------------------------
class ZoningField {
public:
    void init(const WorldExtent& extent, f64 resolution);
    void set(i64 i, i64 j, ZoningZone z) {
        if (i < 0 || j < 0 || i >= static_cast<i64>(width) || j >= static_cast<i64>(height)) return;
        zones[static_cast<size_t>(j) * width + static_cast<size_t>(i)] = static_cast<u8>(z);
    }
    ZoningZone at(i64 i, i64 j) const {
        if (i < 0 || j < 0 || i >= static_cast<i64>(width) || j >= static_cast<i64>(height))
            return ZoningZone::Unknown;
        return static_cast<ZoningZone>(zones[static_cast<size_t>(j) * width + static_cast<size_t>(i)]);
    }
    ZoningZone sample(const Vec2d& world) const {
        const i64 i = static_cast<i64>(std::floor((world.x - originX) / step));
        const i64 j = static_cast<i64>(std::floor((world.y - originZ) / step));
        return at(i, j);
    }
    u32 width = 0, height = 0;
    f64 originX = 0, originZ = 0, step = 100.0;
    std::vector<u8> zones;
};

// -----------------------------------------------------------------------------
//  Per-cell accumulation
// -----------------------------------------------------------------------------
struct CellKey {
    i64 x = 0, z = 0;
    bool operator==(const CellKey& o) const { return x == o.x && z == o.z; }
};
struct CellKeyHash {
    size_t operator()(const CellKey& k) const {
        return static_cast<size_t>(fnv1a64(static_cast<u64>(k.x) * 0x100000001B3ull ^
                                           static_cast<u64>(k.z)));
    }
};

struct CellCounts {
    u32 buildings = 0, roads = 0, roadJunctions = 0, waterPolys = 0, railWays = 0;
    u32 landusePolys = 0, instances = 0, heroAssets = 0, trees = 0, lamps = 0;
    u32 proxies = 0;
};

struct CellData {
    i64 cellX = 0, cellZ = 0;
    Vec2d origin{0, 0};                 // world metres of the cell's south-west corner

    mesh::MeshBuilder terrain[4];       // LOD 0..3
    mesh::MeshBuilder roads;
    mesh::MeshBuilder buildings[4];     // LOD 0..3
    mesh::MeshBuilder water;
    mesh::MeshBuilder rail;
    mesh::MeshBuilder hero;
    mesh::MeshBuilder impostors;        // far-field building billboards

    std::vector<CollisionBox> collision;
    std::vector<NavNode> navNodes;
    std::vector<NavEdge> navEdges;
    std::vector<NavSignalPhase> signals;
    std::vector<InstanceRecord> instances;
    std::vector<BuildingProxy> proxies;   // oriented massing blocks (all LODs)
    std::vector<Vec2d> landuseVerts;
    std::vector<LandusePoly> landusePolys;

    f32 minHeight = 0.0f;
    f32 maxHeight = 0.0f;
    CellCounts counts;

    // Material region raster for this cell (per-texel SurfaceMaterial mask).
    u32 regionWidth = 0, regionHeight = 0;
    std::vector<u8> regions;

    bool empty() const {
        return terrain[0].empty() && roads.empty() && buildings[0].empty() &&
               water.empty() && rail.empty() && hero.empty();
    }

    // One call, every builder: the cell's meshes all scale their UVs by the same
    // material table (A8).
    void setUvScales(const std::vector<f32>* t) {
        mesh::MeshBuilder* all[] = {&terrain[0], &terrain[1], &terrain[2], &terrain[3],
                                    &roads, &buildings[0], &buildings[1], &buildings[2],
                                    &buildings[3], &water, &rail, &hero, &impostors};
        for (mesh::MeshBuilder* m : all) m->setUvScales(t);
    }
};

// -----------------------------------------------------------------------------
//  QA report — machine-readable build validation (Section 7 criteria)
// -----------------------------------------------------------------------------
struct QaReport {
    std::vector<QaCounter> counters;
    // MEASUREMENTS OF THE RUN, NOT OF THE WORLD: how long a stage took on this
    // machine.  They are printed, they are read by a human, and they are kept out
    // of `counters` on purpose — every cell carries the QA report in its
    // `QAReport` section and the manifest carries it in `quality`, and the
    // manifest's sha256 is the identity of the world.  A build whose identity
    // changes because the disk was busy is not an identity; the first version of
    // this file put them in `counters`, and two runs of the same pipeline differed
    // in 13 counters and 80 bytes of every cell (D-051).
    std::vector<QaCounter> timings;
    std::vector<std::string> warnings;
    std::vector<std::string> failures;

    void add(const std::string& name, f64 value, u32 severity = 0);
    // Seconds spent, recorded beside the report rather than in it.
    void addTiming(const std::string& name, f64 seconds);
    // Look up a counter published earlier in this build (0 when absent).  Lets a
    // later measurement be expressed against an earlier one — a budget against
    // the asset count it belongs to — instead of re-deriving it.
    bool has(const std::string& name) const;
    f64 value(const std::string& name) const;
    void warn(const std::string& msg);
    void fail(const std::string& msg);
    bool ok() const { return failures.empty(); }
    void logSummary() const;
};

// -----------------------------------------------------------------------------
//  Build context
// -----------------------------------------------------------------------------
struct Context {
    BuildConfig cfg;
    TransverseMercator proj;
    osm::OsmData osm;

    std::vector<geojson::Feature> wardFeatures;
    geojson::MultiMask cityMask;
    std::vector<geojson::Feature> slumFeatures;
    geojson::MultiMask slumMask;

    terrain::Heightfield terrainHF;
    std::vector<terrain::ControlPoint> controlPoints;
    std::vector<terrain::Heightfield::LowPoint> lowPoints;

    ZoningField zoning;
    MaterialLibrary materials;

    // --- material UV scales (A8) --------------------------------------------
    // One entry per material id, = 1 / tileMetres, so a mesh's UV in metres is
    // divided by the material's physical tile.  Kept in step with the library
    // lazily: `uvScales()` refreshes it whenever the material count has moved
    // (facade variants are minted per building, long after the base library), and
    // every MeshBuilder in the world holds a pointer to this vector, so a cell
    // created early still resolves a material created later.  The vector's
    // address never changes, only its contents.
    std::vector<f32> uvScaleTable;
    // The UV scale for one material (metres -> UV); `fallback` is the caller's
    // legacy scale, used when the material is unknown or the table has no entry.
    f32 uvScaleFor(u32 materialId, f32 fallback = 0.25f) {
        const std::vector<f32>& t = uvScales();
        return materialId < t.size() && t[materialId] > 0.0f ? t[materialId] : fallback;
    }
    const std::vector<f32>& uvScales() {
        if (uvScaleTable.size() != materials.count()) {
            uvScaleTable.resize(materials.count());
            for (u32 i = 0; i < materials.count(); ++i)
                uvScaleTable[i] = materials.uvScaleFor(i, 0.25f);
        }
        return uvScaleTable;
    }

    std::unordered_map<CellKey, CellData, CellKeyHash> cells;
    CellKey originCell{0, 0};

    // Footprints processed by buildBuildings and reused by buildProps and
    // buildHeroAssets, so the OSM way list is walked once, not three times.
    struct FootprintRecord {
        Vec2d centre;
        geo2d::Ring ring;
        f64 groundY = 0;
        f64 height = 0;
        ZoningZone zone = ZoningZone::Unknown;
        u64 seed = 0;
        u32 wallMaterial = 0;
        bool pitchedRoof = false;
    };
    std::vector<FootprintRecord> footprints;

    // --- monsoon flood model (Section 3, pipeline/stages/flood.cpp) ----------
    // Peak submersion depth of the severe design storm, in centimetres, on the
    // same grid the viewer payload uses.  This is a *result*, not an input: it
    // is produced by letting the DEM route the design rainfall.
    std::vector<u8> floodSevere;
    std::vector<u8> floodModerate;
    u32 floodWidth = 0, floodHeight = 0;
    f64 floodStep = 32.0;

    // Synthesised building parcels produced by buildRoads (which is what owns
    // the street lattice) and consumed by buildBuildings.  Every entry carries
    // its own zone/height/seed so the two stages stay decoupled.  Province:
    // DOCS/D-020 (the raw extract carries tags but not geometry).
    struct SynthParcel {
        geo2d::Ring ring;        // world space, CCW
        Vec2d centre;
        u8 kind = 0;             // 0 = formal ward fabric, 1 = informal slum fabric
    };
    std::vector<SynthParcel> synthParcels;

    WorldExtent extent;
    u64 sourceHash = 0;
    QaReport qa;

    struct Stats {
        u64 buildingCount = 0, buildingTriangles = 0;
        u64 roadWays = 0, roadLengthMetres = 0, roadTriangles = 0, junctions = 0;
        u64 navNodes = 0, navEdges = 0, navSignals = 0;
        u64 collisionBoxes = 0;
        u64 instanceCount = 0, heroAssetCount = 0;
        u64 terrainTriangles = 0, waterTriangles = 0, railWays = 0, railTriangles = 0;
        u64 landusePolys = 0;
        f64 landAreaKm2 = 0.0;               // dry land inside the extent (hydro model)
        u64 chunkCount = 0, chunkBytes = 0, totalTriangles = 0;
        u64 heroTriangles = 0;
        std::map<std::string, u64> zoneCounts;
        std::map<std::string, u64> surfaceCounts;
        f64 terrainMinHeight = 0, terrainMaxHeight = 0;
    } stats;

    // --- helpers ------------------------------------------------------------
    CellData& cell(const Vec2d& worldPos);          // cell containing a point
    CellData& cellAt(i64 cx, i64 cz);
    Vec2d cellOrigin(i64 cx, i64 cz) const {
        return {static_cast<f64>(cx) * MACRO_CELL_SIZE, static_cast<f64>(cz) * MACRO_CELL_SIZE};
    }
    CellKey keyOf(const Vec2d& p) const {
        return {static_cast<i64>(std::floor(p.x / MACRO_CELL_SIZE)),
                static_cast<i64>(std::floor(p.y / MACRO_CELL_SIZE))};
    }
    geo2d::Aabb2d worldBounds() const {
        geo2d::Aabb2d b;
        b.expand(extent.min);
        b.expand(extent.max);
        return b;
    }

    // Cell lookup for *emission*.  Returns nullptr when the point lies outside
    // the world extent: the extract covers a far larger area than the world
    // (the raw OSM bounding box is 114 km north-south against a 43 km world),
    // and emitting that content would create chunks with no cell in the grid —
    // which is exactly the class of defect the manifest contract forbids.
    CellData* tryCell(const Vec2d& worldPos) {
        if (worldPos.x < extent.min.x || worldPos.x >= extent.max.x ||
            worldPos.y < extent.min.y || worldPos.y >= extent.max.y) {
            return nullptr;
        }
        return &cellAt(static_cast<i64>(std::floor(worldPos.x / MACRO_CELL_SIZE)),
                       static_cast<i64>(std::floor(worldPos.y / MACRO_CELL_SIZE)));
    }
    bool inExtent(const Vec2d& p) const {
        return p.x >= extent.min.x && p.x < extent.max.x &&
               p.y >= extent.min.y && p.y < extent.max.y;
    }
};

// -----------------------------------------------------------------------------
//  Stages.  Each returns false on a fatal error (already logged).
// -----------------------------------------------------------------------------
bool loadSources(Context& ctx);
bool buildTerrain(Context& ctx);
bool buildZoning(Context& ctx);
bool buildWater(Context& ctx);
bool emitTerrainMeshes(Context& ctx);
bool buildRoads(Context& ctx);
bool buildRailAndLanduse(Context& ctx);
bool buildBuildings(Context& ctx);
bool buildProps(Context& ctx);
bool buildHeroAssets(Context& ctx);
bool buildTextures(Context& ctx);        // procedural PBR content (textures.cpp, A8)
bool buildFloodModel(Context& ctx);      // monsoon hydrology (flood.cpp)
bool writeChunks(Context& ctx);
bool writeManifest(Context& ctx);
// Viewer export: re-reads every chunk through the runtime reader and packs the
// world for viewer/index.html (pipeline/stages/export.cpp).
bool exportViewer(Context& ctx);
// Rewrites the QA block of the world manifest and of the viewer payload from
// the finished report.  Both files are written by stages that run before the
// last per-stage timing counters exist, so publishing their snapshot as-is made
// the payload disagree with the console (63 counters against 66).  This is the
// last step of the build and includes its own timing entry.
bool finaliseQa(Context& ctx, f64 stageStart);
void printReport(const Context& ctx);

// Serialised form of the QA report, shared by the manifest, the viewer payload
// and the finalisation step so the three can never drift apart.
Json qaReportJson(const QaReport& qa);

// Shared geometry helpers used by several stages.
namespace detail {
// Surface material for an OSM highway class, honouring the surface tag.
SurfaceMaterial roadSurface(osm::HighwayClass cls, std::string_view surfaceTag);
// Road cross-section for a class.
struct RoadProfile {
    f64 carriagewayWidth = 7.0;   // total paved width
    f64 laneWidth = 3.5;
    int lanesTotal = 2;
    f64 medianWidth = 0.0;
    f64 kerbWidth = 0.0;
    f64 sidewalkWidth = 0.0;
    f64 layerOffset = 0.03;       // vertical offset above terrain
    bool pedOnly = false;
};
RoadProfile roadProfile(osm::HighwayClass cls, std::string_view lanesTag, bool oneway);

// A road that is going to be extruded.  It is either a real OSM way (`way` set)
// or a synthesised one (`way` null, `synthetic` true).  Both kinds flow through
// the same extrusion, junction, navigation and collision code — which is what
// keeps the synthesised network honest: it is not a special case downstream.
struct WaySpec {
    const osm::Way* way = nullptr;
    osm::HighwayClass cls = osm::HW_Residential;
    RoadProfile profile;
    std::vector<Vec2d> points;       // world space, resampled (mesh chain)
    // Navigation chain: `points` plus a vertex at every at-grade crossing with
    // another road, inserted by detail::planarizeCrossings.  Empty when the road
    // crosses nothing, in which case the navigation graph uses `points`.  Kept
    // apart from `points` so that a junction node — a topology fact — does not
    // cost the ribbon mesh an extra cross-section.
    std::vector<Vec2d> navPoints;
    bool bridge = false, tunnel = false;
    std::string name;
    SurfaceMaterial surface = SurfaceMaterial::AsphaltPatched;
    bool synthetic = false;
    u8 rank = 1;                     // 0 collector, 1 local (synthesised only)
};
ZoningZone zoneForWard(const std::string& wardCode, const Vec2d& world, const Context& ctx);

// Appends the authored corridors (data/authored/transport_corridors.json) and
// the synthesised ward street lattice to `ways`, and stashes the synthesised
// building parcels on `ctx.synthParcels`.  Implemented in
// pipeline/stages/corridors.cpp; rationale in DOCS/D-020 / D-021.
void appendAuthoredAndSynthetic(Context& ctx, std::vector<WaySpec>& ways);
} // namespace detail

} // namespace mumbai::pipeline
