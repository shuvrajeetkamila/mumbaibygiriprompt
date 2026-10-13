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

#include <map>

namespace mumbai::pipeline {

// -----------------------------------------------------------------------------
//  Build configuration
// -----------------------------------------------------------------------------
struct BuildConfig {
    std::string rawDir      = "data/raw";
    std::string authoredDir = "data/authored";
    std::string outDir      = "data/processed";

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

private:
    std::vector<MaterialDef> m_defs;
    std::unordered_map<std::string, u32> m_lookup;
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
};

// -----------------------------------------------------------------------------
//  QA report — machine-readable build validation (Section 7 criteria)
// -----------------------------------------------------------------------------
struct QaReport {
    std::vector<QaCounter> counters;
    std::vector<std::string> warnings;
    std::vector<std::string> failures;

    void add(const std::string& name, f64 value, u32 severity = 0);
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
    std::vector<Vec2d> points;       // world space, resampled
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
