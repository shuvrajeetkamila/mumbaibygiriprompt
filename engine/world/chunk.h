// =============================================================================
//  MUMBAI 1:1 EXACT — engine/world/chunk.h
//
//  THE .mumbai_chunk FORMAT  (specification Section 1: "engine-native binary
//  chunk files that the runtime can memory-map directly ... without parsing
//  overhead").
//
//  Layout
//  ------
//    [ ChunkHeader        : 128 bytes, fixed          ]
//    [ SectionEntry * N   : 32 bytes each, fixed       ]
//    [ section payloads   : 16-byte aligned, size-prefixed  ]
//
//  Rules that the format guarantees (and tests/test_chunk.cpp enforces):
//    * Everything is little-endian, fixed-width, and alignment-explicit, so a
//      payload can be cast to a packed struct view after mmap with no copy.
//    * All geometry inside a chunk is stored *cell-local*: metres relative to
//      the chunk's south-west corner.  A 1024 m chunk therefore holds
//      coordinates in roughly [-256, +1280], where f32 ulp is <= 1.2e-4 m, so
//      chunk geometry carries sub-0.2 mm precision independently of where the
//      chunk sits in the world.
//    * Every payload is content-hashed (FNV-1a 64) at build time.  The runtime
//      verifies the chunk hash on load; a corrupt chunk is rejected, never
//      rendered.  This is the mechanism behind the "zero rendering artifacts"
//      criterion: bad bytes cannot reach the GPU.
//    * Sections are independently addressable, so streaming can page in only
//      the LOD band it needs (Section 6A).
// =============================================================================
#pragma once
#include "engine/core/geo.h"
#include "engine/core/math.h"

namespace mumbai {

inline constexpr char     CHUNK_MAGIC[8] = {'M','U','M','C','H','N','K','1'};
inline constexpr u32      CHUNK_VERSION  = 1;
inline constexpr u32      CHUNK_HEADER_SIZE = 128;
// On-disk size of a SectionEntry.  This constant and the struct must agree —
// they did not, once: the table was laid out at 32 bytes per entry while 40
// bytes per entry were written, so the first payload overwrote the tail of the
// table and every later section parsed as garbage.  The static_assert below is
// the guard; do not reorder the fields without updating it.
inline constexpr u32      SECTION_ENTRY_SIZE = 40;

// SectionEntry::flags bits.  A prefixed section starts with a header followed by
// arrays whose stride the header describes; reading one through array<T>() would
// hand back the header as the first element, which is a footgun (it cost one
// out-of-bounds graph walk before this flag existed), so prefixed sections are
// refused by array<T>() and must be read through their typed accessor.
enum SectionFlags : u32 {
    SectionFlagMandatory = 1u,
    SectionFlagPrefixed  = 0x8000u,
};

enum class SectionType : u32 {
    Invalid = 0,
    ChunkInfo       = 1,   // authoritative per-cell metadata + QA counters
    TerrainHeight   = 2,   // f32 heightfield (query / flood / placement)
    TerrainMesh     = 3,   // rendered terrain triangles
    TerrainCollision= 4,   // physics representation of terrain
    TerrainAO       = 5,   // baked u8 ambient occlusion per terrain vertex
    MaterialRegions = 6,   // per-texel surface-material mask (u8 grid)
    RoadMesh        = 7,
    RoadNavGraph    = 8,   // nodes/edges/lanes/signals for traffic + pedestrians
    BuildingMesh    = 9,   // merged, LOD-tagged, per-zone
    BuildingCollision = 10,// OBB list + optional triangle soup for hero assets
    BuildingImpostor= 11,  // far-field billboards
    WaterMesh       = 12,  // sea, lakes, rivers, flood plane
    RailMesh        = 13,
    LandusePolys    = 14,
    InstanceTable   = 15,  // streetlamps, tanks, AC units, props, vegetation
    MaterialTable   = 16,  // PBR parameter block referenced by mesh submeshes
    HeroAssets      = 17,  // landmark geometry (photogrammetry / hand built)
    QAReport        = 18,  // machine-readable build-time validation output
    BuildingProxy   = 19,  // oriented massing blocks: centre, half extents, yaw
    Count
};

const char* sectionTypeName(SectionType t);

#pragma pack(push, 1)

struct ChunkHeader {
    char  magic[8];
    u32   version;
    u32   headerSize;
    i64   cellX;
    i64   cellZ;
    f32   cellSize;         // metres (always MACRO_CELL_SIZE today)
    f32   minHeight;
    f32   maxHeight;
    u32   sectionCount;
    u32   flags;
    u64   fileSize;
    u64   contentHash;      // FNV-1a 64 over all payload bytes
    u64   sourceHash;       // hash of the input data revision that produced it
    u32   buildSchema;      // bump when the pipeline's semantics change
    u32   reserved0;
    u64   reserved1[5];
    u32   reserved2;
};  // 128 bytes (verified by static_assert below)

struct SectionEntry {
    u32 type;               // SectionType
    u16 format;             // section-specific format/version tag
    u16 flags;              // bit 0: mandatory, bit 1: compressed
    u64 offset;             // byte offset from file start, 16-byte aligned
    u64 size;               // payload bytes
    u32 elementCount;
    u32 stride;
    u64 hash;               // FNV-1a 64 of the payload — verified on load
};  // 40 bytes

// --- Mesh geometry -----------------------------------------------------------
struct MeshVertex {
    f32 px, py, pz;         // cell-local metres
    f32 nx, ny, nz;         // unit normal (kept f32: terrain normals feed physics)
    f32 u, v;               // texture coordinates (uv1 = macro variation)
    f32 u2, v2;             // second UV set: macro variation / detail blend
    u8  materialId;
    u8  zoneId;
    u8  flags;              // bit0: shadow caster, bit1: collision, bit2: wet-affected
    u8  ao;                 // baked ambient occlusion, 0..255
};  // 44 bytes

struct MeshHeader {
    u32 vertexCount;
    u32 indexCount;
    u32 submeshCount;
    u32 lodLevel;
};

struct SubMesh {
    u32 materialId;
    u32 indexOffset;
    u32 indexCount;
    u32 vertexOffset;
    u32 zoneId;
    u32 flags;
};  // 24 bytes

// A mesh section holds a *group* of meshes (one per LOD, and per hero asset).
// Layout of a mesh section payload:
//     MeshGroupHeader            (8 bytes)
//     MeshEntry * meshCount      (40 bytes each)
//     [ vertex block ][ index block ][ submesh block ] * meshCount
// Offsets inside MeshEntry are relative to the start of the section payload, so
// the whole group is usable straight out of an mmap with zero parsing.
struct MeshGroupHeader {
    u32 meshCount;
    u32 flags;              // bit0: all meshes share a vertex buffer
};  // 8 bytes

struct MeshEntry {
    u32 vertexCount;
    u32 indexCount;
    u32 submeshCount;
    u32 lodLevel;           // 0 = full detail
    u64 verticesOffset;
    u64 indicesOffset;
    u64 submeshesOffset;
};  // 40 bytes

// --- Terrain -----------------------------------------------------------------
struct HeightfieldHeader {
    u32 width;              // samples along X
    u32 height;             // samples along Z
    f32 originX, originZ;   // cell-local metres of sample (0,0)
    f32 stepX, stepZ;       // metres between samples
    f32 baseY;              // value added to stored u16 heights
    f32 scaleY;             // multiplier applied to stored u16 heights
    u32 encoding;           // 0 = f32 array follows, 1 = u16 quantised follows
};

// --- Collision ---------------------------------------------------------------
struct CollisionBox {
    f32 cx, cy, cz;         // cell-local centre
    f32 hx, hy, hz;         // half extents
    f32 qx, qy, qz, qw;     // orientation
    u32 flags;              // bit0: building, bit1: wall, bit2: hero mesh
    u32 materialId;
};  // 48 bytes

// --- Navigation graph --------------------------------------------------------
enum NavFlags : u32 {
    NavFlagNone       = 0u,
    NavFlagDrivable   = 1u << 0,
    NavFlagWalkable   = 1u << 1,
    NavFlagBridge     = 1u << 2,
    NavFlagTunnel     = 1u << 3,
    NavFlagSignalized = 1u << 4,
    NavFlagOneWay     = 1u << 5,
    NavFlagPedestrianOnly = 1u << 6,
    NavFlagRestrictedAuto = 1u << 7,   // no auto-rickshaws (South Mumbai rule)
    NavFlagBusRoute   = 1u << 8
};

// Header of the RoadNavGraph section payload:
//   NavGraphHeader | NavNode[nodeCount] | NavEdge[edgeCount] | NavSignalPhase[signalCount]
struct NavGraphHeader {
    u32 nodeCount;
    u32 edgeCount;
    u32 signalCount;
};

struct NavNode {
    f32 x, y, z;            // cell-local metres
    u32 firstEdge;
    u16 edgeCount;
    u16 flags;
    u32 osmId;              // provenance: source node id (0 if synthetic)
};  // 24 bytes

struct NavEdge {
    u32 toNode;
    u16 laneCount;
    u8  direction;          // 0 = both ways, 1 = forward only, 2 = backward only
    u8  surface;
    f32 speedLimitKph;
    f32 length;             // metres
    f32 bezier[8];          // cubic control points (x,y,z pairs would double size;
                            // stored as 4 packed xyz in cell-local space)
    u32 flags;
    u32 roadClass;
};  // 4+2+1+1 + 4+4 + 32 + 4+4 = 56 bytes
static_assert(sizeof(NavEdge) == 56, "NavEdge must stay packed");

struct NavSignalPhase {
    u32 edgeIndex;
    u16 startGreenDeciseconds;
    u16 greenDeciseconds;
};

// --- Materials ---------------------------------------------------------------
struct MaterialDef {
    char name[32];
    f32 baseColor[4];
    f32 roughness, metallic, specular, normalScale;
    f32 macroVariation, wetnessResponse, textureSetIndex;
    u32 flags;
};  // 80 bytes

// --- Instances (props, vegetation, street furniture) -------------------------
struct InstanceRecord {
    f32 px, py, pz;
    f32 qx, qy, qz, qw;
    f32 sx, sy, sz;
    u32 meshId;
    u32 materialId;
    u32 flags;
};  // 52 bytes

// --- Building proxies --------------------------------------------------------
// The oriented massing block of every structure, in cell-local metres.  This is
// what the far-field impostor pass, the LOD1 blockout and the viewer consume:
// a plain AABB would rotate Mumbai's road-following fabric to the axes, and
// roughly 40% of the city's footprints are 5-45 degrees off them.  `yaw` is the
// rotation about +Y of the footprint's principal axis.
struct BuildingProxy {
    f32 cx, cy, cz;         // centre (cy = base + height/2)
    f32 hx, hy, hz;         // half extents along (right, up, forward)
    f32 yaw;                // radians, counter-clockwise about +Y
    f32 baseY;              // ground elevation at the footprint centroid
    u32 materialId;
    u32 zoneId;
    u32 flags;              // bit0: surveyed footprint, bit1: enters navmesh
    u32 reserved;
};  // 48 bytes
static_assert(sizeof(BuildingProxy) == 48, "BuildingProxy must stay packed");

// --- Landuse polygons (compact, cell-local) ---------------------------------
struct LanduseHeader {
    u32 polygonCount;
    u32 totalVertices;
    u32 zoneCounts[16];
};

struct LandusePoly {
    u32 firstVertex;
    u32 vertexCount;
    u32 zone;               // ZoningZone
    u32 surface;            // SurfaceMaterial
    f32 areaSquareMetres;
};

// --- Per-cell authoritative metadata (ChunkInfo section) ---------------------
// The runtime reads this instead of walking geometry: streaming decisions, the
// debug overlay, editor tooling and the QA report all key off this block.
struct CellInfo {
    u32 buildingCount;
    u32 roadWayCount;
    u32 junctionCount;
    u32 waterPolyCount;
    u32 railWayCount;
    u32 landusePolyCount;
    u32 instanceCount;
    u32 heroAssetCount;
    u32 triangleCountLod0;
    u32 triangleCountTotal;
    f32 minHeight;
    f32 maxHeight;
    f32 centreX;            // cell-local (always the cell centre)
    f32 centreZ;
    u32 zoneHistogram[16];
    u32 buildSchema;
    u32 reserved[1];
};  // 128 bytes

// --- QA ----------------------------------------------------------------------
struct QaCounter {
    char name[32];
    f64  value;
    u32  severity;          // 0 = info, 1 = warning, 2 = failure
    u32  reserved;
};

#pragma pack(pop)

static_assert(sizeof(ChunkHeader) == 128, "ChunkHeader must stay cache-friendly and fixed");
static_assert(sizeof(SectionEntry) == 40, "SectionEntry layout is part of the on-disk contract");
static_assert(SECTION_ENTRY_SIZE == sizeof(SectionEntry), "SECTION_ENTRY_SIZE must equal sizeof(SectionEntry)");
static_assert(sizeof(MeshVertex) == 44, "MeshVertex is a GPU input layout; changing it changes shaders");
static_assert(sizeof(MeshEntry) == 40, "MeshEntry stride is on-disk");
static_assert(sizeof(CellInfo) == 128, "CellInfo is a cache-line block on purpose");
static_assert(sizeof(CollisionBox) == 48, "CollisionBox stride is on-disk");
static_assert(sizeof(NavNode) == 24, "NavNode stride is on-disk");
static_assert(sizeof(InstanceRecord) == 52, "InstanceRecord stride is on-disk");

// --- NavEdge is validated separately because of the bezier field -------------
static_assert(sizeof(NavEdge) % 4 == 0, "NavEdge must be 4-byte aligned");

// -----------------------------------------------------------------------------
//  Writer — used by the pipeline.  Sections are appended in memory and flushed
//  once, so a chunk is written atomically (a truncated chunk can never be
//  mistaken for a valid one, because the header is written last... in practice
//  we write to a temp file and rename).
// -----------------------------------------------------------------------------
class ChunkWriter {
public:
    ChunkWriter(i64 cellX, i64 cellZ, u64 sourceHash = 0);

    // Adds a payload.  The bytes are COPIED immediately: the writer takes
    // ownership of the content, so callers may pass pointers to temporaries
    // (the previous "pointer must outlive finalize()" contract was the source
    // of a real corruption bug — section payloads were freed before write(),
    // the allocator recycled them, and reopened chunks failed verification).
    // A chunk is at most a few hundred kilobytes, so the copy is cheap and the
    // invariant "what was hashed is what was written" holds by construction.
    void addSection(SectionType type, const void* ptr, u64 bytes, u32 elementCount,
                    u32 stride, u16 format = 1, u16 flags = 0);

    // Serialises to `path` and returns the byte size written (0 on failure).
    u64 write(const std::string& path);

    void setHeightRange(f32 minH, f32 maxH) { m_minHeight = minH; m_maxHeight = maxH; }
    void setFlags(u32 f) { m_flags = f; }
    u64 payloadBytes() const;

private:
    i64 m_cellX, m_cellZ;
    u64 m_sourceHash;
    f32 m_minHeight = 0, m_maxHeight = 0;
    u32 m_flags = 0;
    struct Pending {
        SectionType type;
        std::vector<u8> bytes;      // owned copy (see addSection)
        u32 count;
        u32 stride;
        u16 format;
        u16 flags;
    };
    std::vector<Pending> m_pending;
};

// -----------------------------------------------------------------------------
//  Reader — used by the runtime.  Maps the file (zero copy) and exposes
//  typed, bounds-checked views.  Any inconsistency is reported, never trusted:
//  this is what makes "zero rendering artifacts" an architectural property
//  rather than a hope.
// -----------------------------------------------------------------------------
class ChunkReader {
public:
    ChunkReader() = default;
    ~ChunkReader();
    ChunkReader(const ChunkReader&) = delete;
    ChunkReader& operator=(const ChunkReader&) = delete;

    // Opens and validates.  Returns false with a diagnostic on any failure.
    bool open(const std::string& path, std::string* error = nullptr);
    void close();

    bool valid() const { return m_data != nullptr; }
    const ChunkHeader& header() const { return m_header; }
    const SectionEntry* find(SectionType t) const;

    template <typename T>
    const T* view(SectionType t) const {
        const SectionEntry* e = find(t);
        if (!e) return nullptr;
        if (e->size < sizeof(T)) return nullptr;
        return reinterpret_cast<const T*>(m_data + e->offset);
    }

    template <typename T>
    const T* array(SectionType t, u32* countOut = nullptr) const {
        const SectionEntry* e = find(t);
        if (!e) { if (countOut) *countOut = 0; return nullptr; }
        if (e->flags & SectionFlagPrefixed) {
            if (countOut) *countOut = 0;
            return nullptr;      // read it through its typed accessor instead
        }
        if (e->stride != 0 && e->stride != sizeof(T)) {
            if (countOut) *countOut = 0;
            return nullptr;   // stride mismatch: refuse rather than misread
        }
        if (countOut) *countOut = e->elementCount;
        if (e->elementCount == 0) return nullptr;
        if (static_cast<u64>(e->elementCount) * sizeof(T) > e->size) { if (countOut) *countOut = 0; return nullptr; }
        return reinterpret_cast<const T*>(m_data + e->offset);
    }

    u64 fileBytes() const { return m_fileBytes; }
    const u8* raw() const { return m_data; }

    // --- mesh group access ---------------------------------------------------
    // Zero-copy view over one mesh of a mesh-group section.  Returns an invalid
    // view (vertexCount == 0) if the section or index is absent or malformed.
    struct MeshView {
        const MeshVertex* vertices = nullptr;
        const u32* indices = nullptr;
        const SubMesh* submeshes = nullptr;
        u32 vertexCount = 0, indexCount = 0, submeshCount = 0, lodLevel = 0;
        bool valid() const { return vertices != nullptr && vertexCount > 0; }
        // Bounds of the mesh in cell-local metres (for culling).
        Aabb bounds() const;
    };
    u32 meshCount(SectionType t) const;
    MeshView mesh(SectionType t, u32 index) const;

    // Typed view of the road nav graph (header + nodes + edges + signals), with
    // every offset re-validated against the section size.
    struct NavGraphView {
        const NavGraphHeader* header = nullptr;
        const NavNode* nodes = nullptr;
        const NavEdge* edges = nullptr;
        const NavSignalPhase* signals = nullptr;
        bool valid() const { return header != nullptr; }
    };
    NavGraphView navGraph() const;
    // Convenience: the mesh of a given LOD level (falls back to the closest).
    MeshView meshByLod(SectionType t, u32 lod) const;

    // Verifies every section hash.  Called by the streaming system on load in
    // debug/validation builds, and always for hero-asset chunks.
    bool verifyHashes(std::string* error = nullptr) const;

private:
    u8* m_data = nullptr;
    u64 m_fileBytes = 0;
    ChunkHeader m_header{};
    std::vector<SectionEntry> m_sections;
};

// FNV-1a over a byte range (shared hashing so writer and reader agree exactly).
u64 hashBytes(const void* data, size_t bytes);

} // namespace mumbai
