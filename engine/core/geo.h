// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/geo.h
//
//  Geographic projection and the floating-origin coordinate system
//  (specification Section 4 — "the most technically critical system").
//
//  PROJECTION
//  ----------
//  Source geodata is WGS84 geographic (degrees). The world is stored in metres
//  on a Transverse Mercator grid whose central meridian is the Mumbai meridian
//  (72.9 E) and whose latitude of origin is 18.9 N, with scale factor 1.0.
//  The implementation uses Krueger's n-series to order 6 (Karney 2011), which
//  is accurate to well under a millimetre over the ~50 km map extent — orders
//  of magnitude tighter than the 1 m vertical / 2 m horizontal data accuracy
//  the completion criteria demand. A conformal projection is used rather than
//  a simple equirectangular one so that shape, angle and local scale are
//  preserved: road widths and building footprints stay true everywhere on the
//  map, which a plate-carree would distort by ~5% at 25 km from the origin.
//
//  FLOATING ORIGIN
//  ---------------
//  The world is divided into 1024 m macro-cells. Every entity stores an integer
//  (cellX, cellZ) plus a local f32 offset that never exceeds the cell size,
//  which bounds local f32 ulp at 1024 m to 6.1e-5 m (0.061 mm). Render-space
//  coordinates are produced relative to the camera's cell and are additionally
//  clamped by the LOD system to a 4096 m radius for full-detail geometry, so
//  even the outermost rendered vertex keeps sub-millimetre granularity.
// =============================================================================
#pragma once
#include "engine/core/math.h"

namespace mumbai {

// -----------------------------------------------------------------------------
//  Projection constants.  These are compiled-in rather than configurable
//  because every chunk on disk is expressed in this frame: changing them
//  silently would invalidate the entire 150 GB world.
// -----------------------------------------------------------------------------
struct ProjectionParams {
    f64 a  = 6378137.0;             // WGS84 semi-major axis
    f64 f  = 1.0 / 298.257223563;   // WGS84 flattening
    f64 latOriginDeg = 18.9000;     // Mumbai, roughly the BMC southern datum
    f64 lonOriginDeg = 72.9000;     // Mumbai central meridian
    f64 k0 = 1.0;                   // scale factor on the central meridian
    f64 falseEasting  = 0.0;
    f64 falseNorthing = 0.0;
};

// Transverse Mercator, Krueger series to order n^6 (~nanometre class on the
// central meridian, <1 mm across the map extent). Easting -> +X, Northing -> +Y.
class TransverseMercator {
public:
    explicit TransverseMercator(const ProjectionParams& p = {});

    // WGS84 degrees -> local metres.  X = easting, Y = northing.
    Vec2d forward(f64 latDeg, f64 lonDeg) const;
    // Local metres -> WGS84 degrees.
    void inverse(f64 x, f64 y, f64& latDeg, f64& lonDeg) const;

    // Local scale factor / meridian convergence at a point (used to validate
    // that the projection has not been silently misparameterised).
    f64 scaleAt(f64 latDeg, f64 lonDeg) const;

    const ProjectionParams& params() const { return m_p; }

private:
    ProjectionParams m_p;
    f64 m_n, m_A, m_e;
    f64 m_alpha[7]{}, m_beta[7]{};
};

// -----------------------------------------------------------------------------
//  World extent.  Derived from the BMC administrative boundary at build time
//  (see DOCS/01_DATA_SOURCES.md): approx 603 km^2, which is the "~600 square
//  kilometres" the specification calls for.
// -----------------------------------------------------------------------------
struct WorldExtent {
    Vec2d min{ 0, 0 };        // metres, projection frame
    Vec2d max{ 0, 0 };
    Vec2d size() const { return max - min; }
    Vec2d center() const { return (min + max) * 0.5; }
    bool contains(const Vec2d& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
    }
};

// -----------------------------------------------------------------------------
//  Global position — the integer-cell + local-offset pair from Section 4A.
// -----------------------------------------------------------------------------
inline constexpr f32 MACRO_CELL_SIZE = 1024.0f;   // metres

struct GlobalPosition {
    i64 cellX = 0;
    i64 cellZ = 0;
    Vec3 localOffset{0, 0, 0};   // always within [0, MACRO_CELL_SIZE)

    // Re-normalises the local offset into cell bounds, carrying into the cell
    // indices.  Uses a single correction step rather than a loop for the
    // common case, with a loop retained for pathological inputs (teleports).
    void normalize() {
        const f32 cs = MACRO_CELL_SIZE;
        if (localOffset.x >= cs || localOffset.x < 0.0f) {
            const f64 cx = std::floor(static_cast<f64>(localOffset.x) / cs);
            cellX += static_cast<i64>(cx);
            localOffset.x = static_cast<f32>(localOffset.x - cx * cs);
        }
        if (localOffset.z >= cs || localOffset.z < 0.0f) {
            const f64 cz = std::floor(static_cast<f64>(localOffset.z) / cs);
            cellZ += static_cast<i64>(cz);
            localOffset.z = static_cast<f32>(localOffset.z - cz * cs);
        }
        if (localOffset.x >= cs) localOffset.x = std::nextafter(cs, 0.0f);
        if (localOffset.z >= cs) localOffset.z = std::nextafter(cs, 0.0f);
    }

    // Full double-precision world position in metres (pipeline / query space).
    Vec2d toWorld() const {
        return { static_cast<f64>(cellX) * MACRO_CELL_SIZE + localOffset.x,
                 static_cast<f64>(cellZ) * MACRO_CELL_SIZE + localOffset.z };
    }
    f64 worldY() const { return localOffset.y; }

    static GlobalPosition fromWorld(const Vec2d& world, f64 y = 0.0) {
        GlobalPosition gp;
        gp.cellX = static_cast<i64>(std::floor(world.x / MACRO_CELL_SIZE));
        gp.cellZ = static_cast<i64>(std::floor(world.y / MACRO_CELL_SIZE));
        gp.localOffset = {
            static_cast<f32>(world.x - static_cast<f64>(gp.cellX) * MACRO_CELL_SIZE),
            static_cast<f32>(y),
            static_cast<f32>(world.y - static_cast<f64>(gp.cellZ) * MACRO_CELL_SIZE)
        };
        gp.normalize();
        return gp;
    }

    // Render-space position relative to a camera cell.  Computed in f64 and
    // narrowed once, so the error is a single f32 rounding of a small number
    // (<= ~4096 m in practice) rather than an accumulation.
    Vec3 toRenderSpace(i64 cameraCellX, i64 cameraCellZ, f64 cameraY = 0.0) const {
        const f64 dx = static_cast<f64>(cellX - cameraCellX) * MACRO_CELL_SIZE + localOffset.x;
        const f64 dz = static_cast<f64>(cellZ - cameraCellZ) * MACRO_CELL_SIZE + localOffset.z;
        return { static_cast<f32>(dx), static_cast<f32>(localOffset.y - cameraY), static_cast<f32>(dz) };
    }

    f64 distanceTo(const GlobalPosition& o) const {
        const Vec2d a = toWorld(), b = o.toWorld();
        const f64 dx = a.x - b.x, dz = a.y - b.y;
        const f64 dy = localOffset.y - o.localOffset.y;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Cell-relative offset of this position inside its own cell.
    Vec2 cellLocal() const { return { localOffset.x, localOffset.z }; }
};

// -----------------------------------------------------------------------------
//  FloatingOriginManager — Section 4B.  Shifts the world so that the camera is
//  always near the local origin, atomically, within one frame, across physics,
//  rendering, particles and audio.
// -----------------------------------------------------------------------------
struct OriginShift {
    i64 deltaCellX = 0;
    i64 deltaCellZ = 0;
    Vec3 deltaLocal{0, 0, 0};   // metres to subtract from every local offset
    bool active() const { return deltaCellX != 0 || deltaCellZ != 0 ||
                                 deltaLocal.x != 0 || deltaLocal.z != 0; }
};

class FloatingOriginManager {
public:
    // Returns the shift that must be applied this frame (inactive if none).
    OriginShift update(const GlobalPosition& cameraPos);

    // Applies a shift to a single position.  Kept exact: the subtraction is
    // performed in f64 and then re-normalised, so repeated shifts cannot drift.
    static void applyShift(GlobalPosition& p, const OriginShift& s);

    i64 currentCellX() const { return m_cellX; }
    i64 currentCellZ() const { return m_cellZ; }
    void setCurrentCell(i64 x, i64 z) { m_cellX = x; m_cellZ = z; m_initialized = true; }
    bool initialized() const { return m_initialized; }
    u64 shiftCount() const { return m_shiftCount; }

private:
    i64 m_cellX = 0, m_cellZ = 0;
    bool m_initialized = false;
    u64 m_shiftCount = 0;
};

// -----------------------------------------------------------------------------
//  Zoning taxonomy (Section 2A) and material taxonomy, shared by the pipeline
//  and the runtime so that a chunk written by one is interpretable by the other.
// -----------------------------------------------------------------------------
enum class ZoningZone : u8 {
    Unknown = 0,
    SouthBombayColonial,     // Fort / Colaba heritage, Indo-Saracenic & Gothic
    BKCCommercial,           // contemporary corporate curtain wall
    SuburbanResidential,     // Andheri..Borivali mid-rise
    InformalSettlement,      // Dharavi, Mithi river settlements
    Industrial,              // MIDC estates, docks
    Institutional,           // hospitals, campuses, government
    Airport,                 // CSMIA
    MangroveWetland,         // Thane creek / Mahim mangroves
    Hillside,                // Malabar, Pali, Gilbert
    Count
};
const char* zoningName(ZoningZone z);

enum class SurfaceMaterial : u8 {
    Unknown = 0,
    AsphaltSmooth,       // highways, major arterials
    AsphaltPatched,      // suburban roads, repair work
    AsphaltSeaLink,      // high-grip, painted markings
    ConcreteRoad,        // flyovers, bridges, decks
    PaverBlock,          // heritage precincts, plazas
    Gravel,              // unpaved service roads
    Dirt,                // informal settlement lanes
    Rock,                // Malabar / Gilbert basalt
    Earth,               // compacted ground
    Sand,                // Juhu, Versova beaches
    WetlandMud,          // mangroves
    ReclamationFill,     // Nariman Point, Cuffe Parade, BKC
    Grass,
    Count
};
const char* surfaceName(SurfaceMaterial m);

enum class BuildingMaterial : u8 {
    Unknown = 0,
    PlasterPainted,      // suburban residential
    StoneAshlar,         // colonial heritage
    StoneRubble,
    CurtainWallGlass,    // BKC towers
    BrickExposed,
    CorrugatedMetal,     // industrial sheds, informal roofing
    ConcreteRaw,
    Count
};
const char* buildingMaterialName(BuildingMaterial m);

} // namespace mumbai
