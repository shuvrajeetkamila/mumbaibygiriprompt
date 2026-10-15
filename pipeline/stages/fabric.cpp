// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/fabric.cpp
//
//  Stage group 3: the built fabric.
//    * buildBuildings — footprints -> procedural architecture per zoning zone
//                       (Section 2A rulesets), with an LOD chain and collision
//    * buildProps     — water tanks, AC units, trees, streetlamps, parked
//                       two-wheelers, and the Queen's Necklace light array
//    * buildHeroAssets— Gateway of India, Taj, CSMT, Bandra-Worli Sea Link,
//                       Marine Drive promenade
//
//  The specification asks for a Houdini-based procedural system or an
//  equivalent native grammar. This is the native grammar (decision D-007):
//  deterministic, dependency-free, and fast enough to process the whole city in
//  one offline pass.
// =============================================================================
#include "pipeline/pipeline.h"

namespace mumbai::pipeline {

namespace {

// --- zoning-driven architectural rules ---------------------------------------
struct ArchitectureRule {
    BuildingMaterial wall;
    f64 minHeight, maxHeight;      // metres
    f64 floorHeight = 3.2;
    f64 parapet = 0.9;
    bool pitchedRoof = false;
    bool groundFloorShops = false;
};

ArchitectureRule ruleForZone(ZoningZone z, const std::string& wardCode) {
    ArchitectureRule r{};
    switch (z) {
        case ZoningZone::SouthBombayColonial:
            r.wall = BuildingMaterial::StoneAshlar;
            r.minHeight = 12.0; r.maxHeight = 22.0;
            r.floorHeight = 4.2;
            r.parapet = 0.6;
            r.pitchedRoof = true;
            r.groundFloorShops = true;
            break;
        case ZoningZone::BKCCommercial:
            r.wall = BuildingMaterial::CurtainWallGlass;
            r.minHeight = 50.0; r.maxHeight = 120.0;
            r.floorHeight = 4.0;
            r.parapet = 1.2;
            break;
        case ZoningZone::InformalSettlement:
            r.wall = BuildingMaterial::CorrugatedMetal;
            r.minHeight = 3.2; r.maxHeight = 9.6;
            r.floorHeight = 3.2;
            r.parapet = 0.15;
            r.groundFloorShops = true;
            break;
        case ZoningZone::Industrial:
            r.wall = BuildingMaterial::CorrugatedMetal;
            r.minHeight = 6.0; r.maxHeight = 16.0;
            r.floorHeight = 5.0;
            r.parapet = 0.5;
            break;
        case ZoningZone::Hillside:
            r.wall = BuildingMaterial::ConcreteRaw;
            r.minHeight = 6.0; r.maxHeight = 14.0;
            r.parapet = 0.8;
            break;
        case ZoningZone::Airport:
            r.wall = BuildingMaterial::ConcreteRaw;
            r.minHeight = 8.0; r.maxHeight = 22.0;
            r.floorHeight = 6.0;
            r.parapet = 0.6;
            break;
        case ZoningZone::Institutional:
            r.wall = BuildingMaterial::PlasterPainted;
            r.minHeight = 10.0; r.maxHeight = 26.0;
            r.parapet = 0.8;
            break;
        case ZoningZone::SuburbanResidential:
        default:
            // The specification's ward-keyed defaults: post-liberalisation
            // construction in the eastern suburbs runs taller than the older
            // western residential fabric.
            r.wall = BuildingMaterial::PlasterPainted;
            if (wardCode == "K/E" || wardCode == "K/W" || wardCode == "L" ||
                wardCode == "H/E" || wardCode == "H/W" || wardCode == "S") {
                r.minHeight = 15.0; r.maxHeight = 45.0;
            } else {
                r.minHeight = 9.0; r.maxHeight = 25.0;
            }
            r.parapet = 0.9;
            r.groundFloorShops = true;
            break;
    }
    return r;
}

// Deterministic per-building material variation. The spec requires that "no two
// buildings on a street look identical despite sharing the same architectural
// grammar", so the seed is the footprint hash: stable across rebuilds, unique
// per building, and independent of iteration order.
struct FacadeVariation {
    f32 tintR, tintG, tintB;
    f32 weathering;
    f32 laundry;       // dangling laundry / clutter density
    u32 windowPattern;
};

FacadeVariation variationFor(u64 seed, ZoningZone zone) {
    Rng rng(seed);
    FacadeVariation v{};
    switch (zone) {
        case ZoningZone::SuburbanResidential: {
            // The palette the specification names explicitly: cream, pale blue,
            // mint green, salmon pink, ochre yellow.
            static const f32 palette[5][3] = {
                {0.86f, 0.81f, 0.68f},   // cream
                {0.68f, 0.78f, 0.85f},   // pale blue
                {0.72f, 0.86f, 0.76f},   // mint
                {0.88f, 0.66f, 0.58f},   // salmon
                {0.84f, 0.72f, 0.42f},   // ochre
            };
            const int p = static_cast<int>(rng.nextU32() % 5);
            v.tintR = palette[p][0]; v.tintG = palette[p][1]; v.tintB = palette[p][2];
            v.weathering = rng.range(0.25f, 0.85f);
            v.laundry = rng.nextF32();
            v.windowPattern = rng.nextU32() % 4;
            break;
        }
        case ZoningZone::SouthBombayColonial:
            v.tintR = 0.72f; v.tintG = 0.64f; v.tintB = 0.52f;
            v.weathering = rng.range(0.35f, 0.9f);
            v.laundry = rng.nextF32() * 0.4f;
            v.windowPattern = rng.nextU32() % 3;
            break;
        case ZoningZone::InformalSettlement:
            v.tintR = 0.55f; v.tintG = 0.55f; v.tintB = 0.57f;
            v.weathering = rng.range(0.55f, 1.0f);
            v.laundry = rng.nextF32();
            v.windowPattern = rng.nextU32() % 2;
            break;
        case ZoningZone::BKCCommercial:
            v.tintR = rng.range(0.42f, 0.56f);   // narrow blue-green glass range
            v.tintG = rng.range(0.52f, 0.64f);
            v.tintB = rng.range(0.58f, 0.70f);
            v.weathering = rng.range(0.02f, 0.18f);
            v.laundry = 0.0f;
            v.windowPattern = rng.nextU32() % 5;
            break;
        default:
            v.tintR = v.tintG = v.tintB = 0.7f;
            v.weathering = 0.3f;
            v.laundry = 0.2f;
            v.windowPattern = rng.nextU32() % 3;
            break;
    }
    return v;
}

// Applies a facade tint + weathering to a material definition, producing a
// per-zone material variant. Bounded so the material table cannot explode: the
// world ships a fixed palette of variants, not one per building.
// -----------------------------------------------------------------------------
//  Oriented massing proxy.
//  The collision box that physics uses is an AABB in the cell frame, which is
//  the right shape for a broadphase but loses the footprint's orientation. The
//  proxy keeps it: the principal axis of the footprint ring gives the yaw, and
//  the extents are the ring's projection onto that axis and its perpendicular.
//  Roughly 40% of Mumbai's surveyed footprints are 5-45 degrees off the cell
//  axes (every road that is not a cardinal grid street), so this is what keeps
//  the massing legible instead of snapping the whole city to a lattice.
// -----------------------------------------------------------------------------
// The rectangle that fits a footprint ring, in the footprint's own frame: its
// centre, the angle of its principal axis, and its half extents along that axis
// and across it.  One function computes it, so the massing the player sees, the
// collision proxy that stops a vehicle and the impostor that stands in for the
// block at 2 km cannot disagree about where the building is — the first version
// of this stage gave the *collision* the axis-aligned bounding box while giving
// the massing the fitted rectangle, which put a 45-degree block's corners 3-4 m
// into the street it stands beside (D-034's lane survey found 12 % of the model's
// own lane points blocked by geometry the two layers otherwise agreed about).
struct Massing {
    Vec2d centre{};
    f64 axisFromX = 0.0;  // angle of the principal axis measured from +X, radians
    f64 yaw = 0.0;      // the same axis in the project's convention: atan2(dx, dz)
    f64 along = 0.0;    // half extent along the principal axis
    f64 across = 0.0;   // half extent perpendicular to it
    Vec2d bbMin{}, bbMax{};
};

Massing massingOf(const geo2d::Ring& ring, const Vec2d& centre) {
    Massing m;
    m.bbMin = m.bbMax = centre;
    if (ring.size() < 3) return m;

    // Second moments about the centroid -> principal axis, in the (x, z) plane.
    f64 sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (const Vec2d& p : ring) {
        const f64 dx = p.x - centre.x, dy = p.y - centre.y;
        sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
        m.bbMin.x = std::min(m.bbMin.x, p.x); m.bbMax.x = std::max(m.bbMax.x, p.x);
        m.bbMin.y = std::min(m.bbMin.y, p.y); m.bbMax.y = std::max(m.bbMax.y, p.y);
    }
    const f64 n = static_cast<f64>(ring.size());
    sxx /= n; syy /= n; sxy /= n;
    f64 phi = 0.0;
    if (std::abs(sxy) > 1e-9 || std::abs(sxx - syy) > 1e-9)
        phi = 0.5 * std::atan2(2.0 * sxy, sxx - syy);

    const f64 c = std::cos(phi), sn = std::sin(phi);
    f64 minU = 1e30, maxU = -1e30, minV = 1e30, maxV = -1e30;
    for (const Vec2d& p : ring) {
        const f64 dx = p.x - centre.x, dy = p.y - centre.y;
        const f64 u = dx * c + dy * sn;
        const f64 v = -dx * sn + dy * c;
        minU = std::min(minU, u); maxU = std::max(maxU, u);
        minV = std::min(minV, v); maxV = std::max(maxV, v);
    }
    m.along = (maxU - minU) * 0.5;
    m.across = (maxV - minV) * 0.5;
    if (!(m.along > 0.5)) m.along = std::max(m.along, 0.5);
    if (!(m.across > 0.5)) m.across = std::max(m.across, 0.5);
    // Re-centre on the fitted rectangle so the block is not offset from its hull.
    const f64 mu = (minU + maxU) * 0.5, mv = (minV + maxV) * 0.5;
    m.centre = Vec2d{centre.x + mu * c - mv * sn, centre.y + mu * sn + mv * c};
    // `phi` measures the axis from +X; the project's yaw measures it from +Z with
    // `atan2(dx, dz)`, which is what `emitProxy`'s consumer and the physics
    // quaternion both use: q = (0, sin(yaw/2), 0, cos(yaw/2)) puts the box's own
    // +Z along the axis.
    m.axisFromX = phi;
    m.yaw = std::atan2(c, sn);
    return m;
}

void emitProxy(CellData& cd, const geo2d::Ring& ring, const Vec2d& centre, f64 groundY,
               f64 height, u32 materialId, u32 zoneId, bool surveyed) {
    if (ring.size() < 3 || !(height > 0.5)) return;

    const Massing ms = massingOf(ring, centre);
    const f64 hx = ms.along, hz = ms.across;
    const Vec2d fitted = ms.centre;

    BuildingProxy bp{};
    bp.cx = static_cast<f32>(fitted.x - cd.origin.x);
    bp.cy = static_cast<f32>(groundY + height * 0.5);
    bp.cz = static_cast<f32>(fitted.y - cd.origin.y);
    bp.hx = static_cast<f32>(hx);
    bp.hy = static_cast<f32>(height * 0.5);
    bp.hz = static_cast<f32>(hz);
    // The proxy keeps the angle-from-+X it has always carried (its consumer is the
    // renderer, which owns that convention); the collision quaternion is built in
    // `collisionBoxFor` from this same rectangle, so the two cannot drift apart.
    bp.yaw = static_cast<f32>(ms.axisFromX);
    bp.baseY = static_cast<f32>(groundY);
    bp.materialId = materialId;
    bp.zoneId = zoneId;
    bp.flags = surveyed ? 1u : 0u;
    cd.proxies.push_back(bp);
    cd.counts.proxies++;
}

u32 materialVariant(Context& ctx, const char* baseName, const FacadeVariation& v,
                    ZoningZone zone) {
    static std::unordered_map<std::string, u32> cache;
    char key[96];
    const int bucket = static_cast<int>(std::min(7.0f, v.weathering * 8.0f));
    const int tint = (static_cast<int>(v.tintR * 7.0f) << 6) |
                     (static_cast<int>(v.tintG * 7.0f) << 3) |
                     static_cast<int>(v.tintB * 7.0f);
    std::snprintf(key, sizeof(key), "%s|%d|%d|%d", baseName, bucket, tint,
                  static_cast<int>(zone));
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;

    const u32 baseId = ctx.materials.id(baseName);
    const std::vector<MaterialDef>& all = ctx.materials.all();
    if (baseId == 0 || baseId >= all.size()) return baseId;
    MaterialDef def = all[baseId];
    def.baseColor[0] = clamp(v.tintR, 0.02f, 1.0f);
    def.baseColor[1] = clamp(v.tintG, 0.02f, 1.0f);
    def.baseColor[2] = clamp(v.tintB, 0.02f, 1.0f);
    // Weathering raises roughness and adds macro variation (paint peeling,
    // water staining) — the specification's age parameter.
    def.roughness = clamp(def.roughness + v.weathering * 0.12f, 0.0f, 1.0f);
    def.macroVariation = clamp(def.macroVariation + v.weathering * 0.3f, 0.0f, 1.0f);
    const u32 id = ctx.materials.add(def);
    cache.emplace(key, id);
    return id;
}

// A building's collision box: the same fitted rectangle as its massing, turned to
// the footprint's own axis.  `flags` is the collider flag word the A2 harness
// reads (bit0 building); `height` runs from `groundY` up, as the massing does.
CollisionBox collisionBoxFor(const CellData& cd, const geo2d::Ring& ring, const Vec2d& centre,
                             f64 groundY, f64 height, u32 materialId, u32 flags) {
    const Massing ms = massingOf(ring, centre);
    CollisionBox cb{};
    cb.cx = static_cast<f32>(ms.centre.x - cd.origin.x);
    cb.cy = static_cast<f32>(groundY + height * 0.5);
    cb.cz = static_cast<f32>(ms.centre.y - cd.origin.y);
    // Local +Z runs along the principal axis (half extent `along`), local +X
    // across it (half extent `across`): the fitted rectangle, not its bounding box.
    cb.hx = static_cast<f32>(ms.across);
    cb.hy = static_cast<f32>(std::max(0.5, height * 0.5));
    cb.hz = static_cast<f32>(ms.along);
    const f64 half = ms.yaw * 0.5;
    cb.qx = 0.0f;
    cb.qy = static_cast<f32>(std::sin(half));
    cb.qz = 0.0f;
    cb.qw = static_cast<f32>(std::cos(half));
    cb.flags = flags;
    cb.materialId = materialId;
    return cb;
}

} // namespace

// -----------------------------------------------------------------------------
//  buildBuildings
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//  Synthesised fabric.  The parcels were produced by the street-lattice
//  generator (pipeline/stages/corridors.cpp) inside the real ward and slum
//  polygons; this is where they become architecture.  Massing is deliberately
//  cheap — a wall prism plus a roof cap and an impostor — because the quantity
//  is what matters at city scale and the LOD chain already collapses detail
//  beyond 500 m.  Material, storey count, parapet and roof form all come from
//  the same per-zone rules as the surveyed footprints, so a Dharavi parcel and
//  a Dharavi building recorded in OSM end up looking like neighbours.
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//  Roads are not building sites.
// -----------------------------------------------------------------------------
// The parcels come from a ward lattice (corridors.cpp) that does not know where
// the OSM road network actually runs, and this stage used to place a building on
// every parcel with area.  Measured on the published world: **14.89 % of the nav
// graph's nodes sit inside a building box** (18,515 of 124,348), 11,345 of them
// inside boxes 8 m or taller.  That single conflict is behind the physics
// acceptance's one red check (a motorcycle's 0.0370 m contact with the city,
// D-034), 1.52 % of lane points where a car cannot be placed and 8.2 % of lane
// points with no collision surface under them (D-036, whose ray trace showed a
// ray started inside a solid at every one of them).  A street with a building
// standing on it is not a street.
//
// The test is the fitted massing — the same rectangle that becomes the collision
// box and the wall prism — against the nav graph's own corridors, with the
// carriageway's half width (`lanes x 3.5 / 2`) plus a quarter metre.  A parcel
// whose building would stand in the road is skipped and counted.
struct RoadCorridorIndex {
    struct Seg { Vec2d a, b; f64 half; };
    std::vector<Seg> segs;
    std::unordered_map<u64, std::vector<u32>> buckets;
    static constexpr f64 kCell = 64.0;
    static u64 key(i64 i, i64 j) {
        return (static_cast<u64>(static_cast<u32>(static_cast<i32>(i))) << 32) |
               static_cast<u32>(static_cast<i32>(j));
    }
    void add(const Vec2d& a, const Vec2d& b, f64 half) {
        const u32 idx = static_cast<u32>(segs.size());
        segs.push_back(Seg{a, b, half});
        const f64 reach = half + 0.5;
        const i64 i0 = static_cast<i64>(std::floor((std::min(a.x, b.x) - reach) / kCell));
        const i64 i1 = static_cast<i64>(std::floor((std::max(a.x, b.x) + reach) / kCell));
        const i64 j0 = static_cast<i64>(std::floor((std::min(a.y, b.y) - reach) / kCell));
        const i64 j1 = static_cast<i64>(std::floor((std::max(a.y, b.y) + reach) / kCell));
        for (i64 j = j0; j <= j1; ++j)
            for (i64 i = i0; i <= i1; ++i) buckets[key(i, j)].push_back(idx);
    }
    // Segment against an oriented rectangle: both are put in the rectangle's own
    // frame and clipped with Liang-Barsky, so a diagonal street and an axis-fitted
    // block are the same problem.
    bool hits(const Massing& ms) const {
        const Vec2d along{std::sin(ms.yaw), std::cos(ms.yaw)};
        const Vec2d across{along.y, -along.x};
        const f64 reach = std::max(ms.along, ms.across) + 8.0;
        const i64 i0 = static_cast<i64>(std::floor((ms.centre.x - reach) / kCell));
        const i64 i1 = static_cast<i64>(std::floor((ms.centre.x + reach) / kCell));
        const i64 j0 = static_cast<i64>(std::floor((ms.centre.y - reach) / kCell));
        const i64 j1 = static_cast<i64>(std::floor((ms.centre.y + reach) / kCell));
        for (i64 j = j0; j <= j1; ++j) {
            for (i64 i = i0; i <= i1; ++i) {
                const auto it = buckets.find(key(i, j));
                if (it == buckets.end()) continue;
                for (u32 si : it->second) {
                    const Seg& sg = segs[si];
                    const f64 pad = sg.half + 0.25;
                    const f64 bu = ms.along + pad;      // local +Z half extent
                    const f64 bv = ms.across + pad;     // local +X half extent
                    f64 v0 = 0.0, u0 = 0.0, v1 = 0.0, u1 = 0.0;
                    const Vec2d da = sg.a - ms.centre;
                    const Vec2d db = sg.b - ms.centre;
                    v0 = da.x * across.x + da.y * across.y;
                    u0 = da.x * along.x + da.y * along.y;
                    v1 = db.x * across.x + db.y * across.y;
                    u1 = db.x * along.x + db.y * along.y;
                    f64 t0 = 0.0, t1 = 1.0;
                    const f64 dv = v1 - v0, du = u1 - u0;
                    bool inside = true;
                    const f64 p[4] = {-dv, dv, -du, du};
                    const f64 q[4] = {v0 + bv, bv - v0, u0 + bu, bu - u0};
                    for (int e = 0; e < 4 && inside; ++e) {
                        if (std::fabs(p[e]) < 1e-12) {
                            if (q[e] < 0.0) inside = false;
                            continue;
                        }
                        const f64 r = q[e] / p[e];
                        if (p[e] < 0.0) {
                            if (r > t1) inside = false;
                            else if (r > t0) t0 = r;
                        } else {
                            if (r < t0) inside = false;
                            else if (r < t1) t1 = r;
                        }
                    }
                    if (inside && t0 <= t1) return true;
                }
            }
        }
        return false;
    }
};

// Every drivable nav edge in the city, once, in world space.  Built after
// `buildRoads` (the nav graph exists) and before any building is placed.
RoadCorridorIndex buildRoadCorridorIndex(const Context& ctx) {
    RoadCorridorIndex roads;
    for (const auto& [cellKey, cd] : ctx.cells) {
        (void)cellKey;
        for (const NavNode& n : cd.navNodes) {
            for (u32 k = 0; k < n.edgeCount; ++k) {
                const u32 ei = n.firstEdge + k;
                if (ei >= cd.navEdges.size()) break;
                const NavEdge& e = cd.navEdges[ei];
                if (e.toNode >= cd.navNodes.size()) continue;
                if ((e.flags & NavFlagDrivable) == 0u) continue;
                const NavNode& t = cd.navNodes[e.toNode];
                const Vec2d a{cd.origin.x + n.x, cd.origin.y + n.z};
                const Vec2d b{cd.origin.x + t.x, cd.origin.y + t.z};
                roads.add(a, b, std::max(0.9, e.laneCount * 3.5 * 0.5));
            }
        }
    }
    return roads;
}

void emitSyntheticFabric(Context& ctx, const RoadCorridorIndex& roads) {
    if (ctx.synthParcels.empty()) return;
    u64 skippedOnRoad = 0, insetForRoad = 0;

    const u32 matRoof = ctx.materials.id("concrete_raw");
    std::unordered_map<std::string, std::string> wardCache;
    auto wardCodeFor = [&](const Vec2d& p) -> std::string {
        char key[64];
        std::snprintf(key, sizeof(key), "%lld_%lld",
                      static_cast<long long>(p.x / 500.0), static_cast<long long>(p.y / 500.0));
        auto it = wardCache.find(key);
        if (it != wardCache.end()) return it->second;
        std::string code;
        for (const auto& f : ctx.wardFeatures) {
            if (geo2d::polygonContains(f.polygon, p)) {
                auto n = f.properties.find("name");
                if (n != f.properties.end()) code = n->second;
                break;
            }
        }
        wardCache.emplace(key, code);
        return code;
    };

    u64 built = 0;
    for (const Context::SynthParcel& parcel : ctx.synthParcels) {
        const geo2d::Ring& ring = parcel.ring;
        if (ring.size() < 3) continue;
        const geo2d::Aabb2d bb = geo2d::bounds(ring);
        const f64 area = std::fabs(geo2d::signedArea(ring));
        if (area < 16.0) continue;

        const Vec2d centre = parcel.centre;
        if (!ctx.cityMask.contains(centre)) continue;

        // The road test comes before anything expensive, and it tries to *keep*
        // the parcel: a building whose fitted rectangle reaches into the
        // carriageway is inset — the same operation the roof cap uses
        // (`geo2d::offsetRing`) — until it clears or it is too small to be a
        // building at all.  Dropping every parcel that touches a corridor is
        // correct but wasteful: measured, 50,097 of 163,971 parcels (30 % of the
        // fabric) have their fitted rectangle within a lane width of a road
        // centre, and most of those are ordinary plots that simply reach the kerb.
        geo2d::Ring buildRing = ring;
        {
            bool cleared = false;
            const f64 insets[4] = {0.0, 1.0, 2.0, 3.0};
            for (const f64 inset : insets) {
                const geo2d::Ring candidate =
                    inset <= 0.0 ? ring : geo2d::offsetRing(ring, inset);
                if (candidate.size() < 3) break;
                if (std::fabs(geo2d::signedArea(candidate)) < 16.0) break;
                if (!roads.hits(massingOf(candidate, centre))) {
                    buildRing = candidate;
                    cleared = true;
                    if (inset > 0.0) ++insetForRoad;
                    break;
                }
            }
            if (!cleared) {
                ++skippedOnRoad;
                continue;
            }
        }

        ZoningZone zone = ctx.zoning.sample(centre);
        if (parcel.kind == 1) zone = ZoningZone::InformalSettlement;
        if (zone == ZoningZone::MangroveWetland || zone == ZoningZone::Unknown) continue;

        const ArchitectureRule rule = ruleForZone(zone, wardCodeFor(centre));
        const u64 seed = fnv1a64(static_cast<u64>(static_cast<i64>(centre.x * 32.0)) * 0x2545F4914F6CDD1Dull ^
                                 static_cast<u64>(static_cast<i64>(centre.y * 32.0)));
        Rng rng(seed ^ 0xA11CE);
        const FacadeVariation var = variationFor(seed, zone);

        f64 height = rng.range(rule.minHeight, rule.maxHeight);
        // Storey snapping keeps the silhouette legible at distance and stops
        // the skyline from looking like noise.
        const f64 floors = std::max(1.0, std::round(height / rule.floorHeight));
        height = floors * rule.floorHeight;
        // Small plots are small buildings: scale the low end by plot area so a
        // 6 m x 8 m parcel does not carry a 20-storey tower.
        if (area < 200.0) height = std::min(height, rule.minHeight + (rule.maxHeight - rule.minHeight) * 0.35);
        if (height < 3.0) height = 3.0;

        const f64 groundY = ctx.terrainHF.sample(centre.x, centre.y);
        const f64 topY = groundY + height;
        const u8 flags = 1;

        CellData* cdp = ctx.tryCell(centre);
        if (!cdp) continue;
        CellData& cd = *cdp;
        const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};

        std::vector<Vec2d> localRing;
        localRing.reserve(buildRing.size());
        for (const Vec2d& p : buildRing)
            localRing.push_back(Vec2d{p.x - cd.origin.x, p.y - cd.origin.y});

        const u32 wallMat = materialVariant(ctx, buildingMaterialName(rule.wall), var, zone);

        mesh::MeshBuilder& lod0 = cd.buildings[0];
        const size_t trisBefore = lod0.triangleCount();
        lod0.beginSubmesh(wallMat, static_cast<u32>(zone));
        lod0.extrudeWalls(localRing, groundY - 0.4, topY, wallMat, static_cast<u32>(zone), flags, 0.3f);
        lod0.endSubmesh();
        lod0.beginSubmesh(matRoof, static_cast<u32>(zone));
        if (rule.pitchedRoof && area < 900.0) {
            const f64 ridgeY = topY + std::min(3.0, (bb.size().x + bb.size().y) * 0.05);
            std::vector<Vec2d> inset;
            for (const Vec2d& p : geo2d::offsetRing(buildRing, -1.0))
                inset.push_back(Vec2d{p.x - cd.origin.x, p.y - cd.origin.y});
            if (inset.size() >= 3) {
                lod0.addRoofCap(inset, ridgeY, matRoof, static_cast<u32>(zone), flags, 0.25f);
                lod0.extrudeWalls(inset, topY, ridgeY, matRoof, static_cast<u32>(zone), flags, 0.3f);
            } else {
                lod0.addRoofCap(localRing, topY, matRoof, static_cast<u32>(zone), flags, 0.25f);
            }
        } else {
            lod0.addRoofCap(localRing, topY, matRoof, static_cast<u32>(zone), flags, 0.25f,
                            0.35, rule.parapet);
        }
        lod0.endSubmesh();
        lod0.bakeAmbientOcclusion(groundY, std::max(4.0, height * 0.35));

        // --- oriented proxy ---------------------------------------------------
        emitProxy(cd, buildRing, centre, groundY, height, wallMat, static_cast<u32>(zone), true);

        // --- impostor ---------------------------------------------------------
        {
            const f64 cx = centre.x - cd.origin.x;
            const f64 cz = centre.y - cd.origin.y;
            const f64 half = std::max(bb.size().x, bb.size().y) * 0.5 + 1.0;
            cd.impostors.addDoubleSidedQuad(
                Vec3{static_cast<f32>(cx - half), static_cast<f32>(groundY), static_cast<f32>(cz)},
                Vec3{static_cast<f32>(cx - half), static_cast<f32>(topY), static_cast<f32>(cz)},
                Vec3{static_cast<f32>(cx + half), static_cast<f32>(topY), static_cast<f32>(cz)},
                Vec3{static_cast<f32>(cx + half), static_cast<f32>(groundY), static_cast<f32>(cz)},
                wallMat, static_cast<u32>(zone), flags, 0.2f);
        }

        // --- collision --------------------------------------------------------
        // The footprint's own rectangle, oriented: an axis-aligned box around a
        // rotated block reaches metres into the street beside it, and a vehicle
        // driven down that street is then stopped by a wall that is not there.
        cd.collision.push_back(collisionBoxFor(cd, buildRing, centre, groundY, height, wallMat, 1u));
        ctx.stats.collisionBoxes++;

        cd.counts.buildings++;
        ctx.stats.buildingCount++;
        // Delta, not the builder's running total: the counter used to re-add the
        // whole cell's triangle count for every building.
        ctx.stats.buildingTriangles += lod0.triangleCount() - trisBefore;
        built++;
    }
    MLOG_INFO("fabric: %llu synthesised buildings placed, %llu of them inset to clear a road, "
              "%llu parcels too small to fit beside one",
              (unsigned long long)built, (unsigned long long)insetForRoad,
              (unsigned long long)skippedOnRoad);
    ctx.qa.add("buildings.synthetic", static_cast<f64>(built));
    ctx.qa.add("fabric.insetForRoad", static_cast<f64>(insetForRoad));
    ctx.qa.add("fabric.skippedOnRoad", static_cast<f64>(skippedOnRoad));
}

bool buildBuildings(Context& ctx) {
    if (!ctx.cfg.buildBuildings) return true;
    ScopedTimer timer("buildBuildings");

    const u32 matShopSign = ctx.materials.id("signage_painted");

    // The corridors, once, for both paths.  The surveyed footprints below are
    // *real* geometry read from the map and are never moved for a road — but where
    // a real footprint's fitted rectangle reaches a lane it is counted, because a
    // number nobody measured is a number nobody can act on.  Measured: 56 of the
    // 148 surveyed footprints have their fitted rectangle over a lane sample
    // (999 of 5,070,900 in the published world), and every one of them is a large
    // irregular polygon — a slum cluster, an SRA block, an industrial plot —
    // approximated by one oriented box, so the box covers ground the polygon does
    // not.  The fleet's spawn gate is what keeps a car out of them (D-034/D-037).
    const RoadCorridorIndex roads = buildRoadCorridorIndex(ctx);
    MLOG_INFO("fabric: %zu road corridors indexed", roads.segs.size());
    ctx.qa.add("fabric.roadCorridors", static_cast<f64>(roads.segs.size()));
    u64 surveyedOnRoad = 0, surveyedCount = 0;

    std::unordered_map<std::string, std::string> wardCache;   // grid key -> ward

    for (const osm::Way& w : ctx.osm.ways()) {
        if (w.buildingClass == 0 || !w.closed() || w.refCount < 4) continue;

        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        if (pts.size() < 4) continue;
        geo2d::Ring ring = geo2d::simplify(pts, ctx.cfg.footprintSimplify);
        if (ring.size() >= 2 && (ring.front() - ring.back()).lengthSq() < 1e-9) ring.pop_back();
        if (ring.size() < 3) continue;

        const geo2d::Aabb2d bb = geo2d::bounds(ring);
        const f64 area = std::fabs(geo2d::signedArea(ring));
        if (area < ctx.cfg.minBuildingArea) continue;

        const Vec2d centre = bb.center();
        if (!ctx.cityMask.contains(centre)) continue;

        // Zoning and ward lookup.
        const ZoningZone zone = ctx.zoning.sample(centre);
        if (zone == ZoningZone::MangroveWetland || zone == ZoningZone::Unknown) continue;
        std::string wardCode;
        for (const auto& f : ctx.wardFeatures) {
            if (geo2d::polygonContains(f.polygon, centre)) {
                auto it = f.properties.find("name");
                if (it != f.properties.end()) wardCode = it->second;
                break;
            }
        }

        const ArchitectureRule rule = ruleForZone(zone, wardCode);
        const u64 seed = fnv1a64(static_cast<u64>(w.id));
        const FacadeVariation var = variationFor(seed, zone);
        Rng rng(seed ^ 0xBEEF);

        // --- height determination -------------------------------------------
        f64 height = 0.0;
        const std::string_view levelsTag = ctx.osm.tag(w, "building:levels");
        const std::string_view heightTag = ctx.osm.tag(w, "height");
        if (!heightTag.empty()) {
            height = std::atof(std::string(heightTag).c_str());
        } else if (!levelsTag.empty()) {
            height = std::atof(std::string(levelsTag).c_str()) * rule.floorHeight;
        }
        if (height < 3.0 || height > 400.0) {
            // Most Mumbai footprints have no height data; the zoning rule plus a
            // deterministic variation supplies the storey count.
            height = rng.range(rule.minHeight, rule.maxHeight);
            if (zone == ZoningZone::SuburbanResidential || zone == ZoningZone::BKCCommercial) {
                // Emit whole storeys so facades do not slice through windows.
                height = std::ceil(height / rule.floorHeight) * rule.floorHeight;
            }
        }

        const f64 groundY = ctx.terrainHF.sample(centre.x, centre.y);
        const f64 topY = groundY + height;
        const u8 flags = 1;   // shadow caster

        // LOD0 materials: wall variant + roof.
        const u32 wallMat = materialVariant(ctx, buildingMaterialName(rule.wall), var, zone);
        const u32 roofMat = materialVariant(ctx, "concrete_raw", var, zone);

        CellData* cdp = ctx.tryCell(centre);
        if (!cdp) continue;                       // outside the world extent
        CellData& cd = *cdp;
        const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};

        // Footprint in cell-local space.
        std::vector<Vec2d> localRing;
        localRing.reserve(ring.size());
        for (const Vec2d& p : ring)
            localRing.push_back(Vec2d{p.x - cd.origin.x, p.y - cd.origin.y});
        std::vector<Vec2d> worldRing = ring;

        // --- LOD 0 -----------------------------------------------------------
        mesh::MeshBuilder& lod0 = cd.buildings[0];
        lod0.beginSubmesh(wallMat, static_cast<u32>(zone));
        {
            f64 base = groundY - 0.4;                     // sink slightly: no gaps
            if (rule.groundFloorShops) {
                // Ground-floor band (shops/entrances) uses the signage material
                // so shopfronts read as a distinct street-level layer.
                lod0.extrudeWalls(localRing, base, groundY + 3.2, wallMat,
                                  static_cast<u32>(zone), flags, 0.3f);
                base = groundY + 3.2;
                lod0.beginSubmesh(matShopSign, static_cast<u32>(zone));
                lod0.extrudeWalls(localRing, base, base + 0.35, matShopSign,
                                  static_cast<u32>(zone), flags, 0.6f);
                lod0.endSubmesh();
                base = groundY + 3.55;
            }
            lod0.extrudeWalls(localRing, base, topY, wallMat, static_cast<u32>(zone), flags, 0.3f);
        }
        lod0.endSubmesh();

        lod0.beginSubmesh(roofMat, static_cast<u32>(zone));
        if (rule.pitchedRoof) {
            // Colonial pitched roof: an inset ridge cap over the ceiling,
            // standing in for Mangalore-tile roofing.
            const f64 ridgeY = topY + std::min(3.5, (bb.size().x + bb.size().y) * 0.06);
            std::vector<Vec2d> localInset;
            for (const Vec2d& p : geo2d::offsetRing(ring, -1.2))
                localInset.push_back(Vec2d{p.x - cd.origin.x, p.y - cd.origin.y});
            if (localInset.size() >= 3) {
                lod0.addRoofCap(localInset, ridgeY, roofMat, static_cast<u32>(zone), flags, 0.25f);
                lod0.extrudeWalls(localInset, topY, ridgeY, roofMat, static_cast<u32>(zone), flags, 0.3f);
            } else {
                lod0.addRoofCap(localRing, topY, roofMat, static_cast<u32>(zone), flags, 0.25f);
            }
        } else {
            lod0.addRoofCap(localRing, topY, roofMat, static_cast<u32>(zone), flags, 0.25f,
                            0.35, rule.parapet);
        }
        lod0.endSubmesh();
        lod0.bakeAmbientOcclusion(groundY, std::max(4.0, height * 0.35));

        // --- LOD 1 (simplified ring, no parapet or shopfront) ----------------
        mesh::MeshBuilder& lod1 = cd.buildings[1];
        std::vector<Vec2d> ring1 = geo2d::simplify(ring, ctx.cfg.footprintSimplify * 3.0);
        if (ring1.size() >= 3) {
            std::vector<Vec2d> local1;
            for (const Vec2d& p : ring1) local1.push_back(Vec2d{p.x - cd.origin.x, p.y - cd.origin.y});
            lod1.beginSubmesh(wallMat, static_cast<u32>(zone));
            lod1.extrudeWalls(local1, groundY - 0.3, topY, wallMat, static_cast<u32>(zone), flags, 0.3f);
            lod1.endSubmesh();
            lod1.beginSubmesh(roofMat, static_cast<u32>(zone));
            lod1.addRoofCap(local1, topY, roofMat, static_cast<u32>(zone), flags, 0.25f);
            lod1.endSubmesh();
        }

        // --- LOD 2 (box prism from the footprint bounds) ----------------------
        mesh::MeshBuilder& lod2 = cd.buildings[2];
        {
            const f64 cx = centre.x - cd.origin.x;
            const f64 cz = centre.y - cd.origin.y;
            const f64 hx = bb.size().x * 0.5, hz = bb.size().y * 0.5;
            std::vector<Vec2d> box{{cx - hx, cz - hz}, {cx + hx, cz - hz},
                                   {cx + hx, cz + hz}, {cx - hx, cz + hz}};
            lod2.beginSubmesh(wallMat, static_cast<u32>(zone));
            lod2.extrudeWalls(box, groundY - 0.3, topY, wallMat, static_cast<u32>(zone), flags, 0.3f);
            lod2.endSubmesh();
            lod2.beginSubmesh(roofMat, static_cast<u32>(zone));
            lod2.addRoofCap(box, topY, roofMat, static_cast<u32>(zone), flags, 0.25f);
            lod2.endSubmesh();
        }

        // --- oriented proxy ---------------------------------------------------
        emitProxy(cd, ring, centre, groundY, height, wallMat, static_cast<u32>(zone), false);

        // --- LOD 3 impostor: one quad, faces the camera in the shader ---------
        {
            const f64 cx = centre.x - cd.origin.x;
            const f64 cz = centre.y - cd.origin.y;
            const f64 half = std::max(bb.size().x, bb.size().y) * 0.5 + 1.0;
            mesh::MeshBuilder& imp = cd.impostors;
            const u32 mat = cd.buildings[0].empty() ? wallMat
                            : ctx.materials.id(buildingMaterialName(rule.wall));
            imp.addDoubleSidedQuad(
                Vec3{static_cast<f32>(cx - half), static_cast<f32>(groundY), static_cast<f32>(cz)},
                Vec3{static_cast<f32>(cx - half), static_cast<f32>(topY), static_cast<f32>(cz)},
                Vec3{static_cast<f32>(cx + half), static_cast<f32>(topY), static_cast<f32>(cz)},
                Vec3{static_cast<f32>(cx + half), static_cast<f32>(groundY), static_cast<f32>(cz)},
                mat, static_cast<u32>(zone), flags, 0.2f);
        }

        // --- the road test, for the record only --------------------------------
        // Real geometry is not moved for a road: a surveyed footprint is what the
        // map says is there, and shaping it to the nav graph would be inventing
        // data.  It is measured instead — where the fitted rectangle reaches a
        // lane, the number is reported (D-037).
        ++surveyedCount;
        if (roads.hits(massingOf(ring, centre))) ++surveyedOnRoad;

        // --- collision --------------------------------------------------------
        // A box proxy per building. Large or irregular footprints are split into
        // a small set of boxes so that the player cannot walk through a
        // courtyard-owning block.
        {
            cd.collision.push_back(collisionBoxFor(cd, ring, centre, groundY, height, wallMat, 1u));
            ctx.stats.collisionBoxes++;

            if (area > 2500.0) {
                geo2d::Polygon poly;
                poly.outer = ring;
                auto boxes = geo2d::decomposeToBoxes(poly, 32.0, 8);
                for (const auto& b : boxes) {
                    CollisionBox sb{};
                    sb.cx = static_cast<f32>((b.min.x + b.max.x) * 0.5 - cd.origin.x);
                    sb.cz = static_cast<f32>((b.min.y + b.max.y) * 0.5 - cd.origin.y);
                    sb.cy = static_cast<f32>(groundY + height * 0.5);
                    sb.hx = static_cast<f32>((b.max.x - b.min.x) * 0.5);
                    sb.hz = static_cast<f32>((b.max.y - b.min.y) * 0.5);
                    sb.hy = static_cast<f32>(height * 0.5);
                    sb.qw = 1.0f;
                    sb.flags = 1;
                    sb.materialId = wallMat;
                    cd.collision.push_back(sb);
                    ctx.stats.collisionBoxes++;
                }
            }
        }

        // --- statistics and downstream hand-off -------------------------------
        cd.counts.buildings++;
        ctx.stats.buildingCount++;
        ctx.stats.zoneCounts[zoningName(zone)] += 1;
        ctx.stats.buildingTriangles += lod0.triangleCount();
        (void)worldRing;

        Context::FootprintRecord rec;
        rec.centre = centre;
        rec.ring = std::move(ring);
        rec.groundY = groundY;
        rec.height = height;
        rec.zone = zone;
        rec.seed = seed;
        rec.wallMaterial = wallMat;
        rec.pitchedRoof = rule.pitchedRoof;
        ctx.footprints.push_back(std::move(rec));
    }

    // Synthesised parcels (ward lattice + slum clusters) become architecture
    // here, through the same per-zone rules as the surveyed footprints.
    emitSyntheticFabric(ctx, roads);

    MLOG_INFO("buildings: %llu surveyed footprints, %llu of them with their fitted rectangle "
              "over a road (real geometry: counted, not moved)",
              (unsigned long long)surveyedCount, (unsigned long long)surveyedOnRoad);
    ctx.qa.add("buildings.surveyed", static_cast<f64>(surveyedCount));
    ctx.qa.add("buildings.surveyedOnRoad", static_cast<f64>(surveyedOnRoad));

    ctx.stats.totalTriangles += ctx.stats.buildingTriangles;
    MLOG_INFO("buildings: %llu structures, %llu LOD0 triangles, %llu collision boxes",
              (unsigned long long)ctx.stats.buildingCount,
              (unsigned long long)ctx.stats.buildingTriangles,
              (unsigned long long)ctx.stats.collisionBoxes);
    ctx.qa.add("buildings.count", static_cast<f64>(ctx.stats.buildingCount));
    ctx.qa.add("buildings.lod0Triangles", static_cast<f64>(ctx.stats.buildingTriangles));
    ctx.qa.add("buildings.collisionBoxes", static_cast<f64>(ctx.stats.collisionBoxes));
    if (ctx.stats.buildingCount < 1000)
        ctx.qa.warn("building count below 1000 — source extract may be incomplete");
    return true;
}

// -----------------------------------------------------------------------------
//  buildProps
// -----------------------------------------------------------------------------
bool buildProps(Context& ctx) {
    if (!ctx.cfg.buildProps) return true;
    ScopedTimer timer("buildProps");

    // Instance mesh ids are a fixed vocabulary shared with the runtime/viewer.
    enum PropMesh : u32 {
        PropBox = 0, PropWaterTank = 1, PropAcUnit = 2, PropTree = 3,
        PropTwoWheeler = 4, PropStreetlamp = 5, PropCableTangle = 6
    };
    const u32 matTank = ctx.materials.id("water_tank_plastic");
    const u32 matAc = ctx.materials.id("ac_unit");
    const u32 matTree = ctx.materials.id("tree_foliage");
    const u32 matLamp = ctx.materials.id("streetlamp_pole");
    const u32 matGlow = ctx.materials.id("streetlamp_glow");
    const u32 matRail = ctx.materials.id("metal_railing");

    auto addInstance = [&](const Vec2d& world, f64 y, f64 yaw, u32 meshId, u32 material,
                           const Vec3& scale) {
        CellData* cdp = ctx.tryCell(world);
        if (!cdp) return;                         // outside the world extent
        CellData& cd = *cdp;
        InstanceRecord ir{};
        ir.px = static_cast<f32>(world.x - cd.origin.x);
        ir.py = static_cast<f32>(y);
        ir.pz = static_cast<f32>(world.y - cd.origin.y);
        const Vec4 q = quatFromEuler(static_cast<f32>(yaw), 0, 0);
        ir.qx = q.x; ir.qy = q.y; ir.qz = q.z; ir.qw = q.w;
        ir.sx = scale.x; ir.sy = scale.y; ir.sz = scale.z;
        ir.meshId = meshId;
        ir.materialId = material;
        ir.flags = 0;
        cd.instances.push_back(ir);
        cd.counts.instances++;
        ctx.stats.instanceCount++;
    };

    // --- 1. Rooftop water tanks and AC units --------------------------------
    // These two props carry an enormous amount of Mumbai's visual identity in
    // the suburban and informal zones, and the specification names both.
    u64 tanks = 0, acs = 0, bikes = 0;
    for (const Context::FootprintRecord& b : ctx.footprints) {
        const geo2d::Aabb2d bb = geo2d::bounds(b.ring);
        const Vec2d& c = b.centre;
        const ZoningZone zone = b.zone;

        Rng rng(b.seed * 7919ull);
        const f64 groundY = b.groundY;
        const f64 height = b.height;
        const f64 roofY = groundY + height;
        const f64 halfX = std::max(1.0, bb.size().x * 0.5 - 1.0);
        const f64 halfZ = std::max(1.0, bb.size().y * 0.5 - 1.0);

        // Water tanks: on roughly half the suburban/informal roofs, one or two.
        if ((zone == ZoningZone::SuburbanResidential || zone == ZoningZone::InformalSettlement ||
             zone == ZoningZone::Hillside) && rng.chance(0.55f)) {
            const int count = rng.chance(0.35f) ? 2 : 1;
            for (int k = 0; k < count; ++k) {
                const Vec2d p{c.x + rng.range(-halfX, halfX) * 0.6,
                              c.y + rng.range(-halfZ, halfZ) * 0.6};
                const f32 scale = rng.range(0.85f, 1.25f);
                addInstance(p, roofY + 0.9 * scale, rng.range(0.0f, 6.28f), PropWaterTank, matTank,
                            Vec3{scale, scale, scale});
                ++tanks;
            }
        }
        // AC condenser units on facades of the taller stock.
        if (height > 9.0 && rng.chance(0.45f)) {
            const Vec2d p{c.x + rng.range(-halfX, halfX),
                          c.y + (rng.chance(0.5f) ? halfZ : -halfZ)};
            addInstance(p, groundY + rng.range(4.0, height - 2.0), rng.range(0.0f, 6.28f),
                        PropAcUnit, matAc, Vec3{1, 1, 1});
            ++acs;
        }
        // Informal settlements: the overhead cable tangles between roofs.
        if (zone == ZoningZone::InformalSettlement && rng.chance(0.25f)) {
            addInstance(c, roofY + 1.5, rng.range(0.0f, 6.28f), PropCableTangle, matRail,
                        Vec3{1.0f, static_cast<f32>(6.0 + rng.range(0.0f, 8.0f)), 1.0f});
        }
        // Parked two-wheelers at the kerb outside the dense residential fabric.
        // Mumbai's streetscape is defined by these as much as by the buildings.
        if ((zone == ZoningZone::SuburbanResidential || zone == ZoningZone::InformalSettlement) &&
            rng.chance(0.42f)) {
            const size_t edge = static_cast<size_t>(rng.nextU32()) % b.ring.size();
            const Vec2d a = b.ring[edge];
            const Vec2d next = b.ring[(edge + 1) % b.ring.size()];
            const Vec2d mid = (a + next) * 0.5;
            const Vec2d outward = (mid - c).normalized();
            const Vec2d park{mid.x + outward.x * 1.4, mid.y + outward.y * 1.4};
            addInstance(park, ctx.terrainHF.sample(park.x, park.y),
                        static_cast<f32>(std::atan2(outward.y, outward.x)),
                        PropTwoWheeler, matRail, Vec3{1, 1, 1});
            ++bikes;
        }
    }

    // --- 2. Streetlamps along the network -----------------------------------
    // Placed every ~32 m along drivable edges, alternating sides. Lamps on the
    // Marine Drive arc are flagged as the Queen's Necklace cluster.
    u64 lamps = 0;
    for (auto& [key, cd] : ctx.cells) {
        (void)key;
        const u32 stride = 4;
        for (u32 i = 0; i + 1 < cd.navNodes.size(); i += stride) {
            const NavNode& n = cd.navNodes[i];
            if (!(n.flags & NavFlagDrivable)) continue;
            if (cd.navEdges.empty()) continue;
            const Vec2d world{cd.origin.x + n.x, cd.origin.y + n.z};
            addInstance(world, n.y, 0.0f, PropStreetlamp, matLamp, Vec3{1, 1, 1});
            ++lamps;
            // Emissive glow quad for the light source itself.
            InstanceRecord glow{};
            glow.px = n.x; glow.py = n.y + 8.6f; glow.pz = n.z;
            glow.qw = 1.0f;
            glow.sx = glow.sy = glow.sz = 1.0f;
            glow.meshId = PropBox;
            glow.materialId = matGlow;
            cd.instances.push_back(glow);
            ctx.stats.instanceCount++;
        }
    }

    // --- 3. Trees from parks, gardens and street-side green ------------------
    u64 trees = 0;
    for (const osm::Way& w : ctx.osm.ways()) {
        if (!w.closed() || w.refCount < 4) continue;
        const auto lu = static_cast<osm::LanduseClass>(w.landuseClass);
        const auto na = static_cast<osm::NaturalClass>(w.naturalClass);
        const bool green = lu == osm::LU_Grass || lu == osm::LU_Recreation ||
                           na == osm::NA_Wood || na == osm::NA_Scrub || na == osm::NA_Grass;
        if (!green) continue;
        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        if (pts.size() < 3) continue;
        const geo2d::Aabb2d bb = geo2d::bounds(pts);
        const f64 area = bb.area();
        const u32 count = static_cast<u32>(std::min<f64>(64.0, std::max<f64>(1.0, area / 260.0)));
        Rng rng(fnv1a64(static_cast<u64>(w.id)));
        for (u32 k = 0; k < count; ++k) {
            const Vec2d p{rng.range(bb.min.x, bb.max.x), rng.range(bb.min.y, bb.max.y)};
            const f64 y = ctx.terrainHF.sample(p.x, p.y);
            addInstance(p, y, rng.range(0.0f, 6.28f), PropTree, matTree,
                        Vec3{1.0f, static_cast<f32>(rng.range(0.7f, 1.5f)), 1.0f});
            ++trees;
        }
    }

    ctx.qa.add("props.waterTanks", static_cast<f64>(tanks));
    ctx.qa.add("props.acUnits", static_cast<f64>(acs));
    ctx.qa.add("props.streetlamps", static_cast<f64>(lamps));
    ctx.qa.add("props.trees", static_cast<f64>(trees));
    ctx.qa.add("props.twoWheelers", static_cast<f64>(bikes));
    MLOG_INFO("props: %llu tanks, %llu AC units, %llu lamps, %llu trees, %llu two-wheelers",
              (unsigned long long)tanks, (unsigned long long)acs,
              (unsigned long long)lamps, (unsigned long long)trees,
              (unsigned long long)bikes);
    return true;
}

} // namespace mumbai::pipeline
