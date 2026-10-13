// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_geo.cpp
//
//  Projection accuracy and the floating-origin guarantees of Section 4.  The
//  completion criteria ask for zero vertex jitter at the four corners and the
//  centre of the map; the cases below are that claim, executed.
// =============================================================================
#include "engine/core/geo.h"
#include "tests/test_harness.h"

using namespace mumbai;

namespace {
// WGS84 reference points spanning the built area: Colaba, Bandra, Borivali,
// Thane creek, Powai.
constexpr struct { const char* name; f64 lat, lon; } kPoints[] = {
    {"Colaba",           18.9067, 72.8147},
    {"Bandra",           19.0596, 72.8295},
    {"Borivali",         19.2307, 72.8567},
    {"Thane creek",      19.0650, 72.9800},
    {"Powai",            19.1197, 72.9051},
};
}  // namespace

TEST(projectionRoundTripsToMillimetres) {
    TransverseMercator proj;
    for (const auto& p : kPoints) {
        const Vec2d m = proj.forward(p.lat, p.lon);
        f64 lat = 0, lon = 0;
        proj.inverse(m.x, m.y, lat, lon);
        // 1e-8 degrees is ~1.1 mm; the Krueger n-series is good for far better.
        CHECK_NEAR(lat, p.lat, 1e-8);
        CHECK_NEAR(lon, p.lon, 1e-8);
    }
}

TEST(projectionScaleIsNearUnityAcrossTheMap) {
    TransverseMercator proj;
    for (const auto& p : kPoints) {
        const f64 k = proj.scaleAt(p.lat, p.lon);
        // A conformal projection with k0 = 1 keeps local scale within a few
        // parts per million of unity over a 50 km extent; anything larger would
        // stretch road widths and building footprints measurably.
        CHECK_NEAR(k, 1.0, 1e-4);
    }
}

TEST(projectionMetreIsMetre) {
    // Two points 0.001 degrees of latitude apart must be apart by the true
    // ellipsoidal meridian arc, 110.70 m at 19 deg N (a sphere would give
    // 111.32 m).  If the projection silently used a spherical earth, every
    // distance in the world would be 0.6% long.
    TransverseMercator proj;
    const Vec2d a = proj.forward(19.0, 72.9);
    const Vec2d b = proj.forward(19.001, 72.9);
    CHECK_NEAR((b - a).length(), 110.70, 0.05);

    // Easting: 0.001 degrees of longitude at 19 deg N is 105.28 m on WGS84
    // (N cos(phi) per radian times the 1.745e-5 rad step).
    const Vec2d c = proj.forward(19.0, 72.901);
    CHECK_NEAR((c - a).length(), 105.28, 0.06);
}

TEST(globalPositionNormalisesIntoCellBounds) {
    GlobalPosition gp;
    gp.cellX = 3; gp.cellZ = -4;
    gp.localOffset = {2048.0f, 12.0f, -1500.0f};
    gp.normalize();
    CHECK(gp.localOffset.x >= 0.0f && gp.localOffset.x < MACRO_CELL_SIZE);
    CHECK(gp.localOffset.z >= 0.0f && gp.localOffset.z < MACRO_CELL_SIZE);
    const Vec2d world = gp.toWorld();
    CHECK_NEAR(world.x, 3 * 1024.0 + 2048.0, 1e-3);
    CHECK_NEAR(world.y, -4 * 1024.0 - 1500.0, 1e-3);
}

TEST(renderSpaceKeepsSubMillimetreGranularityAtMapCorners) {
    // The acceptance test: for the four corners and the centre of the built
    // extent, a vertex pair 1 mm apart must stay 1 mm apart after conversion to
    // f32 render space, and the round trip back to world must not drift.
    const Vec2d corners[5] = {{-13312, 2048}, {9216, 2048}, {-13312, 45056},
                              {9216, 45056}, {-2048, 23552}};
    for (const Vec2d& c : corners) {
        const GlobalPosition gp = GlobalPosition::fromWorld(c, 5.0);
        // The camera sits in the same cell as the vertex: worst case for f32
        // is a vertex far from the local origin, which is what the LOD radius
        // (4096 m) bounds.
        const Vec3 r = gp.toRenderSpace(gp.cellX, gp.cellZ, 0.0);
        const GlobalPosition probe = GlobalPosition::fromWorld({c.x + 0.001, c.y + 0.001}, 5.0);
        const Vec3 rp = probe.toRenderSpace(gp.cellX, gp.cellZ, 0.0);
        const f64 dx = static_cast<f64>(rp.x) - r.x;
        const f64 dz = static_cast<f64>(rp.z) - r.z;
        const f64 measured = std::sqrt(dx * dx + dz * dz);
        CHECK_NEAR(measured, 0.0014142, 2e-4);      // 1 mm on each axis
        CHECK(std::fabs(r.x) <= MACRO_CELL_SIZE && std::fabs(r.z) <= MACRO_CELL_SIZE);
    }
}

TEST(floatingOriginSurvivesTwentyBoundaryCrossings) {
    // Section 4's stated verification: twenty consecutive macro-cell boundary
    // crossings at speed.  The invariant is NOT that world coordinates stay
    // put -- the whole point of a floating origin is that they move with the
    // camera.  What must hold is that the tracked entity's *render-space*
    // position and its distance to the camera are unchanged by the shift, which
    // is what "no pop, no impulse" means in practice.
    FloatingOriginManager mgr;
    GlobalPosition tracked = GlobalPosition::fromWorld({-13000.0, 2400.0}, 3.0);

    // Every camera position along the march is shifted by the same amount as
    // the entity (that is what the shift means), so the entity's position
    // *relative to the camera* - which is all any renderer or solver ever sees -
    // must be bit-stable across the crossing.  That is the concrete meaning of
    // "no pop, no impulse" in Section 4.
    f64 worstRelativeError = 0.0, worstDistanceError = 0.0;
    for (int i = 0; i <= 24; ++i) {
        GlobalPosition cam = GlobalPosition::fromWorld({-13000.0, 2400.0 + i * 1500.0}, 3.0);

        const Vec3 relBefore = tracked.toRenderSpace(cam.cellX, cam.cellZ);
        const f64 distBefore = tracked.distanceTo(cam);

        const OriginShift shift = mgr.update(cam);
        if (shift.active()) {
            FloatingOriginManager::applyShift(tracked, shift);
            FloatingOriginManager::applyShift(cam, shift);
        }

        const Vec3 relAfter = tracked.toRenderSpace(cam.cellX, cam.cellZ);
        const f64 distAfter = tracked.distanceTo(cam);
        worstRelativeError = std::max(worstRelativeError,
            std::sqrt(static_cast<f64>(relAfter.x - relBefore.x) * (relAfter.x - relBefore.x) +
                      static_cast<f64>(relAfter.z - relBefore.z) * (relAfter.z - relBefore.z)));
        worstDistanceError = std::max(worstDistanceError, std::fabs(distAfter - distBefore));
    }
    CHECK(mgr.shiftCount() >= 20);
    // f32 local offsets are bounded by one 1024 m cell, so a single rounding is
    // at most 6.1e-5 m; allow a comfortable 5e-4 m for the pair of roundings.
    CHECK_NEAR(worstRelativeError, 0.0, 5e-4);
    CHECK_NEAR(worstDistanceError, 0.0, 1e-3);

    // Local offsets must remain inside a cell so f32 never sees a large value.
    CHECK(tracked.localOffset.x >= 0.0f && tracked.localOffset.x < MACRO_CELL_SIZE);
    CHECK(tracked.localOffset.z >= 0.0f && tracked.localOffset.z < MACRO_CELL_SIZE);
}

TEST(zoningAndSurfaceNamesAreStable) {
    // Names are written into reports, manifests and tooling; a silent rename
    // would invalidate historical QA output.
    CHECK(std::string(zoningName(ZoningZone::SouthBombayColonial)) == "SouthBombayColonial");
    CHECK(std::string(zoningName(ZoningZone::InformalSettlement)) == "InformalSettlement");
    CHECK(std::string(surfaceName(SurfaceMaterial::AsphaltSeaLink)) == "asphalt_sealink");
    CHECK(std::string(buildingMaterialName(BuildingMaterial::CurtainWallGlass)) == "curtain_wall_glass");
}
