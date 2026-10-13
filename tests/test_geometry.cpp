// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_geometry.cpp
//
//  Geometry and mesh-emission invariants.  The winding tests here are the ones
//  referenced from pipeline/mgeo/mesh.cpp: getting orientation wrong inverts
//  every facade and every roof in a 600 km^2 city, and it is invisible in a
//  wireframe, so it is pinned down by assertion instead.
// =============================================================================
#include "pipeline/mgeo/mesh.h"
#include "tests/test_harness.h"

using namespace mumbai;

namespace {

geo2d::Ring square(f64 cx, f64 cz, f64 size) {
    const f64 h = size * 0.5;
    // Counter-clockwise in the 2D footprint frame, which is the convention the
    // whole pipeline relies on (GeoJSON outer rings are normalised to it).
    return {{cx - h, cz - h}, {cx + h, cz - h}, {cx + h, cz + h}, {cx - h, cz + h}};
}

Vec3 faceNormal(const mesh::MeshBuilder& mb, u32 i0, u32 i1, u32 i2) {
    const auto& v = mb.vertexData();
    const Vec3 a{v[i0].px, v[i0].py, v[i0].pz};
    const Vec3 b{v[i1].px, v[i1].py, v[i1].pz};
    const Vec3 c{v[i2].px, v[i2].py, v[i2].pz};
    const Vec3 u{b.x - a.x, b.y - a.y, b.z - a.z};
    const Vec3 w{c.x - a.x, c.y - a.y, c.z - a.z};
    return {u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x};
}

}  // namespace

TEST(wallsFaceOutward) {
    mesh::MeshBuilder mb;
    const geo2d::Ring ring = square(0, 0, 10.0);
    mb.extrudeWalls(ring, 0.0, 4.0, 1, 0, 1);

    CHECK_EQ(mb.vertexCount(), 16u);          // 4 edges x 4 vertices
    CHECK_EQ(mb.triangleCount(), 8u);

    // Every emitted geometric normal must point away from the footprint centre.
    const auto& v = mb.vertexData();
    const auto& idx = mb.indexData();
    bool allOutward = true, allUpright = true;
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3 n = faceNormal(mb, idx[t], idx[t + 1], idx[t + 2]);
        const f64 len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
        if (len < 1e-9) { allUpright = false; continue; }
        // Face centre relative to the ring centre (ring centre is 0,0 here).
        const f64 cx = (v[idx[t]].px + v[idx[t + 1]].px + v[idx[t + 2]].px) / 3.0;
        const f64 cz = (v[idx[t]].pz + v[idx[t + 1]].pz + v[idx[t + 2]].pz) / 3.0;
        if (n.x * cx + n.z * cz <= 0.0) allOutward = false;
        if (std::fabs(n.y) > 1e-6) allUpright = false;   // walls are vertical
    }
    CHECK(allOutward);
    CHECK(allUpright);
}

TEST(roofCapsFaceUp) {
    mesh::MeshBuilder mb;
    mb.addRoofCap(square(0, 0, 8.0), 12.0, 1, 0, 1);
    const auto& idx = mb.indexData();
    CHECK(idx.size() >= 6);
    bool allUp = true;
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3 n = faceNormal(mb, idx[t], idx[t + 1], idx[t + 2]);
        if (n.y <= 0.0) allUp = false;
    }
    CHECK(allUp);

    // With a parapet lip the cap stays up-facing and the lip adds walls.
    mesh::MeshBuilder mb2;
    mb2.addRoofCap(square(0, 0, 8.0), 12.0, 1, 0, 1, 0.25f, 0.4, 0.9);
    CHECK(mb2.vertexCount() > mb.vertexCount());
    const auto& idx2 = mb2.indexData();
    int upFacing = 0;
    for (size_t t = 0; t + 2 < idx2.size(); t += 3) {
        const Vec3 n = faceNormal(mb2, idx2[t], idx2[t + 1], idx2[t + 2]);
        if (n.y > 0.0) ++upFacing;
    }
    CHECK(upFacing >= 2);
}

TEST(doubleSidedQuadIsTwoFaced) {
    mesh::MeshBuilder mb;
    mb.addDoubleSidedQuad({0, 0, 0}, {0, 4, 0}, {6, 4, 0}, {6, 0, 0}, 1, 0, 1);
    CHECK_EQ(mb.triangleCount(), 4u);          // two triangles each way
    const auto& idx = mb.indexData();
    int pos = 0, neg = 0;
    for (size_t t = 0; t + 2 < idx.size(); t += 3) {
        const Vec3 n = faceNormal(mb, idx[t], idx[t + 1], idx[t + 2]);
        if (n.z > 0) ++pos;
        else if (n.z < 0) ++neg;
    }
    CHECK_EQ(pos, 2);
    CHECK_EQ(neg, 2);
}

TEST(triangulateRingWithHolePreservesArea) {
    geo2d::Polygon poly;
    poly.outer = square(0, 0, 20.0);
    geo2d::Ring hole = square(0, 0, 8.0);          // holes run clockwise
    std::reverse(hole.begin(), hole.end());
    poly.holes.push_back(hole);
    const geo2d::Mesh2 tris = geo2d::triangulate(poly);
    CHECK(tris.indices.size() >= 3);
    CHECK_EQ(tris.indices.size() % 3, 0u);

    f64 area = 0.0;
    for (size_t t = 0; t + 2 < tris.indices.size(); t += 3) {
        const Vec2d a = tris.vertices[tris.indices[t]];
        const Vec2d b = tris.vertices[tris.indices[t + 1]];
        const Vec2d c = tris.vertices[tris.indices[t + 2]];
        area += 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    }
    CHECK_NEAR(area, 20.0 * 20.0 - 8.0 * 8.0, 0.5);   // hole removed, not filled
}

TEST(simplifyKeepsEndpointsAndDropsCollinearPoints) {
    geo2d::Ring line;
    for (int i = 0; i <= 20; ++i) line.push_back({static_cast<f64>(i) * 5.0, 0.0});
    line.push_back({100.0, 7.0});
    const geo2d::Ring simplified = geo2d::simplifyOpen(line, 0.05);
    CHECK(simplified.size() >= 2);
    CHECK_NEAR(simplified.front().x, line.front().x, 1e-9);
    CHECK_NEAR(simplified.back().x, line.back().x, 1e-9);
    CHECK(simplified.size() < line.size());
}

TEST(resampleRespectsMaximumSpacing) {
    std::vector<Vec2d> line{{0, 0}, {100, 0}, {100, 100}};
    const std::vector<Vec2d> dense = geo2d::resampleOpen(line, 4.0);
    CHECK(dense.size() > 40);
    f64 worst = 0.0;
    for (size_t i = 0; i + 1 < dense.size(); ++i)
        worst = std::max(worst, (dense[i + 1] - dense[i]).length());
    CHECK(worst <= 4.0 + 1e-6);
}

TEST(offsetRingShrinks) {
    const geo2d::Ring ring = square(0, 0, 20.0);
    // Positive delta moves a counter-clockwise ring inward (the convention the
    // roof-cap inset uses); negative overhangs it outward.
    const geo2d::Ring inner = geo2d::offsetRing(ring, 2.0);
    CHECK(inner.size() >= 4);
    const f64 outerArea = std::fabs(geo2d::signedArea(ring));
    const f64 innerArea = std::fabs(geo2d::signedArea(inner));
    CHECK(innerArea < outerArea);
}

TEST(polygonContainsHandlesHoles) {
    geo2d::Polygon poly;
    poly.outer = square(0, 0, 30.0);
    poly.holes.push_back(square(0, 0, 10.0));
    CHECK(geo2d::polygonContains(poly, {12.0, 12.0}));
    CHECK_FALSE(geo2d::polygonContains(poly, {0.0, 0.0}));
    CHECK_FALSE(geo2d::polygonContains(poly, {40.0, 0.0}));
}

TEST(offsetRingGrowsOutwardForNegativeDelta) {
    const geo2d::Ring ring = square(0, 0, 20.0);
    const geo2d::Ring outer = geo2d::offsetRing(ring, -1.5);
    CHECK(std::fabs(geo2d::signedArea(outer)) > std::fabs(geo2d::signedArea(ring)));
}

TEST(decomposeToBoxesStaysInsideFootprint) {
    // An L-shaped footprint decomposed into axis-aligned boxes: every box must
    // lie inside the polygon, or physics would collide with air.
    geo2d::Ring l{{0, 0}, {20, 0}, {20, 8}, {8, 8}, {8, 20}, {0, 20}};
    geo2d::Polygon poly;
    poly.outer = l;
    const auto boxes = geo2d::decomposeToBoxes(poly, 4.0, 16);
    CHECK(!boxes.empty());
    for (const auto& b : boxes) {
        CHECK(b.valid());
        for (int sx = 0; sx < 2; ++sx) {
            for (int sz = 0; sz < 2; ++sz) {
                const Vec2d corner{sx ? b.max.x : b.min.x, sz ? b.max.y : b.min.y};
                CHECK(geo2d::polygonContains(poly, corner));
            }
        }
    }
}
