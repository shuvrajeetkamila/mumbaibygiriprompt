// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/stages/hero.cpp
//
//  Hero assets: the structures that define Mumbai's identity and cannot be left
//  to the generic grammar (specification Section 2B).
//
//      Gateway of India      Apollo Bunder basalt arch
//      Taj Mahal Palace      heritage wing + tower wing massing
//      CSMT                  Gothic Revival terminal + train shed
//      Bandra-Worli Sea Link cable-stayed bridge (pylons, stays, deck)
//      Marine Drive          the Queen's Necklace arc, seawall and lamps
//
//  HONEST SCOPE NOTE (DOCS/07_ROADMAP.md): these are procedural massings built
//  from published dimensions at real coordinates — recognisable in silhouette,
//  correctly placed, scaled and oriented — not photogrammetric scans. The
//  specification's 300k-1.2M triangle targets assume a capture pipeline (drone
//  photogrammetry or survey mesh) that this environment cannot run, so every
//  asset reports its actual polygon count to the QA block: the gap is measured
//  and published rather than hidden.
// =============================================================================
#include "pipeline/pipeline.h"

namespace mumbai::pipeline {
namespace {

using LatLonRow = std::array<f64, 2>;

std::vector<Vec2d> projectPolyline(const TransverseMercator& proj,
                                   const std::vector<LatLonRow>& rows) {
    std::vector<Vec2d> out;
    out.reserve(rows.size());
    for (const auto& r : rows) out.push_back(proj.forward(r[0], r[1]));
    return out;
}

inline Vec3 V3(f64 x, f64 y, f64 z) {
    return Vec3{static_cast<f32>(x), static_cast<f32>(y), static_cast<f32>(z)};
}

// A vertical ribbon quad between two lateral offsets along a segment, emitted
// with an explicit outward normal so no winding reasoning is needed.
void ribbonWall(mesh::MeshBuilder& mb, const Vec3& a0, const Vec3& a1,
                const Vec3& b1, const Vec3& b0, const Vec3& normal,
                u32 mat, u32 zone, f32 uvScale = 0.4f) {
    mb.addQuad(a0, b0, b1, a1, normal, mat, zone, 1, uvScale);
}


// -----------------------------------------------------------------------------
//  Detail kit
// -----------------------------------------------------------------------------
// The three point landmarks are authored as clean massing first, then dressed
// with these primitives.  Keeping the dressing in one place means a reviewer can
// check the vocabulary once instead of re-reading three bespoke builders, and it
// is where the specification's "no feature shipped rough" is actually spent:
// cornices, recessed openings, drum ribbing and finials are what make a massing
// read as the building it is at 100 m.

// A projecting band (cornice or string course) around a ring.
void addMoulding(mesh::MeshBuilder& mb, const std::vector<Vec2d>& ring, f64 y, f64 height,
                 f64 out, u32 mat, u32 zone) {
    if (ring.size() < 3) return;
    const auto outer = geo2d::offsetRing(ring, out);
    mb.extrudeWalls(outer, y, y + height, mat, zone, 1, 0.35f);
    mb.addRoofCap(outer, y + height, mat, zone, 1, 0.35f);
}

// One recessed opening: back face plus four reveals, so the hole has depth.
void addRecess(mesh::MeshBuilder& mb, const Vec2d& a, const Vec2d& b, f64 y0, f64 y1,
               const Vec2d& outward, f64 inset, u32 mat, u32 zone) {
    const Vec2d in = a - outward * inset;
    const Vec2d ib = b - outward * inset;
    const Vec2d tangent = (b - a).normalized();
    const Vec3 nOut{static_cast<f32>(outward.x), 0.0f, static_cast<f32>(outward.y)};
    const Vec3 nA{static_cast<f32>(-tangent.x), 0.0f, static_cast<f32>(-tangent.y)};
    const Vec3 nB{static_cast<f32>(tangent.x), 0.0f, static_cast<f32>(tangent.y)};
    const Vec3 down{0.0f, -1.0f, 0.0f};
    const Vec3 up{0.0f, 1.0f, 0.0f};
    // back
    mb.addQuad(V3(in.x, y0, in.y), V3(ib.x, y0, ib.y), V3(ib.x, y1, ib.y), V3(in.x, y1, in.y),
               nOut, mat, zone, 1, 0.5f);
    // reveals
    mb.addQuad(V3(a.x, y0, a.y), V3(in.x, y0, in.y), V3(in.x, y1, in.y), V3(a.x, y1, a.y),
               nA, mat, zone, 1, 0.5f);
    mb.addQuad(V3(ib.x, y0, ib.y), V3(b.x, y0, b.y), V3(b.x, y1, b.y), V3(ib.x, y1, ib.y),
               nB, mat, zone, 1, 0.5f);
    // lintel (faces down) and sill (faces up)
    mb.addQuad(V3(in.x, y1, in.y), V3(ib.x, y1, ib.y), V3(b.x, y1, b.y), V3(a.x, y1, a.y),
               down, mat, zone, 1, 0.5f);
    mb.addQuad(V3(a.x, y0, a.y), V3(b.x, y0, b.y), V3(ib.x, y0, ib.y), V3(in.x, y0, in.y),
               up, mat, zone, 1, 0.5f);
}

// A row of openings across one wall face.  `count` bays, a recess in each.
void addOpeningRow(mesh::MeshBuilder& mb, const Vec2d& a, const Vec2d& b, const Vec2d& outward,
                   f64 y0, f64 y1, int count, f64 inset, u32 mat, u32 zone, f64 duty = 0.62) {
    if (count <= 0) return;
    const f64 span = 1.0 / count;
    const f64 pad = span * (1.0 - duty) * 0.5;
    for (int i = 0; i < count; ++i) {
        const Vec2d p0 = a + (b - a) * (i * span + pad);
        const Vec2d p1 = a + (b - a) * ((i + 1) * span - pad);
        addRecess(mb, p0, p1, y0, y1, outward, inset, mat, zone);
    }
}

// Vertical ribbing around a drum: what separates a cylinder from a dome.
void addDrumRibs(mesh::MeshBuilder& mb, f64 cx, f64 cz, f64 r, f64 y0, f64 y1, int count,
                 u32 mat, u32 zone, f64 proud = 0.5, f64 width = 0.22) {
    for (int i = 0; i < count; ++i) {
        const f64 a = (i / static_cast<f64>(count)) * TAU;
        const Vec2d radial{std::cos(a), std::sin(a)};
        const Vec2d tangent{-std::sin(a), std::cos(a)};
        const f64 w = (TAU * r / count) * width;
        const std::vector<Vec2d> strip{
            {cx + radial.x * r - tangent.x * w, cz + radial.y * r - tangent.y * w},
            {cx + radial.x * (r + proud) - tangent.x * w, cz + radial.y * (r + proud) - tangent.y * w},
            {cx + radial.x * (r + proud) + tangent.x * w, cz + radial.y * (r + proud) + tangent.y * w},
            {cx + radial.x * r + tangent.x * w, cz + radial.y * r + tangent.y * w}};
        mb.extrudeWalls(strip, y0, y1, mat, zone, 1, 0.6f);
        mb.addRoofCap(strip, y1, mat, zone, 1, 0.6f);
    }
}

// Ball and spike on top of a dome or turret.
void addFinial(mesh::MeshBuilder& mb, f64 x, f64 z, f64 y0, f64 y1, u32 mat, u32 zone) {
    std::vector<Vec2d> ball, spike;
    for (int k = 0; k < 8; ++k) {
        const f64 a = (k / 8.0) * TAU;
        ball.push_back({x + std::cos(a) * 0.6, z + std::sin(a) * 0.6});
        spike.push_back({x + std::cos(a) * 0.18, z + std::sin(a) * 0.18});
    }
    mb.extrudeWalls(ball, y0, y0 + 0.8, mat, zone, 1, 0.8f);
    mb.addRoofCap(ball, y0 + 0.8, mat, zone, 1, 0.8f);
    mb.extrudeWalls(spike, y0 + 0.8, y1, mat, zone, 1, 0.8f);
    mb.addRoofCap(spike, y1, mat, zone, 1, 0.8f);
}

// A domed turret: shaft, corbelled cap, drum ring, dome, finial — the unit that
// appears on every Indo-Saracenic corner of the three landmarks.
void addDomedTurret(mesh::MeshBuilder& mb, f64 x, f64 z, f64 r, f64 y0, f64 yCap, f64 yTop,
                    u32 mat, u32 matDome, u32 zone) {
    std::vector<Vec2d> ring, cap, drum;
    for (int k = 0; k < 12; ++k) {
        const f64 a = (k / 12.0) * TAU;
        ring.push_back({x + std::cos(a) * r, z + std::sin(a) * r});
        drum.push_back({x + std::cos(a) * (r * 0.78), z + std::sin(a) * (r * 0.78)});
    }
    mb.extrudeWalls(ring, y0, yCap, mat, zone, 1, 0.4f);
    const auto capRing = geo2d::offsetRing(ring, 0.55);
    mb.extrudeWalls(capRing, yCap, yCap + 0.6, mat, zone, 1, 0.5f);
    mb.addRoofCap(capRing, yCap + 0.6, mat, zone, 1, 0.5f);
    mb.extrudeWalls(drum, yCap + 0.6, yCap + 1.6, mat, zone, 1, 0.5f);
    // dome: three rings from the drum radius to a point
    const int rings = 4;
    for (int g = 0; g < rings; ++g) {
        const f64 t0 = static_cast<f64>(g) / rings, t1 = static_cast<f64>(g + 1) / rings;
        const f64 r0 = (r * 0.78) * std::cos(t0 * PI * 0.5);
        const f64 r1 = (r * 0.78) * std::cos(t1 * PI * 0.5);
        const f64 yy0 = yCap + 1.6 + (yTop - yCap - 1.6) * std::sin(t0 * PI * 0.5);
        const f64 yy1 = yCap + 1.6 + (yTop - yCap - 1.6) * std::sin(t1 * PI * 0.5);
        for (int k = 0; k < 12; ++k) {
            const f64 a0 = (k / 12.0) * TAU, a1 = ((k + 1) / 12.0) * TAU;
            const Vec3 p0 = V3(x + std::cos(a0) * r0, yy0, z + std::sin(a0) * r0);
            const Vec3 p1 = V3(x + std::cos(a1) * r0, yy0, z + std::sin(a1) * r0);
            const Vec3 p2 = V3(x + std::cos(a1) * r1, yy1, z + std::sin(a1) * r1);
            const Vec3 p3 = V3(x + std::cos(a0) * r1, yy1, z + std::sin(a0) * r1);
            const Vec3 n = (p1 - p0).cross(p3 - p0).normalized();
            mb.addQuad(p0, p3, p2, p1, n, matDome, zone, 1, 0.45f);
        }
    }
    addFinial(mb, x, z, yTop, yTop + 1.6, matDome, zone);
}

// A pitched train-shed roof: `bays` gabled spans over a rectangular footprint.
void addTrainShed(mesh::MeshBuilder& mb, f64 cx, f64 cz, f64 hx, f64 hz, f64 y0, f64 rise,
                  int bays, u32 mat, u32 zone) {
    const f64 bayW = (hx * 2.0) / bays;
    for (int b = 0; b < bays; ++b) {
        const f64 x0 = cx - hx + b * bayW, x1 = x0 + bayW;
        const f64 xm = (x0 + x1) * 0.5;
        for (int side = 0; side < 2; ++side) {
            const f64 z0 = side == 0 ? cz - hz : cz + hz;
            const Vec3 p0 = V3(x0, y0, z0), p1 = V3(xm, y0 + rise, z0), p2 = V3(x1, y0, z0);
            const Vec3 n = side == 0 ? V3(0.0f, 0.35f, -0.94f) : V3(0.0f, 0.35f, 0.94f);
            mb.addQuad(p0, p1, p2, p2, n, mat, zone, 1, 0.35f);
            mb.addQuad(p0, p2, p1, p1, n, mat, zone, 1, 0.35f);   // both faces
        }
    }
}

// Detail passes are defined below (after the massing builders) so the dressing
// vocabulary sits in one block; the builders call them.
void gatewayDetail(mesh::MeshBuilder& mb, f64 groundY, u32 matStone, u32 matTerra, u32 zone);
void tajDetail(mesh::MeshBuilder& mb, const Vec2d& base, f64 groundY, u32 matStone,
               u32 matTerra, u32 matConcrete, u32 zone);
void csmtDetail(mesh::MeshBuilder& mb, const Vec2d& base, f64 groundY, u32 matStone,
                u32 matTerra, u32 matSteel, u32 zone);

// -----------------------------------------------------------------------------
//  Gateway of India — 26 m basalt arch, 15 m opening, four corner turrets.
// -----------------------------------------------------------------------------
void buildGateway(Context&, const Vec2d& base, f64 groundY, mesh::MeshBuilder& mb,
                  u32 matStone, u32 matTerra, u32 zone) {
    const f64 hx = 7.5, hz = 7.5;
    const f64 archTop = 20.0;
    const f64 parapet = 26.0;
    const f64 pierW = 3.0;

    // Plinth steps
    mb.beginSubmesh(matStone, zone);
    for (int step = 0; step < 3; ++step) {
        const f64 e = 3.0 - step * 1.0;
        const f64 y0 = groundY + step * 0.35;
        mb.addRoofCap({{-hx - e, -hz - e}, {-hx - e, hz + e}, {hx + e, hz + e}, {hx + e, -hz - e}},
                      y0 + 0.35, matStone, zone, 1, 0.2f);
        mb.extrudeWalls({{-hx - e, -hz - e}, {-hx - e, hz + e}, {hx + e, hz + e}, {hx + e, -hz - e}},
                        y0 - 0.4, y0 + 0.35, matStone, zone, 1, 0.3f);
    }
    const f64 plinthTop = groundY + 1.05;

    // Piers
    for (int side = 0; side < 2; ++side) {
        const f64 x0 = side == 0 ? -hx : hx - pierW;
        const std::vector<Vec2d> ring{{x0, -hz}, {x0, hz}, {x0 + pierW, hz}, {x0 + pierW, -hz}};
        mb.extrudeWalls(ring, plinthTop - 0.5, plinthTop + parapet, matStone, zone, 1, 0.15f);
        mb.addRoofCap(ring, plinthTop + parapet, matStone, zone, 1, 0.2f);
    }

    // Arch soffit: stepped semicircular sweep on both faces.
    const int archSteps = 10;
    for (int i = 0; i < archSteps; ++i) {
        const f64 t0 = static_cast<f64>(i) / archSteps;
        const f64 t1 = static_cast<f64>(i + 1) / archSteps;
        const f64 y0 = plinthTop + archTop * std::sin(t0 * PI * 0.5);
        const f64 y1 = plinthTop + archTop * std::sin(t1 * PI * 0.5);
        const f64 x0 = (hx - pierW) * std::cos(t0 * PI * 0.5);
        const f64 x1 = (hx - pierW) * std::cos(t1 * PI * 0.5);
        for (int face = 0; face < 2; ++face) {
            const f64 z = face == 0 ? -hz : hz;
            const Vec3 n = face == 0 ? V3(0, 0, -1) : V3(0, 0, 1);
            for (int s = 0; s < 2; ++s) {
                const f64 sgn = s == 0 ? -1.0 : 1.0;
                mb.addQuad(V3(sgn * (hx - pierW), y0, z), V3(sgn * x0, y0, z),
                           V3(sgn * x1, y1, z), V3(sgn * x1, y1, z), n, matStone, zone, 1, 0.2f);
            }
        }
    }

    // Spandrel above the arch
    mb.extrudeWalls({{-hx, -hz}, {-hx, hz}, {hx, hz}, {hx, -hz}},
                    plinthTop + archTop, plinthTop + parapet, matStone, zone, 1, 0.15f);
    mb.addRoofCap({{-hx, -hz}, {-hx, hz}, {hx, hz}, {hx, -hz}},
                  plinthTop + parapet, matStone, zone, 1, 0.2f);
    mb.endSubmesh();

    // Turrets
    mb.beginSubmesh(matTerra, zone);
    for (int sx = 0; sx < 2; ++sx) {
        for (int sz = 0; sz < 2; ++sz) {
            const f64 tx = (sx == 0 ? -1.0 : 1.0) * (hx - 1.5);
            const f64 tz = (sz == 0 ? -1.0 : 1.0) * (hz - 1.5);
            std::vector<Vec2d> ring;
            for (int k = 0; k < 10; ++k) {
                const f64 a = (k / 10.0) * TAU;
                ring.push_back({tx + std::cos(a) * 1.6, tz + std::sin(a) * 1.6});
            }
            mb.extrudeWalls(ring, plinthTop + parapet - 0.5, plinthTop + parapet + 3.0,
                            matStone, zone, 1, 0.3f);
            mb.extrudeWalls(ring, plinthTop + parapet + 3.0, plinthTop + parapet + 4.4,
                            matTerra, zone, 1, 0.4f);
            mb.addRoofCap(ring, plinthTop + parapet + 4.4, matTerra, zone, 1, 0.4f);
        }
    }
    mb.endSubmesh();

    gatewayDetail(mb, groundY, matStone, matTerra, zone);
}

// -----------------------------------------------------------------------------
//  Taj Mahal Palace — heritage wing (dome, turrets) + tower wing.
// -----------------------------------------------------------------------------
void buildTaj(Context&, const Vec2d& base, f64 groundY, mesh::MeshBuilder& mb,
              u32 matStone, u32 matTerra, u32 matConcrete, u32 zone) {
    const f64 hx = 30.0, hz = 22.0;
    const std::vector<Vec2d> ring{{base.x - hx, base.y - hz}, {base.x - hx, base.y + hz},
                                  {base.x + hx, base.y + hz}, {base.x + hx, base.y - hz}};
    mb.beginSubmesh(matStone, zone);
    mb.extrudeWalls(ring, groundY - 0.5, groundY + 40.0, matStone, zone, 1, 0.12f);
    mb.addRoofCap(ring, groundY + 40.0, matTerra, zone, 1, 0.2f, 1.2, 1.0);
    mb.endSubmesh();

    // Corner turrets
    mb.beginSubmesh(matTerra, zone);
    for (int sx = 0; sx < 2; ++sx) {
        for (int sz = 0; sz < 2; ++sz) {
            const f64 tx = base.x + (sx == 0 ? -1.0 : 1.0) * (hx - 2.0);
            const f64 tz = base.y + (sz == 0 ? -1.0 : 1.0) * (hz - 2.0);
            std::vector<Vec2d> tr;
            for (int k = 0; k < 8; ++k) {
                const f64 a = (k / 8.0) * TAU;
                tr.push_back({tx + std::cos(a) * 2.6, tz + std::sin(a) * 2.6});
            }
            mb.extrudeWalls(tr, groundY + 33.0, groundY + 46.0, matStone, zone, 1, 0.3f);
            mb.extrudeWalls(tr, groundY + 46.0, groundY + 50.0, matTerra, zone, 1, 0.4f);
            mb.addRoofCap(tr, groundY + 50.0, matTerra, zone, 1, 0.4f, 0.6, 0.0);
        }
    }

    // Central dome on an octagonal drum
    const Vec2d dome{base.x, base.y + 2.0};
    const int seg = 16;
    const f64 drumR = 9.0, drumTop = groundY + 46.0, domeTop = groundY + 56.0;
    {
        std::vector<Vec2d> drumRing;
        for (int k = 0; k < seg; ++k) {
            const f64 a = (k / static_cast<f64>(seg)) * TAU;
            drumRing.push_back({dome.x + std::cos(a) * drumR, dome.y + std::sin(a) * drumR});
        }
        mb.extrudeWalls(drumRing, groundY + 40.0, drumTop, matStone, zone, 1, 0.2f);
    }
    {
        const int rings = 7;
        for (int r = 0; r < rings; ++r) {
            const f64 t0 = static_cast<f64>(r) / rings, t1 = static_cast<f64>(r + 1) / rings;
            const f64 y0 = drumTop + (domeTop - drumTop) * std::sin(t0 * PI * 0.5);
            const f64 y1 = drumTop + (domeTop - drumTop) * std::sin(t1 * PI * 0.5);
            const f64 r0 = drumR * std::cos(t0 * PI * 0.5);
            const f64 r1 = drumR * std::cos(t1 * PI * 0.5);
            for (int k = 0; k < seg; ++k) {
                const f64 a0 = (k / static_cast<f64>(seg)) * TAU;
                const f64 a1 = ((k + 1) / static_cast<f64>(seg)) * TAU;
                const Vec3 p0 = V3(dome.x + std::cos(a0) * r0, y0, dome.y + std::sin(a0) * r0);
                const Vec3 p1 = V3(dome.x + std::cos(a1) * r0, y0, dome.y + std::sin(a1) * r0);
                const Vec3 p2 = V3(dome.x + std::cos(a1) * r1, y1, dome.y + std::sin(a1) * r1);
                const Vec3 p3 = V3(dome.x + std::cos(a0) * r1, y1, dome.y + std::sin(a0) * r1);
                const Vec3 n = (p1 - p0).cross(p3 - p0).normalized();
                mb.addQuad(p0, p3, p2, p1, n, matTerra, zone, 1, 0.3f);
            }
        }
    }
    mb.endSubmesh();

    // Tower wing
    mb.beginSubmesh(matConcrete, zone);
    {
        const Vec2d tower{base.x + 62.0, base.y - 6.0};
        const f64 thx = 14.0, thz = 20.0;
        const std::vector<Vec2d> tr{{tower.x - thx, tower.y - thz}, {tower.x - thx, tower.y + thz},
                                    {tower.x + thx, tower.y + thz}, {tower.x + thx, tower.y - thz}};
        mb.extrudeWalls(tr, groundY - 0.5, groundY + 70.0, matConcrete, zone, 1, 0.14f);
        mb.addRoofCap(tr, groundY + 70.0, matConcrete, zone, 1, 0.2f, 0.8, 1.2);
    }
    mb.endSubmesh();

    tajDetail(mb, base, groundY, matStone, matTerra, matConcrete, zone);
}

// -----------------------------------------------------------------------------
//  Chhatrapati Shivaji Maharaj Terminus — Gothic terminal, dome, train shed.
// -----------------------------------------------------------------------------
void buildCsmt(Context&, const Vec2d& base, f64 groundY, mesh::MeshBuilder& mb,
               u32 matStone, u32 matTerra, u32 matSteel, u32 zone) {
    const f64 hx = 42.0, hz = 30.0;
    mb.beginSubmesh(matStone, zone);
    mb.extrudeWalls({{base.x - hx, base.y - hz}, {base.x - hx, base.y + hz},
                     {base.x + hx, base.y + hz}, {base.x + hx, base.y - hz}},
                    groundY - 0.5, groundY + 24.0, matStone, zone, 1, 0.1f);
    mb.addRoofCap({{base.x - hx, base.y - hz}, {base.x - hx, base.y + hz},
                   {base.x + hx, base.y + hz}, {base.x + hx, base.y - hz}},
                  groundY + 24.0, matStone, zone, 1, 0.2f, 1.5, 1.5);

    // Central block + dome
    const std::vector<Vec2d> block{{base.x - 14.0, base.y - 16.0}, {base.x - 14.0, base.y + 16.0},
                                   {base.x + 14.0, base.y + 16.0}, {base.x + 14.0, base.y - 16.0}};
    mb.extrudeWalls(block, groundY + 24.0, groundY + 34.0, matStone, zone, 1, 0.12f);
    const Vec2d dome{base.x, base.y};
    const int seg = 16;
    const f64 drumR = 8.0, drumTop = groundY + 38.0, domeTop = groundY + 48.0;
    {
        std::vector<Vec2d> drumRing;
        for (int k = 0; k < seg; ++k) {
            const f64 a = (k / static_cast<f64>(seg)) * TAU;
            drumRing.push_back({dome.x + std::cos(a) * drumR, dome.y + std::sin(a) * drumR});
        }
        mb.extrudeWalls(drumRing, groundY + 34.0, drumTop, matStone, zone, 1, 0.15f);
    }
    const int rings = 8;
    for (int r = 0; r < rings; ++r) {
        const f64 t0 = static_cast<f64>(r) / rings, t1 = static_cast<f64>(r + 1) / rings;
        const f64 y0 = drumTop + (domeTop - drumTop) * std::sin(t0 * PI * 0.5);
        const f64 y1 = drumTop + (domeTop - drumTop) * std::sin(t1 * PI * 0.5);
        const f64 r0 = drumR * std::cos(t0 * PI * 0.5);
        const f64 r1 = drumR * std::cos(t1 * PI * 0.5);
        for (int k = 0; k < seg; ++k) {
            const f64 a0 = (k / static_cast<f64>(seg)) * TAU;
            const f64 a1 = ((k + 1) / static_cast<f64>(seg)) * TAU;
            const Vec3 p0 = V3(dome.x + std::cos(a0) * r0, y0, dome.y + std::sin(a0) * r0);
            const Vec3 p1 = V3(dome.x + std::cos(a1) * r0, y0, dome.y + std::sin(a1) * r0);
            const Vec3 p2 = V3(dome.x + std::cos(a1) * r1, y1, dome.y + std::sin(a1) * r1);
            const Vec3 p3 = V3(dome.x + std::cos(a0) * r1, y1, dome.y + std::sin(a0) * r1);
            mb.addQuad(p0, p3, p2, p1, (p1 - p0).cross(p3 - p0).normalized(), matTerra, zone, 1, 0.3f);
        }
    }
    // Progress figure spire
    mb.extrudeWalls({{dome.x - 0.7, dome.y - 0.7}, {dome.x - 0.7, dome.y + 0.7},
                     {dome.x + 0.7, dome.y + 0.7}, {dome.x + 0.7, dome.y - 0.7}},
                    domeTop, domeTop + 5.0, matStone, zone, 1, 0.5f);

    // Flanking gables
    for (int s = 0; s < 2; ++s) {
        const f64 sx = base.x + (s == 0 ? -26.0 : 26.0);
        const std::vector<Vec2d> g{{sx - 9.0, base.y - hz}, {sx - 9.0, base.y + hz},
                                   {sx + 9.0, base.y + hz}, {sx + 9.0, base.y - hz}};
        mb.extrudeWalls(g, groundY + 24.0, groundY + 32.0, matStone, zone, 1, 0.14f);
        mb.addRoofCap(g, groundY + 32.0, matTerra, zone, 1, 0.3f, 1.0, 0.0);
    }
    mb.endSubmesh();

    // Platform level walls under the shed (the shed roofs themselves are part
    // of the detail pass, so the two do not fight over the same volume).
    mb.beginSubmesh(matSteel, zone);
    {
        const Vec2d shed{base.x, base.y + hz + 30.0};
        const f64 shx = 38.0, shz = 28.0;
        mb.extrudeWalls({{shed.x - shx, shed.y - shz}, {shed.x - shx, shed.y + shz},
                         {shed.x + shx, shed.y + shz}, {shed.x + shx, shed.y - shz}},
                        groundY - 0.4, groundY + 16.0, matSteel, zone, 1, 0.2f);
    }
    mb.endSubmesh();

    csmtDetail(mb, base, groundY, matStone, matTerra, matSteel, zone);
}


// -----------------------------------------------------------------------------
//  Detail passes
// -----------------------------------------------------------------------------

// Gateway of India: window rows in the spandrel, string courses, ribbed turrets
// with finials, and jali screens in the flanking piers.
void gatewayDetail(mesh::MeshBuilder& mb, f64 groundY, u32 matStone, u32 matTerra, u32 zone) {
    const f64 hx = 7.5, hz = 7.5;
    const f64 plinthTop = groundY + 1.05;
    const f64 archTop = 20.0, parapet = 26.0;

    mb.beginSubmesh(matStone, zone);
    const std::vector<Vec2d> shell{{-hx, -hz}, {-hx, hz}, {hx, hz}, {hx, -hz}};
    addMoulding(mb, shell, plinthTop + archTop + 0.2, 0.5, 0.45, matStone, zone);
    addMoulding(mb, shell, plinthTop + parapet - 0.9, 0.45, 0.65, matStone, zone);
    addMoulding(mb, shell, plinthTop + 1.4, 0.35, 0.35, matStone, zone);

    // Spandrel windows on both long faces.
    for (int face = 0; face < 2; ++face) {
        const f64 z = face == 0 ? -hz : hz;
        const Vec2d out{0.0, face == 0 ? -1.0 : 1.0};
        addOpeningRow(mb, {-hx + 0.8, z}, {hx - 0.8, z}, out, plinthTop + 22.0,
                      plinthTop + 25.2, 5, 0.35, matStone, zone, 0.55);
    }
    // Jali screens on the side piers.
    for (int side = 0; side < 2; ++side) {
        const f64 x = side == 0 ? -hx : hx;
        const Vec2d out{side == 0 ? -1.0 : 1.0, 0.0};
        addOpeningRow(mb, {x, -hz + 2.0}, {x, hz - 2.0}, out, plinthTop + 6.0,
                      plinthTop + 15.0, 4, 0.3, matStone, zone, 0.5);
    }
    mb.endSubmesh();

    mb.beginSubmesh(matTerra, zone);
    for (int sx = 0; sx < 2; ++sx) {
        for (int sz = 0; sz < 2; ++sz) {
            const f64 tx = (sx == 0 ? -1.0 : 1.0) * (hx - 1.5);
            const f64 tz = (sz == 0 ? -1.0 : 1.0) * (hz - 1.5);
            addDrumRibs(mb, tx, tz, 1.7, plinthTop + parapet - 0.4, plinthTop + parapet + 3.2,
                        8, matStone, zone, 0.35, 0.3);
            addDomedTurret(mb, tx, tz, 2.0, plinthTop + parapet + 3.2, plinthTop + parapet + 4.4,
                           plinthTop + parapet + 7.0, matStone, matTerra, zone);
        }
    }
    mb.endSubmesh();
}

// Taj Mahal Palace: two floors of arched windows, string courses, a ribbed dome
// with corner chhatris, and a banded tower wing.
void tajDetail(mesh::MeshBuilder& mb, const Vec2d& base, f64 groundY, u32 matStone,
               u32 matTerra, u32 matConcrete, u32 zone) {
    const f64 hx = 30.0, hz = 22.0;

    mb.beginSubmesh(matStone, zone);
    const std::vector<Vec2d> shell{{base.x - hx, base.y - hz}, {base.x - hx, base.y + hz},
                                   {base.x + hx, base.y + hz}, {base.x + hx, base.y - hz}};
    addMoulding(mb, shell, groundY + 9.5, 0.6, 0.7, matStone, zone);
    addMoulding(mb, shell, groundY + 22.0, 0.55, 0.6, matStone, zone);
    addMoulding(mb, shell, groundY + 33.5, 0.6, 0.75, matStone, zone);
    addMoulding(mb, shell, groundY + 38.5, 0.7, 1.0, matStone, zone);

    for (int face = 0; face < 2; ++face) {
        const f64 z = face == 0 ? base.y - hz : base.y + hz;
        const Vec2d out{0.0, face == 0 ? -1.0 : 1.0};
        addOpeningRow(mb, {base.x - hx + 2.0, z}, {base.x + hx - 2.0, z}, out,
                      groundY + 11.0, groundY + 17.5, 8, 0.5, matStone, zone);
        addOpeningRow(mb, {base.x - hx + 2.0, z}, {base.x + hx - 2.0, z}, out,
                      groundY + 24.0, groundY + 31.0, 8, 0.5, matStone, zone);
    }
    for (int face = 0; face < 2; ++face) {
        const f64 x = face == 0 ? base.x - hx : base.x + hx;
        const Vec2d out{face == 0 ? -1.0 : 1.0, 0.0};
        addOpeningRow(mb, {x, base.y - hz + 2.0}, {x, base.y + hz - 2.0}, out,
                      groundY + 11.0, groundY + 17.5, 5, 0.5, matStone, zone);
        addOpeningRow(mb, {x, base.y - hz + 2.0}, {x, base.y + hz - 2.0}, out,
                      groundY + 24.0, groundY + 31.0, 5, 0.5, matStone, zone);
    }
    addDrumRibs(mb, base.x, base.y + 2.0, 9.0, groundY + 40.5, groundY + 45.8, 16,
                matStone, zone, 0.55, 0.25);
    mb.endSubmesh();

    mb.beginSubmesh(matTerra, zone);
    for (int sx = 0; sx < 2; ++sx) {
        for (int sz = 0; sz < 2; ++sz) {
            const f64 tx = base.x + (sx == 0 ? -1.0 : 1.0) * (hx - 2.0);
            const f64 tz = base.y + (sz == 0 ? -1.0 : 1.0) * (hz - 2.0);
            addDomedTurret(mb, tx, tz, 2.9, groundY + 50.0, groundY + 51.6, groundY + 55.0,
                           matStone, matTerra, zone);
        }
    }
    addFinial(mb, base.x, base.y + 2.0, groundY + 56.0, groundY + 63.0, matTerra, zone);
    mb.endSubmesh();

    // Tower wing: a banded modern slab, which is what the 1973 tower is.
    mb.beginSubmesh(matConcrete, zone);
    {
        const f64 tx0 = base.x + 62.0 - 14.0, tz0 = base.y - 6.0 - 20.0;
        const std::vector<Vec2d> towerRing{{tx0, tz0}, {tx0, tz0 + 40.0}, {tx0 + 28.0, tz0 + 40.0},
                                          {tx0 + 28.0, tz0}};
        for (int floorIndex = 1; floorIndex < 17; ++floorIndex) {
            const f64 y = groundY + floorIndex * 4.1;
            addMoulding(mb, towerRing, y, 0.22, 0.35, matConcrete, zone);
        }
        for (int face = 0; face < 2; ++face) {
            const f64 z = face == 0 ? tz0 : tz0 + 40.0;
            const Vec2d out{0.0, face == 0 ? -1.0 : 1.0};
            for (int floorIndex = 0; floorIndex < 16; ++floorIndex) {
                const f64 y = groundY + floorIndex * 4.1 + 1.1;
                addOpeningRow(mb, {tx0 + 1.5, z}, {tx0 + 26.5, z}, out, y, y + 2.1, 7, 0.3,
                              matConcrete, zone, 0.6);
            }
        }
    }
    mb.endSubmesh();
}

// CSMT: a pointed-arch arcade, a ribbed dome with the Statue of Progress plinth,
// corner turrets, and the train shed behind the terminal block.
void csmtDetail(mesh::MeshBuilder& mb, const Vec2d& base, f64 groundY, u32 matStone,
                u32 matTerra, u32 matSteel, u32 zone) {
    const f64 hx = 42.0, hz = 30.0;

    mb.beginSubmesh(matStone, zone);
    const std::vector<Vec2d> shell{{base.x - hx, base.y - hz}, {base.x - hx, base.y + hz},
                                   {base.x + hx, base.y + hz}, {base.x + hx, base.y - hz}};
    addMoulding(mb, shell, groundY + 21.5, 0.8, 1.0, matStone, zone);
    addMoulding(mb, shell, groundY + 6.5, 0.5, 0.6, matStone, zone);
    addMoulding(mb, shell, groundY + 1.2, 0.6, 0.8, matStone, zone);

    // Grand arcade on both long faces: recess with a pointed head above it.
    for (int face = 0; face < 2; ++face) {
        const f64 z = face == 0 ? base.y - hz : base.y + hz;
        const Vec2d out{0.0, face == 0 ? -1.0 : 1.0};
        addOpeningRow(mb, {base.x - hx + 3.0, z}, {base.x + hx - 3.0, z}, out,
                      groundY + 7.2, groundY + 17.0, 9, 0.7, matStone, zone, 0.58);
        // pointed heads
        const int count = 9;
        const f64 span = (hx * 2.0 - 6.0) / count;
        for (int i = 0; i < count; ++i) {
            const f64 x0 = base.x - hx + 3.0 + i * span + span * 0.21;
            const f64 x1 = base.x - hx + 3.0 + (i + 1) * span - span * 0.21;
            const f64 xm = (x0 + x1) * 0.5;
            const Vec3 p0 = V3(x0, groundY + 17.0, z);
            const Vec3 p1 = V3(xm, groundY + 19.6, z);
            const Vec3 p2 = V3(x1, groundY + 17.0, z);
            mb.addQuad(p0, p1, p2, p2, V3(out.x, 0.0f, out.y), matStone, zone, 1, 0.5f);
            mb.addQuad(p0, p2, p1, p1, V3(-out.x, 0.0f, -out.y), matStone, zone, 1, 0.5f);
        }
    }
    // Low wing turrets at the corners, and the drum ribbing under the dome.
    addDrumRibs(mb, base.x, base.y, 8.0, groundY + 34.5, groundY + 37.8, 12, matStone, zone, 0.5, 0.22);
    mb.endSubmesh();

    mb.beginSubmesh(matTerra, zone);
    for (int sx = 0; sx < 2; ++sx) {
        for (int sz = 0; sz < 2; ++sz) {
            const f64 tx = base.x + (sx == 0 ? -1.0 : 1.0) * (hx - 6.0);
            const f64 tz = base.y + (sz == 0 ? -1.0 : 1.0) * (hz - 6.0);
            addDomedTurret(mb, tx, tz, 3.4, groundY + 24.0, groundY + 32.5, groundY + 37.5,
                           matStone, matTerra, zone);
        }
    }
    mb.endSubmesh();

    // Train shed: seven gabled bays over the platform level behind the block.
    mb.beginSubmesh(matSteel, zone);
    addTrainShed(mb, base.x, base.y + hz + 30.0, 38.0, 28.0, groundY + 16.0, 7.0, 7,
                 matSteel, zone);
    for (int bay = 0; bay < 8; ++bay) {
        const f64 x = base.x - 38.0 + bay * (76.0 / 7.0);
        const std::vector<Vec2d> rib{{x - 0.45, base.y + hz + 2.0}, {x + 0.45, base.y + hz + 2.0},
                                     {x + 0.45, base.y + hz + 58.0}, {x - 0.45, base.y + hz + 58.0}};
        mb.extrudeWalls(rib, groundY + 16.0, groundY + 22.5, matSteel, zone, 1, 0.5f);
    }
    mb.endSubmesh();
}

// -----------------------------------------------------------------------------
//  Bandra-Worli Sea Link
//
//  Published figures (MSRDC / HCC, as reported by the project's own material and
//  the trade press): the scheme is 5.6 km end to end, of which the over-water
//  crossing is roughly 4 km; the Bandra-channel structure is one central pylon
//  carrying two 250 m main spans, the Worli-channel structure has a 150 m main
//  span; the pylons rise about 128 m above the pile caps in a diamond form; the
//  crossing is a dual carriageway of about 20 m per direction on precast
//  segmental box girders, carried on ~50 m approach spans.
//
//  The OSM extract in data/raw contains exactly one fragment of this bridge
//  (way 48803910, 321 m, several of whose nodes fall outside the extract's
//  bounding box), so OSM locates the Bandra landfall but cannot describe the
//  structure.  The fragment is measured and reported; the alignment is the
//  published one — a curve around the Mahim Bay contour, not a straight line,
//  which is why the spans are modelled on stations along a polyline.
// -----------------------------------------------------------------------------
struct SeaLinkPath {
    std::vector<Vec2d> pts;      // 20 m samples, Bandra landfall first
    std::vector<f64> arc;        // arc length at each sample
    f64 total = 0.0;
    f64 bandraPylon = 0.0;       // station of the Bandra-channel pylon
    f64 worliPylon = 0.0;        // station of the Worli-channel pylon
    f64 osmFragment = 0.0;       // longest OSM "Sea Link" way, for the record
};

f64 seaLinkDeckY(f64 s, const SeaLinkPath& p) {
    // Viaduct soffit clear of fishing traffic; the deck rises over the two
    // navigation channels and settles onto the landfalls at both ends.
    const f64 bandra = std::exp(-std::pow((s - p.bandraPylon) / 340.0, 2.0));
    const f64 worli = std::exp(-std::pow((s - p.worliPylon) / 240.0, 2.0));
    f64 y = 14.0 + 12.0 * bandra + 6.0 * worli;
    const f64 ends = std::min(s, p.total - s);
    if (ends < 300.0) y = 9.0 + (y - 9.0) * std::max(0.0, ends / 300.0);
    return y;
}

SeaLinkPath seaLinkPath(Context& ctx) {
    SeaLinkPath path;
    for (const osm::Way& w : ctx.osm.ways()) {
        const std::string name = std::string(ctx.osm.tag(w, "name"));
        if (name.find("Sea Link") == std::string::npos) continue;
        std::vector<Vec2d> pts;
        ctx.osm.wayPoints(w, ctx.proj, pts);
        f64 len = 0.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) len += (pts[i + 1] - pts[i]).length();
        path.osmFragment = std::max(path.osmFragment, len);
    }

    // Bandra landfall -> Worli landfall, following the Mahim Bay shore: from
    // the K C Marg junction the alignment bears south-west across the bay,
    // straightens for the Bandra channel, then runs almost due south along the
    // Worli sea face.
    static const std::vector<LatLonRow> published = {
        {19.04720, 72.82690}, {19.04530, 72.82480}, {19.04310, 72.82240},
        {19.04080, 72.82010}, {19.03840, 72.81820}, {19.03590, 72.81690},
        {19.03320, 72.81610}, {19.03030, 72.81550}, {19.02720, 72.81500},
        {19.02400, 72.81450}, {19.02100, 72.81410}, {19.01800, 72.81380},
        {19.01630, 72.81370}, {19.01480, 72.81360},
    };
    static const LatLonRow bandraPylonLL{19.03590, 72.81690};
    static const LatLonRow worliPylonLL{19.02400, 72.81450};

    path.pts = geo2d::resampleOpen(projectPolyline(ctx.proj, published), 20.0);
    path.arc.reserve(path.pts.size());
    f64 acc = 0.0;
    for (size_t i = 0; i < path.pts.size(); ++i) {
        if (i) acc += (path.pts[i] - path.pts[i - 1]).length();
        path.arc.push_back(acc);
    }
    path.total = acc;

    const Vec2d bp = ctx.proj.forward(bandraPylonLL[0], bandraPylonLL[1]);
    const Vec2d wp = ctx.proj.forward(worliPylonLL[0], worliPylonLL[1]);
    f64 bestB = 1e30, bestW = 1e30;
    for (size_t i = 0; i < path.pts.size(); ++i) {
        const f64 db = (path.pts[i] - bp).lengthSq();
        const f64 dw = (path.pts[i] - wp).lengthSq();
        if (db < bestB) { bestB = db; path.bandraPylon = path.arc[i]; }
        if (dw < bestW) { bestW = dw; path.worliPylon = path.arc[i]; }
    }
    return path;
}

// A square prism between two arbitrary points, used for pylon legs, masts and
// the stay pipes.  Hero materials are drawn double-sided, so only the normals
// matter for shading and they are passed explicitly.
void boxBetween(mesh::MeshBuilder& mb, const Vec3& a, const Vec3& b, f32 halfA, f32 halfB,
                u32 mat, u32 zone, f32 uvScale = 0.25f) {
    const Vec3 axis = b - a;
    const f32 len = axis.length();
    if (len < 1e-4f) return;
    const Vec3 n = axis * (1.0f / len);
    Vec3 u = n.cross(V3(0, 1, 0));
    if (u.lengthSq() < 1e-8f) u = n.cross(V3(1, 0, 0));
    u = u.normalized();
    const Vec3 v = u.cross(n).normalized();
    const Vec3 A[4] = {a + u * halfA + v * halfA, a - u * halfA + v * halfA,
                       a - u * halfA - v * halfA, a + u * halfA - v * halfA};
    const Vec3 B[4] = {b + u * halfB + v * halfB, b - u * halfB + v * halfB,
                       b - u * halfB - v * halfB, b + u * halfB - v * halfB};
    const Vec3 N[4] = {(u + v).normalized(), (v - u).normalized(),
                       (-u - v).normalized(), (u - v).normalized()};
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        mb.addQuad(A[i], B[i], B[j], A[j], N[i], mat, zone, 1, uvScale);
    }
    mb.addQuad(A[0], A[1], A[2], A[3], -n, mat, zone, 1, uvScale);
    mb.addQuad(B[0], B[3], B[2], B[1], n, mat, zone, 1, uvScale);
}

// Total triangles currently held by every cell's hero builder.  The hero stages
// scatter geometry across the cells that contain it, so a sweep is the only
// honest way to account for what they emitted.
u64 heroTrianglesInCells(Context& ctx) {
    u64 n = 0;
    for (auto& [key, cd] : ctx.cells) n += cd.hero.triangleCount();
    return n;
}

void buildSeaLink(Context& ctx, u32 matPylon, u32 matCable, u32 matDeck, u32 zone) {
    const SeaLinkPath path = seaLinkPath(ctx);
    if (path.pts.size() < 4) return;

    const f64 waterLevel = ctx.cfg.seaLevel + 1.0;
    const f64 gapHalf = 2.0;            // median between the carriageways
    const f64 cwWidth = 19.0;           // one carriageway, ~20 m as reported
    const f64 deckHalf = gapHalf + cwWidth;
    const f64 soffit = 0.95;            // box-girder depth

    // --- deck, parapets and median ------------------------------------------
    for (size_t i = 0; i + 1 < path.pts.size(); ++i) {
        const Vec2d a = path.pts[i], b = path.pts[i + 1];
        const Vec2d dir = (b - a).normalized();
        const Vec2d left = dir.perpendicular();
        CellData* cdp = ctx.tryCell((a + b) * 0.5);
        if (!cdp) continue;                       // outside the world extent
        CellData& cd = *cdp;
        const Vec3 off = V3(cd.origin.x, 0, cd.origin.y);
        auto P = [&](const Vec2d& p, f64 lateral, f64 y) {
            const Vec2d q = p + left * lateral;
            return V3(q.x, y, q.y) - off;
        };
        const f64 y0 = seaLinkDeckY(path.arc[i], path);
        const f64 y1 = seaLinkDeckY(path.arc[i + 1], path);

        cd.hero.beginSubmesh(matDeck, zone);
        for (int side = 0; side < 2; ++side) {
            const f64 sgn = side == 0 ? 1.0 : -1.0;
            const f64 inner = sgn * gapHalf, outer = sgn * deckHalf;
            // Running surface, underside, outer parapet, median barrier.
            cd.hero.addQuad(P(a, inner, y0), P(b, inner, y1), P(b, outer, y1), P(a, outer, y0),
                            V3(0, 1, 0), matDeck, zone, 1, 0.08f);
            cd.hero.addQuad(P(a, outer, y0 - soffit), P(b, outer, y1 - soffit),
                            P(b, inner, y1 - soffit), P(a, inner, y0 - soffit),
                            V3(0, -1, 0), matDeck, zone, 1, 0.08f);
            ribbonWall(cd.hero, P(a, outer, y0), P(b, outer, y1),
                       P(b, outer, y1 + 1.45), P(a, outer, y0 + 1.45),
                       V3(sgn * left.x, 0, sgn * left.y), matDeck, zone, 0.5f);
            ribbonWall(cd.hero, P(a, inner, y0), P(b, inner, y1),
                       P(b, inner, y1 + 0.95), P(a, inner, y0 + 0.95),
                       V3(-sgn * left.x, 0, -sgn * left.y), matDeck, zone, 0.5f);
        }
        cd.hero.endSubmesh();
    }

    // --- approach viaduct: one bent every ~50 m -------------------------------
    u64 piers = 0;
    {
        f64 nextStation = 25.0;
        for (size_t i = 0; i + 1 < path.pts.size(); ++i) {
            if (path.arc[i] < nextStation) continue;
            nextStation = path.arc[i] + 50.0;
            const f64 s = path.arc[i];
            if (std::fabs(s - path.bandraPylon) < 130.0 ||
                std::fabs(s - path.worliPylon) < 110.0)
                continue;                                   // the pylons carry these spans
            const Vec2d p = path.pts[i];
            const Vec2d dir = (path.pts[i + 1] - p).normalized();
            const Vec2d left = dir.perpendicular();
            CellData* cdp = ctx.tryCell(p);
            if (!cdp) continue;
            CellData& cd = *cdp;
            const f64 deckY = seaLinkDeckY(s, path);
            cd.hero.beginSubmesh(matDeck, zone);
            for (int leg = 0; leg < 2; ++leg) {
                const f64 lateral = (leg == 0 ? 1.0 : -1.0) * (gapHalf + cwWidth * 0.5);
                const Vec2d lp = p + left * lateral;
                const std::vector<Vec2d> ring{
                    {lp.x - 1.4 - cd.origin.x, lp.y - 1.4 - cd.origin.y},
                    {lp.x - 1.4 - cd.origin.x, lp.y + 1.4 - cd.origin.y},
                    {lp.x + 1.4 - cd.origin.x, lp.y + 1.4 - cd.origin.y},
                    {lp.x + 1.4 - cd.origin.x, lp.y - 1.4 - cd.origin.y}};
                cd.hero.extrudeWalls(ring, waterLevel - 2.5, deckY - 2.4, matDeck, zone, 1, 0.1f);
            }
            const std::vector<Vec2d> cap{
                {p.x - deckHalf - cd.origin.x, p.y - 1.9 - cd.origin.y},
                {p.x - deckHalf - cd.origin.x, p.y + 1.9 - cd.origin.y},
                {p.x + deckHalf - cd.origin.x, p.y + 1.9 - cd.origin.y},
                {p.x + deckHalf - cd.origin.x, p.y - 1.9 - cd.origin.y}};
            cd.hero.extrudeWalls(cap, deckY - 2.4, deckY - soffit, matDeck, zone, 1, 0.1f);
            cd.hero.endSubmesh();
            ++piers;
        }
        ctx.qa.add("hero.seaLink.piers", static_cast<f64>(piers));
    }

    // --- cable-stayed pylons --------------------------------------------------
    u64 stays = 0, pylonCount = 0;
    struct PylonSpec { f64 station; f64 height; int staysPerSide; f64 maxAnchor; };
    const PylonSpec pylons[2] = {
        {path.bandraPylon, 128.0, 14, 240.0},     // two 250 m main spans
        {path.worliPylon, 96.0, 9, 150.0},        // 150 m main span
    };

    for (const PylonSpec& spec : pylons) {
        size_t idx = 0;
        f64 best = 1e30;
        for (size_t i = 0; i < path.pts.size(); ++i) {
            const f64 d = std::fabs(path.arc[i] - spec.station);
            if (d < best) { best = d; idx = i; }
        }
        const Vec2d base = path.pts[idx];
        const Vec2d fwd = (path.pts[std::min(idx + 1, path.pts.size() - 1)] - base).normalized();
        const Vec2d left = fwd.perpendicular();
        CellData* cdp = ctx.tryCell(base);
        if (!cdp) continue;
        CellData& cd = *cdp;
        const Vec3 off = V3(cd.origin.x, 0, cd.origin.y);
        const f64 deckY = seaLinkDeckY(spec.station, path);
        const f64 top = waterLevel + spec.height;
        const f64 pileCapY = waterLevel + 5.0;
        ++pylonCount;

        cd.hero.beginSubmesh(matPylon, zone);
        {
            const std::vector<Vec2d> cap{
                {base.x - 23.0 - cd.origin.x, base.y - 9.0 - cd.origin.y},
                {base.x - 23.0 - cd.origin.x, base.y + 9.0 - cd.origin.y},
                {base.x + 23.0 - cd.origin.x, base.y + 9.0 - cd.origin.y},
                {base.x + 23.0 - cd.origin.x, base.y - 9.0 - cd.origin.y}};
            cd.hero.extrudeWalls(cap, waterLevel - 1.0, pileCapY, matPylon, zone, 1, 0.12f);
            cd.hero.addRoofCap(cap, pileCapY, matPylon, zone, 1, 0.12f);

            const Vec3 waist = V3(base.x, deckY + 1.5, base.y) - off;
            for (int sx = 0; sx < 2; ++sx) {
                for (int sz = 0; sz < 2; ++sz) {
                    const f64 lx = (sx == 0 ? -1.0 : 1.0) * 18.0;
                    const f64 lz = (sz == 0 ? -1.0 : 1.0) * 6.5;
                    const Vec2d lp = base + left * lx + fwd * lz;
                    boxBetween(cd.hero, V3(lp.x, pileCapY, lp.y) - off, waist, 1.15f, 1.0f,
                               matPylon, zone, 0.15f);
                }
            }
            boxBetween(cd.hero, waist, V3(base.x, top, base.y) - off, 3.1f, 1.9f,
                       matPylon, zone, 0.12f);
            const Vec2d cbA = base + left * 18.0;
            const Vec2d cbB = base - left * 18.0;
            boxBetween(cd.hero, V3(cbA.x, deckY - 1.0, cbA.y) - off,
                       V3(cbB.x, deckY - 1.0, cbB.y) - off, 0.9f, 0.9f, matPylon, zone, 0.2f);
        }
        cd.hero.endSubmesh();

        // Semi-harp stays: two planes, fans out to both spans.
        cd.hero.beginSubmesh(matCable, zone);
        for (int plane = 0; plane < 2; ++plane) {
            const f64 lateral = (plane == 0 ? 1.0 : -1.0) * (gapHalf + cwWidth * 0.62);
            for (int dirSign = 0; dirSign < 2; ++dirSign) {
                const f64 sgn = dirSign == 0 ? 1.0 : -1.0;
                for (int c = 0; c < spec.staysPerSide; ++c) {
                    const f64 t = spec.staysPerSide > 1
                                      ? static_cast<f64>(c) / static_cast<f64>(spec.staysPerSide - 1)
                                      : 1.0;
                    const f64 mastY = deckY + 20.0 + t * (top - deckY - 34.0);
                    const f64 along = 26.0 + t * (spec.maxAnchor - 26.0);
                    const Vec2d anchor = base + fwd * (sgn * along) + left * lateral;
                    const Vec3 topPt = V3(base.x + left.x * lateral * 0.18, mastY,
                                          base.y + left.y * lateral * 0.18) - off;
                    const Vec3 botPt = V3(anchor.x,
                                          seaLinkDeckY(spec.station + sgn * along, path) + 1.1,
                                          anchor.y) - off;
                    boxBetween(cd.hero, topPt, botPt, 0.16f, 0.14f, matCable, zone, 1.0f);
                    ++stays;

                    // Short stays are the ones a vehicle can reach; they carry a
                    // capsule-equivalent collider so a car leaving the deck
                    // interacts with the cable array instead of passing through.
                    if (along <= 70.0) {
                        CollisionBox cb{};
                        cb.cx = static_cast<f32>((topPt.x + botPt.x) * 0.5f);
                        cb.cy = static_cast<f32>((topPt.y + botPt.y) * 0.5f);
                        cb.cz = static_cast<f32>((topPt.z + botPt.z) * 0.5f);
                        cb.hx = 0.22f; cb.hz = 0.22f;
                        cb.hy = static_cast<f32>((botPt - topPt).length() * 0.5);
                        cb.qw = 1.0f;
                        cb.flags = 4;              // 4 = cable collider
                        cd.collision.push_back(cb);
                        ctx.stats.collisionBoxes++;
                    }
                }
            }
        }
        cd.hero.endSubmesh();

        // The pylon legs are a hard obstacle for anything on the deck.
        CollisionBox pc{};
        pc.cx = static_cast<f32>(base.x - cd.origin.x);
        pc.cz = static_cast<f32>(base.y - cd.origin.y);
        pc.cy = static_cast<f32>((pileCapY + deckY) * 0.5);
        pc.hx = 20.0f; pc.hz = 7.5f;
        pc.hy = static_cast<f32>((deckY - pileCapY) * 0.5);
        pc.qw = 1.0f;
        pc.flags = 2;                              // 2 = hero structure collider
        cd.collision.push_back(pc);
        ctx.stats.collisionBoxes++;
    }

    // --- lighting masts -------------------------------------------------------
    {
        u64 masts = 0;
        for (size_t i = 0; i + 1 < path.pts.size(); i += 6) {
            const Vec2d p = path.pts[i];
            CellData* cdp = ctx.tryCell(p);
            if (!cdp) continue;
            CellData& cd = *cdp;
            const Vec2d dir = (path.pts[i + 1] - p).normalized();
            const Vec2d left = dir.perpendicular();
            const Vec2d lp = p + left * (deckHalf - 1.0);
            const f64 y = seaLinkDeckY(path.arc[i], path);
            const std::vector<Vec2d> ring{
                {lp.x - 0.22 - cd.origin.x, lp.y - 0.22 - cd.origin.y},
                {lp.x - 0.22 - cd.origin.x, lp.y + 0.22 - cd.origin.y},
                {lp.x + 0.22 - cd.origin.x, lp.y + 0.22 - cd.origin.y},
                {lp.x + 0.22 - cd.origin.x, lp.y - 0.22 - cd.origin.y}};
            cd.hero.beginSubmesh(matDeck, zone);
            cd.hero.extrudeWalls(ring, y + 1.4, y + 9.5, matDeck, zone, 1, 0.6f);
            cd.hero.endSubmesh();
            ++masts;
        }
        ctx.qa.add("hero.seaLink.lightingMasts", static_cast<f64>(masts));
    }

    ctx.stats.heroAssetCount++;
    ctx.qa.add("hero.seaLink.lengthMetres", path.total);
    ctx.qa.add("hero.seaLink.osmFragmentMetres", path.osmFragment);
    ctx.qa.add("hero.seaLink.pylons", static_cast<f64>(pylonCount));
    ctx.qa.add("hero.seaLink.stays", static_cast<f64>(stays));
    MLOG_INFO("hero: Bandra-Worli Sea Link — %.0f m published alignment "
              "(the OSM extract holds a %.0f m fragment), %llu cable-stayed pylons, "
              "%llu stays, %llu pier bents",
              path.total, path.osmFragment, (unsigned long long)pylonCount,
              (unsigned long long)stays, (unsigned long long)piers);
}

// -----------------------------------------------------------------------------
//  Marine Drive — the Queen's Necklace
// -----------------------------------------------------------------------------
void buildMarineDrive(Context& ctx, u32 matConcrete, u32 matBuilding, u32 matLamp,
                      u32 matGlow, u32 matRoad, u32 zone) {
    static const std::vector<LatLonRow> published = {
        {18.9250, 72.8230}, {18.9280, 72.8215}, {18.9320, 72.8205}, {18.9360, 72.8198},
        {18.9400, 72.8192}, {18.9440, 72.8188}, {18.9480, 72.8186}, {18.9520, 72.8182},
        {18.9550, 72.8172}, {18.9570, 72.8150},
    };
    std::vector<Vec2d> arc = geo2d::resampleOpen(projectPolyline(ctx.proj, published), 40.0);
    const f64 seawallY = ctx.cfg.seaLevel + 2.6;
    const f64 promenadeWidth = 12.0;
    u64 lamps = 0;

    for (size_t i = 0; i + 1 < arc.size(); ++i) {
        const Vec2d a = arc[i], b = arc[i + 1];
        const Vec2d dir = (b - a).normalized();
        const Vec2d left = dir.perpendicular();     // seaward
        const Vec2d mid = (a + b) * 0.5;
        CellData* cdp = ctx.tryCell(mid);
        if (!cdp) continue;                       // outside the world extent
        CellData& cd = *cdp;
        const Vec3 off{static_cast<f32>(cd.origin.x), 0.0f, static_cast<f32>(cd.origin.y)};
        auto P = [&](const Vec2d& p, f64 lateral, f64 y) {
            const Vec2d q = p + left * lateral;
            return Vec3{static_cast<f32>(q.x), static_cast<f32>(y), static_cast<f32>(q.y)} - off;
        };

        cd.hero.beginSubmesh(matConcrete, zone);
        // Promenade deck
        cd.hero.addQuad(P(a, -promenadeWidth, seawallY), P(b, -promenadeWidth, seawallY),
                        P(b, 0.0, seawallY), P(a, 0.0, seawallY),
                        V3(0, 1, 0), matConcrete, zone, 1, 0.15f);
        // Sea-facing bench wall
        ribbonWall(cd.hero, P(a, 0.0, seawallY), P(b, 0.0, seawallY),
                   P(b, 0.0, seawallY + 0.7), P(a, 0.0, seawallY + 0.7),
                   V3(left.x, 0, left.y), matConcrete, zone, 0.5f);
        cd.hero.endSubmesh();

        // The carriageway itself
        cd.roads.beginSubmesh(matRoad, zone);
        cd.roads.addQuad(P(a, -promenadeWidth - 21.0, seawallY + 0.05),
                         P(b, -promenadeWidth - 21.0, seawallY + 0.05),
                         P(b, -promenadeWidth, seawallY + 0.05),
                         P(a, -promenadeWidth, seawallY + 0.05),
                         V3(0, 1, 0), matRoad, zone, 1, 0.1f);
        cd.roads.endSubmesh();

        // Lamp pairs. flags bit1 marks the Queen's Necklace cluster so the
        // runtime can assign the dedicated clustered-light array.
        CellData* lampCdp = ctx.tryCell(mid);
        if (!lampCdp) continue;
        CellData& lampCd = *lampCdp;
        for (int k = 0; k < 2; ++k) {
            const Vec2d p = mid + left * (2.0 + k * 6.0);
            InstanceRecord pole{};
            pole.px = static_cast<f32>(p.x - lampCd.origin.x);
            pole.py = static_cast<f32>(seawallY);
            pole.pz = static_cast<f32>(p.y - lampCd.origin.y);
            pole.qw = 1.0f;
            pole.sx = pole.sy = pole.sz = 1.0f;
            pole.meshId = 5;
            pole.materialId = matLamp;
            pole.flags = 0x02;
            lampCd.instances.push_back(pole);

            InstanceRecord glow = pole;
            glow.py += 8.6f;
            glow.meshId = 0;
            glow.materialId = matGlow;
            lampCd.instances.push_back(glow);
            ctx.stats.instanceCount += 2;
            ++lamps;
        }

        // Art Deco frontage on the landward side every third segment.
        if ((i % 3) == 0) {
            const Vec2d front = mid - left * (promenadeWidth + 24.0);
            const std::vector<Vec2d> block{{front.x - 9.0, front.y - 9.0}, {front.x - 9.0, front.y + 9.0},
                                           {front.x + 9.0, front.y + 9.0}, {front.x + 9.0, front.y - 9.0}};
            CellData* bcdp = ctx.tryCell(front);
            if (!bcdp) continue;
            CellData& bcd = *bcdp;
            const f64 g = ctx.terrainHF.sample(front.x, front.y);
            std::vector<Vec2d> local;
            for (const Vec2d& p : block)
                local.push_back({p.x - bcd.origin.x, p.y - bcd.origin.y});
            bcd.buildings[0].beginSubmesh(matBuilding, zone);
            bcd.buildings[0].extrudeWalls(local, g - 0.4, g + 22.0, matBuilding, zone, 1, 0.25f);
            bcd.buildings[0].addRoofCap(local, g + 22.0, matBuilding, zone, 1, 0.25f, 0.4, 0.5);
            bcd.buildings[0].endSubmesh();
        }
    }

    ctx.stats.heroAssetCount++;
    ctx.qa.add("hero.marineDrive.lamps", static_cast<f64>(lamps));
    MLOG_INFO("hero: Marine Drive — %zu arc segments, %llu lamps in the necklace array",
              arc.size() - 1, (unsigned long long)lamps);
}

} // namespace

bool buildHeroAssets(Context& ctx) {
    if (!ctx.cfg.buildHero) return true;
    ScopedTimer timer("buildHeroAssets");

    const u32 matStone = ctx.materials.id("hero_stone");
    const u32 matTerra = ctx.materials.id("hero_terracotta");
    const u32 matConcrete = ctx.materials.id("hero_concrete");
    const u32 matGlass = ctx.materials.id("curtain_wall_glass");
    const u32 matPylon = ctx.materials.id("sea_link_pylon");
    const u32 matCable = ctx.materials.id("sea_link_cable");
    const u32 matDeck = ctx.materials.id("concrete_road");
    const u32 matLamp = ctx.materials.id("streetlamp_pole");
    const u32 matGlow = ctx.materials.id("streetlamp_glow");
    const u32 matRoad = ctx.materials.id("asphalt_smooth");
    const u32 matPaver = ctx.materials.id("paver_block");
    const u32 zone = static_cast<u32>(ZoningZone::SouthBombayColonial);

    struct Landmark { const char* name; f64 lat, lon; int kind; };
    const Landmark landmarks[] = {
        {"Gateway of India", 18.9220, 72.8347, 0},
        {"Taj Mahal Palace", 18.9216, 72.8332, 1},
        {"Chhatrapati Shivaji Maharaj Terminus", 18.9398, 72.8355, 2},
    };

    for (const Landmark& lm : landmarks) {
        const Vec2d base = ctx.proj.forward(lm.lat, lm.lon);
        const f64 groundY = ctx.terrainHF.sample(base.x, base.y);
        CellData* cdp = ctx.tryCell(base);
        if (!cdp) continue;                       // outside the world extent
        CellData& cd = *cdp;
        // Every hero mesh is authored around its own origin and appended into
        // the owning cell through appendTranslated, so the section holds
        // cell-local coordinates exactly like every other geometry section.
        // (Building straight into the cell builder is how the three landmarks
        // once ended up in two different coordinate spaces at once.)
        mesh::MeshBuilder scratch;
        switch (lm.kind) {
            case 0: buildGateway(ctx, {0.0, 0.0}, groundY, scratch, matStone, matTerra, zone); break;
            case 1: buildTaj(ctx, {0.0, 0.0}, groundY, scratch, matStone, matTerra, matGlass, zone); break;
            default: buildCsmt(ctx, {0.0, 0.0}, groundY, scratch, matStone, matTerra, matConcrete, zone); break;
        }
        const u64 tris = scratch.triangleCount();
        scratch.appendTranslated(cd.hero, Vec3{static_cast<f32>(base.x - cd.origin.x), 0.0f,
                                               static_cast<f32>(base.y - cd.origin.y)});
        ctx.stats.heroTriangles += tris;
        ctx.stats.heroAssetCount++;
        ctx.qa.add(std::string("hero.") + lm.name + ".triangles", static_cast<f64>(tris));
        MLOG_INFO("hero: %s at %.4f,%.4f — %llu triangles (spec target 300k-1.2M)",
                  lm.name, lm.lat, lm.lon, (unsigned long long)tris);

        CollisionBox cb{};
        cb.cx = static_cast<f32>(base.x - cd.origin.x);
        cb.cz = static_cast<f32>(base.y - cd.origin.y);
        cb.cy = static_cast<f32>(groundY + 12.0);
        cb.hx = 20.0f; cb.hz = 20.0f; cb.hy = 12.0f;
        cb.qw = 1.0f;
        cb.flags = 2;     // hero mesh collider
        cd.collision.push_back(cb);
        ctx.stats.collisionBoxes++;
    }

    {
        const u64 before = heroTrianglesInCells(ctx);
        buildSeaLink(ctx, matPylon, matCable, matDeck, static_cast<u32>(ZoningZone::BKCCommercial));
        ctx.qa.add("hero.seaLink.triangles", static_cast<f64>(heroTrianglesInCells(ctx) - before));
    }
    {
        const u64 before = heroTrianglesInCells(ctx);
        buildMarineDrive(ctx, matConcrete, matPaver, matLamp, matGlow, matRoad, zone);
        ctx.qa.add("hero.marineDrive.triangles", static_cast<f64>(heroTrianglesInCells(ctx) - before));
    }

    ctx.qa.add("hero.assetCount", static_cast<f64>(ctx.stats.heroAssetCount));
    ctx.qa.add("hero.totalTriangles", static_cast<f64>(heroTrianglesInCells(ctx)));
    MLOG_INFO("hero assets: %llu assets, %llu triangles total",
              (unsigned long long)ctx.stats.heroAssetCount,
              (unsigned long long)heroTrianglesInCells(ctx));
    return true;
}

} // namespace mumbai::pipeline
