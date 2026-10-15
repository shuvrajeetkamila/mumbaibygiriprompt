// =============================================================================
//  Mumbai 1:1 EXACT — engine/app/phys_main.cpp
//
//  The A2 acceptance harness (mumbai_phys).  It is the instrument behind every
//  physics claim the project makes, and it is deliberately built out of the
//  pipeline's *own* published collision: the same `.mumbai_chunk` files the
//  runtime streams, read through the same mmap'd reader, baked into Jolt by
//  engine/physics/bake.cpp.  Nothing here authors geometry; if the world build
//  puts the Sea Link deck in the wrong place, this harness falls off the bridge
//  and says so — which is how the deck's doubled side sign and inverted pitch
//  were found.
//
//  What it measures, section by section:
//
//    bake        every cell's collision into the world: boxes, capsules, terrain
//                heightfields, optionally road meshes.  Reports the inventory.
//    coverage    a probe grid across every cell of the built extent: the city
//                must have colliders everywhere, not only where it was authored.
//    deck        the Sea Link carriageway chain, recovered from the collision
//                records (thin hero-collider slabs) and confirmed by ray casts.
//    rides       both vehicle controllers on that chain: a motorcycle (lean
//                spring, and a slalom to prove the lean controller leans), a
//                car, the 16 t double-decker, and the three-wheeled auto.
//    braking     a car from 60 km/h, full brake, against v^2 / (2 * mu * g).
//    impact      a 300 kg body dropped 22 m onto the deck, and onto the street
//                grid: it must rest at its analytic height and not tunnel.
//    fleet       N driving vehicles at once: cost per vehicle per step, and the
//                extrapolation to Section 7's 500 vehicles in 500 m.
//    origin      Section 4's floating origin: 20 macro-cell crossings, worst
//                wall time, exactness of the shift, and no velocity impulse.
//    soak        the world stepped for simulated minutes with the fleet live:
//                step-cost percentiles, deepest penetration, body census.
//
//  Usage:
//      mumbai_phys [--chunks data/processed/cells] [--minutes 1] [--seconds 30]
//                  [--fleet 24] [--roads] [--deck-kph 45] [--props 200]
//                  [--max-chunks N] [--report build/physics_report.json]
//                  [--quick] [--quiet] [--trace]
//
//  Exit code: 0 when every check passes, 2 when any check fails, 3 when the
//  world is not on disk (so a data-less CI can tell "not built" from "broken").
// =============================================================================

#include "engine/core/core.h"
#include "engine/core/geo.h"
#include "engine/core/json.h"
#include "engine/core/math.h"
#include "engine/physics/bake.h"
#include "engine/physics/jolt.h"
#include "engine/physics/driver.h"
#include "engine/physics/vehicle.h"
#include "engine/physics/world.h"
#include "engine/sim/traffic.h"
#include "engine/world/chunk.h"

#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

using namespace mumbai;

namespace {

namespace fs = std::filesystem;
namespace J = JPH;

constexpr f64 kPi = 3.14159265358979323846;
constexpr f64 kStep = 1.0 / 60.0;          // the runtime's fixed tick
// A wheel's centre hangs 0.35 * radius below the chassis box's bottom face in
// spawnVehicle (axleY), so a tyre sits on a surface when the *chassis centre* is
// `halfHeight + 0.65 * radius` above it.  Vehicles are placed 5 cm above that and
// settle onto their suspension; the deck must not be asked to push a body out of
// itself, because that penetration would then be measured as the harness's own.
constexpr f64 kSpawnClearance = 0.05;
// Penetration budget.  Jolt's default contact slop is 0.005 m, so anything at or
// under it is "touching", not "penetrating"; the acceptance criterion is zero
// visible interpenetration, which this project reads as 20 mm (a hand's width on
// a 1:1 city is invisible; a wheel in the deck is not).
constexpr f64 kPenetrationBudget = 0.02;
// An impact is measured differently from a resting contact, and the difference is
// the whole point of the number.  The 300 kg body in the impact test is dropped
// from 22 m, so it meets the deck at ~20.6 m/s and *moves 0.34 m in one 60 Hz
// tick*.  Jolt's CCD stops the body at the surface (measured: 19.4 m/s -> 7.3 m/s
// within the tick, the body left 17 mm above the deck), but the contact it reports
// for that tick is the shape cast's own penetration depth over the swept interval
// — 0.25 m here, which is Jolt's `mLinearCastMaxPenetration` (0.25 * inner radius)
// doing exactly what it says.  So the criteria that are actually about the world
// are these three, and all three are checked:
//   * at rest, after the impact, the deepest penetration is inside the ordinary
//     budget (20 mm) — measured 5 mm (the slop);
//   * the body never ends a tick below the surface it hit (no tunnelling);
//   * it rests at its analytic height (one half-extent above the surface).
// The transient is *reported* — 0.25 m for one tick at 20 m/s is a fact about a
// discrete solver, not a defect to be hidden behind a different drop height.
constexpr f64 kImpactTransientBudget = 0.30;

f64 wrapPi(f64 a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

std::string sfmt(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return std::string(buf);
}

bool g_quiet = false;
bool g_trace = false;

// --- checks -----------------------------------------------------------------
struct Check {
    std::string name;
    bool ok = false;
    std::string detail;
};
std::vector<Check> g_checks;

void section(const std::string& title) {
    if (!g_quiet) std::printf("\n== %s ==\n", title.c_str());
}

void require(bool ok, const std::string& name, const std::string& detail = std::string()) {
    Check c;
    c.name = name;
    c.ok = ok;
    c.detail = detail;
    g_checks.push_back(c);
    if (!g_quiet)
        std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", name.c_str(),
                    detail.empty() ? "" : " — ", detail.c_str());
}

// --- options ----------------------------------------------------------------
struct Options {
    std::string chunks = "data/processed/cells";
    std::string anchors;                 // viewer manifest, for the deck anchor
    std::string report = "build/physics_report.json";
    f64 minutes = 1.0;                   // soak length, simulated
    f64 seconds = 30.0;                  // length of each vehicle test
    f64 deckKph = 45.0;                  // motorcycle cruise speed
    u32 fleet = 24;                      // vehicles in the fleet-cost section
    u32 props = 200;                     // dynamic crates in the soak
    u32 drive = 24;                      // vehicles handed to the physical driver
    f64 surveySpacing = 0.0;             // metres between lane/collision survey samples
    u32 maxChunks = 0;                   // 0 = every cell
    // Road-mesh collision on by default: a vehicle drives on the road, and the
    // graded heightfield under it is a 32 m approximation of the same surface.  The
    // measured difference is the wheels: 92.6 % of wheel contacts down on wheels
    // that follow the heightfield, 97.6 % on wheels that follow the road (D-034).
    bool roads = true;                   // bake road meshes city-wide (heavy)
    bool surveyTrace = false;            // dump the rays behind no-surface samples
    bool quick = false;
    bool noSteer = false;                // diagnostic: run the ride with hands off
    f64 holdSteer = std::numeric_limits<f64>::quiet_NaN();   // diagnostic: constant steering
};

Options parse(int argc, char** argv, bool& help) {
    Options opt;
    help = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto nextD = [&](f64 def) { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        auto nextI = [&](u32 def) -> u32 {
            return (i + 1 < argc) ? static_cast<u32>(std::max(0.0, std::atof(argv[++i]))) : def;
        };
        if (a == "--chunks" && i + 1 < argc) opt.chunks = argv[++i];
        else if (a == "--anchors" && i + 1 < argc) opt.anchors = argv[++i];
        else if (a == "--report" && i + 1 < argc) opt.report = argv[++i];
        else if (a == "--minutes") opt.minutes = nextD(opt.minutes);
        else if (a == "--seconds") opt.seconds = nextD(opt.seconds);
        else if (a == "--deck-kph") opt.deckKph = nextD(opt.deckKph);
        else if (a == "--fleet") opt.fleet = nextI(opt.fleet);
        else if (a == "--props") opt.props = nextI(opt.props);
        else if (a == "--drive") opt.drive = nextI(opt.drive);
        else if (a == "--survey") opt.surveySpacing = std::atof(argv[++i]);
        else if (a == "--max-chunks") opt.maxChunks = nextI(0);
        else if (a == "--survey-trace") opt.surveyTrace = true;
        else if (a == "--roads") opt.roads = true;
        else if (a == "--no-roads") opt.roads = false;
        else if (a == "--quick") {
            opt.quick = true;
            opt.minutes = std::min(opt.minutes, 0.25);
            opt.seconds = std::min(opt.seconds, 8.0);
            opt.fleet = std::min(opt.fleet, 8u);
            opt.props = std::min(opt.props, 60u);
            opt.drive = std::min(opt.drive, 12u);
        } else if (a == "--quiet") g_quiet = true;
        else if (a == "--no-steer") opt.noSteer = true;
        else if (a == "--hold-steer") opt.holdSteer = nextD(opt.holdSteer);
        else if (a == "--trace") { g_trace = true; g_quiet = false; }
        else if (a == "--help" || a == "-h") { help = true; }
        else std::printf("  (ignoring unknown argument '%s')\n", a.c_str());
    }
    return opt;
}

// -----------------------------------------------------------------------------
//  The world on disk
// -----------------------------------------------------------------------------
struct CellRef {
    std::string path;
    i64 cellX = 0, cellZ = 0;
    f32 cellSize = MACRO_CELL_SIZE;
};

// A caller may pass data/processed (the build directory) or data/processed/cells
// (the tiles themselves).  Both mean the same thing to the harness.
std::string resolveChunkDir(const std::string& dir) {
    if (fs::exists(dir + "/cells")) return dir + "/cells";
    return dir;
}

std::vector<CellRef> listCells(const std::string& dir) {
    std::vector<CellRef> cells;
    const std::string root = resolveChunkDir(dir);
    if (!fs::exists(root)) return cells;
    for (const auto& e : fs::directory_iterator(root)) {
        const std::string p = e.path().string();
        if (e.path().extension() != ".mumbai_chunk") continue;
        phys::ChunkInfo info;
        if (!phys::readChunkInfo(p, info)) continue;
        CellRef c;
        c.path = p;
        c.cellX = info.cellX;
        c.cellZ = info.cellZ;
        c.cellSize = info.cellSize;
        cells.push_back(c);
    }
    std::sort(cells.begin(), cells.end(), [](const CellRef& a, const CellRef& b) {
        return a.cellZ != b.cellZ ? a.cellZ < b.cellZ : a.cellX < b.cellX;
    });
    return cells;
}

// -----------------------------------------------------------------------------
//  Landmarks: the published payload's own coordinates
// -----------------------------------------------------------------------------
struct Landmark {
    std::string name;
    f64 x = 0, z = 0;      // world metres
    f64 lat = 0, lon = 0;
};

// The viewer manifest holds landmarks in published (viewer-relative) metres and
// carries the extent they are relative to.  The world frame is that frame plus
// the extent's minimum corner — the same arithmetic the runtime's floating origin
// does, and the same one the pipeline writes.
std::vector<Landmark> loadLandmarks(const std::string& path, Vec2d& extentMinOut, bool& haveExtent) {
    std::vector<Landmark> out;
    haveExtent = false;
    std::ifstream in(path);
    if (!in) return out;
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    Json doc;
    std::string err;
    if (!Json::parse(text, doc, &err)) return out;
    f64 minX = 0, minZ = 0;
    const Json& ext = doc["extent"];
    if (ext.isObject()) {
        minX = ext["minX"].asNumber(0.0);
        minZ = ext["minZ"].asNumber(0.0);
        extentMinOut = Vec2d(minX, minZ);
        haveExtent = true;
    }
    const Json& lm = doc["landmarks"];
    if (lm.isArray()) {
        for (size_t i = 0; i < lm.size(); ++i) {
            Landmark l;
            l.name = lm[i]["name"].asStringOr("");
            l.lat = lm[i]["lat"].asNumber(0.0);
            l.lon = lm[i]["lon"].asNumber(0.0);
            l.x = lm[i]["x"].asNumber(0.0) + minX;
            l.z = lm[i]["z"].asNumber(0.0) + minZ;
            if (!l.name.empty()) out.push_back(l);
        }
    }
    return out;
}

// -----------------------------------------------------------------------------
//  The Sea Link deck: recovered from the collision records
// -----------------------------------------------------------------------------
// A deck slab is a hero-collider box (flag bit 4) that is thin in Y: `hy` is
// 0.34 m in buildSeaLink().  There is no other record of the bridge in the
// chunk, and reading the geometry back out of the world the pipeline published
// is the point: it is the same slab the player's wheels will meet.
struct Slab {
    f64 x = 0, y = 0, z = 0;   // collider centre, world metres
    f64 yaw = 0;               // radians, from the record's own quaternion
    f64 hz = 0;                // half length along the slab's forward axis
    f64 hy = 0;                // half thickness: the record says where the top is
    f64 surface = 0;           // the top face, measured by a ray cast
    bool hasSurface = false;
};

struct DeckChain {
    bool ok = false;
    std::vector<Slab> slabs;       // ordered in the drive direction
    f64 length = 0;                // centre-to-centre arc, plus one slab
    f64 yawStart = 0, yawEnd = 0;  // the chain's own heading, start and end
    f64 width = 0;                 // measured by probing across the mid-slab
    f64 minY = 0, maxY = 0;        // the deck's height band
    f64 surfaceOffset = 0;         // record centre -> top face, median over slabs
    f64 surfaceSpread = 0;         // worst deviation of one slab from that offset
    u64 surfaceMissing = 0;        // slabs the probe could not reach
    u64 surfaceObstructed = 0;     // slabs with the bridge's own structure overhead
    std::string note;
};

// The ride surface of a slab: its measured top face when the probe found one,
// and the record's own centre plus the chain's median offset otherwise.
f64 surfaceOf(const DeckChain& chain, size_t i) {
    const Slab& s = chain.slabs[std::min(i, chain.slabs.size() - 1)];
    return s.hasSurface ? s.surface : s.y + chain.surfaceOffset;
}

bool readSlabs(const std::vector<CellRef>& cells, std::vector<Slab>& out) {
    for (const CellRef& c : cells) {
        ChunkReader reader;
        if (!reader.open(c.path)) continue;
        u32 count = 0;
        const CollisionBox* boxes = reader.array<CollisionBox>(SectionType::BuildingCollision, &count);
        if (boxes == nullptr) continue;
        const f64 ox = static_cast<f64>(c.cellX) * c.cellSize;
        const f64 oz = static_cast<f64>(c.cellZ) * c.cellSize;
        for (u32 i = 0; i < count; ++i) {
            const CollisionBox& b = boxes[i];
            if ((b.flags & 4u) == 0) continue;              // not a hero collider
            if (std::fabs(static_cast<f64>(b.hy) - 0.34) > 1e-3) continue;   // not a deck slab
            Slab s;
            s.x = ox + b.cx;
            s.y = b.cy;
            s.z = oz + b.cz;
            s.hz = b.hz;
            s.hy = b.hy;
            // Yaw about +Y: a box is symmetric in Z, so this identifies its axis.
            s.yaw = std::atan2(2.0 * (static_cast<f64>(b.qw) * b.qy + static_cast<f64>(b.qx) * b.qz),
                               1.0 - 2.0 * (static_cast<f64>(b.qy) * b.qy +
                                            static_cast<f64>(b.qx) * b.qx));
            out.push_back(s);
        }
    }
    return !out.empty();
}

// Grows the slab containing (or nearest to) `seed` into a chain by stepping slab
// to slab along the slab's own axis.  Consecutive slabs of one carriageway of the
// Sea Link are 19.6 m apart centre-to-centre; two carriageways are 23 m apart
// laterally, so a step that allows a lateral jump would hop between them and the
// chain's heading would come out wrong (that is exactly how the first version of
// this survey produced a 21.8 deg deck, 6 deg off the bridge's own axis).
DeckChain growChain(const std::vector<Slab>& slabs, size_t seedIndex, f64 dirSign) {
    DeckChain chain;
    std::vector<bool> used(slabs.size(), false);
    std::vector<Slab> fwd, back;
    auto step = [&](const Slab& from, const Slab& to) {
        const f64 dx = to.x - from.x, dz = to.z - from.z;
        const f64 dist = std::hypot(dx, dz);
        // Lateral offset from `from`'s own axis.
        const f64 fx = std::sin(from.yaw), fz = std::cos(from.yaw);
        const f64 lateral = std::fabs(-fz * dx + fx * dz);
        return std::pair<f64, f64>(dist, lateral);
    };
    auto next = [&](const Slab& from, f64 sign) -> int {
        const f64 fx = sign * std::sin(from.yaw), fz = sign * std::cos(from.yaw);
        int best = -1;
        f64 bestCost = 1e30;
        for (size_t i = 0; i < slabs.size(); ++i) {
            if (used[i]) continue;
            const f64 dx = slabs[i].x - from.x, dz = slabs[i].z - from.z;
            const f64 along = dx * fx + dz * fz;
            if (along < 0.35 * from.hz || along > 2.2 * from.hz) continue;
            const auto [dist, lateral] = step(from, slabs[i]);
            if (lateral > 5.0) continue;
            const f64 cost = std::fabs(along - 2.0 * from.hz) + 2.0 * lateral;
            if (cost < bestCost) { bestCost = cost; best = static_cast<int>(i); }
        }
        return best;
    };
    used[seedIndex] = true;
    Slab cur = slabs[seedIndex];
    const Slab seed = cur;
    (void)seed;
    for (int dir = 0; dir < 2; ++dir) {
        const f64 sign = dir == 0 ? dirSign : -dirSign;
        cur = slabs[seedIndex];
        std::vector<Slab>& into = dir == 0 ? fwd : back;
        while (true) {
            const int n = next(cur, sign);
            if (n < 0) break;
            used[n] = true;
            into.push_back(slabs[n]);
            cur = slabs[n];
        }
    }
    std::reverse(back.begin(), back.end());
    chain.slabs.reserve(back.size() + 1 + fwd.size());
    chain.slabs.insert(chain.slabs.end(), back.begin(), back.end());
    chain.slabs.push_back(slabs[seedIndex]);
    chain.slabs.insert(chain.slabs.end(), fwd.begin(), fwd.end());
    if (chain.slabs.size() < 4) return chain;
    f64 len = 0;
    for (size_t i = 1; i < chain.slabs.size(); ++i)
        len += std::hypot(chain.slabs[i].x - chain.slabs[i - 1].x,
                          chain.slabs[i].z - chain.slabs[i - 1].z);
    chain.length = len;
    chain.yawStart = std::atan2(chain.slabs[1].x - chain.slabs[0].x,
                                chain.slabs[1].z - chain.slabs[0].z);
    chain.yawEnd = std::atan2(chain.slabs.back().x - chain.slabs[chain.slabs.size() - 2].x,
                              chain.slabs.back().z - chain.slabs[chain.slabs.size() - 2].z);
    chain.minY = chain.maxY = chain.slabs[0].y;
    for (const Slab& s : chain.slabs) {
        chain.minY = std::min(chain.minY, s.y);
        chain.maxY = std::max(chain.maxY, s.y);
    }
    chain.ok = true;
    return chain;
}

DeckChain surveyDeck(phys::World& world, const std::vector<CellRef>& cells, const Vec2d& anchor) {
    std::vector<Slab> slabs;
    readSlabs(cells, slabs);
    DeckChain chain;
    if (slabs.empty()) {
        chain.note = "no deck-slab collision records in the world";
        return chain;
    }
    size_t seed = 0;
    f64 best = 1e30;
    for (size_t i = 0; i < slabs.size(); ++i) {
        const f64 d = std::hypot(slabs[i].x - anchor.x, slabs[i].z - anchor.y);
        if (d < best) { best = d; seed = i; }
    }
    // Drive south (the Bandra -> Worli direction the published alignment is
    // written in): pick the sign whose half-plane holds more slabs.
    const Slab& s0 = slabs[seed];
    const f64 nx = std::sin(s0.yaw), nz = std::cos(s0.yaw);
    int ahead = 0, behind = 0;
    for (const Slab& s : slabs) {
        if (std::hypot(s.x - s0.x, s.z - s0.z) > 3000.0) continue;
        const f64 along = (s.x - s0.x) * nx + (s.z - s0.z) * nz;
        if (along > 0) ++ahead; else ++behind;
    }
    chain = growChain(slabs, seed, ahead >= behind ? 1.0 : -1.0);
    if (!chain.ok) { chain.note = "the anchor's slab does not chain to its neighbours"; return chain; }

    // Probe every slab: the ride height above each one, and the width across the
    // middle.  This is what proves the record and the collider are the same
    // object, not merely the same intent — and it is where the collider's top
    // face is read, because the record's centre is *not* the surface: buildSeaLink
    // writes the slab as a 0.68 m-thick box centred 0.22 m below the running
    // surface, so the top face sits 0.12 m above the record's `cy`.  A vehicle
    // spawned off the record alone starts inside the deck and is pushed out by the
    // solver — which the harness then measures as its own penetration.
    std::vector<f64> offsets;
    offsets.reserve(chain.slabs.size());
    for (Slab& s : chain.slabs) {
        // Cast from just above the record's own centre: a ray from 20 m up can be
        // intercepted by the bridge's own parapet, cable stays, gantry or the
        // pylon leg above the deck, and then the slab reads as unprobeable.
        const f64 h = world.groundHeightStatic(Vec3(static_cast<f32>(s.x), static_cast<f32>(s.y + 3.0),
                                                    static_cast<f32>(s.z)), 12.0);
        // The record says the slab is `hy` thick with its centre at `y`, so its
        // top face is `y + hy` (less the cosine of a small pitch).  The probe has
        // to find *that*, within 5 cm, or the collider and the record disagree.
        if (std::isfinite(h) && std::fabs(h - (s.y + s.hy)) < 0.05) {
            s.surface = h;
            s.hasSurface = true;
            offsets.push_back(h - s.y);
        } else if (std::isfinite(h) && h > s.y + 1.0) {
            // The probe started inside something: at a pylon the leg and the
            // raked stays cross the carriageway, and the ray from 3 m above the
            // deck begins *inside* them.  That is the bridge's own structure, not
            // a misplaced slab, and it is counted and reported separately.
            ++chain.surfaceObstructed;
            if (g_trace)
                std::printf("      slab %6.1f %6.1f (y %6.2f): obstructed above (probe %s)\n",
                            s.x, s.z, s.y, sfmt("%.2f", h).c_str());
        } else {
            ++chain.surfaceMissing;
            if (g_trace)
                std::printf("      slab %6.1f %6.1f (y %6.2f): no surface within 3 m (probe %s)\n",
                            s.x, s.z, s.y, std::isfinite(h) ? sfmt("%.2f", h).c_str() : "missed");
        }
    }
    if (!offsets.empty()) {
        std::vector<f64> sorted = offsets;
        std::sort(sorted.begin(), sorted.end());
        chain.surfaceOffset = sorted[sorted.size() / 2];
        for (const Slab& s : chain.slabs) {
            if (!s.hasSurface) continue;
            const f64 dev = std::fabs((s.surface - s.y) - chain.surfaceOffset);
            chain.surfaceSpread = std::max(chain.surfaceSpread, dev);
            if (g_trace && dev > 0.01)
                std::printf("      slab %6.1f %6.1f: top face %.3f m over the record's centre, "
                            "%.3f m off the chain's %.3f m\n", s.x, s.z, s.surface - s.y, dev,
                            chain.surfaceOffset);
        }
    }

    const size_t mid = chain.slabs.size() / 2;
    const Slab& m = chain.slabs[mid];
    const f64 rx = std::cos(m.yaw), rz = -std::sin(m.yaw);
    auto heightAt = [&](f64 lateral) {
        return world.groundHeightStatic(Vec3(static_cast<f32>(m.x + rx * lateral),
                                             static_cast<f32>(m.y + 3.0),
                                             static_cast<f32>(m.z + rz * lateral)), 60.0);
    };
    const f64 midY = m.hasSurface ? m.surface : heightAt(0.0);
    if (!std::isfinite(midY)) {
        chain.note = "the mid-chain slab has no collider under it";
        return chain;
    }
    f64 half = 0.0;
    for (f64 t = 0.5; t <= 14.0; t += 0.5) {
        const f64 h = heightAt(t);
        if (!std::isfinite(h) || std::fabs(h - midY) > 1.0) break;
        half = t;
    }
    chain.width = 2.0 * half;
    chain.note = sfmt("%zu slabs, %.0f m, y %.2f..%.2f m", chain.slabs.size(), chain.length,
                      chain.minY, chain.maxY);
    return chain;
}

// -----------------------------------------------------------------------------
//  Riding the chain
// -----------------------------------------------------------------------------
struct RideResult {
    std::string name;
    f64 seconds = 0, metres = 0;
    f64 meanKph = 0, maxKph = 0, minKph = 1e30;
    f64 leanPeak = 0, leanEnd = 0;
    f64 lateralMax = 0;
    f64 wheelsDownFrac = 0;
    f64 lowestBelowDeck = 0;
    f64 deepestPenetration = 0;
    u64 slabStations = 0;
    bool fellOff = false, stalled = false, finite = true;
};

struct RideInput {
    f64 targetKph = 45.0;
    f64 seconds = 30.0;
    f64 slalomAmp = 0.0;       // added to the follow-the-chain steering
    f64 slalomPeriod = 3.0;    // s
    f64 initialSpeed = 6.0;    // m/s the vehicle is already rolling at
    f64 followGain = 0.9;      // steering per radian of heading error
    f64 lookahead = 26.0;      // m of chain to aim along
    bool brakeAtEnd = false;   // full brake instead of throttle (stopping test)
    bool handsOff = false;     // diagnostic: no steering input at all
    f64 holdSteer = std::numeric_limits<f64>::quiet_NaN();   // diagnostic: constant steering
    bool allowFallOff = false; // a test that expects to leave the deck
};

// Distance from a point to the chain's polyline — the cross-track error a driver
// would see, not the distance to the nearest slab centre (which oscillates
// between 0 and half a slab length as the vehicle drives past the joints and
// made the first version of this harness report a 9 m lateral error for a
// perfectly centred ride).
f64 chainDistance(const DeckChain& chain, f64 x, f64 z, size_t* indexOut) {
    f64 best = 1e30;
    size_t bestI = 0;
    for (size_t i = 0; i < chain.slabs.size(); ++i) {
        const f64 ax = chain.slabs[i].x, az = chain.slabs[i].z;
        const f64 bx = i + 1 < chain.slabs.size() ? chain.slabs[i + 1].x : ax;
        const f64 bz = i + 1 < chain.slabs.size() ? chain.slabs[i + 1].z : az;
        const f64 dx = bx - ax, dz = bz - az;
        const f64 len2 = dx * dx + dz * dz;
        f64 t = len2 > 0.0 ? ((x - ax) * dx + (z - az) * dz) / len2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        const f64 px = ax + dx * t, pz = az + dz * t;
        const f64 d = std::hypot(x - px, z - pz);
        if (d < best) { best = d; bestI = i; }
    }
    if (indexOut) *indexOut = bestI;
    return best;
}

RideResult rideChain(phys::World& world, phys::Vehicle& v, const DeckChain& chain,
                     const RideInput& in) {
    RideResult r;
    r.name = sim::archetypeSpec(v.kind).name;
    r.seconds = in.seconds;

    // Put the vehicle on the chain: the middle slab's centre, at riding height,
    // facing along the chain.
    const size_t startIndex = chain.slabs.size() / 4;
    const Slab& startSlab = chain.slabs[startIndex];
    const f64 startSurface = surfaceOf(chain, startIndex);
    const f64 rest = startSurface + v.halfHeight + 0.65 * v.wheelRadius + kSpawnClearance;
    world.setPosition(v.body, Vec3(static_cast<f32>(startSlab.x), static_cast<f32>(rest),
                                   static_cast<f32>(startSlab.z)));
    const size_t aheadIndex = std::min(chain.slabs.size() - 1, chain.slabs.size() / 4 + 1);
    const f64 yaw = std::atan2(chain.slabs[aheadIndex].x - startSlab.x,
                               chain.slabs[aheadIndex].z - startSlab.z);
    world.bodies().SetRotation(v.body.id, J::Quat::sRotation(J::Vec3::sAxisY(),
                                                             static_cast<f32>(yaw)), J::EActivation::Activate);
    world.bodies().SetLinearVelocity(v.body.id,
                                     J::Vec3(std::sin(yaw) * in.initialSpeed, 0.0f,
                                             std::cos(yaw) * in.initialSpeed));
    r.lowestBelowDeck = 0.0;

    const u64 steps = static_cast<u64>(std::llround(in.seconds / kStep));
    const f64 targetMs = in.targetKph / 3.6;
    f64 steer = 0.0;
    f64 controlTimer = 0.0;
    constexpr f64 kControlPeriod = 0.2;      // s between steering decisions
    constexpr f64 kSteerRate = 0.4;          // of full range per second
    f64 time = 0.0, travelled = 0.0, speedSum = 0.0;
    u64 samples = 0, grounded = 0;
    const int wheelCount = v.constraint != nullptr
                               ? static_cast<int>(v.constraint->GetWheels().size()) : 4;
    Vec3 prev = world.position(v.body);
    world.penetration().reset();

    for (u64 i = 0; i < steps; ++i) {
        const Vec3 pos = world.position(v.body);
        const J::Quat q = world.bodies().GetRotation(v.body.id);
        const J::Vec3 fwd = q.RotateAxisZ();
        const f64 heading = std::atan2(fwd.GetX(), fwd.GetZ());

        if (controlTimer <= 0.0 && !in.handsOff) {
            controlTimer = kControlPeriod;
            size_t here = 0;
            chainDistance(chain, pos.x, pos.z, &here);
            // Walk forward along the chain to the aim point.
            f64 acc = 0.0;
            size_t k = here;
            while (k + 1 < chain.slabs.size() && acc < in.lookahead) {
                acc += std::hypot(chain.slabs[k + 1].x - chain.slabs[k].x,
                                  chain.slabs[k + 1].z - chain.slabs[k].z);
                ++k;
            }
            const Slab& aim = chain.slabs[k];
            const f64 want = std::atan2(aim.x - pos.x, aim.z - pos.z);
            const f64 err = wrapPi(want - heading);
            if (g_trace)
                std::printf("      aim slab %zu of %zu (%.1f, %.1f), heading %.1f deg, "
                            "want %.1f deg, err %+.2f rad, steer %+.2f -> %+.2f\n",
                            k, chain.slabs.size(), aim.x, aim.z, heading * 180.0 / kPi,
                            want * 180.0 / kPi, err, steer,
                            std::max(-1.0, std::min(1.0, in.followGain * err)));
            const f64 cmd = std::max(-1.0, std::min(1.0, in.followGain * err));
            const f64 maxStep = kSteerRate * kControlPeriod;
            steer += std::max(-maxStep, std::min(maxStep, cmd - steer));
        }
        controlTimer -= kStep;
        f64 steerCmd = steer;
        if (in.holdSteer == in.holdSteer) steerCmd = in.holdSteer;
        if (in.slalomAmp != 0.0)
            steerCmd = std::max(-1.0, std::min(1.0,
                          steer + in.slalomAmp * std::sin(2.0 * kPi * time / in.slalomPeriod)));

        const f64 speed = phys::forwardSpeed(world, v);
        f32 forwardCmd = 0.0f, brakeCmd = 0.0f;
        if (in.brakeAtEnd) {
            forwardCmd = 0.0f;
            brakeCmd = 1.0f;
        } else {
            const f64 ramp = std::min(1.0, time / 1.5);
            if (speed < targetMs - 0.3) forwardCmd = static_cast<f32>(ramp * std::min(1.0, (targetMs - speed) / 2.0));
            else if (speed > targetMs + 0.3) brakeCmd = static_cast<f32>(std::min(1.0, (speed - targetMs) / 2.0));
        }
        phys::driveVehicle(v, forwardCmd, static_cast<f32>(steerCmd), brakeCmd);

        world.step(static_cast<f32>(kStep));
        time += kStep;

        const Vec3 now = world.position(v.body);
        if (!std::isfinite(now.x) || !std::isfinite(now.y) || !std::isfinite(now.z)) {
            r.finite = false;
            break;
        }
        travelled += std::hypot(static_cast<f64>(now.x) - prev.x, static_cast<f64>(now.z) - prev.z);
        prev = now;
        const f64 kph = std::fabs(phys::forwardSpeed(world, v)) * 3.6;
        speedSum += kph;
        ++samples;
        r.maxKph = std::max(r.maxKph, kph);
        r.minKph = std::min(r.minKph, kph);
        const f64 lean = std::fabs(phys::leanAngleDegrees(world, v));
        r.leanPeak = std::max(r.leanPeak, lean);
        if (phys::wheelsOnGround(v) >= (wheelCount >= 4 ? 3 : 1)) ++grounded;

        size_t nearest = 0;
        const f64 lateral = chainDistance(chain, now.x, now.z, &nearest);
        r.lateralMax = std::max(r.lateralMax, lateral);
        r.slabStations = nearest;
        const f64 below = static_cast<f64>(now.y) - surfaceOf(chain, nearest);
        r.lowestBelowDeck = std::min(r.lowestBelowDeck, below);

        if (g_trace && (i % 20) == 0)
            std::printf("    %-18s t=%5.2f v=%5.1f km/h heading=%7.1f lean=%5.1f wheels=%d "
                        "lateral=%5.1f y=%7.2f vs deck %6.2f\n",
                        r.name.c_str(), time, kph, heading * 180.0 / kPi, lean,
                        phys::wheelsOnGround(v), lateral, static_cast<f64>(now.y),
                        surfaceOf(chain, nearest));

        if (below < -2.0) { r.fellOff = true; break; }
        if (in.brakeAtEnd && kph < 0.5 && time > 1.0) break;
        if (!in.brakeAtEnd && kph < 0.5 && time > 6.0) { r.stalled = true; break; }
    }

    r.metres = travelled;
    r.meanKph = travelled / std::max(0.05, time) * 3.6;
    r.wheelsDownFrac = samples > 0 ? static_cast<f64>(grounded) / static_cast<f64>(samples) : 0.0;
    r.leanEnd = std::fabs(phys::leanAngleDegrees(world, v));
    r.deepestPenetration = world.penetration().deepestStatic;
    return r;
}

// A vehicle placed on the chain, ready to be driven.
phys::Vehicle spawnOnChain(phys::World& world, sim::Archetype kind, const DeckChain& chain,
                           size_t slabIndex, f64 lateral) {
    const Slab& s = chain.slabs[std::min(slabIndex, chain.slabs.size() - 1)];
    const size_t aheadIndex = std::min(chain.slabs.size() - 1, slabIndex + 1);
    const f64 yaw = std::atan2(chain.slabs[aheadIndex].x - s.x, chain.slabs[aheadIndex].z - s.z);
    const f64 rx = std::cos(yaw), rz = -std::sin(yaw);
    return phys::spawnVehicle(world, kind,
                              Vec3(static_cast<f32>(s.x + rx * lateral),
                                   static_cast<f32>(surfaceOf(chain, slabIndex) + 2.0),
                                   static_cast<f32>(s.z + rz * lateral)),
                              yaw);
}

// -----------------------------------------------------------------------------
//  Drop test
// -----------------------------------------------------------------------------
struct DropResult {
    f64 restY = 0, expectedY = 0, error = 0;
    f64 gap = 0;                 // lowest corner to the surface it rests on
    f64 settleSeconds = 0;
    f64 deepest = 0;             // worst penetration anywhere in the drop
    f64 deepestAtRest = 0;       // worst penetration in the last half second
    f64 lowestCentre = 1e30;     // deepest the body's centre ever got
    f64 impactSpeed = 0;
    bool settled = false, finite = true, tunnelled = false;
};

DropResult dropTest(phys::World& world, const Vec3& above, f64 halfExtent, f64 massKg,
                    f64 dropHeight, f64 maxSeconds = 8.0) {
    DropResult r;
    const Vec3 start(above.x, static_cast<f32>(above.y + dropHeight), above.z);
    const phys::BodyHandle body = world.addDynamicBox(start, Vec3(static_cast<f32>(halfExtent),
                                                                 static_cast<f32>(halfExtent),
                                                                 static_cast<f32>(halfExtent)),
                                                      0.0, massKg);
    if (!body.valid()) return r;
    const f64 surface = world.groundHeightStatic(Vec3(above.x, static_cast<f32>(above.y + dropHeight),
                                                      above.z), 2.0 * dropHeight + 60.0);
    r.expectedY = std::isfinite(surface) ? surface + halfExtent : 0.0;
    world.penetration().reset();
    f64 prevY = start.y;
    f64 lastDeepest = 0.0;
    // Rest is a window, not a tick: a body at the apex of a bounce is also
    // momentarily "not moving" (this test reported a crate resting 0.31 m in the
    // air before the window was added).
    constexpr size_t kWindow = 15;      // 0.25 s
    f64 window[kWindow];
    for (f64& w : window) w = start.y;
    size_t windowFill = 0;
    const u64 steps = static_cast<u64>(maxSeconds / kStep);
    for (u64 i = 0; i < steps; ++i) {
        world.step(static_cast<f32>(kStep));
        const Vec3 p = world.position(body);
        if (!std::isfinite(p.y)) { r.finite = false; break; }
        const f64 v = std::fabs(static_cast<f64>(p.y) - prevY) / kStep;
        if (v > r.impactSpeed) r.impactSpeed = v;
        prevY = p.y;
        // A body whose *centre* has passed below the surface it fell on has gone
        // through it, whatever the contact depths say.
        r.lowestCentre = std::min(r.lowestCentre, static_cast<f64>(p.y));
        if (i > 30 && static_cast<f64>(p.y) < r.expectedY - halfExtent)
            r.tunnelled = true;
        r.deepest = std::max(r.deepest, world.penetration().deepestStatic);
        if (i * kStep > maxSeconds - 0.5)
            r.deepestAtRest = std::max(r.deepestAtRest, world.penetration().deepestStatic);
        if (g_trace && i * kStep > 1.9 && i * kStep < 3.0) {
            lastDeepest = world.penetration().deepestStatic;
            std::printf("      drop t=%.3f y=%.3f (surface+%.3f) speed=%.2f deepest=%.4f\n",
                        i * kStep, static_cast<f64>(p.y),
                        std::isfinite(surface) ? static_cast<f64>(p.y) - surface : 0.0, v, lastDeepest);
        }
        window[i % kWindow] = p.y;
        if (windowFill < kWindow) ++windowFill;
        f64 windowMin = 1e30, windowMax = -1e30;
        for (size_t k = 0; k < windowFill; ++k) {
            windowMin = std::min(windowMin, window[k]);
            windowMax = std::max(windowMax, window[k]);
        }
        if (v < 0.05 && windowFill == kWindow && (windowMax - windowMin) < 0.02 && (i * kStep) > 0.5) {
            r.settleSeconds = i * kStep;
            r.settled = true;
            r.restY = p.y;
            r.error = static_cast<f64>(p.y) - r.expectedY;
            // Where the *body* is, not where its centre is: a crate that tumbled
            // and came to rest on an edge has its centre 1.13 * half-extent above
            // the surface, and calling that a 0.33 m error was this check being
            // wrong, not the physics.  The support height of an oriented box is
            // h * sum(|row of the rotation| . up).
            const J::Quat q = world.bodies().GetRotation(body.id);
            const J::Mat44 rot = J::Mat44::sRotation(q);
            f64 support = 0.0;
            for (int c = 0; c < 3; ++c)
                support += halfExtent * std::fabs(static_cast<f64>(rot.GetColumn3(c).GetY()));
            r.gap = (static_cast<f64>(p.y) - support) - surface;
            break;
        }

    }
    world.remove(body);
    return r;
}

// -----------------------------------------------------------------------------
//  The report
// -----------------------------------------------------------------------------
std::string g_joltVersion;
Json g_report = Json::object();

void reportSet(const std::string& key, const Json& v) { g_report.set(key, v); }

Json rideJson(const RideResult& r) {
    Json j = Json::object();
    j.set("metres", Json(r.metres));
    j.set("seconds", Json(r.seconds));
    j.set("meanKph", Json(r.meanKph));
    j.set("maxKph", Json(r.maxKph));
    j.set("minKph", Json(r.minKph < 1e29 ? r.minKph : 0.0));
    j.set("leanPeakDeg", Json(r.leanPeak));
    j.set("leanEndDeg", Json(r.leanEnd));
    j.set("worstLateralErrorM", Json(r.lateralMax));
    j.set("wheelsDownFraction", Json(r.wheelsDownFrac));
    j.set("lowestBelowDeckM", Json(r.lowestBelowDeck));
    j.set("deepestPenetrationM", Json(r.deepestPenetration));
    j.set("fellOff", Json(r.fellOff));
    j.set("stalled", Json(r.stalled));
    j.set("finite", Json(r.finite));
    return j;
}

}  // namespace

// =============================================================================
namespace {

// A trap (Jolt's own `JPH_BREAKPOINT`) used to kill this harness with no message
// and no location, which is the slowest possible way to debug an engine
// integration.  This prints the stack and the instruction pointer, then exits
// with a distinct code.
void trapHandler(int sig) {
    void* frames[32];
    const int n = static_cast<int>(backtrace(frames, 32));
    const char msg[] = "\n  trap: the process hit a breakpoint trap; stack:\n";
    const ssize_t ignored = write(STDERR_FILENO, msg, sizeof(msg) - 1);
    (void)ignored;
    backtrace_symbols_fd(frames, n, STDERR_FILENO);
    _exit(100 + sig);
}

void installTrapHandler() {
    struct sigaction sa{};
    sa.sa_handler = trapHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_NODEFER;
    sigaction(SIGTRAP, &sa, nullptr);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGSEGV, &sa, nullptr);
}

}  // namespace

int main(int argc, char** argv) {
    installTrapHandler();
    bool help = false;
    const Options opt = parse(argc, argv, help);
    if (help) {
        std::printf("mumbai_phys — the A2 physics acceptance harness\n"
                    "  --chunks <dir>     .mumbai_chunk cells (default data/processed/cells)\n"
                    "  --minutes <n>      soak length, simulated (default 1)\n"
                    "  --seconds <n>      length of each vehicle test (default 30)\n"
                    "  --deck-kph <n>     motorcycle cruise (default 45)\n"
                    "  --fleet <n>        vehicles in the fleet-cost section (default 24)\n"
                    "  --props <n>        dynamic crates in the soak (default 200)\n"
                    "  --max-chunks <n>   bake only the first n cells (smoke runs)\n"
                    "  --no-roads         skip road-mesh collision (heightfield only)\n"
                    "  --report <path>    JSON summary (default build/physics_report.json)\n"
                    "  --quick --quiet --trace --survey M\n");
        return 0;
    }

    if (!g_quiet) {
        std::printf("Mumbai 1:1 EXACT — physics acceptance harness (mumbai_phys)\n");
        std::printf("  chunks   %s\n", opt.chunks.c_str());
        std::printf("  cells    %s\n", opt.maxChunks ? "first N of the world" : "all");
        std::printf("  rides    %.0f s each, fleet %u, soak %.2f simulated minutes\n",
                    opt.seconds, opt.fleet, opt.minutes);
        std::printf("  answer   pass when every check below passes; exit 2 otherwise\n");
        std::printf("  timing   step, fleet and origin-shift budgets are CPU-sensitive: run this\n"
                    "           unshared (a concurrent build inflates them; nothing else moves)\n");
    }

    // --- the world on disk ----------------------------------------------------
    const std::string chunkDir = resolveChunkDir(opt.chunks);
    auto cells = listCells(opt.chunks);
    if (opt.maxChunks > 0 && cells.size() > opt.maxChunks) cells.resize(opt.maxChunks);
    if (cells.empty()) {
        std::printf("no .mumbai_chunk files under '%s' — build the world first "
                    "(make world)\n", opt.chunks.c_str());
        return 3;
    }

    Vec2d extentMin(0, 0);
    bool haveExtent = false;
    std::string anchorsPath = opt.anchors;
    if (anchorsPath.empty()) anchorsPath = "viewer/data/mumbai.json";
    const std::vector<Landmark> landmarks = loadLandmarks(anchorsPath, extentMin, haveExtent);
    if (haveExtent)
        reportSet("extentMin", Json(std::string(sfmt("%.0f, %.0f m", extentMin.x, extentMin.y))));

    phys::World world;
    if (!world.init(8192, 64u << 20, 1)) {
        std::printf("Jolt failed to initialise\n");
        return 2;
    }
    g_joltVersion = world.versionString();

    // --- 1. bake --------------------------------------------------------------
    section("engine and bake");
    require(true, "Jolt initialises", g_joltVersion);
    reportSet("jolt", Json(g_joltVersion));

    phys::BakeOptions bakeOptions;
    bakeOptions.boxes = true;
    bakeOptions.terrain = true;
    bakeOptions.roads = opt.roads;
    phys::BakeStats stats;
    const auto bakeStart = std::chrono::steady_clock::now();
    // Baked cell by cell from the same list the coverage grid walks, so that a
    // `--max-chunks` smoke run is a *partial world* in every section rather than
    // a whole world in some of them.
    bool baked = true;
    for (const CellRef& c : cells)
        baked = phys::bakeChunk(world, c.path, bakeOptions, stats) && baked;
    const f64 bakeSeconds = std::chrono::duration<f64>(std::chrono::steady_clock::now() - bakeStart).count();
    require(baked && stats.firstError.empty(), "every cell bakes",
            stats.firstError.empty() ? sfmt("%zu cells in %.1f s", cells.size(), bakeSeconds)
                                     : stats.firstError);
    require(stats.boxes > 100000, "the city has colliders",
            sfmt("%llu boxes, %llu capsules, %llu terrain, %llu road meshes in %llu compounds "
                 "(%llu sliver triangles rejected)",
                 static_cast<unsigned long long>(stats.boxes),
                 static_cast<unsigned long long>(stats.capsules),
                 static_cast<unsigned long long>(stats.terrainShapes),
                 static_cast<unsigned long long>(stats.roadShapes),
                 static_cast<unsigned long long>(stats.compoundGroups),
                 static_cast<unsigned long long>(stats.roadSlivers)));
    {
        Json j = Json::object();
        j.set("cells", Json(static_cast<f64>(cells.size())));
        j.set("boxes", Json(static_cast<f64>(stats.boxes)));
        j.set("capsules", Json(static_cast<f64>(stats.capsules)));
        j.set("terrainShapes", Json(static_cast<f64>(stats.terrainShapes)));
        j.set("roadShapes", Json(static_cast<f64>(stats.roadShapes)));
        j.set("roadTriangles", Json(static_cast<f64>(stats.roadTriangles)));
        j.set("roadSlivers", Json(static_cast<f64>(stats.roadSlivers)));
        j.set("compoundGroups", Json(static_cast<f64>(stats.compoundGroups)));
        j.set("staticBodies", Json(static_cast<f64>(world.bodyCount())));
        j.set("seconds", Json(bakeSeconds));
        reportSet("bake", j);
    }

    // --- 2. coverage ----------------------------------------------------------
    // Every cell must have ground under it: the world is 924 cells of a 1:1 city
    // and a hole in the collision is a vehicle falling through the map.
    section("collision coverage");
    {
        u64 probes = 0, hits = 0;
        f64 lowest = 1e30, highest = -1e30;
        for (const CellRef& c : cells) {
            const f64 ox = static_cast<f64>(c.cellX) * c.cellSize;
            const f64 oz = static_cast<f64>(c.cellZ) * c.cellSize;
            const f64 offs[2] = {0.3, 0.7};
            for (f64 u : offs) {
                for (f64 v : offs) {
                    const f64 x = ox + u * c.cellSize, z = oz + v * c.cellSize;
                    const f64 h = world.groundHeightStatic(
                        Vec3(static_cast<f32>(x), 140.0f, static_cast<f32>(z)), 260.0);
                    ++probes;
                    if (std::isfinite(h)) {
                        ++hits;
                        lowest = std::min(lowest, h);
                        highest = std::max(highest, h);
                    }
                }
            }
        }
        require(hits == probes, "every probe across the built extent finds ground",
                sfmt("%llu / %llu hits, surface %.1f..%.1f m",
                     static_cast<unsigned long long>(hits), static_cast<unsigned long long>(probes),
                     lowest, highest));
        Json j = Json::object();
        j.set("probes", Json(static_cast<f64>(probes)));
        j.set("hits", Json(static_cast<f64>(hits)));
        j.set("minSurfaceY", Json(lowest));
        j.set("maxSurfaceY", Json(highest));
        reportSet("coverage", j);
    }

    // --- 3. the Sea Link deck -------------------------------------------------
    section("Sea Link deck");
    Vec2d deckAnchor(-8632.1, 18202.1);   // published lat/lon through the projection
    bool anchorFromPayload = false;
    for (const Landmark& l : landmarks) {
        if (l.name.find("Sea Link") != std::string::npos) {
            deckAnchor = Vec2d(l.x, l.z);
            anchorFromPayload = true;
        }
    }
    if (!anchorFromPayload) {
        const TransverseMercator proj{};
        deckAnchor = proj.forward(19.0350, 72.8180);
    }
    if (!g_quiet)
        std::printf("  anchor   %.1f, %.1f m (%s)\n", deckAnchor.x, deckAnchor.y,
                    anchorFromPayload ? "from the published viewer manifest"
                                      : "published lat/lon through the projection");
    const DeckChain deck = surveyDeck(world, listCells(opt.chunks), deckAnchor);
    require(deck.ok && deck.slabs.size() >= 40, "the deck is recovered from the collision records",
            deck.ok ? deck.note : deck.note);
    if (!deck.ok) {
        std::printf("  without a deck there is nothing to drive: stopping here\n");
        world.shutdown();
        return 2;
    }
    require(deck.length > 800.0, "the carriageway runs for kilometres",
            sfmt("%.0f m of continuous slabs, y %.2f..%.2f m, width %.1f m",
                 deck.length, deck.minY, deck.maxY, deck.width));
    // The record and the collider must be the same object: every slab's top face
    // is probed, and the chain's offset from `cy` to that face has to be one
    // number, not 400.
    require(deck.surfaceSpread <= 0.05 &&
                (deck.surfaceMissing + deck.surfaceObstructed) * 20 <= deck.slabs.size(),
            "every slab's collider matches its collision record",
            sfmt("%zu slabs probed, top face %.3f m above the record's centre, worst deviation "
                 "%.3f m, %llu obstructed by the bridge's own pylon/stays, %llu unprobeable",
                 deck.slabs.size(), deck.surfaceOffset, deck.surfaceSpread,
                 static_cast<unsigned long long>(deck.surfaceObstructed),
                 static_cast<unsigned long long>(deck.surfaceMissing)));
    {
        const f64 yawDeg = (deck.yawStart + deck.yawEnd) * 0.5 * 180.0 / kPi;
        require(deck.width > 8.0 && deck.width < 26.0, "a carriageway is a carriageway's width",
                sfmt("%.1f m wide, heading %.1f deg", deck.width, yawDeg));
        Json j = Json::object();
        j.set("slabs", Json(static_cast<f64>(deck.slabs.size())));
        j.set("lengthM", Json(deck.length));
        j.set("widthM", Json(deck.width));
        j.set("minY", Json(deck.minY));
        j.set("maxY", Json(deck.maxY));
        j.set("yawStartDeg", Json(deck.yawStart * 180.0 / kPi));
        j.set("yawEndDeg", Json(deck.yawEnd * 180.0 / kPi));
        j.set("surfaceOffsetM", Json(deck.surfaceOffset));
        j.set("surfaceSpreadM", Json(deck.surfaceSpread));
        j.set("surfaceMissing", Json(static_cast<f64>(deck.surfaceMissing)));
        j.set("surfaceObstructed", Json(static_cast<f64>(deck.surfaceObstructed)));
        (void)0;
        reportSet("deck", j);
    }

    // --- 4. rides -------------------------------------------------------------
    section("deck rides");
    Json rides = Json::object();
    bool diagnostic = false;
    {
        // 4a. The marquee shot: a motorcycle along the carriageway.
        phys::Vehicle bike = spawnOnChain(world, sim::Archetype::Motorcycle, deck,
                                          deck.slabs.size() / 4, 0.0);
        if (!bike.valid()) {
            require(false, "a motorcycle spawns on the deck", "spawnVehicle returned nothing");
        } else {
            RideInput in;
            in.targetKph = opt.deckKph;
            in.seconds = opt.seconds;
            in.initialSpeed = 7.0;
            in.handsOff = opt.noSteer;
            in.holdSteer = opt.holdSteer;
            const RideResult r = rideChain(world, bike, deck, in);
            if (opt.noSteer || opt.holdSteer == opt.holdSteer) {
                std::printf("  (diagnostic ride: holdSteer %.2f, handsOff %d)\n",
                            opt.holdSteer, opt.noSteer ? 1 : 0);
                Json j = rideJson(r);
                j.set("handsOff", Json(true));
                reportSet("motorcycle", j);
                phys::destroyVehicle(world, bike);
                diagnostic = true;
            } else {
            require(!r.fellOff && r.finite, "a motorcycle rides the Sea Link without leaving it",
                    sfmt("%.0f m in %.0f s at %.1f km/h mean (max %.0f), wheels down %.0f%%, "
                         "peak lean %.1f deg, worst %.1f m off the chain, lowest %.2f m below it",
                         r.metres, r.seconds, r.meanKph, r.maxKph, r.wheelsDownFrac * 100.0,
                         r.leanPeak, r.lateralMax, r.lowestBelowDeck));
            require(r.wheelsDownFrac > 0.6, "both wheels stay on the carriageway",
                    sfmt("wheels down %.0f%% of the ride", r.wheelsDownFrac * 100.0));
            require(r.metres > 0.6 * in.seconds * in.targetKph / 3.6, "the motorcycle covers ground",
                    sfmt("%.0f m at a %.0f m/s target in %.0f s", r.metres, in.targetKph / 3.6,
                         r.seconds));
            require(r.leanPeak < 46.0, "the motorcycle stays inside its lean limit",
                    sfmt("peak %.1f deg (limit 45, controller clamp)", r.leanPeak));
            require(r.deepestPenetration <= kPenetrationBudget, "the motorcycle does not penetrate the deck",
                    sfmt("deepest %.4f m (budget %.3f)", r.deepestPenetration, kPenetrationBudget));
            reportSet("motorcycle", rideJson(r));
            }
        }

        // 4b. Slalom: the lean controller has to lean, and recover.
        if (bike.valid() && !diagnostic) {
            RideInput in;
            in.targetKph = std::min(opt.deckKph, 40.0);
            in.seconds = std::min(opt.seconds * 1.5, 20.0);
            in.initialSpeed = 7.0;
            in.slalomAmp = 0.22;
            in.slalomPeriod = 3.0;
            const RideResult r = rideChain(world, bike, deck, in);
            require(!r.fellOff && r.finite, "the motorcycle slaloms without falling",
                    sfmt("peak lean %.1f deg, ended at %.1f deg, %.0f m covered, wheels down %.0f%%",
                         r.leanPeak, r.leanEnd, r.metres, r.wheelsDownFrac * 100.0));
            require(r.leanPeak > 8.0 && r.leanPeak < 46.0, "the lean controller leans",
                    sfmt("peak %.1f deg, recovery to %.1f deg", r.leanPeak, r.leanEnd));
            reportSet("slalom", rideJson(r));
            phys::destroyVehicle(world, bike);
        }

        // 4c. The four-wheeled archetypes, on the same chain.
        struct CarTest { sim::Archetype kind; f64 kph; const char* label; f64 maxLean; };
        const CarTest cars[] = {
            {sim::Archetype::Car, 60.0, "car", 12.0},
            {sim::Archetype::Taxi, 55.0, "kaali-peeli taxi", 12.0},
            {sim::Archetype::Bus, 40.0, "BEST double-decker", 14.0},
            {sim::Archetype::AutoRickshaw, 35.0, "auto-rickshaw", 25.0},
        };
        for (const CarTest& c : cars) {
            phys::Vehicle v = spawnOnChain(world, c.kind, deck, deck.slabs.size() / 4, 0.0);
            if (!v.valid()) {
                require(false, sfmt("%s spawns", c.label), "spawnVehicle returned nothing");
                continue;
            }
            RideInput in;
            in.targetKph = c.kph;
            in.seconds = std::min(opt.seconds * 0.7, 20.0);
            in.initialSpeed = 5.0;
            in.handsOff = opt.noSteer;
            in.holdSteer = opt.holdSteer;
            const RideResult r = rideChain(world, v, deck, in);
            require(!r.fellOff && r.finite, sfmt("%s drives the deck", c.label),
                    sfmt("%.0f m at %.1f km/h mean, peak body roll %.1f deg (limit %.0f), "
                         "wheels down %.0f%%", r.metres, r.meanKph, r.leanPeak, c.maxLean,
                         r.wheelsDownFrac * 100.0));
            require(r.leanPeak < c.maxLean, sfmt("%s does not roll over", c.label),
                    sfmt("peak roll %.1f deg (limit %.0f)", r.leanPeak, c.maxLean));
            require(r.deepestPenetration <= kPenetrationBudget,
                    sfmt("%s does not penetrate the deck", c.label),
                    sfmt("deepest %.4f m", r.deepestPenetration));
            rides.set(c.label, rideJson(r));
            phys::destroyVehicle(world, v);
        }
        reportSet("rides", rides);
    }

    // --- 5. braking -----------------------------------------------------------
    section("braking");
    {
        phys::Vehicle car = spawnOnChain(world, sim::Archetype::Car, deck, deck.slabs.size() / 4, 0.0);
        if (!car.valid()) {
            require(false, "a car spawns for the braking test", "");
        } else {
            const Slab& s = deck.slabs[deck.slabs.size() / 4];
            const size_t next = std::min(deck.slabs.size() - 1, deck.slabs.size() / 4 + 1);
            const f64 yaw = std::atan2(deck.slabs[next].x - s.x, deck.slabs[next].z - s.z);
            const f64 rest = surfaceOf(deck, deck.slabs.size() / 4) + car.halfHeight +
                             0.65 * car.wheelRadius + kSpawnClearance;
            world.setPosition(car.body, Vec3(static_cast<f32>(s.x), static_cast<f32>(rest),
                                             static_cast<f32>(s.z)));
            world.bodies().SetRotation(car.body.id,
                                       J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)),
                                       J::EActivation::Activate);
            world.bodies().SetLinearVelocity(car.body.id,
                                             J::Vec3(std::sin(yaw) * 60.0 / 3.6, 0.0f,
                                                     std::cos(yaw) * 60.0 / 3.6));
            RideInput in;
            in.brakeAtEnd = true;
            in.seconds = 12.0;
            in.initialSpeed = 60.0 / 3.6;
            const RideResult r = rideChain(world, car, deck, in);
            // v^2 / (2 * mu * g) with mu = 0.9: 15.7 m.  A car that stops in
            // less has tyres from a cartoon; one that takes much more has brakes
            // that never engage.  The band is wide because the deck has a grade.
            require(r.metres > 9.0 && r.metres < 45.0, "a car brakes in a plausible distance",
                    sfmt("%.1f m from 60 km/h (analytic 15.7 m at mu = 0.9), peak roll %.1f deg",
                         r.metres, r.leanPeak));
            Json j = rideJson(r);
            j.set("analyticM", Json(15.7));
            reportSet("braking", j);
            phys::destroyVehicle(world, car);
        }
    }

    // --- 6. impact ------------------------------------------------------------
    section("impact");
    {
        const Slab& mid = deck.slabs[deck.slabs.size() / 2];
        const DropResult deckDrop = dropTest(world, Vec3(static_cast<f32>(mid.x),
                                                         static_cast<f32>(mid.y),
                                                         static_cast<f32>(mid.z)),
                                             0.8, 300.0, 22.0);
        require(deckDrop.settled && deckDrop.finite, "a 300 kg body dropped 22 m onto the deck settles",
                sfmt("settled in %.2f s, impact %.1f m/s", deckDrop.settleSeconds, deckDrop.impactSpeed));
        require(std::fabs(deckDrop.gap) < 0.05, "it comes to rest on the deck, not in it",
                sfmt("the body's lowest corner is %+.3f m off the surface (centre %.3f, "
                     "error %+.3f m)", deckDrop.gap, deckDrop.restY, deckDrop.error));
        require(!deckDrop.tunnelled, "the impact never puts the body inside the deck",
                sfmt("lowest centre y %.3f vs the %0.3f the surface allows",
                     deckDrop.lowestCentre, deckDrop.expectedY - 0.8));
        require(deckDrop.deepestAtRest <= kPenetrationBudget,
                "the deck is clear once the impact is over",
                sfmt("deepest at rest %.4f m (budget %.3f)", deckDrop.deepestAtRest,
                     kPenetrationBudget));
        require(deckDrop.deepest <= kImpactTransientBudget,
                "the impact transient stays inside Jolt's own CCD allowance",
                sfmt("deepest %.4f m at %.1f m/s (allowance %.2f m = linear-cast max penetration)",
                     deckDrop.deepest, deckDrop.impactSpeed, kImpactTransientBudget));
        {
            Json j = Json::object();
            j.set("settleSeconds", Json(deckDrop.settleSeconds));
            j.set("restY", Json(deckDrop.restY));
            j.set("expectedY", Json(deckDrop.expectedY));
            j.set("restErrorM", Json(deckDrop.error));
            j.set("restGapM", Json(deckDrop.gap));
            j.set("deepestPenetrationM", Json(deckDrop.deepest));
            j.set("deepestAtRestM", Json(deckDrop.deepestAtRest));
            j.set("lowestCentreY", Json(deckDrop.lowestCentre));
            j.set("impactSpeedMs", Json(deckDrop.impactSpeed));
            j.set("tunnelled", Json(deckDrop.tunnelled));
            reportSet("dropTest", j);
        }

        // The street grid: a body dropped over a published landmark must meet the
        // city, not the sea.  Each landmark in the payload is a real address, and
        // the ground under it is the discipline the whole pipeline exists for.
        const Landmark* probe = nullptr;
        for (const Landmark& l : landmarks)
            if (l.name.find("Terminus") != std::string::npos) probe = &l;
        if (probe == nullptr && !landmarks.empty()) probe = &landmarks.front();
        if (probe != nullptr) {
            const f64 ground = world.groundHeightStatic(
                Vec3(static_cast<f32>(probe->x), 120.0f, static_cast<f32>(probe->z)), 200.0);
            require(std::isfinite(ground) && ground > 0.5,
                    sfmt("the street under %s is solid", probe->name.c_str()),
                    std::isfinite(ground) ? sfmt("ground at %.1f, %.1f is y %.2f m",
                                                 probe->x, probe->z, ground)
                                          : std::string("no ground under the landmark"));
            if (std::isfinite(ground)) {
                const DropResult street = dropTest(world,
                    Vec3(static_cast<f32>(probe->x), static_cast<f32>(ground), static_cast<f32>(probe->z)),
                    0.6, 120.0, 16.0);
                require(street.settled && std::fabs(street.gap) < 0.10 && !street.tunnelled,
                        "a body dropped over the landmark rests on the city",
                        sfmt("gap %+.3f m to the surface at %.2f m, centre %.3f (error %+.3f m), "
                             "deepest %.4f m at impact / %.4f m at rest", street.gap, ground,
                             street.restY, street.error, street.deepest, street.deepestAtRest));
                Json j = Json::object();
                j.set("landmark", Json(probe->name));
                j.set("groundY", Json(ground));
                j.set("restY", Json(street.restY));
                j.set("errorM", Json(street.error));
                j.set("restGapM", Json(street.gap));
                j.set("deepestPenetrationM", Json(street.deepest));
                j.set("deepestAtRestM", Json(street.deepestAtRest));
                j.set("lowestCentreY", Json(street.lowestCentre));
                j.set("tunnelled", Json(street.tunnelled));
                reportSet("urbanDrop", j);
            }
        }
    }

    // --- 7. fleet cost --------------------------------------------------------
    section("fleet");
    {
        // Cost per vehicle per step, measured, then extrapolated to the 500
        // vehicles Section 7 asks for within 500 m.  A constraint per vehicle is
        // the most expensive thing this simulation does, so this is the number
        // that decides whether a 500-vehicle frame fits in 33 ms.
        const u32 fleet = opt.fleet;
        std::vector<phys::Vehicle> vehicles;
        vehicles.reserve(fleet);
        const size_t stride = std::max<size_t>(1, deck.slabs.size() / std::max<u32>(1, fleet + 1));
        for (u32 i = 0; i < fleet; ++i) {
            const size_t idx = std::min(deck.slabs.size() - 1, deck.slabs.size() / 4 + i * stride);
            phys::Vehicle v = spawnOnChain(world, (i % 3 == 0) ? sim::Archetype::Taxi
                                                              : sim::Archetype::Car,
                                            deck, idx, (i % 2 == 0) ? -4.5 : 4.5);
            if (!v.valid()) break;
            const Slab& s = deck.slabs[idx];
            const size_t next = std::min(deck.slabs.size() - 1, idx + 1);
            const f64 yaw = std::atan2(deck.slabs[next].x - s.x, deck.slabs[next].z - s.z);
            const f64 rest = surfaceOf(deck, idx) + v.halfHeight + 0.65 * v.wheelRadius +
                             kSpawnClearance;
            world.setPosition(v.body, Vec3(static_cast<f32>(s.x), static_cast<f32>(rest),
                                           static_cast<f32>(s.z)));
            world.bodies().SetRotation(v.body.id,
                                       J::Quat::sRotation(J::Vec3::sAxisY(), static_cast<f32>(yaw)),
                                       J::EActivation::Activate);
            world.bodies().SetLinearVelocity(v.body.id, J::Vec3(std::sin(yaw) * 8.0f, 0.0f,
                                                                std::cos(yaw) * 8.0f));
            vehicles.push_back(v);
        }
        // Settle, then measure 240 steps of steady-state cost.
        for (int i = 0; i < 120; ++i) {
            for (phys::Vehicle& v : vehicles) phys::driveVehicle(v, 0.35f, 0.0f, 0.0f);
            world.step(static_cast<f32>(kStep));
        }
        constexpr int kSamples = 240;
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < kSamples; ++i) {
            for (phys::Vehicle& v : vehicles) phys::driveVehicle(v, 0.35f, 0.0f, 0.0f);
            world.step(static_cast<f32>(kStep));
        }
        const f64 totalMs = std::chrono::duration<f64>(std::chrono::steady_clock::now() - t0).count() * 1000.0;
        const f64 msPerStep = totalMs / kSamples;
        const f64 usPerVehicle = vehicles.empty() ? 0.0 : msPerStep * 1000.0 / vehicles.size();
        const f64 fleet500Ms = usPerVehicle * 500.0 / 1000.0;
        bool allValid = !vehicles.empty();
        for (const phys::Vehicle& v : vehicles) allValid = allValid && v.valid();
        require(allValid, "a fleet of vehicles spawns and stays up",
                sfmt("%zu vehicles, %.0f kg to %.0f kg", vehicles.size(),
                     vehicles.empty() ? 0.0 : vehicles.front().massKg,
                     vehicles.empty() ? 0.0 : vehicles.back().massKg));
        require(fleet500Ms < 16.0, "500 physical vehicles fit in a 30 Hz frame",
                sfmt("%.3f ms per step for %zu vehicles = %.0f us per vehicle = %.2f ms extrapolated "
                     "to 500 (30 Hz budget 33 ms)", msPerStep, vehicles.size(), usPerVehicle, fleet500Ms));
        {
            Json j = Json::object();
            j.set("vehicles", Json(static_cast<f64>(vehicles.size())));
            j.set("msPerStep", Json(msPerStep));
            j.set("usPerVehicle", Json(usPerVehicle));
            j.set("extrapolated500Ms", Json(fleet500Ms));
            reportSet("fleet", j);
        }
        for (phys::Vehicle& v : vehicles) phys::destroyVehicle(world, v);
    }

    // --- Section 5: the traffic model, driven by physics ----------------------
    // Until this section existed, the model and the bodies were two fleets that
    // never met: `engine/sim` decided where every vehicle goes and moved points
    // along a graph, `engine/physics` built real bodies and drove four of them by
    // hand along the Sea Link.  Here the model keeps deciding and the bodies do
    // the moving, with the body's actual position fed back into the model every
    // tick (D-034).
    section("physical traffic (S5)");
    {
        Vec2d anchor = deckAnchor;
        for (const Landmark& l : landmarks)
            if (l.name.find("Marine Drive") != std::string::npos) anchor = Vec2d(l.x, l.z);
        // The model's own population: enough vehicles around the anchor that a
        // driven one has someone to follow.  The population device keeps this
        // number near the anchor as the fleet drives, which is what makes the
        // handoff testable at a fixed point in the city rather than on the deck.
        const u32 population = std::max<u32>(opt.drive * 6, 120);
        std::vector<std::string> files;
        files.reserve(cells.size());
        for (const CellRef& c : cells) files.push_back(c.path);
        auto tBuild0 = std::chrono::steady_clock::now();
        sim::Traffic traffic(files, std::vector<Vec2d>{anchor}, anchor, population, 0x50485953ull);
        const f64 buildMs =
            std::chrono::duration<f64, std::milli>(std::chrono::steady_clock::now() - tBuild0).count();
        // Ten seconds of model-only running, so the disc fills and the platoons
        // form; a handoff test on a fleet that has not started moving yet would
        // measure the population device, not the driver.
        for (int i = 0; i < 600; ++i) traffic.step(kStep);
        const sim::TrafficMetrics before = traffic.metrics(anchor, 500.0);
        require(before.vehicles > opt.drive && before.meanSpeedKph > 5.0,
                "the traffic model runs before anything is handed over",
                sfmt("%zu vehicles around the anchor, mean %.1f km/h, %zu edges, %zu signals, "
                     "built in %.1f s", before.vehicles, before.meanSpeedKph, before.edges,
                     before.signals, buildMs / 1000.0));
        // The graph's own connectivity, measured rather than assumed.  A node
        // whose only out-arm is its own reversal is a place the model can only
        // turn a vehicle around — a cul-de-sac, or a hole where the producer
        // dropped a segment.  The two are told apart by whether another node
        // with arms stands within two metres of it (D-034).
        {
            const sim::Traffic::GraphAudit ga = traffic.graphAudit();
            const f64 n = static_cast<f64>(std::max<u64>(ga.nodes, 1u));
            std::printf("      graph: %llu nodes — out-degree 1: %llu (%.2f %%), 2: %llu, "
                        "3: %llu, 4+: %llu\n"
                        "        of the one-arm nodes %llu (%.1f %%) are their own reversal and "
                        "%llu (%.1f %%) have another node with arms within 2 m\n",
                        static_cast<unsigned long long>(ga.nodes),
                        static_cast<unsigned long long>(ga.out1), 100.0 * ga.out1 / n,
                        static_cast<unsigned long long>(ga.out2),
                        static_cast<unsigned long long>(ga.out3),
                        static_cast<unsigned long long>(ga.out4plus),
                        static_cast<unsigned long long>(ga.oneArmIsReversal),
                        ga.out1 ? 100.0 * static_cast<f64>(ga.oneArmIsReversal) /
                                      static_cast<f64>(ga.out1) : 0.0,
                        static_cast<unsigned long long>(ga.oneArmHasTwinNearby),
                        ga.out1 ? 100.0 * static_cast<f64>(ga.oneArmHasTwinNearby) /
                                      static_cast<f64>(ga.out1) : 0.0);
        }

        // --- Section 5a: the model's roads against the collision layer ------------
        // The one measurement of the whole city rather than of a test track: every
        // `--survey` metres along every lane, ask the collision layer where the road
        // is.  Where the two layers disagree, some system in this project is wrong —
        // the survey says how often, how badly, and where; it does not decide which
        // layer is right.
        if (opt.surveySpacing > 0.0 && !traffic.edges().empty()) {
            section("road/collision survey");
            phys::DriverSettings surveySettings;
            const auto surveyT0 = std::chrono::steady_clock::now();
            phys::PhysicalFleet surveyor;
            surveySettings.surveyTrace = opt.surveyTrace;
            const phys::PhysicalFleet::LaneSurvey lv =
                surveyor.surveyLanes(world, traffic, opt.surveySpacing, surveySettings);
            const f64 secs = std::chrono::duration<f64>(std::chrono::steady_clock::now() -
                                                        surveyT0).count();
            const f64 n = static_cast<f64>(lv.samples);
            std::printf("      %llu lane samples every %.0f m (%.0f s), collision surface vs model road:\n"
                        "        within 0.25 m %5.1f %%   within 0.5 m %5.1f %%   within 2 m %5.1f %%"
                        "   worst %.2f m\n"
                        "        blocked for a car %llu (%.2f %%)   stepped > %.2f m %llu (%.2f %%)\n",
                        static_cast<unsigned long long>(lv.samples), opt.surveySpacing, secs,
                        n > 0 ? 100.0 * lv.withinQuarter / n : 0.0,
                        n > 0 ? 100.0 * lv.withinHalf / n : 0.0,
                        n > 0 ? 100.0 * lv.withinTwo / n : 0.0, lv.worstErrorM,
                        static_cast<unsigned long long>(lv.blocked),
                        n > 0 ? 100.0 * static_cast<f64>(lv.blocked) / n : 0.0,
                        surveySettings.footprintStepTolerance, static_cast<unsigned long long>(lv.stepped),
                        n > 0 ? 100.0 * static_cast<f64>(lv.stepped) / n : 0.0);
            if (lv.worstErrorM > 0.5)
                std::printf("        worst height disagreement at (%.1f, %.1f) on edge %llu\n",
                            lv.worstAt.x, lv.worstAt.y, static_cast<unsigned long long>(lv.worstEdge));
            std::printf("        elevated roads (the model's road more than 3 m above the surface "
                        "below it): %llu samples, of which %llu (%.1f %%) have a collision deck at "
                        "the model's own height; deepest drop below a model road %.2f m at "
                        "(%.1f, %.1f)\n",
                        static_cast<unsigned long long>(lv.elevatedSamples),
                        static_cast<unsigned long long>(lv.elevatedWithDeck),
                        lv.elevatedSamples ? 100.0 * static_cast<f64>(lv.elevatedWithDeck) /
                                                 static_cast<f64>(lv.elevatedSamples) : 0.0,
                        lv.worstElevatedDropM, lv.worstElevatedAt.x, lv.worstElevatedAt.y);
            std::printf("        collision above the model's road (a surface the nav graph does not "
                        "know it is under): %llu samples, worst %.2f m at (%.1f, %.1f)\n",
                        static_cast<unsigned long long>(lv.collisionAbove),
                        lv.worstCollisionAboveM, lv.worstCollisionAboveAt.x,
                        lv.worstCollisionAboveAt.y);
            std::printf("        bridge-flagged edges: %llu samples, within 0.25 m "
                        "%5.1f %%, within 2 m %5.1f %%, %llu with no deck in the collision "
                        "(worst deck disagreement %.2f m at (%.1f, %.1f))\n"
                        "        headroom above the lane: %llu samples, minimum %.2f m at "
                        "(%.1f, %.1f) — %llu (%.2f %%) below a car's 2.6 m, %llu below 2.0 m\n",
                        static_cast<unsigned long long>(lv.bridgeSamples),
                        lv.bridgeSamples ? 100.0 * static_cast<f64>(lv.bridgeWithinQuarter) /
                                               static_cast<f64>(lv.bridgeSamples) : 0.0,
                        lv.bridgeSamples ? 100.0 * static_cast<f64>(lv.bridgeWithinTwo) /
                                               static_cast<f64>(lv.bridgeSamples) : 0.0,
                        static_cast<unsigned long long>(lv.bridgeNoDeck), lv.worstBridgeErrorM,
                        lv.worstBridgeAt.x, lv.worstBridgeAt.y,
                        static_cast<unsigned long long>(lv.headroomSamples),
                        lv.headroomSamples ? lv.minHeadroomM : 0.0, lv.minHeadroomAt.x,
                        lv.minHeadroomAt.y, static_cast<unsigned long long>(lv.headroomUnderCar),
                        n > 0 ? 100.0 * static_cast<f64>(lv.headroomUnderCar) / n : 0.0,
                        static_cast<unsigned long long>(lv.headroomUnderAny));
            std::printf("        no surface in the band under the model's road: %llu samples — "
                        "%llu (%.1f %%) have a structure above them (lowest clearance %.2f m), "
                        "%llu (%.1f %%) have nothing there at all\n",
                        static_cast<unsigned long long>(lv.noSurface),
                        static_cast<unsigned long long>(lv.underStructure),
                        lv.noSurface ? 100.0 * static_cast<f64>(lv.underStructure) /
                                           static_cast<f64>(lv.noSurface) : 0.0,
                        lv.underStructure ? lv.minUnderStructureM : 0.0,
                        static_cast<unsigned long long>(lv.noCollisionAtAll),
                        lv.noSurface ? 100.0 * static_cast<f64>(lv.noCollisionAtAll) /
                                           static_cast<f64>(lv.noSurface) : 0.0);
            std::printf("        what is above those lanes: <0.5 m %llu, 0.5-1.5 m %llu, "
                        "1.5-3 m %llu, 3-8 m %llu, >8 m %llu\n",
                        static_cast<unsigned long long>(lv.overBins[0]),
                        static_cast<unsigned long long>(lv.overBins[1]),
                        static_cast<unsigned long long>(lv.overBins[2]),
                        static_cast<unsigned long long>(lv.overBins[3]),
                        static_cast<unsigned long long>(lv.overBins[4]));
            std::printf("        of the blocked points, %llu have a car's height of clearance or "
                        "less; %llu have the two layers agreeing on "
                        "height to %.1f m and %llu do not; %llu have no collision surface at all\n",
                        static_cast<unsigned long long>(lv.headroomHeightAgrees),
                        static_cast<unsigned long long>(lv.blockedHeightAgrees),
                        surveySettings.heightTolerance,
                        static_cast<unsigned long long>(lv.blockedHeightDiffers),
                        static_cast<unsigned long long>(lv.noSurface));
            for (const phys::PhysicalFleet::BlockedSample& b : lv.examples)
                std::printf("          blocked at (%8.1f, %8.1f) cls %u lanes %u: model %.2f, "
                            "collision %.2f, box base %.2f, %.2f m deep — %s\n",
                            b.at.x, b.at.y, static_cast<unsigned>(b.cls),
                            static_cast<unsigned>(b.lanes), b.model, b.ground, b.probeY - 0.36,
                            b.depth, b.what.c_str());
            if (lv.blocked > 0)
                std::printf("        deepest blockage at (%.1f, %.1f): %.2f m — %s\n", lv.blockedAt.x,
                            lv.blockedAt.y, lv.blockedDepthM, lv.blockedWhat.c_str());
            Json sv = Json::object();
            sv.set("spacing", Json(opt.surveySpacing));
            sv.set("samples", Json(n));
            sv.set("withinQuarter", Json(static_cast<f64>(lv.withinQuarter)));
            sv.set("withinHalf", Json(static_cast<f64>(lv.withinHalf)));
            sv.set("withinTwo", Json(static_cast<f64>(lv.withinTwo)));
            sv.set("blocked", Json(static_cast<f64>(lv.blocked)));
            sv.set("noSurface", Json(static_cast<f64>(lv.noSurface)));
            sv.set("structureOnly", Json(static_cast<f64>(lv.structureOnly)));
            sv.set("underStructure", Json(static_cast<f64>(lv.underStructure)));
            sv.set("noCollisionAtAll", Json(static_cast<f64>(lv.noCollisionAtAll)));
            sv.set("stepped", Json(static_cast<f64>(lv.stepped)));
            sv.set("worstErrorM", Json(lv.worstErrorM));
            sv.set("bridgeSamples", Json(static_cast<f64>(lv.bridgeSamples)));
            sv.set("bridgeWithinQuarter", Json(static_cast<f64>(lv.bridgeWithinQuarter)));
            sv.set("bridgeWithinTwo", Json(static_cast<f64>(lv.bridgeWithinTwo)));
            sv.set("bridgeNoDeck", Json(static_cast<f64>(lv.bridgeNoDeck)));
            sv.set("worstBridgeErrorM", Json(lv.worstBridgeErrorM));
            sv.set("collisionAboveModel", Json(static_cast<f64>(lv.collisionAbove)));
            sv.set("worstCollisionAboveM", Json(lv.worstCollisionAboveM));
            sv.set("elevatedSamples", Json(static_cast<f64>(lv.elevatedSamples)));
            sv.set("elevatedWithDeck", Json(static_cast<f64>(lv.elevatedWithDeck)));
            sv.set("worstElevatedDropM", Json(lv.worstElevatedDropM));
            sv.set("headroomSamples", Json(static_cast<f64>(lv.headroomSamples)));
            sv.set("minHeadroomM", Json(lv.headroomSamples ? lv.minHeadroomM : 0.0));
            sv.set("headroomUnderCar", Json(static_cast<f64>(lv.headroomUnderCar)));
            sv.set("headroomUnderTwo", Json(static_cast<f64>(lv.headroomUnderAny)));
            sv.set("seconds", Json(secs));
            reportSet("survey", sv);
        }

        world.penetration().reset();
        phys::PhysicalFleet fleet;
        phys::DriverSettings ds;
        ds.count = opt.drive;
        ds.attachRadius = 260.0;
        ds.trace = g_trace;
        // The penetration budget of *this* section: the sections before it (deck
        // drop, impact) leave their own deepest contact in the shared stats, and a
        // section that inherits a number cannot be said to have met it (D-034).
        world.penetration().reset();
        const size_t bodiesBefore = world.bodyCount();
        const size_t promoted = fleet.attach(world, traffic, anchor, ds);
        require(promoted >= std::min<size_t>(4, opt.drive),
                "the traffic model hands its vehicles to physics",
                sfmt("%zu of %u requested vehicles promoted near (%.0f, %.0f), %zu bodies, "
                     "model hands over %zu", promoted, opt.drive, anchor.x, anchor.y,
                     world.bodyCount(), traffic.promotedCount()));

        traffic.setAuditTrace(g_trace);
        const u64 ticks = static_cast<u64>(opt.seconds * 60.0);
        for (u64 i = 0; i < ticks; ++i) {
            traffic.step(kStep);
            fleet.step(world, traffic, kStep);
        }
        const phys::DriverMetrics dm = fleet.metrics();
        traffic.audit(extentMin.x, extentMin.x + 22528.0, extentMin.y, extentMin.y + 43008.0);
        const sim::TrafficMetrics after = traffic.metrics(anchor, 500.0);

        require(dm.promoted > 0, "vehicles are still driven at the end of the run",
                sfmt("%zu of %zu promoted vehicles still driven after %.0f s, %llu handed back",
                     dm.promoted, promoted, opt.seconds,
                     static_cast<unsigned long long>(dm.handedBack)));
        // The lane error is split into its two causes: `side` is the driver's own
        // distance from the lane line, `along` is how far the model's `s` is from
        // where the body actually is — which no driver can fix, and which is
        // reported beside it (D-034).
        require(dm.meanSideErrorM < 1.0 && dm.sideOffShare < 0.10,
                "a driven vehicle holds the lane the model put it in",
                sfmt("side error mean %.2f m, %.1f %% of samples more than %.0f m beside the "
                     "lane, worst side %.2f m; the model's own s was %.2f m from the body on "
                     "average, over %zu vehicles and %llu ticks", dm.meanSideErrorM,
                     dm.sideOffShare * 100.0, ds.offRoadMetres, dm.worstSideErrorM,
                     dm.meanAlongErrorM, promoted, static_cast<unsigned long long>(dm.ticks)));
        // Judged on the road the model put the vehicle on: a sample where the model
        // asked for 40 km/h on a road the collision does not have is an off-road
        // event (counted above), not a speed-tracking failure.  Both directions are
        // measured, so the check cannot pass by having no input.
        require(dm.onRoadMeanSpeedErrorMs < 2.5 && dm.onRoadWorstSpeedErrorMs < 9.0 &&
                    dm.onRoadSamples > 0,
                "a driven vehicle tracks the speed its model decided, on its own road",
                sfmt("mean speed %.1f km/h; over %llu driving samples on its own road the error "
                     "to the model's command was %.2f m/s mean / %.2f m/s worst (over every "
                     "sample: %.2f / %.2f m/s)", dm.meanSpeedKph,
                     static_cast<unsigned long long>(dm.onRoadSamples), dm.onRoadMeanSpeedErrorMs,
                     dm.onRoadWorstSpeedErrorMs, dm.meanSpeedErrorMs, dm.worstSpeedErrorMs));
        // A vehicle standing still in a jam is not evidence about wheels; a
        // vehicle driving at 40 km/h with three of its four wheels in the air is.
        // The criterion is therefore about the moving samples, and the count of
        // samples with *nothing* touching is reported beside it.
        require(dm.movingWheelFraction > 0.9,
                "a driven vehicle keeps its wheels on the road while it is moving",
                sfmt("%.1f %% of wheel contacts down over %llu moving samples (of %llu), "
                     "%llu samples with no wheel touching at all, over %zu vehicles",
                     dm.movingWheelFraction * 100.0,
                     static_cast<unsigned long long>(dm.movingSamples),
                     static_cast<unsigned long long>(dm.ticks),
                     static_cast<unsigned long long>(dm.airborneSamples), promoted));
        // Two checks, because one number conflated two events (D-037).  A body *at
        // rest* inside the city is a placement defect at any depth — that is the
        // strict budget.  A body *moving* against a vertical face — a kerb, a
        // parapet, the wall of a ramp — is Jolt's linear cast for one step, and its
        // bound is how far the body travels in a step; the driven-vs-driven check
        // below already grades that class against the same 0.30 m.  Both numbers are
        // printed, and the deepest over everything the section touched is printed
        // beside them, so the split hides nothing.  Measured (D-037): 0.0000 m at
        // rest, 0.0370 m while moving — a motorcycle at 10.7 km/h whose contact point
        // is 0.0001 m below the road surface, against the 2.2 m rise at the road's
        // own edge, not a body buried in a building.
        // Two checks, because a single number conflated two events (D-037), and
        // both can fail:
        //
        //  * *Carried* inside the city — a contact that survives more than three
        //    consecutive ticks (67 ms) while a driven vehicle is over the budget.
        //    Jolt resolves a contact within a step or two, so this is the strict
        //    one, and it is the S0 rule about penetration: a body the solver
        //    leaves inside the world.
        //  * *Passing through* the city — a single-tick depth beyond Jolt's own
        //    linear-cast allowance, the same 0.30 m the deck impact and the
        //    driven-vs-driven check are graded against.  This is the one that
        //    catches tunnelling at speed.
        //
        // Measured (D-037), on the run this comment was written from: longest run
        // 1 tick, deepest sustained 0.0000 m, deepest single-tick 0.0370 m at
        // 10.6 km/h.  The two events that produce 0.0370 m and 0.0308 m are a
        // motorcycle against the face of the raised edge beside its own road: the
        // contact point is 0.0001 m below the road surface and the surface inside
        // the body's own bounds spans 2.2157 m (1.93..4.15 m).  Nothing is carried.
        require(dm.deepestSustainedStaticM < kPenetrationBudget,
                "no driven vehicle is carried inside the city",
                sfmt("deepest contact that survived more than 3 ticks (67 ms) %.4f m (budget "
                     "%.3f m); longest run of consecutive ticks over the budget %llu ticks; the "
                     "deepest single-tick contact was %.4f m at %.1f km/h (%.4f m of it at under "
                     "1.8 km/h), over everything the section touched %.4f m; %llu of %zu vehicles "
                     "handed back to the model",
                     dm.deepestSustainedStaticM, kPenetrationBudget,
                     static_cast<unsigned long long>(dm.longestStaticRunTicks),
                     dm.deepestDrivenStaticM, dm.worstMovingContactSpeedMs * 3.6,
                     dm.deepestDrivenStaticAtRestM, dm.deepestPenetrationM,
                     static_cast<unsigned long long>(dm.handedBack), promoted));
        require(dm.deepestDrivenStaticM < kImpactTransientBudget,
                "no vehicle passes through the city",
                sfmt("deepest single-tick contact of a driven vehicle with the city %.4f m "
                     "(allowance %.2f m, Jolt's linear cast for one step at %.1f km/h); sustained "
                     "beyond 3 ticks %.4f m (budget %.3f m); %llu of %zu vehicles handed back",
                     dm.deepestDrivenStaticM, kImpactTransientBudget,
                     dm.worstMovingContactSpeedMs * 3.6, dm.deepestSustainedStaticM,
                     kPenetrationBudget, static_cast<unsigned long long>(dm.handedBack),
                     promoted));
        // A driven vehicle against another body is a different event with a different
        // budget: two solver objects resolving a contact.  The allowance is Jolt's
        // own, the same 0.30 m of linear-cast transient the deck impact is graded
        // against, and the measured worst here is a 16 t double-decker shunting a
        // 1.15 t taxi (0.1007 m).  Before this check existed the harness read the
        // *static* depth for both cases, so every vehicle-against-vehicle contact in
        // the section was invisible (D-034).
        require(dm.deepestDrivenDynamicM < kImpactTransientBudget,
                "no two driven vehicles are driven through each other",
                sfmt("deepest contact between driven vehicles and other bodies %.4f m "
                     "(allowance %.2f m, Jolt's linear-cast transient); %llu of %zu vehicles "
                     "handed back to the model", dm.deepestDrivenDynamicM,
                     kImpactTransientBudget, static_cast<unsigned long long>(dm.handedBack),
                     promoted));

        // The model must be no worse for having been driven: its own invariants,
        // over a fleet that is now half bodies and half points on a graph.
        const bool modelClean = after.badState == 0 && after.badSpeed == 0 &&
                                after.badPosition == 0 && after.laneViolations == 0;
        require(modelClean && after.edgeCompletions > before.edgeCompletions,
                "the model keeps its invariants while half its fleet is physical",
                sfmt("%zu vehicles (%.1f km/h mean), %llu edge completions (+%llu), %llu lane "
                     "changes, 0 invalid states, %llu lane violations",
                     after.vehicles, after.meanSpeedKph,
                     static_cast<unsigned long long>(after.edgeCompletions),
                     static_cast<unsigned long long>(after.edgeCompletions - before.edgeCompletions),
                     static_cast<unsigned long long>(after.laneChanges),
                     static_cast<unsigned long long>(after.laneViolations)));
        if (g_trace)
            std::printf("      model: %llu anchor releases, %llu lane-list repairs, %llu junction "
                        "holds, %llu dead-end holds, %llu U-turns at dead ends\n",
                        static_cast<unsigned long long>(after.anchorReleases),
                        static_cast<unsigned long long>(after.orphanRepairs),
                        static_cast<unsigned long long>(after.junctionHolds),
                        static_cast<unsigned long long>(after.deadEndHolds),
                        static_cast<unsigned long long>(after.deadEndUTurns));

        if (g_trace) {
            std::printf("      penetration: deepest static %.4f m (dynamic body %u vs %u), "
                        "deepest dynamic %.4f m (body %u vs %u)\n",
                        world.penetration().deepestStatic,
                        world.penetration().deepestStaticBody.GetIndex(),
                        world.penetration().deepestStaticPartner.GetIndex(),
                        world.penetration().deepestDynamic,
                        world.penetration().deepestDynamicBody.GetIndex(),
                        world.penetration().deepestDynamicPartner.GetIndex());
            std::printf("      deepest static contact at (%.1f, %.1f, %.1f), dynamic at "
                        "(%.1f, %.1f, %.1f)\n",
                        world.penetration().deepestStaticAt.x, world.penetration().deepestStaticAt.y,
                        world.penetration().deepestStaticAt.z,
                        world.penetration().deepestDynamicAt.x, world.penetration().deepestDynamicAt.y,
                        world.penetration().deepestDynamicAt.z);
            fleet.report();
        }

        // Reversibility: the handoff is not a one-way door.
        fleet.detach(world, traffic);
        const size_t bodiesAfter = world.bodyCount();
        for (int i = 0; i < 120; ++i) traffic.step(kStep);
        const sim::TrafficMetrics handed = traffic.metrics(anchor, 500.0);
        require(traffic.promotedCount() == 0 && bodiesAfter == bodiesBefore &&
                    handed.badState == 0 && handed.badPosition == 0 &&
                "vehicles handed back resume as model vehicles, with nothing left behind",
                sfmt("%zu bodies before the handover and %zu after the handback (%zu vehicle "
                     "bodies created and destroyed), %zu promotions / %zu demotions, model at "
                     "%zu vehicles and %llu %s", bodiesBefore, bodiesAfter, promoted,
                     traffic.promotions(), traffic.demotions(), handed.vehicles,
                     static_cast<unsigned long long>(handed.badPosition),
                     handed.badPosition ? "positions out of the world's extent"
                                        : "positions in extent"));

        {
            Json j = Json::object();
            j.set("population", Json(static_cast<f64>(population)));
            j.set("graphBuildSeconds", Json(buildMs / 1000.0));
            j.set("modelVehiclesBefore", Json(static_cast<f64>(before.vehicles)));
            j.set("modelVehiclesAfter", Json(static_cast<f64>(after.vehicles)));
            j.set("modelMeanSpeedKphAfter", Json(after.meanSpeedKph));
            j.set("promoted", Json(static_cast<f64>(promoted)));
            j.set("rejectedSpawns", Json(static_cast<f64>(dm.rejectedSpawns)));
            j.set("promotedAtEnd", Json(static_cast<f64>(dm.promoted)));
            j.set("handedBack", Json(static_cast<f64>(dm.handedBack)));
            j.set("ticks", Json(static_cast<f64>(dm.ticks)));
            j.set("meanLateralOffM", Json(dm.meanLateralOffM));
            j.set("meanSideErrorM", Json(dm.meanSideErrorM));
            j.set("meanAlongErrorM", Json(dm.meanAlongErrorM));
            j.set("sideOffShare", Json(dm.sideOffShare));
            j.set("laneErrorRmsM", Json(dm.laneErrorRmsM));
            j.set("onRoadSamples", Json(static_cast<f64>(dm.onRoadSamples)));
            j.set("onRoadMeanSpeedErrorMs", Json(dm.onRoadMeanSpeedErrorMs));
            j.set("onRoadWorstSpeedErrorMs", Json(dm.onRoadWorstSpeedErrorMs));
            j.set("handedBackWrongWay", Json(static_cast<f64>(dm.handedBackWrongWay)));
            j.set("worstLateralOffM", Json(dm.worstLateralOffM));
            j.set("meanSpeedErrorMs", Json(dm.meanSpeedErrorMs));
            j.set("worstSpeedErrorMs", Json(dm.worstSpeedErrorMs));
            j.set("wheelsDownFraction", Json(dm.wheelsDownFraction));
            j.set("movingWheelFraction", Json(dm.movingWheelFraction));
            j.set("movingSamples", Json(static_cast<f64>(dm.movingSamples)));
            j.set("airborneSamples", Json(static_cast<f64>(dm.airborneSamples)));
            j.set("offRoadShare", Json(dm.offRoadShare));
            j.set("meanHeightErrorM", Json(dm.meanHeightErrorM));
            j.set("worstHeightErrorM", Json(dm.worstHeightErrorM));
            j.set("handedBackHeight", Json(static_cast<f64>(dm.handedBackHeight)));
            j.set("handedBackAhead", Json(static_cast<f64>(dm.handedBackAhead)));
            j.set("handedBackOffRoad", Json(static_cast<f64>(dm.handedBackOffRoad)));
            j.set("meanSpeedKph", Json(dm.meanSpeedKph));
            j.set("deepestPenetrationM", Json(dm.deepestPenetrationM));
            j.set("deepestDrivenM", Json(dm.deepestDrivenM));
            j.set("deepestDrivenStaticM", Json(dm.deepestDrivenStaticM));
            j.set("deepestDrivenDynamicM", Json(dm.deepestDrivenDynamicM));
            j.set("rejectedOnTraffic", Json(static_cast<f64>(dm.rejectedOnTraffic)));
            j.set("rejectedOnCity", Json(static_cast<f64>(dm.rejectedOnCity)));
            j.set("laneChanges", Json(static_cast<f64>(after.laneChanges)));
            j.set("edgeCompletions", Json(static_cast<f64>(after.edgeCompletions)));
            j.set("laneViolations", Json(static_cast<f64>(after.laneViolations)));
            reportSet("physicalTraffic", j);
        }
    }

    // --- 8. floating origin ---------------------------------------------------
    section("floating origin");
    {
        std::vector<phys::BodyHandle> movers;
        const Slab& s = deck.slabs[deck.slabs.size() / 2];
        for (int i = 0; i < 4; ++i) {
            phys::BodyHandle h = world.addDynamicBox(
                Vec3(static_cast<f32>(s.x + i * 3.0), static_cast<f32>(s.y + 6.0),
                     static_cast<f32>(s.z)), Vec3(0.5f, 0.5f, 0.5f), 0.0, 40.0);
            if (h.valid()) movers.push_back(h);
        }
        require(!movers.empty(), "there are bodies to move on a crossing",
                sfmt("%zu movers and %u bodies in the world", movers.size(), world.bodyCount()));

        std::vector<Vec3> velocityBefore;
        for (const phys::BodyHandle& h : movers) {
            const J::Vec3 v = world.bodies().GetLinearVelocity(h.id);
            velocityBefore.push_back(Vec3(v.GetX(), v.GetY(), v.GetZ()));
        }
        f64 worstMs = 0.0, meanMs = 0.0, worstError = 0.0;
        u64 crossings = 0, moved = 0;
        const Vec3 step(1024.0f, 0.0f, 0.0f);
        // Warm the path and the cache: the first crossing pays for page faults on
        // 1,454 bodies, and charging that to a per-crossing budget would be
        // measuring the allocator, not the shift.  The pair cancels out.
        world.rebase(step);
        world.rebase(Vec3(-step.x, 0.0f, 0.0f));
        for (int i = 0; i < 20; ++i) {
            const phys::World::RebaseResult rr = world.rebase(i % 2 == 0 ? step : Vec3(-1024.0f, 0, 0));
            // Charge the shift to a whole macro cell of travel: the runtime calls
            // it once per 1024 m, not once per frame.
            worstMs = std::max(worstMs, rr.milliseconds);
            meanMs += rr.milliseconds;
            worstError = std::max(worstError, rr.worstPositionError);
            ++crossings;
            moved = std::max(moved, rr.bodies);
            world.step(static_cast<f32>(kStep));
        }
        meanMs /= static_cast<f64>(crossings);
        require(worstMs < 1.0, "20 macro-cell crossings shift every body in under a millisecond",
                sfmt("%llu bodies, mean %.4f ms, worst %.4f ms", moved, meanMs, worstMs));
        require(worstError < 0.001, "a crossing does not displace a body",
                sfmt("worst position error %.2e m", worstError));
        {
            f64 worstVel = 0.0;
            for (size_t i = 0; i < movers.size(); ++i) {
                const J::Vec3 v = world.bodies().GetLinearVelocity(movers[i].id);
                worstVel = std::max(worstVel, std::fabs(static_cast<f64>(v.GetX()) - velocityBefore[i].x));
                worstVel = std::max(worstVel, std::fabs(static_cast<f64>(v.GetY()) - velocityBefore[i].y));
                worstVel = std::max(worstVel, std::fabs(static_cast<f64>(v.GetZ()) - velocityBefore[i].z));
            }
            // A shift is not an impulse: no body's velocity may be changed by
            // `rebase` beyond what the simulation itself did in the tick after it.
            require(worstVel < 8.0, "a crossing is not an impulse",
                    sfmt("worst velocity change %.2f m/s over 20 crossings including gravity",
                         worstVel));
            Json j = Json::object();
            j.set("crossings", Json(static_cast<f64>(crossings)));
            j.set("bodiesMoved", Json(static_cast<f64>(moved)));
            j.set("meanMs", Json(meanMs));
            j.set("worstMs", Json(worstMs));
            j.set("worstPositionErrorM", Json(worstError));
            reportSet("floatingOrigin", j);
        }
        for (const phys::BodyHandle& h : movers) world.remove(h);
    }

    // --- 9. soak --------------------------------------------------------------
    section("soak");
    {
        // Dynamic props on the deck: boxes stacked on a carriageway, which is how
        // the world's own debris will behave, plus the vehicles of the fleet run.
        std::vector<phys::BodyHandle> props;
        const size_t baseIndex = deck.slabs.size() / 3;
        const Slab& base = deck.slabs[baseIndex];
        const f64 baseSurface = surfaceOf(deck, baseIndex);
        f64 worstPropClearance = 1e30;   // crate underside minus the surface under it
        for (u32 i = 0; i < opt.props; ++i) {
            const u32 col = i % 10, row = (i / 10) % 5;
            const f64 px = base.x + (col - 5) * 1.4;
            const f64 pz = base.z + (i / 50) * 2.0;
            // The deck is pitched and cambered: a row laid out in world axes sits
            // up to 20 cm inside it at its ends, and a harness that authors that
            // overlap then reports it as penetration.  So each crate is placed on
            // the surface *under its own footprint*, casting from the row above.
            const f64 under = world.groundHeightStatic(
                Vec3(static_cast<f32>(px), static_cast<f32>(baseSurface + 1.6 + row * 1.02),
                     static_cast<f32>(pz)), 3.0);
            const f64 floor = std::isfinite(under) ? under : baseSurface;
            const Vec3 at(static_cast<f32>(px),
                          static_cast<f32>(floor + 0.52 + row * 1.02),
                          static_cast<f32>(pz));
            // Measure the authored clearance back: a crate authored inside the
            // deck would show up as solver penetration later, and the point of
            // this harness is to attribute that to the author, not the solver.
            const f64 beneath = world.groundHeightStatic(Vec3(at.x, at.y, at.z), 5.0);
            if (std::isfinite(beneath))
                worstPropClearance = std::min(worstPropClearance, (at.y - 0.5) - beneath);
            phys::BodyHandle h = world.addDynamicBox(at, Vec3(0.5f, 0.5f, 0.5f), 0.0, 40.0);
            if (h.valid()) props.push_back(h);
        }
        require(props.size() == opt.props && worstPropClearance > -0.02,
                "every prop is authored on the surface, not inside it",
                sfmt("%zu crates, worst authored clearance %+.3f m (a negative value would be "
                     "the harness authoring its own penetration)",
                     props.size(), worstPropClearance));
        // The soak measures its own penetration, so its instrument starts clean:
        // without this it reports the deepest contact the world has ever seen,
        // which was the 22 m impact in the section above.
        world.penetration().reset();
        const u64 steps = static_cast<u64>(opt.minutes * 60.0 * 60.0);
        std::vector<f64> costs;
        costs.reserve(static_cast<size_t>(steps));
        f64 deepest = 0.0;
        bool finite = true;
        auto t0 = std::chrono::steady_clock::now();
        for (u64 i = 0; i < steps; ++i) {
            const auto s0 = std::chrono::steady_clock::now();
            world.step(static_cast<f32>(kStep));
            const f64 ms = std::chrono::duration<f64>(std::chrono::steady_clock::now() - s0).count() * 1000.0;
            costs.push_back(ms);
            deepest = std::max(deepest, world.penetration().deepestStatic);
            if (!(i % 600)) {
                for (const phys::BodyHandle& h : props) {
                    const Vec3 p = world.position(h);
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) finite = false;
                }
                if (!finite) break;
            }
        }
        const f64 wallSeconds = std::chrono::duration<f64>(std::chrono::steady_clock::now() - t0).count();
        std::sort(costs.begin(), costs.end());
        auto pct = [&](f64 q) {
            if (costs.empty()) return 0.0;
            return costs[std::min(costs.size() - 1, static_cast<size_t>(q * costs.size()))];
        };
        const f64 p50 = pct(0.50), p99 = pct(0.99), worst = costs.empty() ? 0 : costs.back();
        const f64 mean = costs.empty() ? 0.0
                                       : std::accumulate(costs.begin(), costs.end(), 0.0) /
                                             static_cast<f64>(costs.size());
        require(finite, "no body goes non-finite in the soak",
                sfmt("%llu steps, %zu props", static_cast<unsigned long long>(steps), props.size()));
        require(deepest <= kPenetrationBudget, "the soak holds the penetration budget",
                sfmt("deepest %.4f m (budget %.3f) over %zu props and %u bodies", deepest,
                     kPenetrationBudget, props.size(), world.bodyCount()));
        require(worst < 16.0, "a physics step fits in the frame",
                sfmt("mean %.3f ms, p50 %.3f, p99 %.3f, worst %.3f (%.1f s wall for %.2f "
                     "simulated minutes, %u bodies live)", mean, p50, p99, worst, wallSeconds,
                     opt.minutes, world.bodyCount()));
        {
            Json j = Json::object();
            j.set("simulatedMinutes", Json(opt.minutes));
            j.set("steps", Json(static_cast<f64>(steps)));
            j.set("meanMs", Json(mean));
            j.set("p50Ms", Json(p50));
            j.set("p99Ms", Json(p99));
            j.set("worstMs", Json(worst));
            j.set("deepestPenetrationM", Json(deepest));
            j.set("props", Json(static_cast<f64>(props.size())));
            j.set("bodies", Json(static_cast<f64>(world.bodyCount())));
            reportSet("soak", j);
        }
        for (const phys::BodyHandle& h : props) world.remove(h);
    }

    // --- summary --------------------------------------------------------------
    u64 passed = 0, failed = 0;
    for (const Check& c : g_checks) (c.ok ? passed : failed)++;
    std::printf("\n== summary ==\n");
    std::printf("  %zu checks, %llu failed\n", g_checks.size(),
                static_cast<unsigned long long>(failed));
    for (const Check& c : g_checks)
        if (!c.ok) std::printf("    FAIL  %s — %s\n", c.name.c_str(), c.detail.c_str());

    Json checks = Json::object();
    checks.set("total", Json(static_cast<f64>(g_checks.size())));
    checks.set("passed", Json(static_cast<f64>(passed)));
    checks.set("failed", Json(static_cast<f64>(failed)));
    reportSet("checks", checks);
    reportSet("generatedBy", Json("mumbai_phys (engine/app/phys_main.cpp)"));
    reportSet("exitCode", Json(static_cast<f64>(failed == 0 ? 0 : 2)));

    if (!opt.report.empty()) {
        const fs::path p(opt.report);
        if (!p.parent_path().empty()) fs::create_directories(p.parent_path());
        std::ofstream out(opt.report);
        if (out) out << g_report.dump(2) << "\n";
        if (!g_quiet) std::printf("  report   %s\n", opt.report.c_str());
    }

    world.shutdown();
    return failed == 0 ? 0 : 2;
}
