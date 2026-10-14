// =============================================================================
//  MUMBAI 1:1 EXACT — pipeline/mgeo/texture.cpp
// =============================================================================
#include "pipeline/mgeo/texture.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace mumbai::tex {
namespace {

// -----------------------------------------------------------------------------
//  Deterministic noise.  Everything downstream of here is a pure function of the
//  material's name and the pixel coordinate, so a texture can be regenerated in
//  isolation and two builds agree byte for byte.
// -----------------------------------------------------------------------------
inline u64 mix64(u64 x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

inline f32 hash01(i64 x, i64 y, u64 seed) {
    u64 h = mix64(static_cast<u64>(x) * 0x9E3779B1u ^ mix64(static_cast<u64>(y) * 0x85EBCA77u) ^ seed);
    return static_cast<f32>(h >> 40) * (1.0f / 16777216.0f);
}

inline f32 smoothstep5(f32 t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }
inline f32 smoothstep(f32 e0, f32 e1, f32 x) {
    const f32 t = clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
inline f32 lerp(f32 a, f32 b, f32 t) { return a + (b - a) * t; }

// Tileable value noise on a `period`-cell lattice.  Wrapping the lattice rather
// than the result is what makes the tile seamless: a brick drawn at u = 0 and a
// brick drawn at u = 1 are the same brick, which is why the city can repeat a
// 1 m tile across a 40 m facade without a visible seam.
f32 valueNoise(f32 u, f32 v, i64 period, u64 seed) {
    const f32 x = u * static_cast<f32>(period), y = v * static_cast<f32>(period);
    const i64 x0 = static_cast<i64>(std::floor(x)), y0 = static_cast<i64>(std::floor(y));
    const f32 fx = smoothstep5(x - static_cast<f32>(x0));
    const f32 fy = smoothstep5(y - static_cast<f32>(y0));
    auto wrap = [period](i64 c) { i64 m = c % period; return m < 0 ? m + period : m; };
    const i64 xa = wrap(x0), xb = wrap(x0 + 1);
    const i64 ya = wrap(y0), yb = wrap(y0 + 1);
    const f32 n00 = hash01(xa, ya, seed), n10 = hash01(xb, ya, seed);
    const f32 n01 = hash01(xa, yb, seed), n11 = hash01(xb, yb, seed);
    return lerp(lerp(n00, n10, fx), lerp(n01, n11, fx), fy);
}

// Fractal sum of tileable octaves.  Octaves are integer multiples of the base
// period, which keeps the sum tileable as well.
f32 fbm(f32 u, f32 v, i64 basePeriod, int octaves, f32 gain, u64 seed) {
    f32 sum = 0.0f, amp = 1.0f, norm = 0.0f;
    i64 period = basePeriod;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * valueNoise(u, v, period, seed + static_cast<u64>(o) * 0x9E3779B97F4A7C15ull);
        norm += amp;
        amp *= gain;
        period *= 2;
    }
    return sum / norm;
}

// Ridged noise: creases and veins, for bark, cracks and torn edges.
f32 ridged(f32 u, f32 v, i64 period, int octaves, u64 seed) {
    f32 sum = 0.0f, amp = 1.0f, norm = 0.0f;
    i64 p = period;
    for (int o = 0; o < octaves; ++o) {
        const f32 n = valueNoise(u, v, p, seed + static_cast<u64>(o) * 0x1234567ull);
        sum += amp * (1.0f - std::fabs(n * 2.0f - 1.0f));
        norm += amp;
        amp *= 0.5f;
        p *= 2;
    }
    return sum / norm;
}

// Tileable cellular pattern: returns the distance to the nearest of `cells`
// feature points per axis (in [0, ~1]) and, through `cellId`, which feature it
// was — the id is what gives each brick or cobble its own tint.
f32 worley(f32 u, f32 v, i64 cells, u64 seed, u32* cellId) {
    const f32 x = u * static_cast<f32>(cells), y = v * static_cast<f32>(cells);
    const i64 xi = static_cast<i64>(std::floor(x)), yi = static_cast<i64>(std::floor(y));
    f32 best = 1e9f;
    u32 bestId = 0;
    for (i64 dy = -1; dy <= 1; ++dy) {
        for (i64 dx = -1; dx <= 1; ++dx) {
            const i64 cx = xi + dx, cy = yi + dy;
            i64 wx = cx % cells; if (wx < 0) wx += cells;
            i64 wy = cy % cells; if (wy < 0) wy += cells;
            const f32 jx = hash01(wx, wy, seed), jy = hash01(wx, wy, seed ^ 0xABCDEFull);
            const f32 px = static_cast<f32>(cx) + 0.15f + 0.7f * jx;
            const f32 py = static_cast<f32>(cy) + 0.15f + 0.7f * jy;
            const f32 d = (px - x) * (px - x) + (py - y) * (py - y);
            if (d < best) {
                best = d;
                bestId = static_cast<u32>(mix64(static_cast<u64>(wx) * 131ull + static_cast<u64>(wy)) >> 40);
            }
        }
    }
    if (cellId) *cellId = bestId;
    return std::sqrt(best);
}

// --- colour helpers ----------------------------------------------------------
struct Rgb { f32 r, g, b; };
inline Rgb operator*(const Rgb& c, f32 s) { return {c.r * s, c.g * s, c.b * s}; }
inline Rgb operator+(const Rgb& a, const Rgb& b) { return {a.r + b.r, a.g + b.g, a.b + b.b}; }
inline Rgb operator*(const Rgb& a, const Rgb& b) { return {a.r * b.r, a.g * b.g, a.b * b.b}; }
inline Rgb mix(const Rgb& a, const Rgb& b, f32 t) {
    return {lerp(a.r, b.r, t), lerp(a.g, b.g, t), lerp(a.b, b.b, t)};
}
inline f32 lum(const Rgb& c) { return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

// Linear <-> sRGB.  The albedo in MaterialDef is linear (the pipeline and the
// runtime both work in linear light); a base-colour *texture* is sRGB by
// convention, so the generator writes sRGB and the reader converts back.  Doing
// this wrong is the classic way to ship a city that is either washed out or
// black, so it is done in exactly one place: here.
inline f32 linearToSrgb(f32 c) {
    c = clamp(c, 0.0f, 1.0f);
    return c <= 0.0031308f ? c * 12.92f : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
}
inline f32 srgbToLinear(f32 c) {
    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

// -----------------------------------------------------------------------------
//  A tile under construction: height + albedo + roughness, which is all any of
//  the recipes ever need to describe.  Normal and ORM are derived from these
//  afterwards, so a recipe cannot produce a normal map that disagrees with the
//  surface it describes — the geometry and the shading come from one height field.
// -----------------------------------------------------------------------------
struct Tile {
    u32 size;
    std::vector<f32> height;      // 0..1, meters-agnostic bump amplitude
    std::vector<f32> albedo;      // linear RGB, size*3
    std::vector<f32> rough;       // absolute roughness, size
    std::vector<f32> ao;          // 0..1, 1 = unoccluded
    u32 index(u32 x, u32 y) const { return y * size + x; }
    void init(u32 n) {
        size = n;
        height.assign(static_cast<size_t>(n) * n, 0.5f);
        albedo.assign(static_cast<size_t>(n) * n * 3, 0.5f);
        rough.assign(static_cast<size_t>(n) * n, 0.5f);
        ao.assign(static_cast<size_t>(n) * n, 1.0f);
    }
};

// -----------------------------------------------------------------------------
//  Recipe table.  One row per material family; `tile` is the physical size of
//  one tile in metres, which is what the mesh layer divides its UVs by.
// -----------------------------------------------------------------------------
struct Family {
    const char* name;
    f32 tile;
};

// Longest-match-first, so `asphalt_patched` finds its own row before `asphalt`.
const Family kFamilies[] = {
    {"asphalt_patched",  5.0f},
    {"asphalt_smooth",   6.0f},
    {"asphalt_sealink",  8.0f},
    {"concrete_road",    4.0f},
    {"paver_block",      1.2f},
    {"gravel",           2.0f},
    {"dirt",             3.0f},
    {"rock",             3.0f},
    {"earth",            3.0f},
    {"sand",             4.0f},
    {"wetland_mud",      3.0f},
    {"reclamation_fill", 2.5f},
    {"grass",            2.0f},
    {"plaster_painted",  2.0f},
    {"stone_ashlar",     2.4f},
    {"stone_rubble",     2.0f},
    {"curtain_wall_glass", 4.0f},
    {"brick_exposed",    1.0f},
    {"corrugated_metal", 0.9f},
    {"concrete_raw",     2.5f},
    {"kerb_concrete",    1.5f},
    {"sidewalk_concrete",2.0f},
    {"median_barrier",   2.0f},
    {"water_surface",   12.0f},
    {"inland_water",     8.0f},
    {"rail_ballast",     1.6f},
    {"rail_steel",       1.0f},
    {"rail_sleeper",     1.0f},
    {"water_tank_plastic", 1.0f},
    {"ac_unit",          1.0f},
    {"metal_railing",    1.0f},
    {"signage_painted",  1.0f},
    {"tree_foliage",     1.5f},
    {"tree_bark",        1.0f},
    {"streetlamp_pole",  1.2f},
    {"streetlamp_glow",  0.5f},
    {"hero_stone",       1.6f},
    {"hero_terracotta",  1.2f},
    {"hero_concrete",    2.0f},
    {"sea_link_pylon",   3.0f},
    {"sea_link_cable",   0.8f},
    {"unknown",          2.0f},
};

// Materials whose surface is optically smooth: a mirror-flat curtain wall has no
// grain to resolve, so texturing it would be inventing detail the building does
// not have.  They get a flat tile (mean preserved, no normal) at a UV scale that
// sharpens nothing.
bool isSmoothFamily(const char* f) {
    return std::strcmp(f, "curtain_wall_glass") == 0 ||
           std::strcmp(f, "streetlamp_glow") == 0 ||
           std::strcmp(f, "water_surface") == 0 ||
           std::strcmp(f, "inland_water") == 0 ||
           std::strcmp(f, "rail_steel") == 0;
}

}  // namespace

// -----------------------------------------------------------------------------
//  Family / tile lookup
// -----------------------------------------------------------------------------
//  Exact match first — for the shipping library a material's name *is* its
//  family — then longest prefix, so a minted facade variant ("plaster_painted"
//  with a weathering suffix, say) inherits its base material's family and tile
//  size instead of degrading to `unknown`.  Table order breaks ties, so the
//  result is a pure function of the name: the same world always bakes the same
//  textures and the same UV scales.
const Family* findFamily(const std::string& name) {
    for (const Family& f : kFamilies)
        if (name == f.name) return &f;
    const Family* best = nullptr;
    size_t bestLen = 0;
    for (const Family& f : kFamilies) {
        const size_t n = std::strlen(f.name);
        if (n > bestLen && name.compare(0, n, f.name) == 0) {
            best = &f;
            bestLen = n;
        }
    }
    return best;
}

f32 tileMetresFor(const std::string& materialName) {
    const Family* f = findFamily(materialName);
    return f ? f->tile : 2.0f;
}

const char* familyFor(const std::string& materialName) {
    const Family* f = findFamily(materialName);
    return f ? f->name : "unknown";
}
// -----------------------------------------------------------------------------
//  Recipes.  Each one writes height / albedo / roughness / occlusion for one
//  material; the caller turns those into the three shipping maps.
// -----------------------------------------------------------------------------
Generator::Generator(u32 size) : m_size(size ? size : 256) {}

TextureSet Generator::make(const std::string& name, f32 roughness, f32 metallic,
                           const f32 baseColor[4]) {
    const u32 N = m_size;
    const std::string famStr = familyFor(name);
    const char* fam = famStr.c_str();
    // Seed from the name itself, so two materials in the same family (asphalt
    // smooth vs patched, two facade variants) do not share a pattern.
    u64 s = 1469598103934665603ull;
    for (char c : name) s = (s ^ static_cast<u64>(static_cast<u8>(c))) * 1099511628211ull;

    Tile t;
    t.init(N);
    for (u32 y = 0; y < N; ++y) {
        const f32 v = (static_cast<f32>(y) + 0.5f) / static_cast<f32>(N);
        for (u32 x = 0; x < N; ++x) {
            const f32 u = (static_cast<f32>(x) + 0.5f) / static_cast<f32>(N);
            const u32 i = t.index(x, y);
            f32 h = 0.5f, rough = 0.5f, ao = 1.0f;
            Rgb alb{0.5f, 0.5f, 0.5f};

            // ---- one branch per family -------------------------------------
            if (std::strcmp(fam, "asphalt_smooth") == 0) {
                const f32 grain = fbm(u, v, 32, 4, 0.55f, s);
                const f32 polish = fbm(u, v, 3, 3, 0.5f, s ^ 0x51ull);
                // Aggregate speckle: on a dark surface the *encoded* contrast has
                // to be pushed, or the grain disappears — sRGB squeezes differences
                // at the bottom of the range, and asphalt is the single most
                // visible surface in the city.
                const f32 speck = fbm(u, v, 96, 2, 0.6f, s ^ 0xA7ull);
                h = 0.35f + 0.5f * grain + 0.12f * speck;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.86f + 0.30f * grain) * (0.94f + 0.12f * polish) *
                      (0.93f + 0.14f * speck);
                rough = 0.80f + 0.14f * polish - 0.08f * grain + 0.06f * speck;
                ao = 0.94f + 0.06f * grain;
            } else if (std::strcmp(fam, "asphalt_patched") == 0) {
                const f32 patch = fbm(u, v, 4, 4, 0.55f, s ^ 0x77ull);
                const f32 grain = fbm(u, v, 32, 4, 0.55f, s);
                const f32 crack = ridged(u, v, 6, 4, s ^ 0xC0ull);
                const f32 crackMask = 1.0f - smoothstep(0.0f, 1.0f, clamp((crack - 0.72f) * 6.0f, 0.0f, 1.0f));
                h = 0.42f + 0.4f * grain - 0.22f * (1.0f - crackMask);
                const f32 dark = patch < 0.45f ? 0.80f : 1.0f;      // a patch is darker
                alb = Rgb{1.0f, 1.0f, 1.0f} * dark * (0.90f + 0.18f * grain);
                rough = 0.86f + 0.12f * grain + 0.06f * (1.0f - crackMask);
                ao = 1.0f - 0.25f * (1.0f - crackMask);
            } else if (std::strcmp(fam, "asphalt_sealink") == 0) {
                const f32 grain = fbm(u, v, 48, 3, 0.5f, s);
                // Bridge decks carry transverse construction joints: a shallow,
                // regular line every tile.
                const f32 joint = std::fabs(std::fmod(v * 1.0f, 0.5f) - 0.25f);
                const f32 jointMask = 1.0f - smoothstep(0.0f, 1.0f, clamp((joint - 0.02f) * 40.0f, 0.0f, 1.0f));
                h = 0.4f + 0.45f * grain - 0.3f * (1.0f - jointMask);
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * grain) * (1.0f - 0.15f * (1.0f - jointMask));
                rough = 0.72f + 0.10f * grain;
                ao = 1.0f - 0.18f * (1.0f - jointMask);
            } else if (std::strcmp(fam, "concrete_road") == 0) {
                const f32 grain = fbm(u, v, 24, 4, 0.5f, s);
                const f32 jointU = std::fabs(std::fmod(u, 0.5f) - 0.25f);
                const f32 jointV = std::fabs(std::fmod(v, 0.5f) - 0.25f);
                auto joint = [](f32 d) {
                    return 1.0f - smoothstep(0.0f, 1.0f, clamp((d - 0.012f) * 60.0f, 0.0f, 1.0f));
                };
                const f32 j = std::max(joint(jointU), joint(jointV));
                h = 0.45f + 0.4f * grain - 0.35f * (1.0f - j);
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * grain) * (1.0f - 0.12f * (1.0f - j));
                rough = 0.70f + 0.16f * grain;
                ao = 1.0f - 0.22f * (1.0f - j);
            } else if (std::strcmp(fam, "paver_block") == 0) {
                // Interlocking pavers: a 6 x 12 grid per tile, per-block tint.
                const f32 bu = u * 6.0f, bv = v * 12.0f;
                const i64 bx = static_cast<i64>(bu), by = static_cast<i64>(bv);
                const f32 fu = bu - static_cast<f32>(bx), fv = bv - static_cast<f32>(by);
                const f32 eu = std::min(fu, 1.0f - fu), ev = std::min(fv, 1.0f - fv);
                const f32 mortar = std::min(eu, ev);
                const f32 seam = 1.0f - smoothstep(0.0f, 1.0f, clamp((mortar - 0.05f) * 20.0f, 0.0f, 1.0f));
                u32 id;
                worley(u, v, 6, s, &id);
                const f32 tint = 0.88f + 0.24f * (static_cast<f32>(id % 1024u) / 1023.0f);
                const f32 grain = fbm(u, v, 32, 3, 0.5f, s ^ 0x33ull);
                h = 0.72f - 0.5f * seam + 0.06f * grain;
                alb = Rgb{1.0f, 1.0f, 1.0f} * tint * (0.94f + 0.12f * grain) * (1.0f - 0.25f * seam);
                rough = 0.78f + 0.14f * grain;
                ao = 1.0f - 0.35f * seam;
            } else if (std::strcmp(fam, "gravel") == 0) {
                u32 id;
                const f32 d = worley(u, v, 16, s, &id);
                const f32 stone = clamp(1.0f - d * 2.2f, 0.0f, 1.0f);
                const f32 tint = 0.75f + 0.5f * (static_cast<f32>(id % 512u) / 511.0f);
                h = 0.25f + 0.7f * stone;
                alb = Rgb{1.0f, 1.0f, 1.0f} * tint * (0.72f + 0.5f * stone);
                rough = 0.88f + 0.10f * (1.0f - stone);
                ao = 0.55f + 0.45f * stone;
            } else if (std::strcmp(fam, "dirt") == 0 || std::strcmp(fam, "earth") == 0) {
                const f32 blotch = fbm(u, v, 5, 4, 0.55f, s);
                const f32 grain = fbm(u, v, 32, 4, 0.5f, s ^ 0x11ull);
                h = 0.45f + 0.35f * grain + 0.1f * blotch;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.82f + 0.34f * blotch) * (0.94f + 0.12f * grain);
                rough = 0.90f + 0.08f * grain;
                ao = 0.88f + 0.12f * grain;
            } else if (std::strcmp(fam, "rock") == 0) {
                const f32 fissure = ridged(u, v, 5, 5, s);
                const f32 grain = fbm(u, v, 24, 4, 0.5f, s ^ 0x21ull);
                h = 0.3f + 0.6f * fissure;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.70f + 0.55f * fissure) * (0.93f + 0.14f * grain);
                rough = 0.84f + 0.12f * grain;
                ao = 0.60f + 0.40f * fissure;
            } else if (std::strcmp(fam, "sand") == 0) {
                const f32 ripple = 0.5f + 0.5f * std::sin((u * 6.0f + fbm(u, v, 4, 3, 0.5f, s) * 2.0f) * 6.2831853f);
                const f32 grain = fbm(u, v, 48, 3, 0.5f, s ^ 0x41ull);
                h = 0.4f + 0.25f * ripple + 0.2f * grain;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.90f + 0.18f * ripple) * (0.96f + 0.08f * grain);
                rough = 0.86f + 0.10f * grain;
                ao = 0.92f + 0.08f * ripple;
            } else if (std::strcmp(fam, "wetland_mud") == 0) {
                const f32 pool = fbm(u, v, 4, 4, 0.55f, s);
                const f32 crack = ridged(u, v, 12, 3, s ^ 0x61ull);
                h = 0.35f + 0.3f * pool + 0.2f * crack;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.70f + 0.5f * pool) * (0.9f + 0.2f * crack);
                rough = 0.55f + 0.35f * crack;   // wet mud is *smoother* than dry
                ao = 0.85f + 0.15f * pool;
            } else if (std::strcmp(fam, "reclamation_fill") == 0) {
                u32 id;
                const f32 d = worley(u, v, 10, s, &id);
                const f32 rubble = clamp(1.0f - d * 2.6f, 0.0f, 1.0f);
                const f32 tint = 0.78f + 0.44f * (static_cast<f32>(id % 256u) / 255.0f);
                const f32 dirt = fbm(u, v, 12, 3, 0.5f, s ^ 0x71ull);
                h = 0.3f + 0.6f * rubble + 0.1f * dirt;
                alb = Rgb{1.0f, 1.0f, 1.0f} * tint * (0.80f + 0.35f * rubble);
                rough = 0.88f + 0.10f * dirt;
                ao = 0.62f + 0.38f * rubble;
            } else if (std::strcmp(fam, "grass") == 0) {
                const f32 clump = fbm(u, v, 8, 4, 0.55f, s);
                const f32 blade = ridged(u, v, 48, 3, s ^ 0x81ull);
                const f32 patch = fbm(u, v, 3, 3, 0.5f, s ^ 0x91ull);
                h = 0.35f + 0.4f * clump + 0.25f * blade;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.80f + 0.4f * clump) * (0.82f + 0.36f * patch) *
                      (0.92f + 0.16f * blade);
                rough = 0.90f + 0.08f * blade;
                ao = 0.80f + 0.20f * clump;
            } else if (std::strcmp(fam, "plaster_painted") == 0) {
                const f32 render = fbm(u, v, 16, 4, 0.5f, s);
                const f32 crack = ridged(u, v, 5, 4, s ^ 0xA1ull);
                const f32 crackMask = 1.0f - smoothstep(0.0f, 1.0f, clamp((crack - 0.80f) * 8.0f, 0.0f, 1.0f));
                const f32 stain = fbm(u, v, 3, 3, 0.5f, s ^ 0xB1ull);
                h = 0.6f + 0.2f * render - 0.35f * (1.0f - crackMask);
                // Paint is thinner at the bottom of a wall: the vertical ramp is
                // the water stain every Mumbai facade has.
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.93f + 0.10f * render) *
                      (1.0f - 0.10f * (1.0f - stain)) * (1.0f - 0.08f * (1.0f - v));
                rough = 0.80f + 0.14f * render;
                ao = 1.0f - 0.20f * (1.0f - crackMask);
            } else if (std::strcmp(fam, "stone_ashlar") == 0) {
                const f32 rows = 6.0f;
                const f32 rv = v * rows;
                const i64 row = static_cast<i64>(rv);
                const f32 fv = rv - static_cast<f32>(row);
                // Running bond: alternate courses shift by half a block.
                const f32 cols = 4.0f;
                const f32 ru = u * cols + (row % 2 ? 0.5f : 0.0f);
                const i64 col = static_cast<i64>(ru);
                const f32 fu = ru - static_cast<f32>(col);
                const f32 eu = std::min(fu, 1.0f - fu), ev = std::min(fv, 1.0f - fv);
                const f32 joint = 1.0f - smoothstep(0.0f, 1.0f, clamp((std::min(eu, ev) - 0.045f) * 22.0f, 0.0f, 1.0f));
                u32 id;
                worley(u, v, 4, s, &id);
                const f32 tint = 0.90f + 0.18f * (static_cast<f32>(id % 64u) / 63.0f);
                const f32 grain = fbm(u, v, 40, 3, 0.5f, s ^ 0xC1ull);
                h = 0.74f - 0.5f * joint + 0.05f * grain;
                alb = Rgb{1.0f, 1.0f, 1.0f} * tint * (0.94f + 0.10f * grain) * (1.0f - 0.22f * joint);
                rough = 0.76f + 0.14f * grain;
                ao = 1.0f - 0.32f * joint;
            } else if (std::strcmp(fam, "stone_rubble") == 0) {
                u32 id;
                const f32 d = worley(u, v, 9, s, &id);
                const f32 stone = clamp(1.0f - d * 2.0f, 0.0f, 1.0f);
                const f32 tint = 0.80f + 0.34f * (static_cast<f32>(id % 128u) / 127.0f);
                const f32 grain = fbm(u, v, 32, 3, 0.5f, s ^ 0xD1ull);
                h = 0.25f + 0.65f * stone;
                alb = Rgb{1.0f, 1.0f, 1.0f} * tint * (0.78f + 0.36f * stone) * (0.94f + 0.12f * grain);
                rough = 0.84f + 0.12f * grain;
                ao = 0.58f + 0.42f * stone;
            } else if (std::strcmp(fam, "curtain_wall_glass") == 0) {
                // Optically smooth: no grain, no normal.  What a curtain wall does
                // have is a mullion grid, and that is real geometry on the building,
                // not a relief in the glass.  So the tile stays flat and the mean
                // albedo is carried through unchanged.
                h = 0.5f;
                alb = Rgb{1.0f, 1.0f, 1.0f};
                rough = 1.0f;
                ao = 1.0f;
            } else if (std::strcmp(fam, "brick_exposed") == 0) {
                const f32 rows = 12.0f;                     // ~one brick per tile/12
                const f32 rv = v * rows;
                const i64 row = static_cast<i64>(rv);
                const f32 fv = rv - static_cast<f32>(row);
                const f32 cols = 6.0f;
                const f32 ru = u * cols + (row % 2 ? 0.5f : 0.0f);
                const i64 col = static_cast<i64>(ru);
                const f32 fu = ru - static_cast<f32>(col);
                const f32 eu = std::min(fu, 1.0f - fu), ev = std::min(fv, 1.0f - fv);
                const f32 joint = 1.0f - smoothstep(0.0f, 1.0f, clamp((std::min(eu, ev) - 0.06f) * 16.0f, 0.0f, 1.0f));
                const u32 bid = static_cast<u32>(mix64(static_cast<u64>(col) * 73856093ull ^
                                                       static_cast<u64>(row) * 19349663ull) >> 40);
                const f32 tint = 0.86f + 0.28f * (static_cast<f32>(bid % 128u) / 127.0f);
                const f32 grain = fbm(u, v, 40, 3, 0.5f, s ^ 0xE1ull);
                h = 0.78f - 0.55f * joint + 0.05f * grain;
                // Mortar is greyer and smoother than the brick it holds.
                const Rgb brick = Rgb{1.0f, 1.0f, 1.0f} * tint * (0.94f + 0.10f * grain);
                const Rgb mortar = Rgb{1.35f, 1.35f, 1.30f} * (0.90f + 0.14f * grain);   // rel. to brick albedo
                alb = mix(brick, mortar, joint);
                rough = lerp(0.80f + 0.12f * grain, 0.90f, joint);
                ao = 1.0f - 0.30f * joint;
            } else if (std::strcmp(fam, "corrugated_metal") == 0) {
                const f32 ridge = std::sin(u * 6.2831853f * 10.0f);
                const f32 rust = fbm(u, v, 6, 4, 0.55f, s ^ 0xF1ull);
                const f32 rustMask = smoothstep(0.45f, 0.75f, rust);
                h = 0.5f + 0.45f * ridge;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.92f + 0.16f * ridge) *
                      mix(Rgb{1, 1, 1}, Rgb{1.6f, 0.85f, 0.55f}, rustMask * 0.55f);
                rough = lerp(0.55f, 0.88f, rustMask) + 0.05f * (1.0f - ridge);
                ao = 0.85f + 0.15f * (0.5f + 0.5f * ridge);
            } else if (std::strcmp(fam, "concrete_raw") == 0) {
                const f32 board = std::fabs(std::fmod(v * 8.0f, 1.0f) - 0.5f);   // formwork marks
                const f32 formMask = 1.0f - smoothstep(0.0f, 1.0f, clamp((board - 0.46f) * 24.0f, 0.0f, 1.0f));
                const f32 grain = fbm(u, v, 20, 4, 0.5f, s);
                const f32 stain = fbm(u, v, 3, 3, 0.5f, s ^ 0x101ull);
                const f32 blow = smoothstep(0.68f, 0.88f, fbm(u, v, 7, 3, 0.5f, s ^ 0x202ull));
                h = 0.55f + 0.25f * grain - 0.18f * formMask - 0.25f * blow;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.92f + 0.14f * grain) *
                      (1.0f - 0.12f * (1.0f - stain)) * (1.0f - 0.30f * blow);
                rough = 0.82f + 0.12f * grain;
                ao = 1.0f - 0.20f * formMask - 0.30f * blow;
            } else if (std::strcmp(fam, "kerb_concrete") == 0 ||
                       std::strcmp(fam, "sidewalk_concrete") == 0) {
                const bool kerb = std::strcmp(fam, "kerb_concrete") == 0;
                const f32 grain = fbm(u, v, 28, 4, 0.5f, s);
                const f32 jointU = std::fabs(std::fmod(u * (kerb ? 2.0f : 1.0f), 1.0f) - 0.5f);
                const f32 jm = 1.0f - smoothstep(0.0f, 1.0f, clamp((0.5f - jointU - 0.47f) * 30.0f, 0.0f, 1.0f));
                // A hundred and twenty years of paan stains and footfall.
                const f32 wear = fbm(u, v, 5, 4, 0.55f, s ^ 0x303ull);
                h = 0.5f + 0.3f * grain - 0.25f * jm;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.90f + 0.16f * grain) * (0.86f + 0.26f * wear) *
                      (1.0f - 0.15f * jm);
                rough = 0.86f + 0.10f * grain;
                ao = 1.0f - 0.30f * jm;
            } else if (std::strcmp(fam, "median_barrier") == 0) {
                const f32 paint = fbm(u, v, 12, 3, 0.5f, s);
                const f32 chip = smoothstep(0.72f, 0.92f, fbm(u, v, 20, 3, 0.5f, s ^ 0x404ull));
                const f32 grime = fbm(u, v, 4, 3, 0.5f, s ^ 0x505ull);
                h = 0.6f + 0.2f * paint - 0.3f * chip;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.10f * paint) * (1.0f - 0.22f * grime) *
                      lerp(1.0f, 0.75f, chip);
                rough = 0.72f + 0.22f * chip + 0.08f * paint;
                ao = 0.92f + 0.08f * paint;
            } else if (std::strcmp(fam, "water_surface") == 0 || std::strcmp(fam, "inland_water") == 0) {
                // The ocean's real displacement is Gerstner geometry; what the map
                // contributes is the wind chop finer than any wave mesh resolves.
                const bool sea = std::strcmp(fam, "water_surface") == 0;
                const f32 w1 = std::sin((u * 4.0f + v * 1.0f) * 6.2831853f);
                const f32 w2 = std::sin((u * 1.0f - v * 3.0f) * 6.2831853f + 1.3f);
                const f32 chop = fbm(u, v, 16, 4, 0.55f, s);
                h = 0.5f + 0.25f * (w1 * 0.5f + w2 * 0.5f) + 0.25f * (chop - 0.5f);
                alb = Rgb{1.0f, 1.0f, 1.0f} * (sea ? 0.98f : 0.92f);
                rough = sea ? 1.0f : 1.05f;      // mean roughness is the MaterialDef's
                ao = 1.0f;
            } else if (std::strcmp(fam, "rail_ballast") == 0) {
                u32 id;
                const f32 d = worley(u, v, 20, s, &id);
                const f32 stone = clamp(1.0f - d * 2.4f, 0.0f, 1.0f);
                const f32 grime = fbm(u, v, 5, 3, 0.5f, s ^ 0x606ull);
                h = 0.2f + 0.75f * stone;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.68f + 0.5f * stone) * (0.88f + 0.2f * grime);
                rough = 0.90f + 0.08f * grime;
                ao = 0.5f + 0.5f * stone;
            } else if (std::strcmp(fam, "rail_steel") == 0) {
                const f32 streak = fbm(u, v * 0.25f, 24, 3, 0.5f, s);
                h = 0.5f + 0.1f * streak;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * streak);
                rough = 1.0f;
                ao = 1.0f;
            } else if (std::strcmp(fam, "rail_sleeper") == 0) {
                const f32 grain = ridged(u, v * 0.35f, 20, 3, s);
                const f32 rot = fbm(u, v, 8, 3, 0.5f, s ^ 0x707ull);
                h = 0.45f + 0.3f * grain;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.85f + 0.28f * grain) * (0.92f + 0.16f * rot);
                rough = 0.88f + 0.10f * grain;
                ao = 0.85f + 0.15f * grain;
            } else if (std::strcmp(fam, "water_tank_plastic") == 0) {
                const f32 sheen = fbm(u, v, 10, 3, 0.5f, s);
                const f32 scuff = ridged(u, v, 30, 2, s ^ 0x808ull);
                h = 0.5f + 0.12f * sheen;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * sheen) * (0.95f + 0.10f * scuff);
                rough = 0.55f + 0.25f * sheen;
                ao = 0.95f + 0.05f * sheen;
            } else if (std::strcmp(fam, "ac_unit") == 0) {
                const f32 louvre = 0.5f + 0.5f * std::sin(v * 6.2831853f * 8.0f);
                const f32 dust = fbm(u, v, 12, 3, 0.5f, s ^ 0x909ull);
                h = 0.35f + 0.5f * louvre;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * louvre) * (0.94f + 0.12f * dust);
                rough = 0.55f + 0.20f * dust;
                ao = 0.78f + 0.22f * louvre;
            } else if (std::strcmp(fam, "metal_railing") == 0) {
                const f32 rust = fbm(u, v, 7, 4, 0.55f, s ^ 0xA0Aull);
                const f32 rustMask = smoothstep(0.55f, 0.85f, rust);
                const f32 scratch = ridged(u, v, 40, 2, s ^ 0xB0Bull);
                h = 0.5f + 0.08f * scratch;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.95f + 0.10f * scratch) *
                      mix(Rgb{1, 1, 1}, Rgb{1.45f, 0.80f, 0.5f}, rustMask * 0.6f);
                rough = lerp(0.55f, 0.90f, rustMask);
                ao = 0.94f + 0.06f * scratch;
            } else if (std::strcmp(fam, "signage_painted") == 0) {
                const f32 board = fbm(u, v, 14, 3, 0.5f, s);
                const f32 peel = smoothstep(0.70f, 0.90f, fbm(u, v, 4, 4, 0.55f, s ^ 0xC0C0ull));
                h = 0.6f - 0.25f * peel + 0.1f * board;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * board) * lerp(1.0f, 0.80f, peel);
                rough = 0.70f + 0.25f * peel;
                ao = 1.0f - 0.15f * peel;
            } else if (std::strcmp(fam, "tree_foliage") == 0) {
                const f32 leaf = ridged(u, v, 24, 4, s);
                const f32 clump = fbm(u, v, 5, 4, 0.55f, s ^ 0xD0D0ull);
                const f32 gap = 1.0f - smoothstep(0.55f, 0.95f, fbm(u, v, 16, 4, 0.55f, s ^ 0xE0E0ull));
                h = 0.4f + 0.45f * leaf;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.70f + 0.6f * leaf) * (0.82f + 0.36f * clump);
                rough = 0.88f + 0.10f * leaf;
                ao = 0.45f + 0.55f * (1.0f - gap);
            } else if (std::strcmp(fam, "tree_bark") == 0) {
                const f32 ridge = ridged(u * 0.5f, v, 12, 4, s);
                const f32 moss = fbm(u, v, 4, 3, 0.5f, s ^ 0xF0F0ull);
                h = 0.35f + 0.55f * ridge;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.72f + 0.5f * ridge) * (0.90f + 0.2f * moss);
                rough = 0.90f + 0.08f * ridge;
                ao = 0.55f + 0.45f * ridge;
            } else if (std::strcmp(fam, "streetlamp_pole") == 0) {
                const f32 grime = fbm(u, v, 8, 4, 0.55f, s ^ 0x111ull);
                const f32 chip = smoothstep(0.75f, 0.95f, fbm(u, v, 24, 3, 0.5f, s ^ 0x222ull));
                h = 0.55f + 0.1f * grime - 0.2f * chip;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.10f * grime) * lerp(1.0f, 0.78f, chip);
                rough = 0.62f + 0.25f * chip;
                ao = 0.96f;
            } else if (std::strcmp(fam, "streetlamp_glow") == 0) {
                h = 0.5f;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.96f + 0.08f * fbm(u, v, 6, 2, 0.5f, s));
                rough = 1.0f;
                ao = 1.0f;
            } else if (std::strcmp(fam, "hero_stone") == 0) {
                // Carved stone: a quarried grain with tool marks, no ashlar grid —
                // the landmark assets carry their own carved geometry (B1).
                const f32 grain = fbm(u, v, 40, 4, 0.5f, s);
                const f32 tool = ridged(u, v * 0.6f, 16, 3, s ^ 0x333ull);
                const f32 weather = fbm(u, v, 4, 4, 0.55f, s ^ 0x444ull);
                h = 0.5f + 0.25f * grain + 0.15f * tool;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.90f + 0.20f * grain) * (0.88f + 0.22f * weather) *
                      (0.95f + 0.10f * tool);
                rough = 0.78f + 0.16f * grain;
                ao = 0.88f + 0.12f * grain;
            } else if (std::strcmp(fam, "hero_terracotta") == 0) {
                const f32 tile = ridged(u, v, 18, 3, s);
                const f32 grain = fbm(u, v, 30, 3, 0.5f, s ^ 0x555ull);
                h = 0.45f + 0.35f * tile;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.86f + 0.26f * tile) * (0.94f + 0.12f * grain);
                rough = 0.84f + 0.12f * grain;
                ao = 0.82f + 0.18f * tile;
            } else if (std::strcmp(fam, "hero_concrete") == 0) {
                const f32 grain = fbm(u, v, 24, 4, 0.5f, s);
                const f32 stain = fbm(u, v, 3, 3, 0.5f, s ^ 0x666ull);
                h = 0.55f + 0.2f * grain;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * grain) * (0.90f + 0.18f * stain);
                rough = 0.80f + 0.12f * grain;
                ao = 0.94f + 0.06f * grain;
            } else if (std::strcmp(fam, "sea_link_pylon") == 0) {
                const f32 grain = fbm(u, v, 32, 4, 0.5f, s);
                const f32 lift = std::fabs(std::fmod(v * 4.0f, 1.0f) - 0.5f);
                const f32 line = 1.0f - smoothstep(0.0f, 1.0f, clamp((lift - 0.46f) * 30.0f, 0.0f, 1.0f));
                h = 0.56f + 0.2f * grain - 0.12f * line;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.94f + 0.12f * grain) * (1.0f - 0.08f * line);
                rough = 0.74f + 0.14f * grain;
                ao = 1.0f - 0.12f * line;
            } else if (std::strcmp(fam, "sea_link_cable") == 0) {
                // A stay is a bundle of strands, not a rod: the helical weave is the
                // reason a cable catches the sun in bands.
                const f32 strand = 0.5f + 0.5f * std::sin((u * 6.0f + v * 18.0f) * 6.2831853f);
                const f32 weave = 0.5f + 0.5f * std::sin((u * 18.0f - v * 6.0f) * 6.2831853f);
                h = 0.4f + 0.3f * strand + 0.2f * weave;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.90f + 0.16f * strand);
                rough = 0.55f + 0.25f * (1.0f - strand);
                ao = 0.70f + 0.30f * strand;
            } else {
                const f32 grain = fbm(u, v, 16, 4, 0.5f, s);
                h = 0.4f + 0.4f * grain;
                alb = Rgb{1.0f, 1.0f, 1.0f} * (0.88f + 0.24f * grain);
                rough = 0.80f + 0.14f * grain;
                ao = 0.90f + 0.10f * grain;
            }

            t.height[i] = clamp(h, 0.0f, 1.0f);
            t.albedo[i * 3 + 0] = alb.r;
            t.albedo[i * 3 + 1] = alb.g;
            t.albedo[i * 3 + 2] = alb.b;
            t.rough[i] = clamp(rough, 0.0f, 2.0f);
            t.ao[i] = clamp(ao, 0.0f, 1.0f);
        }
    }

    // --- mean-preserving normalisation --------------------------------------
    // The recipe produces *relative* albedo and roughness; the physical values
    // come from the MaterialDef.  Dividing by the mean and multiplying by the
    // authored value is what makes a texture a detail layer rather than a change
    // of material: the average albedo of the tile is exactly baseColor and the
    // average roughness is exactly the defined roughness, so the calibration the
    // wetness response and the tonemap were verified against still holds.
    f64 mr = 0.0, mg = 0.0, mb = 0.0, mrough = 0.0;
    const size_t n = static_cast<size_t>(N) * N;
    for (size_t i = 0; i < n; ++i) {
        mr += t.albedo[i * 3 + 0];
        mg += t.albedo[i * 3 + 1];
        mb += t.albedo[i * 3 + 2];
        mrough += t.rough[i];
    }
    mr /= n; mg /= n; mb /= n; mrough /= n;
    if (mr < 1e-6) mr = 1e-6;
    if (mg < 1e-6) mg = 1e-6;
    if (mb < 1e-6) mb = 1e-6;
    if (mrough < 1e-6) mrough = 1e-6;

    TextureSet set;
    set.name = name;
    set.tileMetres = tileMetresFor(name);
    set.baseColor.resize(n * 4);
    set.normal.resize(n * 3);
    set.orm.resize(n * 3);

    const bool smooth = isSmoothFamily(fam);
    // Roughness variation is bounded by the authored value on both sides: a
    // mirror cannot get smoother than a mirror, and a matte surface cannot get
    // rougher than fully diffuse.
    const f32 roughSpread = std::min({0.30f, roughness * 0.5f, (1.0f - roughness) * 0.8f + 0.05f});

    for (u32 y = 0; y < N; ++y) {
        for (u32 x = 0; x < N; ++x) {
            const u32 i = t.index(x, y);
            const size_t p = i;
            // base colour: relative albedo -> authored mean, then to sRGB
            const f32 r = t.albedo[p * 3 + 0] / static_cast<f32>(mr) * baseColor[0];
            const f32 g = t.albedo[p * 3 + 1] / static_cast<f32>(mg) * baseColor[1];
            const f32 b = t.albedo[p * 3 + 2] / static_cast<f32>(mb) * baseColor[2];
            set.baseColor[p * 4 + 0] = static_cast<u8>(linearToSrgb(r) * 255.0f + 0.5f);
            set.baseColor[p * 4 + 1] = static_cast<u8>(linearToSrgb(g) * 255.0f + 0.5f);
            set.baseColor[p * 4 + 2] = static_cast<u8>(linearToSrgb(b) * 255.0f + 0.5f);
            set.baseColor[p * 4 + 3] = 255;

            // roughness: relative -> authored mean, absolute in the green channel
            const f32 rel = t.rough[p] / static_cast<f32>(mrough);
            const f32 rough = clamp(roughness * (1.0f + roughSpread * (rel - 1.0f)), 0.0f, 1.0f);
            const f32 aoTex = clamp(1.0f - 0.15f * (1.0f - t.ao[p]), 0.0f, 1.0f);
            set.orm[p * 3 + 0] = static_cast<u8>(aoTex * 255.0f + 0.5f);
            set.orm[p * 3 + 1] = static_cast<u8>(rough * 255.0f + 0.5f);
            set.orm[p * 3 + 2] = static_cast<u8>(clamp(metallic, 0.0f, 1.0f) * 255.0f + 0.5f);
        }
    }

    // --- normal map from the height field -----------------------------------
    // Central differences with wrap-around, so the tangent-space normal is
    // seamless across the tile edge exactly as the albedo is.  The bump strength
    // is derived from the physical tile size: one height unit of 1.0 over a 1 m
    // tile is a much steeper surface than over an 8 m tile, and the relief has to
    // stay plausible in metres rather than in texels.
    const f32 reliefMetres = 0.008f * set.tileMetres;      // ~8 mm of relief per metre of tile
    const f32 texelMetres = set.tileMetres / static_cast<f32>(N);
    // Optically smooth families keep a flat normal: a mirror-flat curtain wall has
    // no grain to resolve, and inventing one would put a fake bump on every glass
    // facade in the city.  Standing water is the exception — a dead-flat 12 m
    // water tile reads as painted glass, and the mesh has no waves of its own
    // (the Gerstner ocean is a separate, GPU-side system for the open sea), so
    // inland water gets a quarter-strength ripple instead.
    const bool rippled = std::strcmp(fam, "water_surface") == 0 ||
                         std::strcmp(fam, "inland_water") == 0;
    const f32 slopeScale = (smooth ? (rippled ? 0.25f : 0.0f) : 1.0f) *
                           reliefMetres / std::max(1e-4f, texelMetres);
    const f32 nz = 1.0f;
    for (u32 y = 0; y < N; ++y) {
        for (u32 x = 0; x < N; ++x) {
            const u32 xm = (x + N - 1) % N, xp = (x + 1) % N;
            const u32 ym = (y + N - 1) % N, yp = (y + 1) % N;
            const f32 hl = t.height[t.index(xm, y)], hr = t.height[t.index(xp, y)];
            const f32 hd = t.height[t.index(x, ym)], hu = t.height[t.index(x, yp)];
            const f32 du = (hr - hl) * 0.5f * slopeScale;
            const f32 dv = (hu - hd) * 0.5f * slopeScale;
            // Tangent space: +U right, +V up (the mesh builder's V runs with
            // height on walls), normal = normalize(-du, +dv, 1).
            f32 nx = -du, ny = dv, nzv = nz;
            const f32 len = std::sqrt(nx * nx + ny * ny + nzv * nzv);
            nx /= len; ny /= len; nzv /= len;
            const size_t p = t.index(x, y);
            set.normal[p * 3 + 0] = static_cast<u8>(clamp(nx * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f + 0.5f);
            set.normal[p * 3 + 1] = static_cast<u8>(clamp(ny * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f + 0.5f);
            set.normal[p * 3 + 2] = static_cast<u8>(clamp(nzv * 0.5f + 0.5f, 0.0f, 1.0f) * 255.0f + 0.5f);
        }
    }
    return set;
}

// -----------------------------------------------------------------------------
//  Mip chains
// -----------------------------------------------------------------------------
void appendMipChain(std::vector<u8>& out, const u8* pixels, u32 size, u32 channels) {
    out.insert(out.end(), pixels, pixels + static_cast<size_t>(size) * size * channels);
    u32 w = size;
    std::vector<u8> src(pixels, pixels + static_cast<size_t>(size) * size * channels);
    while (w > 1) {
        const u32 nw = std::max(1u, w / 2);
        std::vector<u8> dst(static_cast<size_t>(nw) * nw * channels, 0);
        for (u32 y = 0; y < nw; ++y) {
            for (u32 x = 0; x < nw; ++x) {
                for (u32 c = 0; c < channels; ++c) {
                    // A box filter that handles odd sizes by clamping: a 1-pixel
                    // tail is replicated rather than dropped, which is what keeps
                    // a thin kerb line or a mullion from disappearing two mips down.
                    auto at = [&](u32 sx, u32 sy) {
                        const u32 cx = std::min(sx, w - 1), cy = std::min(sy, w - 1);
                        return static_cast<u32>(src[(static_cast<size_t>(cy) * w + cx) * channels + c]);
                    };
                    const u32 sum = at(x * 2, y * 2) + at(x * 2 + 1, y * 2) +
                                    at(x * 2, y * 2 + 1) + at(x * 2 + 1, y * 2 + 1);
                    dst[(static_cast<size_t>(y) * nw + x) * channels + c] =
                        static_cast<u8>((sum + 2) / 4);
                }
            }
        }
        out.insert(out.end(), dst.begin(), dst.end());
        src.swap(dst);
        w = nw;
    }
}

bool writeTexArray(const std::string& path, MapType type, u32 size,
                   const std::vector<LayerInput>& layers) {
    const u32 channels = mapTypeChannels(type);
    const u32 layerCount = static_cast<u32>(layers.size());
    if (layerCount == 0 || channels == 0) return false;
    const u32 mipCount = mipCountFor(size);

    TexArrayHeader h{};
    std::memcpy(h.magic, TEXARRAY_MAGIC, 8);
    h.version = TEXARRAY_VERSION;
    h.mapType = static_cast<u32>(type);
    h.layerCount = layerCount;
    h.size = size;
    h.mipCount = mipCount;
    h.channels = channels;

    std::vector<TexLayer> records(layerCount);
    std::vector<std::vector<u8>> payloads(layerCount);
    u64 offset = TEXARRAY_HEADER_SIZE + static_cast<u64>(TEXLAYER_SIZE) * layerCount;
    u64 contentHash = 1469598103934665603ull;
    for (u32 i = 0; i < layerCount; ++i) {
        if (!layers[i].pixels || layers[i].pixels->size() < static_cast<size_t>(size) * size * channels)
            return false;
        appendMipChain(payloads[i], layers[i].pixels->data(), size, channels);
        // Each layer starts on an 8-byte boundary.  A mip chain is not a whole
        // number of words (a 3-channel 1x1 tail is 3 bytes), so without the pad a
        // later layer would begin mid-word and any path that uploads a layer
        // straight out of the mapping — a Vulkan buffer-to-image copy, a
        // glTexImage — would need a staging copy first.  At most 7 bytes per
        // layer, not counted in the recorded size.
        offset = (offset + 7u) & ~static_cast<u64>(7u);
        records[i].offset = offset;
        records[i].size = payloads[i].size();
        records[i].hash = fnv1a64(payloads[i].data(), payloads[i].size());
        // What the renderer has to know about the bytes: the mean *linear* albedo
        // (its divisor, so the tile modulates colour with mean 1) and the physical
        // size of one tile in metres.  Measured here, from the pixels, so the
        // record cannot claim something the content does not do.
        meanLinearRgb(layers[i].pixels->data(), size, channels, &records[i].meanR);
        records[i].tileMetres = layers[i].tileMetres > 0.0f ? layers[i].tileMetres : 1.0f;
        offset += payloads[i].size();
        contentHash = fnv1a64(payloads[i].data(), payloads[i].size(), contentHash);
    }
    u64 payloadTotal = 0;
    for (const std::vector<u8>& p : payloads) payloadTotal += p.size();
    h.payloadBytes = payloadTotal;      // recorded sizes only, never the alignment pad
    h.contentHash = contentHash;

    std::vector<u8> file;
    file.resize(static_cast<size_t>(offset), 0);
    std::memcpy(file.data(), &h, sizeof(h));
    std::memcpy(file.data() + TEXARRAY_HEADER_SIZE, records.data(),
                sizeof(TexLayer) * layerCount);
    for (u32 i = 0; i < layerCount; ++i)
        std::memcpy(file.data() + records[i].offset, payloads[i].data(), payloads[i].size());
    return writeFile(path, file.data(), file.size());
}

// -----------------------------------------------------------------------------
//  PNG (stored deflate).  A review format: no compressor, no dependency, and
//  byte-identical across machines, which matters more here than the bytes saved.
// -----------------------------------------------------------------------------
namespace {
u32 crcTableEntry(u32 n) {
    u32 c = n;
    for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
    return c;
}
const u32* crcTable() {
    static u32 table[256];
    static bool init = false;
    if (!init) { for (u32 n = 0; n < 256; ++n) table[n] = crcTableEntry(n); init = true; }
    return table;
}
u32 crc32(const u8* data, size_t len) {
    const u32* t = crcTable();
    u32 c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) c = t[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}
u32 adler32(const u8* data, size_t len) {
    u32 a = 1, b = 0;
    for (size_t i = 0; i < len; ++i) { a = (a + data[i]) % 65521u; b = (b + a) % 65521u; }
    return (b << 16) | a;
}
void be32(std::vector<u8>& out, u32 v) {
    out.push_back(static_cast<u8>(v >> 24)); out.push_back(static_cast<u8>(v >> 16));
    out.push_back(static_cast<u8>(v >> 8));  out.push_back(static_cast<u8>(v));
}
void chunk(std::vector<u8>& out, const char type[4], const std::vector<u8>& body) {
    be32(out, static_cast<u32>(body.size()));
    const size_t start = out.size();
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), body.begin(), body.end());
    be32(out, crc32(out.data() + start, out.size() - start));
}
}  // namespace

// -----------------------------------------------------------------------------
//  A real deflate encoder (fixed Huffman + greedy LZ77)
// -----------------------------------------------------------------------------
// The project's dependency policy (D-014) forbids a compression library, and a
// *stored* PNG — valid, but a third of the size it should be — is not something a
// content pipeline can ship: the material review sheets are megabytes of mostly
// flat colour, which is exactly what LZ77 was invented for.  This is the other
// half of deflate: RFC 1951 fixed Huffman codes, a 32 KiB window, a three-byte
// hash chain and greedy matching.  It is deterministic, it is verified by
// decoding every stream it produces in the test suite, and it runs at build time.
namespace {

class BitWriter {
public:
    void put(u32 bits, u32 count) {          // LSB-first (extra bits, headers)
        for (u32 i = 0; i < count; ++i) {
            m_bit |= ((bits >> i) & 1u) << m_used;
            if (++m_used == 8) { m_out.push_back(static_cast<u8>(m_bit)); m_bit = 0; m_used = 0; }
        }
    }
    void putCode(u32 code, u32 bits) {       // MSB-first (Huffman codes)
        for (u32 i = 0; i < bits; ++i)
            put((code >> (bits - 1 - i)) & 1u, 1);
    }
    void finish() { if (m_used) { m_out.push_back(static_cast<u8>(m_bit)); m_bit = 0; m_used = 0; } }
    std::vector<u8> take() { finish(); return std::move(m_out); }
private:
    std::vector<u8> m_out;
    u8 m_bit = 0;
    u32 m_used = 0;
};

// RFC 1951 fixed literal/length code for a symbol (256 = end of block).
void fixedLitCode(u32 sym, u32& code, u32& bits) {
    if (sym <= 143)      { code = 0x30u + sym;          bits = 8; }
    else if (sym <= 255) { code = 0x190u + (sym - 144); bits = 9; }
    else if (sym <= 279) { code = 0x0u + (sym - 256);   bits = 7; }
    else                 { code = 0xC0u + (sym - 280);  bits = 8; }
}

const u16 kLenBase[29] = {3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,
                          131,163,195,227,258};
const u8 kLenExtra[29] = {0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0};
const u16 kDistBase[30] = {1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,
                           1537,2049,3073,4097,6145,8193,12289,16385,24577};
const u8 kDistExtra[30] = {0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13};

// The largest table index whose base does not exceed `v`.  Written as an
// explicit descending loop because the idiomatic `for (i = N; i-- > 0;)` runs its
// body with i already decremented — it silently skips the top index, which for
// distances meant every match farther than 24,577 m was encoded one class too
// low and its extra bits were truncated.  It decoded to the wrong bytes, not to
// an error, and only a >24 KB repetitive input ever exercised it.
u32 baseIndex(u32 v, const u16* base, u32 top) {
    for (u32 i = top; i > 0; --i) if (v >= base[i]) return i;
    return 0;
}
u32 lengthIndex(u32 len) { return baseIndex(len, kLenBase, 28); }
u32 lengthSymbol(u32 len) { return 257 + lengthIndex(len); }
u32 distIndex(u32 dist) { return baseIndex(dist, kDistBase, 29); }

std::vector<u8> deflateFixed(const std::vector<u8>& src) {
    BitWriter bw;
    bw.put(1, 1);          // BFINAL
    bw.put(1, 2);          // BTYPE = 01, fixed Huffman
    constexpr u32 kWindow = 32768;
    constexpr u32 kMaxMatch = 258;
    constexpr u32 kMinMatch = 3;
    constexpr u32 kHashBits = 15;
    constexpr u32 kHashSize = 1u << kHashBits;
    constexpr u32 kMaxChain = 48;

    const size_t n = src.size();
    // `prev[p]` is the previous position whose three bytes hash the same as p's,
    // i.e. the LZ77 chain.  It is indexed by position and padded to the full
    // input length, because a chain whose indices drift by even one byte produces
    // a stream that still decodes — into the wrong bytes.
    std::vector<i32> head(kHashSize, -1);
    std::vector<i32> prev(n, -1);
    auto hash3 = [&](size_t i) {
        const u32 a = src[i], b = src[i + 1], c = src[i + 2];
        return ((a * 0x9E3779B1u) ^ (b * 0x85EBCA77u) ^ (c * 0xC2B2AE3Du)) >> (32 - kHashBits);
    };
    auto registerPos = [&](size_t pos) {
        if (pos + kMinMatch > n) return;
        const u32 h = hash3(pos);
        prev[pos] = head[h];
        head[h] = static_cast<i32>(pos);
    };

    size_t i = 0;
    while (i < n) {
        u32 bestLen = 0, bestDist = 0;
        if (i + kMinMatch <= n) {
            const u32 h = hash3(i);
            i32 cand = head[h];
            u32 chain = 0;
            while (cand >= 0 && chain++ < kMaxChain) {
                const size_t d = i - static_cast<size_t>(cand);
                if (d == 0 || d > kWindow) break;
                if (src[cand + bestLen] == src[i + bestLen]) {
                    u32 len = 0;
                    while (len < kMaxMatch && i + len < n && src[cand + len] == src[i + len]) ++len;
                    if (len > bestLen) { bestLen = len; bestDist = static_cast<u32>(d); }
                }
                cand = prev[cand];
            }
        }
        if (bestLen >= kMinMatch) {
            const u32 li = lengthIndex(bestLen);
            u32 code, bits;
            fixedLitCode(lengthSymbol(bestLen), code, bits);
            bw.putCode(code, bits);
            if (kLenExtra[li]) bw.put(bestLen - kLenBase[li], kLenExtra[li]);
            const u32 di = distIndex(bestDist);
            bw.putCode(di, 5);                       // fixed distance code: 5 bits, MSB-first
            if (kDistExtra[di]) bw.put(bestDist - kDistBase[di], kDistExtra[di]);
            for (u32 k = 0; k < bestLen; ++k) registerPos(i + k);
            i += bestLen;
        } else {
            u32 code, bits;
            fixedLitCode(src[i], code, bits);
            bw.putCode(code, bits);
            registerPos(i);
            ++i;
        }
    }
    u32 code, bits;
    fixedLitCode(256, code, bits);
    bw.putCode(code, bits);
    return bw.take();
}

// PNG row filtering.  The filter is chosen per row by the standard heuristic
// (least sum of absolute signed values): on texture data it is the difference
// between "compresses" and "does not".
u8 paeth(i32 a, i32 b, i32 c) {
    const i32 p = a + b - c;
    const i32 pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return static_cast<u8>(a);
    if (pb <= pc) return static_cast<u8>(b);
    return static_cast<u8>(c);
}

std::vector<u8> filterRows(const u8* pixels, u32 width, u32 height, u32 channels) {
    const size_t stride = static_cast<size_t>(width) * channels;
    std::vector<u8> out;
    out.reserve((stride + 1) * height);
    std::vector<u8> prev(stride, 0), cur(stride, 0), best(stride, 0), cand(stride, 0);
    for (u32 y = 0; y < height; ++y) {
        const u8* row = pixels + static_cast<size_t>(y) * stride;
        u8 bestType = 0;
        u64 bestScore = ~0ull;
        for (u8 type = 0; type <= 4; ++type) {
            u64 score = 0;
            for (size_t x = 0; x < stride; ++x) {
                const i32 a = x >= channels ? row[x - channels] : 0;
                const i32 b = prev[x];
                const i32 c = x >= channels ? prev[x - channels] : 0;
                i32 v = 0;
                switch (type) {
                    case 0: v = row[x]; break;
                    case 1: v = row[x] - a; break;
                    case 2: v = row[x] - b; break;
                    case 3: v = row[x] - ((a + b) >> 1); break;
                    default: v = row[x] - paeth(a, b, c); break;
                }
                const u8 byte = static_cast<u8>(v & 0xFF);
                cand[x] = byte;
                score += byte < 128 ? byte : 256 - byte;
            }
            if (score < bestScore) { bestScore = score; bestType = type; best.swap(cand); }
        }
        out.push_back(bestType);
        out.insert(out.end(), best.begin(), best.end());
        std::memcpy(cur.data(), row, stride);
        prev.swap(cur);
    }
    return out;
}

}  // namespace

bool writePng(const std::string& path, const u8* pixels, u32 width, u32 height,
              u32 channels) {
    if (!pixels || !width || !height || channels < 1 || channels > 4) return false;
    const u8 colorType = (channels == 1) ? 0 : (channels == 2) ? 4 : (channels == 3) ? 2 : 6;

    const std::vector<u8> raw = filterRows(pixels, width, height, channels);
    const std::vector<u8> deflated = deflateFixed(raw);

    std::vector<u8> z;
    z.reserve(deflated.size() + 8);
    z.push_back(0x78); z.push_back(0x01);        // zlib, deflate, 32 KiB window
    z.insert(z.end(), deflated.begin(), deflated.end());
    be32(z, adler32(raw.data(), raw.size()));

    std::vector<u8> out = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    std::vector<u8> ihdr;
    be32(ihdr, width); be32(ihdr, height);
    ihdr.push_back(8);            // bit depth
    ihdr.push_back(colorType);
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);   // deflate, adaptive filter, no interlace
    chunk(out, "IHDR", ihdr);
    chunk(out, "IDAT", z);
    chunk(out, "IEND", {});
    return writeFile(path, out.data(), out.size());
}

// -----------------------------------------------------------------------------
//  Contact sheet
// -----------------------------------------------------------------------------
namespace {
// 5x7 bitmap font, A-Z 0-9 _ . - / — enough to caption a material library.
struct Glyph { char c; u8 rows[7]; };
const Glyph kFont[] = {
    {'A', {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}}, {'B', {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}},
    {'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}}, {'D', {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}},
    {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}}, {'F', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},
    {'G', {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}}, {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}}, {'J', {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}},
    {'K', {0x11,0x12,0x14,0x18,0x14,0x12,0x11}}, {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'M', {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}}, {'N', {0x11,0x19,0x15,0x13,0x11,0x11,0x11}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}}, {'P', {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},
    {'Q', {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}}, {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'S', {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}}, {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
    {'U', {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}}, {'V', {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}},
    {'W', {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}}, {'X', {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}},
    {'Y', {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}}, {'Z', {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}},
    {'0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}}, {'1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},
    {'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}}, {'3', {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},
    {'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}}, {'5', {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},
    {'6', {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}}, {'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
    {'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}}, {'9', {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},
    {'_', {0x00,0x00,0x00,0x00,0x00,0x00,0x1F}}, {'-', {0x00,0x00,0x00,0x0E,0x00,0x00,0x00}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}}, {'/', {0x01,0x02,0x02,0x04,0x08,0x08,0x10}},
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
};
const Glyph* glyphFor(char c) {
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
    for (const Glyph& g : kFont) if (g.c == c) return &g;
    return nullptr;
}
void drawText(std::vector<u8>& img, u32 w, u32 h, u32 channels, i32 x0, i32 y0,
              const char* text, const u8 rgb[3]) {
    i32 x = x0;
    for (const char* p = text; *p; ++p) {
        const Glyph* g = glyphFor(*p);
        if (g) {
            for (i32 r = 0; r < 7; ++r)
                for (i32 c = 0; c < 5; ++c)
                    if (g->rows[r] & (1 << (4 - c))) {
                        const i32 px = x + c, py = y0 + r;
                        if (px < 0 || py < 0 || px >= static_cast<i32>(w) || py >= static_cast<i32>(h))
                            continue;
                        u8* q = &img[(static_cast<size_t>(py) * w + px) * channels];
                        q[0] = rgb[0]; q[1] = rgb[1]; q[2] = rgb[2];
                        if (channels == 4) q[3] = 255;
                    }
        }
        x += 6;
    }
}
}  // namespace

bool writeContactSheet(const std::string& path, const std::vector<std::string>& names,
                       const std::vector<const std::vector<u8>*>& tiles, u32 tile,
                       u32 columns, const char* title) {
    if (names.size() != tiles.size() || names.empty() || !columns) return false;
    const u32 caption = 12;
    const u32 pad = 4;
    const u32 cellW = tile + pad * 2;
    const u32 cellH = tile + caption + pad * 2;
    const u32 rows = static_cast<u32>((names.size() + columns - 1) / columns);
    const u32 header = 24;
    const u32 W = columns * cellW;
    const u32 H = header + rows * cellH + pad;
    std::vector<u8> img(static_cast<size_t>(W) * H * 3, 24);
    // Header band and title.
    for (u32 y = 0; y < header; ++y)
        for (u32 x = 0; x < W; ++x) {
            u8* q = &img[(static_cast<size_t>(y) * W + x) * 3];
            q[0] = 40; q[1] = 42; q[2] = 48;
        }
    const u8 white[3] = {235, 235, 235};
    const u8 grey[3] = {185, 190, 200};
    drawText(img, W, H, 3, static_cast<i32>(pad), 8, title, white);

    for (size_t i = 0; i < names.size(); ++i) {
        const u32 cx = static_cast<u32>(i) % columns;
        const u32 cy = static_cast<u32>(i) / columns;
        const u32 x0 = cx * cellW + pad;
        const u32 y0 = header + cy * cellH + pad;
        // The tile, point-sampled (no filtering: this is a review of texels,
        // not of a mip chain), then the caption under it.
        for (u32 y = 0; y < tile; ++y) {
            for (u32 x = 0; x < tile; ++x) {
                const f32 u = (static_cast<f32>(x) + 0.5f) / static_cast<f32>(tile);
                const f32 v = (static_cast<f32>(y) + 0.5f) / static_cast<f32>(tile);
                (void)v;
                // Tile source is TW x TW x 4 (the generator's default size).
                const u8* src = tiles[i]->data();
                const u32 TW = 256;
                const u32 su = static_cast<u32>(u * static_cast<f32>(TW)) % TW;
                const u32 sv = static_cast<u32>(v * static_cast<f32>(TW)) % TW;
                const u8* s = &src[(static_cast<size_t>(sv) * TW + su) * 4];
                u8* d = &img[(static_cast<size_t>(y0 + y) * W + (x0 + x)) * 3];
                d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
            }
        }
        drawText(img, W, H, 3, static_cast<i32>(x0), static_cast<i32>(y0 + tile + 3),
                 names[i].c_str(), grey);
    }
    return writePng(path, img.data(), W, H, 3);
}

}  // namespace mumbai::tex
