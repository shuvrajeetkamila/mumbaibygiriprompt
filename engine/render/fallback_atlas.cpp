// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/fallback_atlas.cpp
//
//  Tileable value noise, and eight surfaces built from it.  No allocation per
//  pixel, no state, no seed from the clock: this runs while a phone is starting
//  up and it has to give the same answer every time.
// =============================================================================
#include "engine/render/fallback_atlas.h"

#include <algorithm>
#include <cmath>

namespace mumbai::render {
namespace {

// An integer hash with no dependencies: the noise has to be identical on a phone,
// on a desktop and in a test, and <random> is not.
u32 hashU32(u32 x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

f32 lattice(u32 x, u32 y, u32 period, u32 seed) {
    const u32 h = hashU32(hashU32(x % period + seed * 0x9e3779b9u) ^ (y % period) * 0x85ebca6bu);
    return static_cast<f32>(h & 0xFFFFu) / 65535.0f;
}

f32 valueNoise(f32 fx, f32 fy, u32 period, u32 seed) {
    const u32 x0 = static_cast<u32>(std::floor(fx));
    const u32 y0 = static_cast<u32>(std::floor(fy));
    const f32 tx = fx - static_cast<f32>(x0);
    const f32 ty = fy - static_cast<f32>(y0);
    // Smoothstep: a linear interpolation of lattice values leaves the grid visible
    // as a lattice, which is the one artifact a flat colour cannot hide.
    const f32 sx = tx * tx * (3.0f - 2.0f * tx);
    const f32 sy = ty * ty * (3.0f - 2.0f * ty);
    const f32 a = lattice(x0, y0, period, seed);
    const f32 b = lattice(x0 + 1, y0, period, seed);
    const f32 c = lattice(x0, y0 + 1, period, seed);
    const f32 d = lattice(x0 + 1, y0 + 1, period, seed);
    return (a * (1.0f - sx) + b * sx) * (1.0f - sy) + (c * (1.0f - sx) + d * sx) * sy;
}

f32 fbm2(f32 x, f32 y, u32 seed) {
    return valueNoise(x, y, 16, seed) * 0.5f + valueNoise(x * 2.0f, y * 2.0f, 32, seed) * 0.3f +
           valueNoise(x * 4.0f, y * 4.0f, 64, seed) * 0.2f;
}

u8 toByte(f32 v) {
    return static_cast<u8>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
}

}  // namespace

const FallbackMaterial kFallbackMaterials[kFallbackLayers] = {
    {{0.10f, 0.10f, 0.11f}, 0.35f, 0.78f, 0.0f},   // 0 asphalt
    {{0.42f, 0.42f, 0.41f}, 0.22f, 0.62f, 0.0f},   // 1 concrete pavement
    {{0.55f, 0.38f, 0.30f}, 0.30f, 0.70f, 0.0f},   // 2 plaster / brick wall
    {{0.34f, 0.22f, 0.18f}, 0.18f, 0.65f, 0.0f},   // 3 roof tile
    {{0.20f, 0.34f, 0.16f}, 0.45f, 0.85f, 0.0f},   // 4 vegetation
    {{0.08f, 0.18f, 0.24f}, 0.12f, 0.10f, 0.0f},   // 5 water
    {{0.66f, 0.60f, 0.46f}, 0.20f, 0.75f, 0.0f},   // 6 sand
    {{0.62f, 0.64f, 0.66f}, 0.10f, 0.35f, 0.6f},   // 7 metal / glass
};

void buildFallbackAtlasPixels(u32 size, std::vector<u8>& albedo, std::vector<u8>& normal,
                              std::vector<u8>& orm) {
    const size_t pixels = static_cast<size_t>(size) * size * kFallbackLayers;
    albedo.assign(pixels * 4, 0);
    normal.assign(pixels * 4, 0);
    orm.assign(pixels * 4, 0);

    for (u32 layer = 0; layer < kFallbackLayers; ++layer) {
        const FallbackMaterial& m = kFallbackMaterials[layer];
        for (u32 y = 0; y < size; ++y) {
            for (u32 x = 0; x < size; ++x) {
                const f32 n = fbm2(static_cast<f32>(x) / 16.0f, static_cast<f32>(y) / 16.0f,
                                   layer * 131u + 7u);
                const f32 shade = 1.0f + m.mottle * (n - 0.5f);
                const size_t i = (static_cast<size_t>(layer) * size * size + y * size + x) * 4;
                albedo[i + 0] = toByte(m.base[0] * shade);
                albedo[i + 1] = toByte(m.base[1] * shade);
                albedo[i + 2] = toByte(m.base[2] * shade);
                albedo[i + 3] = 255;
                // Flat normals, honestly: a noise normal map on a surface nobody
                // has authored would invent detail the occlusion pass then
                // disagrees with, and the acceptance criterion is "no artifacts",
                // not "some texture".
                normal[i + 0] = 128;
                normal[i + 1] = 128;
                normal[i + 2] = 255;
                normal[i + 3] = 255;
                orm[i + 0] = toByte(0.85f + 0.15f * n);                       // occlusion
                orm[i + 1] = toByte(m.roughness + 0.08f * (n - 0.5f));        // roughness
                orm[i + 2] = toByte(m.metal);                                 // metal
                orm[i + 3] = 255;
            }
        }
    }
}

}  // namespace mumbai::render
