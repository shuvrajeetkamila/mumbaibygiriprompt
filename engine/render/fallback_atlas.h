// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/fallback_atlas.h
//
//  THE ATLAS THE RENDERER DRAWS WITH WHEN THE CITY IS NOT THERE YET.
//
//  The material arrays ship with the world, and the world is a download: a phone
//  that has just installed the client has no atlas, and the first build treated
//  that as a fatal error.  On a device that meant a black screen and a log line,
//  which is the least useful possible answer to "why is my screen black".
//
//  So the absence is a state, not a failure: the renderer generates eight
//  surfaces procedurally — the eight the first-person view actually shows — and
//  starts anyway.  The city then looks like an untextured mock-up rather than a
//  photograph, which is a thing a person can recognise and report, and the panel
//  says which atlas is in use.
//
//  The pixel generation is CPU-only and pure: it is verified by
//  `tests/test_render.cpp` on a machine with no GPU, because the one path that
//  cannot be tested on a phone is the one that should be tested hardest.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <vector>

namespace mumbai::render {

inline constexpr u32 kFallbackLayers = 8;

// The eight surfaces, in the order the material table uses them.
struct FallbackMaterial {
    f32 base[3];
    f32 mottle;      // how much the noise moves the albedo
    f32 roughness;
    f32 metal;
};

extern const FallbackMaterial kFallbackMaterials[kFallbackLayers];

// Fills `albedo`, `normal` and `orm` with size*size*kFallbackLayers RGBA8 pixels:
// albedo is sRGB-encoded output, normal is flat (128,128,255) and ORM carries
// occlusion / roughness / metal.  Deterministic: the same size gives the same
// bytes on every machine, which is what makes it testable.
void buildFallbackAtlasPixels(u32 size, std::vector<u8>& albedo, std::vector<u8>& normal,
                              std::vector<u8>& orm);

}  // namespace mumbai::render
