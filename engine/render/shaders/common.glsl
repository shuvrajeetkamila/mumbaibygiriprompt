// =============================================================================
//  Mumbai 1:1 Exact — shared shader declarations
//
//  The uniform block here is the GLSL side of `FrameUbo` in renderer_impl.h.
//  std140 and a 160-byte struct agree as long as every member is a vec4 or a
//  mat4, which is why the C++ struct is written that way; the static_assert in
//  the header is what keeps the two from drifting.
// =============================================================================
#ifndef MUMBAI_COMMON_GLSL
#define MUMBAI_COMMON_GLSL
#define PI 3.14159265359

layout(set = 0, binding = 0, std140) uniform FrameBlock {
    mat4 viewProj;
    vec4 cameraPos;    // xyz + time
    vec4 sunDir;       // xyz (towards the sun) + intensity
    vec4 sunColour;    // rgb + ambient
    vec4 fog;          // rgb + density per metre
    vec4 envParams;    // humidity, pollution, exposure, wetness
    vec4 skyParams;    // render width, render height, time, unused
} frame;

layout(set = 0, binding = 1) uniform sampler2DArray albedoArray;
layout(set = 0, binding = 2) uniform sampler2DArray normalArray;
layout(set = 0, binding = 3) uniform sampler2DArray ormArray;
layout(set = 0, binding = 4) uniform sampler2D hdrColour;

layout(push_constant) uniform Push {
    // The chunk's macro-cell origin relative to the render origin: the entire
    // render-side floating origin (Section 4A), one vec4 per draw batch.
    vec4 originDelta;
} push;

// ACES filmic tone curve (Narkowicz's fit).  Used by the tonemap pass and by
// nothing else: lighting is computed in linear HDR and stays there.
vec3 acesToneMap(vec3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// Fog as an exponential extinction with a height term: the specification's
// monsoon air is denser at the bottom of the column, and a road that fades into
// its own vapour is most of what "monsoon" looks like from a bike.
vec3 applyFog(vec3 colour, float distance, float height) {
    float h = exp(-max(height, 0.0) / 2200.0);
    float f = 1.0 - exp(-frame.fog.w * distance * h);
    return mix(colour, frame.fog.rgb, clamp(f, 0.0, 1.0));
}
#endif
