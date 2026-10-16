// =============================================================================
//  water.frag — the sea, the creek and every inland body, blended
//
//  The specification's ocean is a Gerstner sum on the GPU (A5).  This is the
//  water that shipped with the world: a flat surface with a Fresnel mix against
//  the sky, a small procedural ripple in the normal, and an alpha that grows with
//  the grazing angle — which is what makes a creek read as a sheet of light
//  rather than as blue paint.  The Gerstner displacement replaces the ripple term
//  when A5 lands; the Fresnel, the depth blend and the fog are already here.
// =============================================================================
#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUv;
layout(location = 3) in vec2 vUv2;
layout(location = 4) flat in uint vMaterial;
layout(location = 5) flat in uint vFlags;
layout(location = 6) in float vAo;
layout(location = 7) in vec3 vTint;

layout(location = 0) out vec4 outColour;

void main() {
    float t = frame.skyParams.z;
    vec3 n = normalize(vNormal);
    // Two low-frequency trains plus one high: the swell the specification names
    // is 80-150 m, which over a 12 m UV tile is a slow, almost-flat undulation.
    vec3 ripple = vec3(0.0);
    ripple.x += sin(vUv.x * 6.2831 * 1.0 + t * 0.35) * 0.030;
    ripple.z += sin(vUv.y * 6.2831 * 0.7 + t * 0.28) * 0.030;
    ripple.x += sin(vUv.x * 6.2831 * 4.0 + vUv.y * 6.2831 * 3.0 + t * 0.9) * 0.012;
    n = normalize(n + ripple);

    vec3 v = normalize(frame.cameraPos.xyz - vWorldPos);
    float fresnel = pow(1.0 - max(dot(n, v), 0.0), 4.0);
    vec3 deep = texture(albedoArray, vec3(vUv, float(vMaterial))).rgb * 0.55;
    // Sky colour: the same hemispheric fill the land uses, so water and land
    // under one sun cannot disagree about what colour the sky is.
    vec3 sky = mix(vec3(0.45, 0.60, 0.85), frame.sunColour.rgb, 0.35);

    vec3 colour = mix(deep, sky, clamp(fresnel * 0.85 + 0.15, 0.0, 1.0));
    // The sun's own glitter: a tight lobe on the perturbed normal.
    vec3 l = normalize(frame.sunDir.xyz);
    float glint = pow(max(dot(reflect(-l, n), v), 0.0), 260.0);
    colour += frame.sunColour.rgb * glint * 2.0 * frame.sunDir.w;

    float distance = length(frame.cameraPos.xyz - vWorldPos);
    colour = applyFog(colour, distance, vWorldPos.y);
    outColour = vec4(colour, mix(0.82, 0.96, fresnel));
}
