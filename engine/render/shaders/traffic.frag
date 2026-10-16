// =============================================================================
//  traffic.frag — a Lambertian panel with the sun on it
//
//  Vehicles are not hero assets in this build, and this shader does not pretend
//  otherwise: it is the sun, the sky fill and the fog, so a bus in the distance
//  sits in the same atmosphere as the road it is on.  Metallic paint, glass and
//  brake lights arrive with the vehicle models.
// =============================================================================
#version 450
#include "common.glsl"

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec3 vColour;
layout(location = 2) in vec3 vWorldPos;
layout(location = 0) out vec4 outColour;

void main() {
    vec3 n = normalize(vNormal);
    vec3 l = normalize(frame.sunDir.xyz);
    float nDotL = max(dot(n, l), 0.0);
    vec3 skyFill = mix(vec3(0.45, 0.60, 0.85), frame.sunColour.rgb, 0.35);
    vec3 colour = vColour * (frame.sunColour.rgb * frame.sunDir.w * nDotL / PI +
                             skyFill * frame.sunColour.w * (0.5 + 0.5 * n.y));
    float wet = frame.envParams.w;
    colour *= mix(1.0, 0.78, wet);
    float distance = length(frame.cameraPos.xyz - vWorldPos);
    colour = applyFog(colour, distance, vWorldPos.y);
    outColour = vec4(colour, 1.0);
}
