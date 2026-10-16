// =============================================================================
//  world.frag — the forward pass for everything on land
//
//  A single directional sun with a hemispheric fill, a Cook-Torrance GGX lobe,
//  the material's ORM array for roughness and metal, the baked vertex AO, the
//  terrain's own normal, and the wetness response the specification asks for:
//  standing water drops roughness toward zero and darkens albedo by 30 %, which
//  is what makes a wet road mirror the golden-hour sky.
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

vec3 fresnelSchlick(float cosTheta, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float distributionGGX(float nDotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float d = nDotH * nDotH * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-7);
}

float geometrySchlick(float nDotV, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return nDotV / (nDotV * (1.0 - k) + k);
}

void main() {
    vec3 albedo = texture(albedoArray, vec3(vUv, float(vMaterial))).rgb * vTint;
    vec3 orm = texture(ormArray, vec3(vUv, float(vMaterial))).rgb;
    float ao = clamp(orm.r * vAo, 0.0, 1.0);
    float roughness = clamp(orm.g, 0.04, 1.0);
    float metallic = clamp(orm.b, 0.0, 1.0);

    // Normal mapping from the shipped array.  The tangent frame is derived from
    // the surface normal and the world up axis, which is exact for the horizontal
    // surfaces this world is mostly made of and an approximation on a wall — the
    // honest fix is a baked tangent attribute, and that is a chunk-format change
    // (recorded in D-043, not hidden here).
    vec3 n = normalize(vNormal);
    vec3 t = texture(normalArray, vec3(vUv, float(vMaterial))).rgb * 2.0 - 1.0;
    vec3 up = abs(n.y) < 0.95 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = normalize(cross(up, n));
    vec3 bitangent = cross(n, tangent);
    vec3 normal = normalize(n + (tangent * t.x + bitangent * t.y) * 0.8);

    // Wetness: the CPU's rain field drives this uniform, and the response is the
    // specification's — smoother, darker, and therefore a mirror of the sky.
    float wet = frame.envParams.w;
    roughness = mix(roughness, 0.035, wet * 0.92);
    albedo *= mix(1.0, 0.70, wet);

    vec3 v = normalize(frame.cameraPos.xyz - vWorldPos);
    vec3 l = normalize(frame.sunDir.xyz);
    vec3 h = normalize(v + l);
    float nDotL = max(dot(normal, l), 0.0);
    float nDotV = max(dot(normal, v), 1e-4);
    float nDotH = max(dot(normal, h), 0.0);
    float vDotH = max(dot(v, h), 0.0);

    vec3 f0 = mix(vec3(0.04), albedo, metallic);
    vec3 f = fresnelSchlick(vDotH, f0);
    float d = distributionGGX(nDotH, roughness);
    float g = geometrySchlick(nDotV, roughness) * geometrySchlick(nDotL, roughness);
    vec3 specular = (d * g) * f / max(4.0 * nDotV * nDotL, 1e-4);

    vec3 kd = (1.0 - f) * (1.0 - metallic);
    vec3 sun = (kd * albedo / PI + specular) * frame.sunColour.rgb * frame.sunDir.w * nDotL;

    // Hemispheric fill: the sky is a big dome light, and its colour is the sun's
    // lifted toward blue by the humidity.
    vec3 skyFill = mix(vec3(0.45, 0.60, 0.85), frame.sunColour.rgb, 0.35);
    vec3 ambient = albedo * skyFill * frame.sunColour.w * (0.5 + 0.5 * normal.y) * ao;

    // A single inexpensive specular bounce of the sun off the surface toward the
    // eye, weighted by wetness: this is the sheen on a wet road at golden hour in
    // one line rather than in a reflection pass.
    float sheenPower = mix(220.0, 900.0, wet);
    vec3 sheen = frame.sunColour.rgb * pow(max(dot(reflect(-l, normal), v), 0.0), sheenPower) *
                 wet * (1.0 - roughness) * 2.4;

    vec3 colour = sun + ambient + sheen * frame.sunDir.w;
    float distance = length(frame.cameraPos.xyz - vWorldPos);
    colour = applyFog(colour, distance, vWorldPos.y);
    outColour = vec4(colour, 1.0);
}
