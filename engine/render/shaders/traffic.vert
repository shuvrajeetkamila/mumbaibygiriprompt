// =============================================================================
//  traffic.vert — instanced vehicles
//
//  One instance per vehicle the model reports within earshot, with its own yaw.
//  The mesh is the archetype's silhouette (see scene.cpp for why the first build
//  has silhouettes rather than modelled vehicles).
// =============================================================================
#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColour;

layout(location = 3) in vec3 iPos;
layout(location = 4) in vec2 iYawScale;
layout(location = 5) in vec3 iColour;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out vec3 vColour;
layout(location = 2) out vec3 vWorldPos;

void main() {
    float c = cos(iYawScale.x), s = sin(iYawScale.x);
    // Yaw about +Y, in the world's convention (atan2(dx, dz)).
    vec3 rotated = vec3(inPosition.x * c + inPosition.z * s, inPosition.y,
                        -inPosition.x * s + inPosition.z * c);
    vec3 world = iPos + rotated * iYawScale.y;
    vNormal = normalize(vec3(inNormal.x * c + inNormal.z * s, inNormal.y,
                             -inNormal.x * s + inNormal.z * c));
    vColour = iColour * inColour;
    vWorldPos = world;
    gl_Position = frame.viewProj * vec4(world, 1.0);
}
