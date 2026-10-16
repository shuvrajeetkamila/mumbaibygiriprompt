// =============================================================================
//  sky.vert — one full-screen triangle, drawn last
//
//  Drawn after the geometry with the depth test left at LESS_OR_EQUAL and a
//  depth of exactly 1.0: the sky therefore shades only the pixels the world did
//  not cover.  On a tiled GPU that is the difference between paying for the sky
//  once and paying for it under every building in Bandra.
// =============================================================================
#version 450
#extension GL_GOOGLE_include_directive : require
#include "common.glsl"

layout(location = 0) out vec2 vNdc;

void main() {
    // (-1,-1) (3,-1) (-1,3): the standard oversized triangle, no vertex buffer.
    vNdc = vec2((gl_VertexIndex == 2) ? 3.0 : -1.0, (gl_VertexIndex == 1) ? 3.0 : -1.0);
    gl_Position = vec4(vNdc, 1.0, 1.0);   // z == w: the far plane, exactly
}
