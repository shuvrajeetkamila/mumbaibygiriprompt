// =============================================================================
//  post.vert — the tonemap's full-screen triangle
// =============================================================================
#version 450
#include "common.glsl"

layout(location = 0) out vec2 vUv;

void main() {
    vec2 ndc = vec2((gl_VertexIndex == 2) ? 3.0 : -1.0, (gl_VertexIndex == 1) ? 3.0 : -1.0);
    vUv = ndc * 0.5 + 0.5;
    gl_Position = vec4(ndc, 0.0, 1.0);
}
