// =============================================================================
//  world.vert — terrain, roads, buildings, rail and water, one shader
//
//  The vertex layout is `MeshVertex` (44 bytes, five attributes) exactly as the
//  pipeline writes it.  Nothing is re-derived: `ao` arrives baked, `materialId`
//  selects the array layer, and the second UV set is the macro-variation pair the
//  material generator produced.
// =============================================================================
#version 450
#include "common.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in vec2 inUv2;
// Four unsigned bytes, not four uints.  MeshVertex packs them into one u32 so
// the vertex is 44 bytes and not 56; declaring `uvec4` here would make the
// driver read the vertex as 16 bytes longer, walk the buffer out of step, and
// render a shredded city with no error anywhere.  tools/verify_shader_layout.py
// compares this against pipeline.cpp's VK_FORMAT_R8G8B8A8_UINT (D-043).
layout(location = 4) in uint inPacked;      // materialId | zoneId | flags | ao

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec2 vUv;
layout(location = 3) out vec2 vUv2;
layout(location = 4) flat out uint vMaterial;
layout(location = 5) flat out uint vFlags;
layout(location = 6) out float vAo;
layout(location = 7) out vec3 vTint;        // per-zone variation, cheap and stable

void main() {
    // Cell-local metres plus the origin delta: this addition is the floating
    // origin, and it is the only place world space is formed.
    vec3 world = inPosition + push.originDelta.xyz;
    vWorldPos = world;
    vNormal = inNormal;
    vUv = inUv;
    vUv2 = inUv2;
    vMaterial = inPacked & 0xFFu;
    vFlags = (inPacked >> 16) & 0xFFu;
    vAo = float((inPacked >> 24) & 0xFFu) / 255.0;

    // Zone tint: a per-zone colour and brightness jitter, derived from the
    // zone id and the world position.  It is what stops the fabric reading as
    // one material tiled across 600 km2, and it costs one hash.
    uint z = (inPacked >> 8) & 0xFFu;   // zoneId
    float h = fract(sin(float(z) * 12.9898 + dot(floor(world.xz * 0.02), vec2(37.0, 57.0))) * 43758.5453);
    vTint = mix(vec3(0.95, 0.95, 0.97), vec3(1.06, 1.02, 0.96), h);

    gl_Position = frame.viewProj * vec4(world, 1.0);
}
