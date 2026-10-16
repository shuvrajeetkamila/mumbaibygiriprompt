// =============================================================================
//  hud.vert — the on-screen panel's geometry
//
//  Positions arrive already in *pixels*, measured from the top-left corner,
//  because that is the coordinate system the client thinks in when it lays out a
//  status line ("eight pixels in, two pixels down").  Converting to clip space is
//  this shader's entire job, and the viewport it needs comes in as a push
//  constant: a HUD whose scale is baked into a pipeline is a HUD that goes stale
//  the first time the screen rotates.
// =============================================================================
#version 450

layout(location = 0) in vec2 aPos;      // pixels, origin top-left
layout(location = 1) in vec2 aUv;       // glyph-atlas texels
layout(location = 2) in vec4 aColour;   // 0xAABBGGRR, unnormalised bytes

layout(push_constant) uniform HudPush {
    vec2 viewport;      // pixels
    float fontLayer;    // reserved: the atlas is a single 2D image today
    float pad;
} pc;

layout(location = 0) out vec2 vUv;
layout(location = 1) out vec4 vColour;

void main() {
    vec2 ndc = vec2(aPos.x / pc.viewport.x * 2.0 - 1.0,
                    1.0 - aPos.y / pc.viewport.y * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
    vUv = aUv;
    vColour = aColour;
}
