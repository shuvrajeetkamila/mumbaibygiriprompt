// =============================================================================
//  hud.frag — the on-screen panel's pixels
//
//  One single-channel texture: the glyph mask.  Colour comes from the vertex, so
//  a line can be green when the frame rate is fine and amber when it is not
//  without a second draw.  A rectangle samples the atlas cell that is filled
//  solid, which is why there is no separate "panel" shader.
//
//  No depth test and no tone mapping: this is text on a screen, not light in a
//  scene, and running it through the exposure would make it dimmer at dusk —
//  exactly when it is most likely to be read.
// =============================================================================
#version 450

layout(set = 0, binding = 0) uniform sampler2D uFont;

layout(location = 0) in vec2 vUv;
layout(location = 1) in vec4 vColour;

layout(location = 0) out vec4 oColour;

void main() {
    float mask = texture(uFont, vUv).r;
    oColour = vec4(vColour.rgb, vColour.a * mask);
}
