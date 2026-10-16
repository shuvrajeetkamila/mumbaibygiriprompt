// =============================================================================
//  post.frag — dynamic resolution downsample, exposure, filmic tonemap, dither
//
//  When the render scale is above 1 the HDR buffer is larger than the swapchain
//  and this pass averages four taps: that is the anti-aliasing this build has.
//  TAA is specified and not yet written (it needs a history buffer and a motion
//  vector — see D-043); until then the render scale is the honest knob, and the
//  tonemap is where the frame's resolution is decided rather than where it is
//  discovered.
//
//  The dither is not decoration: an 8-bit output of a dark, smooth golden-hour
//  gradient bands visibly, and a 1-LSB ordered dither costs one add.
// =============================================================================
#version 450
#include "common.glsl"

layout(location = 0) in vec2 vUv;
layout(location = 0) out vec4 outColour;

void main() {
    vec2 texel = 1.0 / vec2(textureSize(hdrColour, 0));
    vec2 srcScale = vec2(frame.skyParams.x, frame.skyParams.y) * texel;
    vec3 hdr;
    if (srcScale.x > 1.001 || srcScale.y > 1.001) {
        // Four-tap box average: the render scale's whole purpose.
        vec2 o = texel * 0.5;
        hdr = 0.25 * (texture(hdrColour, vUv + vec2(-o.x, -o.y)).rgb +
                      texture(hdrColour, vUv + vec2(o.x, -o.y)).rgb +
                      texture(hdrColour, vUv + vec2(-o.x, o.y)).rgb +
                      texture(hdrColour, vUv + vec2(o.x, o.y)).rgb);
    } else {
        hdr = texture(hdrColour, vUv).rgb;
    }

    vec3 colour = hdr * frame.envParams.z;      // exposure
    colour = acesToneMap(colour);

    // Ordered-ish dither at 1 LSB, from the pixel coordinate.
    float d = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    colour += (d - 0.5) / 255.0;

    outColour = vec4(colour, 1.0);
}
