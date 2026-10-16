// =============================================================================
//  sky.frag — Rayleigh + Mie, cheaply
//
//  A single-scattering approximation: Rayleigh with the real lambda^-4 weights
//  and a Mie lobe whose anisotropy and turbidity come from the environment
//  (Section 3's humidity and pollution), evaluated per pixel with no LUT.  A6 is
//  specified as "Rayleigh + Mie with time of day, humidity and pollution"; this
//  is that model integrated, not a gradient with a sun spot in it, and it costs
//  about forty ALU per pixel.  The full multi-scattering LUT version is a later
//  refinement, not a different model.
// =============================================================================
#version 450
#include "common.glsl"

layout(location = 0) in vec2 vNdc;
layout(location = 0) out vec4 outColour;

void main() {
    vec3 view = normalize(vec3(vNdc, -1.7));   // ~60 degree vertical field
    vec3 dir = normalize(vec3(view.x, -view.y, view.z));
    vec3 sun = normalize(frame.sunDir.xyz);
    float up = clamp(dir.y, -1.0, 1.0);
    float mu = dot(dir, sun);

    // Rayleigh: the sky is blue because short wavelengths scatter forty times
    // more, so the zenith colour is derived rather than chosen.
    vec3 betaR = vec3(5.8e-6, 13.5e-6, 33.1e-6) * 1.6e5;
    float rayPhase = (3.0 / (16.0 * PI)) * (1.0 + up * up);
    float airMass = 1.0 / max(up * 0.86 + 0.24, 0.06);   // secant, horizon-clamped

    // Mie: the forward lobe that makes a low sun a disc with a halo, driven by
    // humidity.  The Henyey-Greenstein form with g from the spec's humidity.
    float g = mix(0.62, 0.88, clamp(frame.envParams.x, 0.0, 1.0));
    float gg = g * g;
    float miePhase = (1.0 - gg) / (4.0 * PI * pow(1.0 + gg - 2.0 * g * mu, 1.5));
    float turbidity = mix(1.2, 3.6, clamp(frame.envParams.y, 0.0, 1.0));
    vec3 betaM = vec3(0.9e-5) * turbidity;

    vec3 sunE = frame.sunColour.rgb * max(frame.sunDir.w, 0.0) * 1.4;
    // Extinction along the view ray: the sun's own colour is what survives it.
    vec3 extinction = exp(-(betaR + betaM) * airMass * 1.35e5);
    vec3 inscatter = (betaR * rayPhase + betaM * miePhase) / (betaR + betaM) *
                     (1.0 - extinction) * sunE;

    // Ground half of the dome: the haze the world sits in, so the horizon line
    // is continuous between the two shaders.
    vec3 horizon = frame.fog.rgb;
    vec3 colour = mix(horizon, inscatter, smoothstep(-0.06, 0.20, up));
    colour += frame.sunColour.rgb * pow(max(mu, 0.0), 900.0) * frame.sunDir.w * 6.0;   // disc

    outColour = vec4(colour, 1.0);
}
