// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/dynamic_resolution.cpp
//
//  The policy described in the header, in the order the frames arrive.  Nothing
//  here touches a graphics API: the caller re-sizes its own render targets.
// =============================================================================
#include "engine/render/dynamic_resolution.h"

#include <cstdio>

namespace mumbai::render {
namespace {

// The scale the loop actually renders at, clamped to the policy's window.  Kept in
// one place because it is the one number every other line here depends on.
f32 clampScale(const ResolutionPolicy& p, f32 scale) {
    // A policy with the bounds the wrong way round (a caller's mistake) must not
    // produce a scale outside the specification's window: the floor wins.
    const f32 lo = std::min(p.minScale, p.maxScale);
    const f32 hi = std::max(p.minScale, p.maxScale);
    return clamp(scale, lo, hi);
}

}  // namespace

DynamicResolution::DynamicResolution(ResolutionPolicy policy) : m_policy(policy) {
    m_scale = clampScale(m_policy, m_policy.maxScale);
}

f32 DynamicResolution::update(f64 frameSeconds) {
    // The first frame has no interval to measure, and a caller that passes a
    // nonsense one (zero, negative, not a number) is not evidence either.
    if (!(frameSeconds > 0.0) || !std::isfinite(frameSeconds)) return m_scale;

    ++m_frames;
    // A frame longer than `ignoreAboveSeconds` is a stall — a chunk being paged in,
    // the process being descheduled, the phone's thermal daemon taking its turn —
    // and folding it into the average would make the app give up resolution it did
    // not need.  It is skipped, not reset: the average keeps its history.
    if (frameSeconds <= ignoreAboveSeconds) {
        if (!m_started) {
            m_smoothed = frameSeconds;
            m_started = true;
        } else {
            const f64 w = clamp(static_cast<f64>(m_policy.smooth), 0.0, 1.0);
            m_smoothed = m_smoothed * (1.0 - w) + frameSeconds * w;
        }
    }
    if (!m_started) return m_scale;          // nothing but stalls so far

    // The dead band, in the units the decision is made in: seconds.
    const f64 target = m_policy.targetSeconds > 0.0 ? m_policy.targetSeconds : 1.0 / 30.0;
    const f64 late = target * (1.0 + m_policy.overrunSlack);
    const f64 early = target * (1.0 - m_policy.headroomSlack);

    const bool wantsDown = m_smoothed > late;
    const bool wantsUp = m_smoothed < early;
    if (!wantsDown && !wantsUp) {
        // Inside the band: no decision, and the cooldown does not accrue a debt
        // that would let the next two frames both move the scale.
        if (m_sinceDecision < m_policy.cooldownFrames) ++m_sinceDecision;
        return m_scale;
    }

    if (m_sinceDecision < m_policy.cooldownFrames) {
        ++m_sinceDecision;
        return m_scale;
    }

    // At the end of its own range, a device that still wants to move is as close to
    // the target as this policy can get.  Count the frame as a decision slot used
    // rather than resetting the timer, so the loop does not re-decide every frame
    // for ever while pinned: it settles.
    const f32 before = m_scale;
    f32 after = before;
    if (wantsDown) after = before - m_policy.downStep;
    else if (before < clampScale(m_policy, m_policy.maxScale)) after = before + m_policy.upStep;
    m_scale = clampScale(m_policy, after);
    m_sinceDecision = 0;
    if (std::fabs(m_scale - before) > 1e-6f) ++m_changes;
    return m_scale;
}

std::string DynamicResolution::status(u32 width, u32 height) const {
    char buffer[128];
    const u32 w = static_cast<u32>(static_cast<f64>(width) * static_cast<f64>(m_scale));
    const u32 h = static_cast<u32>(static_cast<f64>(height) * static_cast<f64>(m_scale));
    std::snprintf(buffer, sizeof(buffer), "render %ux%u  scale %.2f", w, h,
                  static_cast<double>(m_scale));
    return std::string(buffer);
}

}  // namespace mumbai::render
