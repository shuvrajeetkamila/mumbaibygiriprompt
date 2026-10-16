// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/dynamic_resolution.h
//
//  THE KNOB THAT KEEPS THE FRAME RATE, and the only thing in this engine that
//  deliberately trades image quality for time.
//
//  The specification asks for two things that pull in opposite directions: a
//  render resolution between 720p and 1080p (S6), and 30 frames per second in the
//  worst case (S6 again, and S7's acceptance).  A city of 10 million triangles
//  streaming off a phone's flash cannot promise both at a fixed resolution, so the
//  resolution is what moves: the renderer's targets are `swapchain x renderScale`
//  (engine/render/renderer.cpp), and this decides what `renderScale` is, one frame
//  at a time.
//
//  WHY IT IS A SEPARATE FILE, AND PURE.  A control loop that only exists inside a
//  frame loop on a phone is a control loop nobody can test: it needs a GPU, a
//  device at the edge of its budget, and a person watching for oscillation.  This
//  is the policy on its own — fed frame times, it returns a scale — so the tests
//  can drive it with a simulated device that is too slow, one that is fast, and one
//  that stalls for two seconds, and check what a phone would actually have done.
//  It includes nothing from Vulkan and runs anywhere.
//
//  THE POLICY, AND WHY EACH PART OF IT IS THERE.
//
//    * It acts on a *smoothed* frame interval (an exponential moving average), not
//      on one frame.  A single slow frame happens for reasons that have nothing to
//      do with the resolution — a chunk landing, the OS preempting the process, a
//      texture upload — and resizing the render targets because of one of those is
//      both wrong and expensive: `setEnvironment` tears the offscreen images down.
//
//    * The dead band is ASYMMETRIC, and that is what makes it honest.  A device must
//      be genuinely *fast* (`headroomSlack` below the target) before the policy
//      gives resolution back, but it only has to be *late* (`overrunSlack`, zero by
//      default) for the policy to take some away.  A symmetric band is the obvious
//      design and it is wrong in exactly the way that matters: it settles when the
//      frame time is inside the band, which means it accepts `target x (1 + slack)`
//      — 28.7 frames per second against the specification's 30.  With the band this
//      way round the equilibrium is the largest scale the device can hold *at* the
//      target, and the hysteresis still exists, because recovering from a step down
//      needs headroom the device does not have right after it.
//
//    * It acts at most once every `cooldownFrames`, and it steps *down* faster than
//      it steps *up*.  Dropping resolution is what protects the frame rate, so it
//      is allowed to be decisive; taking resolution back risks making the next
//      second stutter, so it is deliberately timid.  Both are one step — the point
//      is to converge, not to fling the scale to a floor on the first bad frame.
//
//    * It never leaves [minScale, maxScale].  `maxScale` is 1.0 because rendering
//      above the panel's own resolution buys sharpness nobody can see and costs
//      memory bandwidth every frame; `minScale` defaults to two thirds, which is
//      720p on a 1080p panel — the specification's floor.
//
//  The measured behaviour of this policy is in D-052, and its tests are in
//  tests/test_render.cpp against simulated devices rather than against a phone.
// =============================================================================
#pragma once
#include "engine/core/core.h"

namespace mumbai::render {

struct ResolutionPolicy {
    // The frame interval the loop is trying to hold.  A third of a second is 30 FPS
    // — the specification's worst case — and the caller may raise it (but not to
    // below it) if the display itself is slower.
    f64 targetSeconds = 1.0 / 30.0;

    f32 minScale = 0.667f;   // 720p on a 1080p panel: the specification's floor
    f32 maxScale = 1.0f;     // never render above the panel's own resolution

    f32 downStep = 0.06f;    // how much resolution one decision gives up
    f32 upStep = 0.03f;      // and how much it takes back

    // How late a frame may be before resolution is given up (0 = none: the target
    // is the target) and how much faster than the target the device must be before
    // resolution is taken back.  See the header on why these are not one number.
    f32 overrunSlack = 0.0f;
    f32 headroomSlack = 0.10f;
    f32 smooth = 0.15f;      // weight of the newest frame in the average
    int cooldownFrames = 30; // at most one decision per this many frames (1 s at 30 FPS)
};

// One scale decision per frame, in the order the frames happened.
class DynamicResolution {
  public:
    DynamicResolution() = default;
    explicit DynamicResolution(ResolutionPolicy policy);

    // Feed the frame that just finished; returns the scale to render the next one
    // at.  The very first frames return `maxScale`, so the pipeline starts at full
    // resolution and gives ground only if the device actually cannot hold it.
    f32 update(f64 frameSeconds);

    f32 scale() const { return m_scale; }
    f64 smoothedSeconds() const { return m_smoothed; }
    u64 frames() const { return m_frames; }
    u64 changes() const { return m_changes; }      // decisions that moved the scale
    const ResolutionPolicy& policy() const { return m_policy; }

    // A frame time the policy ignores entirely: the clock in a frame that was
    // measured across a stall the app did not cause is not evidence about the
    // resolution.  `stallSeconds` is the length at which a frame is not believed;
    // the default leaves normal scheduling jitter well inside the average.
    f64 ignoreAboveSeconds = 0.5;

    // One sentence for the panel, e.g. "render 1280x720  scale 0.67  100 %".
    std::string status(u32 width, u32 height) const;

  private:
    ResolutionPolicy m_policy{};
    f64 m_smoothed = 0.0;
    f32 m_scale = 1.0f;
    int m_sinceDecision = 0;
    u64 m_frames = 0;
    u64 m_changes = 0;
    bool m_started = false;
};

}  // namespace mumbai::render
