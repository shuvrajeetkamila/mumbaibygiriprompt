// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/poll_policy.h
//
//  WHEN THE CLIENT IS ALLOWED TO WAIT, AND WHEN IT IS NOT.
//
//  The client's main loop is one `ALooper_pollOnce` (engine/app/android_main.cpp),
//  and the timeout it passes decides whether the app draws at all.  `pollOnce`
//  returns a source identifier when an event is pending, and a *negative code* only
//  on timeout, wake, callback or error — so a timeout of -1, which means "wait for
//  ever", makes a drain loop's exit condition unreachable.  Everything after that
//  loop then never runs: no first frame, no fallback screen, no log file, no
//  renderer retry.  On a phone that is a black window for ever with nothing on it,
//  which is exactly what was reported ("black screen, no progress bar, ever") and
//  what this header exists to make impossible to write again.
//
//  The rule, therefore: the timeout is always bounded, and it is the *not ready*
//  state — the state the client starts in, before a swapchain exists — that must
//  keep returning to the loop body, because that is where the fallback screen
//  paints and the renderer is started and retried.
//
//  This lives in its own Android-free header so that the rule is testable on a
//  host, where there is no looper to hang: tests/test_app_loop.cpp asserts it.
// =============================================================================
#pragma once

namespace mumbai::app {

// The codes `ALooper_pollOnce` returns when it has no event source to hand back
// (android/looper.h).  Repeated here because a host build has no looper header, and
// because the test below is about these four values, not about the looper.
inline constexpr int kPollWake = -1;
inline constexpr int kPollCallback = -2;
inline constexpr int kPollTimeout = -3;
inline constexpr int kPollError = -4;

// How long the loop may sleep while there is nothing to draw.  One frame at 60 Hz:
// long enough not to spin the CPU on a phone that is showing a status screen, short
// enough that the fallback screen appears and the renderer's retry happens without
// anybody having to touch the glass.
inline constexpr int kIdlePollMs = 16;

// The timeout the loop passes.  Zero with frames to draw — the loop then runs at the
// display's rate rather than being woken by input — and kIdlePollMs otherwise.  It is
// never negative: see the header comment.
inline constexpr int pollTimeoutMs(bool rendererReady) {
    return rendererReady ? 0 : kIdlePollMs;
}

// Whether a `pollOnce` result carries an event source to process.  Every negative
// code means "the queue is empty, go do a frame".
inline constexpr bool pollHasData(int result) { return result >= 0; }

static_assert(pollTimeoutMs(false) > 0,
              "the not-ready state is the one that starts and retries the renderer: a "
              "zero timeout would spin and a negative one would never return");
static_assert(pollTimeoutMs(true) == 0,
              "with frames to draw the loop must return immediately and draw them");
static_assert(!pollHasData(kPollWake) && !pollHasData(kPollCallback) &&
                  !pollHasData(kPollTimeout) && !pollHasData(kPollError),
              "none of the negative poll results carries an event");

}  // namespace mumbai::app
