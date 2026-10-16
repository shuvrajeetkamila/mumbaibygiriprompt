// =============================================================================
//  MUMBAI 1:1 EXACT — tests/test_app_loop.cpp
//
//  THE BUG THIS FILE IS FOR, because it is the one that put a black screen on a
//  phone and kept it there: the client's main loop drained the event queue with
//  `ALooper_pollOnce(-1, ...)`.  A negative timeout means "wait for ever", and
//  `pollOnce` only ever returns a negative code on timeout/wake/callback/error — so
//  the drain loop's exit condition was unreachable, and every line after it was dead
//  code on a real device: the first frame, the fallback screen that paints the
//  reason, the renderer's start and retry, and the log file's writer.  Result: an
//  app that is running, holding a black window, and reporting nothing at all.
//
//  Nothing on a host can observe the looper, so what is tested here is the policy
//  the loop is required to obey (engine/app/poll_policy.h).  It is a small thing to
//  assert — and it is the whole difference between a client that draws and one that
//  does not.
// =============================================================================
#include "engine/app/poll_policy.h"
#include "tests/test_harness.h"

using namespace mumbai;

TEST(theMainLoopNeverWaitsForEverAndAlwaysReturnsToDraw) {
    // The state a freshly launched client is in.  Bounded, and long enough to be
    // cheap when nothing is happening: this is the one that must come back.
    CHECK(app::pollTimeoutMs(false) > 0);
    CHECK(app::pollTimeoutMs(false) <= 32);
    CHECK_EQ(app::pollTimeoutMs(false), app::kIdlePollMs);

    // And once there are frames to draw, it must not sleep at all: the loop runs at
    // the display's rate instead of waiting for input that may never come.
    CHECK_EQ(app::pollTimeoutMs(true), 0);

    // The rule itself, in the form the bug violated: a negative timeout can never
    // expire, so no state may ask for one.
    CHECK(app::pollTimeoutMs(false) >= 0);
    CHECK(app::pollTimeoutMs(true) >= 0);
}

TEST(theMainLoopTreatsEveryNegativePollResultAsAnEmptyQueue) {
    // All four are "no event source for you", and all four have to leave the drain
    // loop rather than keep polling: treating POLL_WAKE or POLL_CALLBACK as data
    // would busy-loop, and treating POLL_TIMEOUT as data would too.
    CHECK_FALSE(app::pollHasData(app::kPollWake));
    CHECK_FALSE(app::pollHasData(app::kPollCallback));
    CHECK_FALSE(app::pollHasData(app::kPollTimeout));
    CHECK_FALSE(app::pollHasData(app::kPollError));

    // The positive results are the event sources the loop has to process.
    CHECK(app::pollHasData(1));   // LOOPER_ID_MAIN: the glue's commands
    CHECK(app::pollHasData(2));   // LOOPER_ID_INPUT: touches and keys
}
