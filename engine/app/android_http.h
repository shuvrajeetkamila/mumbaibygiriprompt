// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_http.h
//
//  THE ONE THING THE ENGINE CANNOT DO FOR ITSELF: HTTPS.
//
//  `engine/world/world_download.h` describes a download as a state machine over a
//  `Fetcher`: give me bytes from this offset, here is what you do with them.  On a
//  phone that Fetcher has to speak TLS to github.com, and the engine has no TLS
//  stack, no certificate store, and no business growing either — Android ships both,
//  maintained, in Java.  So the transport is the platform's HTTP stack behind a JNI
//  call, and this header is the whole of the engine's knowledge of that (D-048).
//
//  WHY NOT A SOCKET AND A VENDORED TLS LIBRARY.  It would be four hundred kilobytes
//  of new code to maintain, a certificate store to ship, and a second place where
//  the app can crash on a hostile certificate — to avoid ninety lines of Java.  The
//  actual cost of the choice is that this APK has a dex in it, which is why
//  android/app/src/main/AndroidManifest.xml no longer says `hasCode="false"`.
//
//  THE INTERFACE TAKES AN ANativeActivity AND NOTHING ELSE, so the client can call
//  it without including <jni.h>: on a host build (no NDK) this compiles to a
//  factory that returns nullptr, and the client reports "no transport" on its panel
//  instead of pretending to download.
// =============================================================================
#pragma once
#include "engine/world/world_download.h"

struct ANativeActivity;

namespace mumbai::app {

// Creates the Android transport for this activity, or nullptr when there is no
// Java VM to talk to (the host build, a NativeActivity that arrived without one).
// The caller owns it: `destroyAndroidFetcher` at shutdown.
world::Fetcher* createAndroidFetcher(struct ANativeActivity* activity);
void destroyAndroidFetcher(world::Fetcher* fetcher);

// -----------------------------------------------------------------------------
//  THE TWO THINGS THAT ONLY JAVA CAN DO
// -----------------------------------------------------------------------------
// Both exist for the same reason, and it is a failure this project has already had
// once: a phone showed a black screen and the only thing anybody could say about it
// was "nothing here" (D-045, D-054).  A log that is only reachable over adb is not a
// log a person can send, and a device's own verdict about Vulkan is not something
// the native side can ask without the platform's installer machinery.
//
// Both return an empty string when there is no Java to talk to (the host build, an
// activity without a VM) or when the call fails — the client treats "nothing to
// report" as nothing to report, and never blocks on either.

// One line naming the device, its Android version, its ABI, and whether it reports
// Vulkan 1.1.  Empty on a host.
std::string androidSystemInfo(struct ANativeActivity* activity);
// Writes `text` where a person can find it with a file manager (Downloads, with the
// app's own external directory as a fallback) and returns where it went.  Empty when
// it could not be written at all.
std::string androidWriteLog(struct ANativeActivity* activity, const std::string& text);

}  // namespace mumbai::app
