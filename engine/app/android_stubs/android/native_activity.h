// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/native_activity.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule.
//
//  NOW VERBATIM, INCLUDING THE THREE JAVA MEMBERS.  The NDK's ANativeActivity begins
//  with `JavaVM* vm; JNIEnv* env; jobject clazz;`, and this shim used to make those
//  three `void*` to keep <jni.h> out of a host compile — which was fine while the
//  client never touched them.  It does now: engine/app/android_http.cpp looks the
//  app's own class up through the *activity's* class loader (the only loader that can
//  see it from a native thread, D-054), so `clazz` is passed to JNI functions that
//  take a jobject.  The shim includes its own <jni.h> instead, and the rule these
//  stubs exist for holds again: the same names with the same types, so a mismatch is
//  a compile error here rather than a silent failure on a phone.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_NATIVE_ACTIVITY_H
#define MUMBAI_ANDROID_STUBS_NATIVE_ACTIVITY_H

#include <stdint.h>

#include <jni.h>

#include <android/asset_manager.h>
#include <android/looper.h>
#include <android/native_window.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ANativeActivity;
struct ANativeActivityCallbacks;
typedef struct ANativeActivity ANativeActivity;

struct ANativeActivity {
    struct ANativeActivityCallbacks* callbacks;

    // The NDK's own types, spelled the same way (see the header comment).
    JavaVM* vm;
    JNIEnv* env;
    jobject clazz;

    // The path the engine's own world data is installed into, and the APK's asset
    // manager.  These two are the client's entire relationship with the platform
    // besides the window.
    const char* internalDataPath;
    const char* externalDataPath;
    int32_t sdkVersion;
    void* instance;
    AAssetManager* assetManager;
    const char* obbPath;
};

void ANativeActivity_finish(ANativeActivity* activity);
void ANativeActivity_setWindowFormat(ANativeActivity* activity, int32_t format);
void ANativeActivity_setWindowFlags(ANativeActivity* activity, uint32_t addFlags,
                                    uint32_t removeFlags);
void ANativeActivity_showSoftInput(ANativeActivity* activity, uint32_t flags);
void ANativeActivity_hideSoftInput(ANativeActivity* activity, uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_NATIVE_ACTIVITY_H
