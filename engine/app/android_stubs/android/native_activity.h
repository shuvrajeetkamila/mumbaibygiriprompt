// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/native_activity.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule.
//
//  THE ONE PLACE THIS SHIM IS NOT VERBATIM, and why it is harmless: the NDK's
//  ANativeActivity begins with `JavaVM* vm; JNIEnv* env; jobject clazz;`, which
//  would drag <jni.h> onto a machine with no JDK.  Those three members are `void*`
//  here — same size, same positions, same names — and the client never touches
//  them (it goes through NativeActivity precisely so that no Java object is ever
//  in its hands).  Every member it *does* touch is the NDK's own type: the two
//  data paths are `const char*` and the asset manager is a real `AAssetManager*`.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_NATIVE_ACTIVITY_H
#define MUMBAI_ANDROID_STUBS_NATIVE_ACTIVITY_H

#include <stdint.h>

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

    // JavaVM*, JNIEnv*, jobject in the NDK; void* here only to keep <jni.h> out of
    // a host compile.  The client reads `vm` — that is how engine/app/android_http.cpp
    // reaches the platform's HTTP stack — and a `static_cast` from void* is what
    // makes the same line compile under both headers.
    void* vm;
    void* env;
    void* clazz;

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
