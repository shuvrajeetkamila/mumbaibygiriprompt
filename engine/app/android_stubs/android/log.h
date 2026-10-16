// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/log.h
//
//  A HOST SHIM, NOT THE NDK.  This header exists so that engine/app/
//  android_main.cpp — the file that is compiled for the NDK only inside CI — can
//  also be compiled on a machine that has no Android SDK, which is where this
//  project is developed.  Without it a typo in the Android client costs a full CI
//  round trip (and CI is where the only Android toolchain lives).
//
//  The rule for everything in android_stubs/: the NAMES, TYPES and ARGUMENT ORDER
//  here are copied from the NDK's own headers (android/log.h, android/
//  asset_manager.h, android/input.h, android/looper.h, android/native_activity.h,
//  android/native_window.h, android_native_app_glue.h, NDK r26).  What is
//  deliberately absent is the implementation and the parts of the platform the
//  client never touches.  A shim that "fixes" a call site by inventing a different
//  signature would defeat the whole point, so the signatures are the contract.
//
//  This directory is on the include path ONLY for the `make android-check` target,
//  which compiles the client with -D__ANDROID__ -isystem engine/app/android_stubs.
//  It is never used by the Android build: the NDK's own headers come first there.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_LOG_H
#define MUMBAI_ANDROID_STUBS_LOG_H

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

// android_LogPriority, as declared by <android/log.h>.  The numeric values are the
// platform's, not made up: the client passes them straight to liblog on device.
typedef enum android_LogPriority {
    ANDROID_LOG_UNKNOWN = 0,
    ANDROID_LOG_DEFAULT,    // only for SetMinPriority()
    ANDROID_LOG_VERBOSE,
    ANDROID_LOG_DEBUG,
    ANDROID_LOG_INFO,
    ANDROID_LOG_WARN,
    ANDROID_LOG_ERROR,
    ANDROID_LOG_FATAL,
    ANDROID_LOG_SILENT,     // only for SetMinPriority(); must be last
} android_LogPriority;

// The variadic form carries the compiler's printf checking on the host, which is
// half of why this shim is worth writing: a format mismatch in the client's
// log lines is a warning here instead of a crash on a phone.
int __android_log_print(int prio, const char* tag, const char* fmt, ...)
    __attribute__((__format__(printf, 3, 4)));

int __android_log_vprint(int prio, const char* tag, const char* fmt, va_list ap);

int __android_log_write(int prio, const char* tag, const char* text);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_LOG_H
