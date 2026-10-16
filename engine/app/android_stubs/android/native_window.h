// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/android_stubs/android/native_window.h
//
//  A HOST SHIM, NOT THE NDK.  See android/log.h in this directory for the rule.
//  The client hands this pointer straight to the renderer, which is what creates
//  the VkSurfaceKHR — so on the host the type is opaque and only its two getters
//  exist, exactly as in the NDK.
// =============================================================================
#ifndef MUMBAI_ANDROID_STUBS_NATIVE_WINDOW_H
#define MUMBAI_ANDROID_STUBS_NATIVE_WINDOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ANativeWindow;
typedef struct ANativeWindow ANativeWindow;

// Pixel formats the NDK declares; the client never names one, but a shim that
// omits the enum invites `#ifdef` guesswork later.
enum ANativeWindow_LegacyFormat {
    WINDOW_FORMAT_RGBA_8888 = 1,
    WINDOW_FORMAT_RGBX_8888 = 2,
    WINDOW_FORMAT_RGB_565 = 4,
};

int ANativeWindow_getWidth(ANativeWindow* window);
int ANativeWindow_getHeight(ANativeWindow* window);
int ANativeWindow_getFormat(ANativeWindow* window);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_NATIVE_WINDOW_H
