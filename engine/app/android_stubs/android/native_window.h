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

// android/rect.h, in the NDK, and included by <android/native_window.h>.
typedef struct ARect {
    int32_t left;
    int32_t top;
    int32_t right;
    int32_t bottom;
} ARect;

// The buffer a locked window hands back.  `stride` is in *pixels*, not bytes:
// the CPU fallback screen multiplies by four before it writes, and getting that
// wrong writes diagonal stripes across the phone.
typedef struct ANativeWindow_Buffer {
    int32_t width;
    int32_t height;
    int32_t stride;
    int32_t format;
    void* bits;
    uint32_t reserved[6];
} ANativeWindow_Buffer;

int ANativeWindow_getWidth(ANativeWindow* window);
int ANativeWindow_getHeight(ANativeWindow* window);
int ANativeWindow_getFormat(ANativeWindow* window);
int32_t ANativeWindow_setBuffersGeometry(ANativeWindow* window, int32_t width, int32_t height,
                                         int32_t format);
int32_t ANativeWindow_lock(ANativeWindow* window, ANativeWindow_Buffer* outBuffer,
                           ARect* inOutDirtyBounds);
int32_t ANativeWindow_unlockAndPost(ANativeWindow* window);

#ifdef __cplusplus
}
#endif

#endif  // MUMBAI_ANDROID_STUBS_NATIVE_WINDOW_H
