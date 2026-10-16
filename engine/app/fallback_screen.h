// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/fallback_screen.h
//
//  THE SCREEN THAT CANNOT BE BLACK.
//
//  The first APK this project put on a phone showed a black screen, and the whole
//  report a person could make was "nothing here".  The GPU panel (engine/render/
//  hud.h) fixes that once there is a frame to draw it in — but the cases where the
//  panel has no frame are exactly the cases a person needs to hear about: the
//  device refused a Vulkan instance, the surface arrived without a swapchain, the
//  app is still copying its world out of the APK, the renderer threw and retried.
//
//  So the client keeps one path that needs no graphics API at all: lock the
//  window's own buffer, paint it on the CPU with the same 5x7 font the panel uses,
//  and post it.  It is not pretty and it is not the product; it is the difference
//  between "black" and a sentence that names what is wrong.
//
//  Android only.  On a host there is no ANativeWindow, and the file compiles to
//  nothing so that `make android-check` can still include it in the compile.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <string>
#include <vector>

#if defined(__ANDROID__)
#include <android/native_window.h>
#endif

namespace mumbai::app {

// Paints `title` and `lines` into the window and posts it.  Returns false when the
// window could not be locked or has no buffer — which the caller reports by
// logging, since there is by definition nothing left to draw on.
#if defined(__ANDROID__)
bool drawFallbackScreen(ANativeWindow* window, const std::string& title,
                        const std::vector<std::string>& lines);
#else
inline bool drawFallbackScreen(void*, const std::string&, const std::vector<std::string>&) {
    return false;
}
#endif

}  // namespace mumbai::app
