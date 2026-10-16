// =============================================================================
//  MUMBAI 1:1 EXACT — engine/app/fallback_screen.cpp
//
//  See the header for why this exists.  In one line: the client must be able to
//  say what is wrong on a screen that has no graphics device behind it.
//
//  HOW IT PAINTS.  A locked ANativeWindow hands back a mapped pixel buffer — the
//  same ANativeWindow that the renderer would have wrapped in a VkSurfaceKHR, so
//  this path is only ever used while no surface exists for Vulkan.  Painting is
//  two passes: the font rasteriser (engine/render/hud.cpp) writes into an
//  eight-bit coverage buffer, and the coverage is expanded into the window's own
//  RGBA pixels over a dark gradient.  It runs at most a few times a second, on a
//  screen that is by definition drawing nothing else.
// =============================================================================
#include "engine/app/fallback_screen.h"

#if defined(__ANDROID__)

#include "engine/render/hud.h"

#include <android/native_window.h>

#include <algorithm>
#include <vector>

namespace mumbai::app {
namespace {

using namespace mumbai::render;

// The palette, as RGB bytes: the expansion below writes RGBA_8888 pixels, which
// is what an Android window's buffer holds.
constexpr u8 kBackgroundTop[3] = {10, 16, 20};
constexpr u8 kBackgroundBottom[3] = {4, 6, 10};
constexpr u8 kTitleColour[3] = {235, 240, 245};
constexpr u8 kTextColour[3] = {200, 208, 216};
constexpr u8 kBadColour[3] = {240, 120, 110};

// Coverage values: what the rasteriser is told to write, and what the expansion
// reads back.  Two numbers, used as a vocabulary rather than as magic.
constexpr u8 kBodyCover = 1;
constexpr u8 kTitleCover = 2;
constexpr u8 kBadCover = 3;

u8 lerpChannel(u8 a, u8 b, u32 t) {
    return static_cast<u8>((static_cast<u32>(a) * (255u - t) + static_cast<u32>(b) * t) / 255u);
}

void writePixel(u8* pixel, const u8 rgb[3]) {
    pixel[0] = rgb[0];
    pixel[1] = rgb[1];
    pixel[2] = rgb[2];
    pixel[3] = 255;
}

// A line is flagged as a failure by its own text, so the caller writes plain
// sentences and this decides how to colour them.  The marker is deliberately the
// same one the log uses for bad news.
bool isBadLine(const std::string& line) {
    return line.rfind("!!", 0) == 0 || line.find("failed") != std::string::npos ||
           line.find("did not start") != std::string::npos;
}

}  // namespace

bool drawFallbackScreen(ANativeWindow* window, const std::string& title,
                        const std::vector<std::string>& lines) {
    if (!window) return false;
    const i32 width = ANativeWindow_getWidth(window);
    const i32 height = ANativeWindow_getHeight(window);
    if (width <= 0 || height <= 0) return false;

    // Ask for a format that is four bytes per pixel; if the platform refuses, the
    // buffer's format is still four bytes per pixel on every device this ships to
    // and the writes below are what an RGBA_8888 window reads.
    ANativeWindow_setBuffersGeometry(window, width, height, WINDOW_FORMAT_RGBA_8888);

    ANativeWindow_Buffer buffer{};
    ARect dirty{0, 0, width, height};
    if (ANativeWindow_lock(window, &buffer, &dirty) != 0) return false;
    if (!buffer.bits || buffer.width <= 0 || buffer.height <= 0) {
        ANativeWindow_unlockAndPost(window);
        return false;
    }

    const i32 w = buffer.width;
    const i32 h = buffer.height;
    const i32 stridePixels = buffer.stride > 0 ? buffer.stride : w;

    // One coverage buffer, kept between calls: at a few hertz the allocation would
    // not be noticed, but the fallback screen should not be the thing that
    // fragments a heap on a phone that is already unhappy.
    static std::vector<u8> coverage;
    coverage.assign(static_cast<size_t>(w) * static_cast<size_t>(h), 0);

    // The type scale is chosen so the longest line fits: six pixels per character
    // (five of glyph, one of advance) and a margin on each side.
    size_t longest = title.size();
    for (const std::string& line : lines) longest = std::max(longest, line.size());
    const i32 margin = std::max(16, w / 24);
    i32 scale = 2;
    if (longest > 0) scale = std::clamp((w - 2 * margin) / static_cast<i32>(longest) / 6, 2, 8);
    const u32 type = static_cast<u32>(scale);

    i32 penY = margin + 4;
    blitText(coverage.data(), w, w, h, margin, penY, title, type, kTitleCover);
    penY += textPixelHeight(type) + scale * 6;

    for (const std::string& line : lines) {
        if (penY + textPixelHeight(type) > h - margin) break;
        blitText(coverage.data(), w, w, h, margin, penY, line, type,
                 isBadLine(line) ? kBadCover : kBodyCover);
        penY += textPixelHeight(type) + scale * 2;
    }

    // Expand: a vertical gradient for the background, then the coverage as one of
    // three foreground colours.
    for (i32 y = 0; y < h; ++y) {
        const u32 t = static_cast<u32>((static_cast<i64>(y) * 255) / std::max(1, h - 1));
        const u8 background[3] = {lerpChannel(kBackgroundTop[0], kBackgroundBottom[0], t),
                                  lerpChannel(kBackgroundTop[1], kBackgroundBottom[1], t),
                                  lerpChannel(kBackgroundTop[2], kBackgroundBottom[2], t)};
        u8* row = static_cast<u8*>(buffer.bits) +
                  static_cast<size_t>(y) * static_cast<size_t>(stridePixels) * 4u;
        const u8* coverRow = coverage.data() + static_cast<size_t>(y) * static_cast<size_t>(w);
        for (i32 x = 0; x < w; ++x) {
            const u8 cover = coverRow[x];
            const u8* colour = background;
            if (cover == kTitleCover) {
                colour = kTitleColour;
            } else if (cover == kBadCover) {
                colour = kBadColour;
            } else if (cover == kBodyCover) {
                colour = kTextColour;
            }
            writePixel(row + static_cast<size_t>(x) * 4u, colour);
        }
    }

    return ANativeWindow_unlockAndPost(window) == 0;
}

}  // namespace mumbai::app

#endif  // __ANDROID__
