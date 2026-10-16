// =============================================================================
//  MUMBAI 1:1 EXACT — engine/render/hud.h
//
//  THE ON-SCREEN PANEL, AND WHY IT EXISTS BEFORE ANYTHING PRETTY.
//
//  A phone cannot tell you what went wrong.  The first APK this project produced
//  drew a black screen on a real device, and that single fact was the whole
//  debugging channel: no frame counter, no reason, no "the world is missing".
//  The log said it, and the screen did not, and the person holding the phone is
//  not holding a serial console.  So the panel is not a debug leftover — it is
//  the client's only way to report itself, and it is built here, before the
//  city, so that every later failure arrives with an explanation attached.
//
//  It is deliberately *not* a font library.  A 5x7 bitmap font, 95 glyphs, one
//  96x48 single-channel atlas, every glyph a quad per lit pixel.  A line of 40
//  characters is therefore a few thousand vertices, which is nothing next to one
//  terrain chunk, and it needs no asset, no shaping, no kerning and no library
//  that a phone has to trust.
//
//  Everything in this header is CPU-only and pure: `buildFontAtlas` and
//  `buildHudVertices` take values and return bytes.  That is on purpose — the
//  panel is verified by `tests/test_render.cpp` on a machine with no GPU, which
//  is the only kind of machine this project's CI runs on.
// =============================================================================
#pragma once
#include "engine/core/core.h"

#include <string>
#include <vector>

namespace mumbai::render {

// --- font geometry -----------------------------------------------------------
// 5x7 glyphs in 6x8 cells: one pixel of advance and one of leading, so a glyph
// never touches its neighbour even at an integer scale.
inline constexpr u32 kGlyphW = 5;
inline constexpr u32 kGlyphH = 7;
inline constexpr u32 kCellW = 6;
inline constexpr u32 kCellH = 8;
inline constexpr u32 kAtlasCols = 16;
inline constexpr u32 kAtlasRows = 6;
inline constexpr u32 kAtlasW = kAtlasCols * kCellW;   // 96
inline constexpr u32 kAtlasH = kAtlasRows * kCellH;   // 48
inline constexpr u32 kFirstGlyph = 32;                // space
inline constexpr u32 kGlyphCount = 95;                // 32..126
// Cell 95 is the one cell the 95 glyphs do not use, and it is filled solid: a
// rectangle is then just two triangles that sample it, so panels, the joystick
// ring and the mode buttons need no second pipeline and no second shader.
inline constexpr u32 kSolidCell = kGlyphCount;
inline constexpr u32 kSolidU0 = (kSolidCell % kAtlasCols) * kCellW;
inline constexpr u32 kSolidV0 = (kSolidCell / kAtlasCols) * kCellH;

// R8, row-major, top-left origin.  `kAtlasW * kAtlasH` bytes.
void buildFontAtlas(std::vector<u8>& out);

// --- what the client draws ---------------------------------------------------
// Colours are 0xAABBGGRR: the byte order the vertex format carries, spelled out
// here so a call site reads as `0xFF00FF00` (opaque green) and not as a puzzle.
inline constexpr u32 rgba(u8 r, u8 g, u8 b, u8 a = 255) {
    return static_cast<u32>(r) | (static_cast<u32>(g) << 8) | (static_cast<u32>(b) << 16) |
           (static_cast<u32>(a) << 24);
}
inline constexpr u32 kHudWhite = rgba(235, 240, 245);
inline constexpr u32 kHudGreen = rgba(120, 230, 140);
inline constexpr u32 kHudAmber = rgba(245, 190, 90);
inline constexpr u32 kHudRed = rgba(240, 110, 100);
inline constexpr u32 kHudPanel = rgba(8, 10, 14, 140);

struct HudLine {
    f32 x = 8.0f;           // pixels from the left edge
    f32 y = 8.0f;           // pixels from the top edge
    f32 scale = 2.0f;       // 2 means a glyph is 10x14 pixels
    u32 colour = kHudWhite;
    std::string text;
};

struct HudRect {
    f32 x = 0.0f, y = 0.0f, w = 0.0f, h = 0.0f;
    u32 colour = kHudPanel;
};

struct HudFrame {
    std::vector<HudRect> rects;   // drawn first, behind the text
    std::vector<HudLine> lines;
};

// The vertex the HUD pipeline consumes: pixels, atlas uv, colour.
struct HudVertex {
    f32 x, y;
    f32 u, v;
    u32 rgba;
};
static_assert(sizeof(HudVertex) == 20, "the HUD vertex layout is part of the pipeline state");

// Appends the frame's geometry to `out` and returns the number of vertices
// appended.  `viewportW`/`viewportH` are not used for positioning (everything is
// already in pixels); they are the clamp that keeps a panel off the edge of a
// screen that is smaller than the layout assumed.
u32 buildHudVertices(const HudFrame& frame, f32 viewportW, f32 viewportH,
                     std::vector<HudVertex>& out);

// Number of pixels one line of text covers, for a panel that has to size itself
// around its contents rather than a hardcoded width.
f32 hudTextWidth(const std::string& text, f32 scale);
f32 hudLineHeight(f32 scale);

// --- the same font, drawn by the CPU -----------------------------------------
// WHY THERE ARE TWO RASTERISERS AND ONE FONT.  When the renderer is not up — the
// device refused a graphics device, the surface arrived without a swapchain, the
// app is still installing its world — there is no frame to put a panel in, and a
// black rectangle is all a person holding the phone gets.  The client therefore
// paints its own startup and failure screens straight into the window's buffer
// with these functions.  One font, two rasterisers: a second font would drift
// from the first, and a screen that says "the renderer did not start" is worthless
// if it cannot be read.
//
// The target is an 8-bit coverage buffer: row-major, top-left origin, `stride`
// bytes per row (the stride a locked Android window hands back is in *pixels*,
// so a caller with a 4-byte-per-pixel window multiplies before calling).
//
// Every blit is clipped to `clipW` x `clipH`, which the caller takes from the
// locked window's own width and height: a rasteriser that trusts its caller to
// stay in bounds is a rasteriser that writes past the end of a mapped buffer.
u8 glyphRow(char c, u32 row);   // 5-bit mask, bit 4 leftmost; 0 for a non-glyph
void blitRect(u8* pixels, i32 stride, i32 clipW, i32 clipH, i32 x, i32 y, i32 w, i32 h,
              u8 value);
void blitText(u8* pixels, i32 stride, i32 clipW, i32 clipH, i32 x, i32 y,
              const std::string& text, u32 scale, u8 value);
i32 textPixelWidth(const std::string& text, u32 scale);
i32 textPixelHeight(u32 scale);

}  // namespace mumbai::render
