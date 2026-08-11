#pragma once

#include <cstdint>

#include "app_shell.hpp"

namespace fluid_demo {

/// Fixed 240x240 launcher raster geometry. Coordinates and probe boxes use
/// half-open ranges: [left, right) x [top, bottom).
inline constexpr int kLauncherWidth = 240;
inline constexpr int kLauncherHeight = 240;

/// Logical RGB565 colors. render_launcher() byte-swaps each selected logical
/// color exactly once at pixel store so DisplayService receives RGB565BE wire
/// order and capture bytes match the panel transfer.
inline constexpr uint16_t kLauncherBackgroundRgb565 = 0x10C3;  // ink: #101819
inline constexpr uint16_t kLauncherBandRgb565 = 0x21A7;        // slate: #21343A
inline constexpr uint16_t kLauncherGlyphRgb565 = 0xEF1A;       // warm ivory
inline constexpr uint16_t kLauncherAccentRgb565 = 0xF50A;      // warm amber
inline constexpr uint16_t kLauncherAffordanceRgb565 = 0x8493;  // muted steel

/// The sole selected-entry band occupies every x on rows [56, 184).
inline constexpr int kLauncherBandTop = 56;
inline constexpr int kLauncherBandBottom = 184;

/// A non-swipe release inside the selected-entry box launches the currently
/// selected registry entry (Fluid Box by default). Swipes anywhere navigate
/// and never launch; the fixed chevrons and page dots are pure visual hints.
inline constexpr int kLauncherLaunchTouchLeft = 56;
inline constexpr int kLauncherLaunchTouchRight = kLauncherWidth;

constexpr bool launcher_accepts_launch_touch(uint16_t x, uint16_t y)
{
    return x >= kLauncherLaunchTouchLeft && x < kLauncherLaunchTouchRight &&
           y >= kLauncherBandTop && y < kLauncherBandBottom;
}

static_assert(launcher_accepts_launch_touch(120, 120));
static_assert(!launcher_accepts_launch_touch(32, 120));
static_assert(!launcher_accepts_launch_touch(120, 32));

/// Minimal dominant-horizontal travel (px) for the software swipe fallback.
inline constexpr int kLauncherSwipeMinPx = 32;

/// Classify one ended contact as a launcher gesture: the controller-reported
/// gesture wins when it is not None; otherwise a dominant horizontal travel
/// of at least kLauncherSwipeMinPx (|dx| >= 32 and |dx| >= |dy|) maps to
/// SwipeLeft/SwipeRight, and everything else is a tap (None). Pure and
/// allocation-free so the sensor lane calls it directly on the End event.
constexpr TouchGesture launcher_swipe_gesture(TouchGesture controller,
                                              uint16_t x0, uint16_t y0,
                                              uint16_t x1, uint16_t y1)
{
    if (controller != TouchGesture::None) {
        return controller;
    }
    const int dx = static_cast<int>(x1) - static_cast<int>(x0);
    const int dy = static_cast<int>(y1) - static_cast<int>(y0);
    const int adx = dx < 0 ? -dx : dx;
    const int ady = dy < 0 ? -dy : dy;
    if (adx >= kLauncherSwipeMinPx && adx > ady) {
        return dx < 0 ? TouchGesture::SwipeLeft : TouchGesture::SwipeRight;
    }
    return TouchGesture::None;
}

static_assert(launcher_swipe_gesture(TouchGesture::SwipeLeft, 0, 0, 0, 0) ==
              TouchGesture::SwipeLeft,  // controller gesture wins verbatim
              "controller-reported gesture must win over the fallback");
static_assert(launcher_swipe_gesture(TouchGesture::None, 10, 10, 60, 12) ==
              TouchGesture::SwipeRight,  // rightward dominant travel
              "dominant rightward travel must classify as SwipeRight");
static_assert(launcher_swipe_gesture(TouchGesture::None, 60, 12, 10, 10) ==
              TouchGesture::SwipeLeft,  // leftward dominant travel
              "dominant leftward travel must classify as SwipeLeft");
static_assert(launcher_swipe_gesture(TouchGesture::None, 10, 10, 30, 10) ==
              TouchGesture::None,  // |dx| below threshold
              "short horizontal travel must stay a tap");
static_assert(launcher_swipe_gesture(TouchGesture::None, 10, 10, 60, 60) ==
              TouchGesture::None,  // diagonal: |dx| == |dy|, not dominant
              "non-dominant horizontal travel must stay a tap");

/// Centered 8x8 Fluid Box vessel glyph. Each set bit is an 8x8 pixel cell, so
/// the exact glyph bounding box is [88, 152) x [88, 152). Bit 7 is the left
/// cell. Unset cells show the band color.
inline constexpr int kLauncherGlyphLeft = 88;
inline constexpr int kLauncherGlyphTop = 88;
inline constexpr int kLauncherGlyphScale = 8;
inline constexpr int kLauncherGlyphExtent = 8 * kLauncherGlyphScale;
inline constexpr uint8_t kLauncherGlyphBitmap[8] = {
    0b11111111,
    0b10000001,
    0b10000001,
    0b10011001,
    0b10100101,
    0b11000011,
    0b11111111,
    0b11111111,
};

/// Exact solid-color capture probe. `logical_rgb565` is pre-wire-swap.
struct LauncherProbeBox {
    int left;
    int top;
    int right;
    int bottom;
    uint16_t logical_rgb565;
};

/// Stable launcher-capture probes for the default built-in Fluid launcher
/// (null descriptor), deliberately clear of all affordances:
/// - background: [8, 16) x [8, 16), kLauncherBackgroundRgb565
/// - selected band: [8, 16) x [112, 120), kLauncherBandRgb565
/// - glyph: [88, 96) x [88, 96), kLauncherGlyphRgb565
inline constexpr LauncherProbeBox kLauncherBackgroundProbe{
    8, 8, 16, 16, kLauncherBackgroundRgb565};
inline constexpr LauncherProbeBox kLauncherBandProbe{
    8, 112, 16, 120, kLauncherBandRgb565};
inline constexpr LauncherProbeBox kLauncherGlyphProbe{
    88, 88, 96, 96, kLauncherGlyphRgb565};

/// Render one complete launcher frame through DisplayFrame's shell-owned
/// transport for the selected app's launcher visual. `selected_index` and
/// `registry_count` drive the fixed page dots (how many entries exist and
/// which one is selected); `visual` supplies the selected entry's palette and
/// fixed 8x8 glyph bitmaps (null renders the exact built-in Fluid Box
/// launcher: logical colors, glyph, capture probes untouched; a non-null
/// descriptor renders at the same geometry with the secondary bitmap drawn
/// with priority over the primary). Fixed left/right swipe chevrons and the
/// page-dot row make swipeability visible for every selection without text or
/// allocation, so capture probes and touch geometry stay meaningful. Transport
/// order is identical for every visual. Returns false (and logs the
/// transport/configuration error) if the frame cannot be completed. No
/// allocation or panel-handle access occurs.
bool render_launcher(DisplayFrame &frame, const LauncherVisual *visual,
                     uint32_t selected_index, uint32_t registry_count);

}  // namespace fluid_demo
