#pragma once

#include <cstdint>

#include "app_shell.hpp"

namespace fluid_demo {

inline constexpr int kLauncherWidth = 240;
inline constexpr int kLauncherHeight = 240;

// Logical RGB565; render_launcher swaps once when writing wire-order pixels.
inline constexpr uint16_t kLauncherBackgroundRgb565 = 0x10C3;
inline constexpr uint16_t kLauncherBandRgb565 = 0x21A7;
inline constexpr uint16_t kLauncherGlyphRgb565 = 0xEF1A;
inline constexpr uint16_t kLauncherAccentRgb565 = 0xF50A;
inline constexpr uint16_t kLauncherAffordanceRgb565 = 0x8493;

inline constexpr int kLauncherBandTop = 56;
inline constexpr int kLauncherBandBottom = 184;

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

inline constexpr int kLauncherSwipeMinPx = 32;

constexpr TouchGesture launcher_swipe_gesture(TouchGesture controller_gesture,
                                              uint16_t start_x,
                                              uint16_t start_y, uint16_t end_x,
                                              uint16_t end_y)
{
    if (controller_gesture != TouchGesture::None) {
        return controller_gesture;
    }
    const int horizontal_delta =
        static_cast<int>(end_x) - static_cast<int>(start_x);
    const int vertical_delta =
        static_cast<int>(end_y) - static_cast<int>(start_y);
    const int horizontal_distance =
        horizontal_delta < 0 ? -horizontal_delta : horizontal_delta;
    const int vertical_distance =
        vertical_delta < 0 ? -vertical_delta : vertical_delta;
    if (horizontal_distance >= kLauncherSwipeMinPx &&
        horizontal_distance > vertical_distance) {
        return horizontal_delta < 0 ? TouchGesture::SwipeLeft
                                    : TouchGesture::SwipeRight;
    }
    return TouchGesture::None;
}

static_assert(launcher_swipe_gesture(TouchGesture::SwipeLeft, 0, 0, 0, 0) ==
              TouchGesture::SwipeLeft);
static_assert(launcher_swipe_gesture(TouchGesture::None, 10, 10, 60, 12) ==
              TouchGesture::SwipeRight);
static_assert(launcher_swipe_gesture(TouchGesture::None, 60, 12, 10, 10) ==
              TouchGesture::SwipeLeft);
static_assert(launcher_swipe_gesture(TouchGesture::None, 10, 10, 30, 10) ==
              TouchGesture::None);
static_assert(launcher_swipe_gesture(TouchGesture::None, 10, 10, 60, 60) ==
              TouchGesture::None);

inline constexpr int kLauncherGlyphLeft = 88;
inline constexpr int kLauncherGlyphTop = 88;
inline constexpr int kLauncherGlyphScale = 8;
inline constexpr int kLauncherGlyphExtent = 8 * kLauncherGlyphScale;
inline constexpr int kLauncherIconSize = kLauncherGlyphExtent;
inline constexpr int kLauncherIconPixels =
    kLauncherIconSize * kLauncherIconSize;
inline constexpr uint16_t kLauncherIconTransparent = 0xF81F;
inline constexpr uint8_t kLauncherGlyphBitmap[8] = {
    0b11111111, 0b10000001, 0b10000001, 0b10011001,
    0b10100101, 0b11000011, 0b11111111, 0b11111111,
};

struct LauncherProbeBox {
    int left;
    int top;
    int right;
    int bottom;
    uint16_t logical_rgb565;
};

inline constexpr LauncherProbeBox kLauncherBackgroundProbe{
    8, 8, 16, 16, kLauncherBackgroundRgb565};
inline constexpr LauncherProbeBox kLauncherBandProbe{8, 112, 16, 120,
                                                     kLauncherBandRgb565};
inline constexpr LauncherProbeBox kLauncherGlyphProbe{88, 88, 96, 96,
                                                      kLauncherGlyphRgb565};

bool render_launcher(DisplayFrame &frame, const LauncherVisual *visual,
                     uint32_t selected_index, uint32_t registry_count);

}
