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

/// Touching the selected entry or its PLUS affordance launches Fluid Box.
/// The left HOME affordance occupies x < 56 and remains a no-op at home.
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

/// Stable launcher-capture probes, deliberately clear of all affordances:
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
/// transport. Returns false (and logs the transport/configuration error) if the
/// frame cannot be completed. No allocation or panel-handle access occurs.
bool render_launcher(DisplayFrame &frame);

}  // namespace fluid_demo
