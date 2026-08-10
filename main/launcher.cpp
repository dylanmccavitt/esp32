#include "launcher.hpp"

#include "esp_err.h"
#include "esp_log.h"

namespace fluid_demo {
namespace {

constexpr char kTag[] = "launcher";

// The left home outline identifies the shell/home destination reached by PWR.
// The right plus is the physical PLUS launch affordance. Both stay secondary
// to the centered Fluid Box glyph and use only fixed, scale-aligned geometry.
constexpr int kHomeCenterX = 32;
constexpr int kHomeRoofTop = 104;
constexpr int kHomeRoofHalfWidth = 12;
constexpr int kHomeWallTop = 116;
constexpr int kHomeBottom = 136;
constexpr int kHomeStroke = 2;

constexpr int kPlusCenterX = 208;
constexpr int kPlusCenterY = 120;
constexpr int kPlusHalfLength = 8;
constexpr int kPlusHalfStroke = 2;

constexpr int abs_int(int value)
{
    return value < 0 ? -value : value;
}

constexpr bool glyph_pixel(int x, int y)
{
    if (x < kLauncherGlyphLeft || x >= kLauncherGlyphLeft + kLauncherGlyphExtent ||
        y < kLauncherGlyphTop || y >= kLauncherGlyphTop + kLauncherGlyphExtent) {
        return false;
    }
    const int row = (y - kLauncherGlyphTop) / kLauncherGlyphScale;
    const int column = (x - kLauncherGlyphLeft) / kLauncherGlyphScale;
    const uint8_t mask = static_cast<uint8_t>(0x80u >> column);
    return (kLauncherGlyphBitmap[row] & mask) != 0;
}

constexpr bool home_pixel(int x, int y)
{
    const int dx = abs_int(x - kHomeCenterX);
    const int roof_y = kHomeRoofTop + dx;
    const bool roof = dx <= kHomeRoofHalfWidth && abs_int(y - roof_y) < kHomeStroke;
    const bool left_wall = x >= kHomeCenterX - kHomeRoofHalfWidth &&
                           x < kHomeCenterX - kHomeRoofHalfWidth + kHomeStroke &&
                           y >= kHomeWallTop && y < kHomeBottom;
    const bool right_wall = x > kHomeCenterX + kHomeRoofHalfWidth - kHomeStroke &&
                            x <= kHomeCenterX + kHomeRoofHalfWidth &&
                            y >= kHomeWallTop && y < kHomeBottom;
    const bool floor = x >= kHomeCenterX - kHomeRoofHalfWidth &&
                       x <= kHomeCenterX + kHomeRoofHalfWidth &&
                       y >= kHomeBottom - kHomeStroke && y < kHomeBottom;
    return roof || left_wall || right_wall || floor;
}

constexpr bool plus_pixel(int x, int y)
{
    const bool horizontal = x >= kPlusCenterX - kPlusHalfLength &&
                            x < kPlusCenterX + kPlusHalfLength &&
                            y >= kPlusCenterY - kPlusHalfStroke &&
                            y < kPlusCenterY + kPlusHalfStroke;
    const bool vertical = x >= kPlusCenterX - kPlusHalfStroke &&
                          x < kPlusCenterX + kPlusHalfStroke &&
                          y >= kPlusCenterY - kPlusHalfLength &&
                          y < kPlusCenterY + kPlusHalfLength;
    return horizontal || vertical;
}

constexpr uint16_t logical_color_at(int x, int y)
{
    uint16_t color = kLauncherBackgroundRgb565;
    if (y >= kLauncherBandTop && y < kLauncherBandBottom) {
        color = kLauncherBandRgb565;
    }
    if (home_pixel(x, y)) {
        color = kLauncherAffordanceRgb565;
    }
    if (plus_pixel(x, y)) {
        color = kLauncherAccentRgb565;
    }
    if (glyph_pixel(x, y)) {
        color = kLauncherGlyphRgb565;
    }
    return color;
}

static_assert(logical_color_at(kLauncherBackgroundProbe.left,
                               kLauncherBackgroundProbe.top) ==
              kLauncherBackgroundProbe.logical_rgb565);
static_assert(logical_color_at(kLauncherBandProbe.left, kLauncherBandProbe.top) ==
              kLauncherBandProbe.logical_rgb565);
static_assert(logical_color_at(kLauncherGlyphProbe.left, kLauncherGlyphProbe.top) ==
              kLauncherGlyphProbe.logical_rgb565);

bool fail(const char *stage, esp_err_t error)
{
    ESP_LOGE(kTag, "launcher %s failed: %s", stage, esp_err_to_name(error));
    return false;
}

bool valid_frame(const DisplayFrame &frame)
{
    if (frame.width != kLauncherWidth || frame.height != kLauncherHeight ||
        frame.stripe_rows <= 0 || frame.stripe_count <= 0 ||
        frame.stripe_count !=
            (kLauncherHeight + frame.stripe_rows - 1) / frame.stripe_rows ||
        frame.stripe[0] == nullptr || frame.stripe[1] == nullptr ||
        frame.transport == nullptr) {
        return false;
    }
    return frame.ops.wait_previous != nullptr && frame.ops.latch_capture != nullptr &&
           frame.ops.submit != nullptr && frame.ops.finish != nullptr;
}

}  // namespace

bool render_launcher(DisplayFrame &frame)
{
    if (!valid_frame(frame)) {
        return fail("configuration", ESP_ERR_INVALID_STATE);
    }

    esp_err_t error = frame.ops.wait_previous(frame.transport);
    if (error != ESP_OK) {
        return fail("wait", error);
    }

    // Latch only after the carried final stripe has retired. Every following
    // submit belongs to this one complete deterministic frame.
    static_cast<void>(frame.ops.latch_capture(frame.transport));

    for (int stripe = 0; stripe < frame.stripe_count; ++stripe) {
        const int y0 = stripe * frame.stripe_rows;
        const int remaining = frame.height - y0;
        const int rows = remaining < frame.stripe_rows ? remaining : frame.stripe_rows;
        uint16_t *pixels = frame.stripe[stripe & 1];

        for (int local_y = 0; local_y < rows; ++local_y) {
            const int y = y0 + local_y;
            uint16_t *row = pixels + local_y * frame.width;
            for (int x = 0; x < frame.width; ++x) {
                // This is the sole logical-RGB565 to wire-order conversion.
                row[x] = __builtin_bswap16(logical_color_at(x, y));
            }
        }

        error = frame.ops.submit(frame.transport, stripe, y0, rows, pixels);
        if (error != ESP_OK) {
            return fail("submit", error);
        }
    }

    error = frame.ops.finish(frame.transport);
    if (error != ESP_OK) {
        return fail("finish", error);
    }
    return true;
}

}  // namespace fluid_demo
