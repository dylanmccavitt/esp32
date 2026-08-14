#include "launcher.hpp"

#include "common_math.hpp"
#include "esp_err.h"
#include "esp_log.h"

namespace fluid_demo {
namespace {

constexpr char kTag[] = "launcher";

constexpr int kChevronCenterY = 120;
constexpr int kChevronHalfExtent = 12;
constexpr int kChevronStroke = 2;
constexpr int kChevronLeftApexX = 30;
constexpr int kChevronRightApexX = 178;

constexpr int kPageDotRowY = 208;
constexpr int kPageDotRadius = 2;
constexpr int kPageDotSpacing = 14;

constexpr int kPlusCenterX = 208;
constexpr int kPlusCenterY = 120;
constexpr int kPlusHalfLength = 8;
constexpr int kPlusHalfStroke = 2;

constexpr bool glyph_cell_set(const uint8_t bitmap[8], int x, int y)
{
    if (x < kLauncherGlyphLeft ||
        x >= kLauncherGlyphLeft + kLauncherGlyphExtent ||
        y < kLauncherGlyphTop ||
        y >= kLauncherGlyphTop + kLauncherGlyphExtent) {
        return false;
    }
    const int row = (y - kLauncherGlyphTop) / kLauncherGlyphScale;
    const int column = (x - kLauncherGlyphLeft) / kLauncherGlyphScale;
    const uint8_t mask = static_cast<uint8_t>(0x80u >> column);
    return (bitmap[row] & mask) != 0;
}

constexpr bool glyph_pixel(int x, int y)
{
    return glyph_cell_set(kLauncherGlyphBitmap, x, y);
}

constexpr bool chevron_pixel(int x, int y, int apex_x, int direction)
{
    if (y < kChevronCenterY - kChevronHalfExtent - kChevronStroke ||
        y > kChevronCenterY + kChevronHalfExtent + kChevronStroke) {
        return false;
    }
    const int distance_from_apex = (x - apex_x) * direction;
    if (distance_from_apex < 0 || distance_from_apex > kChevronHalfExtent) {
        return false;
    }
    const bool upper_arm =
        abs_int(y - (kChevronCenterY - distance_from_apex)) < kChevronStroke;
    const bool lower_arm =
        abs_int(y - (kChevronCenterY + distance_from_apex)) < kChevronStroke;
    return upper_arm || lower_arm;
}

constexpr uint32_t effective_page_count(uint32_t registry_count)
{
    return registry_count == 0 ? 1 : registry_count;
}

constexpr bool is_page_dot_pixel(int x, int y, uint32_t registry_count)
{
    if (y < kPageDotRowY - kPageDotRadius ||
        y > kPageDotRowY + kPageDotRadius) {
        return false;
    }
    const uint32_t dot_count = effective_page_count(registry_count);
    const int dot_span = static_cast<int>(dot_count - 1) * kPageDotSpacing;
    const int first_center_x = kLauncherWidth / 2 - dot_span / 2;
    for (uint32_t dot_index = 0; dot_index < dot_count; ++dot_index) {
        const int center_x =
            first_center_x + static_cast<int>(dot_index) * kPageDotSpacing;
        if (abs_int(x - center_x) <= kPageDotRadius) {
            return true;
        }
    }
    return false;
}

constexpr bool is_selected_page_dot_pixel(int x, int y, uint32_t selected_index,
                                          uint32_t registry_count)
{
    if (y < kPageDotRowY - kPageDotRadius ||
        y > kPageDotRowY + kPageDotRadius) {
        return false;
    }
    const uint32_t dot_count = effective_page_count(registry_count);
    const int dot_span = static_cast<int>(dot_count - 1) * kPageDotSpacing;
    const int first_center_x = kLauncherWidth / 2 - dot_span / 2;
    const int selected_center_x =
        first_center_x +
        static_cast<int>(selected_index % dot_count) * kPageDotSpacing;
    return abs_int(x - selected_center_x) <= kPageDotRadius;
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

constexpr uint16_t logical_color_at(const LauncherVisual *visual, int x, int y,
                                    uint32_t selected_index,
                                    uint32_t registry_count)
{
    uint16_t color = visual != nullptr ? visual->background_rgb565
                                       : kLauncherBackgroundRgb565;
    if (y >= kLauncherBandTop && y < kLauncherBandBottom) {
        color = visual != nullptr ? visual->band_rgb565 : kLauncherBandRgb565;
    }
    if (chevron_pixel(x, y, kChevronLeftApexX, 1) ||
        chevron_pixel(x, y, kChevronRightApexX, -1)) {
        color = visual != nullptr ? visual->affordance_rgb565
                                  : kLauncherAffordanceRgb565;
    }
    if (plus_pixel(x, y)) {
        color =
            visual != nullptr ? visual->accent_rgb565 : kLauncherAccentRgb565;
    }
    if (is_page_dot_pixel(x, y, registry_count)) {
        color = visual != nullptr ? visual->affordance_rgb565
                                  : kLauncherAffordanceRgb565;
        if (is_selected_page_dot_pixel(x, y, selected_index, registry_count)) {
            color = visual != nullptr ? visual->accent_rgb565
                                      : kLauncherAccentRgb565;
        }
    }
    if (visual != nullptr && visual->icon_rgb565 != nullptr) {
        if (x >= kLauncherGlyphLeft &&
            x < kLauncherGlyphLeft + kLauncherIconSize &&
            y >= kLauncherGlyphTop &&
            y < kLauncherGlyphTop + kLauncherIconSize) {
            const uint16_t pixel =
                visual
                    ->icon_rgb565[(y - kLauncherGlyphTop) * kLauncherIconSize +
                                  (x - kLauncherGlyphLeft)];
            if (pixel != kLauncherIconTransparent) {
                color = pixel;
            }
        }
    } else if (visual == nullptr && glyph_pixel(x, y)) {
        color = kLauncherGlyphRgb565;
    }
    return color;
}

static_assert(logical_color_at(nullptr, kLauncherBackgroundProbe.left,
                               kLauncherBackgroundProbe.top, 0,
                               1) == kLauncherBackgroundProbe.logical_rgb565);
static_assert(logical_color_at(nullptr, kLauncherBandProbe.left,
                               kLauncherBandProbe.top, 0,
                               1) == kLauncherBandProbe.logical_rgb565);
static_assert(logical_color_at(nullptr, kLauncherGlyphProbe.left,
                               kLauncherGlyphProbe.top, 0,
                               1) == kLauncherGlyphProbe.logical_rgb565);

bool fail(const char *operation, esp_err_t error)
{
    ESP_LOGE(kTag, "launcher %s failed: %s", operation, esp_err_to_name(error));
    return false;
}

bool valid_frame(const DisplayFrame &frame)
{
    return frame.width == kLauncherWidth && frame.height == kLauncherHeight &&
           frame.stripe_rows > 0 && frame.stripe_count > 0 &&
           frame.stripe_count ==
               (kLauncherHeight + frame.stripe_rows - 1) / frame.stripe_rows &&
           frame.stripe[0] != nullptr && frame.stripe[1] != nullptr &&
           frame.transport != nullptr && frame.ops.wait_previous != nullptr &&
           frame.ops.latch_capture != nullptr && frame.ops.submit != nullptr &&
           frame.ops.finish != nullptr;
}

}

bool render_launcher(DisplayFrame &frame, const LauncherVisual *visual,
                     uint32_t selected_index, uint32_t registry_count)
{
    if (!valid_frame(frame)) {
        return fail("configuration", ESP_ERR_INVALID_STATE);
    }

    esp_err_t error = frame.ops.wait_previous(frame.transport);
    if (error != ESP_OK) {
        return fail("wait", error);
    }

    frame.ops.latch_capture(frame.transport);

    for (int stripe_index = 0; stripe_index < frame.stripe_count;
         ++stripe_index) {
        const int stripe_y = stripe_index * frame.stripe_rows;
        const int remaining_rows = frame.height - stripe_y;
        const int stripe_rows = remaining_rows < frame.stripe_rows
                                    ? remaining_rows
                                    : frame.stripe_rows;
        uint16_t *stripe_pixels = frame.stripe[stripe_index & 1];

        for (int local_y = 0; local_y < stripe_rows; ++local_y) {
            const int screen_y = stripe_y + local_y;
            uint16_t *output_row = stripe_pixels + local_y * frame.width;
            for (int screen_x = 0; screen_x < frame.width; ++screen_x) {
                const uint16_t logical_color = logical_color_at(
                    visual, screen_x, screen_y, selected_index, registry_count);
                output_row[screen_x] = __builtin_bswap16(logical_color);
            }
        }

        error = frame.ops.submit(frame.transport, stripe_index, stripe_y,
                                 stripe_rows, stripe_pixels);
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

}
