#include "launcher.hpp"

#include "esp_err.h"
#include "esp_log.h"

namespace fluid_demo {
namespace {

constexpr char kTag[] = "launcher";

// Fixed swipe-chevron art and a page-dot row make horizontal swipeability
// visible (left swipe = next entry, right swipe = previous). The "<" sits
// just inside the band's left edge, the ">" sits between the glyph box and
// the PLUS, and the dots occupy one row below the band. Chevrons, dots and
// the right plus stay secondary to the centered selected-entry icon and use
// only fixed, allocation-free geometry.
constexpr int kChevronCenterY = 120;  // band vertical center
constexpr int kChevronArm = 12;       // chevron half-width
constexpr int kChevronStroke = 2;
constexpr int kChevronLeftApexX = 30;   // '<' apex, opens right
constexpr int kChevronRightApexX = 178; // '>' apex, opens left

constexpr int kPageDotRowY = 208;  // below the band, on the background
constexpr int kPageDotRadius = 2;
constexpr int kPageDotSpacing = 14;

constexpr int kPlusCenterX = 208;
constexpr int kPlusCenterY = 120;
constexpr int kPlusHalfLength = 8;
constexpr int kPlusHalfStroke = 2;

constexpr int abs_int(int value)
{
    return value < 0 ? -value : value;
}

constexpr bool glyph_cell_set(const uint8_t bitmap[8], int x, int y)
{
    if (x < kLauncherGlyphLeft || x >= kLauncherGlyphLeft + kLauncherGlyphExtent ||
        y < kLauncherGlyphTop || y >= kLauncherGlyphTop + kLauncherGlyphExtent) {
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

constexpr bool chevron_left_pixel(int x, int y)
{
    if (y < kChevronCenterY - kChevronArm - kChevronStroke ||
        y > kChevronCenterY + kChevronArm + kChevronStroke || x < kChevronLeftApexX) {
        return false;
    }
    const int dx = x - kChevronLeftApexX;
    if (dx > kChevronArm) {
        return false;
    }
    const bool top = abs_int(y - (kChevronCenterY - dx)) < kChevronStroke;
    const bool bottom = abs_int(y - (kChevronCenterY + dx)) < kChevronStroke;
    return top || bottom;
}

constexpr bool chevron_right_pixel(int x, int y)
{
    if (y < kChevronCenterY - kChevronArm - kChevronStroke ||
        y > kChevronCenterY + kChevronArm + kChevronStroke || x > kChevronRightApexX) {
        return false;
    }
    const int dx = kChevronRightApexX - x;
    if (dx > kChevronArm) {
        return false;
    }
    const bool top = abs_int(y - (kChevronCenterY - dx)) < kChevronStroke;
    const bool bottom = abs_int(y - (kChevronCenterY + dx)) < kChevronStroke;
    return top || bottom;
}

/// Effective page-dot count: never zero, so a modulo by zero is impossible.
constexpr uint32_t page_dot_count(uint32_t registry_count)
{
    return registry_count == 0 ? 1 : registry_count;
}

/// True when (x, y) is inside any page dot (drawn in the affordance color).
constexpr bool page_dot_any_pixel(int x, int y, uint32_t registry_count)
{
    if (y < kPageDotRowY - kPageDotRadius || y > kPageDotRowY + kPageDotRadius) {
        return false;
    }
    const uint32_t dots = page_dot_count(registry_count);
    const int span = static_cast<int>(dots - 1) * kPageDotSpacing;
    const int first_cx = kLauncherWidth / 2 - span / 2;
    for (uint32_t i = 0; i < dots; ++i) {
        if (abs_int(x - (first_cx + static_cast<int>(i) * kPageDotSpacing)) <=
            kPageDotRadius) {
            return true;
        }
    }
    return false;
}

/// True when (x, y) is inside the selected page dot (drawn in the accent
/// color).
constexpr bool page_dot_selected_pixel(int x, int y, uint32_t selected_index,
                                       uint32_t registry_count)
{
    if (y < kPageDotRowY - kPageDotRadius || y > kPageDotRowY + kPageDotRadius) {
        return false;
    }
    const uint32_t dots = page_dot_count(registry_count);
    const int span = static_cast<int>(dots - 1) * kPageDotSpacing;
    const int first_cx = kLauncherWidth / 2 - span / 2;
    const int cx =
        first_cx + static_cast<int>(selected_index % dots) * kPageDotSpacing;
    return abs_int(x - cx) <= kPageDotRadius;
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

/// One logical RGB565 color per pixel for the selected entry's launcher.
/// `selected_index` and `registry_count` drive the page-dot highlight.
/// A null `visual` keeps the exact built-in Fluid Box launcher colors,
/// glyph and capture probes byte-for-byte; a non-null descriptor supplies its
/// palette and 64x64 icon.
constexpr uint16_t logical_color_at(const LauncherVisual *visual, int x, int y,
                                    uint32_t selected_index,
                                    uint32_t registry_count)
{
    uint16_t color =
        visual != nullptr ? visual->background_rgb565 : kLauncherBackgroundRgb565;
    if (y >= kLauncherBandTop && y < kLauncherBandBottom) {
        color = visual != nullptr ? visual->band_rgb565 : kLauncherBandRgb565;
    }
    if (chevron_left_pixel(x, y) || chevron_right_pixel(x, y)) {
        color =
            visual != nullptr ? visual->affordance_rgb565 : kLauncherAffordanceRgb565;
    }
    if (plus_pixel(x, y)) {
        color = visual != nullptr ? visual->accent_rgb565 : kLauncherAccentRgb565;
    }
    if (page_dot_any_pixel(x, y, registry_count)) {
        color =
            visual != nullptr ? visual->affordance_rgb565 : kLauncherAffordanceRgb565;
        if (page_dot_selected_pixel(x, y, selected_index, registry_count)) {
            color = visual != nullptr ? visual->accent_rgb565 : kLauncherAccentRgb565;
        }
    }
    if (visual != nullptr && visual->icon_rgb565 != nullptr) {
        if (x >= kLauncherGlyphLeft && x < kLauncherGlyphLeft + kLauncherIconSize &&
            y >= kLauncherGlyphTop && y < kLauncherGlyphTop + kLauncherIconSize) {
            const uint16_t pixel =
                visual->icon_rgb565[(y - kLauncherGlyphTop) * kLauncherIconSize +
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

// The null descriptor keeps every capture-probe pixel byte-identical to the
// pre-split Fluid launcher: the chevrons, dots and PLUS occupy geometry the
// probes deliberately avoid.
static_assert(logical_color_at(nullptr, kLauncherBackgroundProbe.left,
                               kLauncherBackgroundProbe.top, 0, 1) ==
              kLauncherBackgroundProbe.logical_rgb565);
static_assert(logical_color_at(nullptr, kLauncherBandProbe.left,
                               kLauncherBandProbe.top, 0, 1) ==
              kLauncherBandProbe.logical_rgb565);
static_assert(logical_color_at(nullptr, kLauncherGlyphProbe.left,
                               kLauncherGlyphProbe.top, 0, 1) ==
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
                // This is the sole logical-RGB565 to wire-order conversion
                // (identical for every launcher visual).
                row[x] = __builtin_bswap16(
                    logical_color_at(visual, x, y, selected_index, registry_count));
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
