#include "attitude_app.hpp"

#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "common_math.hpp"
#include "draw.hpp"
#include "launcher_icons.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "attitude";
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 240;
constexpr int kCenterX = 120;
constexpr int kCenterY = 120;
constexpr int kFaceRadius = 118;
constexpr int kFaceRadiusSquared = kFaceRadius * kFaceRadius;

constexpr uint16_t kBezel = 0x10A2;
constexpr uint16_t kBand = 0x1C4A;
constexpr uint16_t kMuted = 0x7C4F;
constexpr uint16_t kAccent = 0xFE60;
constexpr uint16_t kFluid = 0x1B8A;
constexpr uint16_t kFluidDeep = 0x1167;
constexpr uint16_t kRing = 0x3C70;
constexpr uint16_t kCross = 0xDEFB;
constexpr uint16_t kBubble = 0xEF7D;
constexpr uint16_t kBubbleHighlight = 0xFFFF;
constexpr uint16_t kBubbleEdge = 0x9CD3;
constexpr uint16_t kSnapBubble = 0xFE60;
constexpr uint16_t kSnapRing = 0xFDE0;
constexpr uint16_t kDigit = 0xFFFF;
constexpr uint16_t kDigitSnap = 0xFE60;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadiansPerDegree = kPi / 180.0f;
constexpr float kSnapThresholdRadians = 2.0f * kRadiansPerDegree;
constexpr float kBubblePixelsPerRadian = 70.0f / (15.0f * kRadiansPerDegree);
constexpr int kBubbleRadius = 22;
constexpr int kMaximumBubbleTravel = 78;

constexpr LauncherVisual kLauncherVisual{
    kBezel, kBand, kMuted, kAccent, kIconAttitude,
};

void draw_ring(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
               int center_x, int center_y, int inner_radius, int outer_radius,
               uint16_t color)
{
    if (pixels == nullptr || width <= 0 || stripe_rows <= 0 ||
        inner_radius < 0 || outer_radius < inner_radius) {
        return;
    }

    const int top = max_int(stripe_y, center_y - outer_radius);
    const int bottom =
        min_int(stripe_y + stripe_rows - 1, center_y + outer_radius);
    const int left = max_int(0, center_x - outer_radius);
    const int right = min_int(width - 1, center_x + outer_radius);
    const int inner_radius_squared = inner_radius * inner_radius;
    const int outer_radius_squared = outer_radius * outer_radius;

    for (int screen_y = top; screen_y <= bottom; ++screen_y) {
        uint16_t *row = pixels + (screen_y - stripe_y) * width;
        const int offset_y = screen_y - center_y;
        for (int screen_x = left; screen_x <= right; ++screen_x) {
            const int offset_x = screen_x - center_x;
            const int radius_squared =
                offset_x * offset_x + offset_y * offset_y;
            if (radius_squared >= inner_radius_squared &&
                radius_squared <= outer_radius_squared) {
                row[screen_x] = color;
            }
        }
    }
}

void draw_horizontal_segment(uint16_t *pixels, int width, int stripe_y,
                             int stripe_rows, int start_x, int end_x,
                             int screen_y, int thickness, uint16_t color)
{
    fill_segment(pixels, width, stripe_y, stripe_rows, start_x, screen_y, end_x,
                 screen_y, thickness, color);
}

void draw_vertical_segment(uint16_t *pixels, int width, int stripe_y,
                           int stripe_rows, int screen_x, int start_y,
                           int end_y, int thickness, uint16_t color)
{
    fill_segment(pixels, width, stripe_y, stripe_rows, screen_x, start_y,
                 screen_x, end_y, thickness, color);
}

void draw_digit(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                int origin_x, int origin_y, int digit_height, int digit_value,
                uint16_t color)
{
    if (digit_value < 0 || digit_value > 9) {
        return;
    }

    constexpr uint8_t kDigitSegmentMasks[10] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
    };
    const uint8_t segment_mask = kDigitSegmentMasks[digit_value];
    const int digit_width = (digit_height * 3) / 5;
    const int midpoint_y = origin_y + digit_height / 2;
    const int thickness = max_int(1, digit_height / 10);

    if ((segment_mask & 0x01) != 0) {
        draw_horizontal_segment(
            pixels, width, stripe_y, stripe_rows, origin_x + thickness,
            origin_x + digit_width - thickness, origin_y, thickness, color);
    }
    if ((segment_mask & 0x02) != 0) {
        draw_vertical_segment(pixels, width, stripe_y, stripe_rows,
                              origin_x + digit_width, origin_y + thickness,
                              midpoint_y - thickness, thickness, color);
    }
    if ((segment_mask & 0x04) != 0) {
        draw_vertical_segment(pixels, width, stripe_y, stripe_rows,
                              origin_x + digit_width, midpoint_y + thickness,
                              origin_y + digit_height - thickness, thickness,
                              color);
    }
    if ((segment_mask & 0x08) != 0) {
        draw_horizontal_segment(pixels, width, stripe_y, stripe_rows,
                                origin_x + thickness,
                                origin_x + digit_width - thickness,
                                origin_y + digit_height, thickness, color);
    }
    if ((segment_mask & 0x10) != 0) {
        draw_vertical_segment(pixels, width, stripe_y, stripe_rows, origin_x,
                              midpoint_y + thickness,
                              origin_y + digit_height - thickness, thickness,
                              color);
    }
    if ((segment_mask & 0x20) != 0) {
        draw_vertical_segment(pixels, width, stripe_y, stripe_rows, origin_x,
                              origin_y + thickness, midpoint_y - thickness,
                              thickness, color);
    }
    if ((segment_mask & 0x40) != 0) {
        draw_horizontal_segment(
            pixels, width, stripe_y, stripe_rows, origin_x + thickness,
            origin_x + digit_width - thickness, midpoint_y, thickness, color);
    }
}

void draw_signed_degrees(uint16_t *pixels, int width, int stripe_y,
                         int stripe_rows, int center_x, int top,
                         int digit_height, int value, uint16_t color)
{
    value = max_int(-99, min_int(99, value));

    const int digit_advance = (digit_height * 3) / 5 + 4;
    int digits[2] = {};
    int digit_count = 0;
    const int absolute_value = abs_int(value);
    if (absolute_value >= 10) {
        digits[digit_count++] = absolute_value / 10;
    }
    digits[digit_count++] = absolute_value % 10;

    const bool negative = value < 0;
    const int glyph_count = digit_count + (negative ? 1 : 0);
    int cursor_x = center_x - (glyph_count * digit_advance) / 2;
    if (negative) {
        const int midpoint_y = top + digit_height / 2;
        draw_horizontal_segment(pixels, width, stripe_y, stripe_rows,
                                cursor_x + 1, cursor_x + digit_advance - 6,
                                midpoint_y, max_int(1, digit_height / 10),
                                color);
        cursor_x += digit_advance;
    }

    for (int digit_index = 0; digit_index < digit_count; ++digit_index) {
        draw_digit(pixels, width, stripe_y, stripe_rows, cursor_x, top,
                   digit_height, digits[digit_index], color);
        cursor_x += digit_advance;
    }
    fill_disc(pixels, width, stripe_y, stripe_rows, cursor_x + 3, top + 4, 3,
              color);
    fill_disc(pixels, width, stripe_y, stripe_rows, cursor_x + 3, top + 4, 1,
              kFluidDeep);
}

}

AttitudeApp s_attitude_app;

const LauncherVisual *AttitudeApp::launcher_visual() const
{
    return &kLauncherVisual;
}

esp_err_t AttitudeApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;
    }
    attitude_filter_.reset();
    epoch_ = 1;
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    setup_done_ = true;
    return ESP_OK;
}

esp_err_t AttitudeApp::enter()
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    frames_.drain();
    attitude_filter_.reset();
    portENTER_CRITICAL(&motion_mux_);
    motion_.roll = 0.0f;
    motion_.pitch = 0.0f;
    motion_.yaw = 0.0f;
    portEXIT_CRITICAL(&motion_mux_);
    return ESP_OK;
}

void AttitudeApp::leave() {}

void AttitudeApp::on_plus_press()
{
    reset_requested_.store(true, std::memory_order_release);
    attitude_filter_.request_align();
}

bool AttitudeApp::on_motion(const MotionTick &tick)
{
    const bool physical_sample_valid =
        tick.fresh && std::isfinite(tick.dt) && tick.dt > 0.0f &&
        finite_vec(tick.accel_mps2) && finite_vec(tick.gyro_rads);
    const bool had_reference = attitude_filter_.aligned();
    bool override_accepted =
        tick.override_active && had_reference &&
        attitude_filter_.apply_override(tick.apparent_accel);

    bool physical_sample_accepted = false;
    if (physical_sample_valid) {
        physical_sample_accepted =
            override_accepted ||
            attitude_filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
    }
    if (tick.override_active && !override_accepted &&
        physical_sample_accepted && attitude_filter_.aligned()) {
        override_accepted =
            attitude_filter_.apply_override(tick.apparent_accel);
    }

    if (!physical_sample_accepted && !override_accepted) {
        return false;
    }

    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent_acceleration =
        override_accepted ? tick.apparent_accel
                          : attitude_filter_.mapped_acceleration();
    motion_.roll = attitude_filter_.roll();
    motion_.pitch = attitude_filter_.pitch();
    motion_.yaw = attitude_filter_.yaw();
    if (physical_sample_valid) {
        motion_.raw_acceleration = tick.accel_mps2;
    }
    portEXIT_CRITICAL(&motion_mux_);
    nonfinite_resets_.store(attitude_filter_.nonfinite_resets(),
                            std::memory_order_relaxed);
    return physical_sample_accepted;
}

esp_err_t AttitudeApp::update(float dt)
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t update_start_us = esp_timer_get_time();
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
        ++epoch_;
        if (epoch_ == 0u) {
            epoch_ = 1u;
        }
        ESP_LOGI(kTag, "level align epoch=%" PRIu32, epoch_);
    }

    LevelFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        portENTER_CRITICAL(&motion_mux_);
        snapshot->roll = motion_.roll;
        snapshot->pitch = motion_.pitch;
        portEXIT_CRITICAL(&motion_mux_);
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(
        static_cast<uint32_t>(esp_timer_get_time() - update_start_us),
        std::memory_order_relaxed);
    return ESP_OK;
}

AppStats AttitudeApp::stats()
{
    AppStats result{};
    result.count = 1;
    result.epoch = published_epoch_.load(std::memory_order_relaxed);
    result.nonfinite_resets = nonfinite_resets_.load(std::memory_order_relaxed);
    result.physics_us = physics_us_.load(std::memory_order_relaxed);
    portENTER_CRITICAL(&motion_mux_);
    result.raw[0] = motion_.raw_acceleration.x;
    result.raw[1] = motion_.raw_acceleration.y;
    result.raw[2] = motion_.raw_acceleration.z;
    result.apparent[0] = motion_.apparent_acceleration.x;
    result.apparent[1] = motion_.apparent_acceleration.y;
    result.apparent[2] = motion_.apparent_acceleration.z;
    result.pitch = motion_.pitch;
    result.roll = motion_.roll;
    result.yaw = motion_.yaw;
    portEXIT_CRITICAL(&motion_mux_);
    result.raster_us = raster_us_;
    result.frame_us = frame_us_;
    return result;
}

void AttitudeApp::raster_stripe(const LevelFrame &level, uint16_t *pixels,
                                int width, int stripe_y, int stripe_rows)
{
    const float pitch = level.pitch;
    const float roll = level.roll;
    const float tilt_magnitude = std::sqrt(pitch * pitch + roll * roll);
    const bool is_level = tilt_magnitude <= kSnapThresholdRadians;

    float bubble_center_x =
        static_cast<float>(kCenterX) + roll * kBubblePixelsPerRadian;
    float bubble_center_y =
        static_cast<float>(kCenterY) + pitch * kBubblePixelsPerRadian;
    const float bubble_offset_x =
        bubble_center_x - static_cast<float>(kCenterX);
    const float bubble_offset_y =
        bubble_center_y - static_cast<float>(kCenterY);
    const float bubble_distance = std::sqrt(bubble_offset_x * bubble_offset_x +
                                            bubble_offset_y * bubble_offset_y);
    if (bubble_distance > static_cast<float>(kMaximumBubbleTravel)) {
        const float travel_scale =
            static_cast<float>(kMaximumBubbleTravel) / bubble_distance;
        bubble_center_x =
            static_cast<float>(kCenterX) + bubble_offset_x * travel_scale;
        bubble_center_y =
            static_cast<float>(kCenterY) + bubble_offset_y * travel_scale;
    }

    const int bubble_x = static_cast<int>(bubble_center_x + 0.5f);
    const int bubble_y = static_cast<int>(bubble_center_y + 0.5f);
    const uint16_t fluid_color = is_level ? kFluid : kFluidDeep;
    const uint16_t bubble_color = is_level ? kSnapBubble : kBubble;
    const uint16_t digit_color = is_level ? kDigitSnap : kDigit;

    for (int local_y = 0; local_y < stripe_rows; ++local_y) {
        const int screen_y = stripe_y + local_y;
        const int offset_y = screen_y - kCenterY;
        uint16_t *row = pixels + local_y * width;
        for (int screen_x = 0; screen_x < width; ++screen_x) {
            const int offset_x = screen_x - kCenterX;
            row[screen_x] =
                offset_x * offset_x + offset_y * offset_y <= kFaceRadiusSquared
                    ? fluid_color
                    : kBezel;
        }
    }

    draw_ring(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY, 52, 54,
              kRing);
    draw_ring(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY, 78, 80,
              kRing);
    draw_ring(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY, 102,
              104, kRing);
    if (is_level) {
        draw_ring(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY, 24,
                  28, kSnapRing);
    } else {
        draw_ring(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY, 26,
                  27, kRing);
    }

    fill_segment(pixels, width, stripe_y, stripe_rows, kCenterX - 48, kCenterY,
                 kCenterX + 48, kCenterY, 1, kCross);
    fill_segment(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY - 48,
                 kCenterX, kCenterY + 48, 1, kCross);
    fill_disc(pixels, width, stripe_y, stripe_rows, kCenterX, kCenterY, 3,
              kCross);

    fill_disc(pixels, width, stripe_y, stripe_rows, bubble_x, bubble_y,
              kBubbleRadius + 1, kBubbleEdge);
    fill_disc(pixels, width, stripe_y, stripe_rows, bubble_x, bubble_y,
              kBubbleRadius, bubble_color);
    fill_disc(pixels, width, stripe_y, stripe_rows, bubble_x - 6, bubble_y - 7,
              6, kBubbleHighlight);
    fill_disc(pixels, width, stripe_y, stripe_rows, bubble_x - 7, bubble_y - 8,
              2, kCross);

    const int pitch_degrees = static_cast<int>(pitch / kRadiansPerDegree +
                                               (pitch >= 0.0f ? 0.5f : -0.5f));
    const int roll_degrees = static_cast<int>(roll / kRadiansPerDegree +
                                              (roll >= 0.0f ? 0.5f : -0.5f));
    draw_signed_degrees(pixels, width, stripe_y, stripe_rows, 70, 188, 22,
                        pitch_degrees, digit_color);
    draw_signed_degrees(pixels, width, stripe_y, stripe_rows, 170, 188, 22,
                        roll_degrees, digit_color);

    for (int local_y = 0; local_y < stripe_rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        const int screen_y = stripe_y + local_y;
        const int offset_y = screen_y - kCenterY;
        for (int screen_x = 0; screen_x < width; ++screen_x) {
            const int offset_x = screen_x - kCenterX;
            if (offset_x * offset_x + offset_y * offset_y >
                kFaceRadiusSquared) {
                row[screen_x] = kBezel;
            }
        }
    }
}

bool AttitudeApp::render(DisplayFrame &frame)
{
    if (!setup_done_ || frame.transport == nullptr ||
        frame.width != kPanelWidth || frame.height != kPanelHeight ||
        frame.stripe_rows <= 0 || frame.stripe_count <= 0 ||
        frame.stripe[0] == nullptr || frame.stripe[1] == nullptr ||
        frame.ops.wait_previous == nullptr ||
        frame.ops.latch_capture == nullptr || frame.ops.submit == nullptr ||
        frame.ops.finish == nullptr || frame.ops.capture_copy_us == nullptr) {
        ESP_LOGW(kTag, "render rejected invalid display frame");
        return false;
    }

    const LevelFrame *snapshot = frames_.acquire_latest();
    if (snapshot == nullptr) {
        return false;
    }

    const int64_t frame_start_us = esp_timer_get_time();
    uint32_t total_raster_us = 0;
    esp_err_t transport_result = frame.ops.wait_previous(frame.transport);
    if (transport_result == ESP_OK) {
        frame.ops.latch_capture(frame.transport);
        for (int stripe_index = 0; stripe_index < frame.stripe_count;
             ++stripe_index) {
            const int stripe_y = stripe_index * frame.stripe_rows;
            const int stripe_rows =
                min_int(frame.stripe_rows, frame.height - stripe_y);
            if (stripe_rows <= 0) {
                break;
            }

            uint16_t *stripe_pixels = frame.stripe[stripe_index & 1];
            const int64_t raster_start_us = esp_timer_get_time();
            raster_stripe(*snapshot, stripe_pixels, frame.width, stripe_y,
                          stripe_rows);
            const int pixel_count = frame.width * stripe_rows;
            for (int pixel_index = 0; pixel_index < pixel_count;
                 ++pixel_index) {
                stripe_pixels[pixel_index] =
                    __builtin_bswap16(stripe_pixels[pixel_index]);
            }
            total_raster_us +=
                static_cast<uint32_t>(esp_timer_get_time() - raster_start_us);
            transport_result =
                frame.ops.submit(frame.transport, stripe_index, stripe_y,
                                 stripe_rows, stripe_pixels);
            if (transport_result != ESP_OK) {
                break;
            }
        }
        if (transport_result == ESP_OK) {
            transport_result = frame.ops.finish(frame.transport);
        }
    }

    frames_.release(snapshot);
    if (transport_result != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s",
                 esp_err_to_name(transport_result));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start_us);
    raster_us_ = total_raster_us + frame.ops.capture_copy_us(frame.transport);
    return true;
}

}
