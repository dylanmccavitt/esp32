#include "attitude_app.hpp"

#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "draw.hpp"

namespace fluid_demo {

namespace {

constexpr const char *kTag = "attitude";
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 240;
constexpr int kCx = 120;
constexpr int kCy = 120;
constexpr int kRadius = 118;
constexpr int kRadiusSq = kRadius * kRadius;

constexpr uint16_t kBezel = 0x10A2;
constexpr uint16_t kBand = 0x1C4A;
constexpr uint16_t kMuted = 0x7C4F;
constexpr uint16_t kAccent = 0xFE60;
constexpr uint16_t kFluid = 0x1B8A;
constexpr uint16_t kFluidDeep = 0x1167;
constexpr uint16_t kRing = 0x3C70;
constexpr uint16_t kCross = 0xDEFB;
constexpr uint16_t kBubble = 0xEF7D;
constexpr uint16_t kBubbleHi = 0xFFFF;
constexpr uint16_t kBubbleEdge = 0x9CD3;
constexpr uint16_t kSnapBubble = 0xFE60;
constexpr uint16_t kSnapRing = 0xFDE0;
constexpr uint16_t kDigit = 0xFFFF;
constexpr uint16_t kDigitSnap = 0xFE60;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg = kPi / 180.0f;
constexpr float kSnapRad = 2.0f * kDeg;
constexpr float kBubbleGain = 70.0f / (15.0f * kDeg);
constexpr int kBubbleRadius = 22;
constexpr int kBubbleTravel = 78;

constexpr uint8_t kLauncherVialBitmap[8] = {
    0b00111100,
    0b01111110,
    0b11111111,
    0b11111111,
    0b11111111,
    0b11111111,
    0b01111110,
    0b00111100,
};
constexpr uint8_t kLauncherBubbleBitmap[8] = {
    0b00000000,
    0b00000000,
    0b00011000,
    0b00111100,
    0b00111100,
    0b00011000,
    0b00000000,
    0b00000000,
};
constexpr LauncherVisual kLauncherVisual{
    kBezel,
    kBand,
    kMuted,
    kAccent,
    kFluid,
    kSnapBubble,
    kLauncherVialBitmap,
    kLauncherBubbleBitmap,
};

inline int min_int(int a, int b) { return a < b ? a : b; }
inline int max_int(int a, int b) { return a > b ? a : b; }
inline bool finite_vec(const Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

void draw_ring(uint16_t *pixels, int width, int y0, int rows, int cx, int cy,
               int inner, int outer, uint16_t color)
{
    if (pixels == nullptr || width <= 0 || rows <= 0 || outer < inner || inner < 0) {
        return;
    }
    const int top = max_int(y0, cy - outer);
    const int bottom = min_int(y0 + rows - 1, cy + outer);
    const int left = max_int(0, cx - outer);
    const int right = min_int(width - 1, cx + outer);
    const int outer_sq = outer * outer;
    const int inner_sq = inner * inner;
    for (int y = top; y <= bottom; ++y) {
        uint16_t *row = pixels + (y - y0) * width;
        const int dy = y - cy;
        for (int x = left; x <= right; ++x) {
            const int dx = x - cx;
            const int r2 = dx * dx + dy * dy;
            if (r2 <= outer_sq && r2 >= inner_sq) {
                row[x] = color;
            }
        }
    }
}

void draw_segment_h(uint16_t *pixels, int width, int y0, int rows, int x0, int x1,
                    int y, int thickness, uint16_t color)
{
    fill_segment(pixels, width, y0, rows, x0, y, x1, y, thickness, color);
}

void draw_segment_v(uint16_t *pixels, int width, int y0, int rows, int x, int y0s,
                    int y1, int thickness, uint16_t color)
{
    fill_segment(pixels, width, y0, rows, x, y0s, x, y1, thickness, color);
}

void draw_digit(uint16_t *pixels, int width, int y0, int rows, int x, int y, int h,
                int digit, uint16_t color)
{
    if (digit < 0 || digit > 9) {
        return;
    }
    constexpr uint8_t kSeg[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
                                  0x6D, 0x7D, 0x07, 0x7F, 0x6F};
    const uint8_t mask = kSeg[digit];
    const int w = (h * 3) / 5;
    const int mid = y + h / 2;
    const int t = max_int(1, h / 10);
    if ((mask & 0x01) != 0) {
        draw_segment_h(pixels, width, y0, rows, x + t, x + w - t, y, t, color);
    }
    if ((mask & 0x02) != 0) {
        draw_segment_v(pixels, width, y0, rows, x + w, y + t, mid - t, t, color);
    }
    if ((mask & 0x04) != 0) {
        draw_segment_v(pixels, width, y0, rows, x + w, mid + t, y + h - t, t, color);
    }
    if ((mask & 0x08) != 0) {
        draw_segment_h(pixels, width, y0, rows, x + t, x + w - t, y + h, t, color);
    }
    if ((mask & 0x10) != 0) {
        draw_segment_v(pixels, width, y0, rows, x, mid + t, y + h - t, t, color);
    }
    if ((mask & 0x20) != 0) {
        draw_segment_v(pixels, width, y0, rows, x, y + t, mid - t, t, color);
    }
    if ((mask & 0x40) != 0) {
        draw_segment_h(pixels, width, y0, rows, x + t, x + w - t, mid, t, color);
    }
}

void draw_signed_degrees(uint16_t *pixels, int width, int y0, int rows, int cx,
                         int y, int h, int value, uint16_t color)
{
    if (value > 99) {
        value = 99;
    }
    if (value < -99) {
        value = -99;
    }
    const int w = (h * 3) / 5 + 4;
    int digits[2] = {0, 0};
    int count = 0;
    int abs_v = value < 0 ? -value : value;
    if (abs_v >= 10) {
        digits[count++] = abs_v / 10;
    }
    digits[count++] = abs_v % 10;
    const int minus = value < 0 ? 1 : 0;
    const int total = minus + count;
    int x = cx - (total * w) / 2;
    if (minus != 0) {
        const int mid = y + h / 2;
        draw_segment_h(pixels, width, y0, rows, x + 1, x + w - 6, mid, max_int(1, h / 10),
                       color);
        x += w;
    }
    for (int i = 0; i < count; ++i) {
        draw_digit(pixels, width, y0, rows, x, y, h, digits[i], color);
        x += w;
    }
    fill_disc(pixels, width, y0, rows, x + 3, y + 4, 3, color);
    fill_disc(pixels, width, y0, rows, x + 3, y + 4, 1, kFluidDeep);
}

}  // namespace

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
    filter_.reset();
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
    filter_.reset();
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    motion_.up_x = 0.0f;
    motion_.up_y = 1.0f;
    motion_.up_z = 0.0f;
    motion_.roll = 0.0f;
    motion_.pitch = 0.0f;
    motion_.gyro_abs = 0.0f;
    portEXIT_CRITICAL(&motion_mux_);
    return ESP_OK;
}

void AttitudeApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
}

ShellAction AttitudeApp::handle_event(AppEvent event)
{
    if (event == AppEvent::PlusPress) {
        reset_requested_.store(true, std::memory_order_release);
        filter_.request_align();
    }
    return ShellAction::None;
}

bool AttitudeApp::on_motion(const MotionTick &tick)
{
    const bool physical_valid =
        tick.fresh && std::isfinite(tick.dt) && tick.dt > 0.0f &&
        finite_vec(tick.accel_mps2) && finite_vec(tick.gyro_rads);
    bool physical_accepted = false;
    if (physical_valid) {
        if (tick.override_active) {
            physical_accepted = true;
        } else {
            physical_accepted = filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
        }
    }

    const bool override_valid = tick.override_active && finite_vec(tick.apparent_accel);
    if (override_valid) {
        static_cast<void>(filter_.apply_override(tick.apparent_accel));
    }

    if (!physical_accepted && !override_valid) {
        portENTER_CRITICAL(&motion_mux_);
        motion_.valid = false;
        portEXIT_CRITICAL(&motion_mux_);
        return false;
    }

    const Vec3 up = filter_.up();
    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent = override_valid ? tick.apparent_accel : filter_.mapped_accel();
    motion_.valid = true;
    motion_.up_x = up.x;
    motion_.up_y = up.y;
    motion_.up_z = up.z;
    motion_.roll = filter_.roll();
    motion_.pitch = filter_.pitch();
    motion_.gyro_abs = filter_.gyro_abs();
    if (physical_valid) {
        motion_.raw = tick.accel_mps2;
    }
    portEXIT_CRITICAL(&motion_mux_);
    nonfinite_resets_.store(filter_.nonfinite_resets(), std::memory_order_relaxed);
    return physical_accepted;
}

void AttitudeApp::fill_snapshot(HorizonFrame &snapshot)
{
    ++sequence_;
    if (sequence_ == 0u) {
        sequence_ = 1u;
    }
    snapshot.sequence = sequence_;
    snapshot.epoch = epoch_;
    portENTER_CRITICAL(&motion_mux_);
    snapshot.up_x = motion_.up_x;
    snapshot.up_y = motion_.up_y;
    snapshot.up_z = motion_.up_z;
    snapshot.roll = motion_.roll;
    snapshot.pitch = motion_.pitch;
    snapshot.ladder_ok = true;
    portEXIT_CRITICAL(&motion_mux_);
}

esp_err_t AttitudeApp::update(float dt)
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t update_start = esp_timer_get_time();
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
        ++epoch_;
        if (epoch_ == 0u) {
            epoch_ = 1u;
        }
        ESP_LOGI(kTag, "level align epoch=%" PRIu32, epoch_);
    }

    HorizonFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        fill_snapshot(*snapshot);
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - update_start),
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
    result.raw[0] = motion_.raw.x;
    result.raw[1] = motion_.raw.y;
    result.raw[2] = motion_.raw.z;
    result.apparent[0] = motion_.apparent.x;
    result.apparent[1] = motion_.apparent.y;
    result.apparent[2] = motion_.apparent.z;
    portEXIT_CRITICAL(&motion_mux_);
    result.raster_us = raster_us_;
    result.frame_us = frame_us_;
    return result;
}

void AttitudeApp::raster_stripe(const HorizonFrame &horizon, uint16_t *pixels,
                                int width, int y0, int rows)
{
    const float pitch = horizon.pitch;
    const float roll = horizon.roll;
    const float tilt = std::sqrt(pitch * pitch + roll * roll);
    const bool snapped = tilt <= kSnapRad;
    float bx = static_cast<float>(kCx) + roll * kBubbleGain;
    float by = static_cast<float>(kCy) + pitch * kBubbleGain;
    const float dxb = bx - static_cast<float>(kCx);
    const float dyb = by - static_cast<float>(kCy);
    const float travel = std::sqrt(dxb * dxb + dyb * dyb);
    if (travel > static_cast<float>(kBubbleTravel) && travel > 1e-3f) {
        const float scale = static_cast<float>(kBubbleTravel) / travel;
        bx = static_cast<float>(kCx) + dxb * scale;
        by = static_cast<float>(kCy) + dyb * scale;
    }
    const int bubble_x = static_cast<int>(bx + 0.5f);
    const int bubble_y = static_cast<int>(by + 0.5f);
    const uint16_t fluid = snapped ? kFluid : kFluidDeep;
    const uint16_t bubble = snapped ? kSnapBubble : kBubble;
    const uint16_t digit = snapped ? kDigitSnap : kDigit;

    for (int local_y = 0; local_y < rows; ++local_y) {
        const int y = y0 + local_y;
        const int dy = y - kCy;
        uint16_t *row = pixels + local_y * width;
        for (int x = 0; x < width; ++x) {
            const int dx = x - kCx;
            row[x] = (dx * dx + dy * dy <= kRadiusSq) ? fluid : kBezel;
        }
    }

    draw_ring(pixels, width, y0, rows, kCx, kCy, 52, 54, kRing);
    draw_ring(pixels, width, y0, rows, kCx, kCy, 78, 80, kRing);
    draw_ring(pixels, width, y0, rows, kCx, kCy, 102, 104, kRing);
    if (snapped) {
        draw_ring(pixels, width, y0, rows, kCx, kCy, 24, 28, kSnapRing);
    } else {
        draw_ring(pixels, width, y0, rows, kCx, kCy, 26, 27, kRing);
    }

    fill_segment(pixels, width, y0, rows, kCx - 48, kCy, kCx + 48, kCy, 1, kCross);
    fill_segment(pixels, width, y0, rows, kCx, kCy - 48, kCx, kCy + 48, 1, kCross);
    fill_disc(pixels, width, y0, rows, kCx, kCy, 3, kCross);

    fill_disc(pixels, width, y0, rows, bubble_x, bubble_y, kBubbleRadius + 1,
              kBubbleEdge);
    fill_disc(pixels, width, y0, rows, bubble_x, bubble_y, kBubbleRadius, bubble);
    fill_disc(pixels, width, y0, rows, bubble_x - 6, bubble_y - 7, 6, kBubbleHi);
    fill_disc(pixels, width, y0, rows, bubble_x - 7, bubble_y - 8, 2, kCross);

    const int pitch_deg = static_cast<int>(pitch / kDeg + (pitch >= 0.0f ? 0.5f : -0.5f));
    const int roll_deg = static_cast<int>(roll / kDeg + (roll >= 0.0f ? 0.5f : -0.5f));
    draw_signed_degrees(pixels, width, y0, rows, 70, 188, 22, pitch_deg, digit);
    draw_signed_degrees(pixels, width, y0, rows, 170, 188, 22, roll_deg, digit);

    for (int local_y = 0; local_y < rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        const int y = y0 + local_y;
        const int dy = y - kCy;
        for (int x = 0; x < width; ++x) {
            const int dx = x - kCx;
            if (dx * dx + dy * dy > kRadiusSq) {
                row[x] = kBezel;
            }
        }
    }
}

bool AttitudeApp::render(DisplayFrame &frame)
{
    if (!setup_done_ || frame.transport == nullptr || frame.width != kPanelWidth ||
        frame.height != kPanelHeight || frame.stripe_rows <= 0 ||
        frame.stripe_count <= 0 || frame.stripe[0] == nullptr ||
        frame.stripe[1] == nullptr || frame.ops.wait_previous == nullptr ||
        frame.ops.latch_capture == nullptr || frame.ops.submit == nullptr ||
        frame.ops.finish == nullptr || frame.ops.capture_copy_us == nullptr) {
        ESP_LOGW(kTag, "render rejected invalid display frame");
        return false;
    }

    const HorizonFrame *horizon = frames_.acquire_latest();
    if (horizon == nullptr) {
        return false;
    }

    const int64_t frame_start = esp_timer_get_time();
    uint32_t raster_total = 0;
    esp_err_t result = frame.ops.wait_previous(frame.transport);
    if (result == ESP_OK) {
        static_cast<void>(frame.ops.latch_capture(frame.transport));
        for (int stripe = 0; stripe < frame.stripe_count; ++stripe) {
            const int stripe_y = stripe * frame.stripe_rows;
            const int stripe_rows = min_int(frame.stripe_rows, frame.height - stripe_y);
            if (stripe_rows <= 0) {
                break;
            }
            uint16_t *pixels = frame.stripe[stripe & 1];
            const int64_t raster_start = esp_timer_get_time();
            raster_stripe(*horizon, pixels, frame.width, stripe_y, stripe_rows);
            const int pixel_count = frame.width * stripe_rows;
            for (int pixel = 0; pixel < pixel_count; ++pixel) {
                pixels[pixel] = __builtin_bswap16(pixels[pixel]);
            }
            raster_total +=
                static_cast<uint32_t>(esp_timer_get_time() - raster_start);
            result = frame.ops.submit(frame.transport, stripe, stripe_y, stripe_rows,
                                      pixels);
            if (result != ESP_OK) {
                break;
            }
        }
        if (result == ESP_OK) {
            result = frame.ops.finish(frame.transport);
        }
    }

    frames_.release(horizon);
    if (result != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s", esp_err_to_name(result));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start);
    raster_us_ = raster_total + frame.ops.capture_copy_us(frame.transport);
    return true;
}

}  // namespace fluid_demo
