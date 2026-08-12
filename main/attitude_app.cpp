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
constexpr uint16_t kSky = 0x3C9C;
constexpr uint16_t kGround = 0xC2A6;
constexpr uint16_t kHorizon = 0xFFDF;
constexpr uint16_t kChevron = 0xFFFF;
constexpr uint16_t kTick = 0xDEFB;
constexpr uint16_t kIndex = 0xFE60;
constexpr uint16_t kCenterDot = 0xF800;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg = kPi / 180.0f;
constexpr float kPxPerRad = static_cast<float>(kRadius) / (0.5f * kPi);
constexpr float kLadderGyroMax = 0.45f;

constexpr uint8_t kLauncherSkyBitmap[8] = {
    0b00111100,
    0b01111110,
    0b11111111,
    0b11111111,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
};
constexpr uint8_t kLauncherGroundBitmap[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b11111111,
    0b11111111,
    0b01111110,
    0b00111100,
};
constexpr LauncherVisual kLauncherVisual{
    kBezel,
    kBand,
    kMuted,
    kAccent,
    kSky,
    kGround,
    kLauncherSkyBitmap,
    kLauncherGroundBitmap,
};

inline int min_int(int a, int b) { return a < b ? a : b; }
inline bool finite_vec(const Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

void draw_bezel_tick(uint16_t *pixels, int width, int y0, int rows, float angle_rad,
                     int inner, int outer, uint16_t color)
{
    const float c = std::cos(angle_rad);
    const float s = std::sin(angle_rad);
    const int x0 = kCx + static_cast<int>(static_cast<float>(inner) * s + 0.5f);
    const int y0s = kCy - static_cast<int>(static_cast<float>(inner) * c + 0.5f);
    const int x1 = kCx + static_cast<int>(static_cast<float>(outer) * s + 0.5f);
    const int y1 = kCy - static_cast<int>(static_cast<float>(outer) * c + 0.5f);
    fill_segment(pixels, width, y0, rows, x0, y0s, x1, y1, 1, color);
}

void draw_bank_index(uint16_t *pixels, int width, int y0, int rows, float roll)
{
    const float s = std::sin(roll);
    const float c = std::cos(roll);
    const float r = static_cast<float>(kRadius);
    const float tx = static_cast<float>(kCx) + r * s;
    const float ty = static_cast<float>(kCy) - r * c;
    const float px = c;
    const float py = s;
    const float ix = -s;
    const float iy = c;
    float tri[6] = {
        tx + 9.0f * ix,
        ty + 9.0f * iy,
        tx - 7.0f * px,
        ty - 7.0f * py,
        tx + 7.0f * px,
        ty + 7.0f * py,
    };
    fill_convex(pixels, width, y0, rows, tri, 3, kIndex);
}

void draw_horizon_line(uint16_t *pixels, int width, int y0, int rows, float nx,
                       float ny, float along)
{
    const float px = -ny;
    const float py = nx;
    const float span = 110.0f;
    const float sx0 = static_cast<float>(kCx) + nx * along - px * span;
    const float sy0 = static_cast<float>(kCy) - ny * along + py * span;
    const float sx1 = static_cast<float>(kCx) + nx * along + px * span;
    const float sy1 = static_cast<float>(kCy) - ny * along - py * span;
    fill_segment(pixels, width, y0, rows, static_cast<int>(sx0 + 0.5f),
                 static_cast<int>(sy0 + 0.5f), static_cast<int>(sx1 + 0.5f),
                 static_cast<int>(sy1 + 0.5f), 1, kHorizon);
}

void draw_ladder(uint16_t *pixels, int width, int y0, int rows, float nx, float ny,
                 float horizon_along)
{
    const float px = -ny;
    const float py = nx;
    const float marks[4] = {10.0f, 20.0f, -10.0f, -20.0f};
    for (int i = 0; i < 4; ++i) {
        const float along = horizon_along + marks[i] * kDeg * kPxPerRad;
        if (std::fabs(along) > static_cast<float>(kRadius) - 12.0f) {
            continue;
        }
        const float cx = static_cast<float>(kCx) + nx * along;
        const float cy = static_cast<float>(kCy) - ny * along;
        const float half = 16.0f;
        const float gap = 5.0f;
        fill_segment(pixels, width, y0, rows,
                     static_cast<int>(cx - px * half + 0.5f),
                     static_cast<int>(cy + py * half + 0.5f),
                     static_cast<int>(cx - px * gap + 0.5f),
                     static_cast<int>(cy + py * gap + 0.5f), 1, kHorizon);
        fill_segment(pixels, width, y0, rows,
                     static_cast<int>(cx + px * gap + 0.5f),
                     static_cast<int>(cy - py * gap + 0.5f),
                     static_cast<int>(cx + px * half + 0.5f),
                     static_cast<int>(cy - py * half + 0.5f), 1, kHorizon);
    }
}

void draw_chevron(uint16_t *pixels, int width, int y0, int rows)
{
    fill_segment(pixels, width, y0, rows, 64, 120, 102, 120, 2, kChevron);
    fill_segment(pixels, width, y0, rows, 138, 120, 176, 120, 2, kChevron);
    fill_segment(pixels, width, y0, rows, 102, 120, 120, 112, 2, kChevron);
    fill_segment(pixels, width, y0, rows, 138, 120, 120, 112, 2, kChevron);
    fill_disc(pixels, width, y0, rows, kCx, kCy, 3, kChevron);
    fill_disc(pixels, width, y0, rows, kCx, kCy, 1, kCenterDot);
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
    snapshot.ladder_ok = motion_.gyro_abs < kLadderGyroMax &&
                         std::fabs(motion_.pitch) < 50.0f * kDeg;
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
        ESP_LOGI(kTag, "horizon align epoch=%" PRIu32, epoch_);
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
    const float ux = horizon.up_x;
    const float uy = horizon.up_y;
    const float uz = horizon.up_z;
    const float uxy = std::sqrt(ux * ux + uy * uy);
    const bool zenith = uxy < 0.045f;
    float nx = 0.0f;
    float ny = 1.0f;
    float along = 0.0f;
    if (!zenith) {
        nx = ux / uxy;
        ny = uy / uxy;
        along = -std::atan2(uz, uxy) * kPxPerRad;
    }

    for (int local_y = 0; local_y < rows; ++local_y) {
        const int y = y0 + local_y;
        const int dy = y - kCy;
        uint16_t *row = pixels + local_y * width;
        for (int x = 0; x < width; ++x) {
            const int dx = x - kCx;
            if (dx * dx + dy * dy > kRadiusSq) {
                row[x] = kBezel;
                continue;
            }
            if (zenith) {
                row[x] = uz >= 0.0f ? kSky : kGround;
                continue;
            }
            const float proj =
                static_cast<float>(dx) * nx + static_cast<float>(-dy) * ny;
            row[x] = proj > along ? kSky : kGround;
        }
    }

    if (!zenith) {
        draw_horizon_line(pixels, width, y0, rows, nx, ny, along);
        if (horizon.ladder_ok) {
            draw_ladder(pixels, width, y0, rows, nx, ny, along);
        }
    }

    constexpr float kTickDeg[9] = {0.0f, 10.0f, -10.0f, 20.0f, -20.0f,
                                   30.0f, -30.0f, 60.0f, -60.0f};
    for (int i = 0; i < 9; ++i) {
        const int inner = (i == 0 || i >= 5) ? 106 : 111;
        draw_bezel_tick(pixels, width, y0, rows, kTickDeg[i] * kDeg, inner, kRadius,
                        kTick);
    }
    draw_bank_index(pixels, width, y0, rows, horizon.roll);
    draw_chevron(pixels, width, y0, rows);
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
