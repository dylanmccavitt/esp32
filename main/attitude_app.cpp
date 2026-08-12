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
constexpr uint16_t kSkyDeep = 0x226F;
constexpr uint16_t kGround = 0x9B43;
constexpr uint16_t kGroundDark = 0x7222;
constexpr uint16_t kHorizon = 0xFFFF;
constexpr uint16_t kHorizonEdge = 0x1082;
constexpr uint16_t kGrid = 0x82C3;
constexpr uint16_t kSun = 0xFE60;
constexpr uint16_t kPlane = 0xFFFF;
constexpr uint16_t kPlaneBody = 0xFE60;
constexpr uint16_t kPlaneOutline = 0x0841;
constexpr uint16_t kTick = 0xDEFB;
constexpr uint16_t kIndex = 0xFE60;
constexpr uint16_t kCenterDot = 0xF800;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg = kPi / 180.0f;
constexpr float kPxPerRad = static_cast<float>(kRadius) / (0.55f * kPi);

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

void draw_up_caret(uint16_t *pixels, int width, int y0, int rows)
{
    float tri[6] = {120.0f, 8.0f, 109.0f, 22.0f, 131.0f, 22.0f};
    fill_convex(pixels, width, y0, rows, tri, 3, kIndex);
}

void draw_horizon_band(uint16_t *pixels, int width, int y0, int rows, float nx,
                       float ny, float along, int radius, uint16_t color)
{
    const float px = -ny;
    const float py = nx;
    const float span = 112.0f;
    const float sx0 = static_cast<float>(kCx) + nx * along - px * span;
    const float sy0 = static_cast<float>(kCy) - ny * along + py * span;
    const float sx1 = static_cast<float>(kCx) + nx * along + px * span;
    const float sy1 = static_cast<float>(kCy) - ny * along - py * span;
    fill_segment(pixels, width, y0, rows, static_cast<int>(sx0 + 0.5f),
                 static_cast<int>(sy0 + 0.5f), static_cast<int>(sx1 + 0.5f),
                 static_cast<int>(sy1 + 0.5f), radius, color);
}

void draw_ground_grid(uint16_t *pixels, int width, int y0, int rows, float nx,
                      float ny, float along)
{
    const float px = -ny;
    const float py = nx;
    const float spacings[5] = {16.0f, 34.0f, 56.0f, 82.0f, 112.0f};
    for (int i = 0; i < 5; ++i) {
        const float dist = spacings[i];
        const float line_along = along - dist;
        if (std::fabs(line_along) > static_cast<float>(kRadius) - 6.0f) {
            continue;
        }
        const float half = 18.0f + dist * 0.55f;
        const float cx = static_cast<float>(kCx) + nx * line_along;
        const float cy = static_cast<float>(kCy) - ny * line_along;
        fill_segment(pixels, width, y0, rows,
                     static_cast<int>(cx - px * half + 0.5f),
                     static_cast<int>(cy + py * half + 0.5f),
                     static_cast<int>(cx + px * half + 0.5f),
                     static_cast<int>(cy - py * half + 0.5f), 0, kGrid);
    }
    const float vanish = 160.0f;
    const float vx = static_cast<float>(kCx) + nx * (along - vanish);
    const float vy = static_cast<float>(kCy) - ny * (along - vanish);
    const float offsets[7] = {-70.0f, -42.0f, -18.0f, 0.0f, 18.0f, 42.0f, 70.0f};
    for (int i = 0; i < 7; ++i) {
        const float hx = static_cast<float>(kCx) + nx * along + px * offsets[i];
        const float hy = static_cast<float>(kCy) - ny * along - py * offsets[i];
        fill_segment(pixels, width, y0, rows, static_cast<int>(vx + 0.5f),
                     static_cast<int>(vy + 0.5f), static_cast<int>(hx + 0.5f),
                     static_cast<int>(hy + 0.5f), 0, kGrid);
    }
}

void draw_aircraft(uint16_t *pixels, int width, int y0, int rows)
{
    fill_segment(pixels, width, y0, rows, 28, 120, 102, 120, 5, kPlaneOutline);
    fill_segment(pixels, width, y0, rows, 138, 120, 212, 120, 5, kPlaneOutline);
    fill_segment(pixels, width, y0, rows, 30, 120, 100, 120, 3, kPlane);
    fill_segment(pixels, width, y0, rows, 140, 120, 210, 120, 3, kPlane);

    fill_disc(pixels, width, y0, rows, kCx, kCy, 11, kPlaneOutline);
    fill_disc(pixels, width, y0, rows, kCx, kCy, 8, kPlaneBody);

    float nose_out[6] = {120.0f, 96.0f, 106.0f, 118.0f, 134.0f, 118.0f};
    fill_convex(pixels, width, y0, rows, nose_out, 3, kPlaneOutline);
    float nose[6] = {120.0f, 100.0f, 110.0f, 117.0f, 130.0f, 117.0f};
    fill_convex(pixels, width, y0, rows, nose, 3, kPlane);

    fill_segment(pixels, width, y0, rows, 120, 120, 120, 142, 4, kPlaneOutline);
    fill_segment(pixels, width, y0, rows, 120, 122, 120, 140, 2, kPlane);
    fill_segment(pixels, width, y0, rows, 108, 138, 132, 138, 3, kPlaneOutline);
    fill_segment(pixels, width, y0, rows, 110, 138, 130, 138, 2, kPlane);

    fill_disc(pixels, width, y0, rows, kCx, kCy, 3, kPlane);
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

    const float *R = filter_.matrix();
    Vec3 up{R[1], R[4], R[7]};
    const float mag = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
    if (!std::isfinite(mag) || mag < 1e-5f) {
        up = {0.0f, 1.0f, 0.0f};
    } else {
        up.x /= mag;
        up.y /= mag;
        up.z /= mag;
    }
    const float uxy = std::sqrt(up.x * up.x + up.y * up.y);
    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent = override_valid ? tick.apparent_accel : filter_.mapped_accel();
    motion_.valid = true;
    motion_.up_x = up.x;
    motion_.up_y = up.y;
    motion_.up_z = up.z;
    motion_.roll = std::atan2(up.x, up.y);
    motion_.pitch = std::atan2(up.z, uxy);
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
            if (proj > along) {
                const float sky_t = (proj - along) / static_cast<float>(kRadius);
                row[x] = sky_t > 0.55f ? kSkyDeep : kSky;
            } else {
                const float ground_t = (along - proj) / static_cast<float>(kRadius);
                row[x] = ground_t > 0.62f ? kGroundDark : kGround;
            }
        }
    }

    if (!zenith) {
        draw_ground_grid(pixels, width, y0, rows, nx, ny, along);
        const float sun_along = along + 42.0f;
        if (std::fabs(sun_along) < static_cast<float>(kRadius) - 16.0f) {
            fill_disc(pixels, width, y0, rows,
                      kCx + static_cast<int>(nx * sun_along + 0.5f),
                      kCy - static_cast<int>(ny * sun_along + 0.5f), 9, kSun);
        }
        draw_horizon_band(pixels, width, y0, rows, nx, ny, along, 3, kHorizonEdge);
        draw_horizon_band(pixels, width, y0, rows, nx, ny, along, 2, kHorizon);
    }

    constexpr float kTickDeg[5] = {0.0f, 15.0f, -15.0f, 30.0f, -30.0f};
    for (int i = 0; i < 5; ++i) {
        const int inner = (i == 0) ? 104 : 110;
        draw_bezel_tick(pixels, width, y0, rows, kTickDeg[i] * kDeg, inner, kRadius,
                        kTick);
    }
    draw_up_caret(pixels, width, y0, rows);
    draw_aircraft(pixels, width, y0, rows);

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
