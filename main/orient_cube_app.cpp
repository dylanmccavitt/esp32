#include "orient_cube_app.hpp"

#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "draw.hpp"

namespace fluid_demo {

namespace {

constexpr const char *kTag = "orient_cube";
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 240;

constexpr uint16_t kBackground = 0x10A3;
constexpr uint16_t kBand = 0x2A28;
constexpr uint16_t kMuted = 0x7C4F;
constexpr uint16_t kAccent = 0xF50A;
constexpr uint16_t kFront = 0x3D9B;
constexpr uint16_t kFrontInset = 0x1C75;
constexpr uint16_t kBack = 0x3186;
constexpr uint16_t kBackMark = 0x4A69;
constexpr uint16_t kRight = 0x1C86;
constexpr uint16_t kLeft = 0xC165;
constexpr uint16_t kTop = 0xDEFB;
constexpr uint16_t kUsb = 0xD3E4;
constexpr uint16_t kUsbMark = 0xFEC0;
constexpr uint16_t kEdge = 0x1082;

constexpr uint8_t kLauncherCubeBitmap[8] = {
    0b00011100,
    0b00110110,
    0b01100011,
    0b01100011,
    0b00110110,
    0b00011100,
    0b00001000,
    0b00000000,
};
constexpr uint8_t kLauncherCubeTopBitmap[8] = {
    0b00011100,
    0b00100010,
    0b01000001,
    0b01000001,
    0b00100010,
    0b00011100,
    0b00000000,
    0b00000000,
};
constexpr LauncherVisual kLauncherVisual{
    kBackground,
    kBand,
    kMuted,
    kAccent,
    kFront,
    kUsbMark,
    kLauncherCubeBitmap,
    kLauncherCubeTopBitmap,
};

constexpr float kHalf = 0.82f;
constexpr float kCam = 2.55f;
constexpr float kFocal = 168.0f;
constexpr float kCenter = 120.0f;

constexpr int kFaceVerts[6][4] = {
    {0, 1, 2, 3},  // screen / +Z
    {5, 4, 7, 6},  // back / -Z
    {1, 5, 6, 2},  // right / +X
    {4, 0, 3, 7},  // left / -X
    {3, 2, 6, 7},  // top / +Y
    {4, 5, 1, 0},  // USB / -Y
};
constexpr uint16_t kFaceFill[6] = {kFront, kBack, kRight, kLeft, kTop, kUsb};

inline int min_int(int a, int b) { return a < b ? a : b; }
inline bool finite_vec(const Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

void copy_matrix(float *dst, const float *src)
{
    for (int i = 0; i < 9; ++i) {
        dst[i] = src[i];
    }
}

void project_vertices(const float R[9], float out[8][3])
{
    const float model[8][3] = {
        {-kHalf, -kHalf, kHalf},  {kHalf, -kHalf, kHalf},
        {kHalf, kHalf, kHalf},    {-kHalf, kHalf, kHalf},
        {-kHalf, -kHalf, -kHalf}, {kHalf, -kHalf, -kHalf},
        {kHalf, kHalf, -kHalf},   {-kHalf, kHalf, -kHalf},
    };
    for (int i = 0; i < 8; ++i) {
        const float x = R[0] * model[i][0] + R[1] * model[i][1] + R[2] * model[i][2];
        const float y = R[3] * model[i][0] + R[4] * model[i][1] + R[5] * model[i][2];
        const float z = R[6] * model[i][0] + R[7] * model[i][1] + R[8] * model[i][2];
        float denom = kCam - z;
        if (denom < 0.25f) {
            denom = 0.25f;
        }
        const float persp = kFocal / denom;
        out[i][0] = kCenter + x * persp;
        out[i][1] = kCenter - y * persp;
        out[i][2] = z;
    }
}

float face_area(const float projected[8][3], int face)
{
    float area = 0.0f;
    for (int i = 0; i < 4; ++i) {
        const int a = kFaceVerts[face][i];
        const int b = kFaceVerts[face][(i + 1) & 3];
        area += projected[a][0] * projected[b][1] - projected[b][0] * projected[a][1];
    }
    return area;
}

void face_quad(const float projected[8][3], int face, float xy[8])
{
    for (int i = 0; i < 4; ++i) {
        const int v = kFaceVerts[face][i];
        xy[2 * i] = projected[v][0];
        xy[2 * i + 1] = projected[v][1];
    }
}

void lerp_quad(const float xy[8], float t, float out[8])
{
    const float cx = 0.25f * (xy[0] + xy[2] + xy[4] + xy[6]);
    const float cy = 0.25f * (xy[1] + xy[3] + xy[5] + xy[7]);
    for (int i = 0; i < 4; ++i) {
        out[2 * i] = cx + t * (xy[2 * i] - cx);
        out[2 * i + 1] = cy + t * (xy[2 * i + 1] - cy);
    }
}

void edge_strip(const float xy[8], int a, int b, float t0, float t1, float out[8])
{
    const int c = (a + 3) & 3;
    const int d = (b + 1) & 3;
    const float ax = xy[2 * a];
    const float ay = xy[2 * a + 1];
    const float bx = xy[2 * b];
    const float by = xy[2 * b + 1];
    const float cx = xy[2 * c];
    const float cy = xy[2 * c + 1];
    const float dx = xy[2 * d];
    const float dy = xy[2 * d + 1];
    out[0] = ax + t0 * (cx - ax);
    out[1] = ay + t0 * (cy - ay);
    out[2] = bx + t0 * (dx - bx);
    out[3] = by + t0 * (dy - by);
    out[4] = bx + t1 * (dx - bx);
    out[5] = by + t1 * (dy - by);
    out[6] = ax + t1 * (cx - ax);
    out[7] = ay + t1 * (cy - ay);
}

void draw_edges(const float xy[8], uint16_t *pixels, int width, int y0, int rows)
{
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) & 3;
        fill_segment(pixels, width, y0, rows, static_cast<int>(xy[2 * i] + 0.5f),
                     static_cast<int>(xy[2 * i + 1] + 0.5f),
                     static_cast<int>(xy[2 * j] + 0.5f),
                     static_cast<int>(xy[2 * j + 1] + 0.5f), 0, kEdge);
    }
}

}  // namespace

OrientCubeApp s_orient_cube_app;

const LauncherVisual *OrientCubeApp::launcher_visual() const
{
    return &kLauncherVisual;
}

esp_err_t OrientCubeApp::setup_once()
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

esp_err_t OrientCubeApp::enter()
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    frames_.drain();
    filter_.reset();
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    copy_matrix(motion_.R, filter_.matrix());
    portEXIT_CRITICAL(&motion_mux_);
    return ESP_OK;
}

void OrientCubeApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
}

ShellAction OrientCubeApp::handle_event(AppEvent event)
{
    if (event == AppEvent::PlusPress) {
        reset_requested_.store(true, std::memory_order_release);
        filter_.request_align();
    }
    return ShellAction::None;
}

bool OrientCubeApp::on_motion(const MotionTick &tick)
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

    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent = override_valid ? tick.apparent_accel : filter_.mapped_accel();
    motion_.valid = true;
    copy_matrix(motion_.R, filter_.matrix());
    if (physical_valid) {
        motion_.raw = tick.accel_mps2;
    }
    portEXIT_CRITICAL(&motion_mux_);
    nonfinite_resets_.store(filter_.nonfinite_resets(), std::memory_order_relaxed);
    return physical_accepted;
}

void OrientCubeApp::fill_snapshot(CubeFrame &snapshot)
{
    ++sequence_;
    if (sequence_ == 0u) {
        sequence_ = 1u;
    }
    snapshot.sequence = sequence_;
    snapshot.epoch = epoch_;
    portENTER_CRITICAL(&motion_mux_);
    copy_matrix(snapshot.R, motion_.R);
    portEXIT_CRITICAL(&motion_mux_);
}

esp_err_t OrientCubeApp::update(float dt)
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
        ESP_LOGI(kTag, "cube align epoch=%" PRIu32, epoch_);
    }

    CubeFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        fill_snapshot(*snapshot);
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - update_start),
                      std::memory_order_relaxed);
    return ESP_OK;
}

AppStats OrientCubeApp::stats()
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

void OrientCubeApp::raster_stripe(const float projected[8][3], const int order[6],
                                  const bool visible[6], uint16_t *pixels, int width,
                                  int y0, int rows)
{
    for (int local_y = 0; local_y < rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        for (int x = 0; x < width; ++x) {
            row[x] = kBackground;
        }
    }

    for (int n = 0; n < 6; ++n) {
        const int face = order[n];
        if (!visible[face]) {
            continue;
        }
        float xy[8];
        face_quad(projected, face, xy);
        fill_convex_quad(pixels, width, y0, rows, xy, kFaceFill[face]);

        float inset[8];
        if (face == 0) {
            lerp_quad(xy, 0.78f, inset);
            fill_convex_quad(pixels, width, y0, rows, inset, kFrontInset);
            float usb[8];
            edge_strip(xy, 0, 1, 0.06f, 0.20f, usb);
            fill_convex_quad(pixels, width, y0, rows, usb, kUsbMark);
        } else if (face == 1) {
            lerp_quad(xy, 0.35f, inset);
            fill_convex_quad(pixels, width, y0, rows, inset, kBackMark);
        } else if (face == 5) {
            lerp_quad(xy, 0.45f, inset);
            fill_convex_quad(pixels, width, y0, rows, inset, kUsbMark);
        }
        draw_edges(xy, pixels, width, y0, rows);
    }
}

bool OrientCubeApp::render(DisplayFrame &frame)
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

    const CubeFrame *cube = frames_.acquire_latest();
    if (cube == nullptr) {
        return false;
    }

    float projected[8][3];
    project_vertices(cube->R, projected);
    bool visible[6];
    float depth[6];
    int order[6];
    for (int face = 0; face < 6; ++face) {
        visible[face] = face_area(projected, face) < 0.0f;
        depth[face] = 0.25f * (projected[kFaceVerts[face][0]][2] +
                               projected[kFaceVerts[face][1]][2] +
                               projected[kFaceVerts[face][2]][2] +
                               projected[kFaceVerts[face][3]][2]);
        order[face] = face;
    }
    for (int i = 1; i < 6; ++i) {
        const int face = order[i];
        const float z = depth[face];
        int j = i;
        while (j > 0 && depth[order[j - 1]] > z) {
            order[j] = order[j - 1];
            --j;
        }
        order[j] = face;
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
            raster_stripe(projected, order, visible, pixels, frame.width, stripe_y,
                          stripe_rows);
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

    frames_.release(cube);
    if (result != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s", esp_err_to_name(result));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start);
    raster_us_ = raster_total + frame.ops.capture_copy_us(frame.transport);
    return true;
}

}  // namespace fluid_demo
