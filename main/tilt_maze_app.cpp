#include "tilt_maze_app.hpp"

#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

namespace fluid_demo {

namespace {

constexpr const char *kTag = "tilt_maze";
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 240;

// Logical RGB565 colors; render() converts each stripe to wire order.
constexpr uint16_t kBackground = 0x1103;  // deep blue-black surround
constexpr uint16_t kFloor = 0xE6F9;       // warm, high-contrast maze floor
constexpr uint16_t kWall = 0x3227;        // dense indigo wall mass
constexpr uint16_t kMuted = 0x7C4F;       // wall edge / inactive indicator
constexpr uint16_t kBall = 0xDAA7;        // warm amber player ball
constexpr uint16_t kGoal = 0xD527;        // saturated coral goal
constexpr uint16_t kWin = 0xF6AF;         // bright victory gold
constexpr uint16_t kTaskRunning = 0x2E66;    // green: executing now
constexpr uint16_t kTaskReady = 0x4D7F;      // blue: runnable
constexpr uint16_t kTaskBlocked = 0xE5A3;    // amber: waiting
constexpr uint16_t kTaskSuspended = 0xA2BC;  // magenta: explicitly suspended
constexpr uint16_t kInternalRam = 0x3E8B;    // green internal-memory gauge
constexpr uint16_t kPsram = 0x459D;          // blue external-memory gauge

constexpr std::array<int, 5> kTaskNodeY{{48, 84, 120, 156, 198}};
constexpr std::array<SystemTaskKind, 5> kTaskKinds{{
    SystemTaskKind::Coordinator,
    SystemTaskKind::Sensor,
    SystemTaskKind::Update,
    SystemTaskKind::Render,
    SystemTaskKind::Console,
}};
constexpr std::array<int, 8> kOctantX{{0, 6, 8, 6, 0, -6, -8, -6}};
constexpr std::array<int, 8> kOctantY{{-8, -6, 0, 6, 8, 6, 0, -6}};
constexpr uint32_t kInternalGaugeSegmentBytes = 32u * 1024u;
constexpr uint32_t kPsramGaugeSegmentBytes = 1024u * 1024u;

constexpr float kStartX = 51.0f;
constexpr float kStartY = 197.0f;
constexpr float kGoalX = 189.0f;
constexpr float kGoalY = 59.0f;
constexpr float kBallRadius = 7.0f;
constexpr float kGoalCaptureRadius = 18.0f;
constexpr float kFloorLeft = 28.0f;
constexpr float kFloorTop = 36.0f;
constexpr float kFloorRight = 212.0f;
constexpr float kFloorBottom = 220.0f;

// The three alternating barriers create one deliberately readable S route:
// right, up, left, up, right, up.
struct WallRect {
    float left;
    float top;
    float right;
    float bottom;
};
constexpr std::array<WallRect, 3> kMazeWalls{{
    {28.0f, 170.0f, 166.0f, 178.0f},
    {70.0f, 124.0f, 212.0f, 132.0f},
    {28.0f, 78.0f, 166.0f, 86.0f},
}};

constexpr float kSubstepDt = 1.0f / 60.0f;
constexpr int kSubstepsPerUpdate = 2;
constexpr float kAccelerationScale = 24.0f;  // apparent units -> px/s^2
constexpr float kDampingPerSubstep = 0.985f;
constexpr float kMaximumSpeed = 96.0f;
constexpr float kRestSpeed = 0.025f;
constexpr float kRestAcceleration = 0.025f;
constexpr uint8_t kProgressPipCount = 6;
constexpr uint8_t kRoundPipCount = 4;

constexpr uint8_t kLauncherGoalBitmap[8] = {
    0b00000110,
    0b00001001,
    0b00001001,
    0b00000110,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
};
constexpr uint8_t kLauncherBallBitmap[8] = {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b01110000,
    0b11111000,
    0b01110000,
};
constexpr LauncherVisual kLauncherVisual{
    kBackground,
    kWall,
    kMuted,
    kGoal,
    kGoal,
    kBall,
    kLauncherGoalBitmap,
    kLauncherBallBitmap,
};

inline int min_int(int a, int b) { return a < b ? a : b; }
inline int max_int(int a, int b) { return a > b ? a : b; }
inline int abs_int(int value) { return value < 0 ? -value : value; }
inline float clamp_float(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}
inline bool finite_vec(const Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}
bool circle_overlaps(float cx, float cy, const WallRect &wall)
{
    const float closest_x = clamp_float(cx, wall.left, wall.right);
    const float closest_y = clamp_float(cy, wall.top, wall.bottom);
    const float dx = cx - closest_x;
    const float dy = cy - closest_y;
    return dx * dx + dy * dy < kBallRadius * kBallRadius;
}

}  // namespace

TiltMazeApp s_tilt_maze_app;

const LauncherVisual *TiltMazeApp::launcher_visual() const
{
    return &kLauncherVisual;
}

void TiltMazeApp::on_system_telemetry(const SystemTelemetry &telemetry)
{
    portENTER_CRITICAL(&telemetry_mux_);
    latest_system_telemetry_ = telemetry;
    if (frozen_telemetry_generation_ == 0u && telemetry.generation != 0u) {
        frozen_telemetry_generation_ = telemetry.generation;
        frozen_internal_free_bytes_ = telemetry.internal_free_bytes;
        frozen_internal_largest_free_block_ =
            telemetry.internal_largest_free_block;
    }
    portEXIT_CRITICAL(&telemetry_mux_);
}

esp_err_t TiltMazeApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;
    }

    completed_rounds_ = 0;
    reset_maze(false);
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    setup_done_ = true;
    return ESP_OK;
}

esp_err_t TiltMazeApp::enter()
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }

    // Re-entry preserves the round and pose but discards stale motion/render data.
    frames_.drain();
    velocity_x_ = 0.0f;
    velocity_y_ = 0.0f;
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
    portENTER_CRITICAL(&telemetry_mux_);
    latest_system_telemetry_ = {};
    portEXIT_CRITICAL(&telemetry_mux_);
    return ESP_OK;
}

void TiltMazeApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
}

ShellAction TiltMazeApp::handle_event(AppEvent event)
{
    if (event == AppEvent::PlusPress) {
        reset_requested_.store(true, std::memory_order_release);
    }
    return ShellAction::None;
}

void TiltMazeApp::on_touch(const TouchEvent &event)
{
    static_cast<void>(event);
    reset_requested_.store(true, std::memory_order_release);
}

bool TiltMazeApp::on_motion(const MotionTick &tick)
{
    const bool physical_valid =
        tick.fresh && std::isfinite(tick.dt) && tick.dt > 0.0f &&
        finite_vec(tick.accel_mps2) && finite_vec(tick.gyro_rads);
    bool physical_accepted = false;
    Vec3 filtered{};
    if (physical_valid) {
        filtered = filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
        physical_accepted = filter_.last_sample_accepted() && finite_vec(filtered);
    }

    const bool override_valid = tick.override_active && finite_vec(tick.apparent_accel);
    if (!physical_accepted && !override_valid) {
        portENTER_CRITICAL(&motion_mux_);
        motion_.valid = false;
        portEXIT_CRITICAL(&motion_mux_);
        return false;
    }

    const Vec3 apparent = override_valid ? tick.apparent_accel : filtered;
    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent = apparent;
    motion_.valid = true;
    if (physical_accepted) {
        motion_.raw = tick.accel_mps2;
    }
    portEXIT_CRITICAL(&motion_mux_);
    return physical_accepted;
}


void TiltMazeApp::freeze_memory_target()
{
    portENTER_CRITICAL(&telemetry_mux_);
    frozen_telemetry_generation_ = latest_system_telemetry_.generation;
    frozen_internal_free_bytes_ = latest_system_telemetry_.internal_free_bytes;
    frozen_internal_largest_free_block_ =
        latest_system_telemetry_.internal_largest_free_block;
    portEXIT_CRITICAL(&telemetry_mux_);
}
void TiltMazeApp::reset_maze(bool log_transition)
{
    freeze_memory_target();
    ++epoch_;
    if (epoch_ == 0u) {
        epoch_ = 1u;
    }
    ball_x_ = kStartX;
    ball_y_ = kStartY;
    velocity_x_ = 0.0f;
    velocity_y_ = 0.0f;
    progress_ = 0;
    solved_ = false;
    if (log_transition) {
        ESP_LOGI(kTag, "maze reset epoch=%" PRIu32, epoch_);
    }
}

void TiltMazeApp::move_x(float delta)
{
    if (delta == 0.0f) {
        return;
    }
    float candidate = ball_x_ + delta;
    const float minimum = kFloorLeft + kBallRadius;
    const float maximum = kFloorRight - kBallRadius;
    if (candidate < minimum) {
        candidate = minimum;
        velocity_x_ = 0.0f;
    } else if (candidate > maximum) {
        candidate = maximum;
        velocity_x_ = 0.0f;
    }

    for (const WallRect &wall : kMazeWalls) {
        if (!circle_overlaps(candidate, ball_y_, wall)) {
            continue;
        }
        if (delta > 0.0f) {
            candidate = wall.left - kBallRadius;
        } else {
            candidate = wall.right + kBallRadius;
        }
        velocity_x_ = 0.0f;
    }
    ball_x_ = clamp_float(candidate, minimum, maximum);
}

void TiltMazeApp::move_y(float delta)
{
    if (delta == 0.0f) {
        return;
    }
    float candidate = ball_y_ + delta;
    const float minimum = kFloorTop + kBallRadius;
    const float maximum = kFloorBottom - kBallRadius;
    if (candidate < minimum) {
        candidate = minimum;
        velocity_y_ = 0.0f;
    } else if (candidate > maximum) {
        candidate = maximum;
        velocity_y_ = 0.0f;
    }

    for (const WallRect &wall : kMazeWalls) {
        if (!circle_overlaps(ball_x_, candidate, wall)) {
            continue;
        }
        if (delta > 0.0f) {
            candidate = wall.top - kBallRadius;
        } else {
            candidate = wall.bottom + kBallRadius;
        }
        velocity_y_ = 0.0f;
    }
    ball_y_ = clamp_float(candidate, minimum, maximum);
}

void TiltMazeApp::update_progress_and_win()
{
    // Progress is monotonic and follows the six visible corridor legs. These
    // gates are indicators only; the physical walls remain the route oracle.
    if (progress_ == 0u && ball_x_ >= 184.0f && ball_y_ >= 180.0f) {
        progress_ = 1;
    }
    if (progress_ == 1u && ball_x_ >= 180.0f && ball_y_ <= 145.0f) {
        progress_ = 2;
    }
    if (progress_ == 2u && ball_x_ <= 56.0f && ball_y_ <= 145.0f) {
        progress_ = 3;
    }
    if (progress_ == 3u && ball_x_ <= 60.0f && ball_y_ <= 99.0f) {
        progress_ = 4;
    }
    if (progress_ == 4u && ball_x_ >= 180.0f && ball_y_ <= 99.0f) {
        progress_ = 5;
    }

    const float dx = ball_x_ - kGoalX;
    const float dy = ball_y_ - kGoalY;
    if (!solved_ && dx * dx + dy * dy <= kGoalCaptureRadius * kGoalCaptureRadius) {
        solved_ = true;
        progress_ = kProgressPipCount;
        velocity_x_ = 0.0f;
        velocity_y_ = 0.0f;
        ++completed_rounds_;
        ESP_LOGI(kTag, "maze solved epoch=%" PRIu32 " target_bytes=%" PRIu32,
                 epoch_, frozen_internal_largest_free_block_);
    }
}

void TiltMazeApp::step_substep(float apparent_x, float apparent_y)
{
    if (solved_) {
        return;
    }

    const float accel_x =
        clamp_float(apparent_x, -18.0f, 18.0f) * kAccelerationScale;
    const float accel_y =
        clamp_float(apparent_y, -18.0f, 18.0f) * kAccelerationScale;
    velocity_x_ = (velocity_x_ + accel_x * kSubstepDt) * kDampingPerSubstep;
    velocity_y_ = (velocity_y_ + accel_y * kSubstepDt) * kDampingPerSubstep;
    velocity_x_ = clamp_float(velocity_x_, -kMaximumSpeed, kMaximumSpeed);
    velocity_y_ = clamp_float(velocity_y_, -kMaximumSpeed, kMaximumSpeed);
    if (std::fabs(velocity_x_) < kRestSpeed && std::fabs(accel_x) < kRestAcceleration) {
        velocity_x_ = 0.0f;
    }
    if (std::fabs(velocity_y_) < kRestSpeed && std::fabs(accel_y) < kRestAcceleration) {
        velocity_y_ = 0.0f;
    }

    move_x(velocity_x_ * kSubstepDt);
    move_y(velocity_y_ * kSubstepDt);
    update_progress_and_win();
}

esp_err_t TiltMazeApp::update(float dt)
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t update_start = esp_timer_get_time();
    const bool reset = reset_requested_.exchange(false, std::memory_order_acq_rel);
    if (reset) {
        // Publish the reset pose before applying any held tilt.
        reset_maze(true);
    } else {
        Vec3 apparent{};
        portENTER_CRITICAL(&motion_mux_);
        if (motion_.valid) {
            apparent = motion_.apparent;
        }
        portEXIT_CRITICAL(&motion_mux_);

        for (int step = 0; step < kSubstepsPerUpdate; ++step) {
            step_substep(apparent.x, apparent.y);
        }
    }

    if (!std::isfinite(ball_x_) || !std::isfinite(ball_y_) ||
        !std::isfinite(velocity_x_) || !std::isfinite(velocity_y_)) {
        nonfinite_resets_.fetch_add(1, std::memory_order_relaxed);
        reset_maze(true);
    }

    MazeFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        fill_snapshot(*snapshot);
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - update_start),
                      std::memory_order_relaxed);
    return ESP_OK;
}

void TiltMazeApp::fill_snapshot(MazeFrame &snapshot)
{
    ++sequence_;
    if (sequence_ == 0u) {
        sequence_ = 1u;
    }
    snapshot.sequence = sequence_;
    snapshot.epoch = epoch_;
    snapshot.ball_x = ball_x_;
    snapshot.ball_y = ball_y_;
    snapshot.progress = progress_;
    snapshot.round_pips = static_cast<uint8_t>(
        completed_rounds_ < kRoundPipCount ? completed_rounds_ : kRoundPipCount);
    snapshot.solved = solved_;
    portENTER_CRITICAL(&telemetry_mux_);
    snapshot.system = latest_system_telemetry_;
    snapshot.frozen_telemetry_generation = frozen_telemetry_generation_;
    snapshot.frozen_internal_free_bytes = frozen_internal_free_bytes_;
    snapshot.frozen_internal_largest_free_block =
        frozen_internal_largest_free_block_;
    portEXIT_CRITICAL(&telemetry_mux_);
}

AppStats TiltMazeApp::stats()
{
    AppStats result{};
    result.count = 1;
    result.epoch = published_epoch_.load(std::memory_order_relaxed);
    result.candidate_checks = 0;
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

void TiltMazeApp::draw_rect(uint16_t *pixels, int width, int y0, int rows,
                            int left, int top, int right, int bottom,
                            uint16_t color)
{
    if (pixels == nullptr || width <= 0 || rows <= 0 || left >= right || top >= bottom) {
        return;
    }
    const int clipped_left = max_int(0, left);
    const int clipped_right = min_int(width, right);
    const int clipped_top = max_int(y0, top);
    const int clipped_bottom = min_int(y0 + rows, bottom);
    if (clipped_left >= clipped_right || clipped_top >= clipped_bottom) {
        return;
    }
    for (int y = clipped_top; y < clipped_bottom; ++y) {
        uint16_t *row = pixels + (y - y0) * width;
        for (int x = clipped_left; x < clipped_right; ++x) {
            row[x] = color;
        }
    }
}

void TiltMazeApp::draw_disc(uint16_t *pixels, int width, int y0, int rows,
                            int cx, int cy, int radius, uint16_t color)
{
    if (pixels == nullptr || width <= 0 || rows <= 0 || radius < 0) {
        return;
    }
    const int top = max_int(y0, cy - radius);
    const int bottom = min_int(y0 + rows - 1, cy + radius);
    const int left = max_int(0, cx - radius);
    const int right = min_int(width - 1, cx + radius);
    const int radius_squared = radius * radius;
    for (int y = top; y <= bottom; ++y) {
        uint16_t *row = pixels + (y - y0) * width;
        const int dy = y - cy;
        for (int x = left; x <= right; ++x) {
            const int dx = x - cx;
            if (dx * dx + dy * dy <= radius_squared) {
                row[x] = color;
            }
        }
    }
}

void TiltMazeApp::draw_ring(uint16_t *pixels, int width, int y0, int rows,
                            int cx, int cy, int outer_radius, int inner_radius,
                            uint16_t outer, uint16_t inner)
{
    draw_disc(pixels, width, y0, rows, cx, cy, outer_radius, outer);
    draw_disc(pixels, width, y0, rows, cx, cy, inner_radius, inner);
}

void TiltMazeApp::draw_segment(uint16_t *pixels, int width, int y0, int rows,
                               int x0, int y0_screen, int x1, int y1,
                               int radius, uint16_t color)
{
    const int dx = x1 - x0;
    const int dy = y1 - y0_screen;
    const int extent = max_int(abs_int(dx), abs_int(dy));
    const int stride = max_int(1, radius);
    const int samples = max_int(1, extent / stride);
    for (int sample = 0; sample <= samples; ++sample) {
        draw_disc(pixels, width, y0, rows,
                  x0 + (dx * sample) / samples,
                  y0_screen + (dy * sample) / samples,
                  radius, color);
    }
}

uint16_t TiltMazeApp::task_state_color(SystemTaskState state)
{
    switch (state) {
    case SystemTaskState::Running:
        return kTaskRunning;
    case SystemTaskState::Ready:
        return kTaskReady;
    case SystemTaskState::Blocked:
        return kTaskBlocked;
    case SystemTaskState::Suspended:
        return kTaskSuspended;
    case SystemTaskState::Unknown:
    default:
        return kMuted;
    }
}

void TiltMazeApp::draw_task_node(const SystemTaskTelemetry &task,
                                 std::size_t index, uint16_t *pixels,
                                 int width, int y0, int rows)
{
    if (index >= kTaskNodeY.size()) {
        return;
    }

    // Pinned core 0/1 records live on the left/right rail. An unpinned or
    // not-yet-sampled record stays on the center rail rather than inventing
    // affinity. Placement and paint are visual only, never collision input.
    int x = 120;
    if (task.valid && task.core_id == 0) {
        x = 12;
    } else if (task.valid && task.core_id == 1) {
        x = 228;
    }
    const int y = kTaskNodeY[index];
    const SystemTaskState state =
        task.valid ? task.state : SystemTaskState::Unknown;
    const uint16_t color = task_state_color(state);
    const uint32_t bounded_words =
        task.stack_high_water_words < 2048u ? task.stack_high_water_words : 2048u;
    const int fill_radius =
        task.valid ? 1 + static_cast<int>((bounded_words * 4u) / 2048u) : 1;

    draw_disc(pixels, width, y0, rows, x, y, 7, kWall);
    draw_disc(pixels, width, y0, rows, x, y, fill_radius, color);

    // Five tiny, font-free center marks identify the fixed task order:
    // coordinator dot, sensor dash, update stem, render cross, console box.
    const SystemTaskKind kind = task.valid ? task.kind : kTaskKinds[index];
    switch (kind) {
    case SystemTaskKind::Coordinator:
        draw_disc(pixels, width, y0, rows, x, y, 1, kFloor);
        break;
    case SystemTaskKind::Sensor:
        draw_rect(pixels, width, y0, rows, x - 2, y, x + 3, y + 1, kFloor);
        break;
    case SystemTaskKind::Update:
        draw_rect(pixels, width, y0, rows, x, y - 2, x + 1, y + 3, kFloor);
        break;
    case SystemTaskKind::Render:
        draw_rect(pixels, width, y0, rows, x - 2, y - 2, x - 1, y - 1, kFloor);
        draw_rect(pixels, width, y0, rows, x + 1, y - 2, x + 2, y - 1, kFloor);
        draw_rect(pixels, width, y0, rows, x - 2, y + 1, x - 1, y + 2, kFloor);
        draw_rect(pixels, width, y0, rows, x + 1, y + 1, x + 2, y + 2, kFloor);
        break;
    case SystemTaskKind::Console:
        draw_rect(pixels, width, y0, rows, x - 2, y - 2, x + 3, y + 3, kFloor);
        draw_rect(pixels, width, y0, rows, x - 1, y - 1, x + 2, y + 2, color);
        break;
    }
}

void TiltMazeApp::draw_memory_gauge(uint16_t *pixels, int width, int y0,
                                    int rows, int left, uint32_t free_bytes,
                                    uint32_t bytes_per_segment, uint16_t color,
                                    bool valid)
{
    constexpr int kSegments = 8;
    constexpr int kCellStep = 11;
    constexpr int kFillWidth = 8;
    for (int segment = 0; segment < kSegments; ++segment) {
        const int x = left + segment * kCellStep;
        draw_rect(pixels, width, y0, rows, x, 231, x + 10, 239, kWall);
        draw_rect(pixels, width, y0, rows, x + 1, 233, x + 9, 237, kBackground);
        if (!valid || bytes_per_segment == 0u) {
            continue;
        }
        const uint64_t threshold =
            static_cast<uint64_t>(segment) * bytes_per_segment;
        if (free_bytes <= threshold) {
            continue;
        }
        const uint64_t available = static_cast<uint64_t>(free_bytes) - threshold;
        const uint64_t in_segment =
            available < bytes_per_segment ? available : bytes_per_segment;
        const int fill_width = static_cast<int>(
            (in_segment * kFillWidth + bytes_per_segment - 1u) /
            bytes_per_segment);
        draw_rect(pixels, width, y0, rows, x + 1, 233,
                  x + 1 + fill_width, 237, color);
    }
}

void TiltMazeApp::draw_goal(const MazeFrame &maze, uint16_t *pixels,
                            int width, int y0, int rows)
{
    const bool valid = maze.frozen_telemetry_generation != 0u &&
                       maze.frozen_internal_free_bytes != 0u;
    if (!valid) {
        // A neutral open target and all eight unlit ticks communicate that no
        // allocator sample existed at this round's reset boundary.
        draw_ring(pixels, width, y0, rows, 189, 59, 14, 11, kMuted, kFloor);
        for (std::size_t i = 0; i < kOctantX.size(); ++i) {
            draw_disc(pixels, width, y0, rows,
                      189 + (kOctantX[i] * 14) / 8,
                      59 + (kOctantY[i] * 14) / 8, 1, kMuted);
        }
        draw_disc(pixels, width, y0, rows, 189, 59, 2, kMuted);
        return;
    }

    const uint32_t largest =
        maze.frozen_internal_largest_free_block <
                maze.frozen_internal_free_bytes
            ? maze.frozen_internal_largest_free_block
            : maze.frozen_internal_free_bytes;
    const uint64_t free_bytes = maze.frozen_internal_free_bytes;
    int outer_radius =
        12 + static_cast<int>((static_cast<uint64_t>(largest) * 4u +
                               free_bytes / 2u) /
                              free_bytes);
    outer_radius = min_int(16, max_int(12, outer_radius));
    int lit_segments =
        static_cast<int>((static_cast<uint64_t>(largest) * 8u +
                          free_bytes / 2u) /
                         free_bytes);
    if (largest != 0u) {
        lit_segments = max_int(1, lit_segments);
    }
    lit_segments = min_int(8, lit_segments);

    draw_ring(pixels, width, y0, rows, 189, 59, outer_radius,
              outer_radius - 3, kGoal, kFloor);
    for (std::size_t i = 0; i < kOctantX.size(); ++i) {
        const uint16_t tick =
            static_cast<int>(i) < lit_segments ? kGoal : kMuted;
        draw_disc(pixels, width, y0, rows,
                  189 + (kOctantX[i] * outer_radius) / 8,
                  59 + (kOctantY[i] * outer_radius) / 8, 2, tick);
    }
    draw_ring(pixels, width, y0, rows, 189, 59, 7, 4, kGoal, kFloor);
    draw_disc(pixels, width, y0, rows, 189, 59, 2, kGoal);
}

void TiltMazeApp::raster_stripe(const MazeFrame &maze, uint16_t *pixels,
                                int width, int y0, int rows)
{
    for (int local_y = 0; local_y < rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        for (int x = 0; x < width; ++x) {
            row[x] = kBackground;
        }
    }

    // Two separate indicator groups keep route progress and completed rounds
    // legible without text at the panel's native 240x240 resolution.
    for (uint8_t pip = 0; pip < kProgressPipCount; ++pip) {
        const int x = 85 + static_cast<int>(pip) * 14;
        draw_disc(pixels, width, y0, rows, x, 14, 4, kWall);
        const uint16_t fill = pip < maze.progress
                                  ? kBall
                                  : (pip == maze.progress && !maze.solved ? kGoal : kMuted);
        draw_disc(pixels, width, y0, rows, x, 14, 2, fill);
    }
    for (uint8_t pip = 0; pip < kRoundPipCount; ++pip) {
        const int x = 196 + static_cast<int>(pip) * 10;
        draw_disc(pixels, width, y0, rows, x, 14, 3, kWall);
        if (pip < maze.round_pips) {
            draw_disc(pixels, width, y0, rows, x, 14, 2, kWin);
        }
    }

    // Solid outer wall, inset floor, and three full-weight alternating bars.
    draw_rect(pixels, width, y0, rows, 20, 28, 220, 228, kWall);
    draw_rect(pixels, width, y0, rows, 28, 36, 212, 220, kFloor);
    for (const WallRect &wall : kMazeWalls) {
        const int left = static_cast<int>(wall.left);
        const int top = static_cast<int>(wall.top);
        const int right = static_cast<int>(wall.right);
        const int bottom = static_cast<int>(wall.bottom);
        draw_rect(pixels, width, y0, rows, left, top, right, bottom, kWall);
        draw_rect(pixels, width, y0, rows, left, top, right, top + 2, kMuted);
    }

    // Absolute free-byte gauges use fixed, saturating units rather than
    // implying a percentage of an unavailable total-capacity value. The
    // square marker is internal RAM (32 KiB/cell); the ring is PSRAM
    // (1 MiB/cell). Partial cells preserve sub-unit changes.
    const bool memory_valid = maze.system.generation != 0u;
    draw_rect(pixels, width, y0, rows, 11, 232, 18, 239,
              memory_valid ? kInternalRam : kMuted);
    draw_memory_gauge(pixels, width, y0, rows, 22,
                      maze.system.internal_free_bytes,
                      kInternalGaugeSegmentBytes, kInternalRam, memory_valid);
    draw_ring(pixels, width, y0, rows, 122, 235, 4, 2,
              memory_valid ? kPsram : kMuted, kBackground);
    draw_memory_gauge(pixels, width, y0, rows, 130,
                      maze.system.psram_free_bytes,
                      kPsramGaugeSegmentBytes, kPsram, memory_valid);

    // Fixed array order is Coordinator, Sensor, Update, Render, Console.
    // State/core/stack affect only these overlays; the S-route geometry above
    // and every collision constant remain independent of telemetry.
    for (std::size_t index = 0; index < 5; ++index) {
        draw_task_node(maze.system.tasks[index], index,
                       pixels, width, y0, rows);
    }

    // Start is the allocation token's origin; reaching the goal changes only
    // game state and never calls an allocator.
    draw_ring(pixels, width, y0, rows, 51, 197, 11, 8, kMuted, kFloor);
    draw_disc(pixels, width, y0, rows, 51, 197, 2, kMuted);
    draw_rect(pixels, width, y0, rows, 49, 183, 53, 188, kMuted);
    draw_rect(pixels, width, y0, rows, 49, 206, 53, 211, kMuted);
    draw_rect(pixels, width, y0, rows, 37, 195, 42, 199, kMuted);
    draw_rect(pixels, width, y0, rows, 60, 195, 65, 199, kMuted);

    // Frozen largest/free internal-RAM ratio controls this target for the
    // entire round, even while the live gauges and task nodes keep changing.
    draw_goal(maze, pixels, width, y0, rows);

    // A dark footprint and one small highlight keep the ball unmistakable on
    // both the pale floor and the coral target without antialiasing.
    const int ball_x = static_cast<int>(maze.ball_x + 0.5f);
    const int ball_y = static_cast<int>(maze.ball_y + 0.5f);
    draw_disc(pixels, width, y0, rows, ball_x + 1, ball_y + 2, 8, kWall);
    draw_disc(pixels, width, y0, rows, ball_x, ball_y, 7, kBall);
    draw_disc(pixels, width, y0, rows, ball_x - 2, ball_y - 2, 2, kWin);

    if (maze.solved) {
        // Gold frame + central check medal is deliberately large and flat: the
        // solved state remains unmistakable in a reduced framebuffer capture.
        draw_rect(pixels, width, y0, rows, 20, 28, 220, 34, kWin);
        draw_rect(pixels, width, y0, rows, 20, 222, 220, 228, kWin);
        draw_rect(pixels, width, y0, rows, 20, 28, 26, 228, kWin);
        draw_rect(pixels, width, y0, rows, 214, 28, 220, 228, kWin);
        draw_disc(pixels, width, y0, rows, 120, 128, 48, kWin);
        draw_disc(pixels, width, y0, rows, 120, 128, 39, kBackground);
        draw_segment(pixels, width, y0, rows, 96, 128, 113, 145, 5, kWin);
        draw_segment(pixels, width, y0, rows, 113, 145, 147, 108, 5, kWin);
    }
}

bool TiltMazeApp::render(DisplayFrame &frame)
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

    const MazeFrame *maze = frames_.acquire_latest();
    if (maze == nullptr) {
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
            raster_stripe(*maze, pixels, frame.width, stripe_y, stripe_rows);
            const int pixel_count = frame.width * stripe_rows;
            for (int pixel = 0; pixel < pixel_count; ++pixel) {
                pixels[pixel] = __builtin_bswap16(pixels[pixel]);
            }
            raster_total +=
                static_cast<uint32_t>(esp_timer_get_time() - raster_start);
            result = frame.ops.submit(frame.transport, stripe, stripe_y,
                                      stripe_rows, pixels);
            if (result != ESP_OK) {
                break;
            }
        }
        if (result == ESP_OK) {
            result = frame.ops.finish(frame.transport);
        }
    }

    frames_.release(maze);
    if (result != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s", esp_err_to_name(result));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start);
    raster_us_ = raster_total + frame.ops.capture_copy_us(frame.transport);
    return true;
}

}  // namespace fluid_demo
