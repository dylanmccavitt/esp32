#include "tilt_maze_app.hpp"

#include <algorithm>
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

constexpr char kTag[] = "tilt_maze";
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 240;

constexpr uint16_t kBackground = 0x1103;
constexpr uint16_t kFloor = 0xE6F9;
constexpr uint16_t kWall = 0x3227;
constexpr uint16_t kMuted = 0x7C4F;
constexpr uint16_t kBall = 0xDAA7;
constexpr uint16_t kGoal = 0xD527;
constexpr uint16_t kWin = 0xF6AF;
constexpr uint16_t kTaskRunning = 0x2E66;
constexpr uint16_t kTaskReady = 0x4D7F;
constexpr uint16_t kTaskBlocked = 0xE5A3;
constexpr uint16_t kTaskSuspended = 0xA2BC;
constexpr uint16_t kInternalRam = 0x3E8B;
constexpr uint16_t kPsram = 0x459D;

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
constexpr float kLowerRightCheckpointX = 184.0f;
constexpr float kLowerRightCheckpointY = 180.0f;
constexpr float kMiddleRightCheckpointX = 180.0f;
constexpr float kMiddleCheckpointY = 145.0f;
constexpr float kMiddleLeftCheckpointX = 56.0f;
constexpr float kUpperLeftCheckpointX = 60.0f;
constexpr float kUpperCheckpointY = 99.0f;
constexpr float kUpperRightCheckpointX = 180.0f;

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
constexpr float kMaximumAccelerationInput = 15.0f;
constexpr float kPixelsPerSecondSquaredPerAccelerationUnit = 42.0f;
constexpr float kVelocityDampingPerSubstep = 0.976f;
// Prevent tunneling through the 8 px walls.
constexpr float kMaximumBallSpeed = 210.0f;
constexpr float kRestSpeed = 0.025f;
constexpr float kRestAcceleration = 0.025f;
constexpr uint8_t kProgressPipCount = 6;
constexpr uint8_t kRoundPipCount = 4;

constexpr LauncherVisual kLauncherVisual{
    kBackground, kWall, kMuted, kGoal, kIconMaze,
};

bool circle_overlaps(float center_x, float center_y, const WallRect &wall)
{
    const float closest_x = clamp_float(center_x, wall.left, wall.right);
    const float closest_y = clamp_float(center_y, wall.top, wall.bottom);
    const float horizontal_distance = center_x - closest_x;
    const float vertical_distance = center_y - closest_y;
    return horizontal_distance * horizontal_distance +
               vertical_distance * vertical_distance <
           kBallRadius * kBallRadius;
}

}

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

    frames_.drain();
    velocity_x_ = 0.0f;
    velocity_y_ = 0.0f;
    portENTER_CRITICAL(&motion_mux_);
    motion_.available = false;
    portEXIT_CRITICAL(&motion_mux_);
    portENTER_CRITICAL(&telemetry_mux_);
    latest_system_telemetry_ = {};
    portEXIT_CRITICAL(&telemetry_mux_);
    return ESP_OK;
}

void TiltMazeApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.available = false;
    portEXIT_CRITICAL(&motion_mux_);
}

void TiltMazeApp::on_plus_press()
{
    reset_requested_.store(true, std::memory_order_release);
}

void TiltMazeApp::on_touch_begin(const TouchEvent &)
{
    reset_requested_.store(true, std::memory_order_release);
}

bool TiltMazeApp::on_motion(const MotionTick &tick)
{
    const bool physical_sample_valid =
        tick.fresh && std::isfinite(tick.dt) && tick.dt > 0.0f &&
        finite_vec(tick.accel_mps2) && finite_vec(tick.gyro_rads);
    bool physical_sample_accepted = false;
    Vec3 filtered_acceleration{};
    if (physical_sample_valid) {
        filtered_acceleration = motion_filter_.update(tick.accel_mps2, tick.dt);
        physical_sample_accepted = motion_filter_.last_sample_accepted() &&
                                   finite_vec(filtered_acceleration);
    }

    const bool override_valid =
        tick.override_active && finite_vec(tick.apparent_accel);
    if (!physical_sample_accepted && !override_valid) {
        portENTER_CRITICAL(&motion_mux_);
        motion_.available = false;
        portEXIT_CRITICAL(&motion_mux_);
        return false;
    }

    const Vec3 apparent_acceleration =
        override_valid ? tick.apparent_accel : filtered_acceleration;
    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent_acceleration = apparent_acceleration;
    motion_.available = true;
    if (physical_sample_accepted) {
        motion_.raw_acceleration = tick.accel_mps2;
    }
    portEXIT_CRITICAL(&motion_mux_);
    return physical_sample_accepted;
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

void TiltMazeApp::move_x(float displacement)
{
    if (displacement == 0.0f) {
        return;
    }
    float candidate_position = ball_x_ + displacement;
    const float minimum_position = kFloorLeft + kBallRadius;
    const float maximum_position = kFloorRight - kBallRadius;
    if (candidate_position < minimum_position) {
        candidate_position = minimum_position;
        velocity_x_ = 0.0f;
    } else if (candidate_position > maximum_position) {
        candidate_position = maximum_position;
        velocity_x_ = 0.0f;
    }

    for (const WallRect &wall : kMazeWalls) {
        if (!circle_overlaps(candidate_position, ball_y_, wall)) {
            continue;
        }
        candidate_position = displacement > 0.0f ? wall.left - kBallRadius
                                                 : wall.right + kBallRadius;
        velocity_x_ = 0.0f;
    }
    ball_x_ =
        clamp_float(candidate_position, minimum_position, maximum_position);
}

void TiltMazeApp::move_y(float displacement)
{
    if (displacement == 0.0f) {
        return;
    }
    float candidate_position = ball_y_ + displacement;
    const float minimum_position = kFloorTop + kBallRadius;
    const float maximum_position = kFloorBottom - kBallRadius;
    if (candidate_position < minimum_position) {
        candidate_position = minimum_position;
        velocity_y_ = 0.0f;
    } else if (candidate_position > maximum_position) {
        candidate_position = maximum_position;
        velocity_y_ = 0.0f;
    }

    for (const WallRect &wall : kMazeWalls) {
        if (!circle_overlaps(ball_x_, candidate_position, wall)) {
            continue;
        }
        candidate_position = displacement > 0.0f ? wall.top - kBallRadius
                                                 : wall.bottom + kBallRadius;
        velocity_y_ = 0.0f;
    }
    ball_y_ =
        clamp_float(candidate_position, minimum_position, maximum_position);
}

void TiltMazeApp::update_progress_and_win()
{
    if (progress_ == 0u && ball_x_ >= kLowerRightCheckpointX &&
        ball_y_ >= kLowerRightCheckpointY) {
        progress_ = 1;
    }
    if (progress_ == 1u && ball_x_ >= kMiddleRightCheckpointX &&
        ball_y_ <= kMiddleCheckpointY) {
        progress_ = 2;
    }
    if (progress_ == 2u && ball_x_ <= kMiddleLeftCheckpointX &&
        ball_y_ <= kMiddleCheckpointY) {
        progress_ = 3;
    }
    if (progress_ == 3u && ball_x_ <= kUpperLeftCheckpointX &&
        ball_y_ <= kUpperCheckpointY) {
        progress_ = 4;
    }
    if (progress_ == 4u && ball_x_ >= kUpperRightCheckpointX &&
        ball_y_ <= kUpperCheckpointY) {
        progress_ = 5;
    }

    const float horizontal_goal_distance = ball_x_ - kGoalX;
    const float vertical_goal_distance = ball_y_ - kGoalY;
    if (!solved_ && horizontal_goal_distance * horizontal_goal_distance +
                            vertical_goal_distance * vertical_goal_distance <=
                        kGoalCaptureRadius * kGoalCaptureRadius) {
        solved_ = true;
        progress_ = kProgressPipCount;
        velocity_x_ = 0.0f;
        velocity_y_ = 0.0f;
        ++completed_rounds_;
        ESP_LOGI(kTag, "maze solved epoch=%" PRIu32 " target_bytes=%" PRIu32,
                 epoch_, frozen_internal_largest_free_block_);
    }
}

void TiltMazeApp::step_substep(float screen_acceleration_x,
                               float screen_acceleration_y)
{
    if (solved_) {
        return;
    }

    const float acceleration_x =
        clamp_float(screen_acceleration_x, -kMaximumAccelerationInput,
                    kMaximumAccelerationInput) *
        kPixelsPerSecondSquaredPerAccelerationUnit;
    const float acceleration_y =
        clamp_float(screen_acceleration_y, -kMaximumAccelerationInput,
                    kMaximumAccelerationInput) *
        kPixelsPerSecondSquaredPerAccelerationUnit;
    velocity_x_ = (velocity_x_ + acceleration_x * kSubstepDt) *
                  kVelocityDampingPerSubstep;
    velocity_y_ = (velocity_y_ + acceleration_y * kSubstepDt) *
                  kVelocityDampingPerSubstep;

    const float speed =
        std::sqrt(velocity_x_ * velocity_x_ + velocity_y_ * velocity_y_);
    if (speed > kMaximumBallSpeed) {
        const float speed_scale = kMaximumBallSpeed / speed;
        velocity_x_ *= speed_scale;
        velocity_y_ *= speed_scale;
    }
    if (std::fabs(velocity_x_) < kRestSpeed &&
        std::fabs(acceleration_x) < kRestAcceleration) {
        velocity_x_ = 0.0f;
    }
    if (std::fabs(velocity_y_) < kRestSpeed &&
        std::fabs(acceleration_y) < kRestAcceleration) {
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

    const int64_t update_start_us = esp_timer_get_time();
    const bool should_reset =
        reset_requested_.exchange(false, std::memory_order_acq_rel);
    if (should_reset) {
        reset_maze(true);
    } else {
        Vec3 apparent_acceleration{};
        portENTER_CRITICAL(&motion_mux_);
        if (motion_.available) {
            apparent_acceleration = motion_.apparent_acceleration;
        }
        portEXIT_CRITICAL(&motion_mux_);

        for (int substep_index = 0; substep_index < kSubstepsPerUpdate;
             ++substep_index) {
            step_substep(apparent_acceleration.x, -apparent_acceleration.y);
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
    physics_us_.store(
        static_cast<uint32_t>(esp_timer_get_time() - update_start_us),
        std::memory_order_relaxed);
    return ESP_OK;
}

void TiltMazeApp::fill_snapshot(MazeFrame &snapshot)
{
    snapshot.ball_x = ball_x_;
    snapshot.ball_y = ball_y_;
    snapshot.progress = progress_;
    snapshot.round_pips = static_cast<uint8_t>(
        completed_rounds_ < kRoundPipCount ? completed_rounds_
                                           : kRoundPipCount);
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
    result.nonfinite_resets = nonfinite_resets_.load(std::memory_order_relaxed);
    result.physics_us = physics_us_.load(std::memory_order_relaxed);
    portENTER_CRITICAL(&motion_mux_);
    result.raw[0] = motion_.raw_acceleration.x;
    result.raw[1] = motion_.raw_acceleration.y;
    result.raw[2] = motion_.raw_acceleration.z;
    result.apparent[0] = motion_.apparent_acceleration.x;
    result.apparent[1] = motion_.apparent_acceleration.y;
    result.apparent[2] = motion_.apparent_acceleration.z;
    portEXIT_CRITICAL(&motion_mux_);
    result.raster_us = raster_us_;
    result.frame_us = frame_us_;
    return result;
}

void TiltMazeApp::draw_ring(uint16_t *pixels, int width, int stripe_y,
                            int stripe_rows, int center_x, int center_y,
                            int outer_radius, int inner_radius,
                            uint16_t outer_color, uint16_t inner_color)
{
    fill_disc(pixels, width, stripe_y, stripe_rows, center_x, center_y,
              outer_radius, outer_color);
    fill_disc(pixels, width, stripe_y, stripe_rows, center_x, center_y,
              inner_radius, inner_color);
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
                                 std::size_t index, uint16_t *pixels, int width,
                                 int stripe_y, int stripe_rows)
{
    if (index >= kTaskNodeY.size()) {
        return;
    }

    int node_x = 120;
    if (task.available && task.core_id == 0) {
        node_x = 12;
    } else if (task.available && task.core_id == 1) {
        node_x = 228;
    }
    const int node_y = kTaskNodeY[index];
    const SystemTaskState state =
        task.available ? task.state : SystemTaskState::Unknown;
    const uint16_t state_color = task_state_color(state);
    const uint32_t bounded_stack_words = task.stack_high_water_words < 2048u
                                             ? task.stack_high_water_words
                                             : 2048u;
    const int fill_radius =
        task.available
            ? 1 + static_cast<int>((bounded_stack_words * 4u) / 2048u)
            : 1;

    fill_disc(pixels, width, stripe_y, stripe_rows, node_x, node_y, 7, kWall);
    fill_disc(pixels, width, stripe_y, stripe_rows, node_x, node_y, fill_radius,
              state_color);

    const SystemTaskKind task_kind =
        task.available ? task.kind : kTaskKinds[index];
    switch (task_kind) {
    case SystemTaskKind::Coordinator:
        fill_disc(pixels, width, stripe_y, stripe_rows, node_x, node_y, 1,
                  kFloor);
        break;
    case SystemTaskKind::Sensor:
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x - 2, node_y,
                  node_x + 3, node_y + 1, kFloor);
        break;
    case SystemTaskKind::Update:
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x, node_y - 2,
                  node_x + 1, node_y + 3, kFloor);
        break;
    case SystemTaskKind::Render:
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x - 2, node_y - 2,
                  node_x - 1, node_y - 1, kFloor);
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x + 1, node_y - 2,
                  node_x + 2, node_y - 1, kFloor);
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x - 2, node_y + 1,
                  node_x - 1, node_y + 2, kFloor);
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x + 1, node_y + 1,
                  node_x + 2, node_y + 2, kFloor);
        break;
    case SystemTaskKind::Console:
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x - 2, node_y - 2,
                  node_x + 3, node_y + 3, kFloor);
        fill_rect(pixels, width, stripe_y, stripe_rows, node_x - 1, node_y - 1,
                  node_x + 2, node_y + 2, state_color);
        break;
    }
}

void TiltMazeApp::draw_memory_gauge(uint16_t *pixels, int width, int stripe_y,
                                    int stripe_rows, int gauge_left,
                                    uint32_t free_bytes,
                                    uint32_t bytes_per_segment, uint16_t color,
                                    bool data_available)
{
    constexpr int kSegmentCount = 8;
    constexpr int kCellStride = 11;
    constexpr int kFillWidth = 8;

    for (int segment = 0; segment < kSegmentCount; ++segment) {
        const int cell_left = gauge_left + segment * kCellStride;
        fill_rect(pixels, width, stripe_y, stripe_rows, cell_left, 231,
                  cell_left + 10, 239, kWall);
        fill_rect(pixels, width, stripe_y, stripe_rows, cell_left + 1, 233,
                  cell_left + 9, 237, kBackground);
        if (!data_available || bytes_per_segment == 0u) {
            continue;
        }

        const uint64_t segment_start_bytes =
            static_cast<uint64_t>(segment) * bytes_per_segment;
        if (free_bytes <= segment_start_bytes) {
            continue;
        }
        const uint64_t remaining_bytes =
            static_cast<uint64_t>(free_bytes) - segment_start_bytes;
        const uint64_t segment_bytes = remaining_bytes < bytes_per_segment
                                           ? remaining_bytes
                                           : bytes_per_segment;
        const int fill_width = static_cast<int>(
            (segment_bytes * kFillWidth + bytes_per_segment - 1u) /
            bytes_per_segment);
        fill_rect(pixels, width, stripe_y, stripe_rows, cell_left + 1, 233,
                  cell_left + 1 + fill_width, 237, color);
    }
}

void TiltMazeApp::draw_goal(const MazeFrame &maze, uint16_t *pixels, int width,
                            int stripe_y, int stripe_rows)
{
    const bool has_frozen_memory_sample =
        maze.frozen_telemetry_generation != 0u &&
        maze.frozen_internal_free_bytes != 0u;
    if (!has_frozen_memory_sample) {
        draw_ring(pixels, width, stripe_y, stripe_rows, 189, 59, 14, 11, kMuted,
                  kFloor);
        for (std::size_t index = 0; index < kOctantX.size(); ++index) {
            fill_disc(pixels, width, stripe_y, stripe_rows,
                      189 + (kOctantX[index] * 14) / 8,
                      59 + (kOctantY[index] * 14) / 8, 1, kMuted);
        }
        fill_disc(pixels, width, stripe_y, stripe_rows, 189, 59, 2, kMuted);
        return;
    }

    const uint32_t largest_free_block =
        maze.frozen_internal_largest_free_block <
                maze.frozen_internal_free_bytes
            ? maze.frozen_internal_largest_free_block
            : maze.frozen_internal_free_bytes;
    const uint64_t total_free_bytes = maze.frozen_internal_free_bytes;
    int outer_radius =
        12 + static_cast<int>((static_cast<uint64_t>(largest_free_block) * 4u +
                               total_free_bytes / 2u) /
                              total_free_bytes);
    outer_radius = std::clamp(outer_radius, 12, 16);

    int lit_segment_count =
        static_cast<int>((static_cast<uint64_t>(largest_free_block) * 8u +
                          total_free_bytes / 2u) /
                         total_free_bytes);
    if (largest_free_block != 0u) {
        lit_segment_count = std::max(1, lit_segment_count);
    }
    lit_segment_count = std::min(8, lit_segment_count);

    draw_ring(pixels, width, stripe_y, stripe_rows, 189, 59, outer_radius,
              outer_radius - 3, kGoal, kFloor);
    for (std::size_t index = 0; index < kOctantX.size(); ++index) {
        const uint16_t segment_color =
            static_cast<int>(index) < lit_segment_count ? kGoal : kMuted;
        fill_disc(pixels, width, stripe_y, stripe_rows,
                  189 + (kOctantX[index] * outer_radius) / 8,
                  59 + (kOctantY[index] * outer_radius) / 8, 2, segment_color);
    }
    draw_ring(pixels, width, stripe_y, stripe_rows, 189, 59, 7, 4, kGoal,
              kFloor);
    fill_disc(pixels, width, stripe_y, stripe_rows, 189, 59, 2, kGoal);
}

void TiltMazeApp::raster_stripe(const MazeFrame &maze, uint16_t *pixels,
                                int width, int stripe_y, int stripe_rows)
{
    for (int local_y = 0; local_y < stripe_rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        for (int screen_x = 0; screen_x < width; ++screen_x) {
            row[screen_x] = kBackground;
        }
    }

    for (uint8_t progress_index = 0; progress_index < kProgressPipCount;
         ++progress_index) {
        const int marker_x = 85 + static_cast<int>(progress_index) * 14;
        fill_disc(pixels, width, stripe_y, stripe_rows, marker_x, 14, 4, kWall);
        const uint16_t marker_fill =
            progress_index < maze.progress
                ? kBall
                : (progress_index == maze.progress && !maze.solved ? kGoal
                                                                   : kMuted);
        fill_disc(pixels, width, stripe_y, stripe_rows, marker_x, 14, 2,
                  marker_fill);
    }
    for (uint8_t round_index = 0; round_index < kRoundPipCount; ++round_index) {
        const int marker_x = 196 + static_cast<int>(round_index) * 10;
        fill_disc(pixels, width, stripe_y, stripe_rows, marker_x, 14, 3, kWall);
        if (round_index < maze.round_pips) {
            fill_disc(pixels, width, stripe_y, stripe_rows, marker_x, 14, 2,
                      kWin);
        }
    }

    fill_rect(pixels, width, stripe_y, stripe_rows, 20, 28, 220, 228, kWall);
    fill_rect(pixels, width, stripe_y, stripe_rows, 28, 36, 212, 220, kFloor);
    for (const WallRect &wall : kMazeWalls) {
        const int left = static_cast<int>(wall.left);
        const int top = static_cast<int>(wall.top);
        const int right = static_cast<int>(wall.right);
        const int bottom = static_cast<int>(wall.bottom);
        fill_rect(pixels, width, stripe_y, stripe_rows, left, top, right,
                  bottom, kWall);
        fill_rect(pixels, width, stripe_y, stripe_rows, left, top, right,
                  top + 2, kMuted);
    }

    const bool live_memory_available = maze.system.generation != 0u;
    fill_rect(pixels, width, stripe_y, stripe_rows, 11, 232, 18, 239,
              live_memory_available ? kInternalRam : kMuted);
    draw_memory_gauge(pixels, width, stripe_y, stripe_rows, 22,
                      maze.system.internal_free_bytes,
                      kInternalGaugeSegmentBytes, kInternalRam,
                      live_memory_available);
    draw_ring(pixels, width, stripe_y, stripe_rows, 122, 235, 4, 2,
              live_memory_available ? kPsram : kMuted, kBackground);
    draw_memory_gauge(pixels, width, stripe_y, stripe_rows, 130,
                      maze.system.psram_free_bytes, kPsramGaugeSegmentBytes,
                      kPsram, live_memory_available);

    for (std::size_t index = 0; index < kTaskKinds.size(); ++index) {
        draw_task_node(maze.system.tasks[index], index, pixels, width, stripe_y,
                       stripe_rows);
    }

    draw_ring(pixels, width, stripe_y, stripe_rows, 51, 197, 11, 8, kMuted,
              kFloor);
    fill_disc(pixels, width, stripe_y, stripe_rows, 51, 197, 2, kMuted);
    fill_rect(pixels, width, stripe_y, stripe_rows, 49, 183, 53, 188, kMuted);
    fill_rect(pixels, width, stripe_y, stripe_rows, 49, 206, 53, 211, kMuted);
    fill_rect(pixels, width, stripe_y, stripe_rows, 37, 195, 42, 199, kMuted);
    fill_rect(pixels, width, stripe_y, stripe_rows, 60, 195, 65, 199, kMuted);

    draw_goal(maze, pixels, width, stripe_y, stripe_rows);

    const int ball_x = static_cast<int>(maze.ball_x + 0.5f);
    const int ball_y = static_cast<int>(maze.ball_y + 0.5f);
    fill_disc(pixels, width, stripe_y, stripe_rows, ball_x + 1, ball_y + 2, 8,
              kWall);
    fill_disc(pixels, width, stripe_y, stripe_rows, ball_x, ball_y, 7, kBall);
    fill_disc(pixels, width, stripe_y, stripe_rows, ball_x - 2, ball_y - 2, 2,
              kWin);

    if (maze.solved) {
        fill_rect(pixels, width, stripe_y, stripe_rows, 20, 28, 220, 34, kWin);
        fill_rect(pixels, width, stripe_y, stripe_rows, 20, 222, 220, 228,
                  kWin);
        fill_rect(pixels, width, stripe_y, stripe_rows, 20, 28, 26, 228, kWin);
        fill_rect(pixels, width, stripe_y, stripe_rows, 214, 28, 220, 228,
                  kWin);
        fill_disc(pixels, width, stripe_y, stripe_rows, 120, 128, 48, kWin);
        fill_disc(pixels, width, stripe_y, stripe_rows, 120, 128, 39,
                  kBackground);
        fill_segment(pixels, width, stripe_y, stripe_rows, 96, 128, 113, 145, 5,
                     kWin);
        fill_segment(pixels, width, stripe_y, stripe_rows, 113, 145, 147, 108,
                     5, kWin);
    }
}

bool TiltMazeApp::render(DisplayFrame &frame)
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

    const MazeFrame *snapshot = frames_.acquire_latest();
    if (snapshot == nullptr) {
        return false;
    }

    const int64_t frame_start_us = esp_timer_get_time();
    uint32_t total_raster_us = 0;
    esp_err_t error = frame.ops.wait_previous(frame.transport);
    if (error == ESP_OK) {
        frame.ops.latch_capture(frame.transport);
        for (int stripe_index = 0; stripe_index < frame.stripe_count;
             ++stripe_index) {
            const int stripe_y = stripe_index * frame.stripe_rows;
            const int stripe_rows =
                std::min(frame.stripe_rows, frame.height - stripe_y);
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
            error = frame.ops.submit(frame.transport, stripe_index, stripe_y,
                                     stripe_rows, stripe_pixels);
            if (error != ESP_OK) {
                break;
            }
        }
        if (error == ESP_OK) {
            error = frame.ops.finish(frame.transport);
        }
    }

    frames_.release(snapshot);
    if (error != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s", esp_err_to_name(error));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start_us);
    raster_us_ = total_raster_us + frame.ops.capture_copy_us(frame.transport);
    return true;
}

}
