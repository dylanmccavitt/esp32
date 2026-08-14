#include "ragdoll_avalanche_app.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "common_math.hpp"
#include "draw.hpp"
#include "launcher_icons.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "ragdoll";

constexpr uint16_t kBackground = 0x2D4B;
constexpr uint16_t kSnowDot = 0x5ADB;
constexpr uint16_t kBody = 0xB5B6;
constexpr uint16_t kHead = 0xFFDE;
constexpr uint16_t kSpikeShaft = 0x9CD3;
constexpr uint16_t kSpikeTip = 0xFFDE;
constexpr uint16_t kScoreColor = 0xFE8E;
constexpr uint16_t kBestColor = 0x5CF3;
constexpr uint16_t kGameOverBg = 0xD546;
constexpr uint16_t kSeparator = 0x5AEB;
constexpr uint16_t kMuted = 0x7BEF;
constexpr uint8_t kDigitBitmap[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, {0b010, 0b110, 0b010, 0b010, 0b111},
    {0b111, 0b001, 0b111, 0b100, 0b111}, {0b111, 0b001, 0b111, 0b001, 0b111},
    {0b101, 0b101, 0b111, 0b001, 0b001}, {0b111, 0b100, 0b111, 0b001, 0b111},
    {0b111, 0b100, 0b111, 0b101, 0b111}, {0b111, 0b001, 0b001, 0b001, 0b001},
    {0b111, 0b101, 0b111, 0b101, 0b111}, {0b111, 0b101, 0b111, 0b001, 0b111},
};

constexpr LauncherVisual kLauncherVisual{
    kBackground, kBody, 0x6B4D, kScoreColor, kIconAvalanche,
};

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTau = 2.0f * kPi;

inline float wrap_angle(float angle)
{
    if (!std::isfinite(angle)) {
        return angle;
    }
    while (angle > kPi) {
        angle -= kTau;
    }
    while (angle < -kPi) {
        angle += kTau;
    }
    return angle;
}

inline float random_unit_float()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

constexpr float kSubstepDt = 1.0f / 60.0f;
constexpr int kSubstepsPerUpdate = 2;

void draw_snow_specks(uint16_t *pixels, int width, int stripe_y,
                      int stripe_rows)
{
    for (int speck_index = 0; speck_index < 56; ++speck_index) {
        const int speck_x = (speck_index * 97 + 53) % 240;
        const int speck_y = (speck_index * 149 + 31) % 224;
        if (speck_y < stripe_y || speck_y >= stripe_y + stripe_rows) {
            continue;
        }
        pixels[(speck_y - stripe_y) * width + speck_x] = kSnowDot;
    }
}

}

RagdollAvalancheApp s_ragdoll_avalanche_app;

const LauncherVisual *RagdollAvalancheApp::launcher_visual() const
{
    return &kLauncherVisual;
}

esp_err_t RagdollAvalancheApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;
    }

    if (!nvs_initialized_) {
        esp_err_t nvs_result = nvs_flash_init();
        if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
            nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(kTag, "NVS needs erase, retrying");
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
            nvs_result = nvs_flash_init();
        }
        if (nvs_result != ESP_OK) {
            ESP_LOGE(kTag, "NVS init failed: %s", esp_err_to_name(nvs_result));
        }
        nvs_initialized_ = true;
    }

    std::srand(static_cast<unsigned>(esp_random()));

    load_high_scores();

    reset_game();
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    setup_done_ = true;
    return ESP_OK;
}

esp_err_t RagdollAvalancheApp::enter()
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    frames_.drain();
    portENTER_CRITICAL(&motion_mux_);
    motion_.available = false;
    portEXIT_CRITICAL(&motion_mux_);
    return ESP_OK;
}

void RagdollAvalancheApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.available = false;
    portEXIT_CRITICAL(&motion_mux_);
}

bool RagdollAvalancheApp::on_motion(const MotionTick &tick)
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

void RagdollAvalancheApp::on_touch_begin(const TouchEvent &)
{
    if (game_over_.load(std::memory_order_acquire)) {
        reset_requested_.store(true, std::memory_order_release);
    }
}

void RagdollAvalancheApp::on_plus_press()
{
    reset_requested_.store(true, std::memory_order_release);
}

void RagdollAvalancheApp::spawn_wave()
{
    int target_spike_count =
        kWaveInitialCount +
        static_cast<int>(survival_time_ / kWaveCountRampSeconds);
    if (target_spike_count > kWaveMaxCount) {
        target_spike_count = kWaveMaxCount;
    }

    const bool reserve_escape_lane = first_wave_pending_;
    first_wave_pending_ = false;

    float placed_positions_x[kWaveMaxCount] = {};
    int placed_count = 0;
    constexpr float kSpawnMargin = 24.0f;
    constexpr int kPlacementRetries = 12;
    constexpr float kPlacementJitter = 100.0f;
    const float panel_width = static_cast<float>(kPanelWidth);

    for (int wave_index = 0; wave_index < target_spike_count; ++wave_index) {
        int free_slot_index = -1;
        for (int pool_index = 0; pool_index < kMaxSpikes; ++pool_index) {
            if (!spikes_[pool_index].active) {
                free_slot_index = pool_index;
                break;
            }
        }
        if (free_slot_index < 0) {
            return;
        }

        float spawn_x = kSpawnMargin + random_unit_float() *
                                           (panel_width - 2.0f * kSpawnMargin);
        bool position_available = false;
        for (int placement_attempt = 0; placement_attempt < kPlacementRetries;
             ++placement_attempt) {
            position_available =
                !reserve_escape_lane ||
                std::fabs(spawn_x - player_x_) >= kFirstWaveClearance;
            for (int placed_index = 0;
                 position_available && placed_index < placed_count;
                 ++placed_index) {
                if (std::fabs(spawn_x - placed_positions_x[placed_index]) <
                    kMinimumWaveSpacing) {
                    position_available = false;
                }
            }
            if (position_available) {
                break;
            }
            spawn_x = clamp_float(spawn_x + (random_unit_float() - 0.5f) *
                                                kPlacementJitter,
                                  kSpawnMargin, panel_width - kSpawnMargin);
        }
        if (!position_available) {
            continue;
        }

        SpikeState &spike = spikes_[free_slot_index];
        spike.active = true;
        spike.x = spawn_x;
        spike.y = kSpikeSpawnY;
        const float base_speed =
            kInitialSpikeSpeed + survival_time_ * kSpikeSpeedRampPerSecond;
        const float varied_speed =
            base_speed + random_unit_float() * kSpikeSpeedVariation;
        spike.velocity_y = varied_speed > kMaximumSpikeSpeed
                               ? kMaximumSpikeSpeed
                               : varied_speed;
        placed_positions_x[placed_count++] = spawn_x;
        active_count_.fetch_add(1, std::memory_order_relaxed);
    }
}

void RagdollAvalancheApp::kill_player()
{
    if (game_over_.load(std::memory_order_relaxed)) {
        return;
    }
    if (std::fabs(player_velocity_x_) > 4.0f) {
        death_flop_direction_ = player_velocity_x_ < 0.0f ? -1.0f : 1.0f;
    } else {
        death_flop_direction_ = ragdoll_pose_.body_angle < 0.0f ? -1.0f : 1.0f;
    }
    body_angular_velocity_ += death_flop_direction_ * 2.4f;
    player_velocity_x_ = 0.0f;
    player_velocity_y_ = 0.0f;
    game_over_.store(true, std::memory_order_relaxed);
    save_high_score(score_);
    ESP_LOGI(kTag, "game over score=%" PRId32 " best=%" PRId32, score_,
             best_score_);
}

void RagdollAvalancheApp::reset_game()
{
    ++epoch_;
    if (epoch_ == 0u) {
        epoch_ = 1u;
    }
    for (SpikeState &spike : spikes_) {
        spike.active = false;
    }
    active_count_.store(0, std::memory_order_relaxed);
    player_x_ = 120.0f;
    player_y_ = 120.0f;
    player_velocity_x_ = 0.0f;
    player_velocity_y_ = 0.0f;
    survival_time_ = 0.0f;
    wave_timer_ = 0.0f;
    wave_interval_ = kWaveIntervalInitial;
    first_wave_pending_ = true;
    score_ = 0;
    reset_ragdoll_pose();
    game_over_.store(false, std::memory_order_relaxed);
    ESP_LOGI(kTag, "game reset epoch=%" PRIu32, epoch_);
}

void RagdollAvalancheApp::reset_ragdoll_pose()
{
    ragdoll_pose_.body_angle = 0.0f;
    ragdoll_pose_.joint_angles[LeftUpperArm] = -0.78f;
    ragdoll_pose_.joint_angles[LeftForearm] = -1.08f;
    ragdoll_pose_.joint_angles[RightUpperArm] = 0.88f;
    ragdoll_pose_.joint_angles[RightForearm] = 1.18f;
    ragdoll_pose_.joint_angles[LeftThigh] = -0.18f;
    ragdoll_pose_.joint_angles[LeftShin] = 0.05f;
    ragdoll_pose_.joint_angles[RightThigh] = 0.30f;
    ragdoll_pose_.joint_angles[RightShin] = 0.08f;
    joint_velocities_.fill(0.0f);
    body_angular_velocity_ = 0.0f;
    death_flop_direction_ = 1.0f;
}

RagdollAvalancheApp::RagdollGeometry
RagdollAvalancheApp::calculate_ragdoll_geometry(float center_x, float center_y,
                                                const RagdollPose &pose)
{
    RagdollGeometry geometry{};
    const float body_x = std::sin(pose.body_angle);
    const float body_y = std::cos(pose.body_angle);
    const float perpendicular_x = std::cos(pose.body_angle);
    const float perpendicular_y = -std::sin(pose.body_angle);

    geometry.shoulder = {
        center_x - body_x * kTorsoHalfLength,
        center_y - body_y * kTorsoHalfLength,
    };
    geometry.hip = {
        center_x + body_x * kTorsoHalfLength,
        center_y + body_y * kTorsoHalfLength,
    };
    const float head_offset = kHeadRadius + 3.0f;
    geometry.head = {
        geometry.shoulder.x - body_x * head_offset,
        geometry.shoulder.y - body_y * head_offset,
    };
    geometry.upper_chest = {
        geometry.shoulder.x + body_x * (kTorsoHalfLength * 0.70f),
        geometry.shoulder.y + body_y * (kTorsoHalfLength * 0.70f),
    };

    constexpr float kAnchorOffset = 3.0f;
    auto make_limb = [&](RagdollPoint anchor, JointIndex upper_joint,
                         JointIndex lower_joint, float upper_length,
                         float lower_length) {
        LimbGeometry limb{};
        limb.anchor = anchor;
        const float upper_angle =
            pose.joint_angles[static_cast<std::size_t>(upper_joint)];
        const float lower_angle =
            pose.joint_angles[static_cast<std::size_t>(lower_joint)];
        limb.joint = {
            anchor.x + std::sin(upper_angle) * upper_length,
            anchor.y + std::cos(upper_angle) * upper_length,
        };
        limb.end = {
            limb.joint.x + std::sin(lower_angle) * lower_length,
            limb.joint.y + std::cos(lower_angle) * lower_length,
        };
        return limb;
    };

    const RagdollPoint left_shoulder{
        geometry.shoulder.x - perpendicular_x * kAnchorOffset,
        geometry.shoulder.y - perpendicular_y * kAnchorOffset,
    };
    const RagdollPoint right_shoulder{
        geometry.shoulder.x + perpendicular_x * kAnchorOffset,
        geometry.shoulder.y + perpendicular_y * kAnchorOffset,
    };
    const RagdollPoint left_hip{
        geometry.hip.x - perpendicular_x * kAnchorOffset,
        geometry.hip.y - perpendicular_y * kAnchorOffset,
    };
    const RagdollPoint right_hip{
        geometry.hip.x + perpendicular_x * kAnchorOffset,
        geometry.hip.y + perpendicular_y * kAnchorOffset,
    };
    geometry.limbs = {
        make_limb(left_shoulder, LeftUpperArm, LeftForearm, kUpperArmLength,
                  kForearmLength),
        make_limb(right_shoulder, RightUpperArm, RightForearm, kUpperArmLength,
                  kForearmLength),
        make_limb(left_hip, LeftThigh, LeftShin, kThighLength, kShinLength),
        make_limb(right_hip, RightThigh, RightShin, kThighLength, kShinLength),
    };

    geometry.min_x = geometry.head.x - kHeadRadius;
    geometry.max_x = geometry.head.x + kHeadRadius;
    geometry.min_y = geometry.head.y - kHeadRadius;
    geometry.max_y = geometry.head.y + kHeadRadius;
    auto include_point = [&](const RagdollPoint &point, float radius) {
        geometry.min_x = geometry.min_x < point.x - radius ? geometry.min_x
                                                           : point.x - radius;
        geometry.max_x = geometry.max_x > point.x + radius ? geometry.max_x
                                                           : point.x + radius;
        geometry.min_y = geometry.min_y < point.y - radius ? geometry.min_y
                                                           : point.y - radius;
        geometry.max_y = geometry.max_y > point.y + radius ? geometry.max_y
                                                           : point.y + radius;
    };
    include_point(geometry.shoulder, static_cast<float>(kTorsoRadius));
    include_point(geometry.hip, static_cast<float>(kTorsoRadius));
    for (const LimbGeometry &limb : geometry.limbs) {
        include_point(limb.anchor, static_cast<float>(kLimbRadius));
        include_point(limb.joint, static_cast<float>(kLimbRadius));
        include_point(limb.end, static_cast<float>(kLimbRadius));
    }
    return geometry;
}

void RagdollAvalancheApp::step_ragdoll_pose(float acceleration_x,
                                            float acceleration_y, bool is_dead)
{
    const float acceleration_scale =
        kPixelsPerSecondSquaredPerAccelerationUnit * 6.0f;
    const float normalized_acceleration_x =
        clamp_float(acceleration_x / acceleration_scale, -1.5f, 1.5f);
    const float normalized_acceleration_y =
        clamp_float(acceleration_y / acceleration_scale, -1.5f, 1.5f);
    const float normalized_velocity_x =
        clamp_float(player_velocity_x_ / kMaximumPlayerSpeed, -1.0f, 1.0f);
    const float normalized_velocity_y =
        clamp_float(player_velocity_y_ / kMaximumPlayerSpeed, -1.0f, 1.0f);

    const float hanging_force_x =
        -0.90f * normalized_acceleration_x - 0.35f * normalized_velocity_x;
    const float hanging_force_y = 0.55f - 0.85f * normalized_acceleration_y -
                                  0.20f * normalized_velocity_y;
    const float hanging_angle = std::atan2(hanging_force_x, hanging_force_y);

    const float target_body_angle =
        is_dead ? death_flop_direction_ * 1.45f
                : clamp_float(0.28f * normalized_acceleration_x +
                                  0.22f * normalized_velocity_x,
                              -0.62f, 0.62f);
    const float body_angle_error =
        wrap_angle(target_body_angle - ragdoll_pose_.body_angle);
    body_angular_velocity_ += body_angle_error * 18.0f * kSubstepDt;
    body_angular_velocity_ *= 0.93f;
    body_angular_velocity_ = clamp_float(body_angular_velocity_, -5.5f, 5.5f);
    ragdoll_pose_.body_angle = wrap_angle(ragdoll_pose_.body_angle +
                                          body_angular_velocity_ * kSubstepDt);

    const float normalized_speed =
        clamp_float(std::sqrt(normalized_velocity_x * normalized_velocity_x +
                              normalized_velocity_y * normalized_velocity_y),
                    0.0f, 1.0f);
    const float arm_spread = 0.82f + normalized_speed * 0.20f;
    const float arm_bend = 0.30f + normalized_speed * 0.10f;
    const float leg_spread = 0.24f + normalized_speed * 0.12f;
    const float knee_bend = 0.20f + normalized_speed * 0.08f;
    const float body_follow = ragdoll_pose_.body_angle * 0.16f;

    const float left_upper_arm_target =
        wrap_angle(hanging_angle - arm_spread + body_follow);
    const float right_upper_arm_target =
        wrap_angle(hanging_angle + arm_spread + body_follow);
    const float left_thigh_target =
        wrap_angle(hanging_angle - leg_spread + body_follow);
    const float right_thigh_target =
        wrap_angle(hanging_angle + leg_spread + body_follow);
    const std::array<float, JointCount> target_joint_angles{
        left_upper_arm_target,  wrap_angle(left_upper_arm_target - arm_bend),
        right_upper_arm_target, wrap_angle(right_upper_arm_target + arm_bend),
        left_thigh_target,      wrap_angle(left_thigh_target + knee_bend),
        right_thigh_target,     wrap_angle(right_thigh_target - knee_bend),
    };

    auto step_joint = [&](JointIndex joint, float stiffness, float damping) {
        const std::size_t index = static_cast<std::size_t>(joint);
        const float angle_error = wrap_angle(target_joint_angles[index] -
                                             ragdoll_pose_.joint_angles[index]);
        float &velocity = joint_velocities_[index];
        velocity = (velocity + angle_error * stiffness * kSubstepDt) * damping;
        velocity = clamp_float(velocity, -8.0f, 8.0f);
        ragdoll_pose_.joint_angles[index] = wrap_angle(
            ragdoll_pose_.joint_angles[index] + velocity * kSubstepDt);
    };

    step_joint(LeftUpperArm, 14.0f, 0.95f);
    step_joint(RightUpperArm, 14.0f, 0.95f);
    step_joint(LeftForearm, 10.5f, 0.962f);
    step_joint(RightForearm, 10.5f, 0.962f);
    step_joint(LeftThigh, 16.0f, 0.945f);
    step_joint(RightThigh, 16.0f, 0.945f);
    step_joint(LeftShin, 11.0f, 0.960f);
    step_joint(RightShin, 11.0f, 0.960f);

    auto constrain_bend = [&](JointIndex lower_joint, JointIndex upper_joint,
                              float min_bend, float max_bend) {
        const std::size_t lower_index = static_cast<std::size_t>(lower_joint);
        const std::size_t upper_index = static_cast<std::size_t>(upper_joint);
        const float bend = wrap_angle(ragdoll_pose_.joint_angles[lower_index] -
                                      ragdoll_pose_.joint_angles[upper_index]);
        const float constrained_bend = clamp_float(bend, min_bend, max_bend);
        if (constrained_bend != bend) {
            ragdoll_pose_.joint_angles[lower_index] = wrap_angle(
                ragdoll_pose_.joint_angles[upper_index] + constrained_bend);
            joint_velocities_[lower_index] = joint_velocities_[upper_index];
        }
    };
    constrain_bend(LeftForearm, LeftUpperArm, -0.95f, -0.08f);
    constrain_bend(RightForearm, RightUpperArm, 0.08f, 0.95f);
    constrain_bend(LeftShin, LeftThigh, 0.06f, 0.75f);
    constrain_bend(RightShin, RightThigh, -0.75f, -0.06f);
}

void RagdollAvalancheApp::load_high_scores()
{
    best_score_ = 0;
    top_scores_.fill(0);

    nvs_handle_t nvs_handle;
    const esp_err_t open_error = nvs_open("ragdoll", NVS_READONLY, &nvs_handle);
    if (open_error == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (open_error != ESP_OK) {
        ESP_LOGW(kTag, "highscore load: nvs_open failed: %s",
                 esp_err_to_name(open_error));
        return;
    }

    std::size_t blob_size = top_scores_.size() * sizeof(top_scores_[0]);
    const esp_err_t read_error =
        nvs_get_blob(nvs_handle, "top5", top_scores_.data(), &blob_size);
    nvs_close(nvs_handle);

    if (read_error != ESP_OK) {
        if (read_error != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kTag, "highscore load: nvs_get_blob failed: %s",
                     esp_err_to_name(read_error));
        }
        top_scores_.fill(0);
        return;
    }
    if (blob_size != top_scores_.size() * sizeof(top_scores_[0])) {
        ESP_LOGW(kTag, "highscore load: bad blob size %u, discarding",
                 static_cast<unsigned>(blob_size));
        top_scores_.fill(0);
        return;
    }

    bool scores_are_valid = true;
    bool found_empty_slot = false;
    for (std::size_t index = 0; index < top_scores_.size(); ++index) {
        const int32_t score = top_scores_[index];
        const bool follows_empty_slot = found_empty_slot && score != 0;
        const bool out_of_order = index > 0 && score > top_scores_[index - 1];
        if (score < 0 || follows_empty_slot || out_of_order) {
            scores_are_valid = false;
            break;
        }
        found_empty_slot = score == 0;
    }
    if (!scores_are_valid) {
        ESP_LOGW(kTag,
                 "highscore load: corrupt shape [%d %d %d %d %d], "
                 "discarding",
                 top_scores_[0], top_scores_[1], top_scores_[2], top_scores_[3],
                 top_scores_[4]);
        top_scores_.fill(0);
        return;
    }

    best_score_ = top_scores_[0];
    ESP_LOGI(kTag, "highscores loaded best=%" PRId32, best_score_);
}

void RagdollAvalancheApp::save_high_score(int32_t score)
{
    if (score <= 0 || score <= top_scores_[kHighScoreCount - 1]) {
        return;
    }

    int insertion_index = kHighScoreCount - 1;
    while (insertion_index > 0 && top_scores_[insertion_index - 1] < score) {
        top_scores_[insertion_index] = top_scores_[insertion_index - 1];
        --insertion_index;
    }
    top_scores_[insertion_index] = score;
    best_score_ = top_scores_[0];

    nvs_handle_t nvs_handle;
    esp_err_t error = nvs_open("ragdoll", NVS_READWRITE, &nvs_handle);
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(error));
        return;
    }
    error = nvs_set_blob(nvs_handle, "top5", top_scores_.data(),
                         top_scores_.size() * sizeof(top_scores_[0]));
    if (error == ESP_OK) {
        error = nvs_commit(nvs_handle);
    }
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "nvs write failed: %s", esp_err_to_name(error));
    }
    nvs_close(nvs_handle);
}

void RagdollAvalancheApp::step_substep()
{
    ++animation_tick_;
    if (animation_tick_ == 0u) {
        animation_tick_ = 1u;
    }

    const bool is_dead = game_over_.load(std::memory_order_relaxed);
    float acceleration_x = 0.0f;
    float acceleration_y = 0.0f;
    if (!is_dead) {
        Vec3 apparent_acceleration{};
        bool motion_available = false;
        portENTER_CRITICAL(&motion_mux_);
        if (motion_.available) {
            apparent_acceleration = motion_.apparent_acceleration;
            motion_available = true;
        }
        portEXIT_CRITICAL(&motion_mux_);

        if (motion_available) {
            acceleration_x =
                clamp_float(apparent_acceleration.x, -kMaximumAccelerationInput,
                            kMaximumAccelerationInput) *
                kPixelsPerSecondSquaredPerAccelerationUnit;
            acceleration_y = clamp_float(-apparent_acceleration.y,
                                         -kMaximumAccelerationInput,
                                         kMaximumAccelerationInput) *
                             kPixelsPerSecondSquaredPerAccelerationUnit;
            player_velocity_x_ =
                (player_velocity_x_ + acceleration_x * kSubstepDt) *
                kVelocityDampingPerSubstep;
            player_velocity_y_ =
                (player_velocity_y_ + acceleration_y * kSubstepDt) *
                kVelocityDampingPerSubstep;
            const float player_speed =
                std::sqrt(player_velocity_x_ * player_velocity_x_ +
                          player_velocity_y_ * player_velocity_y_);
            if (player_speed > kMaximumPlayerSpeed) {
                player_velocity_x_ *= kMaximumPlayerSpeed / player_speed;
                player_velocity_y_ *= kMaximumPlayerSpeed / player_speed;
            }
        } else {
            player_velocity_x_ *= kVelocityDampingPerSubstep;
            player_velocity_y_ *= kVelocityDampingPerSubstep;
        }

        player_x_ += player_velocity_x_ * kSubstepDt;
        player_y_ += player_velocity_y_ * kSubstepDt;

        survival_time_ += kSubstepDt;
        const float next_wave_interval =
            kWaveIntervalInitial - survival_time_ * kWaveIntervalTimeRamp;
        wave_interval_ = next_wave_interval < kMinimumWaveInterval
                             ? kMinimumWaveInterval
                             : next_wave_interval;
        wave_timer_ += kSubstepDt;
        if (wave_timer_ >= wave_interval_) {
            wave_timer_ -= wave_interval_;
            spawn_wave();
        }
    }

    step_ragdoll_pose(acceleration_x, acceleration_y, is_dead);

    // Clamp articulated bounds, not the center.
    RagdollGeometry player_geometry =
        calculate_ragdoll_geometry(player_x_, player_y_, ragdoll_pose_);
    float shift_x = 0.0f;
    float shift_y = 0.0f;
    constexpr float kPanelMaxX = static_cast<float>(kPanelWidth - 1);
    constexpr float kPanelMaxY = static_cast<float>(kPanelHeight - 1);
    if (player_geometry.min_x < 0.0f) {
        shift_x = -player_geometry.min_x;
    } else if (player_geometry.max_x > kPanelMaxX) {
        shift_x = kPanelMaxX - player_geometry.max_x;
    }
    if (player_geometry.min_y < 0.0f) {
        shift_y = -player_geometry.min_y;
    } else if (player_geometry.max_y > kPanelMaxY) {
        shift_y = kPanelMaxY - player_geometry.max_y;
    }
    if (shift_x != 0.0f || shift_y != 0.0f) {
        player_x_ += shift_x;
        player_y_ += shift_y;
        if ((shift_x > 0.0f && player_velocity_x_ < 0.0f) ||
            (shift_x < 0.0f && player_velocity_x_ > 0.0f)) {
            player_velocity_x_ = 0.0f;
        }
        if ((shift_y > 0.0f && player_velocity_y_ < 0.0f) ||
            (shift_y < 0.0f && player_velocity_y_ > 0.0f)) {
            player_velocity_y_ = 0.0f;
        }
        player_geometry =
            calculate_ragdoll_geometry(player_x_, player_y_, ragdoll_pose_);
    }

    // Use one death snapshot so spike exit scores do not depend on pool order.
    const bool dead_before_spike_update =
        game_over_.load(std::memory_order_relaxed);
    for (SpikeState &spike : spikes_) {
        if (!spike.active) {
            continue;
        }
        spike.y += spike.velocity_y * kSubstepDt;
        if (spike.y > kSpikeExitY) {
            spike.active = false;
            active_count_.fetch_sub(1, std::memory_order_relaxed);
            if (!dead_before_spike_update) {
                ++score_;
            }
        }
    }

    if (!dead_before_spike_update) {
        const float lethal_dx =
            player_geometry.upper_chest.x - player_geometry.head.x;
        const float lethal_dy =
            player_geometry.upper_chest.y - player_geometry.head.y;
        const float lethal_length_squared =
            lethal_dx * lethal_dx + lethal_dy * lethal_dy;
        const float inverse_lethal_length_squared =
            lethal_length_squared > 0.0f ? 1.0f / lethal_length_squared : 0.0f;
        const float lethal_radius_squared = kLethalRadius * kLethalRadius;
        for (const SpikeState &spike : spikes_) {
            if (!spike.active) {
                continue;
            }
            const float projection =
                clamp_float(((spike.x - player_geometry.head.x) * lethal_dx +
                             (spike.y - player_geometry.head.y) * lethal_dy) *
                                inverse_lethal_length_squared,
                            0.0f, 1.0f);
            const float nearest_x =
                player_geometry.head.x + projection * lethal_dx;
            const float nearest_y =
                player_geometry.head.y + projection * lethal_dy;
            const float distance_x = spike.x - nearest_x;
            const float distance_y = spike.y - nearest_y;
            if (distance_x * distance_x + distance_y * distance_y <=
                lethal_radius_squared) {
                kill_player();
                break;
            }
        }
    }

    bool has_nonfinite_state =
        !std::isfinite(player_x_) || !std::isfinite(player_y_) ||
        !std::isfinite(player_velocity_x_) ||
        !std::isfinite(player_velocity_y_) || !std::isfinite(survival_time_) ||
        !std::isfinite(wave_timer_) || !std::isfinite(wave_interval_) ||
        !std::isfinite(ragdoll_pose_.body_angle) ||
        !std::isfinite(body_angular_velocity_);
    for (std::size_t index = 0; index < JointCount && !has_nonfinite_state;
         ++index) {
        has_nonfinite_state =
            !std::isfinite(ragdoll_pose_.joint_angles[index]) ||
            !std::isfinite(joint_velocities_[index]);
    }
    for (const SpikeState &spike : spikes_) {
        if (has_nonfinite_state) {
            break;
        }
        has_nonfinite_state =
            spike.active &&
            (!std::isfinite(spike.x) || !std::isfinite(spike.y) ||
             !std::isfinite(spike.velocity_y));
    }
    if (has_nonfinite_state) {
        nonfinite_resets_.fetch_add(1, std::memory_order_relaxed);
        reset_game();
    }
}

esp_err_t RagdollAvalancheApp::update(float dt)
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t update_start_us = esp_timer_get_time();

    const bool reset_requested =
        reset_requested_.exchange(false, std::memory_order_acq_rel);
    if (reset_requested) {
        reset_game();
    } else {
        for (int substep_index = 0; substep_index < kSubstepsPerUpdate;
             ++substep_index) {
            step_substep();
        }
    }

    AvalancheFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        snapshot->animation_tick = animation_tick_;
        snapshot->score = score_;
        snapshot->best_score = best_score_;
        snapshot->top_scores = top_scores_;
        snapshot->game_over = game_over_.load(std::memory_order_relaxed);
        snapshot->player_x = player_x_;
        snapshot->player_y = player_y_;
        snapshot->pose = ragdoll_pose_;
        for (int spike_index = 0; spike_index < kMaxSpikes; ++spike_index) {
            const SpikeState &spike = spikes_[spike_index];
            snapshot->spikes[spike_index] = {
                spike.x,
                spike.y,
                spike.active,
            };
        }
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(
        static_cast<uint32_t>(esp_timer_get_time() - update_start_us),
        std::memory_order_relaxed);
    return ESP_OK;
}

AppStats RagdollAvalancheApp::stats()
{
    AppStats result{};
    result.count = active_count_.load(std::memory_order_relaxed);
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

void RagdollAvalancheApp::draw_spike(uint16_t *pixels, int width, int stripe_y,
                                     int stripe_rows, float x, float tip_y,
                                     uint16_t shaft_color, uint16_t tip_color)
{
    const int center_x = static_cast<int>(x + 0.5f);
    const int tip = static_cast<int>(tip_y + 0.5f);
    constexpr int kLength = 30;
    constexpr int kMaxHalfWidth = 3;
    for (int distance = 1; distance <= kLength; ++distance) {
        const int half_width = 1 + (distance * (kMaxHalfWidth - 1)) / kLength;
        fill_rect(pixels, width, stripe_y, stripe_rows, center_x - half_width,
                  tip - distance, center_x + half_width + 1, tip - distance + 1,
                  shaft_color);
    }
    fill_disc(pixels, width, stripe_y, stripe_rows, center_x, tip, 1,
              tip_color);
}

void RagdollAvalancheApp::draw_ragdoll(uint16_t *pixels, int width,
                                       int stripe_y, int stripe_rows, float x,
                                       float y, const RagdollPose &pose,
                                       uint16_t body_color, uint16_t head_color)
{
    const RagdollGeometry geometry = calculate_ragdoll_geometry(x, y, pose);
    auto round_coordinate = [](float value) {
        return static_cast<int>(value + (value >= 0.0f ? 0.5f : -0.5f));
    };
    auto draw_limb = [&](const LimbGeometry &limb) {
        fill_segment(pixels, width, stripe_y, stripe_rows,
                     round_coordinate(limb.anchor.x),
                     round_coordinate(limb.anchor.y),
                     round_coordinate(limb.joint.x),
                     round_coordinate(limb.joint.y), kLimbRadius, body_color);
        fill_segment(pixels, width, stripe_y, stripe_rows,
                     round_coordinate(limb.joint.x),
                     round_coordinate(limb.joint.y),
                     round_coordinate(limb.end.x), round_coordinate(limb.end.y),
                     kLimbRadius, body_color);
    };

    for (const LimbGeometry &limb : geometry.limbs) {
        draw_limb(limb);
    }

    const int shoulder_x = round_coordinate(geometry.shoulder.x);
    const int shoulder_y = round_coordinate(geometry.shoulder.y);
    const int hip_x = round_coordinate(geometry.hip.x);
    const int hip_y = round_coordinate(geometry.hip.y);
    const int head_x = round_coordinate(geometry.head.x);
    const int head_y = round_coordinate(geometry.head.y);
    fill_segment(pixels, width, stripe_y, stripe_rows, shoulder_x, shoulder_y,
                 hip_x, hip_y, kTorsoRadius, body_color);
    fill_disc(pixels, width, stripe_y, stripe_rows, shoulder_x, shoulder_y,
              kTorsoRadius, body_color);
    fill_disc(pixels, width, stripe_y, stripe_rows, hip_x, hip_y, kTorsoRadius,
              body_color);
    fill_segment(pixels, width, stripe_y, stripe_rows, shoulder_x, shoulder_y,
                 head_x, head_y, kLimbRadius, body_color);
    fill_disc(pixels, width, stripe_y, stripe_rows, head_x, head_y,
              static_cast<int>(kHeadRadius), head_color);
}

void RagdollAvalancheApp::draw_digit(uint16_t *pixels, int width, int stripe_y,
                                     int stripe_rows, int digit, int x, int y,
                                     int scale, uint16_t color)
{
    if (digit < 0 || digit > 9 || scale <= 0) {
        return;
    }
    for (int row = 0; row < 5; ++row) {
        const uint8_t row_bits = kDigitBitmap[digit][row];
        for (int column = 0; column < 3; ++column) {
            if ((row_bits >> (2 - column)) & 1u) {
                fill_rect(pixels, width, stripe_y, stripe_rows,
                          x + column * scale, y + row * scale,
                          x + (column + 1) * scale, y + (row + 1) * scale,
                          color);
            }
        }
    }
}

void RagdollAvalancheApp::draw_number(uint16_t *pixels, int width, int stripe_y,
                                      int stripe_rows, int32_t value, int x,
                                      int y, int scale, uint16_t color,
                                      int min_digits)
{
    char digit_text[12];
    const int length =
        std::snprintf(digit_text, sizeof(digit_text), "%" PRId32, value);
    if (length < 0) {
        return;
    }
    const int digit_count = length > 10 ? 10 : length;
    const int padding_count =
        min_digits > digit_count ? min_digits - digit_count : 0;
    const int cell_width = 4 * scale;
    for (int cell_index = 0; cell_index < digit_count + padding_count;
         ++cell_index) {
        int digit_value = 0;
        if (cell_index >= padding_count) {
            digit_value = digit_text[cell_index - padding_count] - '0';
            if (digit_value < 0 || digit_value > 9) {
                digit_value = 0;
            }
        }
        draw_digit(pixels, width, stripe_y, stripe_rows, digit_value,
                   x + cell_index * cell_width, y, scale, color);
    }
}

void RagdollAvalancheApp::raster_stripe(const AvalancheFrame &frame,
                                        uint16_t *pixels, int width,
                                        int stripe_y, int stripe_rows)
{
    for (int local_y = 0; local_y < stripe_rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        for (int screen_x = 0; screen_x < width; ++screen_x) {
            row[screen_x] = kBackground;
        }
    }
    draw_snow_specks(pixels, width, stripe_y, stripe_rows);

    for (const SpikeRender &spike : frame.spikes) {
        if (spike.active) {
            draw_spike(pixels, width, stripe_y, stripe_rows, spike.x, spike.y,
                       kSpikeShaft, kSpikeTip);
        }
    }

    draw_ragdoll(pixels, width, stripe_y, stripe_rows, frame.player_x,
                 frame.player_y, frame.pose, kBody, kHead);

    int score_digit_count = 1;
    for (int32_t remaining = frame.score;
         remaining >= 10 && score_digit_count < 4; remaining /= 10) {
        ++score_digit_count;
    }
    const int score_width = score_digit_count * 8 - 2;
    draw_number(pixels, width, stripe_y, stripe_rows, frame.score,
                120 - score_width / 2, 3, 2, kScoreColor, 1);

    fill_rect(pixels, width, stripe_y, stripe_rows, 204, 4, 208, 6, kBestColor);
    fill_rect(pixels, width, stripe_y, stripe_rows, 212, 6, 216, 7, kBestColor);
    draw_number(pixels, width, stripe_y, stripe_rows, frame.best_score, 210, 9,
                1, kBestColor, 1);

    if (frame.game_over) {
        fill_rect(pixels, width, stripe_y, stripe_rows, 0, 58, width, 204,
                  kGameOverBg);
        fill_rect(pixels, width, stripe_y, stripe_rows, 108, 66, 132, 68,
                  kSpikeTip);
        fill_rect(pixels, width, stripe_y, stripe_rows, 118, 62, 122, 72,
                  kSpikeTip);
        draw_number(pixels, width, stripe_y, stripe_rows, frame.score,
                    120 - score_width / 2, 82, 2, kScoreColor, 1);

        fill_rect(pixels, width, stripe_y, stripe_rows, 56, 113, 184, 114,
                  kSeparator);
        fill_rect(pixels, width, stripe_y, stripe_rows, 56, 151, 184, 152,
                  kSeparator);

        for (int high_score_index = 0; high_score_index < kHighScoreCount;
             ++high_score_index) {
            if (frame.top_scores[high_score_index] == 0) {
                break;
            }
            const int table_y = 118 + high_score_index * 6;
            draw_digit(pixels, width, stripe_y, stripe_rows,
                       high_score_index + 1, 64, table_y, 1, kBestColor);
            fill_rect(pixels, width, stripe_y, stripe_rows, 78, table_y + 2, 82,
                      table_y + 3, kSeparator);
            draw_number(pixels, width, stripe_y, stripe_rows,
                        frame.top_scores[high_score_index], 86, table_y, 1,
                        kScoreColor, 1);
        }

        fill_rect(pixels, width, stripe_y, stripe_rows, 74, 168, 166, 170,
                  kSeparator);
        fill_rect(pixels, width, stripe_y, stripe_rows, 116, 172, 124, 176,
                  frame.animation_tick % 16u < 8u ? kScoreColor : kMuted);
    }
}

bool RagdollAvalancheApp::render(DisplayFrame &frame)
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

    const AvalancheFrame *snapshot = frames_.acquire_latest();
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
