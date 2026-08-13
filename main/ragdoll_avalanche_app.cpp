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

#include "launcher_icons.hpp"

namespace fluid_demo {

namespace {

constexpr const char *kTag = "ragdoll";

// Logical RGB565 colors; render() converts each stripe to wire order.
constexpr uint16_t kBackground = 0x2D4B;       // deep night sky
constexpr uint16_t kSnowDot = 0x5ADB;          // faint static snow specks
constexpr uint16_t kBody = 0xB5B6;             // pale blue-grey ragdoll
constexpr uint16_t kHead = 0xFFDE;             // warm-white head
constexpr uint16_t kSpikeShaft = 0x9CD3;       // steel shaft
constexpr uint16_t kSpikeTip = 0xFFDE;         // white-hot tip
constexpr uint16_t kScoreColor = 0xFE8E;       // warm gold score
constexpr uint16_t kBestColor = 0x5CF3;        // muted teal BEST value
constexpr uint16_t kGameOverBg = 0xD546;       // game-over overlay band
constexpr uint16_t kSeparator = 0x5AEB;        // soft divider
constexpr uint16_t kMuted = 0x7BEF;            // hollow/unused accent

// 3x5 pixel digit font — 5 rows of 3 bits per digit. Bit 2 (MSB of row byte)
// is the left column. Scaled at draw time.
constexpr uint8_t kDigitBitmap[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111},  // 0
    {0b010, 0b110, 0b010, 0b010, 0b111},  // 1
    {0b111, 0b001, 0b111, 0b100, 0b111},  // 2
    {0b111, 0b001, 0b111, 0b001, 0b111},  // 3
    {0b101, 0b101, 0b111, 0b001, 0b001},  // 4
    {0b111, 0b100, 0b111, 0b001, 0b111},  // 5
    {0b111, 0b100, 0b111, 0b101, 0b111},  // 6
    {0b111, 0b001, 0b001, 0b001, 0b001},  // 7
    {0b111, 0b101, 0b111, 0b101, 0b111},  // 8
    {0b111, 0b101, 0b111, 0b001, 0b111},  // 9
};

constexpr LauncherVisual kLauncherVisual{
    kBackground,
    kBody,
    0x6B4D,
    kScoreColor,
    kIconAvalanche,
};

inline int min_int(int a, int b) { return a < b ? a : b; }
inline int max_int(int a, int b) { return a > b ? a : b; }
inline float clamp_float(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

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
inline bool finite_vec(const Vec3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
inline float rand_float()
{
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

constexpr float kSubstepDt = 1.0f / 60.0f;
constexpr int kSubstepsPerUpdate = 2;

/// Deterministic static snow-speck pattern (no storage, recomputed cheaply).
void draw_snow_specks(uint16_t *pixels, int width, int y0, int rows)
{
    for (int i = 0; i < 56; ++i) {
        const int x = (i * 97 + 53) % 240;
        const int y = (i * 149 + 31) % 224;
        if (y < y0 || y >= y0 + rows) {
            continue;
        }
        pixels[(y - y0) * width + x] = kSnowDot;
    }
}

}  // namespace

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

    if (!nvs_inited_) {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
            err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(kTag, "NVS needs erase, retrying");
            ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
            err = nvs_flash_init();
        }
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "NVS init failed: %s", esp_err_to_name(err));
        }
        nvs_inited_ = true;
    }

    std::srand(static_cast<unsigned>(esp_random()));

    load_highscores();

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
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
    return ESP_OK;
}

void RagdollAvalancheApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
}

bool RagdollAvalancheApp::on_motion(const MotionTick &tick)
{
    const bool physical =
        tick.fresh && std::isfinite(tick.dt) && tick.dt > 0.0f &&
        finite_vec(tick.accel_mps2) && finite_vec(tick.gyro_rads);
    bool physical_accepted = false;
    Vec3 filtered{};
    if (physical) {
        filtered = filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
        physical_accepted = filter_.last_sample_accepted() && finite_vec(filtered);
    }

    const bool override_valid =
        tick.override_active && finite_vec(tick.apparent_accel);
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

void RagdollAvalancheApp::on_touch(const TouchEvent &event)
{
    static_cast<void>(event);
    // Tap anywhere restarts after game over; ingame tap is ignored.
    if (game_over_.load(std::memory_order_acquire)) {
        reset_requested_.store(true, std::memory_order_release);
    }
}

ShellAction RagdollAvalancheApp::handle_event(AppEvent event)
{
    if (event == AppEvent::PlusPress) {
        reset_requested_.store(true, std::memory_order_release);
    }
    return ShellAction::None;
}

void RagdollAvalancheApp::spawn_wave()
{
    int count =
        kWaveInitialCount +
        static_cast<int>(survival_time_ / kWaveCountRampSeconds);
    if (count > kWaveMaxCount) {
        count = kWaveMaxCount;
    }

    // Courtesy applies to exactly one wave. Every later wave may target the
    // player's current lane, even before the first spikes have left the panel.
    const bool courtesy = first_wave_pending_;
    first_wave_pending_ = false;

    float placed[kWaveMaxCount] = {};
    int placed_count = 0;

    for (int attempt = 0; attempt < count; ++attempt) {
        // Find a free slot; skip this spike if the pool is exhausted.
        int slot = -1;
        for (int i = 0; i < kMaxSpikes; ++i) {
            if (!spikes_[i].active) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            return;
        }

        float x = 24.0f + rand_float() * (kFWidth - 48.0f);
        // Keep each wave readable while still covering several lanes. The
        // first wave also leaves one full-body-width escape corridor.
        bool spaced = false;
        for (int retry = 0; retry < 12; ++retry) {
            spaced = true;
            if (courtesy && std::fabs(x - player_x_) < kFirstWaveClearance) {
                spaced = false;
            }
            for (int p = 0; spaced && p < placed_count; ++p) {
                if (std::fabs(x - placed[p]) < kWaveSpacingMin) {
                    spaced = false;
                }
            }
            if (spaced) {
                break;
            }
            x = clamp_float(x + (rand_float() - 0.5f) * 100.0f,
                            24.0f, kFWidth - 24.0f);
        }
        if (!spaced) {
            continue;
        }

        SpikeState &s = spikes_[slot];
        s.active = true;
        s.x = x;
        s.y = kSpikeSpawnY;
        const float time_speed =
            kSpikeVInitial + survival_time_ * kSpikeVTimeRamp;
        const float varied_speed =
            time_speed + rand_float() * kSpikeVRandom;
        s.vy = varied_speed > kSpikeVMax ? kSpikeVMax : varied_speed;
        placed[placed_count++] = x;
        active_count_.fetch_add(1, std::memory_order_relaxed);
    }
}

void RagdollAvalancheApp::kill_player()
{
    if (game_over_.load(std::memory_order_relaxed)) {
        return;
    }
    if (std::fabs(player_vx_) > 4.0f) {
        death_flop_direction_ = player_vx_ < 0.0f ? -1.0f : 1.0f;
    } else {
        death_flop_direction_ =
            ragdoll_pose_.body_angle < 0.0f ? -1.0f : 1.0f;
    }
    body_angular_velocity_ += death_flop_direction_ * 2.4f;
    player_vx_ = 0.0f;
    player_vy_ = 0.0f;
    game_over_.store(true, std::memory_order_relaxed);
    save_highscore(score_);
    ESP_LOGI(kTag, "game over score=%" PRId32 " best=%" PRId32,
             score_, best_score_);
}

void RagdollAvalancheApp::reset_game()
{
    ++epoch_;
    if (epoch_ == 0u) {
        epoch_ = 1u;
    }
    for (auto &s : spikes_) {
        s.active = false;
    }
    active_count_.store(0, std::memory_order_relaxed);
    player_x_ = 120.0f;
    player_y_ = 120.0f;
    player_vx_ = 0.0f;
    player_vy_ = 0.0f;
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
RagdollAvalancheApp::calculate_ragdoll_geometry(
    float x, float y, const RagdollPose &pose)
{
    RagdollGeometry geometry{};
    const float body_x = std::sin(pose.body_angle);
    const float body_y = std::cos(pose.body_angle);
    const float perpendicular_x = std::cos(pose.body_angle);
    const float perpendicular_y = -std::sin(pose.body_angle);

    geometry.shoulder = {
        x - body_x * kTorsoHalfLength,
        y - body_y * kTorsoHalfLength,
    };
    geometry.hip = {
        x + body_x * kTorsoHalfLength,
        y + body_y * kTorsoHalfLength,
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
        make_limb(left_shoulder, LeftUpperArm, LeftForearm,
                  kUpperArmLength, kForearmLength),
        make_limb(right_shoulder, RightUpperArm, RightForearm,
                  kUpperArmLength, kForearmLength),
        make_limb(left_hip, LeftThigh, LeftShin,
                  kThighLength, kShinLength),
        make_limb(right_hip, RightThigh, RightShin,
                  kThighLength, kShinLength),
    };

    geometry.min_x = geometry.head.x - kHeadRadius;
    geometry.max_x = geometry.head.x + kHeadRadius;
    geometry.min_y = geometry.head.y - kHeadRadius;
    geometry.max_y = geometry.head.y + kHeadRadius;
    auto include_point = [&](const RagdollPoint &point, float radius) {
        geometry.min_x =
            geometry.min_x < point.x - radius
                ? geometry.min_x
                : point.x - radius;
        geometry.max_x =
            geometry.max_x > point.x + radius
                ? geometry.max_x
                : point.x + radius;
        geometry.min_y =
            geometry.min_y < point.y - radius
                ? geometry.min_y
                : point.y - radius;
        geometry.max_y =
            geometry.max_y > point.y + radius
                ? geometry.max_y
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
                                            float acceleration_y,
                                            bool dead)
{
    const float acceleration_reference = kAccelScale * 6.0f;
    const float normalized_ax =
        clamp_float(acceleration_x / acceleration_reference, -1.5f, 1.5f);
    const float normalized_ay =
        clamp_float(acceleration_y / acceleration_reference, -1.5f, 1.5f);
    const float normalized_vx =
        clamp_float(player_vx_ / kMaxSpeed, -1.0f, 1.0f);
    const float normalized_vy =
        clamp_float(player_vy_ / kMaxSpeed, -1.0f, 1.0f);

    // Effective force in the moving body frame. Translation acceleration and
    // velocity pull the limbs in the opposite direction; a small screen-down
    // bias keeps a readable hanging pose while motion is neutral.
    const float effective_x =
        -0.90f * normalized_ax - 0.35f * normalized_vx;
    const float effective_y =
        0.55f - 0.85f * normalized_ay - 0.20f * normalized_vy;
    const float hang_angle = std::atan2(effective_x, effective_y);

    const float body_target =
        dead ? death_flop_direction_ * 1.45f
             : clamp_float(0.28f * normalized_ax + 0.22f * normalized_vx,
                           -0.62f, 0.62f);
    const float body_error =
        wrap_angle(body_target - ragdoll_pose_.body_angle);
    body_angular_velocity_ += body_error * 18.0f * kSubstepDt;
    body_angular_velocity_ *= 0.93f;
    body_angular_velocity_ =
        clamp_float(body_angular_velocity_, -5.5f, 5.5f);
    ragdoll_pose_.body_angle =
        wrap_angle(ragdoll_pose_.body_angle +
                   body_angular_velocity_ * kSubstepDt);

    const float speed =
        clamp_float(std::sqrt(normalized_vx * normalized_vx +
                              normalized_vy * normalized_vy),
                    0.0f, 1.0f);
    const float arm_spread = 0.82f + speed * 0.20f;
    const float arm_bend = 0.30f + speed * 0.10f;
    const float leg_spread = 0.24f + speed * 0.12f;
    const float knee_bend = 0.20f + speed * 0.08f;
    const float body_follow = ragdoll_pose_.body_angle * 0.16f;

    const float left_upper_arm_target =
        wrap_angle(hang_angle - arm_spread + body_follow);
    const float right_upper_arm_target =
        wrap_angle(hang_angle + arm_spread + body_follow);
    const float left_thigh_target =
        wrap_angle(hang_angle - leg_spread + body_follow);
    const float right_thigh_target =
        wrap_angle(hang_angle + leg_spread + body_follow);
    const std::array<float, JointCount> targets{
        left_upper_arm_target,
        wrap_angle(left_upper_arm_target - arm_bend),
        right_upper_arm_target,
        wrap_angle(right_upper_arm_target + arm_bend),
        left_thigh_target,
        wrap_angle(left_thigh_target + knee_bend),
        right_thigh_target,
        wrap_angle(right_thigh_target - knee_bend),
    };

    auto step_joint = [&](JointIndex joint, float stiffness, float damping) {
        const std::size_t index = static_cast<std::size_t>(joint);
        const float error =
            wrap_angle(targets[index] - ragdoll_pose_.joint_angles[index]);
        float &velocity = joint_velocities_[index];
        velocity = (velocity + error * stiffness * kSubstepDt) * damping;
        velocity = clamp_float(velocity, -8.0f, 8.0f);
        ragdoll_pose_.joint_angles[index] =
            wrap_angle(ragdoll_pose_.joint_angles[index] +
                       velocity * kSubstepDt);
    };

    // Distal joints still lag the driven center, but stronger control plus
    // handed bend limits keeps elbows and knees from turning inside out.
    step_joint(LeftUpperArm, 14.0f, 0.95f);
    step_joint(RightUpperArm, 14.0f, 0.95f);
    step_joint(LeftForearm, 10.5f, 0.962f);
    step_joint(RightForearm, 10.5f, 0.962f);
    step_joint(LeftThigh, 16.0f, 0.945f);
    step_joint(RightThigh, 16.0f, 0.945f);
    step_joint(LeftShin, 11.0f, 0.960f);
    step_joint(RightShin, 11.0f, 0.960f);

    auto constrain_bend = [&](JointIndex lower, JointIndex upper,
                              float min_bend, float max_bend) {
        const std::size_t lower_index = static_cast<std::size_t>(lower);
        const std::size_t upper_index = static_cast<std::size_t>(upper);
        const float bend =
            wrap_angle(ragdoll_pose_.joint_angles[lower_index] -
                       ragdoll_pose_.joint_angles[upper_index]);
        const float constrained = clamp_float(bend, min_bend, max_bend);
        if (constrained != bend) {
            ragdoll_pose_.joint_angles[lower_index] =
                wrap_angle(ragdoll_pose_.joint_angles[upper_index] +
                           constrained);
            joint_velocities_[lower_index] =
                joint_velocities_[upper_index];
        }
    };
    constrain_bend(LeftForearm, LeftUpperArm, -0.95f, -0.08f);
    constrain_bend(RightForearm, RightUpperArm, 0.08f, 0.95f);
    constrain_bend(LeftShin, LeftThigh, 0.06f, 0.75f);
    constrain_bend(RightShin, RightThigh, -0.75f, -0.06f);
}

void RagdollAvalancheApp::load_highscores()
{
    // Any mismatch (missing key, short blob, corrupt shape) falls back to a
    // zeroed table; a later qualifying game over rewrites the key cleanly.
    best_score_ = 0;
    top_scores_.fill(0);

    nvs_handle_t handle;
    const esp_err_t open_err = nvs_open("ragdoll", NVS_READONLY, &handle);
    if (open_err == ESP_ERR_NVS_NOT_FOUND) {
        return;  // first boot: nothing stored yet
    }
    if (open_err != ESP_OK) {
        ESP_LOGW(kTag, "highscore load: nvs_open failed: %s",
                 esp_err_to_name(open_err));
        return;
    }

    std::size_t size = kMaxHighscores * sizeof(int32_t);
    const esp_err_t get_err =
        nvs_get_blob(handle, "top5", top_scores_.data(), &size);
    nvs_close(handle);

    if (get_err != ESP_OK) {
        if (get_err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kTag, "highscore load: nvs_get_blob failed: %s",
                     esp_err_to_name(get_err));
        }
        top_scores_.fill(0);
        return;
    }
    if (size != kMaxHighscores * sizeof(int32_t)) {
        ESP_LOGW(kTag, "highscore load: bad blob size %u, discarding",
                 static_cast<unsigned>(size));
        top_scores_.fill(0);
        return;
    }

    // Shape validation: values descending, positive, and the only legal
    // zero is as a trailing suffix (a nonzero may never follow a zero or a
    // larger value).
    bool valid = true;
    bool seen_zero = false;
    for (int i = 0; i < kMaxHighscores; ++i) {
        const int32_t value = top_scores_[i];
        if (value < 0 || (seen_zero && value != 0)) {
            valid = false;  // negative, or a nonzero after a zero
            break;
        }
        if (i > 0 && value > top_scores_[i - 1]) {
            valid = false;  // not descending
            break;
        }
        if (value == 0) {
            seen_zero = true;
        }
    }
    if (!valid) {
        ESP_LOGW(kTag, "highscore load: corrupt shape [%d %d %d %d %d], "
                       "discarding",
                 top_scores_[0], top_scores_[1], top_scores_[2],
                 top_scores_[3], top_scores_[4]);
        top_scores_.fill(0);
        return;
    }

    best_score_ = top_scores_[0];
    ESP_LOGI(kTag, "highscores loaded best=%" PRId32, best_score_);
}

void RagdollAvalancheApp::save_highscore(int32_t score)
{
    // Runs on the update lane, but only once per game (the fatal hit), when
    // the sim has nothing further to produce: the fixed-dt scheduler absorbs
    // the one-off flash-write stall without visible effect and the score is
    // durable even if the user immediately power-holds the PWR button
    // (leave() would never run). nvs_open allocates a small handle on this
    // transient path only; steady-state gameplay never allocates.
    //
    // A score earns a slot only when it strictly beats the current 5th
    // entry (or the table still has empty slots, whose value is 0). Ties
    // at the boundary do not displace an existing entry.
    if (score <= 0 || score <= top_scores_[kMaxHighscores - 1]) {
        return;
    }

    // Insert into the descending-sorted table, shifting lower scores down.
    int pos = kMaxHighscores - 1;
    while (pos > 0 && top_scores_[pos - 1] < score) {
        top_scores_[pos] = top_scores_[pos - 1];
        --pos;
    }
    top_scores_[pos] = score;
    best_score_ = top_scores_[0];

    nvs_handle_t handle;
    esp_err_t err = nvs_open("ragdoll", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(handle, "top5", top_scores_.data(),
                       kMaxHighscores * sizeof(int32_t));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs write failed: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

void RagdollAvalancheApp::step_substep()
{
    ++tick_;
    if (tick_ == 0u) {
        tick_ = 1u;
    }

    // --- Fast, slippery center-of-mass steering ---
    const bool dead = game_over_.load(std::memory_order_relaxed);
    float acceleration_x = 0.0f;
    float acceleration_y = 0.0f;
    if (!dead) {
        Vec3 apparent{};
        bool valid = false;
        portENTER_CRITICAL(&motion_mux_);
        if (motion_.valid) {
            apparent = motion_.apparent;
            valid = true;
        }
        portEXIT_CRITICAL(&motion_mux_);

        if (valid) {
            // Match FluidBoxApp's box-to-screen conversion: box +y is
            // screen-up, while player +y is screen-down.
            acceleration_x =
                clamp_float(apparent.x, -kMoveClamp, kMoveClamp) * kAccelScale;
            acceleration_y =
                clamp_float(-apparent.y, -kMoveClamp, kMoveClamp) * kAccelScale;
            player_vx_ =
                (player_vx_ + acceleration_x * kSubstepDt) * kDamping;
            player_vy_ =
                (player_vy_ + acceleration_y * kSubstepDt) * kDamping;
            const float speed = std::sqrt(player_vx_ * player_vx_ +
                                          player_vy_ * player_vy_);
            if (speed > kMaxSpeed) {
                player_vx_ *= kMaxSpeed / speed;
                player_vy_ *= kMaxSpeed / speed;
            }
        } else {
            player_vx_ *= kDamping;
            player_vy_ *= kDamping;
        }

        player_x_ += player_vx_ * kSubstepDt;
        player_y_ += player_vy_ * kSubstepDt;

        // Every difficulty axis advances with alive time, including while no
        // spike has yet crossed the bottom edge to increment the score.
        survival_time_ += kSubstepDt;
        const float ramped_interval =
            kWaveIntervalInitial -
            survival_time_ * kWaveIntervalTimeRamp;
        wave_interval_ =
            ramped_interval < kWaveIntervalMin
                ? kWaveIntervalMin
                : ramped_interval;
        wave_timer_ += kSubstepDt;
        if (wave_timer_ >= wave_interval_) {
            wave_timer_ -= wave_interval_;
            spawn_wave();
        }
    }

    // Pose physics continues after death so the body settles instead of
    // snapping to a canned horizontal sprite.
    step_ragdoll_pose(acceleration_x, acceleration_y, dead);

    // Clamp the current articulated silhouette, not its center, so compact
    // poses can reach the panel edge without any limb or head being clipped.
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
        if ((shift_x > 0.0f && player_vx_ < 0.0f) ||
            (shift_x < 0.0f && player_vx_ > 0.0f)) {
            player_vx_ = 0.0f;
        }
        if ((shift_y > 0.0f && player_vy_ < 0.0f) ||
            (shift_y < 0.0f && player_vy_ > 0.0f)) {
            player_vy_ = 0.0f;
        }
        player_geometry =
            calculate_ragdoll_geometry(player_x_, player_y_, ragdoll_pose_);
    }

    // --- Spikes: pass 1 integrates and scores every exit using one
    // game-over snapshot, so a spike that exited the same substep the player
    // was hit scores identically regardless of slot order. ---
    const bool was_dead = game_over_.load(std::memory_order_relaxed);
    for (int i = 0; i < kMaxSpikes; ++i) {
        SpikeState &s = spikes_[i];
        if (!s.active) {
            continue;
        }
        s.y += s.vy * kSubstepDt;
        if (s.y > kSpikeExitY) {
            s.active = false;
            active_count_.fetch_sub(1, std::memory_order_relaxed);
            if (!was_dead) {
                ++score_;  // dodged
            }
        }
    }

    // Only the head and upper chest are lethal. A spike may visibly pass
    // through an arm, hip, or leg without ending the run.
    if (!was_dead) {
        const float lethal_dx =
            player_geometry.upper_chest.x - player_geometry.head.x;
        const float lethal_dy =
            player_geometry.upper_chest.y - player_geometry.head.y;
        const float lethal_length_sq =
            lethal_dx * lethal_dx + lethal_dy * lethal_dy;
        const float inverse_lethal_length_sq =
            lethal_length_sq > 0.0f ? 1.0f / lethal_length_sq : 0.0f;
        const float lethal_radius_sq = kLethalRadius * kLethalRadius;
        for (int i = 0; i < kMaxSpikes; ++i) {
            const SpikeState &s = spikes_[i];
            if (!s.active) {
                continue;
            }
            const float projection = clamp_float(
                ((s.x - player_geometry.head.x) * lethal_dx +
                 (s.y - player_geometry.head.y) * lethal_dy) *
                    inverse_lethal_length_sq,
                0.0f, 1.0f);
            const float nearest_x =
                player_geometry.head.x + projection * lethal_dx;
            const float nearest_y =
                player_geometry.head.y + projection * lethal_dy;
            const float dx = s.x - nearest_x;
            const float dy = s.y - nearest_y;
            if (dx * dx + dy * dy <= lethal_radius_sq) {
                kill_player();
                break;
            }
        }
    }

    // --- Nonfinite guards ---
    bool bad = !std::isfinite(player_x_) || !std::isfinite(player_y_) ||
               !std::isfinite(player_vx_) || !std::isfinite(player_vy_) ||
               !std::isfinite(survival_time_) ||
               !std::isfinite(wave_timer_) ||
               !std::isfinite(wave_interval_) ||
               !std::isfinite(ragdoll_pose_.body_angle) ||
               !std::isfinite(body_angular_velocity_);
    for (std::size_t i = 0; i < JointCount && !bad; ++i) {
        bad = !std::isfinite(ragdoll_pose_.joint_angles[i]) ||
              !std::isfinite(joint_velocities_[i]);
    }
    for (int i = 0; i < kMaxSpikes && !bad; ++i) {
        const SpikeState &s = spikes_[i];
        if (s.active &&
            (!std::isfinite(s.x) || !std::isfinite(s.y) ||
             !std::isfinite(s.vy))) {
            bad = true;
        }
    }
    if (bad) {
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

    const int64_t update_start = esp_timer_get_time();

    const bool reset = reset_requested_.exchange(false, std::memory_order_acq_rel);
    if (reset) {
        // Publish the exact fresh state this update; held tilt acting on a
        // cleared field starts with the following fixed update, exactly like
        // the reference maze reset.
        reset_game();
    } else {
        for (int step = 0; step < kSubstepsPerUpdate; ++step) {
            step_substep();
        }
    }

    AvalancheFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        fill_snapshot(*snapshot);
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - update_start),
                      std::memory_order_relaxed);
    return ESP_OK;
}

void RagdollAvalancheApp::fill_snapshot(AvalancheFrame &snapshot)
{
    ++sequence_;
    if (sequence_ == 0u) {
        sequence_ = 1u;
    }
    snapshot.sequence = sequence_;
    snapshot.epoch = epoch_;
    snapshot.tick = tick_;
    snapshot.score = score_;
    snapshot.best_score = best_score_;
    snapshot.top_scores = top_scores_;
    snapshot.game_over = game_over_.load(std::memory_order_relaxed);
    snapshot.player_x = player_x_;
    snapshot.player_y = player_y_;
    snapshot.pose = ragdoll_pose_;

    for (int i = 0; i < kMaxSpikes; ++i) {
        const SpikeState &src = spikes_[i];
        snapshot.spikes[i].x = src.x;
        snapshot.spikes[i].y = src.y;
        snapshot.spikes[i].active = src.active;
    }
}

AppStats RagdollAvalancheApp::stats()
{
    AppStats result{};
    result.count = active_count_.load(std::memory_order_relaxed);
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

void RagdollAvalancheApp::draw_rect(uint16_t *pixels, int width, int y0,
                                    int rows, int left, int top,
                                    int right, int bottom, uint16_t color)
{
    if (pixels == nullptr || width <= 0 || rows <= 0 || left >= right || top >= bottom) {
        return;
    }
    const int c_left = max_int(0, left);
    const int c_right = min_int(width, right);
    const int c_top = max_int(y0, top);
    const int c_bot = min_int(y0 + rows, bottom);
    if (c_left >= c_right || c_top >= c_bot) {
        return;
    }
    for (int y = c_top; y < c_bot; ++y) {
        uint16_t *row = pixels + (y - y0) * width;
        for (int x = c_left; x < c_right; ++x) {
            row[x] = color;
        }
    }
}

void RagdollAvalancheApp::draw_disc(uint16_t *pixels, int width, int y0,
                                    int rows, int cx, int cy, int radius,
                                    uint16_t color)
{
    if (pixels == nullptr || width <= 0 || rows <= 0 || radius < 0) {
        return;
    }
    const int top = max_int(y0, cy - radius);
    const int bottom = min_int(y0 + rows - 1, cy + radius);
    const int left = max_int(0, cx - radius);
    const int right = min_int(width - 1, cx + radius);
    const int r_sq = radius * radius;
    for (int y = top; y <= bottom; ++y) {
        uint16_t *row = pixels + (y - y0) * width;
        const int dy = y - cy;
        for (int x = left; x <= right; ++x) {
            const int dx = x - cx;
            if (dx * dx + dy * dy <= r_sq) {
                row[x] = color;
            }
        }
    }
}

void RagdollAvalancheApp::draw_segment(uint16_t *pixels, int width, int y0,
                                       int rows, int x0, int y0_screen,
                                       int x1, int y1, int radius,
                                       uint16_t color)
{
    const int dx = x1 - x0;
    const int dy = y1 - y0_screen;
    const int extent = max_int(dx < 0 ? -dx : dx, dy < 0 ? -dy : dy);
    const int stride = max_int(1, radius);
    const int samples = max_int(1, extent / stride);
    for (int sample = 0; sample <= samples; ++sample) {
        draw_disc(pixels, width, y0, rows,
                  x0 + (dx * sample) / samples,
                  y0_screen + (dy * sample) / samples,
                  radius, color);
    }
}

void RagdollAvalancheApp::draw_spike(uint16_t *pixels, int width, int y0,
                                     int rows, float x, float tip_y,
                                     uint16_t shaft_color, uint16_t tip_color)
{
    const int cx = static_cast<int>(x + 0.5f);
    const int tip = static_cast<int>(tip_y + 0.5f);
    constexpr int kLength = 30;
    constexpr int kMaxHalfWidth = 3;
    // A long needle widens away from its downward-facing point.
    for (int distance = 1; distance <= kLength; ++distance) {
        const int half_width =
            1 + (distance * (kMaxHalfWidth - 1)) / kLength;
        draw_rect(pixels, width, y0, rows,
                  cx - half_width, tip - distance,
                  cx + half_width + 1, tip - distance + 1, shaft_color);
    }
    draw_disc(pixels, width, y0, rows, cx, tip, 1, tip_color);
}

void RagdollAvalancheApp::draw_ragdoll(uint16_t *pixels, int width, int y0,
                                       int rows, float x, float y,
                                       const RagdollPose &pose,
                                       uint16_t body_color,
                                       uint16_t head_color)
{
    // Render the exact geometry used by bounds and collision; immutable
    // snapshot inputs keep joints continuous across stripe boundaries.
    const RagdollGeometry geometry =
        calculate_ragdoll_geometry(x, y, pose);
    auto round_coordinate = [](float value) {
        return static_cast<int>(value + (value >= 0.0f ? 0.5f : -0.5f));
    };
    auto draw_limb = [&](const LimbGeometry &limb) {
        draw_segment(pixels, width, y0, rows,
                     round_coordinate(limb.anchor.x),
                     round_coordinate(limb.anchor.y),
                     round_coordinate(limb.joint.x),
                     round_coordinate(limb.joint.y),
                     kLimbRadius, body_color);
        draw_segment(pixels, width, y0, rows,
                     round_coordinate(limb.joint.x),
                     round_coordinate(limb.joint.y),
                     round_coordinate(limb.end.x),
                     round_coordinate(limb.end.y),
                     kLimbRadius, body_color);
    };

    // One body color closes the shoulder, elbow, hip, and knee seams while
    // rounded capsule ends remain readable as hands and feet.
    for (const LimbGeometry &limb : geometry.limbs) {
        draw_limb(limb);
    }

    const int shoulder_x = round_coordinate(geometry.shoulder.x);
    const int shoulder_y = round_coordinate(geometry.shoulder.y);
    const int hip_x = round_coordinate(geometry.hip.x);
    const int hip_y = round_coordinate(geometry.hip.y);
    const int head_x = round_coordinate(geometry.head.x);
    const int head_y = round_coordinate(geometry.head.y);
    draw_segment(pixels, width, y0, rows,
                 shoulder_x, shoulder_y, hip_x, hip_y,
                 kTorsoRadius, body_color);
    draw_disc(pixels, width, y0, rows,
              shoulder_x, shoulder_y, kTorsoRadius, body_color);
    draw_disc(pixels, width, y0, rows,
              hip_x, hip_y, kTorsoRadius, body_color);
    draw_segment(pixels, width, y0, rows,
                 shoulder_x, shoulder_y, head_x, head_y,
                 kLimbRadius, body_color);
    draw_disc(pixels, width, y0, rows, head_x, head_y,
              static_cast<int>(kHeadRadius), head_color);
}

void RagdollAvalancheApp::draw_digit(uint16_t *pixels, int width, int y0,
                                     int rows, int digit, int x, int y,
                                     int scale, uint16_t color)
{
    if (digit < 0 || digit > 9 || scale <= 0) {
        return;
    }
    for (int row = 0; row < 5; ++row) {
        const uint8_t bits = kDigitBitmap[digit][row];
        for (int col = 0; col < 3; ++col) {
            if ((bits >> (2 - col)) & 1) {
                draw_rect(pixels, width, y0, rows,
                          x + col * scale, y + row * scale,
                          x + (col + 1) * scale, y + (row + 1) * scale,
                          color);
            }
        }
    }
}

void RagdollAvalancheApp::draw_number(uint16_t *pixels, int width, int y0,
                                      int rows, int32_t value, int x, int y,
                                      int scale, uint16_t color, int min_digits)
{
    char buf[12];
    const int len = std::snprintf(buf, sizeof(buf), "%" PRId32, value);
    if (len < 0) {
        return;
    }
    const int digits = len > 10 ? 10 : len;
    const int pad = min_digits > digits ? min_digits - digits : 0;
    const int cell = 4 * scale;
    for (int i = 0; i < digits + pad; ++i) {
        int d;
        if (i < pad) {
            d = 0;
        } else {
            d = buf[i - pad] - '0';
            if (d < 0 || d > 9) {
                d = 0;
            }
        }
        draw_digit(pixels, width, y0, rows, d, x + i * cell, y, scale, color);
    }
}

void RagdollAvalancheApp::raster_stripe(const AvalancheFrame &frame,
                                        uint16_t *pixels, int width,
                                        int y0, int rows)
{
    // Background + static snow specks.
    for (int ly = 0; ly < rows; ++ly) {
        uint16_t *row = pixels + ly * width;
        for (int x = 0; x < width; ++x) {
            row[x] = kBackground;
        }
    }
    draw_snow_specks(pixels, width, y0, rows);

    // Spikes (behind the player).
    for (int i = 0; i < kMaxSpikes; ++i) {
        const auto &s = frame.spikes[i];
        if (!s.active) {
            continue;
        }
        draw_spike(pixels, width, y0, rows, s.x, s.y, kSpikeShaft, kSpikeTip);
    }

    // The update lane publishes every articulated joint; rendering never
    // substitutes a canned pose, including after death.
    draw_ragdoll(pixels, width, y0, rows,
                 frame.player_x, frame.player_y, frame.pose,
                 kBody, kHead);

    // --- HUD (drawn last so nothing covers it) ---
    // Score: scale-2 digits, centered top.
    char buf[12];
    const int len = std::snprintf(buf, sizeof(buf), "%" PRId32, frame.score);
    const int digits = len > 4 ? 4 : (len < 1 ? 1 : len);
    const int total_w = digits * 8 - 2;  // 4px cell * scale 2, minus last gap
    draw_number(pixels, width, y0, rows, frame.score,
                120 - total_w / 2, 3, 2, kScoreColor, 1);

    // Best: scale-1 digits, top-right with a small crown mark.
    draw_rect(pixels, width, y0, rows, 204, 4, 208, 6, kBestColor);
    draw_rect(pixels, width, y0, rows, 212, 6, 216, 7, kBestColor);
    draw_number(pixels, width, y0, rows, frame.best_score,
                210, 9, 1, kBestColor, 1);

    // --- Game over overlay ---
    if (frame.game_over) {
        draw_rect(pixels, width, y0, rows, 0, 58, width, 204, kGameOverBg);
        // Fatal-hit X marker.
        draw_rect(pixels, width, y0, rows, 108, 66, 132, 68, kSpikeTip);
        draw_rect(pixels, width, y0, rows, 118, 62, 122, 72, kSpikeTip);

        // Final score, scale 2, centered under the marker.
        char sbuf[12];
        const int slen =
            std::snprintf(sbuf, sizeof(sbuf), "%" PRId32, frame.score);
        const int sdigits = slen > 4 ? 4 : (slen < 1 ? 1 : slen);
        const int stotal = sdigits * 8 - 2;
        draw_number(pixels, width, y0, rows, frame.score,
                    120 - stotal / 2, 82, 2, kScoreColor, 1);

        // Table separators.
        draw_rect(pixels, width, y0, rows, 56, 113, 184, 114, kSeparator);
        draw_rect(pixels, width, y0, rows, 56, 151, 184, 152, kSeparator);

        // Top-5 table.
        for (int hi = 0; hi < kMaxHighscores; ++hi) {
            if (frame.top_scores[hi] == 0) {
                break;
            }
            const int sy = 118 + hi * 6;
            draw_digit(pixels, width, y0, rows, hi + 1, 64, sy, 1, kBestColor);
            draw_rect(pixels, width, y0, rows, 78, sy + 2, 82, sy + 3,
                      kSeparator);
            draw_number(pixels, width, y0, rows, frame.top_scores[hi],
                        86, sy, 1, kScoreColor, 1);
        }

        // Tap-to-retry affordance (chevron pointing down at the button).
        draw_rect(pixels, width, y0, rows, 74, 168, 166, 170, kSeparator);
        draw_rect(pixels, width, y0, rows, 116, 172, 124, 176,
                  frame.tick % 16u < 8u ? kScoreColor : kMuted);
    }
}

bool RagdollAvalancheApp::render(DisplayFrame &frame)
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

    const AvalancheFrame *av = frames_.acquire_latest();
    if (av == nullptr) {
        return false;
    }

    const int64_t frame_start = esp_timer_get_time();
    uint32_t raster_total = 0;
    esp_err_t result = frame.ops.wait_previous(frame.transport);
    if (result == ESP_OK) {
        static_cast<void>(frame.ops.latch_capture(frame.transport));
        for (int stripe = 0; stripe < frame.stripe_count; ++stripe) {
            const int stripe_y = stripe * frame.stripe_rows;
            const int stripe_rows = min_int(frame.stripe_rows,
                                            frame.height - stripe_y);
            if (stripe_rows <= 0) {
                break;
            }
            uint16_t *pixels = frame.stripe[stripe & 1];
            const int64_t raster_start = esp_timer_get_time();
            raster_stripe(*av, pixels, frame.width, stripe_y, stripe_rows);
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

    frames_.release(av);
    if (result != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s", esp_err_to_name(result));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start);
    raster_us_ = raster_total + frame.ops.capture_copy_us(frame.transport);
    return true;
}

}  // namespace fluid_demo
