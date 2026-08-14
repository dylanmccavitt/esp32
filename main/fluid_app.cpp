#include "fluid_app.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "common_math.hpp"
#include "draw.hpp"
#include "launcher.hpp"
#include "launcher_icons.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "fluid_demo";

// Raw particle motion uses Q8.8 pixels per fixed frame.
constexpr int kAccelerationRawPerInputUnit = 64;
constexpr int kMaxAccelerationRaw = 1024;
constexpr float kEnterFlatMagnitude = 0.45f;
constexpr float kExitFlatMagnitude = 0.75f;
constexpr float kMinFlatFaceMagnitude = 0.6f;
constexpr float kMinValidAccelerationMagnitude = 0.6f;
constexpr float kMinAccelerationUnits = 9.0f;
constexpr int kVelocityDragShift = 7;
constexpr int kMaxVelocityRaw = 4096;
constexpr int kMaxWalkSteps = 18;
constexpr uint32_t kStepGovernorThreshold =
    (3000u * (kMaxWalkSteps + 4) * 3u) / 4u;
constexpr int kGovernorMaxDisplacementRaw = 1024;
constexpr int kGovernorStepLimit = 6;
constexpr int kWallRestitutionNumerator = 153;
constexpr int kFloorFrictionShift = 3;
constexpr int kSideFrictionShift = 5;
constexpr int kWallJitterRaw = 64;
constexpr int kBounceStopVelocityRaw = 90;
constexpr int kParticleTangentialDampingNumerator = 230;
constexpr int kKickTriggerRaw = 512;
constexpr int kKickUpRaw = 640;
constexpr int kKickLateralRaw = 230;
constexpr int kMaxSlideSpeedRaw = 205;
constexpr int kSleepSpeedThresholdRaw = 90;
constexpr int kSleepFrames = 10;
constexpr float kWakeDirectionCosineThreshold = 0.966f;
constexpr uint32_t kRestGateFrameCount = 15;
constexpr float kGravityOctantThreshold = 0.414f;
constexpr int kInverseSqrtTwoQ8 = 181;
constexpr int kSlideVelocityRaw = 180;
constexpr int kMaxSlideHops = 2;
constexpr int kGlowLevelThresholdsRaw[7] = {
    90, 205, 448, 832, 1408, 2304, 3584,
};
constexpr uint16_t kWallColor = 0x31A6;
constexpr uint8_t kFluidRgb[6][3] = {
    {0x18, 0x52, 0x68}, {0x1f, 0x6f, 0x86}, {0x2b, 0x8f, 0xa6},
    {0x43, 0xb3, 0xc4}, {0x70, 0xd2, 0xd8}, {0xa8, 0xec, 0xe8},
};
constexpr uint8_t kGlowRampRgb[8][3] = {
    {0x00, 0x00, 0x00}, {0xd8, 0xff, 0xf2}, {0x9f, 0xf4, 0xe5},
    {0xff, 0xe0, 0x58}, {0xff, 0xb8, 0x2e}, {0xff, 0x6a, 0x10},
    {0xff, 0x3d, 0x0c}, {0xff, 0x1a, 0x08},
};
constexpr uint8_t kGlowRampWeight[8] = {
    0, 90, 140, 180, 210, 235, 250, 255,
};
constexpr int32_t kMinPositionRaw = 256;
constexpr int32_t kMaxPositionRaw = 61183;
constexpr size_t kParticleArenaBytesPerParticle =
    sizeof(uint16_t) * 2 + sizeof(int16_t) * 2 + sizeof(uint8_t) * 2;

inline int16_t saturate_i16(int32_t value)
{
    return static_cast<int16_t>(clamp_i32(value, -32768, 32767));
}

constexpr LauncherVisual kLauncherVisual{
    kLauncherBackgroundRgb565, kLauncherBandRgb565, kLauncherAffordanceRgb565,
    kLauncherAccentRgb565,     kIconFluid,
};

}

FluidBoxApp s_fluid_app;

FluidBoxApp::~FluidBoxApp()
{
    heap_caps_free(particle_arena_);
    heap_caps_free(occupancy_grid_);
    particle_arena_ = nullptr;
    occupancy_grid_ = nullptr;
}

uint32_t FluidBoxApp::next_random()
{
    random_state_ ^= random_state_ << 13;
    random_state_ ^= random_state_ >> 17;
    random_state_ ^= random_state_ << 5;
    return random_state_;
}

esp_err_t FluidBoxApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;
    }

    const size_t arena_bytes =
        static_cast<size_t>(kParticleCount) * kParticleArenaBytesPerParticle;
    uint8_t *particle_arena = static_cast<uint8_t *>(
        heap_caps_malloc(arena_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    uint8_t *occupancy_grid = static_cast<uint8_t *>(
        heap_caps_malloc(static_cast<size_t>(kGridWidth) * kGridHeight,
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (particle_arena == nullptr || occupancy_grid == nullptr) {
        heap_caps_free(particle_arena);
        heap_caps_free(occupancy_grid);
        ESP_LOGE(kTag, "particle allocation failed");
        return ESP_ERR_NO_MEM;
    }
    particle_arena_ = particle_arena;
    particle_x_ = reinterpret_cast<uint16_t *>(particle_arena);
    particle_y_ = particle_x_ + kParticleCount;
    particle_velocity_x_ =
        reinterpret_cast<int16_t *>(particle_y_ + kParticleCount);
    particle_velocity_y_ = particle_velocity_x_ + kParticleCount;
    particle_state_ =
        reinterpret_cast<uint8_t *>(particle_velocity_y_ + kParticleCount);
    particle_rest_frames_ = particle_state_ + kParticleCount;
    occupancy_grid_ = occupancy_grid;

    for (int glow_level = 0; glow_level < kGlowLevelCount; ++glow_level) {
        const int blend_weight = kGlowRampWeight[glow_level];
        for (int shade = 0; shade < kShadeCount; ++shade) {
            const int red =
                kFluidRgb[shade][0] +
                ((kGlowRampRgb[glow_level][0] - kFluidRgb[shade][0]) *
                 blend_weight) /
                    255;
            const int green =
                kFluidRgb[shade][1] +
                ((kGlowRampRgb[glow_level][1] - kFluidRgb[shade][1]) *
                 blend_weight) /
                    255;
            const int blue =
                kFluidRgb[shade][2] +
                ((kGlowRampRgb[glow_level][2] - kFluidRgb[shade][2]) *
                 blend_weight) /
                    255;
            wire_palette_[(glow_level << 3) | (shade + 1)] = __builtin_bswap16(
                rgb565(static_cast<uint8_t>(red), static_cast<uint8_t>(green),
                       static_cast<uint8_t>(blue)));
        }
    }

    reset_particles();
    setup_done_ = true;
    ESP_LOGI(kTag, "specks initialized: %d ballistic particles",
             kParticleCount);
    return ESP_OK;
}

void FluidBoxApp::reset_particles()
{
    random_state_ = 0x2545F491u;
    for (int particle_index = 0; particle_index < kParticleCount;
         ++particle_index) {
        const int cell_x = 1 + (particle_index % (kGridWidth - 2));
        const int cell_y =
            (kGridHeight - 2) - (particle_index / (kGridWidth - 2));
        particle_x_[particle_index] =
            static_cast<uint16_t>((cell_x << 8) | 128);
        particle_y_[particle_index] =
            static_cast<uint16_t>((cell_y << 8) | 128);
        particle_velocity_x_[particle_index] = 0;
        particle_velocity_y_[particle_index] = 0;
        particle_state_[particle_index] =
            static_cast<uint8_t>((next_random() % 6) | kStateAsleep);
        particle_rest_frames_[particle_index] = kSleepFrames;
    }
    std::memset(occupancy_grid_, 0,
                static_cast<size_t>(kGridWidth) * kGridHeight);
    for (int particle_index = 0; particle_index < kParticleCount;
         ++particle_index) {
        const size_t cell_index =
            static_cast<size_t>(particle_y_[particle_index] >> 8) * kGridWidth +
            (particle_x_[particle_index] >> 8);
        occupancy_grid_[cell_index] = static_cast<uint8_t>(
            (particle_state_[particle_index] & kStateShadeMask) + 1);
    }
    awake_count_ = 0;
    rest_gate_frames_ = 0;
    frame_parity_ = 0;
    gravity_ = {};
    epoch_.fetch_add(1, std::memory_order_relaxed);
}

esp_err_t FluidBoxApp::enter()
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    frames_.drain();
    return ESP_OK;
}

void FluidBoxApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.available = false;
    portEXIT_CRITICAL(&motion_mux_);
}

void FluidBoxApp::on_plus_press()
{
    reset_requested_.store(true, std::memory_order_release);
}

const LauncherVisual *FluidBoxApp::launcher_visual() const
{
    return &kLauncherVisual;
}

bool FluidBoxApp::on_motion(const MotionTick &tick)
{
    if (!tick.fresh) {
        if (tick.override_active) {
            portENTER_CRITICAL(&motion_mux_);
            motion_.apparent_acceleration = tick.apparent_accel;
            motion_.available = true;
            portEXIT_CRITICAL(&motion_mux_);
        }
        return false;
    }

    const Vec3 filtered_acceleration =
        motion_filter_.update(tick.accel_mps2, tick.dt);
    const bool sample_accepted = motion_filter_.last_sample_accepted();
    if (!sample_accepted) {
        if (tick.override_active) {
            portENTER_CRITICAL(&motion_mux_);
            motion_.apparent_acceleration = tick.apparent_accel;
            motion_.available = true;
            portEXIT_CRITICAL(&motion_mux_);
        }
        return false;
    }
    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent_acceleration =
        tick.override_active ? tick.apparent_accel : filtered_acceleration;
    motion_.raw_acceleration = tick.accel_mps2;
    motion_.available = true;
    portEXIT_CRITICAL(&motion_mux_);
    return true;
}

esp_err_t FluidBoxApp::update(float dt)
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
        reset_particles();
        ESP_LOGI(kTag, "PLUS press - specks reset (epoch %u)",
                 static_cast<unsigned>(epoch_.load(std::memory_order_relaxed)));
    } else {
        Vec3 apparent_acceleration{0.0f, 0.0f, 9.0f};
        portENTER_CRITICAL(&motion_mux_);
        if (motion_.available) {
            apparent_acceleration = motion_.apparent_acceleration;
        }
        portEXIT_CRITICAL(&motion_mux_);

        const uint32_t walk_steps =
            step_particles(apparent_acceleration.x, -apparent_acceleration.y,
                           apparent_acceleration.z);
        walk_steps_.fetch_add(walk_steps, std::memory_order_relaxed);
    }

    FluidFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        fill_frame(*snapshot);
        frames_.publish(snapshot);
    }
    physics_us_.store(
        static_cast<uint32_t>(esp_timer_get_time() - update_start_us),
        std::memory_order_relaxed);
    return ESP_OK;
}

FluidBoxApp::GravityState::ResolvedGravity FluidBoxApp::GravityState::resolve(
    float screen_gravity_x, float screen_gravity_y, float screen_gravity_z)
{
    float acceleration_units = kMinAccelerationUnits;
    const bool input_finite = std::isfinite(screen_gravity_x) &&
                              std::isfinite(screen_gravity_y) &&
                              std::isfinite(screen_gravity_z);
    if (input_finite) {
        const float in_plane_squared = screen_gravity_x * screen_gravity_x +
                                       screen_gravity_y * screen_gravity_y;
        const float total_squared =
            in_plane_squared + screen_gravity_z * screen_gravity_z;
        const bool magnitude_valid =
            std::isfinite(total_squared) &&
            total_squared >=
                kMinValidAccelerationMagnitude * kMinValidAccelerationMagnitude;
        if (magnitude_valid) {
            const float in_plane_magnitude = std::sqrt(in_plane_squared);
            const float absolute_gravity_z = std::fabs(screen_gravity_z);
            bool use_in_plane_tilt = false;

            if (mode == Mode::Flat) {
                if (in_plane_magnitude >= kExitFlatMagnitude) {
                    mode = Mode::Tilted;
                    use_in_plane_tilt = true;
                } else if (absolute_gravity_z >= kMinFlatFaceMagnitude) {
                    direction_x = 0.0f;
                    direction_y = screen_gravity_z >= 0.0f ? 1.0f : -1.0f;
                }
            } else if (in_plane_magnitude <= kEnterFlatMagnitude) {
                if (absolute_gravity_z >= kMinFlatFaceMagnitude) {
                    mode = Mode::Flat;
                    direction_x = 0.0f;
                    direction_y = screen_gravity_z >= 0.0f ? 1.0f : -1.0f;
                }
            } else {
                use_in_plane_tilt = true;
            }

            if (use_in_plane_tilt) {
                const float inverse_magnitude = 1.0f / in_plane_magnitude;
                direction_x = screen_gravity_x * inverse_magnitude;
                direction_y = screen_gravity_y * inverse_magnitude;
                acceleration_units = in_plane_magnitude > kMinAccelerationUnits
                                         ? in_plane_magnitude
                                         : kMinAccelerationUnits;
                const float max_acceleration_units =
                    static_cast<float>(kMaxAccelerationRaw) /
                    kAccelerationRawPerInputUnit;
                if (acceleration_units > max_acceleration_units) {
                    acceleration_units = max_acceleration_units;
                }
            }
        }
    }

    const bool wake_all =
        direction_x * wake_anchor_x + direction_y * wake_anchor_y <
        kWakeDirectionCosineThreshold;
    if (wake_all) {
        wake_anchor_x = direction_x;
        wake_anchor_y = direction_y;
    }
    return {
        direction_x,
        direction_y,
        acceleration_units,
        wake_all,
    };
}

uint32_t FluidBoxApp::step_particles(float screen_gravity_x,
                                     float screen_gravity_y,
                                     float screen_gravity_z)
{
    const GravityState::ResolvedGravity gravity =
        gravity_.resolve(screen_gravity_x, screen_gravity_y, screen_gravity_z);
    const float acceleration_raw =
        gravity.acceleration_units *
        static_cast<float>(kAccelerationRawPerInputUnit);
    const int32_t velocity_delta_x = clamp_i32(
        static_cast<int32_t>(lroundf(gravity.direction_x * acceleration_raw)),
        -kMaxAccelerationRaw, kMaxAccelerationRaw);
    const int32_t velocity_delta_y = clamp_i32(
        static_cast<int32_t>(lroundf(gravity.direction_y * acceleration_raw)),
        -kMaxAccelerationRaw, kMaxAccelerationRaw);

    int gravity_step_x = 0;
    int gravity_step_y = 0;
    const float absolute_gravity_x = std::fabs(gravity.direction_x);
    const float absolute_gravity_y = std::fabs(gravity.direction_y);
    if (absolute_gravity_x > kGravityOctantThreshold * absolute_gravity_y) {
        gravity_step_x = gravity.direction_x >= 0.0f ? 1 : -1;
    }
    if (absolute_gravity_y > kGravityOctantThreshold * absolute_gravity_x) {
        gravity_step_y = gravity.direction_y >= 0.0f ? 1 : -1;
    }
    const bool horizontal_gravity_dominant =
        absolute_gravity_x >= absolute_gravity_y;

    if (gravity.wake_all) {
        for (int particle_index = 0; particle_index < kParticleCount;
             ++particle_index) {
            particle_state_[particle_index] &=
                static_cast<uint8_t>(~kStateAsleep);
            particle_rest_frames_[particle_index] = 0;
        }
        awake_count_ = kParticleCount;
    }

    if (!gravity.wake_all && awake_count_ == 0) {
        if (rest_gate_frames_ < kRestGateFrameCount) {
            ++rest_gate_frames_;
        }
    } else {
        rest_gate_frames_ = 0;
    }
    if (rest_gate_frames_ >= kRestGateFrameCount) {
        return 0;
    }

    uint8_t *occupancy_grid = occupancy_grid_;
    const bool diagonal_gravity = gravity_step_x != 0 && gravity_step_y != 0;
    const int32_t opposing_kick_raw =
        diagonal_gravity ? (kKickUpRaw * kInverseSqrtTwoQ8) >> 8 : kKickUpRaw;
    const int32_t lateral_kick_raw =
        diagonal_gravity ? (kKickLateralRaw * kInverseSqrtTwoQ8) >> 8
                         : kKickLateralRaw;

    for (int particle_index = 0; particle_index < kParticleCount;
         ++particle_index) {
        const size_t cell_index =
            static_cast<size_t>(particle_y_[particle_index] >> 8) * kGridWidth +
            (particle_x_[particle_index] >> 8);
        const uint32_t kick_count =
            static_cast<uint32_t>(occupancy_grid[cell_index]) >> kGridKickShift;
        if (kick_count == 0u) {
            continue;
        }
        if (kick_count < kWakeOnlyKickCount) {
            const int lateral_sign = (next_random() & 1u) != 0u ? 1 : -1;
            particle_velocity_x_[particle_index] = saturate_i16(
                particle_velocity_x_[particle_index] -
                gravity_step_x * static_cast<int32_t>(kick_count) *
                    opposing_kick_raw -
                gravity_step_y * lateral_sign *
                    static_cast<int32_t>(kick_count) * lateral_kick_raw);
            particle_velocity_y_[particle_index] = saturate_i16(
                particle_velocity_y_[particle_index] -
                gravity_step_y * static_cast<int32_t>(kick_count) *
                    opposing_kick_raw +
                gravity_step_x * lateral_sign *
                    static_cast<int32_t>(kick_count) * lateral_kick_raw);
        }
        particle_state_[particle_index] &= static_cast<uint8_t>(~kStateAsleep);
        particle_rest_frames_[particle_index] = 0;
    }

    std::memset(occupancy_grid, 0,
                static_cast<size_t>(kGridWidth) * kGridHeight);
    for (int particle_index = 0; particle_index < kParticleCount;
         ++particle_index) {
        const size_t cell_index =
            static_cast<size_t>(particle_y_[particle_index] >> 8) * kGridWidth +
            (particle_x_[particle_index] >> 8);
        if (occupancy_grid[cell_index] == 0u) {
            occupancy_grid[cell_index] = static_cast<uint8_t>(
                (particle_state_[particle_index] & kStateGlowMask) |
                ((particle_state_[particle_index] & kStateShadeMask) + 1));
        } else {
            const uint32_t random_bits = next_random();
            particle_velocity_x_[particle_index] =
                saturate_i16(particle_velocity_x_[particle_index] +
                             ((random_bits & 1u) != 0u ? 256 : -256));
            particle_velocity_y_[particle_index] =
                saturate_i16(particle_velocity_y_[particle_index] +
                             ((random_bits & 2u) != 0u ? 256 : -256));
            particle_state_[particle_index] = static_cast<uint8_t>(
                (particle_state_[particle_index] & ~kStateAsleep) |
                kStateOverlap);
            particle_rest_frames_[particle_index] = 0;
        }
    }

    uint32_t walk_steps = 0;
    uint32_t awake_particles = 0;
    const bool forward_sweep = ((frame_parity_++ & 1u) == 0u);
    for (int sweep_index = 0; sweep_index < kParticleCount; ++sweep_index) {
        const int particle_index =
            forward_sweep ? sweep_index : kParticleCount - 1 - sweep_index;
        uint8_t state = particle_state_[particle_index];
        if ((state & kStateAsleep) != 0u) {
            continue;
        }

        int32_t velocity_x =
            particle_velocity_x_[particle_index] + velocity_delta_x;
        int32_t velocity_y =
            particle_velocity_y_[particle_index] + velocity_delta_y;
        velocity_x -= velocity_x >> kVelocityDragShift;
        velocity_y -= velocity_y >> kVelocityDragShift;
        velocity_x = clamp_i32(velocity_x, -kMaxVelocityRaw, kMaxVelocityRaw);
        velocity_y = clamp_i32(velocity_y, -kMaxVelocityRaw, kMaxVelocityRaw);

        int32_t position_x = particle_x_[particle_index];
        int32_t position_y = particle_y_[particle_index];
        int cell_x = position_x >> 8;
        int cell_y = position_y >> 8;
        const int start_cell_x = cell_x;
        const int start_cell_y = cell_y;
        const size_t start_cell_index =
            static_cast<size_t>(cell_y) * kGridWidth + cell_x;
        const bool overlap_loser = (state & kStateOverlap) != 0u;
        state &= static_cast<uint8_t>(~kStateOverlap);
        uint8_t pending_kick_bits = 0;
        if (!overlap_loser) {
            pending_kick_bits = static_cast<uint8_t>(
                occupancy_grid[start_cell_index] & kGridKickMask);
            occupancy_grid[start_cell_index] = 0;
        }

        int32_t remaining_x = velocity_x;
        int32_t remaining_y = velocity_y;
        int walk_step_budget = kMaxWalkSteps + 4;
        if (walk_steps > kStepGovernorThreshold) {
            governor_hits_.fetch_add(1, std::memory_order_relaxed);
            remaining_x = clamp_i32(remaining_x, -kGovernorMaxDisplacementRaw,
                                    kGovernorMaxDisplacementRaw);
            remaining_y = clamp_i32(remaining_y, -kGovernorMaxDisplacementRaw,
                                    kGovernorMaxDisplacementRaw);
            walk_step_budget = kGovernorStepLimit;
        }

        // A bounce changes the remaining displacement and invalidates this
        // step.
        int32_t substeps_remaining = 0;
        int32_t step_x = 0;
        int32_t step_y = 0;
        while ((remaining_x != 0 || remaining_y != 0) &&
               walk_step_budget-- > 0) {
            if (substeps_remaining <= 0) {
                const int32_t absolute_remaining_x = abs_int(remaining_x);
                const int32_t absolute_remaining_y = abs_int(remaining_y);
                const int32_t maximum_remaining =
                    absolute_remaining_x > absolute_remaining_y
                        ? absolute_remaining_x
                        : absolute_remaining_y;
                substeps_remaining = (maximum_remaining + 255) >> 8;
                if (substeps_remaining <= 0) {
                    break;
                }
                step_x = remaining_x / substeps_remaining;
                step_y = remaining_y / substeps_remaining;
                if (step_x == 0 && step_y == 0) {
                    break;
                }
            }
            ++walk_steps;
            int32_t next_position_x = position_x + step_x;
            int32_t next_position_y = position_y + step_y;
            int next_cell_x = next_position_x >> 8;
            int next_cell_y = next_position_y >> 8;
            bool bounced_x = false;
            bool bounced_y = false;

            if (next_cell_x != cell_x) {
                bool blocked = false;
                bool hit_wall = false;
                if (next_cell_x < 1 || next_cell_x > kGridWidth - 2) {
                    blocked = true;
                    hit_wall = true;
                } else if (occupancy_grid[static_cast<size_t>(cell_y) *
                                              kGridWidth +
                                          next_cell_x] != 0u) {
                    blocked = true;
                }
                if (blocked) {
                    if (hit_wall) {
                        velocity_x =
                            -(velocity_x * kWallRestitutionNumerator) >> 8;
                        remaining_x =
                            -(remaining_x * kWallRestitutionNumerator) >> 8;
                        const int friction_shift = horizontal_gravity_dominant
                                                       ? kFloorFrictionShift
                                                       : kSideFrictionShift;
                        velocity_y -= velocity_y >> friction_shift;
                        velocity_y +=
                            static_cast<int32_t>(next_random() %
                                                 (2 * kWallJitterRaw + 1)) -
                            kWallJitterRaw;
                        if (abs_int(velocity_x) < kBounceStopVelocityRaw) {
                            velocity_x = 0;
                            remaining_x = 0;
                        }
                    } else {
                        const int support_cell_x = next_cell_x + gravity_step_x;
                        const int support_cell_y = cell_y + gravity_step_y;
                        const bool target_supported =
                            support_cell_x < 1 ||
                            support_cell_x > kGridWidth - 2 ||
                            support_cell_y < 1 ||
                            support_cell_y > kGridHeight - 2 ||
                            occupancy_grid[static_cast<size_t>(support_cell_y) *
                                               kGridWidth +
                                           support_cell_x] != 0u;
                        if (target_supported &&
                            abs_int(velocity_x) >= kKickTriggerRaw) {
                            uint8_t &target_cell =
                                occupancy_grid[static_cast<size_t>(cell_y) *
                                                   kGridWidth +
                                               next_cell_x];
                            if ((target_cell >> kGridKickShift) <
                                kMaxImpactKickCount) {
                                target_cell = static_cast<uint8_t>(
                                    target_cell + kGridKickIncrement);
                            }
                        }
                        velocity_x >>= target_supported ? 2 : 1;
                        remaining_x = 0;
                        velocity_y = (velocity_y *
                                      kParticleTangentialDampingNumerator) >>
                                     8;
                    }
                    next_position_x =
                        step_x > 0 ? (cell_x << 8) | 0xFF : cell_x << 8;
                    next_cell_x = cell_x;
                    bounced_x = true;
                }
            }
            if (next_cell_y != cell_y) {
                bool blocked = false;
                bool hit_wall = false;
                if (next_cell_y < 1 || next_cell_y > kGridHeight - 2) {
                    blocked = true;
                    hit_wall = true;
                } else if (occupancy_grid[static_cast<size_t>(next_cell_y) *
                                              kGridWidth +
                                          next_cell_x] != 0u) {
                    blocked = true;
                }
                if (blocked) {
                    if (hit_wall) {
                        velocity_y =
                            -(velocity_y * kWallRestitutionNumerator) >> 8;
                        remaining_y =
                            -(remaining_y * kWallRestitutionNumerator) >> 8;
                        const int friction_shift = horizontal_gravity_dominant
                                                       ? kSideFrictionShift
                                                       : kFloorFrictionShift;
                        velocity_x -= velocity_x >> friction_shift;
                        velocity_x +=
                            static_cast<int32_t>(next_random() %
                                                 (2 * kWallJitterRaw + 1)) -
                            kWallJitterRaw;
                        if (abs_int(velocity_y) < kBounceStopVelocityRaw) {
                            velocity_y = 0;
                            remaining_y = 0;
                        }
                    } else {
                        const int support_cell_x = next_cell_x + gravity_step_x;
                        const int support_cell_y = next_cell_y + gravity_step_y;
                        const bool target_supported =
                            support_cell_x < 1 ||
                            support_cell_x > kGridWidth - 2 ||
                            support_cell_y < 1 ||
                            support_cell_y > kGridHeight - 2 ||
                            occupancy_grid[static_cast<size_t>(support_cell_y) *
                                               kGridWidth +
                                           support_cell_x] != 0u;
                        if (target_supported &&
                            abs_int(velocity_y) >= kKickTriggerRaw) {
                            uint8_t &target_cell = occupancy_grid
                                [static_cast<size_t>(next_cell_y) * kGridWidth +
                                 next_cell_x];
                            if ((target_cell >> kGridKickShift) <
                                kMaxImpactKickCount) {
                                target_cell = static_cast<uint8_t>(
                                    target_cell + kGridKickIncrement);
                            }
                        }
                        velocity_y >>= target_supported ? 2 : 1;
                        remaining_y = 0;
                        velocity_x = (velocity_x *
                                      kParticleTangentialDampingNumerator) >>
                                     8;
                    }
                    next_position_y =
                        step_y > 0 ? (cell_y << 8) | 0xFF : cell_y << 8;
                    next_cell_y = cell_y;
                    bounced_y = true;
                }
            }

            position_x = next_position_x;
            position_y = next_position_y;
            cell_x = next_cell_x;
            cell_y = next_cell_y;
            if (!bounced_x) {
                remaining_x -= step_x;
            }
            if (!bounced_y) {
                remaining_y -= step_y;
            }
            if (bounced_x || bounced_y) {
                substeps_remaining = 0;
            } else {
                --substeps_remaining;
            }
        }

        position_x = clamp_i32(position_x, kMinPositionRaw, kMaxPositionRaw);
        position_y = clamp_i32(position_y, kMinPositionRaw, kMaxPositionRaw);
        cell_x = position_x >> 8;
        cell_y = position_y >> 8;

        int32_t speed_raw = abs_int(velocity_x) + abs_int(velocity_y);
        const int down_cell_x = cell_x + gravity_step_x;
        const int down_cell_y = cell_y + gravity_step_y;
        const bool down_blocked =
            down_cell_x < 1 || down_cell_x > kGridWidth - 2 ||
            down_cell_y < 1 || down_cell_y > kGridHeight - 2 ||
            occupancy_grid[static_cast<size_t>(down_cell_y) * kGridWidth +
                           down_cell_x] != 0u;
        bool supported = down_blocked;
        bool leveled = false;

        if (speed_raw < kMaxSlideSpeedRaw && down_blocked &&
            (gravity_step_x | gravity_step_y) != 0) {
            int first_slide_x;
            int first_slide_y;
            int second_slide_x;
            int second_slide_y;
            if (gravity_step_x != 0 && gravity_step_y != 0) {
                first_slide_x = gravity_step_x;
                first_slide_y = 0;
                second_slide_x = 0;
                second_slide_y = gravity_step_y;
            } else if (gravity_step_x != 0) {
                first_slide_x = gravity_step_x;
                first_slide_y = 1;
                second_slide_x = gravity_step_x;
                second_slide_y = -1;
            } else {
                first_slide_x = 1;
                first_slide_y = gravity_step_y;
                second_slide_x = -1;
                second_slide_y = gravity_step_y;
            }

            int slide_delta_x = 0;
            int slide_delta_y = 0;
            for (int hop_index = 0; hop_index < kMaxSlideHops; ++hop_index) {
                const uint32_t random_bits = next_random();
                const bool try_second_first = (random_bits & 1u) != 0u;
                const int primary_slide_x =
                    try_second_first ? second_slide_x : first_slide_x;
                const int primary_slide_y =
                    try_second_first ? second_slide_y : first_slide_y;
                const int alternate_slide_x =
                    try_second_first ? first_slide_x : second_slide_x;
                const int alternate_slide_y =
                    try_second_first ? first_slide_y : second_slide_y;

                bool moved = false;
                for (int attempt_index = 0; attempt_index < 2 && !moved;
                     ++attempt_index) {
                    const int candidate_x =
                        cell_x + (attempt_index == 0 ? primary_slide_x
                                                     : alternate_slide_x);
                    const int candidate_y =
                        cell_y + (attempt_index == 0 ? primary_slide_y
                                                     : alternate_slide_y);
                    const bool candidate_open =
                        candidate_x >= 1 && candidate_x <= kGridWidth - 2 &&
                        candidate_y >= 1 && candidate_y <= kGridHeight - 2 &&
                        occupancy_grid[static_cast<size_t>(candidate_y) *
                                           kGridWidth +
                                       candidate_x] == 0u;
                    if (!candidate_open) {
                        continue;
                    }

                    slide_delta_x = candidate_x - cell_x;
                    slide_delta_y = candidate_y - cell_y;
                    cell_x = candidate_x;
                    cell_y = candidate_y;
                    position_x = (candidate_x << 8) | (position_x & 0xFF);
                    position_y = (candidate_y << 8) | (position_y & 0xFF);
                    moved = true;
                    leveled = true;
                }

                if (!moved && !leveled && (random_bits & 2u) != 0u) {
                    const int lateral_delta_x =
                        gravity_step_y != 0
                            ? ((random_bits & 4u) != 0u ? 1 : -1)
                            : 0;
                    const int lateral_delta_y =
                        gravity_step_x != 0 && gravity_step_y == 0
                            ? ((random_bits & 4u) != 0u ? 1 : -1)
                            : 0;
                    const int candidate_x = cell_x + lateral_delta_x;
                    const int candidate_y = cell_y + lateral_delta_y;
                    const int candidate_down_x = candidate_x + gravity_step_x;
                    const int candidate_down_y = candidate_y + gravity_step_y;
                    const bool lateral_open =
                        (lateral_delta_x | lateral_delta_y) != 0 &&
                        candidate_x >= 1 && candidate_x <= kGridWidth - 2 &&
                        candidate_y >= 1 && candidate_y <= kGridHeight - 2 &&
                        occupancy_grid[static_cast<size_t>(candidate_y) *
                                           kGridWidth +
                                       candidate_x] == 0u &&
                        candidate_down_x >= 1 &&
                        candidate_down_x <= kGridWidth - 2 &&
                        candidate_down_y >= 1 &&
                        candidate_down_y <= kGridHeight - 2 &&
                        occupancy_grid[static_cast<size_t>(candidate_down_y) *
                                           kGridWidth +
                                       candidate_down_x] == 0u;
                    if (lateral_open) {
                        slide_delta_x = lateral_delta_x;
                        slide_delta_y = lateral_delta_y;
                        cell_x = candidate_x;
                        cell_y = candidate_y;
                        position_x = (candidate_x << 8) | (position_x & 0xFF);
                        position_y = (candidate_y << 8) | (position_y & 0xFF);
                        leveled = true;
                    }
                }

                if (!moved) {
                    break;
                }
            }

            if (leveled) {
                velocity_x = slide_delta_x * kSlideVelocityRaw;
                velocity_y = slide_delta_y * kSlideVelocityRaw;
                particle_rest_frames_[particle_index] = 0;
            }

            const int support_cell_x = cell_x + gravity_step_x;
            const int support_cell_y = cell_y + gravity_step_y;
            supported = support_cell_x < 1 || support_cell_x > kGridWidth - 2 ||
                        support_cell_y < 1 ||
                        support_cell_y > kGridHeight - 2 ||
                        occupancy_grid[static_cast<size_t>(support_cell_y) *
                                           kGridWidth +
                                       support_cell_x] != 0u;
        }
        speed_raw = abs_int(velocity_x) + abs_int(velocity_y);

        int speed_glow_level = 0;
        while (speed_glow_level < kGlowLevelCount - 1 &&
               speed_raw >= kGlowLevelThresholdsRaw[speed_glow_level]) {
            ++speed_glow_level;
        }
        const int previous_glow_level =
            (state & kStateGlowMask) >> kStateGlowShift;
        const int glow_level = speed_glow_level > previous_glow_level - 1
                                   ? speed_glow_level
                                   : previous_glow_level - 1;

        bool fell_asleep = false;
        if (speed_raw < kSleepSpeedThresholdRaw && glow_level == 0 &&
            supported && !leveled) {
            if (particle_rest_frames_[particle_index] < UINT8_MAX) {
                ++particle_rest_frames_[particle_index];
            }
            if (particle_rest_frames_[particle_index] >= kSleepFrames) {
                velocity_x = 0;
                velocity_y = 0;
                state |= kStateAsleep;
                fell_asleep = true;
            }
        } else {
            particle_rest_frames_[particle_index] = 0;
        }

        state = static_cast<uint8_t>(
            (state & ~kStateGlowMask) |
            (static_cast<uint8_t>(glow_level) << kStateGlowShift));
        particle_state_[particle_index] = state;
        particle_velocity_x_[particle_index] = saturate_i16(velocity_x);
        particle_velocity_y_[particle_index] = saturate_i16(velocity_y);
        particle_x_[particle_index] = static_cast<uint16_t>(position_x);
        particle_y_[particle_index] = static_cast<uint16_t>(position_y);
        if (!fell_asleep) {
            ++awake_particles;
        }

        uint8_t &destination_cell =
            occupancy_grid[static_cast<size_t>(cell_y) * kGridWidth + cell_x];
        if (destination_cell == 0u) {
            destination_cell = static_cast<uint8_t>(
                pending_kick_bits |
                (static_cast<uint8_t>(glow_level) << kStateGlowShift) |
                ((state & kStateShadeMask) + 1));
        }

        // Wake a particle that lost support when this cell moved away.
        if (!overlap_loser &&
            (cell_x != start_cell_x || cell_y != start_cell_y) &&
            (gravity_step_x | gravity_step_y) != 0) {
            const int wake_cell_x = start_cell_x - gravity_step_x;
            const int wake_cell_y = start_cell_y - gravity_step_y;
            if ((wake_cell_x != cell_x || wake_cell_y != cell_y) &&
                wake_cell_x >= 1 && wake_cell_x <= kGridWidth - 2 &&
                wake_cell_y >= 1 && wake_cell_y <= kGridHeight - 2) {
                uint8_t &wake_cell =
                    occupancy_grid[static_cast<size_t>(wake_cell_y) *
                                       kGridWidth +
                                   wake_cell_x];
                if (wake_cell != 0u && (wake_cell >> kGridKickShift) == 0u) {
                    wake_cell |= kGridKickMask;
                }
            }
        }
    }

    awake_count_ = awake_particles;
    return walk_steps;
}

void FluidBoxApp::fill_frame(FluidFrame &frame)
{
    frame.count = 0;
    const uint32_t grid_cell_count = kGridWidth * kGridHeight;
    for (uint32_t cell_index = 0; cell_index < grid_cell_count; ++cell_index) {
        const uint8_t color_index =
            occupancy_grid_[cell_index] & kGridColorMask;
        if (color_index == 0u) {
            continue;
        }
        if (frame.count == kParticleCount) {
            assert(false);
            break;
        }

        frame.cells[frame.count] = static_cast<uint16_t>(cell_index);
        frame.colors[frame.count] = color_index;
        ++frame.count;
    }
}

void FluidBoxApp::draw_frame(const FluidFrame &frame, uint16_t *pixels,
                             int stripe_y, int stripe_rows)
{
    const uint16_t wall_color = __builtin_bswap16(kWallColor);
    const int stripe_bottom = stripe_y + stripe_rows;
    for (int screen_y = stripe_y; screen_y < stripe_bottom; ++screen_y) {
        uint16_t *row =
            pixels + static_cast<size_t>(screen_y - stripe_y) * kGridWidth;
        if (screen_y == 0 || screen_y == kGridHeight - 1) {
            for (int screen_x = 0; screen_x < kGridWidth; ++screen_x) {
                row[screen_x] = wall_color;
            }
            continue;
        }

        row[0] = wall_color;
        row[kGridWidth - 1] = wall_color;
    }

    for (uint16_t particle_index = 0; particle_index < frame.count;
         ++particle_index) {
        const int cell_index = frame.cells[particle_index];
        const int screen_y = cell_index / kGridWidth;
        if (screen_y < stripe_y || screen_y >= stripe_bottom) {
            continue;
        }

        pixels[static_cast<size_t>(screen_y - stripe_y) * kGridWidth +
               static_cast<size_t>(cell_index % kGridWidth)] =
            wire_palette_[frame.colors[particle_index]];
    }
}

bool FluidBoxApp::render(DisplayFrame &frame)
{
    if (!setup_done_ || frame.transport == nullptr) {
        ESP_LOGW(kTag, "render failed: %s",
                 esp_err_to_name(ESP_ERR_INVALID_STATE));
        return false;
    }
    const int64_t frame_start_us = esp_timer_get_time();

    const FluidFrame *snapshot = frames_.acquire_latest();
    if (snapshot == nullptr) {
        return false;
    }

    esp_err_t transport_result = frame.ops.wait_previous(frame.transport);
    uint32_t total_raster_us = 0;
    if (transport_result == ESP_OK) {
        frame.ops.latch_capture(frame.transport);
        for (int stripe_index = 0; stripe_index < frame.stripe_count;
             ++stripe_index) {
            const int stripe_y = stripe_index * frame.stripe_rows;
            const int stripe_rows =
                min_int(frame.stripe_rows, frame.height - stripe_y);
            uint16_t *stripe_pixels = frame.stripe[stripe_index & 1];

            const int64_t raster_start_us = esp_timer_get_time();
            std::memset(stripe_pixels, 0,
                        static_cast<size_t>(frame.width) * stripe_rows *
                            sizeof(uint16_t));
            draw_frame(*snapshot, stripe_pixels, stripe_y, stripe_rows);
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
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start_us);
    raster_us_ = total_raster_us + frame.ops.capture_copy_us(frame.transport);
    return true;
}

AppStats FluidBoxApp::stats()
{
    AppStats result{};
    result.count = kParticleCount;
    result.epoch = epoch_.load(std::memory_order_relaxed);
    result.candidate_checks = walk_steps_.load(std::memory_order_relaxed);
    result.governor_hits = governor_hits_.load(std::memory_order_relaxed);
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

}
