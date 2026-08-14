#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "frame_exchange.hpp"
#include "motion.hpp"

namespace fluid_demo {

class RagdollAvalancheApp final : public App {
  public:
    RagdollAvalancheApp() = default;
    ~RagdollAvalancheApp() override = default;
    RagdollAvalancheApp(const RagdollAvalancheApp &) = delete;
    RagdollAvalancheApp &operator=(const RagdollAvalancheApp &) = delete;

    esp_err_t setup_once() override;
    esp_err_t enter() override;
    bool on_motion(const MotionTick &tick) override;
    void on_touch_begin(const TouchEvent &event) override;
    const LauncherVisual *launcher_visual() const override;
    void on_plus_press() override;
    esp_err_t update(float dt = kPhysicsDt) override;
    bool render(DisplayFrame &frame) override;
    AppStats stats() override;
    void leave() override;

  private:
    static constexpr int kMaxSpikes = 24;
    static constexpr int kPanelWidth = 240;
    static constexpr int kPanelHeight = 240;
    static constexpr float kHeadRadius = 7.0f;
    static constexpr float kTorsoHalfLength = 11.0f;
    static constexpr float kUpperArmLength = 13.0f;
    static constexpr float kForearmLength = 13.0f;
    static constexpr float kThighLength = 15.0f;
    static constexpr float kShinLength = 16.0f;
    static constexpr int kTorsoRadius = 4;
    static constexpr int kLimbRadius = 3;
    static constexpr float kPixelsPerSecondSquaredPerAccelerationUnit = 42.0f;
    static constexpr float kVelocityDampingPerSubstep = 0.976f;
    static constexpr float kMaximumPlayerSpeed = 210.0f;
    static constexpr float kMaximumAccelerationInput = 15.0f;
    static constexpr float kLethalRadius = 7.5f;

    static constexpr float kInitialSpikeSpeed = 125.0f;
    static constexpr float kSpikeSpeedRampPerSecond = 1.2f;
    static constexpr float kMaximumSpikeSpeed = 260.0f;
    static constexpr float kSpikeSpeedVariation = 40.0f;
    static constexpr float kSpikeSpawnY = -6.0f;
    static constexpr float kSpikeExitY = 270.0f;

    static constexpr float kWaveIntervalInitial = 1.35f;
    static constexpr float kMinimumWaveInterval = 0.42f;
    static constexpr float kWaveIntervalTimeRamp = 0.010f;
    static constexpr float kWaveCountRampSeconds = 24.0f;
    static constexpr int kWaveInitialCount = 2;
    static constexpr int kWaveMaxCount = 5;
    static constexpr float kMinimumWaveSpacing = 30.0f;
    static constexpr float kFirstWaveClearance = 50.0f;

    static constexpr int kHighScoreCount = 5;

    enum JointIndex : std::size_t {
        LeftUpperArm = 0,
        LeftForearm,
        RightUpperArm,
        RightForearm,
        LeftThigh,
        LeftShin,
        RightThigh,
        RightShin,
        JointCount,
    };

    struct RagdollPose {
        float body_angle = 0.0f;
        std::array<float, JointCount> joint_angles{};
    };

    struct RagdollPoint {
        float x = 0.0f;
        float y = 0.0f;
    };

    struct LimbGeometry {
        RagdollPoint anchor{};
        RagdollPoint joint{};
        RagdollPoint end{};
    };

    struct RagdollGeometry {
        RagdollPoint shoulder{};
        RagdollPoint hip{};
        RagdollPoint head{};
        RagdollPoint upper_chest{};
        std::array<LimbGeometry, 4> limbs{};
        float min_x = 0.0f;
        float max_x = 0.0f;
        float min_y = 0.0f;
        float max_y = 0.0f;
    };

    struct SpikeState {
        float x = 0.0f;
        float y = 0.0f;
        float velocity_y = 0.0f;
        bool active = false;
    };

    struct SpikeRender {
        float x = 0.0f;
        float y = 0.0f;
        bool active = false;
    };

    struct AvalancheFrame {
        uint32_t animation_tick = 0;
        int32_t score = 0;
        int32_t best_score = 0;
        std::array<int32_t, kHighScoreCount> top_scores{};
        bool game_over = false;
        float player_x = 120.0f;
        float player_y = 120.0f;
        RagdollPose pose{};
        std::array<SpikeRender, kMaxSpikes> spikes{};
    };

    struct SharedMotion {
        Vec3 apparent_acceleration{0.0f, 0.0f, 6.0f};
        Vec3 raw_acceleration{0.0f, 0.0f, 0.0f};
        bool available = false;
    };

    void spawn_wave();
    void kill_player();
    void reset_game();
    void step_substep();
    void reset_ragdoll_pose();
    void step_ragdoll_pose(float acceleration_x, float acceleration_y,
                           bool is_dead);
    static RagdollGeometry calculate_ragdoll_geometry(float center_x,
                                                      float center_y,
                                                      const RagdollPose &pose);
    void load_high_scores();
    void save_high_score(int32_t score);

    static void draw_spike(uint16_t *pixels, int width, int stripe_y,
                           int stripe_rows, float x, float tip_y,
                           uint16_t shaft_color, uint16_t tip_color);
    static void draw_ragdoll(uint16_t *pixels, int width, int stripe_y,
                             int stripe_rows, float x, float y,
                             const RagdollPose &pose, uint16_t body_color,
                             uint16_t head_color);
    static void draw_digit(uint16_t *pixels, int width, int stripe_y,
                           int stripe_rows, int digit, int x, int y, int scale,
                           uint16_t color);
    static void draw_number(uint16_t *pixels, int width, int stripe_y,
                            int stripe_rows, int32_t value, int x, int y,
                            int scale, uint16_t color, int min_digits);
    static void raster_stripe(const AvalancheFrame &frame, uint16_t *pixels,
                              int width, int stripe_y, int stripe_rows);

    MotionFilter motion_filter_;
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_{};

    float player_x_ = 120.0f;
    float player_y_ = 120.0f;
    float player_velocity_x_ = 0.0f;
    float player_velocity_y_ = 0.0f;
    RagdollPose ragdoll_pose_{};
    std::array<float, JointCount> joint_velocities_{};
    float body_angular_velocity_ = 0.0f;
    float death_flop_direction_ = 1.0f;
    std::array<SpikeState, kMaxSpikes> spikes_{};
    float survival_time_ = 0.0f;
    float wave_timer_ = 0.0f;
    float wave_interval_ = kWaveIntervalInitial;
    bool first_wave_pending_ = true;
    int32_t score_ = 0;
    uint32_t animation_tick_ = 0;
    uint32_t epoch_ = 0;
    std::atomic<bool> game_over_{false};
    std::atomic<bool> reset_requested_{false};

    int32_t best_score_ = 0;
    std::array<int32_t, kHighScoreCount> top_scores_{};

    LatestFrameExchange<AvalancheFrame, 3> frames_;

    std::atomic<uint32_t> published_epoch_{0};
    std::atomic<uint32_t> active_count_{0};
    std::atomic<uint32_t> physics_us_{0};
    std::atomic<uint32_t> nonfinite_resets_{0};
    uint32_t raster_us_ = 0;
    uint32_t frame_us_ = 0;
    bool setup_done_ = false;
    bool nvs_initialized_ = false;
};

extern RagdollAvalancheApp s_ragdoll_avalanche_app;

}
