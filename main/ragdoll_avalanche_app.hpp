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

/// Ragdoll Avalanche II adaptation: a floppy ragdoll, steered by tilting the
/// board, dodges an unending avalanche of falling spikes. One hit ends the
/// run; the score is the number of spikes dodged. Top-5 scores persist in
/// NVS. Single-producer/single-consumer triple snapshot exchange for
/// lane-safe render.
class RagdollAvalancheApp final : public App {
public:
    RagdollAvalancheApp() = default;
    ~RagdollAvalancheApp() override = default;
    RagdollAvalancheApp(const RagdollAvalancheApp &) = delete;
    RagdollAvalancheApp &operator=(const RagdollAvalancheApp &) = delete;

    esp_err_t setup_once() override;
    esp_err_t enter() override;
    bool on_motion(const MotionTick &tick) override;
    void on_touch(const TouchEvent &event) override;
    const LauncherVisual *launcher_visual() const override;
    ShellAction handle_event(AppEvent event) override;
    esp_err_t update(float dt = kPhysicsDt) override;
    bool render(DisplayFrame &frame) override;
    AppStats stats() override;
    void leave() override;

private:
    static constexpr int kMaxSpikes = 24;
    static constexpr int kPanelWidth = 240;
    static constexpr int kPanelHeight = 240;
    static constexpr float kFWidth = 240.0f;
    static constexpr float kFHeight = 240.0f;

    // Player translation is deliberately quick and slippery; the articulated
    // pose lags this driven center of mass.
    static constexpr float kHeadRadius = 7.0f;
    static constexpr float kTorsoHalfLength = 11.0f;
    static constexpr float kUpperArmLength = 13.0f;
    static constexpr float kForearmLength = 13.0f;
    static constexpr float kThighLength = 15.0f;
    static constexpr float kShinLength = 16.0f;
    static constexpr int kTorsoRadius = 4;
    static constexpr int kLimbRadius = 3;
    static constexpr float kAccelScale = 42.0f;  ///< apparent unit -> px/s^2
    static constexpr float kDamping = 0.976f;    ///< per substep, slippery
    static constexpr float kMaxSpeed = 210.0f;
    static constexpr float kMoveClamp = 15.0f;   ///< tilt clamp (sim units)
    static constexpr float kLethalRadius = 7.5f;

    // Spikes.
    static constexpr float kSpikeVInitial = 125.0f;  ///< px/s
    static constexpr float kSpikeVTimeRamp = 1.2f;   ///< px/s per alive second
    static constexpr float kSpikeVMax = 260.0f;
    static constexpr float kSpikeVRandom = 40.0f;
    static constexpr float kSpikeSpawnY = -6.0f;
    static constexpr float kSpikeExitY = 270.0f;

    // Waves become denser, faster, and more frequent with uninterrupted
    // survival time rather than waiting for the score to advance.
    static constexpr float kWaveIntervalInitial = 1.35f;
    static constexpr float kWaveIntervalMin = 0.42f;
    static constexpr float kWaveIntervalTimeRamp = 0.010f;
    static constexpr float kWaveCountRampSeconds = 24.0f;
    static constexpr int kWaveInitialCount = 2;
    static constexpr int kWaveMaxCount = 5;
    static constexpr float kWaveSpacingMin = 30.0f;
    static constexpr float kFirstWaveClearance = 50.0f;

    static constexpr int kMaxHighscores = 5;

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
        float vy = 0.0f;
        bool active = false;
    };

    struct SpikeRender {
        float x = 0.0f;
        float y = 0.0f;
        bool active = false;
    };

    struct AvalancheFrame {
        uint32_t sequence = 0;
        uint32_t epoch = 0;
        uint32_t tick = 0;               ///< 60 Hz UI animation clock
        int32_t score = 0;               ///< spikes dodged
        int32_t best_score = 0;
        std::array<int32_t, kMaxHighscores> top_scores{};
        bool game_over = false;
        float player_x = 120.0f;
        float player_y = 120.0f;
        RagdollPose pose{};
        std::array<SpikeRender, kMaxSpikes> spikes{};
    };

    struct SharedMotion {
        Vec3 apparent{0.0f, 0.0f, 6.0f};
        Vec3 raw{0.0f, 0.0f, 0.0f};
        bool valid = false;
    };

    void spawn_wave();
    void kill_player();
    void reset_game();
    void step_substep();
    void reset_ragdoll_pose();
    void step_ragdoll_pose(float acceleration_x, float acceleration_y,
                           bool dead);
    static RagdollGeometry
    calculate_ragdoll_geometry(float x, float y, const RagdollPose &pose);
    void load_highscores();
    void save_highscore(int32_t score);

    void fill_snapshot(AvalancheFrame &snapshot);

    // Stripe-aware pixel drawing (screen coordinates).
    static void draw_rect(uint16_t *pixels, int width, int y0, int rows,
                          int left, int top, int right, int bottom,
                          uint16_t color);
    static void draw_disc(uint16_t *pixels, int width, int y0, int rows,
                          int cx, int cy, int radius, uint16_t color);
    static void draw_segment(uint16_t *pixels, int width, int y0, int rows,
                             int x0, int y0_screen, int x1, int y1,
                             int radius, uint16_t color);
    static void draw_spike(uint16_t *pixels, int width, int y0, int rows,
                           float x, float tip_y, uint16_t shaft_color,
                           uint16_t tip_color);
    static void draw_ragdoll(uint16_t *pixels, int width, int y0, int rows,
                             float x, float y, const RagdollPose &pose,
                             uint16_t body_color, uint16_t head_color);
    static void draw_digit(uint16_t *pixels, int width, int y0, int rows,
                           int digit, int x, int y, int scale, uint16_t color);
    static void draw_number(uint16_t *pixels, int width, int y0, int rows,
                            int32_t value, int x, int y, int scale,
                            uint16_t color, int min_digits);
    static void raster_stripe(const AvalancheFrame &frame, uint16_t *pixels,
                              int width, int y0, int rows);

    MotionFilter filter_;
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_{};

    // Update-lane-only model.
    float player_x_ = 120.0f;
    float player_y_ = 120.0f;
    float player_vx_ = 0.0f;
    float player_vy_ = 0.0f;
    RagdollPose ragdoll_pose_{};
    std::array<float, JointCount> joint_velocities_{};
    float body_angular_velocity_ = 0.0f;
    float death_flop_direction_ = 1.0f;
    std::array<SpikeState, kMaxSpikes> spikes_{};
    float survival_time_ = 0.0f;
    float wave_timer_ = 0.0f;
    float wave_interval_ = kWaveIntervalInitial;
    bool first_wave_pending_ = true;
    int32_t score_ = 0;               ///< spikes dodged
    uint32_t tick_ = 0;               ///< 60 Hz UI animation clock
    uint32_t epoch_ = 0;
    uint32_t sequence_ = 0;
    // Written by the update lane, read by the sensor lane in on_touch()
    // (restart-after-game-over tap). Relaxed is sufficient: a stale read
    // only delays or duplicates a restart request.
    std::atomic<bool> game_over_{false};
    std::atomic<bool> reset_requested_{false};

    int32_t best_score_ = 0;
    std::array<int32_t, kMaxHighscores> top_scores_{};

    LatestFrameExchange<AvalancheFrame, 3> frames_;

    std::atomic<uint32_t> published_epoch_{0};
    std::atomic<uint32_t> active_count_{0};
    std::atomic<uint32_t> physics_us_{0};
    std::atomic<uint32_t> nonfinite_resets_{0};
    uint32_t raster_us_ = 0;
    uint32_t frame_us_ = 0;
    bool setup_done_ = false;
    bool nvs_inited_ = false;
};

extern RagdollAvalancheApp s_ragdoll_avalanche_app;

}  // namespace fluid_demo
