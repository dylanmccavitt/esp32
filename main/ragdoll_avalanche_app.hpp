#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
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
    static constexpr std::size_t kSnapshotSlotCount = 3;
    static constexpr int kMaxSpikes = 24;
    static constexpr int kPanelWidth = 240;
    static constexpr int kPanelHeight = 240;
    static constexpr float kFWidth = 240.0f;
    static constexpr float kFHeight = 240.0f;

    // Player (the ragdoll): slippery steering, like the original.
    static constexpr float kBodRadius = 6.0f;   ///< collision circle
    static constexpr float kHeadRadius = 4.0f;
    static constexpr float kAccelScale = 18.0f; ///< apparent unit -> px/s^2
    static constexpr float kDamping = 0.965f;   ///< per substep, slippery
    static constexpr float kMaxSpeed = 150.0f;
    static constexpr float kMoveClamp = 15.0f;  ///< tilt clamp (sim units)

    // Spikes.
    static constexpr float kSpikeVInitial = 95.0f;   ///< px/s
    static constexpr float kSpikeVRamp = 0.16f;      ///< px/s per dodge
    static constexpr float kSpikeVMax = 200.0f;
    static constexpr float kSpikeVRandom = 30.0f;
    static constexpr float kSpikeHalfWidth = 3.0f;
    static constexpr float kSpikeHitRadius = 12.0f;  ///< circle+tip leniency
    static constexpr float kSpikeSpawnY = -18.0f;
    static constexpr float kSpikeExitY = 252.0f;

    // Waves.
    static constexpr float kWaveIntervalInitial = 2.4f;
    static constexpr float kWaveIntervalMin = 0.55f;
    static constexpr float kWaveIntervalRamp = 0.028f;  ///< s per dodge
    static constexpr int kWaveMaxCount = 4;
    static constexpr float kWaveSpacingMin = 26.0f;

    static constexpr int kMaxHighscores = 5;
    static constexpr float kPlayerMinX = kBodRadius + 2.0f;
    static constexpr float kPlayerMaxX = kFWidth - kBodRadius - 2.0f;
    static constexpr float kPlayerMinY = 20.0f;  ///< head clears score HUD
    static constexpr float kPlayerMaxY = kFHeight - kBodRadius - 2.0f;

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
        uint32_t tick = 0;               ///< animation clock (rendered side)
        int32_t score = 0;               ///< spikes dodged
        int32_t best_score = 0;
        std::array<int32_t, kMaxHighscores> top_scores{};
        bool game_over = false;
        float player_x = 120.0f;
        float player_y = 120.0f;
        float player_rot = 0.0f;         ///< velocity lean
        float player_vx = 0.0f;          ///< limb flail direction
        std::array<SpikeRender, kMaxSpikes> spikes{};
    };

    enum class SlotState : uint8_t {
        Free,
        Writing,
        Ready,
        Reading,
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
    void load_highscores();
    void save_highscore(int32_t score);

    AvalancheFrame *begin_snapshot();
    void publish_snapshot(AvalancheFrame *snapshot);
    const AvalancheFrame *acquire_snapshot();
    void release_snapshot(const AvalancheFrame *snapshot);
    void drain_snapshots();
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
                             float x, float y, float lean, uint32_t tick,
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
    std::array<SpikeState, kMaxSpikes> spikes_{};
    float wave_timer_ = 0.0f;
    float wave_interval_ = kWaveIntervalInitial;
    int32_t score_ = 0;               ///< spikes dodged
    uint32_t tick_ = 0;               ///< animation clock, 60 Hz
    uint32_t epoch_ = 0;
    uint32_t sequence_ = 0;
    // Written by the update lane, read by the sensor lane in on_touch()
    // (restart-after-game-over tap). Relaxed is sufficient: a stale read
    // only delays or duplicates a restart request.
    std::atomic<bool> game_over_{false};
    std::atomic<bool> reset_requested_{false};

    int32_t best_score_ = 0;
    std::array<int32_t, kMaxHighscores> top_scores_{};

    std::array<AvalancheFrame, kSnapshotSlotCount> snapshots_{};
    std::array<std::atomic<SlotState>, kSnapshotSlotCount> snapshot_states_{};
    std::array<std::atomic<uint32_t>, kSnapshotSlotCount> snapshot_sequences_{};
    AvalancheFrame *write_snapshot_ = nullptr;

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
