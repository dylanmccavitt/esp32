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

/// Fixed-capacity top-down tilt maze. The sensor, update, and render lanes
/// communicate only through short motion/telemetry critical sections, reset
/// atomics, and a non-blocking triple snapshot exchange.
class TiltMazeApp final : public App {
public:
    TiltMazeApp() = default;
    ~TiltMazeApp() override = default;
    TiltMazeApp(const TiltMazeApp &) = delete;
    TiltMazeApp &operator=(const TiltMazeApp &) = delete;

    esp_err_t setup_once() override;
    esp_err_t enter() override;
    bool on_motion(const MotionTick &tick) override;
    void on_touch_begin(const TouchEvent &event) override;
    const LauncherVisual *launcher_visual() const override;
    void on_plus_press() override;
    void on_system_telemetry(const SystemTelemetry &telemetry) override;
    esp_err_t update(float dt = kPhysicsDt) override;
    bool render(DisplayFrame &frame) override;
    AppStats stats() override;
    void leave() override;

private:
    struct MazeFrame {
        uint32_t sequence = 0;
        uint32_t epoch = 0;
        float ball_x = 51.0f;
        float ball_y = 197.0f;
        uint8_t progress = 0;
        uint8_t round_pips = 0;
        bool solved = false;

        // Fixed-value copy: rasterization never observes telemetry changing
        // beneath a frame and no task handles or names enter app storage.
        SystemTelemetry system{};
        uint32_t frozen_telemetry_generation = 0;
        uint32_t frozen_internal_free_bytes = 0;
        uint32_t frozen_internal_largest_free_block = 0;
    };

    struct SharedMotion {
        Vec3 apparent{0.0f, 0.0f, 6.0f};
        Vec3 raw{0.0f, 0.0f, 0.0f};
        bool valid = false;
    };

    void reset_maze(bool log_transition);
    void step_substep(float apparent_x, float apparent_y);
    void move_x(float delta);
    void move_y(float delta);
    void update_progress_and_win();

    void fill_snapshot(MazeFrame &snapshot);
    void freeze_memory_target();

    static void draw_rect(uint16_t *pixels, int width, int y0, int rows,
                          int left, int top, int right, int bottom, uint16_t color);
    static void draw_disc(uint16_t *pixels, int width, int y0, int rows,
                          int cx, int cy, int radius, uint16_t color);
    static void draw_ring(uint16_t *pixels, int width, int y0, int rows,
                          int cx, int cy, int outer_radius, int inner_radius,
                          uint16_t outer, uint16_t inner);
    static void draw_segment(uint16_t *pixels, int width, int y0, int rows,
                             int x0, int y0_screen, int x1, int y1,
                             int radius, uint16_t color);
    static uint16_t task_state_color(SystemTaskState state);
    static void draw_task_node(const SystemTaskTelemetry &task, std::size_t index,
                               uint16_t *pixels, int width, int y0, int rows);
    static void draw_memory_gauge(uint16_t *pixels, int width, int y0, int rows,
                                  int left, uint32_t free_bytes,
                                  uint32_t bytes_per_segment, uint16_t color,
                                  bool valid);
    static void draw_goal(const MazeFrame &maze, uint16_t *pixels,
                          int width, int y0, int rows);
    static void raster_stripe(const MazeFrame &maze, uint16_t *pixels,
                              int width, int y0, int rows);

    // Sensor-lane-owned filter; the filtered result crosses lanes under mux.
    MotionFilter filter_;
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_{};
    // Render callback and update/reset can overlap. This mux protects only
    // short fixed-struct copies; telemetry is sampled externally at <=1 Hz.
    portMUX_TYPE telemetry_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SystemTelemetry latest_system_telemetry_{};
    uint32_t frozen_telemetry_generation_ = 0;
    uint32_t frozen_internal_free_bytes_ = 0;
    uint32_t frozen_internal_largest_free_block_ = 0;
    std::atomic<bool> reset_requested_{false};

    // Update-lane-only model. Home preserves these fields; enter only removes
    // velocity and stale transport snapshots.
    float ball_x_ = 51.0f;
    float ball_y_ = 197.0f;
    float velocity_x_ = 0.0f;
    float velocity_y_ = 0.0f;
    uint32_t sequence_ = 0;
    uint32_t epoch_ = 0;
    uint32_t completed_rounds_ = 0;
    uint8_t progress_ = 0;
    bool solved_ = false;

    LatestFrameExchange<MazeFrame, 3> frames_;

    // Cross-lane telemetry; raster/frame timings are render-lane-only.
    std::atomic<uint32_t> published_epoch_{0};
    std::atomic<uint32_t> physics_us_{0};
    std::atomic<uint32_t> nonfinite_resets_{0};
    uint32_t raster_us_ = 0;
    uint32_t frame_us_ = 0;
    bool setup_done_ = false;
};

/// Namespace-scope registered instance. Every byte of model and exchange
/// storage is inline; setup and steady-state operation allocate nothing.
extern TiltMazeApp s_tilt_maze_app;

}  // namespace fluid_demo
