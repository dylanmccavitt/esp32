#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "fluid.hpp"
#include "motion.hpp"
#include "snapshot_exchange.hpp"

namespace fluid_demo {

/// Fluid Box app: owns the Fluid simulation (non-movable, in-place value), the
/// MotionFilter, the SnapshotExchange, the reset atomic, the app motion mux,
/// the telemetry atomics, and the whole Fluid raster half (Projected/Edge,
/// surface maps, LUTs, edges, raster caller) that renderer.cpp used to own.
///
/// All heavy state lives inside this one namespace-scope object (s_fluid_app),
/// never on a task stack. The sensor lane calls on_motion(), the physics lane
/// update(), the render lane render(); the app allocates nothing between
/// setup_once() and the end of the process.
class FluidBoxApp final : public App {
public:
    /// Tuned startup count for 30 Hz physics on the 240x240 panel.
    static constexpr uint16_t kInitialParticles = 216;

    FluidBoxApp() = default;
    ~FluidBoxApp() override;
    FluidBoxApp(const FluidBoxApp &) = delete;
    FluidBoxApp &operator=(const FluidBoxApp &) = delete;

    /// Transactional one-time setup: fluid lattice + raster buffers + LUTs.
    /// Rollback + ESP_ERR_NO_MEM on any allocation failure; idempotent.
    esp_err_t setup_once() override;

    /// Post-barrier entry: drains stale snapshots, frame_seen_ = false, does
    /// NOT clear a pending reset (honored by the first post-enter update()).
    esp_err_t enter() override;

    /// Sensor lane: filter (or override bypass) + publish under the app mux.
    /// Returns true iff the fresh physical sample was accepted by the filter.
    bool on_motion(const MotionTick &tick) override;

    /// PlusPress sets the app's reset atomic; no event needs a shell action.
    ShellAction handle_event(AppEvent) override;

    /// Physics lane at fixed dt: consume pending reset exactly once, step the
    /// simulation, publish a frame.
    esp_err_t update(float dt = kPhysicsDt) override;

    /// Render lane: acquire internally, raster + stream via DisplayFrame ops.
    /// Returns true iff a frame was rendered to completion.
    bool render(DisplayFrame &frame) override;

    /// Telemetry: atomics written by update/render lanes. Non-const because it
    /// samples the motion state under the app's portMUX critical section.
    AppStats stats() override;

    /// No-allocation leave: quiesce motion validity.
    void leave() override;

    /// Console/PLUS reset path: sets the app's reset atomic. Consumed by the
    /// first post-enter update() via exchange(false) exactly once.
    void request_fluid_reset();

private:
    // ---- Fluid raster half (moved from renderer.cpp, byte-for-byte) ----
    struct Projected {
        int16_t x;          ///< Full-resolution screen center x (px).
        int16_t y;          ///< Full-resolution screen center y (px).
        uint16_t radius;    ///< Full-resolution particle radius (px), >= 1.
        uint8_t speed_idx;  ///< Palette index for the velocity color.
        uint8_t active;     ///< 0 = particle skipped (non-finite/out of range).
        uint32_t z_fx;      ///< Center depth, fixed point, 1/4096 world unit.
        uint32_t rw_fx;     ///< World radius, fixed point, 1/4096 world unit.
    };

    struct Edge {
        int x0;
        int y0;
        int x1;
        int y1;
    };

    void build_luts();
    void preproject(const ParticleFrame &frame, int count);
    void project_box_edges();
    void build_surface();
    void draw_box_edges(uint16_t *buf, int y0, int rows);
    void shade_surface(uint16_t *buf, int y0, int rows);
    void free_buffers();
    esp_err_t render_frame(const ParticleFrame &frame, DisplayFrame &df);

    static uint16_t hsv_to_rgb565(float h, float s, float v);

    // ---- app-owned simulation/motion state (absorbed from app_main) ----
    Fluid fluid_;                // non-movable; in-place value, never moved
    MotionFilter filter_;
    SnapshotExchange snapshots_;

    /// Motion state published by the sensor lane to the update lane under a
    /// short critical section (same portMUX pattern as legacy app_main).
    struct SharedMotion {
        Vec3 apparent_accel{0.0f, 0.0f, 6.0f};  // rest-gravity placeholder
        Vec3 raw_accel{0.0f, 0.0f, 0.0f};
        bool valid{false};
    };
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_;

    /// Reset request: any task sets, the update lane consumes once.
    std::atomic<bool> reset_requested_{false};

    // Fluid telemetry copied by the update lane into atomics (the render lane
    // reads them for telemetry; reading FluidStats directly would race).
    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint64_t> candidate_checks_{0};
    std::atomic<uint32_t> nonfinite_resets_{0};
    std::atomic<uint32_t> physics_us_{0};

    // ---- raster state ----
    uint16_t *surface_field_ = nullptr;
    uint16_t *surface_heat_ = nullptr;
    uint16_t *surface_depth_ = nullptr;
    Projected *proj_ = nullptr;
    int active_count_ = 0;

    // Render-task-only telemetry for the last frame.
    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;

    // Look-up tables built at setup: sphere-front depth and velocity palette.
    uint16_t nz_lut_[256] = {};
    uint16_t palette_[256] = {};

    Edge edges_[12] = {};

    // Update-lane-only producer sequence; sensor-lane/other plain state.
    uint32_t sequence_ = 0;
    bool setup_done_ = false;
    bool frame_seen_ = false;  // first-frame gate, opened by enter()
};

/// One namespace-scope instance of the registered Fluid Box app (defined in
/// fluid_app.cpp; the registry binds &s_fluid_app at the runtime slice).
extern FluidBoxApp s_fluid_app;

}  // namespace fluid_demo
