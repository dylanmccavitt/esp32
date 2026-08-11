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
    /// Startup count, set by measured worst-case hardware budget: with the
    /// contact kernel (h = 1.35 * spacing), vigorous shaking compresses the
    /// pile and roughly doubles step cost versus rest (400 grains: ~20 ms
    /// rest but ~45 ms shaken = visible slow-motion chop; 768: ~49 ms even
    /// at rest). 256 grains keep the hardest shake under the 33.3 ms step.
    /// Apparent grain count is multiplied by the renderer's speck clusters
    /// (kSpecksPerGrain in fluid_app.cpp). The namespace-level
    /// kInitialParticles (app_types.hpp) is the geometry baseline; a
    /// different startup count rescales spacing, not the occupied volume.
    static constexpr uint16_t kInitialParticles = 256;
    static_assert(kInitialParticles >= kMinParticles &&
                      kInitialParticles <= kMaxParticles,
                  "startup count must stay within Fluid's supported range");

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
    // ---- Fluid raster half: depth-sorted pre-shaded sphere sprites ----
    struct Projected {
        int16_t x;          ///< Screen center x (px).
        int16_t y;          ///< Screen center y (px).
        uint16_t radius;    ///< Sprite radius (px), in [1, kMaxSpriteRadius].
        uint8_t speed_idx;  ///< Palette index for the velocity color.
        uint8_t active;     ///< 0 = particle skipped (non-finite/out of range).
        uint8_t spread_px;  ///< Speck cluster scatter radius (px).
        uint32_t z_fx;      ///< Center depth, fixed point, 1/4096 world unit (sort key).
        uint32_t fade;      ///< Depth brightness scale, 0..255 (far = dim).
    };

    struct Edge {
        int x0;
        int y0;
        int x1;
        int y1;
    };

    void build_luts();
    void build_sprites();
    void preproject(const ParticleFrame &frame, int count);
    void sort_back_to_front();
    void project_box_edges();
    void draw_box_edges(uint16_t *buf, int y0, int rows);
    void draw_particles(uint16_t *buf, int y0, int rows);
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
    Projected *proj_ = nullptr;
    int active_count_ = 0;

    // Render-task-only telemetry for the last frame.
    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;

    // Velocity palette built at setup.
    uint16_t palette_[256] = {};

    /// Pre-shaded sphere sprites, one per integer radius r in [1,
    /// kMaxSpriteRadius]: (2r+1)^2 premultiplied brightness bytes (Lambert
    /// shading from the upper-left with a ~1 px antialiased rim; 0 = fully
    /// transparent). Painter's order needs no blending or depth buffer.
    static constexpr int kMaxSpriteRadius = 14;
    static constexpr size_t kSpriteLutBytes = 4494;  // sum of (2r+1)^2, r = 1..14
    uint8_t sprite_lut_[kSpriteLutBytes] = {};
    uint16_t sprite_off_[kMaxSpriteRadius + 1] = {};

    /// Indices of active particles sorted far-to-near for painter's rendering.
    uint16_t draw_order_[kMaxParticles] = {};

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
