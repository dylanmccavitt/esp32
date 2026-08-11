#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "motion.hpp"

namespace fluid_demo {

/// Fluid Box app, bouncy-speck edition: ~3000 ballistic 1-pixel particles
/// with real Q8.8 velocities, wall/particle restitution, kick-wake
/// agitation and zero-repose leveling. Specks fly in arcs, bounce off
/// the box and each other, and drain into a shallow level bed instead of
/// piling into heaps. Color tracks speed: sand shades at rest, gold ->
/// orange -> red with velocity, decaying as afterglow.
///
/// Threading: the sensor lane feeds on_motion() through the MotionFilter
/// exactly as before. The engine is single-threaded — it steps and
/// rasterizes inside render() on the render lane (positions are the only
/// ground truth; the occupancy grid is rebuilt from them every frame and
/// doubles as the raster source). update() (physics lane) is a no-op.
class FluidBoxApp final : public App {
public:
    FluidBoxApp() = default;
    ~FluidBoxApp() override;
    FluidBoxApp(const FluidBoxApp &) = delete;
    FluidBoxApp &operator=(const FluidBoxApp &) = delete;

    /// One-time setup: allocate the particle arrays and occupancy grid,
    /// build the wire-order palette, lay down the starting bed. Rollback +
    /// ESP_ERR_NO_MEM on allocation failure; idempotent.
    esp_err_t setup_once() override;

    /// Post-barrier entry; no allocation. A pending reset set while
    /// inactive stays set and is consumed by the first render() step.
    esp_err_t enter() override;

    /// Sensor lane: filter (or override bypass) + publish under the app mux.
    bool on_motion(const MotionTick &tick) override;

    /// PlusPress sets the reset atomic; no event needs a shell action.
    ShellAction handle_event(AppEvent) override;

    /// Physics lane: no-op — the engine steps in render() so no state
    /// crosses cores.
    esp_err_t update(float dt = kPhysicsDt) override;

    /// Render lane: consume a pending reset, step the particle engine
    /// toward the current gravity, rasterize the grid through DisplayFrame.
    bool render(DisplayFrame &frame) override;

    /// Telemetry: count = particle population, candidate_checks =
    /// cumulative walk steps, nonfinite_resets = step-governor hits,
    /// physics_us = last engine step time.
    AppStats stats() override;

    /// No-allocation leave: quiesce motion validity.
    void leave() override;

    /// Console/PLUS reset path: rebuild the starting bed (render lane
    /// consumes the atomic exactly once).
    void request_fluid_reset();

private:
    // ---- geometry ----
    static constexpr int kGridW = 240;
    static constexpr int kGridH = 240;
    static constexpr int kParticleCount = 3000;  ///< bed ~12.6 px on an edge

    // Particle state byte: bits 0-2 base shade (0..5), bits 3-5 displayed
    // glow level, bit 6 ASLEEP. Grid byte: 0 empty, else
    // (level << 3) | (shade + 1) with bits 6-7 = saturating kick counter
    // harvested one frame later; the raster masks & 0x3F so kick bits are
    // invisible.
    static constexpr uint8_t kStateShadeMask = 0x07;
    static constexpr int kStateGlowShift = 3;
    static constexpr uint8_t kStateGlowMask = 0x38;
    static constexpr uint8_t kStateAsleep = 0x40;
    /// Rebuild overlap loser this frame: its start cell holds the winner's
    /// byte, so its walk must not lift or wake-tag around that cell.
    static constexpr uint8_t kStateOverlap = 0x80;

    void reset_particles();
    /// One fixed-dt engine step toward screen-space gravity (sim units,
    /// sgx right, sgy down). Returns walk steps consumed (telemetry).
    uint32_t step_particles(float sgx, float sgy);
    void draw_grid(uint16_t *buf, int y0, int rows);
    inline uint32_t rnd();

    MotionFilter filter_;

    /// Motion state published by the sensor lane to the render lane under a
    /// short critical section (same portMUX pattern as previous builds).
    struct SharedMotion {
        Vec3 apparent_accel{0.0f, 0.0f, 9.0f};  // rest-gravity placeholder
        Vec3 raw_accel{0.0f, 0.0f, 0.0f};
        bool valid{false};
    };
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_;

    /// Reset request: any task sets, the render lane consumes once.
    std::atomic<bool> reset_requested_{false};

    // Telemetry atomics (stats() may be called from another lane).
    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint64_t> walk_steps_{0};
    std::atomic<uint32_t> governor_hits_{0};
    std::atomic<uint32_t> physics_us_{0};

    // ---- particle SoA (one internal-heap arena) + occupancy grid ----
    void *arena_ = nullptr;      ///< owning pointer for the SoA block
    uint16_t *px_ = nullptr;     ///< Q8.8 pixels, raw [256, 61183]
    uint16_t *py_ = nullptr;
    int16_t *vx_ = nullptr;      ///< Q8.8 px/frame
    int16_t *vy_ = nullptr;
    uint8_t *pstate_ = nullptr;  ///< shade | glow | ASLEEP
    uint8_t *prest_ = nullptr;   ///< consecutive quiet frames toward sleep
    uint8_t *grid_ = nullptr;    ///< kGridW * kGridH occupancy + raster bytes

    uint32_t rng_ = 0x2545F491u;  ///< xorshift32 state (render lane only)
    uint32_t frame_parity_ = 0;   ///< alternates sweep direction per frame
    uint32_t awake_count_ = 0;    ///< particles not ASLEEP after last step
    uint32_t rest_gate_frames_ = 0;
    float dir_gx_ = 0.0f;         ///< remembered gravity direction (unit);
    float dir_gy_ = 1.0f;         ///< down always exists, boot = screen-down
    float prev_gx_ = 0.0f;        ///< wake-reference in-plane accel anchor
    float prev_gy_ = 0.0f;

    /// Wire-order (pre-swapped) RGB565 palette indexed by the grid byte
    /// (level << 3 | shade + 1) & 0x3F; shade-0 slots unused.
    uint16_t shade_wire_[64] = {};

    // Render-lane-only telemetry for the last frame.
    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;

    bool setup_done_ = false;
};

/// One namespace-scope instance of the registered Fluid Box app (defined in
/// fluid_app.cpp; the registry binds &s_fluid_app at the runtime slice).
extern FluidBoxApp s_fluid_app;

}  // namespace fluid_demo
