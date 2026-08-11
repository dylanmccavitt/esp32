#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "motion.hpp"

namespace fluid_demo {

/// Fluid Box app, sand edition: a tilt-driven falling-sand cellular
/// automaton at native panel resolution. One grain occupies one screen
/// pixel cell; ~14,000 independent grains fall and topple toward the
/// current in-plane gravity direction, giving dense piles, real angles
/// of repose and grain-by-grain avalanches with no rendering tricks.
///
/// Threading: the sensor lane feeds on_motion() through the MotionFilter
/// exactly as before. The automaton itself is single-threaded — it steps
/// and rasterizes inside render() on the render lane, so the grid needs
/// no cross-core snapshotting; update() (physics lane) is a no-op. All
/// heavy state lives inside this one namespace-scope object.
class FluidBoxApp final : public App {
public:
    FluidBoxApp() = default;
    ~FluidBoxApp() override;
    FluidBoxApp(const FluidBoxApp &) = delete;
    FluidBoxApp &operator=(const FluidBoxApp &) = delete;

    /// One-time setup: allocate the cell grid, build the wire-order shade
    /// table, lay down the deterministic starting pile. Rollback +
    /// ESP_ERR_NO_MEM on allocation failure; idempotent.
    esp_err_t setup_once() override;

    /// Post-barrier entry; no allocation. A pending reset set while
    /// inactive stays set and is consumed by the first render() step.
    esp_err_t enter() override;

    /// Sensor lane: filter (or override bypass) + publish under the app mux.
    bool on_motion(const MotionTick &tick) override;

    /// PlusPress sets the reset atomic; no event needs a shell action.
    ShellAction handle_event(AppEvent) override;

    /// Physics lane: no-op — the automaton steps in render() so the grid
    /// never crosses cores.
    esp_err_t update(float dt = kPhysicsDt) override;

    /// Render lane: consume a pending reset, step the automaton toward the
    /// current gravity, rasterize the grid through DisplayFrame ops.
    bool render(DisplayFrame &frame) override;

    /// Telemetry: count = grain population, candidate_checks = cumulative
    /// grain moves, physics_us = last automaton step time.
    AppStats stats() override;

    /// No-allocation leave: quiesce motion validity.
    void leave() override;

    /// Console/PLUS reset path: rebuild the starting pile (render lane
    /// consumes the atomic exactly once).
    void request_fluid_reset();

private:
    // ---- automaton geometry ----
    static constexpr int kGridW = 240;
    static constexpr int kGridH = 240;
    static constexpr int kMargin = 1;     ///< solid one-cell border walls
    static constexpr int kPileRows = 60;  ///< reset pile depth (rows of sand)
    static constexpr int kShadeCount = 6; ///< per-grain fixed sand shades

    // Cell byte layout: bits 0-2 shade (1..kShadeCount, 0 = empty cell),
    // bits 3-5 movement heat (0..kHeatMax). A move writes full heat; the
    // raster decays it one level per frame, driving the reactive palette.
    static constexpr int kShadeBits = 3;
    static constexpr uint8_t kShadeMask = 0x07;
    static constexpr int kHeatMax = 7;
    static constexpr uint8_t kHeatBits = kHeatMax << kShadeBits;

    void reset_grid();
    /// One automaton substep toward screen-space gravity (sgx right,
    /// sgy down). Returns the number of grains that moved.
    uint32_t step_sand(float sgx, float sgy);
    void draw_sand(uint16_t *buf, int y0, int rows);
    inline uint32_t rnd();

    MotionFilter filter_;

    /// Motion state published by the sensor lane to the render lane under a
    /// short critical section (same portMUX pattern as the fluid build).
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
    std::atomic<uint64_t> moved_cells_{0};
    std::atomic<uint32_t> physics_us_{0};

    /// Cell grid, kGridW * kGridH bytes: 0 = empty, otherwise a grain
    /// carrying its fixed shade index plus movement heat (see the cell
    /// byte layout above). Allocated once from internal heap.
    uint8_t *grid_ = nullptr;

    uint32_t rng_ = 0x2545F491u;      ///< xorshift32 state (render lane only)
    uint32_t grain_count_ = 0;        ///< population laid down by reset_grid()
    uint32_t frame_parity_ = 0;       ///< alternates scan direction per frame

    /// Wire-order (pre-swapped) RGB565 palette indexed by the full cell
    /// byte (heat << kShadeBits | shade); shade-0 slots unused.
    uint16_t shade_wire_[(kHeatMax + 1) << kShadeBits] = {};

    // Render-lane-only telemetry for the last frame.
    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;

    bool setup_done_ = false;
};

/// One namespace-scope instance of the registered Fluid Box app (defined in
/// fluid_app.cpp; the registry binds &s_fluid_app at the runtime slice).
extern FluidBoxApp s_fluid_app;

}  // namespace fluid_demo
