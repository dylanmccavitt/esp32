#pragma once

#include <cstdint>

#include "esp_err.h"

#include "app_types.hpp"

namespace fluid_demo {

/// Lifecycle mode of the shell (coordinator-side state only; no app sees it).
enum class AppMode : uint8_t {
    Launcher = 0,
    Entering = 1,
    Running = 2,
    Transition = 3,
};

/// User events routed to the active app.
enum class AppEvent : uint8_t {
    PlusPress = 0,
};

/// Action an app requests after handling an event; no shell actions exist yet.
enum class ShellAction : uint8_t {
    None = 0,
};

/// One motion sample delivered by the shell's raw-motion pipeline.
///
/// The shell's sensor lane and MotionService own the IMU poll, the
/// [2 ms, 100 ms] dt clamp and the dev-console override check. The
/// app owns the MotionFilter and decides to bypass it verbatim when an
/// override is active (matches the legacy sensor task, app_main.cpp:312).
struct MotionTick {
    Vec3 accel_mps2{};      ///< Raw sensor-frame acceleration (m/s^2).
    Vec3 gyro_rads{};       ///< Raw sensor-frame angular rate (rad/s, telemetry only).
    Vec3 apparent_accel{};  ///< Box-frame apparent acceleration (sim units), valid
                            ///< only when override_active.
    float dt = 0.0f;        ///< Seconds since the previous fresh sample, clamped.
    bool fresh = false;     ///< This tick consumed a new valid IMU read.
    bool override_active = false;  ///< Shell supplied apparent_accel verbatim.
};

/// Per-second telemetry owned by the app. The update lane writes the sim/phys
/// fields, the render lane writes raster/frame; the shell composes the
/// display-transport fields (dma wait, missed transfers) and heap minima into
/// the byte-identical ESP_LOGI line.
struct AppStats {
    uint32_t count = 0;              ///< Particle count (fixed at setup).
    uint32_t epoch = 0;              ///< Fluid reset epoch.
    uint64_t candidate_checks = 0;   ///< Cumulative solver neighbor checks.
    uint32_t nonfinite_resets = 0;   ///< Cumulative deterministic resets.
    uint32_t physics_us = 0;         ///< Last fluid step wall time.
    float raw[3] = {0.0f, 0.0f, 0.0f};       ///< Last valid raw sensor accel.
    float apparent[3] = {0.0f, 0.0f, 6.0f};  ///< Last published apparent accel.
    uint32_t raster_us = 0;          ///< Last frame raster time (excl. DMA waits).
    uint32_t frame_us = 0;           ///< Last frame total time.
};

/// Panel transport view handed to the app's render path. The shell binds the
/// ops to its DisplayService instance, so the app drives render sequencing
/// through ops only and never touches panel/I/O handles. Wire order, capture
/// mirroring and DMA timing stay entirely in the shell's DisplayService.
struct DisplayFrame {
    uint16_t *stripe[2] = {nullptr, nullptr};
    int width = 240;
    int height = 240;
    int stripe_rows = 16;
    int stripe_count = 15;
    void *transport = nullptr;  ///< Opaque shell transport handle (DisplayService*).

    struct Ops {
        /// Retire the carried final-stripe transfer (blocks up to the DMA
        /// deadline). ESP_OK or ESP_ERR_TIMEOUT (in-flight stays set).
        esp_err_t (*wait_previous)(void *transport) = nullptr;
        /// Frame-boundary capture latch; never fails (always true).
        bool (*latch_capture)(void *transport) = nullptr;
        /// Mirror (if armed), wait for the previous transfer, draw stripe `s`.
        esp_err_t (*submit)(void *transport, int s, int y0, int rows,
                            const uint16_t *pixels) = nullptr;
        /// Close the frame: latch capture_ready iff the whole frame was armed.
        esp_err_t (*finish)(void *transport) = nullptr;
        /// Per-frame capture-copy microseconds (shell accumulates PSRAM mirror
        /// time across the armed frame's stripes since the last latch).
        uint32_t (*capture_copy_us)(void *transport) = nullptr;
    } ops;
};

/// Abstract multi-app shell app. FluidBoxApp is the sole registered app.
class App {
public:
    /// Fixed simulation step, independent of wake-up granularity.
    static constexpr float kPhysicsDt = 1.0f / 30.0f;

    virtual ~App() = default;

    /// One-time transactional setup: allocate all heavy state. No panel
    /// access. Rolled back to ESP_ERR_NO_MEM on any allocation failure and
    /// idempotent on success. Runs once per boot before the lanes start.
    virtual esp_err_t setup_once() = 0;

    /// Enter the app (post-barrier, render lane idle): no allocation, drains
    /// stale snapshots, opens the first-frame gate, and does NOT clear a
    /// pending reset — the first post-enter update() consumes it exactly once.
    virtual esp_err_t enter() = 0;

    /// One raw motion tick from the shell (sensor lane). Returns true iff the
    /// fresh physical sample was accepted by the app's filter, so the caller
    /// advances its IMU time anchor only on acceptance.
    virtual bool on_motion(const MotionTick &tick) = 0;

    /// Route a user event; returns the requested shell action.
    virtual ShellAction handle_event(AppEvent event) = 0;

    /// Advance simulation + publish one frame (update lane, fixed dt).
    virtual esp_err_t update(float dt = kPhysicsDt) = 0;

    /// Render the newest frame through the shell-bound DisplayFrame (render
    /// lane). Acquires internally; a null snapshot is a blank pass that
    /// touches no transport. Returns true iff a frame was actually rendered
    /// to completion, so the caller only updates per-frame timing on
    /// completed frames.
    virtual bool render(DisplayFrame &frame) = 0;

    /// Telemetry snapshot (render lane reads; fields written by their owners).
    virtual AppStats stats() = 0;

    /// Leave the app: no allocation, quiesce motion validity.
    virtual void leave() = 0;
};

}  // namespace fluid_demo
