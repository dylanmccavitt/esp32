#pragma once

#include <cstdint>

#include "esp_err.h"

#include "app_types.hpp"

namespace fluid_demo {

enum class AppMode : uint8_t {
    Launcher = 0,
    Entering = 1,
    Running = 2,
    Transition = 3,
};

enum class AppEvent : uint8_t {
    PlusPress = 0,
};

enum class ShellAction : uint8_t {
    None = 0,
};

/// Phase of one physical contact, assigned by the shell's InputService as it
/// forwards controller reports. A running app receives exactly one TouchEvent
/// per contact, always with phase Begin; Move/End reports are consumed by the
/// shell (launcher swipe/tap classification) and never reach apps.
enum class TouchPhase : uint8_t {
    Begin = 0, ///< First fresh pressed report of a new contact.
    Move = 1,  ///< Fresh pressed report while the contact continues.
    End = 2,   ///< The contact released; coordinates retain the last report.
};

/// Launcher swipe classification for an ended contact. The controller's
/// gesture register wins; the software fallback requires a dominant
/// horizontal travel of at least `kLauncherSwipeMinPx`.
enum class TouchGesture : uint8_t {
    None = 0,       ///< Not a swipe (tap/drag without dominant travel).
    SwipeLeft = 1,  ///< Horizontal leftward swipe.
    SwipeRight = 2, ///< Horizontal rightward swipe.
};

/// One touch event from the shell's controller, in display space
/// (0..width-1, 0..height-1). `phase` marks the contact progression
/// Begin -> Move* -> End; `gesture` is only meaningful on an End report (the
/// controller-encoded swipe, else None). The sensor lane polls the shell
/// InputService, which forwards every fresh report with this exact structure
/// and retains the last pressed coordinates for an End event.
struct TouchEvent {
    uint16_t x = 0;
    uint16_t y = 0;
    TouchPhase phase = TouchPhase::Begin;
    TouchGesture gesture = TouchGesture::None;
};

/// One motion sample delivered by the shell's raw-motion pipeline.
struct MotionTick {
    Vec3 accel_mps2{};      ///< Raw sensor-frame acceleration (m/s^2).
    Vec3 gyro_rads{};       ///< Raw sensor-frame angular rate (rad/s, telemetry only).
    Vec3 apparent_accel{};  ///< Box-frame apparent acceleration (sim units), valid
                            ///< only when override_active.
    float dt = 0.0f;        ///< Seconds since the previous fresh sample, clamped.
    bool fresh = false;     ///< This tick consumed a new valid IMU read.
    bool override_active = false;  ///< Shell supplied apparent_accel verbatim.
};

/// Per-second telemetry owned by the app.
struct AppStats {
    uint32_t count = 0;              ///< Particle count (fixed at setup).
    uint32_t epoch = 0;              ///< Fluid reset epoch.
    uint64_t candidate_checks = 0;   ///< Cumulative solver neighbor checks.
    uint32_t nonfinite_resets = 0;   ///< Cumulative deterministic resets.
    uint32_t physics_us = 0;         ///< Last fluid step wall time.
    float raw[3] = {0.0f, 0.0f, 0.0f};       ///< Last valid raw sensor accel.
    float apparent[3] = {0.0f, 0.0f, 6.0f};  ///< Last published apparent accel.
    float pitch = 0.0f;              ///< Relative pitch (rad), Cube/Level.
    float roll = 0.0f;               ///< Relative roll (rad), Cube/Level.
    float yaw = 0.0f;                ///< Relative yaw (rad), Cube/Level.
    uint32_t raster_us = 0;          ///< Last frame raster time (excl. DMA waits).
    uint32_t frame_us = 0;           ///< Last frame total time.
};

/// Fixed task roster order used by SystemTelemetry.
enum class SystemTaskKind : uint8_t {
    Coordinator = 0,
    Sensor = 1,
    Update = 2,
    Render = 3,
    Console = 4,
};

/// Conservative mapping of a live FreeRTOS task state. Kernel-internal
/// eDeleted/eInvalid collapse to Unknown so apps never see raw kernel states.
enum class SystemTaskState : uint8_t {
    Running = 0,
    Ready = 1,
    Blocked = 2,
    Suspended = 3,
    Unknown = 4,
};

/// One sampled shell task in the fixed 5-slot roster. `valid` is false while
/// the persistent handle is not (yet) resolvable — e.g. the console REPL if
/// its one-time startup lookup failed — and state/core/stack must be ignored.
struct SystemTaskTelemetry {
    SystemTaskKind kind = SystemTaskKind::Coordinator;
    SystemTaskState state = SystemTaskState::Unknown;
    int8_t core_id = -1;                 ///< Affinity as 0, 1, or -1 (not pinned).
    uint32_t stack_high_water_words = 0; ///< uxTaskGetStackHighWaterMark2 (words).
    bool valid = false;
};

/// Pointer-free live-system snapshot published by the render lane.
struct SystemTelemetry {
    uint32_t generation = 0;                 ///< Render-lane run generation.
    uint32_t internal_free_bytes = 0;        ///< INTERNAL|8BIT total free.
    uint32_t internal_largest_free_block = 0;  ///< INTERNAL|8BIT largest block.
    uint32_t psram_free_bytes = 0;           ///< SPIRAM|8BIT total free.
    SystemTaskTelemetry tasks[5];            ///< Fixed roster, fixed order.
};

/// Shell-owned panel transport exposed to app renderers.
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

/// Palette and 64x64 RGB565 icon for one launcher entry.
struct LauncherVisual {
    uint16_t background_rgb565 = 0;  ///< Full-frame fill outside the band.
    uint16_t band_rgb565 = 0;        ///< Sole selected-entry band color.
    uint16_t affordance_rgb565 = 0;  ///< Left chevron and unselected dot color.
    uint16_t accent_rgb565 = 0;      ///< Right PLUS and selected dot color.
    const uint16_t *icon_rgb565 = nullptr;  ///< kLauncherIconPixels logical RGB565.
};

/// App lifecycle and lane callbacks.
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

    /// One discrete, contact-qualified touch (sensor lane, display space).
    /// The shell delivers exactly one Begin event per physical contact;
    /// Move/End are shell-internal. Default no-op; apps that use touch
    /// override it.
    virtual void on_touch(const TouchEvent &) {}

    /// Launcher entry visual descriptor (app-owned static-lifetime data).
    /// nullptr (the default) renders the exact built-in Fluid Box launcher;
    /// a non-null descriptor supplies the generic launcher's palette and
    /// 64x64 RGB565 icon for this entry.
    virtual const LauncherVisual *launcher_visual() const { return nullptr; }

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

    /// Live system telemetry snapshot, invoked by the shell's render lane at
    /// most once per second while this app is the running generation (never
    /// during console dumps or transitions). The struct is a transient
    /// stack-local copy; render code must not retain the reference past this
    /// call. Default no-op.
    virtual void on_system_telemetry(const SystemTelemetry &) {}

    /// Leave the app: no allocation, quiesce motion validity.
    virtual void leave() = 0;
};

}  // namespace fluid_demo
