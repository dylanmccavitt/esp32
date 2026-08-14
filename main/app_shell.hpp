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

enum class TouchPhase : uint8_t {
    Begin = 0,
    Move = 1,
    End = 2,
};

enum class TouchGesture : uint8_t {
    None = 0,
    SwipeLeft = 1,
    SwipeRight = 2,
};

struct TouchEvent {
    uint16_t x = 0;
    uint16_t y = 0;
    TouchPhase phase = TouchPhase::Begin;
    TouchGesture gesture = TouchGesture::None;
};

struct MotionTick {
    Vec3 accel_mps2{};
    Vec3 gyro_rads{};
    Vec3 apparent_accel{};
    float dt = 0.0f;
    bool fresh = false;
    bool override_active = false;
};

struct AppStats {
    uint32_t count = 0;
    uint32_t epoch = 0;
    uint64_t candidate_checks = 0;
    uint32_t nonfinite_resets = 0;
    uint32_t governor_hits = 0;
    uint32_t physics_us = 0;
    float raw[3] = {0.0f, 0.0f, 0.0f};
    float apparent[3] = {0.0f, 0.0f, 6.0f};
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    uint32_t raster_us = 0;
    uint32_t frame_us = 0;
};

enum class SystemTaskKind : uint8_t {
    Coordinator = 0,
    Sensor = 1,
    Update = 2,
    Render = 3,
    Console = 4,
};

enum class SystemTaskState : uint8_t {
    Running = 0,
    Ready = 1,
    Blocked = 2,
    Suspended = 3,
    Unknown = 4,
};

struct SystemTaskTelemetry {
    SystemTaskKind kind = SystemTaskKind::Coordinator;
    SystemTaskState state = SystemTaskState::Unknown;
    int8_t core_id = -1;
    uint32_t stack_high_water_words = 0;
    bool available = false;
};

struct SystemTelemetry {
    uint32_t generation = 0;
    uint32_t internal_free_bytes = 0;
    uint32_t internal_largest_free_block = 0;
    uint32_t psram_free_bytes = 0;
    SystemTaskTelemetry tasks[5];
};

struct DisplayFrame {
    uint16_t *stripe[2] = {nullptr, nullptr};
    int width = 240;
    int height = 240;
    int stripe_rows = 16;
    int stripe_count = 15;
    void *transport = nullptr;

    struct Ops {
        esp_err_t (*wait_previous)(void *transport) = nullptr;
        void (*latch_capture)(void *transport) = nullptr;
        esp_err_t (*submit)(void *transport, int stripe_index, int stripe_y,
                            int stripe_rows, const uint16_t *pixels) = nullptr;
        esp_err_t (*finish)(void *transport) = nullptr;
        uint32_t (*capture_copy_us)(void *transport) = nullptr;
    } ops;
};

struct LauncherVisual {
    uint16_t background_rgb565 = 0;
    uint16_t band_rgb565 = 0;
    uint16_t affordance_rgb565 = 0;
    uint16_t accent_rgb565 = 0;
    const uint16_t *icon_rgb565 = nullptr;
};

class App {
public:
    static constexpr float kPhysicsDt = 1.0f / 30.0f;

    virtual ~App() = default;
    virtual esp_err_t setup_once() = 0;
    virtual esp_err_t enter() = 0;

    // True when the caller may advance its IMU integration anchor.
    virtual bool on_motion(const MotionTick &tick) = 0;
    virtual void on_plus_press() = 0;
    virtual void on_touch_begin(const TouchEvent &) {}

    // nullptr selects the built-in Fluid Box launcher.
    virtual const LauncherVisual *launcher_visual() const { return nullptr; }

    virtual esp_err_t update(float dt = kPhysicsDt) = 0;
    // True when transport completed a frame.
    virtual bool render(DisplayFrame &frame) = 0;
    virtual AppStats stats() = 0;

    // Called at most once per second for the running generation.
    virtual void on_system_telemetry(const SystemTelemetry &) {}
    virtual void leave() = 0;
};

}
