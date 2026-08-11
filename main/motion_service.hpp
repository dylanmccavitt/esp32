#pragma once

#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"

namespace fluid_demo {

/// Owns the raw IMU poll, the [2 ms, 100 ms] dt clamp and the dev-console
/// motion override state — everything the legacy sensor task's poll block and
/// dev_console.cpp (:37-108, override expiry :356-367) used to own.
///
/// The dt anchor advances ONLY through an explicit acceptance acknowledgement:
/// the caller feeds this service's MotionTick to the app's on_motion() and
/// passes the boolean result back to acknowledge(). A fresh physical sample
/// accepted by the app's filter advances the anchor; a rejected sample
/// re-clamps against the previous accepted anchor (legacy app_main.cpp:336-
/// 341). An active override still publishes verbatim on rejection — the tick
/// carries override_active/apparent_accel regardless of acknowledgement.
///
/// No allocation; safe to construct as a namespace-scope static.
class MotionService {
public:
    static constexpr uint32_t kSensorHz = 100;  // legacy sensor cadence
    static constexpr float kMinDt = 0.002f;     // legacy dt clamp floor
    static constexpr float kMaxDt = 0.100f;     // legacy dt clamp ceiling

    /// Expiry-evaluated override view for the console `status` line.
    struct OverrideSnapshot {
        Vec3 acceleration{0.0f, 0.0f, 6.0f};
        bool active = false;
    };

    /// Poll one raw IMU sample, compute the clamped dt and fold in an active
    /// (non-expired) override. `fresh` marks a valid read; stale reads keep
    /// the last tick fields and still honor an override. Does NOT advance the
    /// dt anchor — the caller must acknowledge() app acceptance.
    MotionTick motion_tick();

    /// Advance the IMU time anchor to the sample produced by the most recent
    /// motion_tick() iff `accepted`, i.e. the app's on_motion() result. The
    /// anchor therefore advances only when the app accepts a fresh physical
    /// sample.
    void acknowledge(bool accepted);

    /// Console override control; identical mux protocol to dev_console.cpp:91-108.
    void set_override(const Vec3 &acceleration, uint32_t duration_ms);
    void clear_override();

    /// Expiry-evaluated override snapshot for the console `status` command
    /// (mirrors the legacy side-effecting read at dev_console.cpp:128-136).
    OverrideSnapshot override_snapshot();

    /// Last board_read_motion() result. The sensor lane owns the
    /// once-per-second throttled, dump-gated ESP_LOGW (legacy app_main.cpp:
    /// 311-317); this service only reports the error.
    esp_err_t last_read_error() const { return last_read_error_; }

private:
    portMUX_TYPE override_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct OverrideState {
        Vec3 acceleration{0.0f, 0.0f, 6.0f};
        int64_t until_us = 0;  // zero means no automatic expiry
        bool active = false;
    } override_;

    int64_t last_poll_us_ = 0;    // now_us captured by the most recent motion_tick()
    int64_t last_motion_us_ = 0;  // accepted-sample anchor; 0 => first dt clamps
    esp_err_t last_read_error_ = ESP_OK;
};

}  // namespace fluid_demo
