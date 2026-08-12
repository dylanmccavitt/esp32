#pragma once

#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"

namespace fluid_demo {

/// Polls the IMU and applies the development-console motion override.
class MotionService {
public:
    static constexpr uint32_t kSensorHz = 100;
    static constexpr float kMinDt = 0.002f;
    static constexpr float kMaxDt = 0.100f;

    /// Expiry-evaluated override view for the console `status` line.
    struct OverrideSnapshot {
        Vec3 acceleration{0.0f, 0.0f, 6.0f};
        bool active = false;
    };

    /// The caller must acknowledge whether the app accepted the sample.
    MotionTick motion_tick();

    void acknowledge(bool accepted);

    void set_override(const Vec3 &acceleration, uint32_t duration_ms);
    void clear_override();

    OverrideSnapshot override_snapshot();

    esp_err_t last_read_error() const { return last_read_error_; }

private:
    portMUX_TYPE override_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct OverrideState {
        Vec3 acceleration{0.0f, 0.0f, 6.0f};
        int64_t until_us = 0;
        bool active = false;
    } override_;

    int64_t last_poll_us_ = 0;
    int64_t last_motion_us_ = 0;
    esp_err_t last_read_error_ = ESP_OK;
};

}  // namespace fluid_demo
