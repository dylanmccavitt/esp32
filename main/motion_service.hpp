#pragma once

#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"

namespace fluid_demo {

class MotionService {
public:
    struct OverrideSnapshot {
        Vec3 acceleration{0.0f, 0.0f, 6.0f};
        bool active = false;
    };

    MotionTick poll();
    void acknowledge_sample(bool sample_accepted);
    void set_override(const Vec3 &acceleration, uint32_t duration_ms);
    void clear_override();
    OverrideSnapshot override_snapshot();
    esp_err_t last_read_error() const { return last_read_error_; }

private:
    static constexpr uint32_t kSensorFrequencyHz = 100;
    static constexpr float kMinimumDeltaTime = 0.002f;
    static constexpr float kMaximumDeltaTime = 0.100f;

    portMUX_TYPE override_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct OverrideState {
        Vec3 acceleration{0.0f, 0.0f, 6.0f};
        int64_t expiration_time_us = 0;
        bool active = false;
    } override_;

    int64_t current_poll_time_us_ = 0;
    int64_t last_accepted_sample_time_us_ = 0;
    esp_err_t last_read_error_ = ESP_OK;
};

}
