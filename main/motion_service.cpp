#include "motion_service.hpp"

#include <cmath>
#include <cstdint>

#include "esp_timer.h"

#include "board.hpp"

namespace fluid_demo {

MotionTick MotionService::poll()
{
    const int64_t poll_time_us = esp_timer_get_time();
    current_poll_time_us_ = poll_time_us;

    MotionTick tick{};
    Vec3 acceleration_mps2{};
    Vec3 angular_rate_rads{};
    bool sample_fresh = false;
    last_read_error_ =
        board_read_motion(acceleration_mps2, angular_rate_rads, sample_fresh);
    tick.accel_mps2 = acceleration_mps2;
    tick.gyro_rads = angular_rate_rads;
    tick.fresh = last_read_error_ == ESP_OK && sample_fresh;
    if (tick.fresh) {
        float sample_delta_seconds =
            static_cast<float>(poll_time_us - last_accepted_sample_time_us_) *
            1e-6f;
        if (!std::isfinite(sample_delta_seconds) ||
            sample_delta_seconds < kMinimumDeltaTime ||
            sample_delta_seconds > kMaximumDeltaTime) {
            sample_delta_seconds =
                1.0f / static_cast<float>(kSensorFrequencyHz);
        }
        tick.dt = sample_delta_seconds;
    }
    const int64_t override_check_time_us = esp_timer_get_time();

    portENTER_CRITICAL(&override_mux_);
    if (override_.active && override_.expiration_time_us != 0 &&
        override_check_time_us >= override_.expiration_time_us) {
        override_.active = false;
    }
    if (override_.active) {
        tick.apparent_accel = override_.acceleration;
        tick.override_active = true;
    }
    portEXIT_CRITICAL(&override_mux_);
    return tick;
}

void MotionService::acknowledge_sample(bool sample_accepted)
{
    if (sample_accepted) {
        last_accepted_sample_time_us_ = current_poll_time_us_;
    }
}

void MotionService::set_override(const Vec3 &acceleration, uint32_t duration_ms)
{
    const int64_t expiration_time_us =
        duration_ms == 0
            ? 0
            : esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
    portENTER_CRITICAL(&override_mux_);
    override_.acceleration = acceleration;
    override_.expiration_time_us = expiration_time_us;
    override_.active = true;
    portEXIT_CRITICAL(&override_mux_);
}

void MotionService::clear_override()
{
    portENTER_CRITICAL(&override_mux_);
    override_.active = false;
    override_.expiration_time_us = 0;
    portEXIT_CRITICAL(&override_mux_);
}

MotionService::OverrideSnapshot MotionService::override_snapshot()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&override_mux_);
    if (override_.active && override_.expiration_time_us != 0 &&
        now_us >= override_.expiration_time_us) {
        override_.active = false;
    }
    OverrideSnapshot snapshot;
    snapshot.acceleration = override_.acceleration;
    snapshot.active = override_.active;
    portEXIT_CRITICAL(&override_mux_);
    return snapshot;
}

}
