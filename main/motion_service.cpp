// motion_service.cpp — shell-owned raw-motion pipeline.
//
// All behaviors moved verbatim from the legacy sensor task's poll block
// (app_main.cpp:298-341) and dev_console.cpp's override state (mux protocol
// :91-108, expiry check :356-367). The dt anchor advance is the one change:
// the app's on_motion() acceptance result is fed back via acknowledge(), so a
// rejected fresh sample re-clamps against the previous accepted anchor exactly
// as the legacy `if (on_motion(tick)) last_motion_us = now_us;` did.

#include "motion_service.hpp"

#include <cstdint>

#include "esp_timer.h"

#include "board.hpp"

namespace fluid_demo {

MotionTick MotionService::motion_tick()
{
    const int64_t now_us = esp_timer_get_time();
    last_poll_us_ = now_us;

    MotionTick tick{};
    Vec3 accel_mps2{}, gyro_rads{};
    bool fresh = false;
    last_read_error_ = board_read_motion(&accel_mps2, &gyro_rads, &fresh);
    tick.accel_mps2 = accel_mps2;
    tick.gyro_rads = gyro_rads;
    tick.fresh = (last_read_error_ == ESP_OK) && fresh;
    if (tick.fresh) {
        float dt = static_cast<float>(now_us - last_motion_us_) * 1e-6f;
        if (dt < kMinDt || dt > kMaxDt || dt != dt) {  // clamp; NaN check included
            dt = 1.0f / static_cast<float>(kSensorHz);
        }
        tick.dt = dt;
    }

    // A development-console drive bypasses the physical IMU deterministically.
    // Expiry is evaluated here (dev_console.cpp:356-367) and again under the
    // same mux in override_snapshot() for the console's status read.
    const int64_t override_now_us = esp_timer_get_time();
    portENTER_CRITICAL(&override_mux_);
    if (override_.active && override_.until_us != 0 &&
        override_now_us >= override_.until_us) {
        override_.active = false;
    }
    if (override_.active) {
        tick.apparent_accel = override_.acceleration;
        tick.override_active = true;
    }
    portEXIT_CRITICAL(&override_mux_);
    return tick;
}

void MotionService::acknowledge(bool accepted)
{
    if (accepted) {
        // Advance the anchor only when the app accepted the fresh physical
        // sample; a rejected sample (non-finite/out of range) is re-clamped
        // against the previous accepted anchor on the next attempt.
        last_motion_us_ = last_poll_us_;
    }
}

void MotionService::set_override(const Vec3 &acceleration, uint32_t duration_ms)
{
    const int64_t until_us =
        duration_ms == 0 ? 0 : esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
    portENTER_CRITICAL(&override_mux_);
    override_.acceleration = acceleration;
    override_.until_us = until_us;
    override_.active = true;
    portEXIT_CRITICAL(&override_mux_);
}

void MotionService::clear_override()
{
    portENTER_CRITICAL(&override_mux_);
    override_.active = false;
    override_.until_us = 0;
    portEXIT_CRITICAL(&override_mux_);
}

MotionService::OverrideSnapshot MotionService::override_snapshot()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&override_mux_);
    if (override_.active && override_.until_us != 0 && now_us >= override_.until_us) {
        override_.active = false;
    }
    OverrideSnapshot out;
    out.acceleration = override_.acceleration;
    out.active = override_.active;
    portEXIT_CRITICAL(&override_mux_);
    return out;
}

}  // namespace fluid_demo
