#pragma once

#include "app_types.hpp"

namespace fluid_demo {

/// Maps IMU acceleration into box coordinates, separates gravity from shake,
/// and clamps both to simulation-safe magnitudes.
class MotionFilter {
public:
    static constexpr float kSimG = 9.0f;
    static constexpr float kOneG = 9.807f;

    MotionFilter() = default;

    void reset();

    /// Invalid samples retain and return the previous output.
    Vec3 update(const Vec3 &accel_mps2, const Vec3 &gyro_rads, float dt);

    Vec3 last_output() const { return out_; }
    Vec3 gravity() const { return gravity_; }
    Vec3 shake() const { return shake_; }
    Vec3 mapped_accel() const { return mapped_; }
    Vec3 raw_accel() const { return raw_accel_; }
    Vec3 gyro() const { return gyro_rads_; }
    bool initialized() const { return initialized_; }
    bool last_sample_accepted() const { return accepted_last_; }

private:
    static float clampf(float v, float lo, float hi);
    static bool finite(const Vec3 &v);
    static bool valid_sample(const Vec3 &accel);

    Vec3 low_pass_state_{};
    Vec3 gravity_{};
    Vec3 shake_{};
    Vec3 out_{};
    Vec3 mapped_{};
    Vec3 raw_accel_{};
    Vec3 gyro_rads_{};
    bool initialized_ = false;
    bool accepted_last_ = false;
};

}  // namespace fluid_demo
