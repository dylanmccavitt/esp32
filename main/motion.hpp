#pragma once

#include "app_types.hpp"

namespace fluid_demo {

class MotionFilter {
public:
    MotionFilter() = default;

    void reset();

    // Rejected samples leave the previous output unchanged.
    Vec3 update(const Vec3 &acceleration_mps2, float dt);
    bool last_sample_accepted() const { return last_sample_was_accepted_; }

private:
    static bool valid_sample(const Vec3 &acceleration);

    Vec3 low_pass_acceleration_{};
    Vec3 gravity_{};
    Vec3 output_{};
    bool initialized_ = false;
    bool last_sample_was_accepted_ = false;
};

}
