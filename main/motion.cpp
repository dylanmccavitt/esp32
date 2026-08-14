#include "motion.hpp"

#include <cmath>

#include "common_math.hpp"

namespace fluid_demo {
namespace {

constexpr float kSimulationGravity = 9.0f;
constexpr float kStandardGravity = 9.807f;
constexpr float kLowPassTimeConstant = 0.08f;
constexpr float kShakeGain = 0.5f;
constexpr float kMaximumShakeMagnitude = 1.25f * kSimulationGravity;
constexpr float kMaximumOutputMagnitude = 2.25f * kSimulationGravity;
constexpr float kMinimumDeltaTime = 1e-4f;
constexpr float kMaximumDeltaTime = 0.1f;
constexpr float kMinimumGravityMagnitude = 0.05f * kStandardGravity;
constexpr float kMaximumAccelerationMagnitude = 5.0f * kStandardGravity;
constexpr float kNormalizationEpsilon = 1e-6f;

}

bool MotionFilter::valid_sample(const Vec3 &acceleration)
{
    if (!finite_vec(acceleration)) {
        return false;
    }
    const float magnitude = std::sqrt(acceleration.x * acceleration.x +
                                      acceleration.y * acceleration.y +
                                      acceleration.z * acceleration.z);
    return magnitude >= kMinimumGravityMagnitude &&
           magnitude <= kMaximumAccelerationMagnitude;
}

void MotionFilter::reset()
{
    low_pass_acceleration_ = {};
    gravity_ = {};
    output_ = {};
    initialized_ = false;
    last_sample_was_accepted_ = false;
}

Vec3 MotionFilter::update(const Vec3 &acceleration_mps2, float dt)
{
    last_sample_was_accepted_ = false;
    if (!valid_sample(acceleration_mps2)) {
        return output_;
    }
    last_sample_was_accepted_ = true;

    // USB-C at screen bottom: sensor +Y maps up and display X is reversed.
    const Vec3 mapped_acceleration{
        acceleration_mps2.x,
        -acceleration_mps2.y,
        -acceleration_mps2.z,
    };

    const float clamped_dt =
        clamp_float(dt, kMinimumDeltaTime, kMaximumDeltaTime);
    const float new_sample_weight =
        1.0f - std::exp(-clamped_dt / kLowPassTimeConstant);
    const float previous_sample_weight = 1.0f - new_sample_weight;

    if (!initialized_) {
        low_pass_acceleration_ = mapped_acceleration;
        initialized_ = true;

        const float magnitude =
            std::sqrt(mapped_acceleration.x * mapped_acceleration.x +
                      mapped_acceleration.y * mapped_acceleration.y +
                      mapped_acceleration.z * mapped_acceleration.z);
        if (magnitude > kNormalizationEpsilon) {
            const float normalization_scale = kSimulationGravity / magnitude;
            gravity_ = {
                mapped_acceleration.x * normalization_scale,
                mapped_acceleration.y * normalization_scale,
                mapped_acceleration.z * normalization_scale,
            };
        } else {
            gravity_ = {0.0f, 0.0f, kSimulationGravity};
        }
        output_ = gravity_;
        return output_;
    }

    const Vec3 high_pass_acceleration{
        mapped_acceleration.x - low_pass_acceleration_.x,
        mapped_acceleration.y - low_pass_acceleration_.y,
        mapped_acceleration.z - low_pass_acceleration_.z,
    };

    low_pass_acceleration_.x =
        previous_sample_weight * low_pass_acceleration_.x +
        new_sample_weight * mapped_acceleration.x;
    low_pass_acceleration_.y =
        previous_sample_weight * low_pass_acceleration_.y +
        new_sample_weight * mapped_acceleration.y;
    low_pass_acceleration_.z =
        previous_sample_weight * low_pass_acceleration_.z +
        new_sample_weight * mapped_acceleration.z;

    const float gravity_magnitude =
        std::sqrt(low_pass_acceleration_.x * low_pass_acceleration_.x +
                  low_pass_acceleration_.y * low_pass_acceleration_.y +
                  low_pass_acceleration_.z * low_pass_acceleration_.z);
    if (gravity_magnitude > kNormalizationEpsilon) {
        const float normalization_scale =
            kSimulationGravity / gravity_magnitude;
        gravity_ = {
            low_pass_acceleration_.x * normalization_scale,
            low_pass_acceleration_.y * normalization_scale,
            low_pass_acceleration_.z * normalization_scale,
        };
    }

    const float acceleration_scale =
        kShakeGain * kSimulationGravity / kStandardGravity;
    Vec3 shake{
        high_pass_acceleration.x * acceleration_scale,
        high_pass_acceleration.y * acceleration_scale,
        high_pass_acceleration.z * acceleration_scale,
    };
    const float shake_magnitude =
        std::sqrt(shake.x * shake.x + shake.y * shake.y + shake.z * shake.z);
    if (shake_magnitude > kMaximumShakeMagnitude) {
        const float clamp_scale = kMaximumShakeMagnitude / shake_magnitude;
        shake = {
            shake.x * clamp_scale,
            shake.y * clamp_scale,
            shake.z * clamp_scale,
        };
    }

    Vec3 candidate_output{
        gravity_.x + shake.x,
        gravity_.y + shake.y,
        gravity_.z + shake.z,
    };
    const float output_magnitude =
        std::sqrt(candidate_output.x * candidate_output.x +
                  candidate_output.y * candidate_output.y +
                  candidate_output.z * candidate_output.z);
    if (output_magnitude > kMaximumOutputMagnitude) {
        const float clamp_scale = kMaximumOutputMagnitude / output_magnitude;
        candidate_output = {
            candidate_output.x * clamp_scale,
            candidate_output.y * clamp_scale,
            candidate_output.z * clamp_scale,
        };
    }

    if (!finite_vec(candidate_output)) {
        last_sample_was_accepted_ = false;
        return output_;
    }

    output_ = candidate_output;
    return output_;
}

}
