#include "motion.hpp"

#include <cmath>

namespace fluid_demo {
namespace {

constexpr float kLowpassTauSec = 0.08f;
constexpr float kShakeGain = 0.5f;

constexpr float kShakeClampSim = 1.25f * MotionFilter::kSimG;
constexpr float kTotalClampSim = 2.25f * MotionFilter::kSimG;

constexpr float kDtMin = 1e-4f;
constexpr float kDtMax = 0.1f;
constexpr float kMinGravityMag = 0.05f * MotionFilter::kOneG;
constexpr float kMaxAccelMag = 5.0f * MotionFilter::kOneG;
constexpr float kEps = 1e-6f;

}  // namespace

float MotionFilter::clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

bool MotionFilter::finite(const Vec3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool MotionFilter::valid_sample(const Vec3 &accel)
{
    if (!finite(accel)) {
        return false;
    }
    const float mag = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    return mag >= kMinGravityMag && mag <= kMaxAccelMag;
}

void MotionFilter::reset()
{
    low_pass_state_ = Vec3{};
    gravity_ = Vec3{};
    shake_ = Vec3{};
    out_ = Vec3{};
    mapped_ = Vec3{};
    raw_accel_ = Vec3{};
    gyro_rads_ = Vec3{};
    initialized_ = false;
    accepted_last_ = false;
}

Vec3 MotionFilter::update(const Vec3 &accel_mps2, const Vec3 &gyro_rads, float dt)
{
    accepted_last_ = false;
    if (!valid_sample(accel_mps2)) {
        return out_;
    }
    accepted_last_ = true;

    // Live calibration with USB-C at screen-bottom: upright reads +sensor-y.
    // The viewed ST7789 x scan direction is reversed, so sensor x keeps its sign.
    const Vec3 mapped = {
        accel_mps2.x,
        -accel_mps2.y,
        -accel_mps2.z,
    };

    const float clamped_dt = clampf(dt, kDtMin, kDtMax);
    const float blend = 1.0f - std::exp(-clamped_dt / kLowpassTauSec);
    const float retained = 1.0f - blend;

    if (!initialized_) {
        low_pass_state_ = mapped;
        raw_accel_ = accel_mps2;
        mapped_ = mapped;
        gyro_rads_ = gyro_rads;
        initialized_ = true;

        const float magnitude =
            std::sqrt(mapped.x * mapped.x + mapped.y * mapped.y +
                      mapped.z * mapped.z);
        if (magnitude > kEps) {
            const float scale = kSimG / magnitude;
            gravity_ =
                Vec3{mapped.x * scale, mapped.y * scale, mapped.z * scale};
        } else {
            gravity_ = Vec3{0.0f, 0.0f, kSimG};  // screen-up fallback
        }
        shake_ = Vec3{};
        out_ = gravity_;
        return out_;
    }

    const Vec3 high_pass = {
        mapped.x - low_pass_state_.x,
        mapped.y - low_pass_state_.y,
        mapped.z - low_pass_state_.z,
    };

    low_pass_state_.x = retained * low_pass_state_.x + blend * mapped.x;
    low_pass_state_.y = retained * low_pass_state_.y + blend * mapped.y;
    low_pass_state_.z = retained * low_pass_state_.z + blend * mapped.z;

    const float gravity_magnitude =
        std::sqrt(low_pass_state_.x * low_pass_state_.x +
                  low_pass_state_.y * low_pass_state_.y +
                  low_pass_state_.z * low_pass_state_.z);
    if (gravity_magnitude > kEps) {
        const float scale = kSimG / gravity_magnitude;
        gravity_ = Vec3{low_pass_state_.x * scale,
                        low_pass_state_.y * scale,
                        low_pass_state_.z * scale};
    }

    const float scale = kShakeGain * kSimG / kOneG;
    const float shake_x = high_pass.x * scale;
    const float shake_y = high_pass.y * scale;
    const float shake_z = high_pass.z * scale;
    const float shake_magnitude =
        std::sqrt(shake_x * shake_x + shake_y * shake_y + shake_z * shake_z);
    if (shake_magnitude > kShakeClampSim) {
        const float clamp_scale = kShakeClampSim / shake_magnitude;
        shake_ = Vec3{shake_x * clamp_scale,
                      shake_y * clamp_scale,
                      shake_z * clamp_scale};
    } else {
        shake_ = Vec3{shake_x, shake_y, shake_z};
    }

    Vec3 out = {
        gravity_.x + shake_.x,
        gravity_.y + shake_.y,
        gravity_.z + shake_.z,
    };
    const float output_magnitude =
        std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
    if (output_magnitude > kTotalClampSim) {
        const float clamp_scale = kTotalClampSim / output_magnitude;
        out = Vec3{out.x * clamp_scale,
                   out.y * clamp_scale,
                   out.z * clamp_scale};
    }

    if (!finite(out)) {
        accepted_last_ = false;
        return out_;
    }

    raw_accel_ = accel_mps2;
    mapped_ = mapped;
    if (finite(gyro_rads)) {
        gyro_rads_ = gyro_rads;
    }
    out_ = out;
    return out_;
}

}  // namespace fluid_demo
