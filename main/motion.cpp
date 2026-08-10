#include "motion.hpp"

#include <cmath>

namespace fluid_demo {
namespace {

// A short low-pass removes hand tremor without lagging deliberate tilt.
constexpr float kLowpassTauSec = 0.08f;
constexpr float kShakeGain = 0.5f;

// Bound shake separately from gravity so vigorous motion remains controlled.
constexpr float kShakeClampSim = 1.25f * MotionFilter::kSimG;
constexpr float kTotalClampSim = 2.25f * MotionFilter::kSimG;

constexpr float kDtMin = 1e-4f;   /* guard against division by tiny/negative dt */
constexpr float kDtMax = 0.1f;    /* filter converges to the new measurement */
constexpr float kMinGravityMag = 0.05f * MotionFilter::kOneG; /* <0.05g: no gravity info */
constexpr float kMaxAccelMag = 5.0f * MotionFilter::kOneG;    /* beyond 4G-range saturation */
constexpr float kEps = 1e-6f;

} // namespace

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
        /* Retain last valid state and output (first valid sample initializes later). */
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

    /* dt-aware one-pole low-pass on the mapped acceleration (m/s^2). */
    const float d = clampf(dt, kDtMin, kDtMax);
    const float alpha = 1.0f - std::exp(-d / kLowpassTauSec);
    const float a = alpha;
    const float ia = 1.0f - alpha;

    if (!initialized_) {
        /* First valid sample seeds the filter: low-pass starts at the measurement,
         * gravity is the normalized direction (or a flat default if the device is
         * in near free fall at boot), no shake. */
        low_pass_state_ = mapped;
        raw_accel_ = accel_mps2;
        mapped_ = mapped;
        gyro_rads_ = gyro_rads;
        initialized_ = true;

        const float mag = std::sqrt(mapped.x * mapped.x + mapped.y * mapped.y + mapped.z * mapped.z);
        if (mag > kEps) {
            const float s = kSimG / mag;
            gravity_ = Vec3{mapped.x * s, mapped.y * s, mapped.z * s};
        } else {
            gravity_ = Vec3{0.0f, 0.0f, kSimG};  // screen-up fallback
        }
        shake_ = Vec3{};
        out_ = gravity_;
        return out_;
    }

    /* Shake is the high-pass of the same one-pole pair: current mapped minus the
     * previous low-pass state (computed before the state advances). */
    Vec3 hp = {
        mapped.x - low_pass_state_.x,
        mapped.y - low_pass_state_.y,
        mapped.z - low_pass_state_.z,
    };

    /* Low-pass state advance. */
    low_pass_state_.x = ia * low_pass_state_.x + a * mapped.x;
    low_pass_state_.y = ia * low_pass_state_.y + a * mapped.y;
    low_pass_state_.z = ia * low_pass_state_.z + a * mapped.z;

    /* Direction-preserving normalization of the filtered gravity to |gravity| = kSimG.
     * Scaling (never per-axis clamping) means the z component is preserved. */
    const float gm = std::sqrt(low_pass_state_.x * low_pass_state_.x +
                               low_pass_state_.y * low_pass_state_.y +
                               low_pass_state_.z * low_pass_state_.z);
    if (gm > kEps) {
        const float s = kSimG / gm;
        gravity_ = Vec3{low_pass_state_.x * s, low_pass_state_.y * s, low_pass_state_.z * s};
    }
    /* else: near free fall (|lp| ~ 0): keep the previous gravity direction. */

    // High-pass shake is deliberately attenuated: orientation supplies the
    // sustained force while translation contributes a short, bounded impulse.
    const float scale = kShakeGain * kSimG / kOneG;
    const float hx = hp.x * scale;
    const float hy = hp.y * scale;
    const float hz = hp.z * scale;
    const float hm = std::sqrt(hx * hx + hy * hy + hz * hz);
    if (hm > kShakeClampSim) {
        const float s = kShakeClampSim / hm;
        shake_ = Vec3{hx * s, hy * s, hz * s};
    } else {
        shake_ = Vec3{hx, hy, hz};
    }

    /* Total apparent acceleration = gravity + shake, magnitude-clamped to a safe
     * finite bound. Magnitude-only clamping preserves z. */
    Vec3 out = {
        gravity_.x + shake_.x,
        gravity_.y + shake_.y,
        gravity_.z + shake_.z,
    };
    const float om = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
    if (om > kTotalClampSim) {
        const float s = kTotalClampSim / om;
        out = Vec3{out.x * s, out.y * s, out.z * s};
    }

    /* Belt-and-braces finite guard; if clamping ever produced a degenerate value,
     * retain the last valid output instead of poisoning the physics. */
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

} // namespace fluid_demo
