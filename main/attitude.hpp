#pragma once

#include <atomic>
#include <cstdint>

#include "app_types.hpp"

namespace fluid_demo {

/// Board-frame attitude from gyro integration plus accelerometer tilt.
/// USB-at-bottom axis map matches MotionFilter. `matrix()` is the display
/// pose `q_ref^{-1} * q_body` (body-to-world relative to the last zero).
/// Plus/reset re-inits that world from the current sample so relative R is
/// identity and stays there while held still. Accel corrects tilt of
/// `q_body` only; yaw around gravity is gyro-only. Gyro bias is captured at
/// zero so a still hold does not crawl. `request_yaw` is a one-shot body yaw
/// for host captures.
class AttitudeFilter {
public:
    static constexpr float kOneG = 9.807f;

    AttitudeFilter() = default;

    void reset();
    void request_align();
    static void request_yaw(float radians);

    /// Physical IMU sample. `gyro_rads` is ignored when `freeze_gyro`.
    /// Override gravity is supplied via `apply_override` instead.
    bool update(const Vec3 &accel_mps2, const Vec3 &gyro_rads, float dt);

    /// Treat `apparent_accel` as gravity in box frame and freeze gyro.
    bool apply_override(const Vec3 &apparent_accel);

    const float *matrix() const { return R_; }
    Vec3 up() const { return up_; }
    float pitch() const { return pitch_; }
    float roll() const { return roll_; }
    float yaw() const { return yaw_; }
    float gyro_abs() const { return gyro_abs_; }
    Vec3 mapped_accel() const { return mapped_; }
    Vec3 raw_accel() const { return raw_accel_; }
    bool aligned() const { return have_ref_; }
    bool last_sample_accepted() const { return accepted_last_; }
    uint32_t nonfinite_resets() const { return nonfinite_resets_; }

private:
    struct Quat {
        float w = 1.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    static Vec3 map_box(const Vec3 &v);
    static bool finite_vec(const Vec3 &v);
    static bool finite_quat(const Quat &q);
    static float clampf(float v, float lo, float hi);
    static float vec_length(const Vec3 &v);
    static Vec3 vec_scale(const Vec3 &v, float s);
    static Vec3 vec_cross(const Vec3 &a, const Vec3 &b);
    static float vec_dot(const Vec3 &a, const Vec3 &b);
    static bool vec_normalize(Vec3 &v);
    static Quat quat_mul(const Quat &a, const Quat &b);
    static Quat quat_conj(const Quat &q);
    static bool quat_normalize(Quat &q);
    static Vec3 rotate(const Quat &q, const Vec3 &v);
    static Quat quat_between(const Vec3 &from, const Vec3 &to);

    bool valid_gravity(const Vec3 &v) const;
    void init_world(const Vec3 &g_meas);
    void pull_toward_gravity(const Vec3 &g_meas, float alpha);
    void apply_yaw(float radians);
    void consume_yaw_request();
    void recompute();
    void hard_reset();

    Quat q_{};
    Quat q_ref_{};
    float R_[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    Vec3 g_world_{0.0f, -kOneG, 0.0f};
    Vec3 up_{0.0f, 1.0f, 0.0f};
    Vec3 mapped_{};
    Vec3 raw_accel_{};
    Vec3 gyro_bias_{};
    float pitch_ = 0.0f;
    float roll_ = 0.0f;
    float yaw_ = 0.0f;
    float gyro_abs_ = 0.0f;
    bool have_ref_ = false;
    bool accepted_last_ = false;
    uint32_t nonfinite_resets_ = 0;
    std::atomic<bool> align_pending_{true};
};

}  // namespace fluid_demo
