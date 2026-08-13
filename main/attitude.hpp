#pragma once

#include <atomic>
#include <cstdint>

#include "app_types.hpp"

namespace fluid_demo {

/// Board-frame attitude from gyro integration plus accelerometer tilt.
/// Relative mode treats the current pose as identity at reset and supports
/// display gain. GravityAligned mode keeps a one-to-one physical pose:
/// roll/pitch stay tied to gravity and yaw is gyro-relative. USB-at-bottom
/// axis mapping matches MotionFilter.
class AttitudeFilter {
public:
    enum class ReferenceMode : uint8_t {
        Relative,
        GravityAligned,
    };

    static constexpr float kOneG = 9.807f;

    explicit AttitudeFilter(ReferenceMode reference_mode = ReferenceMode::Relative)
        : reference_mode_(reference_mode) {}

    void reset();
    void request_align();
    static void request_yaw(float radians);
    static void set_axes(int sx, int sy, int sz);
    static void set_gain(float gain);  ///< Relative-mode display gain only.
    static void set_tau(float seconds);
    static void axes(int *sx, int *sy, int *sz);
    static float gain();
    static float tau();

    /// Integrate one physical IMU sample. Override gravity is supplied through
    /// `apply_override` instead.
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
    static Quat scale_rotation(const Quat &q, float gain);
    static void quat_to_matrix(const Quat &q, float out[9]);

    bool valid_gravity(const Vec3 &v) const;
    void init_world(const Vec3 &g_meas);
    void pull_toward_gravity(const Vec3 &g_meas, float alpha);
    void apply_yaw(float radians);
    void consume_yaw_request();
    void maybe_axes_realign();
    void recompute();
    void hard_reset();

    const ReferenceMode reference_mode_;

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
    uint32_t axes_gen_ = 1;
    std::atomic<bool> align_pending_{true};
};

}  // namespace fluid_demo
