#pragma once

#include <atomic>
#include <cstdint>

#include "app_types.hpp"

namespace fluid_demo {

class AttitudeFilter {
public:
    enum class ReferenceMode : uint8_t {
        Relative,
        GravityAligned,
    };

    static constexpr float kOneG = 9.807f;

    explicit AttitudeFilter(
        ReferenceMode reference_mode = ReferenceMode::Relative)
        : reference_mode_(reference_mode)
    {
    }

    void reset();
    void request_align();
    static void request_yaw(float radians);
    static void set_axes(int x_sign, int y_sign, int z_sign);
    static void set_gain(float gain);
    static void set_tau(float seconds);
    static void axes(int *x_sign, int *y_sign, int *z_sign);
    static float gain();
    static float tau();

    bool update(const Vec3 &accel_mps2, const Vec3 &gyro_rads, float dt);

    bool apply_override(const Vec3 &apparent_acceleration);

    const float *rotation_matrix() const { return rotation_matrix_; }
    float pitch() const { return pitch_; }
    float roll() const { return roll_; }
    float yaw() const { return yaw_; }
    Vec3 mapped_acceleration() const { return mapped_acceleration_; }
    bool aligned() const { return has_reference_; }
    bool last_sample_accepted() const { return last_sample_was_accepted_; }
    uint32_t nonfinite_resets() const { return nonfinite_resets_; }

private:
    struct Quaternion {
        float w = 1.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    static Vec3 map_to_box_frame(const Vec3 &vector, uint32_t axes_config);
    static bool finite_quaternion(const Quaternion &quaternion);
    static float vector_length(const Vec3 &vector);
    static Vec3 vector_cross(const Vec3 &left, const Vec3 &right);
    static float vector_dot(const Vec3 &left, const Vec3 &right);
    static bool normalize_vector(Vec3 &vector);
    static Quaternion multiply_quaternions(const Quaternion &left,
                                           const Quaternion &right);
    static Quaternion quaternion_conjugate(const Quaternion &quaternion);
    static bool normalize_quaternion(Quaternion &quaternion);
    static Vec3 rotate(const Quaternion &quaternion, const Vec3 &vector);
    static Quaternion rotation_between(const Vec3 &from, const Vec3 &to);
    static Quaternion scale_rotation(const Quaternion &rotation, float gain);
    static void quaternion_to_matrix(const Quaternion &quaternion,
                                     float matrix[9]);

    bool valid_gravity_sample(const Vec3 &acceleration) const;
    void initialize_world_reference(const Vec3 &measured_gravity);
    void pull_toward_gravity(const Vec3 &measured_gravity,
                             float correction_gain);
    void apply_yaw(float radians);
    void consume_yaw_request();
    void handle_axes_change(uint32_t axes_config);
    void recompute_outputs();
    void hard_reset();
    void reset_state();

    const ReferenceMode reference_mode_;

    Quaternion orientation_{};
    Quaternion reference_orientation_{};
    float rotation_matrix_[9] = {
        1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    };
    Vec3 world_gravity_{0.0f, 0.0f, kOneG};
    Vec3 mapped_acceleration_{};
    Vec3 gyro_bias_{};
    float pitch_ = 0.0f;
    float roll_ = 0.0f;
    float yaw_ = 0.0f;
    bool has_reference_ = false;
    bool last_sample_was_accepted_ = false;
    uint32_t nonfinite_resets_ = 0;
    uint32_t axes_generation_ = 1u << 3;
    std::atomic<bool> alignment_pending_{true};
};

}
