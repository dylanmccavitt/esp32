#include "attitude.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>

#include "common_math.hpp"

namespace fluid_demo {
namespace {

constexpr float kMinimumDeltaTime = 1e-4f;
constexpr float kMaximumDeltaTime = 0.1f;
constexpr float kMinimumGravityMagnitude = 0.05f * AttitudeFilter::kOneG;
constexpr float kMaximumAccelerationMagnitude = 5.0f * AttitudeFilter::kOneG;
constexpr float kNormalizationEpsilon = 1e-6f;
constexpr float kComplementaryTimeConstant = 0.35f;
constexpr float kOverrideTimeConstant = 0.04f;
constexpr float kMaximumInitialGyroscopeBiasMagnitude = 0.12f;
constexpr float kMaximumGyroscopeBiasAdaptationMagnitude = 0.04f;
constexpr float kGyroscopeBiasTimeConstant = 1.5f;
constexpr float kMinimumDisplayGain = 0.0f;
constexpr float kMaximumDisplayGain = 4.0f;
constexpr float kMinimumCorrectionTimeConstant = 0.05f;
constexpr float kMaximumCorrectionTimeConstant = 2.0f;

constexpr uint32_t kAxisXPositive = 1u << 0;
constexpr uint32_t kAxisYPositive = 1u << 1;
constexpr uint32_t kAxisZPositive = 1u << 2;
constexpr uint32_t kAxisSignMask =
    kAxisXPositive | kAxisYPositive | kAxisZPositive;
constexpr uint32_t kAxisGenerationStep = 1u << 3;
constexpr uint32_t kAxisGenerationMask = ~kAxisSignMask;

constexpr int axis_direction(uint32_t config, uint32_t positive_bit)
{
    return (config & positive_bit) != 0u ? 1 : -1;
}

std::atomic<float> s_yaw_request{0.0f};
std::atomic<uint32_t> s_axes_configuration{kAxisGenerationStep |
                                           kAxisXPositive};
std::atomic<float> s_display_gain{1.0f};
std::atomic<float> s_correction_time_constant{kComplementaryTimeConstant};

}

Vec3 AttitudeFilter::map_to_box_frame(const Vec3 &vector, uint32_t axes_config)
{
    return {
        static_cast<float>(axis_direction(axes_config, kAxisXPositive)) *
            vector.x,
        static_cast<float>(axis_direction(axes_config, kAxisYPositive)) *
            vector.y,
        static_cast<float>(axis_direction(axes_config, kAxisZPositive)) *
            vector.z,
    };
}

bool AttitudeFilter::finite_quaternion(const Quaternion &quaternion)
{
    return std::isfinite(quaternion.w) && std::isfinite(quaternion.x) &&
           std::isfinite(quaternion.y) && std::isfinite(quaternion.z);
}

float AttitudeFilter::vector_length(const Vec3 &vector)
{
    return std::sqrt(vector.x * vector.x + vector.y * vector.y +
                     vector.z * vector.z);
}

Vec3 AttitudeFilter::vector_cross(const Vec3 &left, const Vec3 &right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

float AttitudeFilter::vector_dot(const Vec3 &left, const Vec3 &right)
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

bool AttitudeFilter::normalize_vector(Vec3 &vector)
{
    const float magnitude = vector_length(vector);
    if (!std::isfinite(magnitude) || magnitude < kNormalizationEpsilon) {
        return false;
    }
    vector.x /= magnitude;
    vector.y /= magnitude;
    vector.z /= magnitude;
    return finite_vec(vector);
}

AttitudeFilter::Quaternion
AttitudeFilter::multiply_quaternions(const Quaternion &left,
                                     const Quaternion &right)
{
    return {
        left.w * right.w - left.x * right.x - left.y * right.y -
            left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z -
            left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w +
            left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x +
            left.z * right.w,
    };
}

AttitudeFilter::Quaternion
AttitudeFilter::quaternion_conjugate(const Quaternion &quaternion)
{
    return {
        quaternion.w,
        -quaternion.x,
        -quaternion.y,
        -quaternion.z,
    };
}

bool AttitudeFilter::normalize_quaternion(Quaternion &quaternion)
{
    const float magnitude =
        std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x +
                  quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    if (!std::isfinite(magnitude) || magnitude < kNormalizationEpsilon) {
        return false;
    }
    quaternion.w /= magnitude;
    quaternion.x /= magnitude;
    quaternion.y /= magnitude;
    quaternion.z /= magnitude;
    return finite_quaternion(quaternion);
}

Vec3 AttitudeFilter::rotate(const Quaternion &quaternion, const Vec3 &vector)
{
    const Vec3 rotation_axis{
        quaternion.x,
        quaternion.y,
        quaternion.z,
    };
    const Vec3 first_cross = vector_cross(rotation_axis, vector);
    const Vec3 second_cross = vector_cross(rotation_axis, first_cross);
    return {
        vector.x + 2.0f * (quaternion.w * first_cross.x + second_cross.x),
        vector.y + 2.0f * (quaternion.w * first_cross.y + second_cross.y),
        vector.z + 2.0f * (quaternion.w * first_cross.z + second_cross.z),
    };
}

AttitudeFilter::Quaternion AttitudeFilter::rotation_between(const Vec3 &from,
                                                            const Vec3 &to)
{
    const float dot_product = clamp_float(vector_dot(from, to), -1.0f, 1.0f);
    const Vec3 cross_product = vector_cross(from, to);
    Quaternion rotation{
        dot_product + 1.0f,
        cross_product.x,
        cross_product.y,
        cross_product.z,
    };
    if (normalize_quaternion(rotation)) {
        return rotation;
    }

    Vec3 perpendicular_axis = vector_cross(from, {1.0f, 0.0f, 0.0f});
    if (vector_length(perpendicular_axis) < 1e-3f) {
        perpendicular_axis = vector_cross(from, {0.0f, 1.0f, 0.0f});
    }
    if (!normalize_vector(perpendicular_axis)) {
        return {};
    }
    return {
        0.0f,
        perpendicular_axis.x,
        perpendicular_axis.y,
        perpendicular_axis.z,
    };
}

AttitudeFilter::Quaternion
AttitudeFilter::scale_rotation(const Quaternion &rotation, float gain)
{
    const float clamped_scalar = clamp_float(rotation.w, -1.0f, 1.0f);
    const float half_angle = std::acos(clamped_scalar);
    const float vector_magnitude =
        std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y +
                  rotation.z * rotation.z);
    if (vector_magnitude < kNormalizationEpsilon || half_angle < 1e-6f) {
        return {};
    }
    const float scaled_half_angle = gain * half_angle;
    const float vector_scale = std::sin(scaled_half_angle) / vector_magnitude;
    Quaternion scaled_rotation{
        std::cos(scaled_half_angle),
        rotation.x * vector_scale,
        rotation.y * vector_scale,
        rotation.z * vector_scale,
    };
    if (!normalize_quaternion(scaled_rotation)) {
        return {};
    }
    return scaled_rotation;
}

void AttitudeFilter::quaternion_to_matrix(const Quaternion &quaternion,
                                          float matrix[9])
{
    const float xx = quaternion.x * quaternion.x;
    const float yy = quaternion.y * quaternion.y;
    const float zz = quaternion.z * quaternion.z;
    const float xy = quaternion.x * quaternion.y;
    const float xz = quaternion.x * quaternion.z;
    const float yz = quaternion.y * quaternion.z;
    const float wx = quaternion.w * quaternion.x;
    const float wy = quaternion.w * quaternion.y;
    const float wz = quaternion.w * quaternion.z;
    matrix[0] = 1.0f - 2.0f * (yy + zz);
    matrix[1] = 2.0f * (xy - wz);
    matrix[2] = 2.0f * (xz + wy);
    matrix[3] = 2.0f * (xy + wz);
    matrix[4] = 1.0f - 2.0f * (xx + zz);
    matrix[5] = 2.0f * (yz - wx);
    matrix[6] = 2.0f * (xz - wy);
    matrix[7] = 2.0f * (yz + wx);
    matrix[8] = 1.0f - 2.0f * (xx + yy);
}

bool AttitudeFilter::valid_gravity_sample(const Vec3 &acceleration) const
{
    if (!finite_vec(acceleration)) {
        return false;
    }
    const float magnitude = vector_length(acceleration);
    return magnitude >= kMinimumGravityMagnitude &&
           magnitude <= kMaximumAccelerationMagnitude;
}

void AttitudeFilter::reset_state()
{
    orientation_ = {};
    reference_orientation_ = {};
    rotation_matrix_[0] = 1.0f;
    rotation_matrix_[1] = 0.0f;
    rotation_matrix_[2] = 0.0f;
    rotation_matrix_[3] = 0.0f;
    rotation_matrix_[4] = 1.0f;
    rotation_matrix_[5] = 0.0f;
    rotation_matrix_[6] = 0.0f;
    rotation_matrix_[7] = 0.0f;
    rotation_matrix_[8] = 1.0f;
    world_gravity_ = {0.0f, 0.0f, kOneG};
    mapped_acceleration_ = {};
    pitch_ = 0.0f;
    roll_ = 0.0f;
    yaw_ = 0.0f;
    gyro_bias_ = {};
    has_reference_ = false;
    last_sample_was_accepted_ = false;
    alignment_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::hard_reset()
{
    ++nonfinite_resets_;
    reset_state();
}

void AttitudeFilter::reset()
{
    reset_state();
}

void AttitudeFilter::request_align()
{
    alignment_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::request_yaw(float radians)
{
    if (!std::isfinite(radians)) {
        return;
    }
    s_yaw_request.store(radians, std::memory_order_release);
}

void AttitudeFilter::set_axes(int x_sign, int y_sign, int z_sign)
{
    if ((x_sign != 1 && x_sign != -1) || (y_sign != 1 && y_sign != -1) ||
        (z_sign != 1 && z_sign != -1)) {
        return;
    }
    const uint32_t axis_signs = (x_sign > 0 ? kAxisXPositive : 0u) |
                                (y_sign > 0 ? kAxisYPositive : 0u) |
                                (z_sign > 0 ? kAxisZPositive : 0u);
    uint32_t current_configuration =
        s_axes_configuration.load(std::memory_order_relaxed);
    for (;;) {
        const uint32_t next_generation =
            ((current_configuration & kAxisGenerationMask) +
             kAxisGenerationStep) &
            kAxisGenerationMask;
        const uint32_t next_configuration = next_generation | axis_signs;
        if (s_axes_configuration.compare_exchange_weak(
                current_configuration, next_configuration,
                std::memory_order_release, std::memory_order_relaxed)) {
            return;
        }
    }
}

void AttitudeFilter::set_gain(float gain)
{
    if (!std::isfinite(gain)) {
        return;
    }
    s_display_gain.store(
        clamp_float(gain, kMinimumDisplayGain, kMaximumDisplayGain),
        std::memory_order_relaxed);
}

void AttitudeFilter::set_tau(float seconds)
{
    if (!std::isfinite(seconds)) {
        return;
    }
    s_correction_time_constant.store(
        clamp_float(seconds, kMinimumCorrectionTimeConstant,
                    kMaximumCorrectionTimeConstant),
        std::memory_order_relaxed);
}

void AttitudeFilter::axes(int *x_sign, int *y_sign, int *z_sign)
{
    const uint32_t config =
        s_axes_configuration.load(std::memory_order_acquire);
    if (x_sign != nullptr) {
        *x_sign = axis_direction(config, kAxisXPositive);
    }
    if (y_sign != nullptr) {
        *y_sign = axis_direction(config, kAxisYPositive);
    }
    if (z_sign != nullptr) {
        *z_sign = axis_direction(config, kAxisZPositive);
    }
}

float AttitudeFilter::gain()
{
    return s_display_gain.load(std::memory_order_relaxed);
}

float AttitudeFilter::tau()
{
    return s_correction_time_constant.load(std::memory_order_relaxed);
}

void AttitudeFilter::apply_yaw(float radians)
{
    const float half_angle = 0.5f * radians;
    Quaternion yaw_rotation{
        std::cos(half_angle),
        0.0f,
        std::sin(half_angle),
        0.0f,
    };
    if (!normalize_quaternion(yaw_rotation)) {
        return;
    }
    orientation_ = multiply_quaternions(orientation_, yaw_rotation);
    if (!normalize_quaternion(orientation_)) {
        hard_reset();
        return;
    }
    recompute_outputs();
}

void AttitudeFilter::consume_yaw_request()
{
    const float requested_yaw =
        s_yaw_request.exchange(0.0f, std::memory_order_acq_rel);
    if (requested_yaw == 0.0f || !std::isfinite(requested_yaw) ||
        !has_reference_) {
        return;
    }
    apply_yaw(requested_yaw);
}

void AttitudeFilter::handle_axes_change(uint32_t axes_config)
{
    const uint32_t generation = axes_config & kAxisGenerationMask;
    if (generation != axes_generation_) {
        axes_generation_ = generation;
        alignment_pending_.store(true, std::memory_order_release);
    }
}

void AttitudeFilter::recompute_outputs()
{
    Quaternion display_orientation = multiply_quaternions(
        quaternion_conjugate(reference_orientation_), orientation_);
    if (!normalize_quaternion(display_orientation)) {
        hard_reset();
        return;
    }

    // Scaling absolute orientation aliases distinct physical poses.
    if (reference_mode_ == ReferenceMode::Relative) {
        const float display_gain =
            s_display_gain.load(std::memory_order_relaxed);
        if (std::fabs(display_gain - 1.0f) > 1e-6f) {
            display_orientation =
                scale_rotation(display_orientation, display_gain);
            if (!normalize_quaternion(display_orientation)) {
                hard_reset();
                return;
            }
        }
    }

    quaternion_to_matrix(display_orientation, rotation_matrix_);

    const Vec3 screen_up = rotate(display_orientation, {0.0f, 1.0f, 0.0f});
    if (!finite_vec(screen_up)) {
        hard_reset();
        return;
    }

    const float horizontal_up_magnitude =
        std::sqrt(screen_up.x * screen_up.x + screen_up.y * screen_up.y);
    roll_ = std::atan2(screen_up.x, screen_up.y);
    pitch_ = std::atan2(screen_up.z, horizontal_up_magnitude);
    yaw_ = std::atan2(
        2.0f * (display_orientation.w * display_orientation.y +
                display_orientation.x * display_orientation.z),
        1.0f - 2.0f * (display_orientation.y * display_orientation.y +
                       display_orientation.z * display_orientation.z));
    if (!std::isfinite(roll_)) {
        roll_ = 0.0f;
    }
    if (!std::isfinite(pitch_)) {
        pitch_ = 0.0f;
    }
    if (!std::isfinite(yaw_)) {
        yaw_ = 0.0f;
    }
}

void AttitudeFilter::initialize_world_reference(const Vec3 &measured_gravity)
{
    reference_orientation_ = {};
    if (reference_mode_ == ReferenceMode::Relative) {
        orientation_ = {};
        world_gravity_ = measured_gravity;
    } else {
        Vec3 measured_direction = measured_gravity;
        Vec3 world_gravity_direction{0.0f, 0.0f, 1.0f};
        if (!normalize_vector(measured_direction)) {
            hard_reset();
            return;
        }
        orientation_ =
            rotation_between(measured_direction, world_gravity_direction);
        if (!normalize_quaternion(orientation_)) {
            hard_reset();
            return;
        }
        world_gravity_ = {0.0f, 0.0f, kOneG};
    }
    has_reference_ = true;
    recompute_outputs();
}

void AttitudeFilter::pull_toward_gravity(const Vec3 &measured_gravity,
                                         float correction_gain)
{
    // Shortest-arc gravity correction leaves yaw gyro-only.
    Vec3 measured_direction = measured_gravity;
    Vec3 target_direction = world_gravity_;
    if (!normalize_vector(measured_direction) ||
        !normalize_vector(target_direction)) {
        return;
    }
    Vec3 estimated_direction = rotate(orientation_, measured_direction);
    if (!normalize_vector(estimated_direction)) {
        return;
    }
    const float alignment = vector_dot(estimated_direction, target_direction);
    if (alignment > 0.999999f) {
        return;
    }
    Quaternion correction =
        rotation_between(estimated_direction, target_direction);
    if (correction_gain < 0.999f) {
        correction.w = 1.0f + correction_gain * (correction.w - 1.0f);
        correction.x *= correction_gain;
        correction.y *= correction_gain;
        correction.z *= correction_gain;
        if (!normalize_quaternion(correction)) {
            return;
        }
    }
    orientation_ = multiply_quaternions(correction, orientation_);
    if (!normalize_quaternion(orientation_)) {
        hard_reset();
    }
}

bool AttitudeFilter::update(const Vec3 &accel_mps2, const Vec3 &gyro_rads,
                            float dt)
{
    last_sample_was_accepted_ = false;
    const uint32_t axes_config =
        s_axes_configuration.load(std::memory_order_acquire);
    handle_axes_change(axes_config);
    if (!valid_gravity_sample(accel_mps2) || !finite_vec(gyro_rads) ||
        !std::isfinite(dt) || dt <= 0.0f) {
        return false;
    }

    const Vec3 mapped_acceleration = map_to_box_frame(accel_mps2, axes_config);
    const Vec3 mapped_angular_rate = map_to_box_frame(gyro_rads, axes_config);
    if (!valid_gravity_sample(mapped_acceleration) ||
        !finite_vec(mapped_angular_rate)) {
        return false;
    }

    const float clamped_dt =
        clamp_float(dt, kMinimumDeltaTime, kMaximumDeltaTime);
    const float angular_rate_magnitude = vector_length(mapped_angular_rate);
    mapped_acceleration_ = mapped_acceleration;
    last_sample_was_accepted_ = true;

    const bool alignment_requested =
        alignment_pending_.exchange(false, std::memory_order_acq_rel);
    if (!has_reference_ || alignment_requested) {
        initialize_world_reference(mapped_acceleration);
        gyro_bias_ =
            angular_rate_magnitude < kMaximumInitialGyroscopeBiasMagnitude
                ? mapped_angular_rate
                : Vec3{};
        consume_yaw_request();
        return true;
    }

    Vec3 corrected_angular_rate{
        mapped_angular_rate.x - gyro_bias_.x,
        mapped_angular_rate.y - gyro_bias_.y,
        mapped_angular_rate.z - gyro_bias_.z,
    };
    const float corrected_rate_magnitude =
        vector_length(corrected_angular_rate);
    if (corrected_rate_magnitude < kMaximumGyroscopeBiasAdaptationMagnitude) {
        const float bias_blend =
            1.0f - std::exp(-clamped_dt / kGyroscopeBiasTimeConstant);
        gyro_bias_.x += bias_blend * corrected_angular_rate.x;
        gyro_bias_.y += bias_blend * corrected_angular_rate.y;
        gyro_bias_.z += bias_blend * corrected_angular_rate.z;
        corrected_angular_rate = {
            mapped_angular_rate.x - gyro_bias_.x,
            mapped_angular_rate.y - gyro_bias_.y,
            mapped_angular_rate.z - gyro_bias_.z,
        };
    }

    const float half_step = 0.5f * clamped_dt;
    Quaternion gyroscope_step{
        1.0f,
        corrected_angular_rate.x * half_step,
        corrected_angular_rate.y * half_step,
        corrected_angular_rate.z * half_step,
    };
    if (!normalize_quaternion(gyroscope_step)) {
        hard_reset();
        return false;
    }
    orientation_ = multiply_quaternions(orientation_, gyroscope_step);
    if (!normalize_quaternion(orientation_)) {
        hard_reset();
        return false;
    }

    const float acceleration_magnitude = vector_length(mapped_acceleration);
    const float gravity_error_fraction =
        std::fabs(acceleration_magnitude - kOneG) / kOneG;
    const float gravity_trust =
        1.0f / (1.0f + 8.0f * gravity_error_fraction * gravity_error_fraction);
    const float correction_tau =
        s_correction_time_constant.load(std::memory_order_relaxed);
    const float gravity_correction =
        (1.0f - std::exp(-clamped_dt / correction_tau)) * gravity_trust;
    pull_toward_gravity(mapped_acceleration,
                        clamp_float(gravity_correction, 0.0f, 1.0f));
    consume_yaw_request();
    recompute_outputs();
    if (!finite_quaternion(orientation_)) {
        hard_reset();
        return false;
    }
    return true;
}

bool AttitudeFilter::apply_override(const Vec3 &apparent_acceleration)
{
    last_sample_was_accepted_ = false;
    const uint32_t axes_config =
        s_axes_configuration.load(std::memory_order_acquire);
    handle_axes_change(axes_config);
    if (!has_reference_ ||
        alignment_pending_.load(std::memory_order_acquire) ||
        !valid_gravity_sample(apparent_acceleration)) {
        return false;
    }

    mapped_acceleration_ = apparent_acceleration;
    last_sample_was_accepted_ = true;

    const float correction_gain =
        1.0f - std::exp(-0.01f / kOverrideTimeConstant);
    pull_toward_gravity(apparent_acceleration,
                        clamp_float(correction_gain, 0.0f, 1.0f));
    consume_yaw_request();
    recompute_outputs();
    if (!finite_quaternion(orientation_)) {
        hard_reset();
        return false;
    }
    return true;
}

}
