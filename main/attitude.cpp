#include "attitude.hpp"

#include <cmath>

namespace fluid_demo {
namespace {

constexpr float kDtMin = 1e-4f;
constexpr float kDtMax = 0.1f;
constexpr float kMinGravityMag = 0.05f * AttitudeFilter::kOneG;
constexpr float kMaxAccelMag = 5.0f * AttitudeFilter::kOneG;
constexpr float kEps = 1e-6f;
constexpr float kComplementaryTau = 0.35f;
constexpr float kOverrideTau = 0.04f;

}  // namespace

Vec3 AttitudeFilter::map_box(const Vec3 &v)
{
    return {v.x, -v.y, -v.z};
}

bool AttitudeFilter::finite_vec(const Vec3 &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool AttitudeFilter::finite_quat(const Quat &q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z);
}

float AttitudeFilter::clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

float AttitudeFilter::vec_length(const Vec3 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 AttitudeFilter::vec_scale(const Vec3 &v, float s)
{
    return {v.x * s, v.y * s, v.z * s};
}

Vec3 AttitudeFilter::vec_cross(const Vec3 &a, const Vec3 &b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float AttitudeFilter::vec_dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

bool AttitudeFilter::vec_normalize(Vec3 &v)
{
    const float mag = vec_length(v);
    if (!std::isfinite(mag) || mag < kEps) {
        return false;
    }
    v.x /= mag;
    v.y /= mag;
    v.z /= mag;
    return finite_vec(v);
}

AttitudeFilter::Quat AttitudeFilter::quat_mul(const Quat &a, const Quat &b)
{
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

bool AttitudeFilter::quat_normalize(Quat &q)
{
    const float mag = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!std::isfinite(mag) || mag < kEps) {
        return false;
    }
    q.w /= mag;
    q.x /= mag;
    q.y /= mag;
    q.z /= mag;
    return finite_quat(q);
}

Vec3 AttitudeFilter::rotate(const Quat &q, const Vec3 &v)
{
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 uv = vec_cross(u, v);
    const Vec3 uuv = vec_cross(u, uv);
    return {v.x + 2.0f * (q.w * uv.x + uuv.x),
            v.y + 2.0f * (q.w * uv.y + uuv.y),
            v.z + 2.0f * (q.w * uv.z + uuv.z)};
}

AttitudeFilter::Quat AttitudeFilter::quat_between(const Vec3 &from, const Vec3 &to)
{
    const float d = vec_dot(from, to);
    const Vec3 c = vec_cross(from, to);
    if (d < -0.999999f) {
        Vec3 axis = vec_cross(from, {1.0f, 0.0f, 0.0f});
        if (vec_length(axis) < 1e-3f) {
            axis = vec_cross(from, {0.0f, 1.0f, 0.0f});
        }
        if (!vec_normalize(axis)) {
            return {};
        }
        return {0.0f, axis.x, axis.y, axis.z};
    }
    Quat q{d + 1.0f, c.x, c.y, c.z};
    if (!quat_normalize(q)) {
        return {};
    }
    return q;
}

bool AttitudeFilter::valid_gravity(const Vec3 &v) const
{
    if (!finite_vec(v)) {
        return false;
    }
    const float mag = vec_length(v);
    return mag >= kMinGravityMag && mag <= kMaxAccelMag;
}

void AttitudeFilter::hard_reset()
{
    q_ = {};
    R_[0] = 1.0f;
    R_[1] = 0.0f;
    R_[2] = 0.0f;
    R_[3] = 0.0f;
    R_[4] = 1.0f;
    R_[5] = 0.0f;
    R_[6] = 0.0f;
    R_[7] = 0.0f;
    R_[8] = 1.0f;
    g_ref_ = {0.0f, -kOneG, 0.0f};
    up_ = {0.0f, 1.0f, 0.0f};
    mapped_ = {};
    raw_accel_ = {};
    pitch_ = 0.0f;
    roll_ = 0.0f;
    yaw_ = 0.0f;
    gyro_abs_ = 0.0f;
    have_ref_ = false;
    accepted_last_ = false;
    ++nonfinite_resets_;
    align_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::reset()
{
    q_ = {};
    R_[0] = 1.0f;
    R_[1] = 0.0f;
    R_[2] = 0.0f;
    R_[3] = 0.0f;
    R_[4] = 1.0f;
    R_[5] = 0.0f;
    R_[6] = 0.0f;
    R_[7] = 0.0f;
    R_[8] = 1.0f;
    g_ref_ = {0.0f, -kOneG, 0.0f};
    up_ = {0.0f, 1.0f, 0.0f};
    mapped_ = {};
    raw_accel_ = {};
    pitch_ = 0.0f;
    roll_ = 0.0f;
    yaw_ = 0.0f;
    gyro_abs_ = 0.0f;
    have_ref_ = false;
    accepted_last_ = false;
    align_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::request_align()
{
    align_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::recompute()
{
    const float xx = q_.x * q_.x;
    const float yy = q_.y * q_.y;
    const float zz = q_.z * q_.z;
    const float xy = q_.x * q_.y;
    const float xz = q_.x * q_.z;
    const float yz = q_.y * q_.z;
    const float wx = q_.w * q_.x;
    const float wy = q_.w * q_.y;
    const float wz = q_.w * q_.z;
    R_[0] = 1.0f - 2.0f * (yy + zz);
    R_[1] = 2.0f * (xy - wz);
    R_[2] = 2.0f * (xz + wy);
    R_[3] = 2.0f * (xy + wz);
    R_[4] = 1.0f - 2.0f * (xx + zz);
    R_[5] = 2.0f * (yz - wx);
    R_[6] = 2.0f * (xz - wy);
    R_[7] = 2.0f * (yz + wx);
    R_[8] = 1.0f - 2.0f * (xx + yy);

    Vec3 g_ref = g_ref_;
    if (!vec_normalize(g_ref)) {
        hard_reset();
        return;
    }
    up_ = rotate(q_, {-g_ref.x, -g_ref.y, -g_ref.z});
    if (!finite_vec(up_)) {
        hard_reset();
        return;
    }

    const float uxy = std::sqrt(up_.x * up_.x + up_.y * up_.y);
    roll_ = std::atan2(up_.x, up_.y);
    pitch_ = std::atan2(up_.z, uxy);
    yaw_ = std::atan2(2.0f * (q_.w * q_.y + q_.x * q_.z),
                      1.0f - 2.0f * (q_.y * q_.y + q_.z * q_.z));
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

void AttitudeFilter::set_identity_from_gravity(const Vec3 &g_meas)
{
    q_ = {};
    g_ref_ = g_meas;
    have_ref_ = true;
    gyro_abs_ = 0.0f;
    recompute();
}

void AttitudeFilter::pull_toward_gravity(const Vec3 &g_meas, float alpha)
{
    Vec3 g_est = rotate(q_, g_ref_);
    Vec3 meas = g_meas;
    if (!vec_normalize(g_est) || !vec_normalize(meas)) {
        return;
    }
    const float d = vec_dot(g_est, meas);
    if (d > 0.999999f) {
        return;
    }
    Quat delta = quat_between(g_est, meas);
    if (alpha < 0.999f) {
        delta.w = 1.0f + alpha * (delta.w - 1.0f);
        delta.x *= alpha;
        delta.y *= alpha;
        delta.z *= alpha;
        if (!quat_normalize(delta)) {
            return;
        }
    }
    q_ = quat_mul(delta, q_);
    if (!quat_normalize(q_)) {
        hard_reset();
    }
}

bool AttitudeFilter::update(const Vec3 &accel_mps2, const Vec3 &gyro_rads, float dt)
{
    accepted_last_ = false;
    if (!valid_gravity(accel_mps2) || !finite_vec(gyro_rads) || !std::isfinite(dt) ||
        dt <= 0.0f) {
        return false;
    }

    const Vec3 mapped = map_box(accel_mps2);
    const Vec3 gyro = map_box(gyro_rads);
    if (!valid_gravity(mapped) || !finite_vec(gyro)) {
        return false;
    }

    const float clamped_dt = clampf(dt, kDtMin, kDtMax);
    gyro_abs_ = vec_length(gyro);
    mapped_ = mapped;
    raw_accel_ = accel_mps2;
    accepted_last_ = true;

    const bool align = align_pending_.exchange(false, std::memory_order_acq_rel);
    if (align || !have_ref_) {
        set_identity_from_gravity(mapped);
        return true;
    }

    const float half = 0.5f * clamped_dt;
    Quat dq{1.0f, gyro.x * half, gyro.y * half, gyro.z * half};
    if (!quat_normalize(dq)) {
        hard_reset();
        return false;
    }
    q_ = quat_mul(q_, dq);
    if (!quat_normalize(q_)) {
        hard_reset();
        return false;
    }

    const float mag = vec_length(mapped);
    const float g_err = std::fabs(mag - kOneG) / kOneG;
    const float trust = 1.0f / (1.0f + 8.0f * g_err * g_err);
    const float alpha = (1.0f - std::exp(-clamped_dt / kComplementaryTau)) * trust;
    pull_toward_gravity(mapped, clampf(alpha, 0.0f, 1.0f));
    recompute();
    if (!finite_quat(q_) || !finite_vec(up_)) {
        hard_reset();
        return false;
    }
    return true;
}

bool AttitudeFilter::apply_override(const Vec3 &apparent_accel)
{
    accepted_last_ = false;
    if (!valid_gravity(apparent_accel)) {
        return false;
    }

    mapped_ = apparent_accel;
    gyro_abs_ = 0.0f;
    accepted_last_ = true;

    const bool align = align_pending_.exchange(false, std::memory_order_acq_rel);
    if (align || !have_ref_) {
        set_identity_from_gravity(apparent_accel);
        return true;
    }

    const float alpha = 1.0f - std::exp(-0.01f / kOverrideTau);
    pull_toward_gravity(apparent_accel, clampf(alpha, 0.0f, 1.0f));
    recompute();
    if (!finite_quat(q_) || !finite_vec(up_)) {
        hard_reset();
        return false;
    }
    return true;
}

}  // namespace fluid_demo
