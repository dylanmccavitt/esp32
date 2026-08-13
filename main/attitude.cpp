#include "attitude.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>

namespace fluid_demo {
namespace {

constexpr float kDtMin = 1e-4f;
constexpr float kDtMax = 0.1f;
constexpr float kMinGravityMag = 0.05f * AttitudeFilter::kOneG;
constexpr float kMaxAccelMag = 5.0f * AttitudeFilter::kOneG;
constexpr float kEps = 1e-6f;
constexpr float kComplementaryTau = 0.35f;
constexpr float kOverrideTau = 0.04f;
constexpr float kGyroBiasStill = 0.12f;
constexpr float kGyroBiasTau = 1.5f;
constexpr float kGainMin = 0.0f;
constexpr float kGainMax = 4.0f;
constexpr float kTauMin = 0.05f;
constexpr float kTauMax = 2.0f;

std::atomic<float> s_yaw_request{0.0f};
std::atomic<int> s_sx{1};
std::atomic<int> s_sy{-1};
std::atomic<int> s_sz{-1};
std::atomic<float> s_gain{1.0f};
std::atomic<float> s_tau{kComplementaryTau};
std::atomic<uint32_t> s_axes_gen{1};

}  // namespace

Vec3 AttitudeFilter::map_box(const Vec3 &v)
{
    return {static_cast<float>(s_sx.load(std::memory_order_relaxed)) * v.x,
            static_cast<float>(s_sy.load(std::memory_order_relaxed)) * v.y,
            static_cast<float>(s_sz.load(std::memory_order_relaxed)) * v.z};
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

AttitudeFilter::Quat AttitudeFilter::quat_conj(const Quat &q)
{
    return {q.w, -q.x, -q.y, -q.z};
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
    const float d = clampf(vec_dot(from, to), -1.0f, 1.0f);
    const Vec3 c = vec_cross(from, to);
    Quat q{d + 1.0f, c.x, c.y, c.z};
    if (quat_normalize(q)) {
        return q;
    }

    Vec3 axis = vec_cross(from, {1.0f, 0.0f, 0.0f});
    if (vec_length(axis) < 1e-3f) {
        axis = vec_cross(from, {0.0f, 1.0f, 0.0f});
    }
    if (!vec_normalize(axis)) {
        return {};
    }
    return {0.0f, axis.x, axis.y, axis.z};
}

AttitudeFilter::Quat AttitudeFilter::scale_rotation(const Quat &q, float gain)
{
    const float w = clampf(q.w, -1.0f, 1.0f);
    const float half = std::acos(w);
    const float mag = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (mag < kEps || half < 1e-6f) {
        return {};
    }
    const float new_half = gain * half;
    const float s = std::sin(new_half) / mag;
    Quat out{std::cos(new_half), q.x * s, q.y * s, q.z * s};
    if (!quat_normalize(out)) {
        return {};
    }
    return out;
}

void AttitudeFilter::quat_to_matrix(const Quat &q, float out[9])
{
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    out[0] = 1.0f - 2.0f * (yy + zz);
    out[1] = 2.0f * (xy - wz);
    out[2] = 2.0f * (xz + wy);
    out[3] = 2.0f * (xy + wz);
    out[4] = 1.0f - 2.0f * (xx + zz);
    out[5] = 2.0f * (yz - wx);
    out[6] = 2.0f * (xz - wy);
    out[7] = 2.0f * (yz + wx);
    out[8] = 1.0f - 2.0f * (xx + yy);
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
    q_ref_ = {};
    R_[0] = 1.0f;
    R_[1] = 0.0f;
    R_[2] = 0.0f;
    R_[3] = 0.0f;
    R_[4] = 1.0f;
    R_[5] = 0.0f;
    R_[6] = 0.0f;
    R_[7] = 0.0f;
    R_[8] = 1.0f;
    g_world_ = {0.0f, -kOneG, 0.0f};
    up_ = {0.0f, 1.0f, 0.0f};
    mapped_ = {};
    raw_accel_ = {};
    pitch_ = 0.0f;
    roll_ = 0.0f;
    yaw_ = 0.0f;
    gyro_abs_ = 0.0f;
    gyro_bias_ = {};
    have_ref_ = false;
    accepted_last_ = false;
    ++nonfinite_resets_;
    align_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::reset()
{
    q_ = {};
    q_ref_ = {};
    R_[0] = 1.0f;
    R_[1] = 0.0f;
    R_[2] = 0.0f;
    R_[3] = 0.0f;
    R_[4] = 1.0f;
    R_[5] = 0.0f;
    R_[6] = 0.0f;
    R_[7] = 0.0f;
    R_[8] = 1.0f;
    g_world_ = {0.0f, -kOneG, 0.0f};
    up_ = {0.0f, 1.0f, 0.0f};
    mapped_ = {};
    raw_accel_ = {};
    pitch_ = 0.0f;
    roll_ = 0.0f;
    yaw_ = 0.0f;
    gyro_abs_ = 0.0f;
    gyro_bias_ = {};
    have_ref_ = false;
    accepted_last_ = false;
    align_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::request_align()
{
    align_pending_.store(true, std::memory_order_release);
}

void AttitudeFilter::request_yaw(float radians)
{
    if (!std::isfinite(radians)) {
        return;
    }
    s_yaw_request.store(radians, std::memory_order_release);
}

void AttitudeFilter::set_axes(int sx, int sy, int sz)
{
    if ((sx != 1 && sx != -1) || (sy != 1 && sy != -1) || (sz != 1 && sz != -1)) {
        return;
    }
    s_sx.store(sx, std::memory_order_relaxed);
    s_sy.store(sy, std::memory_order_relaxed);
    s_sz.store(sz, std::memory_order_relaxed);
    s_axes_gen.fetch_add(1, std::memory_order_release);
}

void AttitudeFilter::set_gain(float gain)
{
    if (!std::isfinite(gain)) {
        return;
    }
    s_gain.store(clampf(gain, kGainMin, kGainMax), std::memory_order_relaxed);
}

void AttitudeFilter::set_tau(float seconds)
{
    if (!std::isfinite(seconds)) {
        return;
    }
    s_tau.store(clampf(seconds, kTauMin, kTauMax), std::memory_order_relaxed);
}

void AttitudeFilter::axes(int *sx, int *sy, int *sz)
{
    if (sx != nullptr) {
        *sx = s_sx.load(std::memory_order_relaxed);
    }
    if (sy != nullptr) {
        *sy = s_sy.load(std::memory_order_relaxed);
    }
    if (sz != nullptr) {
        *sz = s_sz.load(std::memory_order_relaxed);
    }
}

float AttitudeFilter::gain()
{
    return s_gain.load(std::memory_order_relaxed);
}

float AttitudeFilter::tau()
{
    return s_tau.load(std::memory_order_relaxed);
}

void AttitudeFilter::apply_yaw(float radians)
{
    const float half = 0.5f * radians;
    Quat dq{std::cos(half), 0.0f, std::sin(half), 0.0f};
    if (!quat_normalize(dq)) {
        return;
    }
    q_ = quat_mul(q_, dq);
    if (!quat_normalize(q_)) {
        hard_reset();
        return;
    }
    recompute();
}

void AttitudeFilter::consume_yaw_request()
{
    const float yaw = s_yaw_request.exchange(0.0f, std::memory_order_acq_rel);
    if (yaw == 0.0f || !std::isfinite(yaw) || !have_ref_) {
        return;
    }
    apply_yaw(yaw);
}

void AttitudeFilter::maybe_axes_realign()
{
    const uint32_t gen = s_axes_gen.load(std::memory_order_acquire);
    if (gen != axes_gen_) {
        axes_gen_ = gen;
        align_pending_.store(true, std::memory_order_release);
    }
}

void AttitudeFilter::recompute()
{
    Quat q_disp = quat_mul(quat_conj(q_ref_), q_);
    if (!quat_normalize(q_disp)) {
        hard_reset();
        return;
    }

    // Gravity-aligned attitude must remain a one-to-one physical pose. Scaling
    // a full orientation aliases ordinary 90-degree turns (gain 4 => 360
    // degrees), so gain is intentionally limited to relative displays.
    if (reference_mode_ == ReferenceMode::Relative) {
        const float gain = s_gain.load(std::memory_order_relaxed);
        if (std::fabs(gain - 1.0f) > 1e-6f) {
            q_disp = scale_rotation(q_disp, gain);
            if (!quat_normalize(q_disp)) {
                hard_reset();
                return;
            }
        }
    }

    quat_to_matrix(q_disp, R_);

    // Board +Y is screen-up (USB at -Y). Relative identity => up = (0,1,0).
    up_ = rotate(q_disp, {0.0f, 1.0f, 0.0f});
    if (!finite_vec(up_)) {
        hard_reset();
        return;
    }

    const float uxy = std::sqrt(up_.x * up_.x + up_.y * up_.y);
    roll_ = std::atan2(up_.x, up_.y);
    pitch_ = std::atan2(up_.z, uxy);
    yaw_ = std::atan2(2.0f * (q_disp.w * q_disp.y + q_disp.x * q_disp.z),
                      1.0f - 2.0f * (q_disp.y * q_disp.y + q_disp.z * q_disp.z));
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

void AttitudeFilter::init_world(const Vec3 &g_meas)
{
    q_ref_ = {};
    if (reference_mode_ == ReferenceMode::Relative) {
        q_ = {};
        g_world_ = g_meas;
    } else {
        Vec3 measured = g_meas;
        Vec3 gravity{0.0f, -1.0f, 0.0f};
        if (!vec_normalize(measured) || !vec_normalize(gravity)) {
            hard_reset();
            return;
        }
        q_ = quat_between(measured, gravity);
        if (!quat_normalize(q_)) {
            hard_reset();
            return;
        }
        g_world_ = {0.0f, -kOneG, 0.0f};
    }
    have_ref_ = true;
    gyro_abs_ = 0.0f;
    recompute();
}

void AttitudeFilter::pull_toward_gravity(const Vec3 &g_meas, float alpha)
{
    // Tilt-only: rotate world so estimated gravity matches the active
    // reference. The shortest-arc correction has no component around gravity,
    // so yaw stays gyro-only.
    Vec3 meas = g_meas;
    Vec3 g_world = g_world_;
    if (!vec_normalize(meas) || !vec_normalize(g_world)) {
        return;
    }
    Vec3 g_est = rotate(q_, meas);
    if (!vec_normalize(g_est)) {
        return;
    }
    const float d = vec_dot(g_est, g_world);
    if (d > 0.999999f) {
        return;
    }
    Quat delta = quat_between(g_est, g_world);
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
    maybe_axes_realign();
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
    if (!have_ref_ || align) {
        // Re-zero: current pose is identity and current gravity is world.
        // Skip gyro/tilt on this sample so rest cannot jump off the snapshot.
        init_world(mapped);
        gyro_bias_ = gyro;
        consume_yaw_request();
        return true;
    }

    Vec3 gyro_corr{gyro.x - gyro_bias_.x, gyro.y - gyro_bias_.y,
                   gyro.z - gyro_bias_.z};
    float corr_abs = vec_length(gyro_corr);
    if (corr_abs < kGyroBiasStill) {
        const float blend = 1.0f - std::exp(-clamped_dt / kGyroBiasTau);
        gyro_bias_.x += blend * (gyro.x - gyro_bias_.x);
        gyro_bias_.y += blend * (gyro.y - gyro_bias_.y);
        gyro_bias_.z += blend * (gyro.z - gyro_bias_.z);
        gyro_corr = {gyro.x - gyro_bias_.x, gyro.y - gyro_bias_.y,
                     gyro.z - gyro_bias_.z};
        corr_abs = vec_length(gyro_corr);
    }
    gyro_abs_ = corr_abs;

    const float half = 0.5f * clamped_dt;
    Quat dq{1.0f, gyro_corr.x * half, gyro_corr.y * half, gyro_corr.z * half};
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
    const float tau = s_tau.load(std::memory_order_relaxed);
    const float alpha = (1.0f - std::exp(-clamped_dt / tau)) * trust;
    pull_toward_gravity(mapped, clampf(alpha, 0.0f, 1.0f));
    consume_yaw_request();
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
    maybe_axes_realign();
    if (!valid_gravity(apparent_accel)) {
        return false;
    }

    mapped_ = apparent_accel;
    gyro_abs_ = 0.0f;
    accepted_last_ = true;

    const bool align = align_pending_.exchange(false, std::memory_order_acq_rel);
    if (!have_ref_ || align) {
        init_world(apparent_accel);
        gyro_bias_ = {};
        consume_yaw_request();
        return true;
    }

    const float alpha = 1.0f - std::exp(-0.01f / kOverrideTau);
    pull_toward_gravity(apparent_accel, clampf(alpha, 0.0f, 1.0f));
    consume_yaw_request();
    recompute();
    if (!finite_quat(q_) || !finite_vec(up_)) {
        hard_reset();
        return false;
    }
    return true;
}

}  // namespace fluid_demo
