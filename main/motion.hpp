#pragma once

#include "app_types.hpp"

namespace fluid_demo {

/**
 * @brief Pure-C++ orientation filter mapping raw IMU acceleration into the box frame.
 *
 * Pipeline per valid sample (M = box-mapped accel, sensor frame in m/s^2):
 *   1. map calibrated box coordinates: {a.x, -a.y, -a.z};
 *   2. one-pole low-pass with an 80 ms time constant;
 *   3. normalize the filtered vector to simulation gravity magnitude kSimG (6.0),
 *      direction-preserving so the z component is never dropped;
 *   4. halve and clamp high-pass shake to 1.25g;
 *   5. clamp apparent acceleration to 2.25g.
 *
 * No heap, no allocation, no exceptions/RTTI; safe to construct statically.
 * Nonfinite or out-of-range samples are ignored: the filter keeps its last valid
 * output and `update` returns it unchanged. The first valid sample initializes the
 * filter state. Gyro is retained for telemetry only and does not feed the transform.
 */
class MotionFilter {
public:
    /** Simulation gravity magnitude the filtered gravity is normalized to. */
    static constexpr float kSimG = 6.0f;
    /** Standard gravity used for the shake scaling and sanity bounds. */
    static constexpr float kOneG = 9.807f;

    MotionFilter() = default;

    /** Reset all filter state; the next valid sample re-initializes it. */
    void reset();

    /**
     * @brief Feed one fresh IMU sample.
     * @param accel_mps2 Sensor-frame acceleration (m/s^2).
     * @param gyro_rads  Sensor-frame angular rate (rad/s), retained for telemetry.
     * @param dt         Seconds since the previous valid sample (clamped internally).
     * @return Apparent acceleration in simulation units (kSimG gravity + shake),
     *         always finite; equals the last valid output for invalid input.
     */
    Vec3 update(const Vec3 &accel_mps2, const Vec3 &gyro_rads, float dt);

    /** Last valid output of update(). */
    Vec3 last_output() const { return out_; }
    /** Normalized low-pass gravity direction vector (sim units, magnitude kSimG). */
    Vec3 gravity() const { return gravity_; }
    /** Clamped high-pass shake applied to gravity (sim units). */
    Vec3 shake() const { return shake_; }
    /** Last valid box-mapped acceleration, real m/s^2 (pre-filter). */
    Vec3 mapped_accel() const { return mapped_; }
    /** Last valid raw sensor-frame acceleration, real m/s^2. */
    Vec3 raw_accel() const { return raw_accel_; }
    /** Last valid gyro sample, rad/s (telemetry only). */
    Vec3 gyro() const { return gyro_rads_; }
    /** True once at least one valid sample has been processed. */
    bool initialized() const { return initialized_; }
    /** True iff the most recent update consumed a valid accelerometer sample. */
    bool last_sample_accepted() const { return accepted_last_; }

private:
    static float clampf(float v, float lo, float hi);
    static bool finite(const Vec3 &v);
    static bool valid_sample(const Vec3 &accel);

    Vec3 low_pass_state_{}; /* box-mapped low-pass filter state, m/s^2 */
    Vec3 gravity_{};        /* normalized gravity, sim units */
    Vec3 shake_{};          /* applied shake, sim units */
    Vec3 out_{};            /* last valid output, sim units */
    Vec3 mapped_{};         /* last valid box-mapped accel, m/s^2 */
    Vec3 raw_accel_{};      /* last valid sensor-frame accel, m/s^2 */
    Vec3 gyro_rads_{};      /* last valid gyro, rad/s */
    bool initialized_ = false;
    bool accepted_last_ = false;
};

} // namespace fluid_demo
