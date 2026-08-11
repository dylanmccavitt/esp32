#include "fluid_app.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace fluid_demo {

namespace {

constexpr const char *kTag = "fluid_demo";

// ---------------------------------------------------------------------------
// Engine constants ("Popcorn Walker" spec). Units: positions Q8.8 pixels,
// velocities Q8.8 px/frame, dt = 1/30 s fixed (one step per render call);
// every time-based constant is pre-baked for that dt.
// ---------------------------------------------------------------------------

// Gravity / input. |apparent| = 9.0 sim units at full edge tilt.
constexpr int kAccelQ8PerUnit = 64;      // raw px/frame^2 per sim unit
constexpr int kAccelClampRaw = 1024;     // +/-4 px/frame/axis per frame
// Down ALWAYS exists. Above this in-plane magnitude the live tilt sets
// the gravity direction; below it the last-known direction keeps pulling
// (a box seen through a side window always has a floor). Particles can
// never hang in the air, whatever the device orientation.
constexpr float kGravUpdateMag = 0.6f;
// Minimum pull strength: the applied magnitude is max(|a|, this), so
// even the remembered-direction pull at device-flat is a hard yank.
constexpr float kAccelFloorUnits = 4.0f;

// Motion.
constexpr int kDragShift = 7;      // v -= v>>7 per frame (~0.992)
constexpr int kVmaxRaw = 4096;     // 16 px/frame per axis (480 px/s)
constexpr int kMaxWalkSteps = 18;  // per particle per frame (covers vmax)
// 75% of the reachable walk-step ceiling (3000 particles * (18+4) guard);
// past this the frame is under genuine overload and the brake engages.
constexpr uint32_t kStepGovernor = (3000u * (kMaxWalkSteps + 4) * 3u) / 4u;
constexpr int kGovernorDispRaw = 1024;  // 4 px displacement clamp when hit
constexpr int kGovernorGuard = 6;       // walk iteration cap when governed

// Collision response.
constexpr int kWallRestNum = 153;   // e_wall = 0.60
constexpr int kFloorFricShift = 3;  // tangential loss on the gravity axis wall
constexpr int kSideFricShift = 5;   // tangential loss on other walls
constexpr int kWallJitterRaw = 64;  // +/-0.25 px/frame tangential jitter
constexpr int kDeadStopRaw = 90;    // post-bounce |v_n| < 0.35 px/frame -> 0
constexpr int kPpTangNum = 230;     // particle hit: v_t *= 0.9; v_n = -(v_n>>1)

// Kick (grid bits 6-7, harvested next frame). Impact kicks occupy counts
// 1-2; count 3 (0xC0) is reserved as a wake-only tag written by
// support-loss, so a vacated-support particle wakes and falls under
// gravity instead of being launched upward (which self-pumps).
constexpr int kKickTriggerRaw = 512;  // pre-impact |v_n| >= 2 px/frame
constexpr int kKickUpRaw = 307;       // 1.2 px/frame anti-gravity per count
constexpr int kKickLatRaw = 115;      // 0.45 px/frame random-sign lateral

// Leveling / sleep / wake.
constexpr int kSlideMaxRaw = 205;   // water rule below 0.8 px/frame
constexpr int kSleepSpeedRaw = 90;  // quiet threshold
constexpr int kSleepFrames = 10;    // ~0.33 s supported + quiet -> ASLEEP
constexpr float kWakeDirCos = 0.966f;   // >15 deg gravity swing wakes all
constexpr float kWakeMagDelta = 1.5f;   // |d|a|| jump wakes all
constexpr int kSimmerCount = 64;        // wake attempts/frame while tilted
constexpr int kSimmerMinRaw = 77;       // 0.3..0.6 px/frame simmer speed
constexpr int kSimmerSpanRaw = 78;

// Rest gate (also the simmer and wake-all threshold); above the gravity
// arm point so desk-tilt noise cannot hold the gate open.
constexpr float kRestGateMag = 0.6f;
constexpr uint32_t kRestGateFrames = 15;

// Speed -> palette level thresholds (Manhattan |vx|+|vy|, raw Q8.8).
constexpr int kLevelThreshRaw[7] = {90, 205, 448, 832, 1408, 2304, 3584};

// Dim border ring marking the box walls (logical RGB565, swapped at setup).
constexpr uint16_t kWallColor = 0x31A6;  // dark warm gray

// Fixed per-particle sand shades, dark to light, as 8-bit RGB.
constexpr uint8_t kSandRgb[6][3] = {
    {0x8a, 0x6b, 0x3e},  // deep tan
    {0xa3, 0x7c, 0x48},
    {0xb9, 0x8f, 0x55},
    {0xcf, 0xa6, 0x68},
    {0xe0, 0xb8, 0x78},
    {0xf0, 0xcd, 0x8e},  // pale gold
};

// Velocity ramp anchors, indexed by glow level 1..7 (0 = base shade).
constexpr uint8_t kRampRgb[8][3] = {
    {0x00, 0x00, 0x00},  // level 0: unused (base shade)
    {0xff, 0xdd, 0x70},  // glint
    {0xff, 0xd2, 0x3f},  // gold
    {0xff, 0xb8, 0x2e},  // amber
    {0xff, 0x9a, 0x20},  // light orange
    {0xff, 0x6a, 0x10},  // vivid orange
    {0xff, 0x3d, 0x0c},  // orange-red
    {0xff, 0x1a, 0x08},  // red
};

// Blend weight (/255) of the ramp anchor over the base shade per level.
constexpr uint8_t kRampWeight[8] = {0, 90, 140, 180, 210, 235, 250, 255};

// Position raw bounds: [1.0 px, 239.0 px) keeps cells inside [1, 238].
constexpr int32_t kPosMinRaw = 256;
constexpr int32_t kPosMaxRaw = 61183;

inline uint16_t pack565(int r, int g, int b)
{
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

inline int min_int(int a, int b) { return a < b ? a : b; }

inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/// Every velocity write funnels through this: int32 math, then saturate.
inline int16_t sat16(int32_t v)
{
    return static_cast<int16_t>(clamp_i32(v, -32768, 32767));
}

inline int abs_int(int v) { return v < 0 ? -v : v; }

}  // namespace

FluidBoxApp s_fluid_app;

FluidBoxApp::~FluidBoxApp()
{
    heap_caps_free(arena_);
    heap_caps_free(grid_);
    arena_ = nullptr;
    grid_ = nullptr;
}

inline uint32_t FluidBoxApp::rnd()
{
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
}

// ---------------------------------------------------------------------------
// Setup / lifecycle
// ---------------------------------------------------------------------------

esp_err_t FluidBoxApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;  // idempotent
    }

    // SoA arena: px, py (u16), vx, vy (i16), state, rest (u8) = 10 B each.
    const size_t arena_bytes = static_cast<size_t>(kParticleCount) * 10;
    uint8_t *arena = static_cast<uint8_t *>(
        heap_caps_malloc(arena_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    uint8_t *grid = static_cast<uint8_t *>(heap_caps_malloc(
        static_cast<size_t>(kGridW) * kGridH, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (arena == nullptr || grid == nullptr) {
        heap_caps_free(arena);
        heap_caps_free(grid);
        ESP_LOGE(kTag, "particle allocation failed");
        return ESP_ERR_NO_MEM;
    }
    arena_ = arena;
    px_ = reinterpret_cast<uint16_t *>(arena);
    py_ = px_ + kParticleCount;
    vx_ = reinterpret_cast<int16_t *>(py_ + kParticleCount);
    vy_ = vx_ + kParticleCount;
    pstate_ = reinterpret_cast<uint8_t *>(vy_ + kParticleCount);
    prest_ = pstate_ + kParticleCount;
    grid_ = grid;

    // Palette: each glow level blends the base shade toward its ramp anchor.
    for (int lv = 0; lv < 8; ++lv) {
        const int w = kRampWeight[lv];
        for (int s = 0; s < 6; ++s) {
            const int r =
                kSandRgb[s][0] + ((kRampRgb[lv][0] - kSandRgb[s][0]) * w) / 255;
            const int gc =
                kSandRgb[s][1] + ((kRampRgb[lv][1] - kSandRgb[s][1]) * w) / 255;
            const int b =
                kSandRgb[s][2] + ((kRampRgb[lv][2] - kSandRgb[s][2]) * w) / 255;
            shade_wire_[(lv << 3) | (s + 1)] = __builtin_bswap16(pack565(r, gc, b));
        }
    }

    reset_particles();
    setup_done_ = true;
    ESP_LOGI(kTag, "specks initialized: %d ballistic particles", kParticleCount);
    return ESP_OK;
}

void FluidBoxApp::reset_particles()
{
    // Deterministic flat starting bed: fill the bottom rows left-to-right,
    // ~12.6 rows deep, cell-centered, asleep, glow 0.
    rng_ = 0x2545F491u;
    for (int i = 0; i < kParticleCount; ++i) {
        const int x = 1 + (i % (kGridW - 2));
        const int y = (kGridH - 2) - (i / (kGridW - 2));
        px_[i] = static_cast<uint16_t>((x << 8) | 128);
        py_[i] = static_cast<uint16_t>((y << 8) | 128);
        vx_[i] = 0;
        vy_[i] = 0;
        pstate_[i] = static_cast<uint8_t>((rnd() % 6) | kStateAsleep);
        prest_[i] = kSleepFrames;
    }
    std::memset(grid_, 0, static_cast<size_t>(kGridW) * kGridH);
    for (int i = 0; i < kParticleCount; ++i) {
        const size_t cell = static_cast<size_t>(py_[i] >> 8) * kGridW + (px_[i] >> 8);
        grid_[cell] = static_cast<uint8_t>((pstate_[i] & kStateShadeMask) + 1);
    }
    awake_count_ = 0;
    rest_gate_frames_ = 0;
    epoch_.fetch_add(1, std::memory_order_relaxed);
}

esp_err_t FluidBoxApp::enter()
{
    return ESP_OK;
}

void FluidBoxApp::leave()
{
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
}

ShellAction FluidBoxApp::handle_event(AppEvent event)
{
    if (event == AppEvent::PlusPress) {
        request_fluid_reset();
    }
    return ShellAction::None;
}

void FluidBoxApp::request_fluid_reset()
{
    reset_requested_.store(true, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Sensor lane: motion filter + override bypass (unchanged pattern)
// ---------------------------------------------------------------------------

bool FluidBoxApp::on_motion(const MotionTick &tick)
{
    if (!tick.fresh) {
        if (tick.override_active) {
            portENTER_CRITICAL(&motion_mux_);
            motion_.apparent_accel = tick.apparent_accel;
            motion_.valid = true;
            portEXIT_CRITICAL(&motion_mux_);
        }
        return false;
    }

    const Vec3 apparent = filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
    const bool accepted = filter_.last_sample_accepted();
    if (!accepted) {
        if (tick.override_active) {
            portENTER_CRITICAL(&motion_mux_);
            motion_.apparent_accel = tick.apparent_accel;
            motion_.valid = true;
            portEXIT_CRITICAL(&motion_mux_);
        }
        return false;
    }
    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent_accel = tick.override_active ? tick.apparent_accel : apparent;
    motion_.raw_accel = tick.accel_mps2;
    motion_.valid = true;
    portEXIT_CRITICAL(&motion_mux_);
    return true;
}

// ---------------------------------------------------------------------------
// Physics lane: intentionally idle
// ---------------------------------------------------------------------------

esp_err_t FluidBoxApp::update(float)
{
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// The engine: one fixed-dt step
// ---------------------------------------------------------------------------

uint32_t FluidBoxApp::step_particles(float sgx, float sgy)
{
    // -- Per-frame gravity terms (the only float math in the step) --------
    float ax = sgx;
    float ay = sgy;
    if (!std::isfinite(ax) || !std::isfinite(ay)) {
        ax = 0.0f;
        ay = 0.0f;
    }
    const float m2 = ax * ax + ay * ay;
    const float m_in = std::sqrt(m2);
    if (m_in >= kGravUpdateMag) {
        const float inv = 1.0f / m_in;
        dir_gx_ = ax * inv;
        dir_gy_ = ay * inv;
    }
    // Down always exists: live direction when tilted, remembered direction
    // when flat, magnitude never below the floor.
    const float eff =
        (m_in > kAccelFloorUnits ? m_in : kAccelFloorUnits) *
        static_cast<float>(kAccelQ8PerUnit);
    const int32_t dvx = clamp_i32(static_cast<int32_t>(lroundf(dir_gx_ * eff)),
                                  -kAccelClampRaw, kAccelClampRaw);
    const int32_t dvy = clamp_i32(static_cast<int32_t>(lroundf(dir_gy_ * eff)),
                                  -kAccelClampRaw, kAccelClampRaw);

    // Quantized gravity octant (each component -1/0/1) for leveling, kick
    // and simmer directions. tan(22.5 deg) ~ 0.414 splits the octants.
    int gox = 0;
    int goy = 0;
    {
        const float axa = std::fabs(dir_gx_);
        const float aya = std::fabs(dir_gy_);
        if (axa > 0.414f * aya) {
            gox = (dir_gx_ >= 0.0f) ? 1 : -1;
        }
        if (aya > 0.414f * axa) {
            goy = (dir_gy_ >= 0.0f) ? 1 : -1;
        }
    }
    const bool grav_x_dom = std::fabs(dir_gx_) >= std::fabs(dir_gy_);

    // -- Global wake: gravity swung, jumped, or flipped -------------------
    const float mag = std::sqrt(m2);
    {
        const float pmag =
            std::sqrt(prev_gx_ * prev_gx_ + prev_gy_ * prev_gy_);
        const float dot = ax * prev_gx_ + ay * prev_gy_;
        bool wake_all = false;
        if (mag >= kRestGateMag) {
            if (dot < 0.0f && pmag > 0.2f) {
                wake_all = true;  // flipped
            } else if (pmag > 0.2f && dot < kWakeDirCos * mag * pmag) {
                wake_all = true;  // direction swing > ~15 deg
            } else if (std::fabs(mag - pmag) > kWakeMagDelta) {
                wake_all = true;  // magnitude jump
            }
        }
        if (wake_all) {
            for (int i = 0; i < kParticleCount; ++i) {
                pstate_[i] &= static_cast<uint8_t>(~kStateAsleep);
                prest_[i] = 0;
            }
            awake_count_ = kParticleCount;
        }
        // Anchor, not previous-frame: it only re-arms on a wake or while
        // quiet, so a slow tilt accumulates direction/magnitude change
        // until a wake fires (periodic avalanches), instead of the bed
        // freezing solid because no single frame crossed a threshold.
        if (wake_all || mag < kRestGateMag) {
            prev_gx_ = ax;
            prev_gy_ = ay;
        }
    }

    // -- Rest gate: quiet and everyone asleep -> skip the step ------------
    if (mag < kRestGateMag && awake_count_ == 0) {
        if (rest_gate_frames_ < kRestGateFrames) {
            ++rest_gate_frames_;
        }
    } else {
        rest_gate_frames_ = 0;
    }
    if (rest_gate_frames_ >= kRestGateFrames) {
        return 0;  // scene is settled and bit-stable; grid stays valid
    }

    uint8_t *g = grid_;

    // Diagonal octants carry sqrt(2) more Euclidean punch per component;
    // scale impulse coefficients by 181/256 so tilt compass direction does
    // not change agitation intensity.
    const bool go_diag = (gox != 0 && goy != 0);
    const int32_t kick_up = go_diag ? (kKickUpRaw * 181) >> 8 : kKickUpRaw;
    const int32_t kick_lat = go_diag ? (kKickLatRaw * 181) >> 8 : kKickLatRaw;

    // -- Kick harvest (reads LAST frame's grid, before the rebuild) -------
    for (int i = 0; i < kParticleCount; ++i) {
        const size_t cell =
            static_cast<size_t>(py_[i] >> 8) * kGridW + (px_[i] >> 8);
        const uint32_t k = static_cast<uint32_t>(g[cell]) >> 6;
        if (k == 0u) {
            continue;
        }
        if (k < 3u) {
            // Counts 1-2 = impact kicks with impulse; 3 = wake-only tag.
            const int lat_sign = ((rnd() & 1u) != 0u) ? 1 : -1;
            vx_[i] = sat16(vx_[i] - gox * static_cast<int32_t>(k) * kick_up -
                           goy * lat_sign * static_cast<int32_t>(k) * kick_lat);
            vy_[i] = sat16(vy_[i] - goy * static_cast<int32_t>(k) * kick_up +
                           gox * lat_sign * static_cast<int32_t>(k) * kick_lat);
        }
        pstate_[i] &= static_cast<uint8_t>(~kStateAsleep);
        prest_[i] = 0;
    }

    // -- Grid rebuild from positions (positions are the only truth) -------
    std::memset(g, 0, static_cast<size_t>(kGridW) * kGridH);
    for (int i = 0; i < kParticleCount; ++i) {
        const size_t cell =
            static_cast<size_t>(py_[i] >> 8) * kGridW + (px_[i] >> 8);
        if (g[cell] == 0u) {
            g[cell] = static_cast<uint8_t>(
                ((pstate_[i] & kStateGlowMask) << 0) |
                ((pstate_[i] & kStateShadeMask) + 1));
        } else {
            // Overlap (possible for one frame after a flip): separation
            // impulse; the loser is not drawn this frame and must not
            // lift the winner's byte during its walk.
            const uint32_t r = rnd();
            vx_[i] = sat16(vx_[i] + (((r & 1u) != 0u) ? 256 : -256));
            vy_[i] = sat16(vy_[i] + (((r & 2u) != 0u) ? 256 : -256));
            pstate_[i] = static_cast<uint8_t>(
                (pstate_[i] & ~kStateAsleep) | kStateOverlap);
            prest_[i] = 0;
        }
    }

    // -- Particle update --------------------------------------------------
    uint32_t steps_used = 0;
    uint32_t awake = 0;
    const bool fwd_sweep = ((frame_parity_++ & 1u) == 0u);
    for (int n = 0; n < kParticleCount; ++n) {
        const int i = fwd_sweep ? n : (kParticleCount - 1 - n);
        uint8_t st = pstate_[i];
        if ((st & kStateAsleep) != 0u) {
            continue;
        }

        // Integrate: gravity, drag, per-axis clamp.
        int32_t vx = vx_[i] + dvx;
        int32_t vy = vy_[i] + dvy;
        vx -= vx >> kDragShift;
        vy -= vy >> kDragShift;
        vx = clamp_i32(vx, -kVmaxRaw, kVmaxRaw);
        vy = clamp_i32(vy, -kVmaxRaw, kVmaxRaw);

        int32_t posx = px_[i];
        int32_t posy = py_[i];
        int cx = posx >> 8;
        int cy = posy >> 8;
        const int start_cx = cx;
        const int start_cy = cy;
        const size_t start_cell = static_cast<size_t>(cy) * kGridW + cx;
        // Overlap losers don't own their start cell's byte: leave it.
        const bool overlap_loser = (st & kStateOverlap) != 0u;
        st &= static_cast<uint8_t>(~kStateOverlap);
        uint8_t kick_bits = 0;
        if (!overlap_loser) {
            // Lift the particle out of the grid for the walk, carrying any
            // kick/wake bits written at it earlier this frame so they
            // survive to next frame's harvest.
            kick_bits = static_cast<uint8_t>(g[start_cell] & 0xC0u);
            g[start_cell] = 0;
        }

        int32_t rx = vx;
        int32_t ry = vy;
        int guard = kMaxWalkSteps + 4;
        if (steps_used > kStepGovernor) {
            governor_hits_.fetch_add(1, std::memory_order_relaxed);
            rx = clamp_i32(rx, -kGovernorDispRaw, kGovernorDispRaw);
            ry = clamp_i32(ry, -kGovernorDispRaw, kGovernorDispRaw);
            guard = kGovernorGuard;
        }

        // Cell walk: sub-steps of <= 1 px per axis, probing before entry.
        // The substep increments (2 hw divides) are loop-invariant between
        // bounces, so they are recomputed only when nsub hits 0.
        int32_t nsub = 0;
        int32_t sx = 0;
        int32_t sy = 0;
        while ((rx != 0 || ry != 0) && guard-- > 0) {
            if (nsub <= 0) {
                const int32_t axr = abs_int(rx);
                const int32_t ayr = abs_int(ry);
                const int32_t m = axr > ayr ? axr : ayr;
                nsub = (m + 255) >> 8;
                if (nsub <= 0) {
                    break;
                }
                sx = rx / nsub;
                sy = ry / nsub;
                if (sx == 0 && sy == 0) {
                    break;
                }
            }
            ++steps_used;
            int32_t npx = posx + sx;
            int32_t npy = posy + sy;
            int ncx = npx >> 8;
            int ncy = npy >> 8;
            bool bounced_x = false;
            bool bounced_y = false;

            // X axis crossing.
            if (ncx != cx) {
                bool blocked = false;
                bool wall = false;
                if (ncx < 1 || ncx > kGridW - 2) {
                    blocked = true;
                    wall = true;
                } else if (g[static_cast<size_t>(cy) * kGridW + ncx] != 0u) {
                    blocked = true;
                }
                if (blocked) {
                    if (wall) {
                        vx = -(vx * kWallRestNum) >> 8;
                        rx = -(rx * kWallRestNum) >> 8;
                        const int fs =
                            grav_x_dom ? kFloorFricShift : kSideFricShift;
                        vy -= vy >> fs;
                        vy += static_cast<int32_t>(rnd() % (2 * kWallJitterRaw + 1)) -
                              kWallJitterRaw;
                        if (abs_int(vx) < kDeadStopRaw) {
                            vx = 0;
                            rx = 0;
                        }
                    } else {
                        // Pass-through blocking, not reflection: co-falling
                        // neighbors must not ricochet off each other. The
                        // follower keeps its direction (damped) and resumes
                        // the moment the cell clears, so a cloud rains at
                        // one rate. Kicks only fire into supported (bedded)
                        // targets - a mid-air hit must not shove another
                        // faller against the rain.
                        const int sdx = ncx + gox;
                        const int sdy = cy + goy;
                        const bool bedded =
                            sdx < 1 || sdx > kGridW - 2 || sdy < 1 ||
                            sdy > kGridH - 2 ||
                            g[static_cast<size_t>(sdy) * kGridW + sdx] != 0u;
                        if (bedded && abs_int(vx) >= kKickTriggerRaw) {
                            uint8_t &tc =
                                g[static_cast<size_t>(cy) * kGridW + ncx];
                            if ((tc >> 6) < 2u) {
                                tc = static_cast<uint8_t>(tc + 0x40u);
                            }
                        }
                        // Damped keep-sign block; the retry happens next
                        // frame (rem zeroed), never as an in-frame storm.
                        // Pressing into the bed sheds energy fast (>>2) so
                        // pressed grains still level and settle.
                        vx = vx >> (bedded ? 2 : 1);
                        rx = 0;
                        vy = (vy * kPpTangNum) >> 8;
                    }
                    // Flush against the boundary of the current cell.
                    npx = (sx > 0) ? ((cx << 8) | 0xFF) : (cx << 8);
                    ncx = cx;
                    bounced_x = true;
                }
            }
            // Y axis crossing (against the possibly X-corrected column).
            if (ncy != cy) {
                bool blocked = false;
                bool wall = false;
                if (ncy < 1 || ncy > kGridH - 2) {
                    blocked = true;
                    wall = true;
                } else if (g[static_cast<size_t>(ncy) * kGridW + ncx] != 0u) {
                    blocked = true;
                }
                if (blocked) {
                    if (wall) {
                        vy = -(vy * kWallRestNum) >> 8;
                        ry = -(ry * kWallRestNum) >> 8;
                        const int fs =
                            grav_x_dom ? kSideFricShift : kFloorFricShift;
                        vx -= vx >> fs;
                        vx += static_cast<int32_t>(rnd() % (2 * kWallJitterRaw + 1)) -
                              kWallJitterRaw;
                        if (abs_int(vy) < kDeadStopRaw) {
                            vy = 0;
                            ry = 0;
                        }
                    } else {
                        const int sdx = ncx + gox;
                        const int sdy = ncy + goy;
                        const bool bedded =
                            sdx < 1 || sdx > kGridW - 2 || sdy < 1 ||
                            sdy > kGridH - 2 ||
                            g[static_cast<size_t>(sdy) * kGridW + sdx] != 0u;
                        if (bedded && abs_int(vy) >= kKickTriggerRaw) {
                            uint8_t &tc =
                                g[static_cast<size_t>(ncy) * kGridW + ncx];
                            if ((tc >> 6) < 2u) {
                                tc = static_cast<uint8_t>(tc + 0x40u);
                            }
                        }
                        vy = vy >> (bedded ? 2 : 1);
                        ry = 0;
                        vx = (vx * kPpTangNum) >> 8;
                    }
                    npy = (sy > 0) ? ((cy << 8) | 0xFF) : (cy << 8);
                    ncy = cy;
                    bounced_y = true;
                }
            }

            posx = npx;
            posy = npy;
            cx = ncx;
            cy = ncy;
            // Consume the taken step per axis; a bounced axis keeps its
            // reflected remainder instead. Any bounce invalidates the
            // hoisted substep increments.
            if (!bounced_x) {
                rx -= sx;
            }
            if (!bounced_y) {
                ry -= sy;
            }
            if (bounced_x || bounced_y) {
                nsub = 0;
            } else {
                --nsub;
            }
        }

        posx = clamp_i32(posx, kPosMinRaw, kPosMaxRaw);
        posy = clamp_i32(posy, kPosMinRaw, kPosMaxRaw);
        cx = posx >> 8;
        cy = posy >> 8;

        // -- Zero-repose leveling: slow + supported drains toward flat.
        // Up to two slide hops per frame, and a successful slide writes a
        // small velocity along the hop so the drain keeps flowing (and
        // glows) instead of stuttering asleep mid-avalanche.
        int32_t speed = abs_int(vx) + abs_int(vy);
        const int dcx = cx + gox;
        const int dcy = cy + goy;
        const bool down_blocked =
            (dcx < 1 || dcx > kGridW - 2 || dcy < 1 || dcy > kGridH - 2) ||
            (g[static_cast<size_t>(dcy) * kGridW + dcx] != 0u);
        bool supported = down_blocked;
        bool leveled = false;
        if (speed < kSlideMaxRaw && down_blocked && (gox | goy) != 0) {
            // Slide neighbors: for a cardinal octant the two down-diagonals;
            // for a diagonal octant the two adjacent cardinals.
            int s1x, s1y, s2x, s2y;
            if (gox != 0 && goy != 0) {
                s1x = gox; s1y = 0;
                s2x = 0;   s2y = goy;
            } else if (gox != 0) {
                s1x = gox; s1y = 1;
                s2x = gox; s2y = -1;
            } else {
                s1x = 1;  s1y = goy;
                s2x = -1; s2y = goy;
            }
            int hop_dx = 0;
            int hop_dy = 0;
            for (int hop = 0; hop < 2; ++hop) {
                const uint32_t r = rnd();
                const bool swap = (r & 1u) != 0u;
                const int a1x = swap ? s2x : s1x, a1y = swap ? s2y : s1y;
                const int a2x = swap ? s1x : s2x, a2y = swap ? s1y : s2y;
                bool moved = false;
                for (int attempt = 0; attempt < 2 && !moved; ++attempt) {
                    const int tx = cx + (attempt == 0 ? a1x : a2x);
                    const int ty = cy + (attempt == 0 ? a1y : a2y);
                    if (tx >= 1 && tx <= kGridW - 2 && ty >= 1 &&
                        ty <= kGridH - 2 &&
                        g[static_cast<size_t>(ty) * kGridW + tx] == 0u) {
                        hop_dx = tx - cx;
                        hop_dy = ty - cy;
                        cx = tx;
                        cy = ty;
                        posx = (tx << 8) | (posx & 0xFF);
                        posy = (ty << 8) | (posy & 0xFF);
                        moved = true;
                        leveled = true;
                    }
                }
                if (!moved && !leveled && (r & 2u) != 0u) {
                    // Lateral hop, only into a cell whose down cell is empty.
                    const int lx = (goy != 0) ? (((r & 4u) != 0u) ? 1 : -1) : 0;
                    const int ly =
                        (gox != 0 && goy == 0) ? (((r & 4u) != 0u) ? 1 : -1) : 0;
                    const int tx = cx + lx;
                    const int ty = cy + ly;
                    const int tdx = tx + gox;
                    const int tdy = ty + goy;
                    if ((lx | ly) != 0 && tx >= 1 && tx <= kGridW - 2 &&
                        ty >= 1 && ty <= kGridH - 2 &&
                        g[static_cast<size_t>(ty) * kGridW + tx] == 0u &&
                        tdx >= 1 && tdx <= kGridW - 2 && tdy >= 1 &&
                        tdy <= kGridH - 2 &&
                        g[static_cast<size_t>(tdy) * kGridW + tdx] == 0u) {
                        hop_dx = lx;
                        hop_dy = ly;
                        cx = tx;
                        cy = ty;
                        posx = (tx << 8) | (posx & 0xFF);
                        posy = (ty << 8) | (posy & 0xFF);
                        leveled = true;
                    }
                }
                if (!moved) {
                    break;
                }
            }
            if (leveled) {
                // Flow velocity along the drain: keeps the avalanche
                // continuous and lights the surface gold.
                vx = hop_dx * 180;
                vy = hop_dy * 180;
                prest_[i] = 0;
            }
            // Support may have changed after sliding.
            const int ddx = cx + gox;
            const int ddy = cy + goy;
            supported =
                (ddx < 1 || ddx > kGridW - 2 || ddy < 1 || ddy > kGridH - 2) ||
                (g[static_cast<size_t>(ddy) * kGridW + ddx] != 0u);
        }
        speed = abs_int(vx) + abs_int(vy);

        // -- Glow: raw level from speed, one-level-per-frame afterglow.
        int raw_level = 0;
        while (raw_level < 7 && speed >= kLevelThreshRaw[raw_level]) {
            ++raw_level;
        }
        const int prev_level = (st & kStateGlowMask) >> kStateGlowShift;
        const int level =
            raw_level > prev_level - 1 ? raw_level : prev_level - 1;

        // -- Sleep: quiet, cooled, supported, not mid-drain, hysteresis.
        // Support is mandatory: down always exists, so nothing may ever
        // rest mid-air.
        bool slept = false;
        if (speed < kSleepSpeedRaw && level == 0 && supported && !leveled) {
            if (prest_[i] < 255u) {
                ++prest_[i];
            }
            if (prest_[i] >= kSleepFrames) {
                vx = 0;
                vy = 0;
                st |= kStateAsleep;
                slept = true;
            }
        } else {
            prest_[i] = 0;
        }

        st = static_cast<uint8_t>(
            (st & ~kStateGlowMask) |
            (static_cast<uint8_t>(level) << kStateGlowShift));
        pstate_[i] = st;
        vx_[i] = sat16(vx);
        vy_[i] = sat16(vy);
        px_[i] = static_cast<uint16_t>(posx);
        py_[i] = static_cast<uint16_t>(posy);
        if (!slept) {
            ++awake;
        }

        // Place back into the grid with the updated level, carrying kick
        // bits written at the start cell earlier this frame so next
        // frame's harvest still sees them. If someone claimed the cell
        // mid-walk (rare), the position self-heals at next rebuild.
        uint8_t *dst = &g[static_cast<size_t>(cy) * kGridW + cx];
        if (*dst == 0u) {
            *dst = static_cast<uint8_t>(
                kick_bits | (static_cast<uint8_t>(level) << 3) |
                ((st & kStateShadeMask) + 1));
        }

        // Support-loss wake: tag the anti-gravity neighbor of a vacated
        // cell with the wake-only code (0xC0) so nothing hangs as a
        // floating shelf — a wake, not a launch. Never tag the mover's
        // own new cell (self-pump), and overlap losers vacated nothing.
        if (!overlap_loser && (cx != start_cx || cy != start_cy) &&
            (gox | goy) != 0) {
            const int ux = start_cx - gox;
            const int uy = start_cy - goy;
            if ((ux != cx || uy != cy) && ux >= 1 && ux <= kGridW - 2 &&
                uy >= 1 && uy <= kGridH - 2) {
                uint8_t &above = g[static_cast<size_t>(uy) * kGridW + ux];
                if (above != 0u && (above >> 6) == 0u) {
                    above |= 0xC0u;
                }
            }
        }
    }

    // -- Simmer: while tilted, tickle a few sleepers on the surface -------
    if (mag >= kRestGateMag) {
        for (int t = 0; t < kSimmerCount; ++t) {
            const uint32_t r = rnd();
            const int i = static_cast<int>(r % kParticleCount);
            if ((pstate_[i] & kStateAsleep) == 0u) {
                continue;
            }
            const int cx = px_[i] >> 8;
            const int cy = py_[i] >> 8;
            // Surface test: any cardinal neighbor empty (not fully
            // buried) — the free surface is not always the anti-gravity
            // face (a bottom bed under sideways gravity exposes its top).
            const size_t cc = static_cast<size_t>(cy) * kGridW + cx;
            const bool buried =
                g[cc - 1] != 0u && g[cc + 1] != 0u &&
                g[cc - kGridW] != 0u && g[cc + kGridW] != 0u;
            if (buried) {
                continue;
            }
            int32_t sp =
                kSimmerMinRaw + static_cast<int32_t>((r >> 8) % kSimmerSpanRaw);
            int32_t lat = static_cast<int32_t>((r >> 16) % (sp + 1)) -
                          (sp >> 1);
            if (go_diag) {
                sp = (sp * 181) >> 8;
                lat = (lat * 181) >> 8;
            }
            vx_[i] = sat16(-gox * sp - goy * lat);
            vy_[i] = sat16(-goy * sp + gox * lat);
            pstate_[i] &= static_cast<uint8_t>(~kStateAsleep);
            prest_[i] = 0;
            ++awake;
        }
    }

    awake_count_ = awake;
    return steps_used;
}

// ---------------------------------------------------------------------------
// Render lane: step + rasterize
// ---------------------------------------------------------------------------

void FluidBoxApp::draw_grid(uint16_t *buf, int y0, int rows)
{
    const uint16_t wall = __builtin_bswap16(kWallColor);
    const int y1 = y0 + rows;
    for (int y = y0; y < y1; ++y) {
        uint16_t *out = buf + static_cast<size_t>(y - y0) * kGridW;
        if (y == 0 || y == kGridH - 1) {
            for (int x = 0; x < kGridW; ++x) {
                out[x] = wall;
            }
            continue;
        }
        out[0] = wall;
        out[kGridW - 1] = wall;
        const uint8_t *row = grid_ + static_cast<size_t>(y) * kGridW;
        for (int x = 1; x < kGridW - 1; ++x) {
            const uint8_t c = row[x];
            if (c != 0u) {
                out[x] = shade_wire_[c & 0x3Fu];
            }
        }
    }
}

bool FluidBoxApp::render(DisplayFrame &frame)
{
    if (!setup_done_ || frame.transport == nullptr) {
        ESP_LOGW(kTag, "render failed: %s", esp_err_to_name(ESP_ERR_INVALID_STATE));
        return false;
    }
    const int64_t t_frame = esp_timer_get_time();

    if (reset_requested_.exchange(false)) {
        reset_particles();
        ESP_LOGI(kTag, "PLUS press - specks reset (epoch %u)",
                 static_cast<unsigned>(epoch_.load(std::memory_order_relaxed)));
    }

    // Latest apparent acceleration; box +y is screen-up, so screen-down
    // gravity is -y. z (into the case) has no in-plane effect.
    Vec3 apparent{0.0f, 0.0f, 9.0f};
    portENTER_CRITICAL(&motion_mux_);
    if (motion_.valid) {
        apparent = motion_.apparent_accel;
    }
    portEXIT_CRITICAL(&motion_mux_);

    const int64_t t_step = esp_timer_get_time();
    const uint32_t steps = step_particles(apparent.x, -apparent.y);
    walk_steps_.fetch_add(steps, std::memory_order_relaxed);
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - t_step),
                      std::memory_order_relaxed);

    if (frame.ops.wait_previous(frame.transport) != ESP_OK) {
        return false;
    }
    static_cast<void>(frame.ops.latch_capture(frame.transport));

    uint32_t raster_total = 0;
    for (int s = 0; s < frame.stripe_count; s++) {
        const int y0 = s * frame.stripe_rows;
        const int rows = min_int(frame.stripe_rows, frame.height - y0);
        uint16_t *buf = frame.stripe[s & 1];

        const int64_t t_raster = esp_timer_get_time();
        std::memset(buf, 0, static_cast<size_t>(frame.width) * rows * sizeof(uint16_t));
        draw_grid(buf, y0, rows);
        raster_total += static_cast<uint32_t>(esp_timer_get_time() - t_raster);

        if (frame.ops.submit(frame.transport, s, y0, rows, buf) != ESP_OK) {
            return false;
        }
    }
    if (frame.ops.finish(frame.transport) != ESP_OK) {
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - t_frame);
    raster_us_ = raster_total + frame.ops.capture_copy_us(frame.transport);
    return true;
}

AppStats FluidBoxApp::stats()
{
    AppStats st = {};
    st.count = kParticleCount;
    st.epoch = epoch_.load(std::memory_order_relaxed);
    st.candidate_checks = walk_steps_.load(std::memory_order_relaxed);
    st.nonfinite_resets = governor_hits_.load(std::memory_order_relaxed);
    st.physics_us = physics_us_.load(std::memory_order_relaxed);
    portENTER_CRITICAL(&motion_mux_);
    st.raw[0] = motion_.raw_accel.x;
    st.raw[1] = motion_.raw_accel.y;
    st.raw[2] = motion_.raw_accel.z;
    st.apparent[0] = motion_.apparent_accel.x;
    st.apparent[1] = motion_.apparent_accel.y;
    st.apparent[2] = motion_.apparent_accel.z;
    portEXIT_CRITICAL(&motion_mux_);
    st.raster_us = raster_us_;
    st.frame_us = frame_us_;
    return st;
}

}  // namespace fluid_demo
