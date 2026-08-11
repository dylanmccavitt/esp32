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

// In-plane gravity below this magnitude (sim units; |g| = 9 when tilted
// fully on edge) leaves the sand at rest — the device is lying flat.
constexpr float kRestGate = 1.2f;

// Substep thresholds: steeper tilt runs more automaton substeps per frame,
// so sand falls visibly faster the harder you tilt.
constexpr float kFastTilt = 4.5f;
constexpr float kFullTilt = 7.5f;

// Dim border ring marking the box walls (logical RGB565, swapped at setup).
constexpr uint16_t kWallColor = 0x31A6;  // dark warm gray

// Fixed per-grain sand shades, dark to light (logical RGB565). A grain
// keeps its shade for life, which gives the bed its static granular
// texture; motion reads from displacement alone.
constexpr uint16_t kSandShades[6] = {
    0x8B47,  // #8a6b3e deep tan
    0xA3E9,  // #a37c48
    0xBC6A,  // #b98f55
    0xCD2D,  // #cfa668
    0xE5CF,  // #e0b878
    0xF671,  // #f0cd8e pale gold
};

inline int min_int(int a, int b) { return a < b ? a : b; }

}  // namespace

FluidBoxApp s_fluid_app;

FluidBoxApp::~FluidBoxApp()
{
    heap_caps_free(grid_);
    grid_ = nullptr;
}

inline uint32_t FluidBoxApp::rnd()
{
    // xorshift32: cheap, deterministic from the fixed seed at boot.
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

    uint8_t *grid = static_cast<uint8_t *>(heap_caps_malloc(
        static_cast<size_t>(kGridW) * kGridH, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (grid == nullptr) {
        ESP_LOGE(kTag, "sand grid allocation failed");
        return ESP_ERR_NO_MEM;
    }
    grid_ = grid;

    for (int i = 0; i < kShadeCount; ++i) {
        shade_wire_[i + 1] = __builtin_bswap16(kSandShades[i]);
    }
    shade_wire_[0] = 0;

    reset_grid();
    setup_done_ = true;
    ESP_LOGI(kTag, "sand initialized: %u grains on a %dx%d grid",
             static_cast<unsigned>(grain_count_), kGridW, kGridH);
    return ESP_OK;
}

void FluidBoxApp::reset_grid()
{
    std::memset(grid_, 0, static_cast<size_t>(kGridW) * kGridH);
    // Deterministic starting pile: a solid block on the screen-bottom rows.
    // Shades come from a position hash, so every reset is identical.
    uint32_t count = 0;
    const int y0 = kGridH - 1 - kMargin - kPileRows;
    const int y1 = kGridH - 1 - kMargin;
    for (int y = y0; y <= y1; ++y) {
        uint8_t *row = grid_ + static_cast<size_t>(y) * kGridW;
        for (int x = kMargin; x <= kGridW - 1 - kMargin; ++x) {
            const uint32_t h = (static_cast<uint32_t>(x) * 0x9E3779B9u) ^
                               (static_cast<uint32_t>(y) * 0x85EBCA6Bu);
            row[x] = static_cast<uint8_t>(1u + (h >> 8) % kShadeCount);
            ++count;
        }
    }
    grain_count_ = count;
    epoch_.fetch_add(1, std::memory_order_relaxed);
}

esp_err_t FluidBoxApp::enter()
{
    // Nothing to drain: the automaton owns no cross-lane frame state. A
    // pending reset set while inactive stays set; the first render() step
    // consumes it exactly once.
    return ESP_OK;
}

void FluidBoxApp::leave()
{
    // No allocation; quiesce motion validity so a resumed run never applies
    // a stale gravity vector from a previous lifecycle.
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
    // The automaton steps inside render() on the render lane, so the grid
    // never crosses cores and needs no snapshotting.
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// The automaton
// ---------------------------------------------------------------------------

uint32_t FluidBoxApp::step_sand(float sgx, float sgy)
{
    uint8_t *g = grid_;
    uint32_t moved = 0;
    constexpr int kLo = kMargin;
    constexpr int kHiX = kGridW - 1 - kMargin;
    constexpr int kHiY = kGridH - 1 - kMargin;

    const auto try_move = [&](int x, int y, int nx, int ny, uint8_t c) {
        if (nx < kLo || nx > kHiX || ny < kLo || ny > kHiY) {
            return false;
        }
        uint8_t &dst = g[static_cast<size_t>(ny) * kGridW + nx];
        if (dst != 0u) {
            return false;
        }
        dst = c;
        g[static_cast<size_t>(y) * kGridW + x] = 0u;
        ++moved;
        return true;
    };

    const float ax = std::fabs(sgx);
    const float ay = std::fabs(sgy);

    if (ay >= ax) {
        // Vertical-dominant fall. Scanning from the settled side (against
        // gravity) guarantees at most one move per grain per substep. The
        // lateral probability blends the fall direction between the axes so
        // the pile's slope tracks the true tilt angle, not 45-degree steps.
        const int dy = (sgy >= 0.0f) ? 1 : -1;
        const int lat = (sgx >= 0.0f) ? 1 : -1;
        const uint32_t p_lat =
            static_cast<uint32_t>(256.0f * ax / (ay + 1.0e-6f));
        const int y_from = (dy > 0) ? kHiY : kLo;
        const int y_to = (dy > 0) ? kLo : kHiY;
        for (int y = y_from; y != y_to - dy; y -= dy) {
            const bool ltr = ((frame_parity_ ^ static_cast<uint32_t>(y)) & 1u) != 0u;
            const int x_from = ltr ? kLo : kHiX;
            const int x_to = ltr ? kHiX : kLo;
            const int dx_scan = ltr ? 1 : -1;
            uint8_t *row = g + static_cast<size_t>(y) * kGridW;
            for (int x = x_from; x != x_to + dx_scan; x += dx_scan) {
                const uint8_t c = row[x];
                if (c == 0u) {
                    continue;
                }
                const int dxp = ((rnd() & 255u) < p_lat) ? lat : 0;
                if (try_move(x, y, x + dxp, y + dy, c)) {
                    continue;
                }
                // Topple: the two remaining forward diagonals, biased order.
                const int first = (dxp == 0) ? lat : -dxp;
                if (try_move(x, y, x + first, y + dy, c)) {
                    continue;
                }
                static_cast<void>(try_move(x, y, x - first, y + dy, c));
            }
        }
    } else {
        // Horizontal-dominant fall: the mirror case, scanning columns from
        // the settled side.
        const int dx = (sgx >= 0.0f) ? 1 : -1;
        const int lat = (sgy >= 0.0f) ? 1 : -1;
        const uint32_t p_lat =
            static_cast<uint32_t>(256.0f * ay / (ax + 1.0e-6f));
        const int x_from = (dx > 0) ? kHiX : kLo;
        const int x_to = (dx > 0) ? kLo : kHiX;
        for (int x = x_from; x != x_to - dx; x -= dx) {
            const bool ttb = ((frame_parity_ ^ static_cast<uint32_t>(x)) & 1u) != 0u;
            const int y_from = ttb ? kLo : kHiY;
            const int y_to = ttb ? kHiY : kLo;
            const int dy_scan = ttb ? 1 : -1;
            for (int y = y_from; y != y_to + dy_scan; y += dy_scan) {
                const uint8_t c = g[static_cast<size_t>(y) * kGridW + x];
                if (c == 0u) {
                    continue;
                }
                const int dyp = ((rnd() & 255u) < p_lat) ? lat : 0;
                if (try_move(x, y, x + dx, y + dyp, c)) {
                    continue;
                }
                const int first = (dyp == 0) ? lat : -dyp;
                if (try_move(x, y, x + dx, y + first, c)) {
                    continue;
                }
                static_cast<void>(try_move(x, y, x + dx, y - first, c));
            }
        }
    }
    ++frame_parity_;
    return moved;
}

// ---------------------------------------------------------------------------
// Render lane: step + rasterize
// ---------------------------------------------------------------------------

void FluidBoxApp::draw_sand(uint16_t *buf, int y0, int rows)
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
                out[x] = shade_wire_[c];
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

    // Reset (PLUS / console), consumed exactly once in this lane's context.
    if (reset_requested_.exchange(false)) {
        reset_grid();
        ESP_LOGI(kTag, "PLUS press - sand reset (epoch %u)",
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
    const float sgx = apparent.x;
    const float sgy = -apparent.y;

    // Step the automaton: more substeps the harder the tilt, none at rest.
    const int64_t t_step = esp_timer_get_time();
    const float m2 = sgx * sgx + sgy * sgy;
    if (m2 >= kRestGate * kRestGate) {
        const float m = std::sqrt(m2);
        const int substeps = 1 + (m > kFastTilt ? 1 : 0) + (m > kFullTilt ? 1 : 0);
        uint32_t moved = 0;
        for (int s = 0; s < substeps; ++s) {
            moved += step_sand(sgx, sgy);
        }
        moved_cells_.fetch_add(moved, std::memory_order_relaxed);
    }
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - t_step),
                      std::memory_order_relaxed);

    // Stream the grid through the shell transport, same stripe protocol as
    // every app: retire the carried stripe, latch capture, raster+submit.
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
        draw_sand(buf, y0, rows);
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
    st.count = grain_count_;
    st.epoch = epoch_.load(std::memory_order_relaxed);
    st.candidate_checks = moved_cells_.load(std::memory_order_relaxed);
    st.nonfinite_resets = 0;
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
