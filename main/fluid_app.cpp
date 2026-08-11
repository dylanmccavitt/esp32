#include "fluid_app.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace fluid_demo {

namespace {

constexpr const char *kTag = "fluid_demo";

// Native square panel geometry (mirrors DisplayService constants; the app
// drives sequencing through DisplayFrame ops).
constexpr int kWidth = 240;
constexpr int kHeight = 240;

// Physical volume: 27.72 mm square active LCD and 22.5 mm enclosure depth.
// The front display is z=0; +z enters the case.
constexpr float kBoxHalfX = 0.5f;
constexpr float kBoxHalfY = 0.5f;
constexpr float kBoxDepthZ = 0.812f;

// Perspective makes the 22.5 mm enclosure depth legible without over-shrinking
// particles at the back. The front face is 210 px square with a 15 px border.
constexpr float kFocal = 1.3f;
constexpr float kPxPerWorld = 210.0f;

// Depth fixed point: 1/4096 world unit (Projected::z_fx sort key + fade).
constexpr float kDepthFxScale = 4096.0f;

// Depth-as-opacity: brightness loses up to this many /255 at the back wall.
constexpr uint32_t kDepthFadeMax = 130;


// Dim wireframe color for the 3D box edges (drawn before particles).
constexpr uint16_t kEdgeColor = 0x1905;  // dark blue-gray RGB565

// Velocity palette reference: speed >= kSpeedRef maps to the hottest color.
constexpr float kSpeedRef = 2.5f;

inline int min_int(int a, int b) { return a < b ? a : b; }
inline int max_int(int a, int b) { return a > b ? a : b; }

// Sand speck rendering: every solver grain is a parcel of sand drawn as
// kSpecksPerGrain flat one-pixel specks (2x2 blocks near the front — no
// sphere shading, so nothing can read as a ball). Offsets and per-speck
// brightness come from a 256-entry jitter table built once at setup; the
// per-grain index stride makes every grain's scatter pattern unique across
// the whole population, and the spread (~0.55 of the rest pitch) makes
// neighboring parcels overlap into one continuous bed of sand.
constexpr int kSpecksPerGrain = 16;
constexpr uint32_t kSpeckTableSize = 256;  // power of two for cheap masking

// Speck scatter radius as a multiple of the projected grain radius:
// 0.55 of the rest pitch, with spacing = radius_world / 0.088 (sand build).
constexpr float kSpeckSpreadPerRadius = 0.55f / 0.088f;

}  // namespace

FluidBoxApp s_fluid_app;

FluidBoxApp::~FluidBoxApp()
{
    free_buffers();
}

// ---------------------------------------------------------------------------
// Setup / teardown
// ---------------------------------------------------------------------------

esp_err_t FluidBoxApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;  // idempotent
    }

    if (!fluid_.init(kInitialParticles)) {
        return ESP_ERR_INVALID_ARG;
    }

    Projected *proj = static_cast<Projected *>(
        heap_caps_malloc(sizeof(Projected) * kMaxParticles, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    if (proj == nullptr) {
        ESP_LOGE(kTag, "fluid app buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }

    proj_ = proj;
    build_luts();
    build_specks();
    // setup_once() initializes Fluid before the lanes start. Publish the same
    // initialized epoch/stats immediately so launcher-era telemetry cannot
    // expose the atomics' zero defaults while Fluid already holds epoch 1.
    const FluidStats &initial_stats = fluid_.stats();
    epoch_.store(fluid_.reset_epoch(), std::memory_order_relaxed);
    candidate_checks_.store(initial_stats.candidate_checks, std::memory_order_relaxed);
    nonfinite_resets_.store(initial_stats.nonfinite_resets, std::memory_order_relaxed);
    setup_done_ = true;

    ESP_LOGI(kTag, "fluid initialized: %u particles, radius %.4f",
             static_cast<unsigned>(fluid_.count()), fluid_.particle_radius());
    return ESP_OK;
}

void FluidBoxApp::free_buffers()
{
    heap_caps_free(proj_);
    proj_ = nullptr;
    setup_done_ = false;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

esp_err_t FluidBoxApp::enter()
{
    // Post-barrier: the render lane is idle, so this drain is the sole
    // snapshot consumer. Retire any stale READY frames so the first post-enter
    // render shows a fresh exchange entry rather than a pre-transition frame.
    const ParticleFrame *stale = snapshots_.acquire_latest();
    while (stale != nullptr) {
        snapshots_.release(stale);
        stale = snapshots_.acquire_latest();
    }
    frame_seen_ = false;
    // A pending reset set while inactive stays set: the first update() below
    // consumes it via exchange(false) exactly once (never dropped, never
    // doubled). Running-time requests were already consumed before the lanes
    // acked the quiesce, so nothing stale leaks across the transition.
    return ESP_OK;
}

void FluidBoxApp::leave()
{
    // No allocation; quiesce motion validity so a resumed run never applies a
    // stale gravity vector from a previous lifecycle.
    portENTER_CRITICAL(&motion_mux_);
    motion_.valid = false;
    portEXIT_CRITICAL(&motion_mux_);
}

ShellAction FluidBoxApp::handle_event(AppEvent event)
{
    // PlusPress is the only routed event: it sets the app's reset atomic,
    // consumed exactly once by the first post-enter update(). No event
    // requires a shell action in this slice.
    if (event == AppEvent::PlusPress) {
        request_fluid_reset();
    }
    return ShellAction::None;
}

// ---------------------------------------------------------------------------
// Sensor lane: motion filter + override bypass
// ---------------------------------------------------------------------------

bool FluidBoxApp::on_motion(const MotionTick &tick)
{
    if (!tick.fresh) {
        // A stale/failed IMU read still honors an active override (legacy
        // app_main.cpp:313-320): the injected value publishes, the last valid
        // raw read stays untouched.
        if (tick.override_active) {
            portENTER_CRITICAL(&motion_mux_);
            motion_.apparent_accel = tick.apparent_accel;
            motion_.valid = true;
            portEXIT_CRITICAL(&motion_mux_);
        }
        return false;
    }

    // The filter always tracks the physical IMU, so a later `release` resumes
    // from a live state. An override only replaces the published apparent
    // value (bypasses the filter verbatim, matching app_main.cpp:296-320).
    const Vec3 apparent = filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
    const bool accepted = filter_.last_sample_accepted();
    if (!accepted) {
        // A rejected fresh sample keeps the last valid raw/filter state, but
        // an active override still publishes verbatim (the legacy override
        // check ran after the filter branch regardless of acceptance).
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

void FluidBoxApp::request_fluid_reset()
{
    reset_requested_.store(true, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Physics lane: consume reset, step, publish
// ---------------------------------------------------------------------------

esp_err_t FluidBoxApp::update(float dt)
{
    // Handle a reset notification in this lane's context: swap the flag, then
    // re-roll the fluid into its deterministic lattice. A request issued while
    // inactive was never cleared by enter(), so this is its exactly-once
    // consumption point.
    if (reset_requested_.exchange(false)) {
        fluid_.reset();
        ESP_LOGI(kTag, "PLUS press - fluid reset (epoch %u)", fluid_.reset_epoch());
    }

    // Latest apparent acceleration from the sensor lane.
    Vec3 apparent{0.0f, 0.0f, 6.0f};
    portENTER_CRITICAL(&motion_mux_);
    if (motion_.valid) {
        apparent = motion_.apparent_accel;
    }
    portEXIT_CRITICAL(&motion_mux_);

    // step() returns false only on a skip (never happens here: count is set at
    // boot and dt is fixed finite) or when nonfinite state forced a
    // deterministic reset — either way the frame below is still valid.
    const int64_t step_start_us = esp_timer_get_time();
    static_cast<void>(fluid_.step(apparent, dt));
    physics_us_.store(static_cast<uint32_t>(esp_timer_get_time() - step_start_us));
    const FluidStats &fluid_stats = fluid_.stats();
    epoch_.store(fluid_.reset_epoch());
    candidate_checks_.store(fluid_stats.candidate_checks);
    nonfinite_resets_.store(fluid_stats.nonfinite_resets);

    ParticleFrame *slot = snapshots_.begin_write();
    if (slot == nullptr) {
        return ESP_OK;  // defensive pool exhaustion; the lane guard preserves WDT
    }
    fluid_.fill_frame(*slot, ++sequence_);
    snapshots_.publish(slot);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Render lane
// ---------------------------------------------------------------------------

AppStats FluidBoxApp::stats()
{
    AppStats st = {};
    st.count = fluid_.count();
    st.epoch = epoch_.load();
    st.candidate_checks = candidate_checks_.load();
    st.nonfinite_resets = nonfinite_resets_.load();
    st.physics_us = physics_us_.load();
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

bool FluidBoxApp::render(DisplayFrame &frame)
{
    if (!setup_done_ || frame.transport == nullptr) {
        ESP_LOGW(kTag, "render failed: %s", esp_err_to_name(ESP_ERR_INVALID_STATE));
        return false;
    }
    const ParticleFrame *snapshot = snapshots_.acquire_latest();
    if (snapshot == nullptr) {
        return false;  // blank pass: no new frame, no transport touched
    }
    const esp_err_t ret = render_frame(*snapshot, frame);
    snapshots_.release(snapshot);
    if (ret != ESP_OK) {
        ESP_LOGW(kTag, "render failed: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

esp_err_t FluidBoxApp::render_frame(const ParticleFrame &frame, DisplayFrame &df)
{
    const int64_t t_frame = esp_timer_get_time();
    uint32_t raster_total = 0;

    // The previous frame leaves its final stripe queued. Retire it before
    // touching either ping-pong buffer; after a timeout, retrying this wait on
    // the next frame remains safe because the in-flight flag stays set.
    if (df.ops.wait_previous(df.transport) != ESP_OK) {
        return ESP_ERR_TIMEOUT;
    }

    const int count = (frame.count > kMaxParticles) ? kMaxParticles : frame.count;
    preproject(frame, count);
    project_box_edges();
    const int64_t t_sort = esp_timer_get_time();
    sort_back_to_front();
    raster_total += static_cast<uint32_t>(esp_timer_get_time() - t_sort);

    // Frame-boundary capture latch (the old renderer.cpp:602 exchange point).
    // The DisplayService mirrors when armed and re-arms on any mid-frame
    // failure, so a request is never latched lazily mid-frame.
    static_cast<void>(df.ops.latch_capture(df.transport));

    for (int s = 0; s < df.stripe_count; s++) {
        const int y0 = s * df.stripe_rows;
        const int rows = min_int(df.stripe_rows, df.height - y0);
        uint16_t *buf = df.stripe[s & 1];

        // Rasterize into the idle stripe buffer while the previous stripe's DMA
        // (stripe s-1, the other buffer) is still running.
        const int64_t t_raster = esp_timer_get_time();
        std::memset(buf, 0, static_cast<size_t>(df.width) * rows * sizeof(uint16_t));
        draw_box_edges(buf, y0, rows);
        draw_particles(buf, y0, rows);
        raster_total += static_cast<uint32_t>(esp_timer_get_time() - t_raster);

        // Mirror (if armed) -> wait for the previous transfer (including the
        // one carried from the previous frame) -> draw -> mark in flight.
        esp_err_t ret = df.ops.submit(df.transport, s, y0, rows, buf);
        if (ret != ESP_OK) {
            // The DisplayService handles re-arming a failed capture; frame
            // telemetry keeps the last completed frame's values.
            return ret;
        }
    }

    esp_err_t ret = df.ops.finish(df.transport);
    if (ret != ESP_OK) {
        return ret;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - t_frame);
    // The DisplayService resets its capture-copy counter at each latch and
    // accumulates PSRAM mirror time across the armed frame's stripes; the
    // reported raster duration includes it (byte-identical to the pre-split
    // renderer's frame telemetry).
    raster_us_ = raster_total + df.ops.capture_copy_us(df.transport);
    frame_seen_ = true;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Raster helpers (from renderer.cpp; splat support, iso-threshold, AA width and depth fade retuned)
// ---------------------------------------------------------------------------

void FluidBoxApp::build_luts()
{
    // Sand palette: dim warm tan for resting grains ramping to bright
    // sun-lit amber for fast ones (hue 38 -> 28 deg, modest saturation),
    // so the medium reads as sand rather than colored balls.
    for (int i = 0; i < 256; i++) {
        const float u = static_cast<float>(i) / 255.0f;
        const float hue = 38.0f - 10.0f * u;
        const float sat = 0.50f + 0.25f * u;
        const float val = 0.62f + 0.38f * u;  // floor keeps resting sand visible
        palette_[i] = hsv_to_rgb565(hue, sat, val);
    }
}

void FluidBoxApp::build_specks()
{
    // Deterministic LCG fills the jitter table once at setup: offsets are
    // rejection-sampled to the unit disc (1/128 fixed point) so parcels stay
    // round, and the brightness jitter gives the bed its granular sparkle.
    uint32_t state = 0x2545F491u;
    const auto next = [&state]() {
        state = state * 1664525u + 1013904223u;
        return state;
    };
    for (uint32_t i = 0; i < kSpeckTableSize; ++i) {
        int ox = 0;
        int oy = 0;
        for (;;) {
            ox = static_cast<int>(next() >> 24) - 128;  // -128..127
            oy = static_cast<int>(next() >> 24) - 128;
            if (ox * ox + oy * oy <= 127 * 127) {
                break;
            }
        }
        speck_lut_[i].x = static_cast<int8_t>(ox == -128 ? -127 : ox);
        speck_lut_[i].y = static_cast<int8_t>(oy == -128 ? -127 : oy);
        speck_lut_[i].bright = static_cast<uint8_t>(150u + (next() % 106u));
    }
    ESP_LOGI(kTag, "speck jitter table built: %u entries, %d specks/grain",
             static_cast<unsigned>(kSpeckTableSize), kSpecksPerGrain);
}

uint16_t FluidBoxApp::hsv_to_rgb565(float h, float s, float v)
{
    const float c = v * s;
    const float hp = h / 60.0f;
    const float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
    float r = 0.0f;
    float g = 0.0f;
    float bl = 0.0f;
    if (hp < 1.0f) {
        r = c;
        g = x;
    } else if (hp < 2.0f) {
        r = x;
        g = c;
    } else if (hp < 3.0f) {
        g = c;
        bl = x;
    } else if (hp < 4.0f) {
        g = x;
        bl = c;
    } else if (hp < 5.0f) {
        r = x;
        bl = c;
    } else {
        r = c;
        bl = x;
    }
    const float m = v - c;
    r += m;
    g += m;
    bl += m;
    const uint32_t r5 = static_cast<uint32_t>(r * 31.0f + 0.5f) & 0x1Fu;
    const uint32_t g6 = static_cast<uint32_t>(g * 63.0f + 0.5f) & 0x3Fu;
    const uint32_t b5 = static_cast<uint32_t>(bl * 31.0f + 0.5f) & 0x1Fu;
    return static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5);
}

void FluidBoxApp::preproject(const ParticleFrame &frame, int count)
{
    active_count_ = count;
    const float radius_world = frame.particle_radius;
    for (int i = 0; i < kMaxParticles; i++) {
        proj_[i].active = 0;
    }
    for (int i = 0; i < count; i++) {
        Projected &p = proj_[i];
        const RenderParticle &src = frame.particles[i];
        if (!std::isfinite(src.x) || !std::isfinite(src.y) || !std::isfinite(src.z) ||
            !std::isfinite(src.speed) || radius_world <= 0.0f) {
            continue;
        }
        const float scale = kPxPerWorld * (kFocal / (kFocal + src.z));
        const float fx = kWidth * 0.5f + src.x * scale;
        // World +y is screen-up; particles rest at the bottom of the box.
        const float fy = kHeight * 0.5f - src.y * scale;
        const float fr = radius_world * scale;

        constexpr uint32_t kBoxDepthFx =
            static_cast<uint32_t>(kBoxDepthZ * kDepthFxScale + 0.5f);
        float zc = (src.z < 0.0f) ? 0.0f : src.z;
        p.z_fx = static_cast<uint32_t>(zc * kDepthFxScale + 0.5f);
        uint32_t drop = (p.z_fx * kDepthFadeMax) / kBoxDepthFx;
        if (drop > kDepthFadeMax) {
            drop = kDepthFadeMax;
        }
        const uint8_t fresh_fade = static_cast<uint8_t>(255u - drop);

        const float sp = (src.speed < 0.0f) ? 0.0f : src.speed;
        const uint8_t fresh_speed =
            (sp >= kSpeedRef) ? 255u
                              : static_cast<uint8_t>((sp / kSpeedRef) * 255.0f);

        // Grain radius only sizes the speck blocks: 2x2 px near the front
        // (rad >= 2), a single pixel deeper in.
        int rad = static_cast<int>(std::lrintf(fr));
        if (rad < 1) {
            rad = 1;
        }
        if (rad > 3) {
            rad = 3;
        }
        int spread = static_cast<int>(std::lrintf(fr * kSpeckSpreadPerRadius));
        if (spread > 255) {
            spread = 255;
        }
        p.x = static_cast<int16_t>(std::lrintf(fx));
        p.y = static_cast<int16_t>(std::lrintf(fy));
        p.radius = static_cast<uint16_t>(rad);
        p.spread_px = static_cast<uint8_t>(spread);
        p.fade = fresh_fade;
        p.speed_idx = fresh_speed;
        p.active = 1;
    }
}

void FluidBoxApp::sort_back_to_front()
{
    // Painter's order: far particles first, near ones drawn over them. With
    // temporal coherence std::sort on ~343 u16 indices costs tens of
    // microseconds; no depth buffer or per-pixel blend is needed.
    int n = 0;
    for (int i = 0; i < active_count_; ++i) {
        if (proj_[i].active) {
            draw_order_[n++] = static_cast<uint16_t>(i);
        }
    }
    active_count_ = n;
    // Quantized depth key (1/64 world unit) with an index tie-break: beads at
    // near-identical depth keep a fixed overlap order, so the solver's rest
    // wobble cannot flutter the painter's order frame to frame.
    std::sort(draw_order_, draw_order_ + n, [this](uint16_t a, uint16_t b) {
        const uint32_t za = proj_[a].z_fx >> 6;
        const uint32_t zb = proj_[b].z_fx >> 6;
        return za != zb ? za > zb : a < b;
    });
}

void FluidBoxApp::project_box_edges()
{
    constexpr float corners[8][3] = {
        {-kBoxHalfX, kBoxHalfY, 0.0f},   // front TL
        {kBoxHalfX, kBoxHalfY, 0.0f},    // front TR
        {kBoxHalfX, -kBoxHalfY, 0.0f},   // front BR
        {-kBoxHalfX, -kBoxHalfY, 0.0f},  // front BL
        {-kBoxHalfX, kBoxHalfY, kBoxDepthZ},    // back TL
        {kBoxHalfX, kBoxHalfY, kBoxDepthZ},     // back TR
        {kBoxHalfX, -kBoxHalfY, kBoxDepthZ},    // back BR
        {-kBoxHalfX, -kBoxHalfY, kBoxDepthZ},   // back BL
    };
    int sx[8] = {};
    int sy[8] = {};
    for (int i = 0; i < 8; i++) {
        const float scale = kPxPerWorld * (kFocal / (kFocal + corners[i][2]));
        sx[i] = static_cast<int>(std::lrintf(kWidth * 0.5f + corners[i][0] * scale));
        sy[i] = static_cast<int>(std::lrintf(kHeight * 0.5f - corners[i][1] * scale));
    }
    // Front face (z=0): 0-1-2-3; back face: 4-5-6-7; connectors front->back.
    edges_[0] = {sx[0], sy[0], sx[1], sy[1]};
    edges_[1] = {sx[1], sy[1], sx[2], sy[2]};
    edges_[2] = {sx[2], sy[2], sx[3], sy[3]};
    edges_[3] = {sx[3], sy[3], sx[0], sy[0]};
    edges_[4] = {sx[4], sy[4], sx[5], sy[5]};
    edges_[5] = {sx[5], sy[5], sx[6], sy[6]};
    edges_[6] = {sx[6], sy[6], sx[7], sy[7]};
    edges_[7] = {sx[7], sy[7], sx[4], sy[4]};
    edges_[8] = {sx[0], sy[0], sx[4], sy[4]};
    edges_[9] = {sx[1], sy[1], sx[5], sy[5]};
    edges_[10] = {sx[2], sy[2], sx[6], sy[6]};
    edges_[11] = {sx[3], sy[3], sx[7], sy[7]};
}

void FluidBoxApp::draw_box_edges(uint16_t *buf, int y0, int rows)
{
    const int y1 = y0 + rows;
    const uint16_t edge = __builtin_bswap16(kEdgeColor);
    for (int e = 0; e < 12; e++) {
        const Edge &seg = edges_[e];
        const int ya = min_int(seg.y0, seg.y1);
        const int yb = max_int(seg.y0, seg.y1);
        if (yb < y0 || ya >= y1) {
            continue;
        }
        if (ya == yb) {
            // Horizontal span on a single row.
            if (ya < y0 || ya >= y1) {
                continue;
            }
            int xa = min_int(seg.x0, seg.x1);
            int xb = max_int(seg.x0, seg.x1);
            xa = max_int(xa, 0);
            xb = min_int(xb, kWidth - 1);
            uint16_t *row = buf + static_cast<size_t>(ya - y0) * kWidth;
            for (int x = xa; x <= xb; x++) {
                row[x] = edge;
            }
            continue;
        }
        // Diagonal (or vertical) edge: step x per row with 16-bit fraction.
        const int xa = (seg.y0 == ya) ? seg.x0 : seg.x1;
        const int xb = (seg.y0 == ya) ? seg.x1 : seg.x0;
        const int dy = yb - ya;  // > 0
        int32_t x16 = static_cast<int32_t>(xa) << 16;
        int32_t step = (static_cast<int32_t>(xb - xa) << 16) / dy;
        int r0 = max_int(ya, y0);
        const int r1 = min_int(yb, y1 - 1);
        x16 += step * (r0 - ya);
        for (int y = r0; y <= r1; y++) {
            const int x = static_cast<int>(x16 >> 16);
            if (x >= 0 && x < kWidth) {
                buf[static_cast<size_t>(y - y0) * kWidth + x] = edge;
            }
            x16 += step;
        }
    }
}

void FluidBoxApp::draw_particles(uint16_t *buf, int y0, int rows)
{
    // Painter's blit of pre-shaded sphere sprites, clipped to this stripe.
    // Premultiplied sprite brightness scales the particle's velocity color;
    // texel 0 is transparent, so each ball keeps a crisp antialiased edge and
    // a subtle dark rim where it overlaps a deeper ball.
    const int y1 = y0 + rows;
    for (int n = 0; n < active_count_; ++n) {
        const uint16_t gi = draw_order_[n];
        const Projected &p = proj_[gi];
        const uint16_t pal = palette_[p.speed_idx];
        const uint32_t pr = (pal >> 11) & 0x1Fu;
        const uint32_t pg = (pal >> 5) & 0x3Fu;
        const uint32_t pb = pal & 0x1Fu;
        const uint32_t fade = p.fade;
        const int spread = static_cast<int>(p.spread_px);
        const int block = (p.radius >= 2) ? 2 : 1;

        // The index stride (odd constants mod the power-of-two table) gives
        // every grain in the population a distinct scatter pattern.
        for (int k = 0; k < kSpecksPerGrain; ++k) {
            const Speck &s = speck_lut_[
                (static_cast<uint32_t>(gi) * 61u + static_cast<uint32_t>(k) * 97u) &
                (kSpeckTableSize - 1)];
            const int cx = p.x + ((static_cast<int>(s.x) * spread) >> 7);
            const int cy = p.y + ((static_cast<int>(s.y) * spread) >> 7);
            const int ry0 = max_int(cy, y0);
            const int ry1 = min_int(cy + block - 1, y1 - 1);
            const int rx0 = max_int(cx, 0);
            const int rx1 = min_int(cx + block - 1, kWidth - 1);
            if (ry0 > ry1 || rx0 > rx1) {
                continue;
            }
            const uint32_t bright = (fade * s.bright) >> 8;
            const uint32_t r5 = (pr * bright) >> 8;
            const uint32_t g6 = (pg * bright) >> 8;
            const uint32_t b5 = (pb * bright) >> 8;
            const uint16_t px = __builtin_bswap16(
                static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5));
            for (int y = ry0; y <= ry1; ++y) {
                uint16_t *out = buf + static_cast<size_t>(y - y0) * kWidth;
                for (int x = rx0; x <= rx1; ++x) {
                    out[x] = px;
                }
            }
        }
    }
}

}  // namespace fluid_demo
