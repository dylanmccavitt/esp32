#include "renderer.hpp"

#include <cmath>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace fluid_demo {

namespace {

constexpr const char *kTAG = "renderer";

// Native square panel geometry.
constexpr int kWidth = 240;
constexpr int kHeight = 240;
constexpr int kStripeRows = 16;
constexpr int kStripeCount = kHeight / kStripeRows;  // 15 full stripes
constexpr int kSurfaceScale = 2;
constexpr int kSurfaceWidth = kWidth / kSurfaceScale;
constexpr int kSurfaceHeight = kHeight / kSurfaceScale;
constexpr uint16_t kSurfaceThreshold = 40;
constexpr uint16_t kSurfaceAaWidth = 28;

// Physical volume: 27.72 mm square active LCD and 22.5 mm enclosure depth.
// The front display is z=0; +z enters the case.
constexpr float kBoxHalfX = 0.5f;
constexpr float kBoxHalfY = 0.5f;
constexpr float kBoxDepthZ = 0.812f;

// Perspective makes the 22.5 mm enclosure depth legible without over-shrinking
// particles at the back. The front face is 210 px square with a 15 px border.
constexpr float kFocal = 1.3f;
constexpr float kPxPerWorld = 210.0f;

// Depth buffer fixed point: 1/4096 world unit. All particle surfaces stay far
// below 65535, so 0xFFFF (bytes 0xFF) is a valid "empty" marker.
constexpr float kDepthFxScale = 4096.0f;

// Dim wireframe color for the 3D box edges (drawn before particles).
constexpr uint16_t kEdgeColor = 0x1905;  // dark blue-gray RGB565

// A 240x16 RGB565 stripe is 7680 bytes. At 40 MHz one-bit SPI it transfers in
// roughly 1.5 ms; 200 ms is a generous hardware-fault margin.
constexpr int kDmaWaitTimeoutMs = 200;

// Velocity palette reference: speed >= kSpeedRef maps to the hottest color.
constexpr float kSpeedRef = 2.5f;

inline int min_int(int a, int b) { return a < b ? a : b; }
inline int max_int(int a, int b) { return a > b ? a : b; }

}  // namespace

Renderer::Renderer() = default;

Renderer::~Renderer()
{
    free_buffers();
}

esp_err_t Renderer::init(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t io)
{
    if (panel == nullptr || io == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (initialized_) {
        return ESP_OK;  // idempotent
    }

    const size_t stripe_bytes = (size_t)kWidth * kStripeRows * sizeof(uint16_t);
    const size_t surface_bytes =
        (size_t)kSurfaceWidth * kSurfaceHeight * sizeof(uint16_t);
    const size_t capture_bytes = kCaptureBytes;
    uint16_t *a = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    uint16_t *b = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    uint16_t *field = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, surface_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    uint16_t *heat = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, surface_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    uint16_t *depth = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, surface_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    uint16_t *capture = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, capture_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    Projected *proj = static_cast<Projected *>(
        heap_caps_malloc(sizeof(Projected) * kMaxParticles, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    SemaphoreHandle_t sem = xSemaphoreCreateBinary();

    if (a == nullptr || b == nullptr || field == nullptr || heat == nullptr ||
        depth == nullptr || capture == nullptr || proj == nullptr || sem == nullptr) {
        heap_caps_free(a);
        heap_caps_free(b);
        heap_caps_free(field);
        heap_caps_free(heat);
        heap_caps_free(depth);
        heap_caps_free(capture);
        heap_caps_free(proj);
        if (sem != nullptr) {
            vSemaphoreDelete(sem);
        }
        ESP_LOGE(kTAG, "surface renderer buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }

    stripe_[0] = a;
    stripe_[1] = b;
    surface_field_ = field;
    surface_heat_ = heat;
    surface_depth_ = depth;
    capture_ = capture;
    proj_ = proj;
    sem_ = sem;
    build_luts();

    esp_lcd_panel_io_callbacks_t cbs = {};
    cbs.on_color_trans_done = &Renderer::on_color_trans_done;
    esp_err_t ret = esp_lcd_panel_io_register_event_callbacks(io, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(kTAG, "register on_color_trans_done failed: %s", esp_err_to_name(ret));
        free_buffers();
        return ret;
    }

    panel_ = panel;
    io_ = io;
    initialized_ = true;
    ESP_LOGI(kTAG, "init: 240x240x16bpp, %d stripes x %d rows, %dx%d connected surface",
             kStripeCount, kStripeRows, kSurfaceWidth, kSurfaceHeight);
    return ESP_OK;
}

void Renderer::free_buffers()
{
    if (sem_ != nullptr) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(sem_));
        sem_ = nullptr;
    }
    heap_caps_free(stripe_[0]);
    heap_caps_free(stripe_[1]);
    heap_caps_free(surface_field_);
    heap_caps_free(surface_heat_);
    heap_caps_free(surface_depth_);
    heap_caps_free(capture_);
    heap_caps_free(proj_);
    stripe_[0] = stripe_[1] = nullptr;
    surface_field_ = nullptr;
    surface_heat_ = nullptr;
    surface_depth_ = nullptr;
    capture_ = nullptr;
    proj_ = nullptr;
    transfer_in_flight_ = false;
    initialized_ = false;
}

RenderStats Renderer::stats() const
{
    RenderStats s = {};
    s.frame_us = frame_us_;
    s.raster_us = raster_us_;
    s.dma_wait_us = dma_wait_us_;
    s.missed_transfers = missed_transfers_;
    return s;
}

esp_err_t Renderer::request_capture()
{
    if (!initialized_ || capture_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (capture_requested_.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }
    capture_ready_.store(false, std::memory_order_release);
    bool expected = false;
    if (!capture_requested_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel, std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

bool Renderer::capture_ready() const
{
    return capture_ready_.load(std::memory_order_acquire);
}

const uint8_t *Renderer::capture_data() const
{
    return reinterpret_cast<const uint8_t *>(capture_);
}

bool Renderer::on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                             esp_lcd_panel_io_event_data_t *edata,
                                             void *user_ctx)
{
    (void)io;
    (void)edata;
    Renderer *self = static_cast<Renderer *>(user_ctx);
    return self->trans_done_isr();
}

bool Renderer::trans_done_isr()
{
    BaseType_t hpw = pdFALSE;
    if (sem_ != nullptr) {
        xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(sem_), &hpw);
    }
    return hpw == pdTRUE;
}

bool Renderer::wait_previous_transfer(uint32_t *wait_us)
{
    *wait_us = 0;
    if (!transfer_in_flight_) {
        return true;
    }
    SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(sem_);
    int64_t t0 = esp_timer_get_time();
    const bool ok = xSemaphoreTake(sem, pdMS_TO_TICKS(kDmaWaitTimeoutMs)) == pdTRUE;
    *wait_us = static_cast<uint32_t>(esp_timer_get_time() - t0);
    if (!ok) {
        missed_transfers_++;
        return false;
    }
    transfer_in_flight_ = false;
    return true;
}

void Renderer::build_luts()
{
    // sqrt/shade LUT: nz = sqrt(1 - t), t = (d/r)^2, in [0, 1].
    // Used both for the front-surface depth reduction and the radial shade.
    // Fixed point 0.16.
    for (int i = 0; i < 256; i++) {
        const float t = static_cast<float>(i) / 255.0f;
        const float nz = std::sqrt(1.0f - t);
        nz_lut_[i] = static_cast<uint16_t>(nz * 65535.0f + 0.5f);
    }

    // Velocity palette: slow = blue/cyan, fast = orange/red (hue 230 -> 0 deg),
    // ramping saturation/value so slow particles are dim and fast ones glow.
    for (int i = 0; i < 256; i++) {
        const float u = static_cast<float>(i) / 255.0f;
        const float hue = (1.0f - u) * 230.0f;
        const float sat = 0.85f;
        const float val = 0.45f + 0.55f * u;
        palette_[i] = hsv_to_rgb565(hue, sat, val);
    }
}

uint16_t Renderer::hsv_to_rgb565(float h, float s, float v)
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

void Renderer::preproject(const ParticleFrame &frame, int count)
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
        const int sx = static_cast<int>(std::lrintf(kWidth * 0.5f + src.x * scale));
        // World +y is screen-up; particles rest at the bottom of the box.
        const int sy = static_cast<int>(std::lrintf(kHeight * 0.5f - src.y * scale));
        int rad = static_cast<int>(std::lrintf(radius_world * scale));
        if (rad < 1) {
            rad = 1;
        }
        p.x = static_cast<int16_t>(sx);
        p.y = static_cast<int16_t>(sy);
        p.radius = static_cast<uint16_t>(rad);

        float zc = (src.z < 0.0f) ? 0.0f : src.z;
        p.z_fx = static_cast<uint32_t>(zc * kDepthFxScale + 0.5f);
        p.rw_fx = static_cast<uint32_t>(radius_world * kDepthFxScale + 0.5f);

        const float sp = (src.speed < 0.0f) ? 0.0f : src.speed;
        p.speed_idx = (sp >= kSpeedRef) ? 255u
                                        : static_cast<uint8_t>((sp / kSpeedRef) * 255.0f);
        p.active = 1;
    }
}

void Renderer::project_box_edges()
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

void Renderer::draw_box_edges(uint16_t *buf, int y0, int rows)
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

void Renderer::build_surface()
{
    constexpr size_t kSurfacePixels =
        static_cast<size_t>(kSurfaceWidth) * kSurfaceHeight;
    std::memset(surface_field_, 0, kSurfacePixels * sizeof(uint16_t));
    std::memset(surface_heat_, 0, kSurfacePixels * sizeof(uint16_t));
    std::memset(surface_depth_, 0xFF, kSurfacePixels * sizeof(uint16_t));

    // Screen-space implicit surface. At half resolution, a support radius equal
    // to the full-resolution particle radius is twice the drawn particle radius
    // in screen space. Neighbor kernels therefore meet near the iso-threshold,
    // turning the point cloud into one smooth liquid body.
    for (int i = 0; i < active_count_; ++i) {
        const Projected &p = proj_[i];
        if (!p.active) {
            continue;
        }
        const int cx = p.x / kSurfaceScale;
        const int cy = p.y / kSurfaceScale;
        const int radius = max_int(static_cast<int>(p.radius), 2);
        const uint32_t r2 = static_cast<uint32_t>(radius * radius);
        const uint32_t d2_mul =
            static_cast<uint32_t>((255.0f * 65536.0f) / static_cast<float>(r2) + 0.5f);
        const int x0 = max_int(cx - radius, 0);
        const int x1 = min_int(cx + radius, kSurfaceWidth - 1);
        const int y0 = max_int(cy - radius, 0);
        const int y1 = min_int(cy + radius, kSurfaceHeight - 1);

        for (int y = y0; y <= y1; ++y) {
            const int dy = y - cy;
            const uint32_t dy2 = static_cast<uint32_t>(dy * dy);
            const size_t row = static_cast<size_t>(y) * kSurfaceWidth;
            for (int x = x0; x <= x1; ++x) {
                const int dx = x - cx;
                const uint32_t d2 = static_cast<uint32_t>(dx * dx) + dy2;
                const uint32_t lut_idx = (d2 * d2_mul) >> 16;
                if (lut_idx >= 256u) {
                    continue;
                }

                const uint32_t falloff = 255u - lut_idx;
                const uint32_t weight = (falloff * falloff) >> 8;
                if (weight == 0u) {
                    continue;
                }
                const size_t at = row + static_cast<size_t>(x);
                const uint32_t field = surface_field_[at] + weight;
                surface_field_[at] =
                    static_cast<uint16_t>(field > UINT16_MAX ? UINT16_MAX : field);

                // Store heat in field-weight units. Dividing heat by field when
                // shading gives a smooth weighted velocity palette index.
                const uint32_t heat_add = (weight * p.speed_idx + 127u) / 255u;
                const uint32_t heat = surface_heat_[at] + heat_add;
                surface_heat_[at] =
                    static_cast<uint16_t>(heat > UINT16_MAX ? UINT16_MAX : heat);

                const uint32_t nz = nz_lut_[lut_idx];
                int32_t surf = static_cast<int32_t>(p.z_fx) -
                               static_cast<int32_t>((p.rw_fx * nz) >> 16);
                if (surf < 0) {
                    surf = 0;
                }
                if (static_cast<uint32_t>(surf) < surface_depth_[at]) {
                    surface_depth_[at] = static_cast<uint16_t>(surf);
                }
            }
        }
    }
}

void Renderer::shade_surface(uint16_t *buf, int y0, int rows)
{
    constexpr uint32_t kBoxDepthFx =
        static_cast<uint32_t>(kBoxDepthZ * kDepthFxScale + 0.5f);
    const int y1 = y0 + rows;
    for (int y = y0; y < y1; ++y) {
        const int sy = y / kSurfaceScale;
        const int sy1 = min_int(sy + 1, kSurfaceHeight - 1);
        const int sym = max_int(sy - 1, 0);
        const int fy = y & 1;
        const uint32_t wy0 = static_cast<uint32_t>(2 - fy);
        const uint32_t wy1 = static_cast<uint32_t>(fy);
        const size_t row0 = static_cast<size_t>(sy) * kSurfaceWidth;
        const size_t row1 = static_cast<size_t>(sy1) * kSurfaceWidth;
        const size_t rowm = static_cast<size_t>(sym) * kSurfaceWidth;
        uint16_t *out = buf + static_cast<size_t>(y - y0) * kWidth;

        for (int x = 0; x < kWidth; ++x) {
            const int sx = x / kSurfaceScale;
            const int sx1 = min_int(sx + 1, kSurfaceWidth - 1);
            const int sxm = max_int(sx - 1, 0);
            const int fx = x & 1;
            const uint32_t wx0 = static_cast<uint32_t>(2 - fx);
            const uint32_t wx1 = static_cast<uint32_t>(fx);

            const uint32_t w00 = wx0 * wy0;
            const uint32_t w10 = wx1 * wy0;
            const uint32_t w01 = wx0 * wy1;
            const uint32_t w11 = wx1 * wy1;
            const uint32_t field =
                (surface_field_[row0 + sx] * w00 +
                 surface_field_[row0 + sx1] * w10 +
                 surface_field_[row1 + sx] * w01 +
                 surface_field_[row1 + sx1] * w11) >> 2;
            if (field < kSurfaceThreshold) {
                continue;
            }
            const uint32_t heat =
                (surface_heat_[row0 + sx] * w00 +
                 surface_heat_[row0 + sx1] * w10 +
                 surface_heat_[row1 + sx] * w01 +
                 surface_heat_[row1 + sx1] * w11) >> 2;
            uint32_t speed_idx = (heat * 255u + field / 2u) / field;
            if (speed_idx > 255u) {
                speed_idx = 255u;
            }

            // The scalar-field gradient supplies a continuous surface normal,
            // eliminating per-particle circular highlights. Light comes from
            // upper-left; accumulated field adds a modest body highlight.
            const int32_t gx = static_cast<int32_t>(surface_field_[row0 + sx1]) -
                               static_cast<int32_t>(surface_field_[row0 + sxm]);
            const int32_t gy = static_cast<int32_t>(surface_field_[row1 + sx]) -
                               static_cast<int32_t>(surface_field_[rowm + sx]);
            int32_t brightness =
                188 + min_int(static_cast<int>((field - kSurfaceThreshold) >> 2), 42) +
                (gx + gy) / 8;
            brightness = max_int(72, min_int(brightness, 255));

            uint16_t z = surface_depth_[row0 + sx];
            const uint16_t z10 = surface_depth_[row0 + sx1];
            const uint16_t z01 = surface_depth_[row1 + sx];
            const uint16_t z11 = surface_depth_[row1 + sx1];
            if (z10 < z) z = z10;
            if (z01 < z) z = z01;
            if (z11 < z) z = z11;
            if (z != UINT16_MAX) {
                const uint32_t fade =
                    255u - min_int(static_cast<int>((static_cast<uint32_t>(z) * 65u) /
                                                   kBoxDepthFx),
                                   65);
                brightness = static_cast<int32_t>(
                    (static_cast<uint32_t>(brightness) * fade) >> 8);
            }

            // Fade the first 28 field units above the iso-threshold for a
            // one-pixel antialiased silhouette after 2x bilinear upsampling.
            const uint32_t coverage =
                min_int(static_cast<int>(((field - kSurfaceThreshold) * 255u) /
                                         kSurfaceAaWidth),
                        255);
            brightness = static_cast<int32_t>(
                (static_cast<uint32_t>(brightness) * coverage) >> 8);

            const uint16_t pal = palette_[speed_idx];
            const uint32_t r5 = ((pal >> 11) & 0x1Fu) * brightness >> 8;
            const uint32_t g6 = ((pal >> 5) & 0x3Fu) * brightness >> 8;
            const uint32_t b5 = (pal & 0x1Fu) * brightness >> 8;
            out[x] = __builtin_bswap16(
                static_cast<uint16_t>((r5 << 11) | (g6 << 5) | b5));
        }
    }
}

esp_err_t Renderer::render(const ParticleFrame &frame)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t t_frame = esp_timer_get_time();
    uint32_t raster_total = 0;
    uint32_t dma_wait_total = 0;

    // The previous frame leaves its final stripe queued. Retire it before
    // touching either ping-pong buffer; after a timeout, retrying this wait on
    // the next frame remains safe because the in-flight flag stays set.
    uint32_t carry_wait_us = 0;
    if (!wait_previous_transfer(&carry_wait_us)) {
        return ESP_ERR_TIMEOUT;
    }
    dma_wait_total += carry_wait_us;

    const int count = (frame.count > kMaxParticles) ? kMaxParticles : frame.count;
    preproject(frame, count);
    project_box_edges();
    const int64_t t_surface = esp_timer_get_time();
    build_surface();
    raster_total += static_cast<uint32_t>(esp_timer_get_time() - t_surface);
    const bool capture_frame = capture_requested_.exchange(false, std::memory_order_acq_rel);

    for (int s = 0; s < kStripeCount; s++) {
        const int y0 = s * kStripeRows;
        const int rows = min_int(kStripeRows, kHeight - y0);
        uint16_t *buf = stripe_[s & 1];

        // Rasterize into the idle stripe buffer while the previous stripe's DMA
        // (stripe s-1, the other buffer) is still running.
        const int64_t t_raster = esp_timer_get_time();
        std::memset(buf, 0, static_cast<size_t>(kWidth) * rows * sizeof(uint16_t));
        draw_box_edges(buf, y0, rows);
        shade_surface(buf, y0, rows);
        if (capture_frame) {
            std::memcpy(capture_ + static_cast<size_t>(y0) * kWidth, buf,
                        static_cast<size_t>(kWidth) * rows * sizeof(uint16_t));
        }
        raster_total += static_cast<uint32_t>(esp_timer_get_time() - t_raster);

        // Explicitly wait for the previous transfer (including the one carried
        // from the previous frame) before submitting the next stripe.
        uint32_t wait_us = 0;
        if (!wait_previous_transfer(&wait_us)) {
            if (capture_frame) {
                capture_requested_.store(true, std::memory_order_release);
            }
            return ESP_ERR_TIMEOUT;
        }
        dma_wait_total += wait_us;

        esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_, 0, y0, kWidth, y0 + rows, buf);
        if (ret != ESP_OK) {
            if (capture_frame) {
                capture_requested_.store(true, std::memory_order_release);
            }
            ESP_LOGE(kTAG, "draw_bitmap stripe %d failed: %s", s, esp_err_to_name(ret));
            return ret;
        }
        transfer_in_flight_ = true;  // exactly one outstanding rectangle now
    }
    if (capture_frame) {
        capture_ready_.store(true, std::memory_order_release);
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - t_frame);
    raster_us_ = raster_total;
    dma_wait_us_ = dma_wait_total;
    return ESP_OK;
}

}  // namespace fluid_demo
