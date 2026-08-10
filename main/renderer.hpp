#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

#include "app_types.hpp"
#include "display_service.hpp"

namespace fluid_demo {

/// Per-frame rendering telemetry. All values are maintained by the render task;
/// `stats()` is only meaningful from the same task.
struct RenderStats {
    uint32_t frame_us;          ///< Total time spent in the last `render()` call.
    uint32_t raster_us;         ///< Time rasterizing stripes (excludes DMA waits).
    uint32_t dma_wait_us;       ///< Time blocked waiting for the previous stripe DMA.
    uint32_t missed_transfers;  ///< Cumulative count of DMA completion timeouts.
};

/// Raster half of the 240x240 ST7789 striped renderer.
///
/// The panel transport (DMA stripes, DMA-done semaphore, in-flight transfer,
/// PSRAM capture) is owned by the shell's DisplayService; Renderer binds it
/// once at init() and drives it through stripe_buffer()/submit_stripe()/
/// finish_frame() and the explicit capture latch.
///
///  - One 120x120 scalar-field/depth/heat surface map set and one kMaxParticles
///    projection scratch allocation.
///  - Exactly one outstanding panel rectangle at any time: stripe *s* is
///    rasterized while stripe *s-1* DMA still runs, then submit_stripe()
///    explicitly waits for the previous transfer before drawing stripe *s*.
///    The final stripe's transfer is carried across frames and drained by the
///    next frame's first wait.
///  - Wire order: exactly one `__builtin_bswap16(logical_rgb565)` at pixel store.
///  - No heap allocation, panel calls, sorting, or logging inside the render path.
class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    /// Allocate surface/scratch buffers, build LUTs, and bind the shell's
    /// DisplayService for transport. Idempotent; on failure everything is
    /// released and an error is returned.
    esp_err_t init(DisplayService *display);

    /// Rasterize `frame` into full-height stripes and stream them to the panel
    /// through the bound DisplayService. Blocks only while the previous
    /// stripe's DMA completes (explicit wait before each submit, including the
    /// transfer carried from the prior frame). Must be called from a single
    /// task (the render task).
    esp_err_t render(const ParticleFrame &frame);

    /// Telemetry for the most recent completed frame (render task only).
    RenderStats stats() const;

    // Capture facade: the shell's DisplayService owns the one-frame RGB565BE
    // PSRAM mirror. These forward unchanged so the console API stays the same.
    static constexpr int kCaptureWidth = DisplayService::kCaptureWidth;
    static constexpr int kCaptureHeight = DisplayService::kCaptureHeight;
    static constexpr size_t kCaptureBytes = DisplayService::kCaptureBytes;

    /// Arm a one-frame RGB565BE mirror via the bound DisplayService.
    esp_err_t request_capture();
    bool capture_ready() const;
    const uint8_t *capture_data() const;

private:
    struct Projected {
        int16_t x;          ///< Full-resolution screen center x (px).
        int16_t y;          ///< Full-resolution screen center y (px).
        uint16_t radius;    ///< Full-resolution particle radius (px), >= 1.
        uint8_t speed_idx;  ///< Palette index for the velocity color.
        uint8_t active;     ///< 0 = particle skipped (non-finite/out of range).
        uint32_t z_fx;      ///< Center depth, fixed point, 1/4096 world unit.
        uint32_t rw_fx;     ///< World radius, fixed point, 1/4096 world unit.
    };

    struct Edge {
        int x0;
        int y0;
        int x1;
        int y1;
    };

    void build_luts();
    void preproject(const ParticleFrame &frame, int count);
    void project_box_edges();
    void build_surface();
    void draw_box_edges(uint16_t *buf, int y0, int rows);
    void shade_surface(uint16_t *buf, int y0, int rows);
    void free_buffers();

    static uint16_t hsv_to_rgb565(float h, float s, float v);

    DisplayService *display_ = nullptr;
    bool initialized_ = false;

    /// Low-resolution connected-surface maps and projection scratch.
    uint16_t *surface_field_ = nullptr;
    uint16_t *surface_heat_ = nullptr;
    uint16_t *surface_depth_ = nullptr;
    Projected *proj_ = nullptr;
    int active_count_ = 0;

    // Render-task-only telemetry for the last frame.
    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;
    uint32_t dma_wait_us_ = 0;

    // Look-up tables built at init: sphere-front depth and velocity palette.
    uint16_t nz_lut_[256] = {};
    uint16_t palette_[256] = {};

    Edge edges_[12] = {};
};

}  // namespace fluid_demo
