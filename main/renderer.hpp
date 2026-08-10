#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_types.h"

#include "app_types.hpp"

namespace fluid_demo {

/// Per-frame rendering telemetry. All values are maintained by the render task;
/// `stats()` is only meaningful from the same task.
struct RenderStats {
    uint32_t frame_us;          ///< Total time spent in the last `render()` call.
    uint32_t raster_us;         ///< Time rasterizing stripes (excludes DMA waits).
    uint32_t dma_wait_us;       ///< Time blocked waiting for the previous stripe DMA.
    uint32_t missed_transfers;  ///< Cumulative count of DMA completion timeouts.
};

/// Direct RGB565 striped renderer for the Waveshare 240x240 ST7789 LCD.
///
///  - Two 240x16 RGB565 DMA buffers, a 120x120 scalar-field/depth/heat surface
///    map, and one kMaxParticles projection scratch allocation.
///  - Exactly one outstanding panel rectangle at any time: stripe *s* is
///    rasterized while stripe *s-1* DMA still runs, then we explicitly wait for
///    the previous transfer before submitting stripe *s*. The final stripe's
///    transfer is carried across frames and drained by the next frame's first
///    submit.
///  - `on_color_trans_done` (invoked from the SPI driver ISR) only performs an
///    ISR-safe semaphore give.
///  - Wire order: exactly one `__builtin_bswap16(logical_rgb565)` at pixel store.
///  - No heap allocation, panel calls, sorting, or logging inside the render path.
class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    /// Allocate stripe/surface/scratch buffers, build LUTs, create the DMA-done
    /// semaphore and register the color-transfer-done callback on `io`.
    /// Idempotent; on failure everything is released and an error is returned.
    esp_err_t init(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t io);

    /// Rasterize `frame` into full-height stripes and stream them to the panel.
    /// Blocks only while the previous stripe's DMA completes (explicit wait
    /// before each submit, including the transfer carried from the prior frame).
    /// Must be called from a single task (the render task).
    esp_err_t render(const ParticleFrame &frame);

    /// Telemetry for the most recent completed frame (render task only).
    RenderStats stats() const;
    static constexpr int kCaptureWidth = 240;
    static constexpr int kCaptureHeight = 240;
    static constexpr size_t kCaptureBytes =
        static_cast<size_t>(kCaptureWidth) * kCaptureHeight * sizeof(uint16_t);

    /// Arm a one-frame RGB565BE mirror into PSRAM. The render task fulfills it
    /// without adding steady-state framebuffer copies.
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

    // ISR (SPI post-transaction) callback: give the DMA-done semaphore only.
    IRAM_ATTR static bool on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                              esp_lcd_panel_io_event_data_t *edata,
                                              void *user_ctx);
    IRAM_ATTR bool trans_done_isr();

    /// Wait for the outstanding stripe transfer (if any); returns false on timeout.
    bool wait_previous_transfer(uint32_t *wait_us);
    void build_luts();
    void preproject(const ParticleFrame &frame, int count);
    void project_box_edges();
    void build_surface();
    void draw_box_edges(uint16_t *buf, int y0, int rows);
    void shade_surface(uint16_t *buf, int y0, int rows);
    void free_buffers();

    static uint16_t hsv_to_rgb565(float h, float s, float v);

    esp_lcd_panel_handle_t panel_ = nullptr;
    esp_lcd_panel_io_handle_t io_ = nullptr;
    bool initialized_ = false;

    /// DMA stripes, low-resolution connected-surface maps, projection scratch,
    /// and an on-demand PSRAM screenshot buffer.
    uint16_t *stripe_[2] = {nullptr, nullptr};
    uint16_t *surface_field_ = nullptr;
    uint16_t *surface_heat_ = nullptr;
    uint16_t *surface_depth_ = nullptr;
    uint16_t *capture_ = nullptr;
    Projected *proj_ = nullptr;
    int active_count_ = 0;

    /// DMA-done semaphore (FreRTOS binary); only the ISR callback gives it.
    void *sem_ = nullptr;
    bool transfer_in_flight_ = false;
    std::atomic<bool> capture_requested_{false};
    std::atomic<bool> capture_ready_{false};

    // Render-task-only telemetry for the last frame / cumulative counter.
    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;
    uint32_t dma_wait_us_ = 0;
    uint32_t missed_transfers_ = 0;

    // Look-up tables built at init: sphere-front depth and velocity palette.
    uint16_t nz_lut_[256] = {};
    uint16_t palette_[256] = {};

    Edge edges_[12] = {};
};

}  // namespace fluid_demo
