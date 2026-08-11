#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_types.h"

namespace fluid_demo {

/// Shell-owned panel transport + one-frame PSRAM capture for the 240x240 ST7789.
///
/// Owns the panel/IO handles, the DMA-done binary semaphore, the two 240x16
/// RGB565 DMA stripes, the carried in-flight transfer state, the 240x240
/// RGB565BE PSRAM capture mirror, and the explicit `capture_armed_` frame state
/// machine. The wire order is identical to the pre-split renderer: exactly one
/// `__builtin_bswap16(logical_rgb565)` at pixel store, and the capture mirrors
/// those wire-order bytes verbatim.
///
/// Capture state machine (never a lazy mid-frame latch):
///   request_capture()  CAS-arms capture_requested_ and clears capture_ready_ (any task)
///   latch_capture()    frame boundary: capture_armed_ = capture_requested_.exchange(false)
///                      (render task, at the old renderer.cpp:602 exchange point)
///   submit_stripe()    mirrors into capture_[y0 .. y0+rows) iff capture_armed_
///   finish_frame()     capture_ready_ = true iff capture_armed_, then clears
///   timeout/draw error capture_armed_ -> false AND capture_requested_ re-armed, so a
///                      failed frame is retried whole on the next frame, never
///                      emitted partially.
class DisplayService {
public:
    static constexpr int kWidth = 240;
    static constexpr int kHeight = 240;
    static constexpr int kStripeRows = 16;
    static constexpr int kStripeCount = kHeight / kStripeRows;  // 15 full stripes
    static constexpr int kCaptureWidth = kWidth;
    static constexpr int kCaptureHeight = kHeight;
    static constexpr size_t kCaptureBytes =
        static_cast<size_t>(kCaptureWidth) * kCaptureHeight * sizeof(uint16_t);  // 115200
    static constexpr int kDmaWaitTimeoutMs = 200;

    DisplayService() = default;
    ~DisplayService();
    DisplayService(const DisplayService &) = delete;
    DisplayService &operator=(const DisplayService &) = delete;

    /// Allocate the two DMA stripes + PSRAM capture + DMA-done semaphore and
    /// register the color-transfer-done callback. Idempotent; on failure
    /// everything is released and an error is returned. The callback is
    /// registered exactly once per boot (never re-registered later).
    esp_err_t init(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t io);

    /// Ping-pong stripe buffer for stripe index `s` (render task).
    uint16_t *stripe_buffer(int s);

    /// Retire the carried transfer: block on the DMA-done semaphore (200 ms)
    /// for the outstanding rectangle, if any. `transfer_in_flight_` stays set
    /// on timeout so a later retry waits again; returns ESP_OK or
    /// ESP_ERR_TIMEOUT.
    esp_err_t wait_previous_transfer();

    /// Mirror into the capture buffer iff armed, then wait for the previous
    /// transfer, draw the stripe, and mark one rectangle in flight. On a wait
    /// timeout or draw error the capture latch is dropped and the request
    /// re-armed (reproduces renderer.cpp:602-641 failure paths).
    esp_err_t submit_stripe(int s, int y0, int rows, const uint16_t *pixels);

    /// Frame-boundary capture latch (the old renderer.cpp:602 exchange point).
    void latch_capture();

    /// Close the frame: latches capture_ready_ iff the whole frame was
    /// captured (armed), then clears the armed state. Returns ESP_OK.
    esp_err_t finish_frame();

    /// Arm a one-frame RGB565BE mirror into PSRAM. The render task fulfills it
    /// without adding steady-state framebuffer copies.
    esp_err_t request_capture();
    bool capture_ready() const;
    const uint8_t *capture_data() const;

    /// Barrier primitive: retire the carried final stripe transfer.
    /// Returns ESP_OK or ESP_ERR_TIMEOUT (timeout leaks nothing further).
    esp_err_t drain();

    /// Cumulative transport telemetry.
    uint32_t dma_wait_us() const;
    uint32_t missed_transfers() const;
    uint32_t capture_copy_us() const;

private:
    // ISR (SPI post-transaction) callback: give the DMA-done semaphore only.
    IRAM_ATTR static bool on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                              esp_lcd_panel_io_event_data_t *edata,
                                              void *user_ctx);
    IRAM_ATTR bool trans_done_isr();

    /// Failure path: a failed frame must never emit a partial capture -- drop
    /// the armed latch and re-arm the request so the next frame retries whole.
    void disarm_capture();
    void free_buffers();

    esp_lcd_panel_handle_t panel_ = nullptr;
    esp_lcd_panel_io_handle_t io_ = nullptr;
    bool initialized_ = false;

    /// DMA stripes and the on-demand PSRAM screenshot buffer.
    uint16_t *stripe_[2] = {nullptr, nullptr};
    uint16_t *capture_ = nullptr;

    /// DMA-done semaphore (FreeRTOS binary); only the ISR callback gives it.
    void *sem_ = nullptr;
    bool transfer_in_flight_ = false;

    // Capture state. capture_armed_ is render-task-only frame state; the two
    // atomics cross the console boundary.
    bool capture_armed_ = false;
    std::atomic<bool> capture_requested_{false};
    std::atomic<bool> capture_ready_{false};

    // Cumulative transport telemetry (render task maintains; console may read).
    uint32_t dma_wait_us_ = 0;
    uint32_t capture_copy_us_ = 0;
    uint32_t missed_transfers_ = 0;
};

}  // namespace fluid_demo
