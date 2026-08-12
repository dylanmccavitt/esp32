#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_types.h"

namespace fluid_demo {

/// Shell-owned panel transport and one-frame PSRAM capture buffer.
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

    esp_err_t init(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t io);

    uint16_t *stripe_buffer(int s);

    esp_err_t wait_previous_transfer();

    /// Mirror an armed capture, retire the previous transfer, and draw a stripe.
    esp_err_t submit_stripe(int s, int y0, int rows, const uint16_t *pixels);

    void latch_capture();

    esp_err_t finish_frame();

    esp_err_t request_capture();
    bool capture_ready() const;
    const uint8_t *capture_data() const;

    esp_err_t drain();

    uint32_t dma_wait_us() const;
    uint32_t missed_transfers() const;
    uint32_t capture_copy_us() const;

private:
    // ISR (SPI post-transaction) callback: give the DMA-done semaphore only.
    IRAM_ATTR static bool on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                              esp_lcd_panel_io_event_data_t *edata,
                                              void *user_ctx);
    IRAM_ATTR bool trans_done_isr();

    /// Re-arm a capture after an incomplete frame.
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
