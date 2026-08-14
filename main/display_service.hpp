#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_types.h"

namespace fluid_demo {

class DisplayService {
public:
    static constexpr int kWidth = 240;
    static constexpr int kHeight = 240;
    static constexpr int kStripeRows = 16;
    static constexpr int kStripeCount = kHeight / kStripeRows;
    static constexpr int kCaptureWidth = kWidth;
    static constexpr int kCaptureHeight = kHeight;
    static constexpr size_t kCaptureBytes =
        static_cast<size_t>(kCaptureWidth) * kCaptureHeight * sizeof(uint16_t);

    DisplayService() = default;
    ~DisplayService();
    DisplayService(const DisplayService &) = delete;
    DisplayService &operator=(const DisplayService &) = delete;

    esp_err_t init(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t io);
    uint16_t *stripe_buffer(int stripe_index);
    esp_err_t wait_previous_transfer();
    esp_err_t submit_stripe(int stripe_index, int stripe_y, int stripe_rows,
                            const uint16_t *pixels);
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
    static constexpr int kDmaWaitTimeoutMs = 200;

    IRAM_ATTR static bool
    on_color_transfer_done(esp_lcd_panel_io_handle_t,
                           esp_lcd_panel_io_event_data_t *, void *user_context);
    IRAM_ATTR bool color_transfer_done_isr();

    void requeue_capture();
    void free_buffers();

    esp_lcd_panel_handle_t panel_ = nullptr;
    bool initialized_ = false;
    uint16_t *stripe_buffers_[2] = {nullptr, nullptr};
    uint16_t *capture_buffer_ = nullptr;
    void *transfer_done_semaphore_ = nullptr;
    bool transfer_in_flight_ = false;

    // Render owns capture_armed_; the atomics cross the console boundary.
    bool capture_armed_ = false;
    std::atomic<bool> capture_requested_{false};
    std::atomic<bool> capture_ready_{false};

    uint32_t dma_wait_us_ = 0;
    uint32_t capture_copy_us_ = 0;
    uint32_t missed_transfers_ = 0;
};

}
