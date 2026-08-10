#include "display_service.hpp"

#include <cstring>

#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace fluid_demo {

namespace {

constexpr const char *kTAG = "display_service";

}  // namespace

DisplayService::~DisplayService()
{
    free_buffers();
}

esp_err_t DisplayService::init(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t io)
{
    if (panel == nullptr || io == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (initialized_) {
        return ESP_OK;  // idempotent: the callback is registered once per boot
    }

    const size_t stripe_bytes = (size_t)kWidth * kStripeRows * sizeof(uint16_t);
    const size_t capture_bytes = kCaptureBytes;
    uint16_t *a = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    uint16_t *b = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    uint16_t *capture = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(16, capture_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    SemaphoreHandle_t sem = xSemaphoreCreateBinary();

    if (a == nullptr || b == nullptr || capture == nullptr || sem == nullptr) {
        heap_caps_free(a);
        heap_caps_free(b);
        heap_caps_free(capture);
        if (sem != nullptr) {
            vSemaphoreDelete(sem);
        }
        ESP_LOGE(kTAG, "display transport allocation failed");
        return ESP_ERR_NO_MEM;
    }

    stripe_[0] = a;
    stripe_[1] = b;
    capture_ = capture;
    sem_ = sem;

    esp_lcd_panel_io_callbacks_t cbs = {};
    cbs.on_color_trans_done = &DisplayService::on_color_trans_done;
    esp_err_t ret = esp_lcd_panel_io_register_event_callbacks(io, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(kTAG, "register on_color_trans_done failed: %s", esp_err_to_name(ret));
        free_buffers();
        return ret;
    }

    panel_ = panel;
    io_ = io;
    initialized_ = true;
    ESP_LOGI(kTAG, "init: %dx%d, %d stripes x %d rows, %u B PSRAM capture, "
                   "on_color_trans_done registered",
             kWidth, kHeight, kStripeCount, kStripeRows,
             static_cast<unsigned>(kCaptureBytes));
    return ESP_OK;
}

void DisplayService::free_buffers()
{
    if (sem_ != nullptr) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(sem_));
        sem_ = nullptr;
    }
    heap_caps_free(stripe_[0]);
    heap_caps_free(stripe_[1]);
    heap_caps_free(capture_);
    stripe_[0] = stripe_[1] = nullptr;
    capture_ = nullptr;
    transfer_in_flight_ = false;
    initialized_ = false;
}

uint16_t *DisplayService::stripe_buffer(int s)
{
    return stripe_[s & 1];
}

esp_err_t DisplayService::wait_previous_transfer()
{
    if (!transfer_in_flight_) {
        return ESP_OK;
    }
    SemaphoreHandle_t sem = static_cast<SemaphoreHandle_t>(sem_);
    const int64_t t0 = esp_timer_get_time();
    const bool ok = xSemaphoreTake(sem, pdMS_TO_TICKS(kDmaWaitTimeoutMs)) == pdTRUE;
    dma_wait_us_ += static_cast<uint32_t>(esp_timer_get_time() - t0);
    if (!ok) {
        missed_transfers_++;
        return ESP_ERR_TIMEOUT;  // in_flight stays set: a later retry waits again
    }
    transfer_in_flight_ = false;
    return ESP_OK;
}

void DisplayService::disarm_capture()
{
    // A failed frame never emits a partial capture: drop the armed latch and
    // re-arm the request so the next frame retries in full.
    capture_armed_ = false;
    capture_requested_.store(true, std::memory_order_release);
}

esp_err_t DisplayService::submit_stripe(int s, int y0, int rows, const uint16_t *pixels)
{
    // Mirror the wire-order stripe while the previous stripe's DMA still runs,
    // matching the original capture-mirror -> wait -> draw_bitmap ordering.
    if (capture_armed_) {
        const int64_t copy_start_us = esp_timer_get_time();
        std::memcpy(capture_ + static_cast<size_t>(y0) * kWidth, pixels,
                    static_cast<size_t>(kWidth) * rows * sizeof(uint16_t));
        capture_copy_us_ +=
            static_cast<uint32_t>(esp_timer_get_time() - copy_start_us);
    }

    esp_err_t ret = wait_previous_transfer();
    if (ret != ESP_OK) {
        if (capture_armed_) {
            disarm_capture();
        }
        return ret;
    }

    ret = esp_lcd_panel_draw_bitmap(panel_, 0, y0, kWidth, y0 + rows, pixels);
    if (ret != ESP_OK) {
        if (capture_armed_) {
            disarm_capture();
        }
        ESP_LOGE(kTAG, "draw_bitmap stripe %d failed: %s", s, esp_err_to_name(ret));
        return ret;
    }
    transfer_in_flight_ = true;  // exactly one outstanding rectangle now
    return ESP_OK;
}

void DisplayService::latch_capture()
{
    capture_copy_us_ = 0;
    capture_armed_ = capture_requested_.exchange(false, std::memory_order_acq_rel);
}

esp_err_t DisplayService::finish_frame()
{
    if (capture_armed_) {
        capture_ready_.store(true, std::memory_order_release);
    }
    capture_armed_ = false;
    return ESP_OK;
}

esp_err_t DisplayService::request_capture()
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

bool DisplayService::capture_ready() const
{
    return capture_ready_.load(std::memory_order_acquire);
}

const uint8_t *DisplayService::capture_data() const
{
    return reinterpret_cast<const uint8_t *>(capture_);
}

esp_err_t DisplayService::drain()
{
    return wait_previous_transfer();
}

uint32_t DisplayService::dma_wait_us() const
{
    return dma_wait_us_;
}

uint32_t DisplayService::capture_copy_us() const
{
    return capture_copy_us_;
}

uint32_t DisplayService::missed_transfers() const
{
    return missed_transfers_;
}

bool DisplayService::on_color_trans_done(esp_lcd_panel_io_handle_t io,
                                         esp_lcd_panel_io_event_data_t *edata,
                                         void *user_ctx)
{
    (void)io;
    (void)edata;
    DisplayService *self = static_cast<DisplayService *>(user_ctx);
    return self->trans_done_isr();
}

bool DisplayService::trans_done_isr()
{
    BaseType_t hpw = pdFALSE;
    if (sem_ != nullptr) {
        xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(sem_), &hpw);
    }
    return hpw == pdTRUE;
}

}  // namespace fluid_demo
