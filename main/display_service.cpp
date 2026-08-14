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

constexpr char kTag[] = "display_service";

}

DisplayService::~DisplayService()
{
    free_buffers();
}

esp_err_t DisplayService::init(esp_lcd_panel_handle_t panel,
                               esp_lcd_panel_io_handle_t io)
{
    if (panel == nullptr || io == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (initialized_) {
        return ESP_OK;
    }

    const size_t stripe_bytes =
        static_cast<size_t>(kWidth) * kStripeRows * sizeof(uint16_t);
    uint16_t *first_stripe_buffer =
        static_cast<uint16_t *>(heap_caps_aligned_alloc(
            16, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    uint16_t *second_stripe_buffer =
        static_cast<uint16_t *>(heap_caps_aligned_alloc(
            16, stripe_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    uint16_t *capture_buffer = static_cast<uint16_t *>(heap_caps_aligned_alloc(
        16, kCaptureBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    SemaphoreHandle_t transfer_done_semaphore = xSemaphoreCreateBinary();

    if (first_stripe_buffer == nullptr || second_stripe_buffer == nullptr ||
        capture_buffer == nullptr || transfer_done_semaphore == nullptr) {
        heap_caps_free(first_stripe_buffer);
        heap_caps_free(second_stripe_buffer);
        heap_caps_free(capture_buffer);
        if (transfer_done_semaphore != nullptr) {
            vSemaphoreDelete(transfer_done_semaphore);
        }
        ESP_LOGE(kTag, "display transport allocation failed");
        return ESP_ERR_NO_MEM;
    }

    stripe_buffers_[0] = first_stripe_buffer;
    stripe_buffers_[1] = second_stripe_buffer;
    capture_buffer_ = capture_buffer;
    transfer_done_semaphore_ = transfer_done_semaphore;

    esp_lcd_panel_io_callbacks_t callbacks{};
    callbacks.on_color_trans_done = &DisplayService::on_color_transfer_done;
    const esp_err_t callback_result =
        esp_lcd_panel_io_register_event_callbacks(io, &callbacks, this);
    if (callback_result != ESP_OK) {
        ESP_LOGE(kTag, "register on_color_trans_done failed: %s",
                 esp_err_to_name(callback_result));
        free_buffers();
        return callback_result;
    }

    panel_ = panel;
    initialized_ = true;
    ESP_LOGI(kTag,
             "init: %dx%d, %d stripes x %d rows, "
             "%u B PSRAM capture",
             kWidth, kHeight, kStripeCount, kStripeRows,
             static_cast<unsigned>(kCaptureBytes));
    return ESP_OK;
}

void DisplayService::free_buffers()
{
    if (transfer_done_semaphore_ != nullptr) {
        vSemaphoreDelete(
            static_cast<SemaphoreHandle_t>(transfer_done_semaphore_));
        transfer_done_semaphore_ = nullptr;
    }
    heap_caps_free(stripe_buffers_[0]);
    heap_caps_free(stripe_buffers_[1]);
    heap_caps_free(capture_buffer_);
    stripe_buffers_[0] = nullptr;
    stripe_buffers_[1] = nullptr;
    capture_buffer_ = nullptr;
    transfer_in_flight_ = false;
    initialized_ = false;
}

uint16_t *DisplayService::stripe_buffer(int stripe_index)
{
    return stripe_buffers_[stripe_index & 1];
}

esp_err_t DisplayService::wait_previous_transfer()
{
    if (!transfer_in_flight_) {
        return ESP_OK;
    }

    const SemaphoreHandle_t transfer_done_semaphore =
        static_cast<SemaphoreHandle_t>(transfer_done_semaphore_);
    const int64_t wait_start_us = esp_timer_get_time();
    const bool transfer_completed =
        xSemaphoreTake(transfer_done_semaphore,
                       pdMS_TO_TICKS(kDmaWaitTimeoutMs)) == pdTRUE;
    dma_wait_us_ += static_cast<uint32_t>(esp_timer_get_time() - wait_start_us);
    if (!transfer_completed) {
        ++missed_transfers_;
        // Keep the transfer pending so a later retry waits.
        return ESP_ERR_TIMEOUT;
    }
    transfer_in_flight_ = false;
    return ESP_OK;
}

void DisplayService::requeue_capture()
{
    capture_armed_ = false;
    capture_requested_.store(true, std::memory_order_release);
}

esp_err_t DisplayService::submit_stripe(int stripe_index, int stripe_y,
                                        int stripe_rows, const uint16_t *pixels)
{
    // Copy while the previous stripe's DMA transfer is running.
    if (capture_armed_) {
        const int64_t copy_start_us = esp_timer_get_time();
        std::memcpy(
            capture_buffer_ + static_cast<size_t>(stripe_y) * kWidth, pixels,
            static_cast<size_t>(kWidth) * stripe_rows * sizeof(uint16_t));
        capture_copy_us_ +=
            static_cast<uint32_t>(esp_timer_get_time() - copy_start_us);
    }

    esp_err_t transfer_result = wait_previous_transfer();
    if (transfer_result != ESP_OK) {
        if (capture_armed_) {
            requeue_capture();
        }
        return transfer_result;
    }

    transfer_result = esp_lcd_panel_draw_bitmap(panel_, 0, stripe_y, kWidth,
                                                stripe_y + stripe_rows, pixels);
    if (transfer_result != ESP_OK) {
        if (capture_armed_) {
            requeue_capture();
        }
        ESP_LOGE(kTag, "draw_bitmap stripe %d failed: %s", stripe_index,
                 esp_err_to_name(transfer_result));
        return transfer_result;
    }
    transfer_in_flight_ = true;
    return ESP_OK;
}

void DisplayService::latch_capture()
{
    capture_copy_us_ = 0;
    capture_armed_ =
        capture_requested_.exchange(false, std::memory_order_acq_rel);
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
    if (!initialized_ || capture_buffer_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (capture_requested_.load(std::memory_order_acquire)) {
        return ESP_ERR_INVALID_STATE;
    }
    capture_ready_.store(false, std::memory_order_release);
    bool expected = false;
    if (!capture_requested_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
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
    return reinterpret_cast<const uint8_t *>(capture_buffer_);
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

bool DisplayService::on_color_transfer_done(esp_lcd_panel_io_handle_t,
                                            esp_lcd_panel_io_event_data_t *,
                                            void *user_context)
{
    auto *display = static_cast<DisplayService *>(user_context);
    return display->color_transfer_done_isr();
}

bool DisplayService::color_transfer_done_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (transfer_done_semaphore_ != nullptr) {
        xSemaphoreGiveFromISR(
            static_cast<SemaphoreHandle_t>(transfer_done_semaphore_),
            &higher_priority_task_woken);
    }
    return higher_priority_task_woken == pdTRUE;
}

}
