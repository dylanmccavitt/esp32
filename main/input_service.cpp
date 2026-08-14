#include "input_service.hpp"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

#include "board.hpp"

namespace fluid_demo {

namespace {
constexpr char kTag[] = "input_service";
}

esp_err_t InputService::enqueue_button_gesture(const ButtonId button,
                                               const uint32_t hold_ms)
{
    portENTER_CRITICAL(&synthetic_input_mux_);
    const bool queue_full =
        button_gesture_queue_count_ >= kButtonGestureQueueCapacity;
    if (!queue_full) {
        button_gesture_queue_[(button_gesture_queue_head_ +
                               button_gesture_queue_count_) %
                              kButtonGestureQueueCapacity] = {button, hold_ms};
        ++button_gesture_queue_count_;
    }
    portEXIT_CRITICAL(&synthetic_input_mux_);
    return queue_full ? ESP_ERR_NO_MEM : ESP_OK;
}

esp_err_t InputService::enqueue_swipe(const TouchGesture gesture)
{
    if (gesture != TouchGesture::SwipeLeft &&
        gesture != TouchGesture::SwipeRight) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&synthetic_input_mux_);
    const bool swipe_already_pending = swipe_pending_;
    if (!swipe_already_pending) {
        swipe_pending_ = true;
        swipe_started_ = false;
        swipe_gesture_ = gesture;
        swipe_start_y_ = 120;
        swipe_end_y_ = 120;
        if (gesture == TouchGesture::SwipeLeft) {
            swipe_start_x_ = 200;
            swipe_end_x_ = 40;
        } else {
            swipe_start_x_ = 40;
            swipe_end_x_ = 200;
        }
    }
    portEXIT_CRITICAL(&synthetic_input_mux_);
    return swipe_already_pending ? ESP_ERR_NO_MEM : ESP_OK;
}

bool InputService::poll_touch(TouchEvent &event)
{
    bool has_synthetic_swipe_event = false;
    TouchEvent synthetic_swipe_event{};
    portENTER_CRITICAL(&synthetic_input_mux_);
    if (swipe_pending_ && !touch_contact_active_ &&
        !touch_quarantined_until_release_) {
        has_synthetic_swipe_event = true;
        if (!swipe_started_) {
            synthetic_swipe_event.x = swipe_start_x_;
            synthetic_swipe_event.y = swipe_start_y_;
            synthetic_swipe_event.phase = TouchPhase::Begin;
            synthetic_swipe_event.gesture = TouchGesture::None;
            swipe_started_ = true;
        } else {
            synthetic_swipe_event.x = swipe_end_x_;
            synthetic_swipe_event.y = swipe_end_y_;
            synthetic_swipe_event.phase = TouchPhase::End;
            synthetic_swipe_event.gesture = swipe_gesture_;
            swipe_pending_ = false;
            swipe_started_ = false;
        }
    }
    portEXIT_CRITICAL(&synthetic_input_mux_);
    if (has_synthetic_swipe_event) {
        event = synthetic_swipe_event;
        return true;
    }

    TouchSample touch_sample{};
    last_touch_error_ = fluid_demo::board_read_touch(touch_sample);
    if (last_touch_error_ != ESP_OK) {
        touch_contact_active_ = false;
        touch_quarantined_until_release_ = true;
        return false;
    }
    if (!touch_sample.fresh) {
        return false;
    }
    if (touch_quarantined_until_release_) {
        if (!touch_sample.pressed) {
            touch_quarantined_until_release_ = false;
        }
        return false;
    }
    if (!touch_sample.pressed) {
        // Release reports omit coordinates.
        const bool contact_was_active = touch_contact_active_;
        touch_contact_active_ = false;
        if (contact_was_active) {
            event.x = last_touch_x_;
            event.y = last_touch_y_;
            event.phase = TouchPhase::End;
            event.gesture = touch_sample.gesture;
        }
        return contact_was_active;
    }

    last_touch_x_ = touch_sample.x;
    last_touch_y_ = touch_sample.y;
    if (touch_contact_active_) {
        event.x = touch_sample.x;
        event.y = touch_sample.y;
        event.phase = TouchPhase::Move;
        event.gesture = TouchGesture::None;
        return true;
    }

    touch_contact_active_ = true;
    event.x = touch_sample.x;
    event.y = touch_sample.y;
    event.phase = TouchPhase::Begin;
    event.gesture = TouchGesture::None;
    return true;
}

void InputService::set_power_off_marker(const ActionMarker marker)
{
    power_off_marker_ = marker;
}

void InputService::set_reboot_marker(const ActionMarker marker)
{
    reboot_marker_ = marker;
}

bool InputService::button_level(const ButtonId button,
                                const uint32_t now_ms) const
{
    if (active_button_gesture_.active &&
        active_button_gesture_.button == button) {
        return now_ms - active_button_gesture_.hold_start_ms <
               active_button_gesture_.hold_ms;
    }
    switch (button) {
    case ButtonId::Plus:
        return fluid_demo::board_reset_pressed();
    case ButtonId::Power:
        return fluid_demo::board_power_pressed();
    case ButtonId::Boot:
        return fluid_demo::board_boot_pressed();
    default:
        return false;
    }
}

bool InputService::poll(uint32_t now_ms, ButtonEvent &event)
{
    if (active_button_gesture_.active) {
        const uint32_t elapsed_ms =
            now_ms - active_button_gesture_.hold_start_ms;
        if (elapsed_ms >= active_button_gesture_.hold_ms + kGestureReleaseMs) {
            active_button_gesture_.active = false;
        }
    }
    if (!active_button_gesture_.active) {
        portENTER_CRITICAL(&synthetic_input_mux_);
        if (button_gesture_queue_count_ != 0) {
            const ButtonGesture button_gesture =
                button_gesture_queue_[button_gesture_queue_head_];
            button_gesture_queue_head_ =
                (button_gesture_queue_head_ + 1) % kButtonGestureQueueCapacity;
            --button_gesture_queue_count_;
            active_button_gesture_.active = true;
            active_button_gesture_.button = button_gesture.button;
            active_button_gesture_.hold_start_ms = now_ms;
            active_button_gesture_.hold_ms = button_gesture.hold_ms;
        }
        portEXIT_CRITICAL(&synthetic_input_mux_);
    }

    bool button_event_emitted = false;
    if (process_plus_button(button_level(ButtonId::Plus, now_ms))) {
        event = ButtonEvent::PlusPress;
        button_event_emitted = true;
    } else if (process_power_button(now_ms,
                                    button_level(ButtonId::Power, now_ms))) {
        event = ButtonEvent::PowerShort;
        button_event_emitted = true;
    }

    process_boot_button(now_ms, button_level(ButtonId::Boot, now_ms));
    return button_event_emitted;
}

bool InputService::process_plus_button(const bool raw_pressed)
{
    if (raw_pressed != plus_button_.pressed_debounced) {
        if (++plus_button_.stable_count >= kButtonDebounceSamples) {
            plus_button_.stable_count = 0;
            plus_button_.pressed_debounced = raw_pressed;
            if (raw_pressed && plus_button_.armed) {
                plus_button_.armed = false;
                return true;
            }
            if (!raw_pressed) {
                plus_button_.armed = true;
            }
        }
    } else {
        plus_button_.stable_count = 0;
    }
    return false;
}

bool InputService::process_power_button(const uint32_t now_ms,
                                        const bool raw_pressed)
{
    bool short_press_fired = false;
    if (raw_pressed != power_button_.pressed_debounced) {
        if (++power_button_.stable_count >= kButtonDebounceSamples) {
            power_button_.stable_count = 0;
            power_button_.pressed_debounced = raw_pressed;
            if (raw_pressed) {
                if (power_button_.armed) {
                    power_button_.press_start_ms = now_ms;
                }
            } else {
                const bool long_hold_handled = power_button_.power_off_sent;
                const uint32_t hold_duration_ms =
                    power_button_.press_start_ms != 0
                        ? now_ms - power_button_.press_start_ms
                        : 0;
                power_button_.armed = true;
                power_button_.power_off_sent = false;
                power_button_.press_start_ms = 0;
                if (!long_hold_handled && hold_duration_ms != 0 &&
                    hold_duration_ms < kPowerOffHoldMs) {
                    short_press_fired = true;
                }
            }
        }
    } else {
        power_button_.stable_count = 0;
        if (!raw_pressed && !power_button_.armed) {
            power_button_.armed = true;
        }
    }

    if (power_button_.pressed_debounced && power_button_.armed &&
        !power_button_.power_off_sent && power_button_.press_start_ms != 0 &&
        now_ms - power_button_.press_start_ms >= kPowerOffHoldMs) {
        power_button_.power_off_sent = true;
        if (power_off_marker_ != nullptr) {
            power_off_marker_();
        }
        const esp_err_t power_off_result = fluid_demo::board_power_off();
        if (power_off_result != ESP_OK) {
            ESP_LOGE(kTag, "board_power_off failed: %s",
                     esp_err_to_name(power_off_result));
        }
    }
    return short_press_fired;
}

void InputService::process_boot_button(const uint32_t now_ms,
                                       const bool raw_pressed)
{
    const bool level_changed = raw_pressed != boot_button_.pressed_debounced;
    if (level_changed) {
        if (++boot_button_.stable_count >= kButtonDebounceSamples) {
            boot_button_.stable_count = 0;
            boot_button_.pressed_debounced = raw_pressed;
            if (raw_pressed) {
                if (boot_button_.armed && now_ms >= kBootGraceMs) {
                    boot_button_.press_start_ms = now_ms;
                }
            } else {
                if (boot_button_.press_start_ms != 0) {
                    const uint32_t hold_duration_ms =
                        now_ms - boot_button_.press_start_ms;
                    if (hold_duration_ms <= kBootShortMaxMs) {
                        ESP_LOGI(kTag, "BOOT short press (%u ms) - rebooting",
                                 hold_duration_ms);
                        if (reboot_marker_ != nullptr) {
                            reboot_marker_();
                        }
                        vTaskDelay(pdMS_TO_TICKS(50));
                        esp_restart();
                    } else {
                        ESP_LOGI(kTag, "BOOT held %u ms - long press ignored",
                                 hold_duration_ms);
                    }
                    boot_button_.press_start_ms = 0;
                }
            }
            boot_button_.armed = true;
        }
    } else {
        boot_button_.stable_count = 0;
        if (!raw_pressed && !boot_button_.armed) {
            boot_button_.armed = true;
        }
    }
}

}
