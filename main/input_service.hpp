#pragma once

#include <cstdint>

#include "app_shell.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace fluid_demo {

enum class ButtonEvent : uint8_t {
    PlusPress = 0,
    PowerShort = 1,
};

enum class ButtonId : uint8_t {
    Plus = 0,
    Power = 1,
    Boot = 2,
};

using ActionMarker = void (*)();

class InputService {
public:
    static constexpr uint32_t kMinimumGestureHoldMs = 40;
    static constexpr uint32_t kDefaultGestureHoldMs = 120;

    esp_err_t enqueue_button_gesture(ButtonId button, uint32_t hold_ms);
    esp_err_t enqueue_swipe(TouchGesture gesture);
    void set_power_off_marker(ActionMarker marker);
    void set_reboot_marker(ActionMarker marker);
    bool poll(uint32_t now_ms, ButtonEvent &event);
    bool poll_touch(TouchEvent &event);
    esp_err_t last_touch_error() const { return last_touch_error_; }

private:
    static constexpr uint8_t kButtonDebounceSamples = 4;
    static constexpr uint32_t kBootGraceMs = 500;
    static constexpr uint32_t kBootShortMaxMs = 1000;
    static constexpr uint32_t kPowerOffHoldMs = 2000;
    static constexpr uint32_t kGestureReleaseMs = 40;

    static constexpr uint32_t kButtonGestureQueueCapacity = 16;
    struct ButtonGesture {
        ButtonId button = ButtonId::Plus;
        uint32_t hold_ms = 0;
    };
    ButtonGesture button_gesture_queue_[kButtonGestureQueueCapacity];
    uint32_t button_gesture_queue_head_ = 0;
    uint32_t button_gesture_queue_count_ = 0;
    portMUX_TYPE synthetic_input_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct ActiveButtonGesture {
        bool active = false;
        ButtonId button = ButtonId::Plus;
        uint32_t hold_start_ms = 0;
        uint32_t hold_ms = 0;
    };
    ActiveButtonGesture active_button_gesture_{};

    ActionMarker power_off_marker_ = nullptr;
    ActionMarker reboot_marker_ = nullptr;

    bool button_level(ButtonId button, uint32_t now_ms) const;

    struct BootButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = false;
        uint32_t press_start_ms = 0;
    };

    struct PlusButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = true;
    };

    struct PowerButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = false;
        bool power_off_sent = false;
        uint32_t press_start_ms = 0;
    };

    void process_boot_button(uint32_t now_ms, bool raw_pressed);
    bool process_plus_button(bool raw_pressed);
    bool process_power_button(uint32_t now_ms, bool raw_pressed);

    BootButton boot_button_;
    PlusButton plus_button_;
    PowerButton power_button_;

    bool touch_contact_active_ = false;
    bool touch_quarantined_until_release_ = false;
    uint16_t last_touch_x_ = 0;
    uint16_t last_touch_y_ = 0;
    esp_err_t last_touch_error_ = ESP_OK;

    bool swipe_pending_ = false;
    bool swipe_started_ = false;
    TouchGesture swipe_gesture_ = TouchGesture::None;
    uint16_t swipe_start_x_ = 0;
    uint16_t swipe_start_y_ = 0;
    uint16_t swipe_end_x_ = 0;
    uint16_t swipe_end_y_ = 0;
};

}
