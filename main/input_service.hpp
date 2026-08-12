#pragma once

#include <cstdint>

#include "app_shell.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace fluid_demo {

enum class ButtonEvent : uint8_t {
    PlusPress = 0,
    PwrShort = 1,
};

enum class ButtonId : uint8_t {
    Plus = 0,
    Pwr = 1,
    Boot = 2,
};

using PowerOffMarker = void (*)();
using RebootMarker = void (*)();

/// Debounces physical and synthetic button input on the sensor lane.
class InputService {
public:
    static constexpr uint8_t kBootDebounceSamples = 4;
    static constexpr uint32_t kBootGraceMs = 500;
    static constexpr uint32_t kBootShortMaxMs = 1000;
    static constexpr uint32_t kPowerOffHoldMs = 2000;
    static constexpr uint32_t kMinimumGestureHoldMs = 40;
    static constexpr uint32_t kGestureReleaseMs = 40;
    static constexpr uint32_t kDefaultGestureHoldMs = 120;

    /// Enqueue one synthetic gesture: `button` is logically pressed for
    /// `hold_ms`, then released, and replayed through the physical
    /// debounce/classification pipeline by poll(). Never blocks. ESP_OK when
    /// queued; ESP_ERR_NO_MEM when the bounded FIFO is full.
    esp_err_t enqueue_gesture(ButtonId button, uint32_t hold_ms);

    void set_power_off_marker(PowerOffMarker marker);
    void set_reboot_marker(RebootMarker marker);

    /// Poll buttons in PLUS, PWR, BOOT order and emit at most one event.
    bool poll(uint32_t now_ms, ButtonEvent *out);

    /// Convert controller reports into Begin, Move, and End events.
    bool poll_touch(TouchEvent *out);

    esp_err_t last_touch_error() const { return last_touch_error_; }

private:
    static constexpr uint32_t kGestureFifoCapacity = 16;
    struct Gesture {
        ButtonId button = ButtonId::Plus;
        uint32_t hold_ms = 0;
    };
    Gesture fifo_[kGestureFifoCapacity];
    uint32_t fifo_head_ = 0;
    uint32_t fifo_count_ = 0;
    portMUX_TYPE fifo_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct ActiveGesture {
        bool active = false;
        ButtonId button = ButtonId::Plus;
        uint32_t hold_start_ms = 0;
        uint32_t hold_ms = 0;
    };
    ActiveGesture active_gesture_{};

    PowerOffMarker power_off_marker_ = nullptr;
    RebootMarker reboot_marker_ = nullptr;

    /// Raw level for `button` this tick: the in-flight synthetic gesture's
    /// level while its hold is running, else the physical board level.
    bool synthetic_level(ButtonId button, uint32_t now_ms) const;

    struct BootButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = false;
        uint32_t press_start_ms = 0;
    };

    struct ResetButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = true;
    };

    struct PowerButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = false;
        bool poweroff_sent = false;
        uint32_t press_start_ms = 0;
    };

    void process_boot_button(uint32_t now_ms, bool raw_pressed);
    bool process_reset_button(bool raw_pressed);
    bool process_power_button(uint32_t now_ms, bool raw_pressed);

    BootButton boot_;
    ResetButton reset_button_;
    PowerButton power_button_;

    /// Per-contact touch state: active while the contact is down (gates Begin
    /// to once per contact); the last pressed coordinates feed the End event
    /// because the controller drops coordinates on the finger-up report.
    bool touch_contact_active_ = false;
    // Fail-closed quarantine after an uncertain report: pressed reports are
    // ignored until a zero-finger release proves the contact boundary.
    bool touch_contact_cancelled_ = false;
    uint16_t touch_last_x_ = 0;
    uint16_t touch_last_y_ = 0;
    esp_err_t last_touch_error_ = ESP_OK;
};

}  // namespace fluid_demo
