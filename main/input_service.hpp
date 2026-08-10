#pragma once

#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace fluid_demo {

/// Events produced by the shell's button pipeline (sensor lane). The shell
/// routes PlusPress to the active app (Running) or a Launch request (launcher)
/// and PwrShort to a Home request (Running) or a no-op (launcher).
enum class ButtonEvent : uint8_t {
    PlusPress = 0,  ///< Validated PLUS press edge (app reset / launch).
    PwrShort = 1,   ///< Validated PWR release < kPowerOffHoldMs, never after a long hold.
};

/// Synthetic button id for the dev-console `input <plus|pwr|boot>` command.
enum class ButtonId : uint8_t {
    Plus = 0,
    Pwr = 1,
    Boot = 2,
};

/// Marker invoked immediately before board_power_off(), from the sensor lane,
/// so the @DEV POWEROFF wire line is emitted and flushed first. Bound exactly
/// once at startup (never re-bound).
using PowerOffMarker = void (*)();
/// Marker invoked immediately before a BOOT short-press restart so the
/// @DEV REBOOTING wire line is emitted and flushed first. Bound exactly once
/// at startup (never re-bound).
using RebootMarker = void (*)();

/// Owns the BOOT/PLUS/PWR debounce + classify pipeline moved verbatim from
/// app_main.cpp:122-270: BOOT debounce/short-reboot (grace + short window),
/// PLUS validated press edge, PWR validated release (< 2000 ms) and the
/// PWR >= 2 s board_power_off hold, including held-through-boot disarm and
/// poweroff_sent suppression. board.cpp keeps all GPIO init/levels and the
/// power-off sequence unchanged; this service calls board_power_off() inside
/// poll() from the sensor task, exactly the legacy task context.
///
/// A bounded, thread-safe synthetic gesture FIFO (console REPL -> sensor lane)
/// lets `input` drive the SAME four-sample debounce/classification pipeline as
/// the physical buttons: enqueue_gesture() stores {button, hold_ms}, and
/// poll() presents the gesture's raw level (pressed for hold_ms, then
/// released) exactly where the physical board level would be read. Short/long
/// classification, the 2 s power-off hold, no-short-PWR-after-a-long-hold and
/// the BOOT short reboot therefore behave identically for `input` and for a
/// physical press.
///
/// Sensor-lane-owned: poll() must run at the 100 Hz cadence.
class InputService {
public:
    /// Debounce and classification constants (legacy app_main.cpp:76-80).
    static constexpr uint8_t kBootDebounceSamples = 4;  // 40 ms at 100 Hz
    static constexpr uint32_t kBootGraceMs = 500;       // ignore BOOT right after boot
    static constexpr uint32_t kBootShortMaxMs = 1000;   // longer hold => not a reboot
    static constexpr uint32_t kPowerOffHoldMs = 2000;   // matches factory long-press default
    /// Synthetic press/release phases each span at least four sensor polls, so
    /// every queued gesture reaches both validated debounce edges.
    static constexpr uint32_t kMinimumGestureHoldMs = 40;
    static constexpr uint32_t kGestureReleaseMs = 40;
    static constexpr uint32_t kDefaultGestureHoldMs = 120;

    /// Enqueue one synthetic gesture: `button` is logically pressed for
    /// `hold_ms`, then released, and replayed through the physical
    /// debounce/classification pipeline by poll(). Never blocks. ESP_OK when
    /// queued; ESP_ERR_NO_MEM when the bounded FIFO is full.
    esp_err_t enqueue_gesture(ButtonId button, uint32_t hold_ms);

    /// Bind the power-off marker exactly once at startup (never re-bound).
    void set_power_off_marker(PowerOffMarker marker);
    /// Bind the reboot marker exactly once at startup (never re-bound).
    void set_reboot_marker(RebootMarker marker);

    /// Debounce all three buttons and classify this tick. Returns true and
    /// writes `out` when one event fired. BOOT short -> esp_restart() and
    /// PWR hold -> board_power_off() happen inside, never returning on
    /// success. Processing order is the legacy order (PLUS, PWR, BOOT) and
    /// the first event in that order wins the single out slot.
    bool poll(uint32_t now_ms, ButtonEvent *out);

private:
    // ---- synthetic gesture FIFO (bounded, thread-safe; no allocation) ----
    static constexpr uint32_t kGestureFifoCapacity = 16;
    struct Gesture {
        ButtonId button = ButtonId::Plus;
        uint32_t hold_ms = 0;
    };
    Gesture fifo_[kGestureFifoCapacity];
    uint32_t fifo_head_ = 0;    // next slot to dequeue
    uint32_t fifo_count_ = 0;   // entries currently queued
    portMUX_TYPE fifo_mux_ = portMUX_INITIALIZER_UNLOCKED;

    struct ActiveGesture {
        bool active = false;
        ButtonId button = ButtonId::Plus;
        uint32_t hold_start_ms = 0;  // press start; release phase follows hold_ms
        uint32_t hold_ms = 0;
    };
    ActiveGesture active_gesture_{};

    PowerOffMarker power_off_marker_ = nullptr;
    RebootMarker reboot_marker_ = nullptr;

    /// Raw level for `button` this tick: the in-flight synthetic gesture's
    /// level while its hold is running, else the physical board level.
    bool synthetic_level(ButtonId button, uint32_t now_ms) const;

    // ---- processing state machines (moved verbatim from app_main.cpp) ----
    // Each takes the resolved raw level (physical or synthetic) so the exact
    // same debounce/classification code serves both paths.

    /// BOOT button debounce state. A validated press edge records a start
    /// timestamp only when armed and past the boot grace; the release edge
    /// reboots iff that press was <= kBootShortMaxMs.
    struct BootButton {
        bool pressed_debounced = false;  // debounced level (true = pressed)
        uint8_t stable_count = 0;        // consecutive agreeing samples
        bool armed = false;              // a validated release has been seen
        uint32_t press_start_ms = 0;
    };

    /// PLUS button debounce state. One validated press creates one reset
    /// request; release re-arms it.
    struct ResetButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = true;
    };

    /// PWR starts disarmed so holding it through power-up cannot immediately
    /// turn the board back off. The first validated release arms the 2 s hold.
    struct PowerButton {
        bool pressed_debounced = false;
        uint8_t stable_count = 0;
        bool armed = false;
        bool poweroff_sent = false;
        uint32_t press_start_ms = 0;
    };

    void process_boot_button(uint32_t now_ms, bool raw_pressed);  // short press -> esp_restart()
    bool process_reset_button(bool raw_pressed);                  // true on validated press edge
    bool process_power_button(uint32_t now_ms, bool raw_pressed); // true on validated short release

    BootButton boot_;
    ResetButton reset_button_;
    PowerButton power_button_;
};

}  // namespace fluid_demo
