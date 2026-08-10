#pragma once

#include <cstdint>

namespace fluid_demo {

/// Events produced by the shell's button pipeline (sensor lane). PwrShort is
/// deliberately routed nowhere until the launcher slice adds shell-side
/// home/launch behavior.
enum class ButtonEvent : uint8_t {
    PlusPress = 0,  ///< Validated PLUS press edge (app reset).
    PwrShort = 1,   ///< Validated PWR release < kPowerOffHoldMs, never after a long hold.
};

/// Owns the BOOT/PLUS/PWR debounce + classify pipeline moved verbatim from
/// app_main.cpp:122-270: BOOT debounce/short-reboot (grace + short window),
/// PLUS validated press edge, PWR validated release (< 2000 ms) and the
/// PWR >= 2 s board_power_off hold, including held-through-boot disarm and
/// poweroff_sent suppression. board.cpp keeps all GPIO init/levels and the
/// power-off sequence unchanged; this service calls board_power_off() inside
/// poll() from the sensor task, exactly the legacy task context.
///
/// Sensor-lane-owned: poll() must run at the 100 Hz cadence.
class InputService {
public:
    /// Debounce and classification constants (legacy app_main.cpp:76-80).
    static constexpr uint8_t kBootDebounceSamples = 4;  // ~33 ms at ~120 Hz
    static constexpr uint32_t kBootGraceMs = 500;       // ignore BOOT right after boot
    static constexpr uint32_t kBootShortMaxMs = 1000;   // longer hold => not a reboot
    static constexpr uint32_t kPowerOffHoldMs = 2000;   // matches factory long-press default

    /// Debounce all three buttons and classify this tick. Returns true and
    /// writes `out` when one event fired. BOOT short -> esp_restart() and
    /// PWR hold -> board_power_off() happen inside, never returning on
    /// success. Processing order is the legacy order (PLUS, PWR, BOOT) and
    /// the first event in that order wins the single out slot.
    bool poll(uint32_t now_ms, ButtonEvent *out);

private:
    // ---- debounce state machines (moved verbatim from app_main.cpp) ----

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

    void process_boot_button(uint32_t now_ms);     // short press -> esp_restart()
    bool process_reset_button();                   // true on validated press edge
    bool process_power_button(uint32_t now_ms);    // true on validated short release

    BootButton boot_;
    ResetButton reset_button_;
    PowerButton power_button_;
};

}  // namespace fluid_demo
