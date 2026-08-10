// input_service.cpp — shell-owned BOOT/PLUS/PWR button pipeline.
//
// The three debounce state machines and their classification rules are moved
// VERBATIM from the legacy sensor task (app_main.cpp:122-270): BOOT
// debounce/short-reboot with grace + short window, PLUS validated press edge
// (one reset per press), PWR validated release (short event, never after a
// long hold) plus the PWR >= 2 s board_power_off hold with held-through-boot
// disarm and poweroff_sent suppression. board.cpp's GPIO init, levels and the
// BAT_EN power-off sequence are untouched.

#include "input_service.hpp"

#include "esp_log.h"
#include "esp_system.h"

#include "board.hpp"

namespace fluid_demo {

namespace {
constexpr char kTag[] = "input_service";
}  // namespace

bool InputService::poll(uint32_t now_ms, ButtonEvent *out)
{
    bool emitted = false;

    // Legacy processing order (app_main.cpp:343-347): PLUS, then PWR, then
    // BOOT — BOOT may esp_restart() (never returns) and PWR may
    // board_power_off(). The first event in that order wins the single slot.
    if (process_reset_button()) {
        emitted = true;
        if (out != nullptr) {
            *out = ButtonEvent::PlusPress;
        }
    }
    if (!emitted && process_power_button(now_ms)) {
        emitted = true;
        if (out != nullptr) {
            *out = ButtonEvent::PwrShort;
        }
    }
    process_boot_button(now_ms);
    return emitted;
}

bool InputService::process_reset_button()
{
    const bool raw_pressed = fluid_demo::board_reset_pressed();
    if (raw_pressed != reset_button_.pressed_debounced) {
        if (++reset_button_.stable_count >= kBootDebounceSamples) {
            reset_button_.stable_count = 0;
            reset_button_.pressed_debounced = raw_pressed;
            if (raw_pressed && reset_button_.armed) {
                reset_button_.armed = false;
                return true;
            } else if (!raw_pressed) {
                reset_button_.armed = true;
            }
        }
    } else {
        reset_button_.stable_count = 0;
    }
    return false;
}

bool InputService::process_power_button(const uint32_t now_ms)
{
    const bool raw_pressed = fluid_demo::board_power_pressed();
    bool fired = false;
    if (raw_pressed != power_button_.pressed_debounced) {
        if (++power_button_.stable_count >= kBootDebounceSamples) {
            power_button_.stable_count = 0;
            power_button_.pressed_debounced = raw_pressed;
            if (raw_pressed) {
                if (power_button_.armed) {
                    power_button_.press_start_ms = now_ms;
                }
            } else {
                // Validated release edge. Report a short press only when it
                // never grew into a 2 s hold (poweroff_sent suppression): the
                // board-side power-off path already ran or was attempted, so
                // no PwrShort may follow a long hold.
                const bool powered_off = power_button_.poweroff_sent;
                const uint32_t held_ms =
                    power_button_.press_start_ms != 0
                        ? now_ms - power_button_.press_start_ms
                        : 0;
                power_button_.armed = true;
                power_button_.poweroff_sent = false;
                power_button_.press_start_ms = 0;
                if (!powered_off && held_ms != 0 && held_ms < kPowerOffHoldMs) {
                    fired = true;
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
        !power_button_.poweroff_sent && power_button_.press_start_ms != 0 &&
        now_ms - power_button_.press_start_ms >= kPowerOffHoldMs) {
        power_button_.poweroff_sent = true;
        const esp_err_t err = fluid_demo::board_power_off();
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "board_power_off failed: %s", esp_err_to_name(err));
        }
    }
    return fired;
}

void InputService::process_boot_button(const uint32_t now_ms)
{
    // board_boot_pressed() reports the pressed state of the active-low GPIO0
    // level: true when the button is down (level low).
    const bool raw_pressed = fluid_demo::board_boot_pressed();
    const bool level_changed = raw_pressed != boot_.pressed_debounced;
    if (level_changed) {
        if (++boot_.stable_count >= kBootDebounceSamples) {
            boot_.stable_count = 0;
            boot_.pressed_debounced = raw_pressed;
            if (raw_pressed) {
                // Validated press edge; record start only when armed so a
                // button held across boot can never schedule a reboot.
                if (boot_.armed && now_ms >= kBootGraceMs) {
                    boot_.press_start_ms = now_ms;
                }
            } else {
                // Validated release edge: reboot if it was a short press.
                if (boot_.press_start_ms != 0) {
                    const uint32_t held_ms = now_ms - boot_.press_start_ms;
                    if (held_ms <= kBootShortMaxMs) {
                        ESP_LOGI(kTag, "BOOT short press (%u ms) - rebooting", held_ms);
                        esp_restart();
                    } else {
                        ESP_LOGI(kTag, "BOOT held %u ms - long press ignored", held_ms);
                    }
                    boot_.press_start_ms = 0;
                }
            }
            boot_.armed = true;
        }
    } else {
        boot_.stable_count = 0;
    }
}

}  // namespace fluid_demo
