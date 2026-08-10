// input_service.cpp — shell-owned BOOT/PLUS/PWR button pipeline.
//
// The three debounce state machines and their classification rules are moved
// VERBATIM from the legacy sensor task (app_main.cpp:122-270): BOOT
// debounce/short-reboot with grace + short window, PLUS validated press edge
// (one reset per press), PWR validated release (short event, never after a
// long hold) plus the PWR >= 2 s board_power_off hold with held-through-boot
// disarm and poweroff_sent suppression. board.cpp's GPIO init, levels and the
// BAT_EN power-off sequence are untouched.
//
// Dev-console `input` gestures are enqueued into a bounded, thread-safe FIFO
// and replayed as synthetic raw levels through these exact state machines:
// poll() presents "pressed from dequeue until hold_ms elapses, then
// released" in place of the physical board level, so classification (short/
// long, the 2 s power-off hold, poweroff_sent suppression, BOOT short reboot)
// is identical for `input` and for a physical press.

#include "input_service.hpp"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

#include "board.hpp"

namespace fluid_demo {

namespace {
constexpr char kTag[] = "input_service";
}  // namespace

esp_err_t InputService::enqueue_gesture(const ButtonId button, const uint32_t hold_ms)
{
    // Bounded, thread-safe (console REPL task -> sensor lane): a tiny ring
    // under a portMUX critical section — no allocation, never blocks.
    portENTER_CRITICAL(&fifo_mux_);
    const bool full = fifo_count_ >= kGestureFifoCapacity;
    if (!full) {
        fifo_[(fifo_head_ + fifo_count_) % kGestureFifoCapacity] = {button, hold_ms};
        ++fifo_count_;
    }
    portEXIT_CRITICAL(&fifo_mux_);
    return full ? ESP_ERR_NO_MEM : ESP_OK;
}

bool InputService::poll_touch(TouchEvent *out)
{
    TouchSample sample{};
    last_touch_error_ = fluid_demo::board_read_touch(&sample);
    if (last_touch_error_ != ESP_OK || !sample.fresh) {
        return false;
    }
    if (!sample.pressed) {
        touch_contact_active_ = false;
        return false;
    }
    if (touch_contact_active_) {
        return false;
    }
    touch_contact_active_ = true;
    if (out != nullptr) {
        out->x = sample.x;
        out->y = sample.y;
    }
    return true;
}

void InputService::set_power_off_marker(const PowerOffMarker marker)
{
    power_off_marker_ = marker;
}

void InputService::set_reboot_marker(const RebootMarker marker)
{
    reboot_marker_ = marker;
}

bool InputService::synthetic_level(const ButtonId button, const uint32_t now_ms) const
{
    if (active_gesture_.active && active_gesture_.button == button) {
        // Unsigned wrap-safe: a stale gesture (now << start) reads as a huge
        // duration => released, and poll() deactivates it.
        return (now_ms - active_gesture_.hold_start_ms) < active_gesture_.hold_ms;
    }
    switch (button) {
        case ButtonId::Plus:
            return fluid_demo::board_reset_pressed();
        case ButtonId::Pwr:
            return fluid_demo::board_power_pressed();
        case ButtonId::Boot:
        default:
            return fluid_demo::board_boot_pressed();
    }
}

bool InputService::poll(uint32_t now_ms, ButtonEvent *out)
{
    // Keep the gesture active through a fixed release phase after its hold.
    // This guarantees four raw-release samples before a queued gesture of the
    // same button may start, so distinct FIFO entries cannot merge.
    if (active_gesture_.active) {
        const uint32_t elapsed_ms = now_ms - active_gesture_.hold_start_ms;
        if (elapsed_ms >= active_gesture_.hold_ms + kGestureReleaseMs) {
            active_gesture_.active = false;
        }
    }
    if (!active_gesture_.active) {
        portENTER_CRITICAL(&fifo_mux_);
        if (fifo_count_ != 0) {
            const Gesture gesture = fifo_[fifo_head_];
            fifo_head_ = (fifo_head_ + 1) % kGestureFifoCapacity;
            --fifo_count_;
            active_gesture_.active = true;
            active_gesture_.button = gesture.button;
            active_gesture_.hold_start_ms = now_ms;
            active_gesture_.hold_ms = gesture.hold_ms;
        }
        portEXIT_CRITICAL(&fifo_mux_);
    }

    bool emitted = false;

    // Legacy processing order (app_main.cpp:343-347): PLUS, then PWR, then
    // BOOT — BOOT may esp_restart() (never returns) and PWR may
    // board_power_off(). The first event in that order wins the single slot.
    if (process_reset_button(synthetic_level(ButtonId::Plus, now_ms))) {
        emitted = true;
        if (out != nullptr) {
            *out = ButtonEvent::PlusPress;
        }
    }
    if (!emitted && process_power_button(now_ms, synthetic_level(ButtonId::Pwr, now_ms))) {
        emitted = true;
        if (out != nullptr) {
            *out = ButtonEvent::PwrShort;
        }
    }
    process_boot_button(now_ms, synthetic_level(ButtonId::Boot, now_ms));
    return emitted;
}

bool InputService::process_reset_button(const bool raw_pressed)
{
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

bool InputService::process_power_button(const uint32_t now_ms, const bool raw_pressed)
{
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
        // Emit and flush @DEV POWEROFF immediately before the off sequence;
        // poweroff_sent then suppresses any PwrShort at the release edge, so
        // a long hold can never also produce a home event.
        if (power_off_marker_ != nullptr) {
            power_off_marker_();
        }
        const esp_err_t err = fluid_demo::board_power_off();
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "board_power_off failed: %s", esp_err_to_name(err));
        }
    }
    return fired;
}

void InputService::process_boot_button(const uint32_t now_ms, const bool raw_pressed)
{
    // board_boot_pressed() reports the pressed state of the active-low GPIO0
    // level: true when the button is down (level low). `raw_pressed` is the
    // resolved level (physical or synthetic).
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
                        if (reboot_marker_ != nullptr) {
                            reboot_marker_();
                        }
                        vTaskDelay(pdMS_TO_TICKS(50));
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
        // Arm only after observing idle. A BOOT button held through startup
        // stays disarmed, while the first synthetic/physical short press after
        // a normal idle boot is accepted.
        if (!raw_pressed && !boot_.armed) {
            boot_.armed = true;
        }
    }
}

}  // namespace fluid_demo
