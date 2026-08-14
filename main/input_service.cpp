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

esp_err_t InputService::enqueue_swipe(const TouchGesture gesture)
{
    if (gesture != TouchGesture::SwipeLeft && gesture != TouchGesture::SwipeRight) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&fifo_mux_);
    const bool busy = swipe_pending_;
    if (!busy) {
        swipe_pending_ = true;
        swipe_began_ = false;
        swipe_gesture_ = gesture;
        swipe_y0_ = 120;
        swipe_y1_ = 120;
        if (gesture == TouchGesture::SwipeLeft) {
            swipe_x0_ = 200;
            swipe_x1_ = 40;
        } else {
            swipe_x0_ = 40;
            swipe_x1_ = 200;
        }
    }
    portEXIT_CRITICAL(&fifo_mux_);
    return busy ? ESP_ERR_NO_MEM : ESP_OK;
}

bool InputService::poll_touch(TouchEvent *out)
{
    bool emit_swipe = false;
    TouchEvent swipe_event{};
    portENTER_CRITICAL(&fifo_mux_);
    if (swipe_pending_ && !touch_contact_active_ && !touch_contact_cancelled_) {
        emit_swipe = true;
        if (!swipe_began_) {
            swipe_event.x = swipe_x0_;
            swipe_event.y = swipe_y0_;
            swipe_event.phase = TouchPhase::Begin;
            swipe_event.gesture = TouchGesture::None;
            swipe_began_ = true;
        } else {
            swipe_event.x = swipe_x1_;
            swipe_event.y = swipe_y1_;
            swipe_event.phase = TouchPhase::End;
            swipe_event.gesture = swipe_gesture_;
            swipe_pending_ = false;
            swipe_began_ = false;
        }
    }
    portEXIT_CRITICAL(&fifo_mux_);
    if (emit_swipe) {
        if (out != nullptr) {
            *out = swipe_event;
        }
        return true;
    }

    TouchSample sample{};
    last_touch_error_ = fluid_demo::board_read_touch(sample);
    if (last_touch_error_ != ESP_OK) {
        // Fail closed when the controller's short report window is lost.
        // The missing report could be any phase. Drop the retained contact
        // and quarantine pressed reports until an explicit finger-up report;
        // otherwise a later report from the same finger could become a second
        // Begin, or a new contact could complete the old one.
        touch_contact_active_ = false;
        touch_contact_cancelled_ = true;
        return false;
    }
    if (!sample.fresh) {
        return false;
    }
    if (touch_contact_cancelled_) {
        // Consume the remainder of the uncertain physical contact without
        // emitting Begin/Move/End. If the lost report was already End, this
        // deliberately drops the next complete contact and uses its release
        // only to re-arm; false launcher/app actions are worse than one
        // ignored touch after a transport failure.
        if (!sample.pressed) {
            touch_contact_cancelled_ = false;
        }
        return false;
    }
    if (!sample.pressed) {
        // Release report: coordinates are dropped by the board, so the End
        // event carries the last pressed coordinates retained above, plus the
        // controller-encoded gesture from the release report. Re-arms the
        // contact so the next physical contact emits a fresh Begin.
        const bool was_active = touch_contact_active_;
        touch_contact_active_ = false;
        if (was_active && out != nullptr) {
            out->x = touch_last_x_;
            out->y = touch_last_y_;
            out->phase = TouchPhase::End;
            out->gesture = sample.gesture;
        }
        return was_active;
    }
    if (touch_contact_active_) {
        // Continuing contact: forward a Move with the fresh pressed
        // coordinates and remember them for the eventual End event.
        touch_last_x_ = sample.x;
        touch_last_y_ = sample.y;
        if (out != nullptr) {
            out->x = sample.x;
            out->y = sample.y;
            out->phase = TouchPhase::Move;
            out->gesture = TouchGesture::None;
        }
        return true;
    }
    // First pressed report of a new contact: Begin arms the contact. The
    // gesture field is None on Begin/Move because the controller's gesture
    // register only carries the just-completed contact's swipe.
    touch_contact_active_ = true;
    touch_last_x_ = sample.x;
    touch_last_y_ = sample.y;
    if (out != nullptr) {
        out->x = sample.x;
        out->y = sample.y;
        out->phase = TouchPhase::Begin;
        out->gesture = TouchGesture::None;
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
