// app_main.cpp — fluid_box_3d application glue.
//
// Brings the board, motion, fluid, snapshot-exchange and renderer modules
// together in three pinned RTOS tasks:
//
//   sensor/control  core0  prio 7  ~100 Hz  IMU filtering, motion publish,
//                                           PLUS reset, PWR off, BOOT reboot
//   physics         core1  prio 8  ~30 Hz   PBF step + frame publish
//   render          core0  prio 5  ~30 Hz   acquire -> render -> release,
//                                           once-a-second telemetry
//
// All large state (fluid arrays, snapshot slots, renderer buffers) lives in
// file-scope statics, never on task stacks. Nothing here writes flash.

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "app_types.hpp"
#include "board.hpp"
#include "dev_console.hpp"
#include "display_service.hpp"
#include "fluid.hpp"
#include "motion.hpp"
#include "renderer.hpp"
#include "snapshot_exchange.hpp"

namespace {

using fluid_demo::DisplayService;
using fluid_demo::Fluid;
using fluid_demo::FluidStats;
using fluid_demo::MotionFilter;
using fluid_demo::ParticleFrame;
using fluid_demo::Renderer;
using fluid_demo::RenderStats;
using fluid_demo::SnapshotExchange;
using fluid_demo::Vec3;

constexpr char kTag[] = "fluid_demo";

// ---------------------------------------------------------------------------
// Timing. The sensor loop runs at 100 Hz while physics and display run at
// 30 Hz. With CONFIG_FREERTOS_HZ=1000 the periods are 10 and 34 ticks.
// vTaskDelayUntil keeps long-term cadence without an unbounded catch-up loop.
// ---------------------------------------------------------------------------

constexpr uint32_t kSensorHz = 100;
constexpr uint32_t kPhysicsHz = 30;
constexpr uint32_t kRenderHz = 30;

constexpr TickType_t kSensorPeriodTicks =
    static_cast<TickType_t>((1000000ULL / kSensorHz * configTICK_RATE_HZ + 999999ULL) / 1000000ULL);
constexpr TickType_t kPhysicsPeriodTicks =
    static_cast<TickType_t>((1000000ULL / kPhysicsHz * configTICK_RATE_HZ + 999999ULL) / 1000000ULL);
constexpr TickType_t kRenderPeriodTicks =
    static_cast<TickType_t>((1000000ULL / kRenderHz * configTICK_RATE_HZ + 999999ULL) / 1000000ULL);

// Fixed simulation step, independent of wake-up granularity.
constexpr float kPhysicsDt = 1.0f / 30.0f;

// Sensor dt clamp [2 ms, 100 ms] keeps the motion filter stable across
// scheduling jitter and the first sample.
constexpr float kMinDt = 0.002f;
constexpr float kMaxDt = 0.100f;

// BOOT (GPIO0, active low) debounce and short-press window.
constexpr uint8_t kBootDebounceSamples = 4;    // ~33 ms at ~120 Hz
constexpr uint32_t kBootGraceMs = 500;         // ignore the button right after boot
constexpr uint32_t kBootShortMaxMs = 1000;     // longer hold => not a reboot press
constexpr uint32_t kPowerOffHoldMs = 2000;     // matches factory long-press default

// Bounded task stacks; all heavy objects are static.
constexpr uint32_t kSensorStackBytes = 4096;
constexpr uint32_t kPhysicsStackBytes = 4096;
constexpr uint32_t kRenderStackBytes = 4096;

// ---------------------------------------------------------------------------
// Global/static state — never on task stacks.
// ---------------------------------------------------------------------------

fluid_demo::BoardHandles s_board;
MotionFilter s_filter;
Fluid s_fluid;
SnapshotExchange s_snapshots;
DisplayService s_display;
Renderer s_renderer;

// Motion state published by the sensor task to the physics task under a short
// critical section.
struct SharedMotion {
    Vec3 apparent_accel{0.0f, 0.0f, 6.0f};  // rest-gravity placeholder until a valid sample
    Vec3 raw_accel{0.0f, 0.0f, 0.0f};
    bool valid{false};
};
portMUX_TYPE s_motion_mux = portMUX_INITIALIZER_UNLOCKED;
SharedMotion s_motion;

// Reset request: sensor task sets, physics task consumes (and applies) in its
// own context.
std::atomic<bool> s_reset_requested{false};

void request_fluid_reset() {
    s_reset_requested.store(true, std::memory_order_release);
}

// Physics step wall time, written by the physics task, read for telemetry.
// Fluid telemetry is copied by its owner (physics task) into atomics. Reading
// FluidStats directly from the render core would be a C++ data race.
std::atomic<uint32_t> s_fluid_epoch{0};
std::atomic<uint64_t> s_candidate_checks{0};
std::atomic<uint32_t> s_nonfinite_resets{0};
std::atomic<uint32_t> s_physics_us{0};

// BOOT button debounce state (sensor task only).
struct BootButton {
    bool pressed_debounced = false;   // debounced level (true = pressed)
    uint8_t stable_count = 0;         // consecutive agreeing samples
    bool armed = false;               // a validated release has been seen
    uint32_t press_start_ms = 0;
};
BootButton s_boot;

// PLUS button debounce state (sensor task only). One validated press creates
// one reset request; release re-arms it.
struct ResetButton {
    bool pressed_debounced = false;
    uint8_t stable_count = 0;
    bool armed = true;
};
ResetButton s_reset_button;

// PWR starts disarmed so holding it through power-up cannot immediately turn
// the board back off. The first validated release arms the 2-second hold.
struct PowerButton {
    bool pressed_debounced = false;
    uint8_t stable_count = 0;
    bool armed = false;
    bool poweroff_sent = false;
    uint32_t press_start_ms = 0;
};
PowerButton s_power_button;

// ---------------------------------------------------------------- utilities

void log_startup_info() {
    esp_chip_info_t chip{};
    esp_chip_info(&chip);
    uint32_t flash_bytes = 0;
    ESP_ERROR_CHECK(esp_flash_get_size(nullptr, &flash_bytes));
    const size_t flash_mb = flash_bytes / (1024 * 1024);
    const size_t psram_mb = esp_psram_get_size() / (1024 * 1024);
    ESP_LOGI(kTag,
             "ESP32-S3 rev %u, %d cores, %u MB flash, %u MB PSRAM, tick %u Hz",
             chip.revision, chip.cores, static_cast<unsigned>(flash_mb),
             static_cast<unsigned>(psram_mb), static_cast<unsigned>(configTICK_RATE_HZ));
    ESP_LOGI(kTag,
             "free heap %u B internal, %u B PSRAM",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
}

[[noreturn]] void fatal_startup(const char *what, esp_err_t err) {
    ESP_LOGE(kTag, "FATAL: %s failed: %s - entering recovery idle",
             what, esp_err_to_name(err));
    for (;;) {
        // Keep USB/ROM recovery available instead of entering a reboot loop.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---------------------------------------------------------------------------
// Sensor/control task — core0, prio 7, 100 Hz.
// ---------------------------------------------------------------------------

// Debounced BOOT press: treated as a reboot only when a validated short press
// fully releases; never on an initial low or while held.
void process_boot_button(const uint32_t now_ms) {
    // board_boot_pressed() reports the pressed state of the active-low GPIO0
    // level: true when the button is down (level low).
    const bool raw_pressed = fluid_demo::board_boot_pressed();
    const bool level_changed = raw_pressed != s_boot.pressed_debounced;
    if (level_changed) {
        if (++s_boot.stable_count >= kBootDebounceSamples) {
            s_boot.stable_count = 0;
            s_boot.pressed_debounced = raw_pressed;
            if (raw_pressed) {
                // Validated press edge; record start only when armed so a
                // button held across boot can never schedule a reboot.
                if (s_boot.armed && now_ms >= kBootGraceMs) {
                    s_boot.press_start_ms = now_ms;
                }
            } else {
                // Validated release edge: reboot if it was a short press.
                if (s_boot.press_start_ms != 0) {
                    const uint32_t held_ms = now_ms - s_boot.press_start_ms;
                    if (held_ms <= kBootShortMaxMs) {
                        ESP_LOGI(kTag, "BOOT short press (%u ms) - rebooting", held_ms);
                        esp_restart();
                    } else {
                        ESP_LOGI(kTag, "BOOT held %u ms - long press ignored", held_ms);
                    }
                    s_boot.press_start_ms = 0;
                }
            }
            s_boot.armed = true;
        }
    } else {
        s_boot.stable_count = 0;
    }
}

void process_reset_button() {
    const bool raw_pressed = fluid_demo::board_reset_pressed();
    if (raw_pressed != s_reset_button.pressed_debounced) {
        if (++s_reset_button.stable_count >= kBootDebounceSamples) {
            s_reset_button.stable_count = 0;
            s_reset_button.pressed_debounced = raw_pressed;
            if (raw_pressed && s_reset_button.armed) {
                s_reset_button.armed = false;
                s_reset_requested.store(true);
            } else if (!raw_pressed) {
                s_reset_button.armed = true;
            }
        }
    } else {
        s_reset_button.stable_count = 0;
    }
}

void process_power_button(const uint32_t now_ms) {
    const bool raw_pressed = fluid_demo::board_power_pressed();
    if (raw_pressed != s_power_button.pressed_debounced) {
        if (++s_power_button.stable_count >= kBootDebounceSamples) {
            s_power_button.stable_count = 0;
            s_power_button.pressed_debounced = raw_pressed;
            if (raw_pressed) {
                if (s_power_button.armed) {
                    s_power_button.press_start_ms = now_ms;
                }
            } else {
                s_power_button.armed = true;
                s_power_button.poweroff_sent = false;
                s_power_button.press_start_ms = 0;
            }
        }
    } else {
        s_power_button.stable_count = 0;
        if (!raw_pressed && !s_power_button.armed) {
            s_power_button.armed = true;
        }
    }

    if (s_power_button.pressed_debounced && s_power_button.armed &&
        !s_power_button.poweroff_sent && s_power_button.press_start_ms != 0 &&
        now_ms - s_power_button.press_start_ms >= kPowerOffHoldMs) {
        s_power_button.poweroff_sent = true;
        const esp_err_t err = fluid_demo::board_power_off();
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "board_power_off failed: %s", esp_err_to_name(err));
        }
    }
}

void sensor_task(void *arg) {
    static_cast<void>(arg);

    int64_t last_motion_us = esp_timer_get_time();
    uint32_t last_err_log_s = 0;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kSensorPeriodTicks);

        const int64_t now_us = esp_timer_get_time();

        // --- IMU + motion filter ---
        Vec3 accel_mps2{}, gyro_rads{};
        bool fresh = false;
        const esp_err_t motion_err = board_read_motion(&accel_mps2, &gyro_rads, &fresh);
        if (motion_err == ESP_OK && fresh) {
            float dt = static_cast<float>(now_us - last_motion_us) * 1e-6f;
            if (dt < kMinDt || dt > kMaxDt || dt != dt) {  // clamp; NaN check included
                dt = 1.0f / static_cast<float>(kSensorHz);
            }
            const Vec3 apparent = s_filter.update(accel_mps2, gyro_rads, dt);
            if (s_filter.last_sample_accepted()) {
                last_motion_us = now_us;
                portENTER_CRITICAL(&s_motion_mux);
                s_motion.apparent_accel = apparent;
                s_motion.raw_accel = accel_mps2;
                s_motion.valid = true;
                portEXIT_CRITICAL(&s_motion_mux);
            }
        } else if (motion_err != ESP_OK) {
            const uint32_t now_s = static_cast<uint32_t>(now_us / 1000000ULL);
            if (now_s != last_err_log_s && !fluid_demo::dev_console_dump_active()) {
                last_err_log_s = now_s;
                ESP_LOGW(kTag, "board_read_motion failed: %s", esp_err_to_name(motion_err));
            }
        }

        // A development-console drive bypasses the physical IMU deterministically.
        Vec3 injected{};
        if (fluid_demo::dev_console_motion_override(&injected)) {
            portENTER_CRITICAL(&s_motion_mux);
            s_motion.apparent_accel = injected;
            s_motion.valid = true;
            portEXIT_CRITICAL(&s_motion_mux);
        }

        const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000ULL);
        // --- PLUS press -> reset; PWR 2-second hold -> power off; BOOT tap -> reboot ---
        process_reset_button();
        process_power_button(now_ms);
        process_boot_button(now_ms);
        // Always give the watched idle task a scheduling window, even if an
        // I2C timeout made this periodic iteration overrun.
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// Physics task — core1, prio 8, ~30 Hz. Owns the fixed simulation clock and
// publishes the newest frame.
// ---------------------------------------------------------------------------

void physics_task(void *arg) {
    static_cast<void>(arg);

    uint32_t sequence = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kPhysicsPeriodTicks);

        // Handle a reset notification in this task's context: swap the flag,
        // then re-roll the fluid into its deterministic lattice.
        if (s_reset_requested.exchange(false)) {
            s_fluid.reset();
            ESP_LOGI(kTag, "PLUS press - fluid reset (epoch %u)", s_fluid.reset_epoch());
        }

        // Latest apparent acceleration from the sensor task.
        Vec3 apparent{0.0f, 0.0f, 6.0f};
        portENTER_CRITICAL(&s_motion_mux);
        if (s_motion.valid) {
            apparent = s_motion.apparent_accel;
        }
        portEXIT_CRITICAL(&s_motion_mux);

        // step() returns false only on a skip (never happens here: count is
        // set at boot and dt is fixed finite) or when nonfinite state forced
        // a deterministic reset — either way the frame below is still valid.
        const int64_t step_start_us = esp_timer_get_time();
        static_cast<void>(s_fluid.step(apparent, kPhysicsDt));
        s_physics_us.store(static_cast<uint32_t>(esp_timer_get_time() - step_start_us));
        const FluidStats &fluid_stats = s_fluid.stats();
        s_fluid_epoch.store(s_fluid.reset_epoch());
        s_candidate_checks.store(fluid_stats.candidate_checks);
        s_nonfinite_resets.store(fluid_stats.nonfinite_resets);

        ParticleFrame *slot = s_snapshots.begin_write();
        if (slot == nullptr) {
            vTaskDelay(1);
            continue;  // defensive pool exhaustion; preserve idle watchdog
        }
        s_fluid.fill_frame(*slot, ++sequence);
        s_snapshots.publish(slot);
        // vTaskDelayUntil() does not block after an overrun. One tick here
        // guarantees idle/watchdog service without changing on-time cadence.
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// Render task — core0, prio 5, ~30 Hz. Holds a snapshot only through render.
// ---------------------------------------------------------------------------

void log_telemetry() {
    const RenderStats rs = s_renderer.stats();
    const uint64_t current_checks = s_candidate_checks.load();
    static uint64_t last_candidate_checks = 0;
    const uint64_t candidate_delta =
        current_checks >= last_candidate_checks
            ? current_checks - last_candidate_checks
            : current_checks;  // reset() clears the per-run counter
    last_candidate_checks = current_checks;

    SharedMotion motion{};
    portENTER_CRITICAL(&s_motion_mux);
    motion = s_motion;
    portEXIT_CRITICAL(&s_motion_mux);

    ESP_LOGI(kTag,
             "count=%u epoch=%u phys=%uus raster=%uus dma=%uus frame=%uus "
             "cand/s=%llu raw=(%.2f,%.2f,%.2f) sim=(%.2f,%.2f,%.2f) "
             "heap_int_min=%u heap_psram_min=%u missed=%u nonfinite=%u",
             static_cast<unsigned>(s_fluid.count()),
             static_cast<unsigned>(s_fluid_epoch.load()),
             static_cast<unsigned>(s_physics_us.load()),
             static_cast<unsigned>(rs.raster_us),
             static_cast<unsigned>(rs.dma_wait_us),
             static_cast<unsigned>(rs.frame_us),
             static_cast<unsigned long long>(candidate_delta),
             static_cast<double>(motion.raw_accel.x),
             static_cast<double>(motion.raw_accel.y),
             static_cast<double>(motion.raw_accel.z),
             static_cast<double>(motion.apparent_accel.x),
             static_cast<double>(motion.apparent_accel.y),
             static_cast<double>(motion.apparent_accel.z),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM)),
             static_cast<unsigned>(rs.missed_transfers),
             static_cast<unsigned>(s_nonfinite_resets.load()));
}

void render_task(void *arg) {
    static_cast<void>(arg);

    uint32_t last_log_s = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kRenderPeriodTicks);
        if (fluid_demo::dev_console_dump_active()) {
            // The console also runs on core 0. Park raster work while it owns
            // stdout so protocol lines stay intact and IDLE0 services the WDT.
            vTaskDelay(pdMS_TO_TICKS(10));
            last_wake = xTaskGetTickCount();
            continue;
        }

        const ParticleFrame *frame = s_snapshots.acquire_latest();
        if (frame != nullptr) {
            const esp_err_t rerr = s_renderer.render(*frame);
            if (rerr != ESP_OK) {
                ESP_LOGW(kTag, "render failed: %s", esp_err_to_name(rerr));
            }
            s_snapshots.release(frame);
        }

        // Once-a-second telemetry.
        const uint32_t now_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
        if (now_s != last_log_s && !fluid_demo::dev_console_dump_active()) {
            last_log_s = now_s;
            log_telemetry();
        }
        // Same overrun guard as physics: harmless while on schedule.
        vTaskDelay(1);
    }
}

}  // namespace

// ---------------------------------------------------------------- entry point

extern "C" void app_main(void) {
    log_startup_info();

    // Board: direct ST7789 display, QMI8658 IMU, PLUS/BOOT inputs.
    ESP_LOGI(kTag, "board_init (ESP32-S3-Touch-LCD-1.54)");
    esp_err_t err = board_init(&s_board);
    if (err != ESP_OK) {
        fatal_startup("board init", err);
    }

    // The shell's DisplayService owns the panel transport + PSRAM capture and
    // must init first; the Renderer binds it for stripe streaming.
    ESP_LOGI(kTag, "display init");
    err = s_display.init(s_board.panel, s_board.io);
    if (err != ESP_OK) {
        fatal_startup("display init", err);
    }

    ESP_LOGI(kTag, "renderer init");
    err = s_renderer.init(&s_display);
    if (err != ESP_OK) {
        fatal_startup("renderer init", err);
    }

    // Tuned count for the 240x240 panel and measured 30 Hz physics budget.
    if (!s_fluid.init(fluid_demo::kInitialParticles)) {
        fatal_startup("fluid init", ESP_ERR_INVALID_ARG);
    }
    ESP_LOGI(kTag, "fluid initialized: %u particles, radius %.4f",
             static_cast<unsigned>(s_fluid.count()), s_fluid.particle_radius());

    err = fluid_demo::dev_console_start(&s_renderer, request_fluid_reset);
    if (err != ESP_OK) {
        fatal_startup("dev console init", err);
    }

    BaseType_t ok = xTaskCreatePinnedToCore(sensor_task, "sensor", kSensorStackBytes,
                                            nullptr, 7, nullptr, 0);
    if (ok != pdPASS) {
        fatal_startup("sensor task create", ESP_ERR_NO_MEM);
    }
    ok = xTaskCreatePinnedToCore(physics_task, "physics", kPhysicsStackBytes,
                                 nullptr, 8, nullptr, 1);
    if (ok != pdPASS) {
        fatal_startup("physics task create", ESP_ERR_NO_MEM);
    }
    ok = xTaskCreatePinnedToCore(render_task, "render", kRenderStackBytes,
                                 nullptr, 5, nullptr, 0);
    if (ok != pdPASS) {
        fatal_startup("render task create", ESP_ERR_NO_MEM);
    }

    // app_main returns; the IDF-provided main task is then reclaimed.
}
