// app_main.cpp — fluid_box_3d application glue.
//
// Brings the board, the shell's DisplayService/InputService/MotionService/
// ConsoleService and the registered FluidBoxApp together in three pinned RTOS
// tasks (direct boot, Fluid stays the running app until the launcher slice):
//
//   sensor/control  core0  prio 7  ~100 Hz  MotionService poll + app.on_motion,
//                                           InputService buttons (PLUS reset,
//                                           PWR off, BOOT reboot)
//   physics         core1  prio 8  ~30 Hz   app.update(): PBF step + frame publish
//   render          core0  prio 5  ~30 Hz   app.render(): acquire -> raster,
//                                           once-a-second telemetry
//
// All heavy state lives inside the namespace-scope s_fluid_app (Fluid,
// MotionFilter, SnapshotExchange, reset/telemetry atomics, raster buffers) and
// the shell services (s_display DMA stripes/capture, s_input debounce state,
// s_motion override state), never on task stacks. Nothing here writes flash.
//
// The render lane measures the per-frame display DMA wait around
// s_fluid_app.render() (the DisplayService only exposes a cumulative counter
// and the app drives transport exclusively through DisplayFrame ops), feeding
// the byte-identical telemetry line composed with the app's stats.

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

#include "app_shell.hpp"
#include "app_types.hpp"
#include "board.hpp"
#include "console_service.hpp"
#include "display_service.hpp"
#include "fluid_app.hpp"
#include "input_service.hpp"
#include "motion_service.hpp"

namespace {

using fluid_demo::AppEvent;
using fluid_demo::AppStats;
using fluid_demo::ButtonEvent;
using fluid_demo::ConsoleService;
using fluid_demo::DisplayFrame;
using fluid_demo::DisplayService;
using fluid_demo::InputService;
using fluid_demo::MotionService;
using fluid_demo::MotionTick;
using fluid_demo::s_fluid_app;

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

// Bounded task stacks; all heavy objects live in s_fluid_app / s_display.
constexpr uint32_t kSensorStackBytes = 4096;
constexpr uint32_t kPhysicsStackBytes = 4096;
constexpr uint32_t kRenderStackBytes = 4096;

// ---------------------------------------------------------------------------
// Global/static state — never on task stacks. All simulation, motion and
// raster state is owned by the app; the shell keeps only board/display.
// ---------------------------------------------------------------------------

fluid_demo::BoardHandles s_board;
DisplayService s_display;
DisplayFrame s_display_frame;

// Shell services (S3): buttons/reboot/power-off, raw motion + override, REPL.
InputService s_input;
MotionService s_motion;
ConsoleService s_console;

// Shell-bound transport ops: the app drives render sequencing through them.
esp_err_t frame_wait_previous(void *transport)
{
    return static_cast<DisplayService *>(transport)->wait_previous_transfer();
}

bool frame_latch_capture(void *transport)
{
    static_cast<DisplayService *>(transport)->latch_capture();
    return true;
}

esp_err_t frame_submit(void *transport, int s, int y0, int rows, const uint16_t *pixels)
{
    return static_cast<DisplayService *>(transport)->submit_stripe(s, y0, rows, pixels);
}

esp_err_t frame_finish(void *transport)
{
    return static_cast<DisplayService *>(transport)->finish_frame();
}

uint32_t frame_capture_copy_us(void *transport)
{
    return static_cast<DisplayService *>(transport)->capture_copy_us();
}

void bind_display_frame()
{
    s_display_frame.stripe[0] = s_display.stripe_buffer(0);
    s_display_frame.stripe[1] = s_display.stripe_buffer(1);
    s_display_frame.transport = &s_display;
    s_display_frame.ops.wait_previous = frame_wait_previous;
    s_display_frame.ops.latch_capture = frame_latch_capture;
    s_display_frame.ops.submit = frame_submit;
    s_display_frame.ops.finish = frame_finish;
    s_display_frame.ops.capture_copy_us = frame_capture_copy_us;
}

// Reset trampoline consumed by the console; the app owns the atomic. Bound
// exactly once at console start and never rebound (non-lifecycle callback).
void app_reset_trampoline()
{
    s_fluid_app.request_fluid_reset();
}

// Per-frame display DMA wait (render task only): cumulative counter delta
// measured around s_fluid_app.render(), printed by the telemetry line.
uint32_t s_last_frame_dma_us = 0;

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
// Sensor/control task — core0, prio 7, 100 Hz. A thin poll/router: the IMU
// poll, dt clamp and override live in MotionService (whose dt anchor advances
// only through the app's acceptance acknowledgement), and the BOOT/PLUS/PWR
// debounce + power/reboot actions live in InputService. PwrShort is observed
// here but deliberately routed nowhere until the launcher slice wires it
// shell-side (home/launch).
// ---------------------------------------------------------------------------

void sensor_task(void *arg) {
    static_cast<void>(arg);

    uint32_t last_err_log_s = 0;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kSensorPeriodTicks);

        const int64_t now_us = esp_timer_get_time();

        // --- raw motion via MotionService: poll + dt clamp + override ---
        const MotionTick tick = s_motion.motion_tick();
        const esp_err_t motion_err = s_motion.last_read_error();
        if (motion_err != ESP_OK) {
            const uint32_t now_s = static_cast<uint32_t>(now_us / 1000000ULL);
            if (now_s != last_err_log_s && !s_console.dump_active()) {
                last_err_log_s = now_s;
                ESP_LOGW(kTag, "board_read_motion failed: %s", esp_err_to_name(motion_err));
            }
        }
        // Feed the app's on_motion() result back so the service advances its
        // IMU time anchor only when the app accepts a fresh physical sample;
        // a rejected sample re-clamps against the previous accepted anchor.
        // An active override still publishes verbatim on rejection.
        s_motion.acknowledge(s_fluid_app.on_motion(tick));

        // --- buttons: PLUS -> app event; PWR short -> observed, no action ---
        const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000ULL);
        ButtonEvent event;
        if (s_input.poll(now_ms, &event)) {
            if (event == ButtonEvent::PlusPress) {
                static_cast<void>(s_fluid_app.handle_event(AppEvent::PlusPress));
            }
            // ButtonEvent::PwrShort: no action until the launcher slice routes
            // it through the coordinator (queue arrives with the runtime slice).
        }
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

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kPhysicsPeriodTicks);

        // Pending reset consumption, motion read, PBF step and snapshot
        // publish all live inside the app (update lane).
        static_cast<void>(s_fluid_app.update(fluid_demo::App::kPhysicsDt));

        // vTaskDelayUntil() does not block after an overrun. One tick here
        // guarantees idle/watchdog service without changing on-time cadence.
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// Render task — core0, prio 5, ~30 Hz. The app internally holds a snapshot
// only through render.
// ---------------------------------------------------------------------------

void log_telemetry() {
    const AppStats st = s_fluid_app.stats();
    const uint64_t current_checks = st.candidate_checks;
    static uint64_t last_candidate_checks = 0;
    const uint64_t candidate_delta =
        current_checks >= last_candidate_checks
            ? current_checks - last_candidate_checks
            : current_checks;  // reset() clears the per-run counter
    last_candidate_checks = current_checks;

    ESP_LOGI(kTag,
             "count=%u epoch=%u phys=%uus raster=%uus dma=%uus frame=%uus "
             "cand/s=%llu raw=(%.2f,%.2f,%.2f) sim=(%.2f,%.2f,%.2f) "
             "heap_int_min=%u heap_psram_min=%u missed=%u nonfinite=%u",
             static_cast<unsigned>(st.count),
             static_cast<unsigned>(st.epoch),
             static_cast<unsigned>(st.physics_us),
             static_cast<unsigned>(st.raster_us),
             static_cast<unsigned>(s_last_frame_dma_us),
             static_cast<unsigned>(st.frame_us),
             static_cast<unsigned long long>(candidate_delta),
             static_cast<double>(st.raw[0]),
             static_cast<double>(st.raw[1]),
             static_cast<double>(st.raw[2]),
             static_cast<double>(st.apparent[0]),
             static_cast<double>(st.apparent[1]),
             static_cast<double>(st.apparent[2]),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM)),
             static_cast<unsigned>(s_display.missed_transfers()),
             static_cast<unsigned>(st.nonfinite_resets));
}

void render_task(void *arg) {
    static_cast<void>(arg);

    uint32_t last_log_s = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kRenderPeriodTicks);
        if (s_console.dump_active()) {
            // The console also runs on core 0. Park raster work while it owns
            // stdout so protocol lines stay intact and IDLE0 services the WDT.
            vTaskDelay(pdMS_TO_TICKS(10));
            last_wake = xTaskGetTickCount();
            continue;
        }

        // Per-frame display DMA wait: cumulative counter delta across the
        // app's render call. The delta is stored only when a frame was
        // actually rendered — a blank pass (no new snapshot) or a failed
        // pass touches no/incomplete transport, so the last completed
        // frame's coherent timing tuple is preserved. Kept in sync with the
        // app's own last-frame raster/frame telemetry for the composed line.
        const uint32_t dma_wait_begin = s_display.dma_wait_us();
        if (s_fluid_app.render(s_display_frame)) {
            s_last_frame_dma_us = s_display.dma_wait_us() - dma_wait_begin;
        }

        // Once-a-second telemetry.
        const uint32_t now_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
        if (now_s != last_log_s && !s_console.dump_active()) {
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
    // must init first; the app then drives it through a bound DisplayFrame.
    ESP_LOGI(kTag, "display init");
    err = s_display.init(s_board.panel, s_board.io);
    if (err != ESP_OK) {
        fatal_startup("display init", err);
    }

    bind_display_frame();

    ESP_LOGI(kTag, "fluid app init");
    err = s_fluid_app.setup_once();
    if (err != ESP_OK) {
        fatal_startup("fluid app init", err);
    }

    // The console binds the shell's DisplayService (capture) and MotionService
    // (motion/release/status); the reset callback is the app's reset atomic
    // via a trampoline bound exactly once here, never rebound.
    err = s_console.start(&s_display, &s_motion, app_reset_trampoline);
    if (err != ESP_OK) {
        fatal_startup("console service init", err);
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
