// runtime.cpp — S4 dispatch + generation barrier (coordinator).
//
// Ownership moved out of app_main: this file owns the compile-time registry
// (exactly {fluid_box, "Fluid Box"}), the shell service instances, the packed
// active-selection word, the per-lane acknowledgement bits, the three pinned
// lanes (created exactly once) and the coordinator loop that runs on the ESP
// main task. app_main is startup wiring only: it calls runtime_run(), which
// boots (board -> display -> console -> Fluid setup_once -> enter), spawns
// the lanes, then never returns.
//
// Transition protocol (every request, both directions, committed mode change
// emitted as "@DEV MODE fluid_box|launcher"):
//   1. Quiesce: clear the per-lane ack mask, set mode=Transition, bump the
//      generation and publish the packed word with kNoAppIndex (one release
//      store — lanes do exactly one acquire load per iteration, so they can
//      never observe a torn (app, generation) pair).
//   2. Wait for all three lane ack bits within 500 ms; on timeout the system
//      fails CLOSED with esp_restart(). Each lane acks exactly once per
//      transition, after its last old-generation app callback has returned.
//   3. Mandatory DisplayService::drain() before any leave — retire the
//      carried final stripe so the incoming app starts on a drained panel;
//      a failure also fails closed (esp_restart, never reused).
//   4. leave() the outgoing app (no allocation).
//   5. mode=Entering, enter() the incoming app (no allocation; drains stale
//      snapshots, opens the first-frame gate, leaves a pending reset alone),
//      then mode=Running. enter() failure fails closed.
//   6. Publish the run generation; each lane rebases its vTaskDelayUntil
//      anchor (last_wake = xTaskGetTickCount()) on the new generation before
//      resuming callbacks, so resume never bursts/catches up and the cadence
//      is 30/30/100 Hz on the immediate next iteration.
//   7. Emit "@DEV MODE <id|launcher>" (additive host protocol line).
//
// While no app is selected (launcher/idle), the sensor lane keeps polling the
// shell InputService so BOOT-reboot/PWR-off/PLUS stay live, motion/on_motion
// stop, and the render lane parks on a 10 ms vTaskDelay so IDLE0 keeps
// feeding the task watchdog. No task is recreated and no hardware is
// re-initialized after boot; enter/leave allocate nothing.
//
// Constants here (cores, priorities, stacks, 100/30/30 cadences, dump gate,
// telemetry line, motion acceptance ack, capture sequencing, reset trampoline
// and legacy console commands) are preserved verbatim from the pre-split
// app_main/dev_console behavior.

#include "runtime.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "board.hpp"
#include "console_service.hpp"
#include "display_service.hpp"
#include "fluid_app.hpp"
#include "input_service.hpp"
#include "motion_service.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "fluid_demo";

// ---------------------------------------------------------------------------
// Compile-time registry — exactly one entry: Fluid Box.
// ---------------------------------------------------------------------------

constexpr RegistryEntry kRegistry[] = {
    {"fluid_box", "Fluid Box", &s_fluid_app},
};
constexpr size_t kRegistryCount = sizeof(kRegistry) / sizeof(kRegistry[0]);

// The packed selection stores the index in the low 4 bits; the reserved
// kNoAppIndex must never collide with a real registry index.
static_assert(kRegistryCount <= kNoAppIndex,
              "registry must fit below the reserved kNoAppIndex slot");

constexpr uint32_t kAppGenMask = ~kAppIndexMask;

uint32_t pack_selection(uint32_t index, uint32_t generation)
{
    return ((generation << kAppGenShift) & kAppGenMask) | (index & kAppIndexMask);
}

// ---------------------------------------------------------------------------
// Coordinator state (main-task-owned; the lanes only load s_active_selection).
// ---------------------------------------------------------------------------

/// Packed dispatch word: app index (low 4) + quiesce/run generation (high 28).
/// One acquire load per lane iteration => no torn (app, generation) pair.
/// AppMode (s_mode) is coordinator-side only.
std::atomic<uint32_t> s_active_selection{pack_selection(0, 0)};  // Fluid, gen 0

/// Per-lane quiesce acknowledgement bits (never an ambiguous counter): each
/// lane sets its own bit exactly once per transition, after its last old-
/// generation app callback returned. The coordinator clears the mask before
/// every quiesce publish, so a stale bit can never complete a wait early; an
/// already-parked lane re-acks harmlessly on the next generation.
constexpr uint32_t kAckSensor = 1u << 0;
constexpr uint32_t kAckPhysics = 1u << 1;
constexpr uint32_t kAckRender = 1u << 2;
constexpr uint32_t kAckAll = kAckSensor | kAckPhysics | kAckRender;
std::atomic<uint32_t> s_lane_acks{0};

AppMode s_mode = AppMode::Running;
const RegistryEntry *s_active_app = nullptr;  // coordinator-side mirror of the word
uint32_t s_generation = 0;

constexpr uint32_t kBarrierTimeoutMs = 500;  // fail-closed quiesce deadline

/// Small bounded request queue; consumed exclusively on the ESP main task.
QueueHandle_t s_request_queue = nullptr;

// ---------------------------------------------------------------------------
// Shell service instances + transport binding (moved out of app_main).
// ---------------------------------------------------------------------------

BoardHandles s_board;
DisplayService s_display;
InputService s_input;
MotionService s_motion;
ConsoleService s_console;
DisplayFrame s_display_frame;

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
// measured around the app's render(), printed by the telemetry line.
uint32_t s_last_frame_dma_us = 0;

// ---------------------------------------------------------------------------
// Utilities.
// ---------------------------------------------------------------------------

void log_startup_info()
{
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

[[noreturn]] void fatal_startup(const char *what, esp_err_t err)
{
    ESP_LOGE(kTag, "FATAL: %s failed: %s - entering recovery idle",
             what, esp_err_to_name(err));
    for (;;) {
        // Keep USB/ROM recovery available instead of entering a reboot loop.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/// Fail-closed: any barrier/drain/enter failure restarts instead of letting
/// the shell continue with a torn transition. Called from the main task.
[[noreturn]] void fail_closed(const char *what)
{
    ESP_LOGE(kTag, "S4 BARRIER: %s - fail-closed esp_restart()", what);
    esp_restart();
}

// ---------------------------------------------------------------------------
// Lane parameters — exactly the pre-split task topology.
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

// Bounded task stacks; all heavy objects live in the app / shell services.
constexpr uint32_t kSensorStackBytes = 4096;
constexpr uint32_t kPhysicsStackBytes = 4096;
constexpr uint32_t kRenderStackBytes = 4096;

/// The dispatch check shared by every lane: exactly ONE acquire load per
/// iteration (returned to the caller), then react to the generation once
/// (quiesce -> park + ack; run -> rebase the cadence anchor). Callers use the
/// returned word for all gating so no lane ever branches on a second, possibly
/// newer observation — a torn (app, generation) pair is impossible.
uint32_t dispatch_step(uint32_t *seen_gen, TickType_t *last_wake, uint32_t ack_bit)
{
    const uint32_t word = s_active_selection.load(std::memory_order_acquire);
    const uint32_t index = word & kAppIndexMask;
    const uint32_t gen = word >> kAppGenShift;
    if (gen != *seen_gen) {
        if (index == kNoAppIndex) {
            // New quiesce generation: this lane's app callbacks are done.
            // Ack exactly once per transition (an already-parked lane re-acks
            // harmlessly — the coordinator clears the mask before publishing).
            s_lane_acks.fetch_or(ack_bit, std::memory_order_release);
        } else {
            // New run generation: rebase before resuming so the cadence is
            // exact on the next iteration — never a catch-up burst.
            *last_wake = xTaskGetTickCount();
        }
        *seen_gen = gen;
    }
    return word;
}

// ---------------------------------------------------------------------------
// Sensor/control task — core0, prio 7, 100 Hz while an app runs. A thin
// poll/router: the IMU poll, dt clamp and override live in MotionService
// (whose dt anchor advances only through the app's acceptance acknowledge-
// ment), and the BOOT/PLUS/PWR debounce + power/reboot actions live in
// InputService. Button polling stays live in every mode — the shell's
// input/power/reboot stays responsive in launcher/idle — while motion ticks
// and app callbacks stop when no app is selected or a transition is parked.
// PwrShort is observed here but deliberately routed nowhere until the
// launcher slice wires it shell-side (home/launch).
// ---------------------------------------------------------------------------

void sensor_task(void *arg)
{
    static_cast<void>(arg);

    // Sentinel forces the first observed generation through dispatch_step:
    // a boot-time queued transition cannot make this lane miss its quiesce ack.
    uint32_t seen_gen = UINT32_MAX;

    uint32_t last_err_log_s = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kSensorPeriodTicks);
        const uint32_t word = dispatch_step(&seen_gen, &last_wake, kAckSensor);
        const bool app_active = (word & kAppIndexMask) != kNoAppIndex;

        if (app_active) {
            // --- raw motion via MotionService: poll + dt clamp + override ---
            const MotionTick tick = s_motion.motion_tick();
            const esp_err_t motion_err = s_motion.last_read_error();
            if (motion_err != ESP_OK) {
                const uint32_t now_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
                if (now_s != last_err_log_s && !s_console.dump_active()) {
                    last_err_log_s = now_s;
                    ESP_LOGW(kTag, "board_read_motion failed: %s", esp_err_to_name(motion_err));
                }
            }
            // Feed the app's on_motion() result back so the service advances
            // its IMU time anchor only when the app accepts a fresh physical
            // sample; a rejected sample re-clamps against the previous
            // accepted anchor. An active override still publishes verbatim.
            s_motion.acknowledge(s_fluid_app.on_motion(tick));
        }

        // --- buttons: PLUS -> app event (only while an app runs); PWR short
        // --- observed, no action; BOOT/PWR hold handled inside InputService ---
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        ButtonEvent event;
        if (s_input.poll(now_ms, &event)) {
            if (event == ButtonEvent::PlusPress && app_active) {
                static_cast<void>(s_fluid_app.handle_event(AppEvent::PlusPress));
            }
            // ButtonEvent::PwrShort: no action until the launcher slice routes
            // it through the coordinator.
        }
        // Always give the watched idle task a scheduling window, even if an
        // I2C timeout made this periodic iteration overrun. While parked
        // (transition/launcher) this is the whole body: poll buttons at the
        // 100 Hz cadence, then yield.
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// Physics task — core1, prio 8, ~30 Hz. Owns the fixed simulation clock and
// publishes the newest frame. Parks (no app callbacks) while no app runs.
// ---------------------------------------------------------------------------

void physics_task(void *arg)
{
    static_cast<void>(arg);

    // Sentinel forces the first observed generation through dispatch_step.
    uint32_t seen_gen = UINT32_MAX;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kPhysicsPeriodTicks);
        const uint32_t word = dispatch_step(&seen_gen, &last_wake, kAckPhysics);
        if ((word & kAppIndexMask) == kNoAppIndex) {
            // Parked: no app callbacks during transition or launcher/idle.
            vTaskDelay(1);
            continue;
        }

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
// only through render. Parks on a 10 ms vTaskDelay while idle so IDLE0 keeps
// feeding the task watchdog, exactly like the dump gate.
// ---------------------------------------------------------------------------

void log_telemetry()
{
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

void render_task(void *arg)
{
    static_cast<void>(arg);

    // Sentinel forces the first observed generation through dispatch_step.
    uint32_t seen_gen = UINT32_MAX;

    uint32_t last_log_s = 0;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kRenderPeriodTicks);
        const uint32_t word = dispatch_step(&seen_gen, &last_wake, kAckRender);
        if ((word & kAppIndexMask) == kNoAppIndex) {
            // Parked (transition or launcher/idle): keep IDLE0 runnable so
            // the WDT is served; no raster or telemetry without an app.
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
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

        // Once-a-second telemetry (only while an app runs; the line itself is
        // byte-identical to the pre-split output).
        const uint32_t now_s = static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
        if (now_s != last_log_s && !s_console.dump_active()) {
            last_log_s = now_s;
            log_telemetry();
        }
        // Same overrun guard as physics: harmless while on schedule.
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// Coordinator — runs on the ESP main task.
// ---------------------------------------------------------------------------

void emit_mode_line(const char *name)
{
    // Additive host-protocol line on every committed mode change (S4; the
    // temporary `mode` command is removed at the S6 clean cutover).
    std::printf("@DEV MODE %s\r\n", name);
    std::fflush(stdout);
}

/// Execute one transition request synchronously (main task; the queue
/// serializes requests, so transitions never overlap). No allocation, no
/// task recreation, no hardware reinit.
void commit_transition()
{
    const RegistryEntry *old_app = s_active_app;
    const bool leaving_app = old_app != nullptr;
    const uint32_t next_index = leaving_app ? kNoAppIndex : 0;
    const RegistryEntry *next_app = leaving_app ? nullptr : &kRegistry[0];

    // 1. Publish Quiesce: clear per-lane ack bits, mode=Transition, bump the
    //    generation, store the packed (kNoAppIndex, gen) word once (release).
    s_mode = AppMode::Transition;
    s_lane_acks.store(0, std::memory_order_release);
    const uint32_t quiesce_gen = ++s_generation;
    s_active_selection.store(pack_selection(kNoAppIndex, quiesce_gen),
                             std::memory_order_release);

    // 2. Wait for all three lane acks; fail closed on a 500 ms timeout.
    //    Every lane finishes its current callback before acking, so no app
    //    callback runs after its lane's acknowledgement.
    const int64_t deadline_us =
        esp_timer_get_time() + static_cast<int64_t>(kBarrierTimeoutMs) * 1000;
    while ((s_lane_acks.load(std::memory_order_acquire) & kAckAll) != kAckAll) {
        if (esp_timer_get_time() >= deadline_us) {
            fail_closed("quiesce ack timeout");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 3. Mandatory DisplayService::drain() before any leave: retire the
    //    carried final stripe so the incoming app starts on a drained panel.
    if (s_display.drain() != ESP_OK) {
        fail_closed("display drain failed");
    }

    // 4. leave() the outgoing app (no allocation).
    if (old_app != nullptr) {
        old_app->app->leave();
    }

    // 5. Entering -> enter() the incoming app (no allocation), then Running.
    s_mode = AppMode::Entering;
    s_active_app = next_app;
    if (next_app != nullptr) {
        const esp_err_t err = next_app->app->enter();
        if (err != ESP_OK) {
            fail_closed("app enter failed");
        }
    }
    s_mode = AppMode::Running;

    // 6. Publish the run generation; lanes rebase and resume asynchronously.
    const uint32_t run_gen = ++s_generation;
    s_active_selection.store(pack_selection(next_index, run_gen),
                             std::memory_order_release);

    // 7. Committed mode change on the wire.
    const char *name = next_app != nullptr ? next_app->id : "launcher";
    emit_mode_line(name);

    ESP_LOGV(kTag, "transition committed: mode=%u generation=%u",
             static_cast<unsigned>(s_mode), static_cast<unsigned>(s_generation));
}

}  // namespace

const RegistryEntry *registry()
{
    return kRegistry;
}

esp_err_t runtime_enqueue_request(RuntimeRequest request)
{
    // Thread-safe (console REPL -> coordinator); never blocks and never runs
    // the transition inline.
    const QueueHandle_t queue = s_request_queue;
    if (queue == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueSend(queue, &request, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

[[noreturn]] void runtime_run()
{
    // The small bounded request queue is consumed on this (ESP main) task.
    s_request_queue = xQueueCreate(8, sizeof(RuntimeRequest));
    if (s_request_queue == nullptr) {
        fatal_startup("request queue create", ESP_ERR_NO_MEM);
    }

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
    // via a trampoline bound exactly once here, never rebound. The temporary
    // `mode next` command enqueues into the coordinator queue created above.
    err = s_console.start(&s_display, &s_motion, app_reset_trampoline);
    if (err != ESP_OK) {
        fatal_startup("console service init", err);
    }

    // Boot into Fluid: enter happens before the lanes start so the first run
    // generation is already published when they first acquire-load the word.
    s_active_app = &kRegistry[0];
    s_mode = AppMode::Entering;
    err = s_active_app->app->enter();
    if (err != ESP_OK) {
        fatal_startup("fluid app enter", err);
    }
    s_mode = AppMode::Running;
    s_generation = 0;
    s_active_selection.store(pack_selection(0, 0), std::memory_order_release);
    emit_mode_line(kRegistry[0].id);

    // Create the three pinned lanes exactly once; they are never recreated.
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

    ESP_LOGI(kTag,
             "S4 coordinator live: %u registered app(s), `mode next` toggles "
             "Fluid Box <-> launcher/idle",
             static_cast<unsigned>(kRegistryCount));

    // Coordinator loop on the ESP main task; transitions run synchronously,
    // one at a time, in queue order.
    for (;;) {
        RuntimeRequest request;
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) != pdTRUE) {
            continue;  // unreachable: portMAX_DELAY only returns on a send
        }
        if (request == RuntimeRequest::NextMode) {
            commit_transition();
        }
        // Unknown enum values cannot be produced; ignore them defensively.
    }
}

}  // namespace fluid_demo
