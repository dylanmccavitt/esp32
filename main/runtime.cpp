// runtime.cpp — persistent app dispatch and generation-barrier coordinator.
//
// Ownership moved out of app_main: this file owns the compile-time registry
// (exactly {fluid_box, "Fluid Box"}), the shell service instances, the packed
// dispatch word, the per-lane acknowledgement bits, the three pinned lanes
// (created exactly once) and the coordinator loop that runs on the ESP main
// task. app_main is startup wiring only: it calls runtime_run(), which boots
// (board -> display -> console -> Fluid setup_once; the launcher stable mode
// boots WITHOUT calling the Fluid app's enter()), spawns the lanes, then
// never returns.
//
// Packed dispatch word (one std::atomic<uint32_t>, one acquire load per lane
// iteration): low 4 bits = registry index (kNoAppIndex while no app runs),
// bits 4-5 = the published AppMode (Transition while quiescing; Launcher or
// Running once committed), high 26 bits = the monotonic quiesce/run
// generation. A lane can never observe a torn (app, mode, generation) triple.
//
// Transition protocol (Launch/Home, both directions, committed mode change
// emitted as "@DEV MODE fluid_box|launcher"):
//   1. Quiesce: clear the per-lane ack mask, set mode=Transition, bump the
//      generation and publish the packed (kNoAppIndex, Transition, gen) word
//      (one release store).
//   2. Wait for all three lane ack bits within 500 ms; on timeout the system
//      fails CLOSED with esp_restart(). Each lane acks exactly once per
//      transition, after its last old-generation app callback has returned;
//      Transition mode parks every lane, so no display work happens mid-
//      transition and no app callback runs after a lane acks.
//   3. Mandatory DisplayService::drain() before any leave — retire the
//      carried final stripe so the incoming app starts on a drained panel;
//      a failure also fails closed (esp_restart, never reused).
//   4. leave() the outgoing app (no allocation).
//   5. mode=Entering, enter() the incoming app (no allocation; drains stale
//      snapshots, opens the first-frame gate, leaves a pending reset alone),
//      then the committed stable mode (Running or Launcher). enter() failure
//      fails closed.
//   6. Publish the run generation with the committed mode; each lane rebases
//      its vTaskDelayUntil anchor (last_wake = xTaskGetTickCount()) on a new
//      Running generation before resuming callbacks, so resume never
//      bursts/catches up and the cadence is 30/30/100 Hz on the immediate
//      next iteration. A Launcher generation parks physics and makes the
//      render lane draw the launcher.
//   7. Emit "@DEV MODE <id|launcher>" (additive host protocol line) and
//      publish the stable-mode name for status telemetry.
//
// While the launcher is committed, the sensor lane keeps polling the shell
// InputService so BOOT-reboot/PWR-off/PLUS stay live and routes PLUS ->
// Launch / short PWR -> Home through the request queue; motion/on_motion
// stop; the render lane draws the launcher frame at the 30 Hz render cadence
// through the same bound DisplayFrame transport, and vanishes entirely during
// a Transition (only the coordinator's mandatory drain touches the display
// then). No task is recreated and no hardware is re-initialized after boot;
// enter/leave allocate nothing.
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
#include "launcher.hpp"
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

constexpr uint32_t kAppGenMask = ~(kAppIndexMask | kAppModeMask);

uint32_t pack_selection(uint32_t index, uint32_t mode, uint32_t generation)
{
    return ((generation << kAppGenShift) & kAppGenMask) |
           ((mode << kAppModeShift) & kAppModeMask) |
           (index & kAppIndexMask);
}

/// AppMode published in the packed word. Entering is coordinator-side only
/// and never appears in the word: between the quiesce and run publishes every
/// lane parks on the Transition mode.
AppMode word_mode(uint32_t word)
{
    return static_cast<AppMode>((word >> kAppModeShift) & 0x3u);
}

static_assert(static_cast<uint32_t>(AppMode::Transition) < (1u << 2),
              "AppMode must fit the packed 2-bit mode field");

// ---------------------------------------------------------------------------
// Coordinator state (main-task-owned; the lanes only load s_active_selection).
// ---------------------------------------------------------------------------

/// Packed dispatch word: app index (low 4) + AppMode (bits 4-5) +
/// quiesce/run generation (high 26). One acquire load per lane iteration =>
/// no torn (app, mode, generation) triple. Boots into the Launcher stable
/// mode (runtime_run() republishes the same value before the lanes start).
std::atomic<uint32_t> s_active_selection{
    pack_selection(kNoAppIndex, static_cast<uint32_t>(AppMode::Launcher), 0)};

/// Coordinator-side committed stable-mode name mirrored for any-task status
/// telemetry; updated once per committed transition and at boot.
std::atomic<const char *> s_mode_name{"launcher"};

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

// Power-off marker trampoline consumed by InputService: the console's
// @DEV POWEROFF line, emitted and flushed immediately before
// board_power_off(), from the sensor lane. Bound exactly once at startup.
void emit_poweroff_marker()
{
    s_console.emit_poweroff();
}

// Reboot marker trampoline consumed by InputService: the console's
// @DEV REBOOTING line, emitted and flushed immediately before esp_restart()
// from the sensor lane. Bound exactly once at startup.
void emit_reboot_marker()
{
    s_console.emit_rebooting();
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
    ESP_LOGE(kTag, "TRANSITION BARRIER: %s - fail-closed esp_restart()", what);
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
/// (Transition -> park + ack; Running -> rebase the cadence anchor; Launcher
/// -> park without ack). Callers use the returned word for all gating so no
/// lane ever branches on a second, possibly newer observation — a torn (app,
/// mode, generation) triple is impossible.
uint32_t dispatch_step(uint32_t *seen_gen, TickType_t *last_wake, uint32_t ack_bit)
{
    const uint32_t word = s_active_selection.load(std::memory_order_acquire);
    const uint32_t gen = word >> kAppGenShift;
    if (gen != *seen_gen) {
        if (word_mode(word) == AppMode::Transition) {
            // New quiesce generation: this lane's old-generation app
            // callbacks are done. Ack exactly once per transition (an
            // already-parked lane re-acks harmlessly — the coordinator clears
            // the mask before publishing). Because the same word also gates
            // app callbacks to off, no app callback can run after this ack.
            s_lane_acks.fetch_or(ack_bit, std::memory_order_release);
        } else if (word_mode(word) == AppMode::Running) {
            // New run generation: rebase before resuming so the cadence is
            // exact on the next iteration — never a catch-up burst.
            *last_wake = xTaskGetTickCount();
        }
        // Launcher: stable park mode — no app callbacks, no ack.
        *seen_gen = gen;
    }
    return word;
}

// ---------------------------------------------------------------------------
// Sensor/control task — core0, prio 7, 100 Hz. A thin poll/router: the IMU
// poll, dt clamp and override live in MotionService; its dt anchor advances
// only through the app's acceptance acknowledgement. InputService owns the
// dev-console synthetic gesture FIFO plus physical button and IRQ-latched
// touch polling in every mode. Motion ticks and app callbacks stop when the
// committed mode is Launcher or a transition is parked. PLUS routes to the
// app while Running or to Launch from launcher; a touch on the selected
// launcher entry also requests Launch. Short PWR routes Home while Running
// and no-ops at launcher.
// ---------------------------------------------------------------------------

void sensor_task(void *arg)
{
    static_cast<void>(arg);

    // Sentinel forces the first observed generation through dispatch_step:
    // a boot-time queued transition cannot make this lane miss its quiesce ack.
    uint32_t seen_gen = UINT32_MAX;

    uint32_t last_err_log_s = 0;
    uint32_t last_touch_err_log_s = UINT32_MAX;
    // Sensor-originated target requests are never dropped on coordinator
    // queue backpressure. Launch/Home are idempotent target intents, so one
    // retained slot is sufficient while the committed mode is unchanged.
    RuntimeRequest pending_request = RuntimeRequest::Launch;
    bool request_pending = false;
    auto enqueue_or_retain = [&](RuntimeRequest request) {
        if (request_pending) {
            pending_request = request;
            return;
        }
        if (runtime_enqueue_request(request) != ESP_OK) {
            pending_request = request;
            request_pending = true;
        }
    };
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, kSensorPeriodTicks);
        const uint32_t word = dispatch_step(&seen_gen, &last_wake, kAckSensor);
        const AppMode mode = word_mode(word);
        const bool app_active = mode == AppMode::Running;
        if (request_pending &&
            runtime_enqueue_request(pending_request) == ESP_OK) {
            request_pending = false;
        }

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

        // --- shell input: buttons preserve their existing launch/reset/home
        // --- behavior. Touch reports are consumed in every mode, but only the
        // --- visible selected launcher target has touch semantics; no app
        // --- callback or lifecycle work runs directly from this lane.
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        ButtonEvent event;
        const bool button_emitted = s_input.poll(now_ms, &event);
        if (button_emitted) {
            if (event == ButtonEvent::PlusPress) {
                if (mode == AppMode::Running) {
                    // PLUS while Fluid runs resets the simulation.
                    static_cast<void>(s_fluid_app.handle_event(AppEvent::PlusPress));
                } else if (mode == AppMode::Launcher) {
                    // PLUS from the launcher launches Fluid. During a
                    // Transition the event is dropped (no lifecycle work).
                    enqueue_or_retain(RuntimeRequest::Launch);
                }
            } else if (event == ButtonEvent::PwrShort) {
                if (mode == AppMode::Running) {
                    // Short PWR while Fluid runs returns to the launcher;
                    // InputService guarantees this is never a leftover of a
                    // long hold (poweroff_sent suppression).
                    enqueue_or_retain(RuntimeRequest::Home);
                }
                // Short PWR at the launcher: nothing to return to — no-op.
            }
        }

        TouchEvent touch;
        if (s_input.poll_touch(&touch)) {
            if (!button_emitted && mode == AppMode::Launcher &&
                launcher_accepts_launch_touch(touch.x, touch.y)) {
                ESP_LOGI(kTag, "touch launch x=%u y=%u",
                         static_cast<unsigned>(touch.x),
                         static_cast<unsigned>(touch.y));
                enqueue_or_retain(RuntimeRequest::Launch);
            }
        } else {
            const esp_err_t touch_err = s_input.last_touch_error();
            if (touch_err != ESP_OK) {
                const uint32_t now_s =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
                if (now_s != last_touch_err_log_s && !s_console.dump_active()) {
                    last_touch_err_log_s = now_s;
                    ESP_LOGW(kTag, "board_read_touch failed: %s",
                             esp_err_to_name(touch_err));
                }
            }
        }
        // Always give the watched idle task a scheduling window, even if an
        // I2C timeout made this periodic iteration overrun. While parked, this
        // polling plus the yield is the whole sensor-lane body.
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
        if (word_mode(word) != AppMode::Running) {
            // Parked: no app callbacks during a Transition or the launcher.
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
        if (word_mode(word) == AppMode::Transition) {
            // Mid-transition: no display work at all — the coordinator's
            // mandatory drain retires the carried stripe once all lanes ack.
            // Park on 10 ms so IDLE0 keeps feeding the task watchdog.
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (word_mode(word) == AppMode::Launcher) {
            if (s_console.dump_active()) {
                // The console owns stdout; park raster work so protocol lines
                // stay intact and IDLE0 services the WDT (same rule as Fluid).
                vTaskDelay(pdMS_TO_TICKS(10));
                last_wake = xTaskGetTickCount();
                continue;
            }
            // Launcher stable mode: draw the full launcher frame through the
            // same bound DisplayFrame transport at the 30 Hz render cadence.
            // No app telemetry exists here.
            static_cast<void>(render_launcher(s_display_frame));
            vTaskDelay(1);
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
    // Additive host-protocol line on every committed mode change.
    std::printf("\r\n@DEV MODE %s\r\n", name);
    std::fflush(stdout);
}

/// Execute one transition request synchronously (main task; the queue
/// serializes requests, so transitions never overlap). No allocation, no
/// task recreation, no hardware reinit. A request whose target equals the
/// already committed side is a no-op: no barrier, no wire line.
void commit_transition(RuntimeRequest request)
{
    const RegistryEntry *old_app = s_active_app;
    // nullptr target means the launcher stable mode.
    const RegistryEntry *next_app = nullptr;
    switch (request) {
        case RuntimeRequest::Launch:
            // PLUS from the launcher: Fluid Box is selected but not running.
            if (s_mode != AppMode::Launcher) {
                return;
            }
            next_app = &kRegistry[0];
            break;
        case RuntimeRequest::Home:
            // Short PWR while Fluid runs: return to the launcher. A short
            // PWR at the launcher is a no-op by design.
            if (s_mode != AppMode::Running) {
                return;
            }
            break;
        default:
            return;  // defensive; the queue only ever carries the above
    }
    const uint32_t next_index = next_app != nullptr
                                    ? static_cast<uint32_t>(next_app - kRegistry)
                                    : kNoAppIndex;

    // 1. Publish Quiesce: clear per-lane ack bits, mode=Transition, bump the
    //    generation, store the packed (kNoAppIndex, Transition, gen) word
    //    once (release).
    s_mode = AppMode::Transition;
    s_lane_acks.store(0, std::memory_order_release);
    const uint32_t quiesce_gen = ++s_generation;
    s_active_selection.store(
        pack_selection(kNoAppIndex, static_cast<uint32_t>(AppMode::Transition),
                       quiesce_gen),
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
    //    carried final stripe so the incoming side starts on a drained panel
    //    (the launcher's last frame in either direction).
    if (s_display.drain() != ESP_OK) {
        fail_closed("display drain failed");
    }

    // 4. leave() the outgoing app (no allocation). Nothing to leave when the
    //    launcher is the outgoing side.
    if (old_app != nullptr) {
        old_app->app->leave();
    }

    // 5. Entering -> enter() the incoming app (no allocation; leaves a
    //    pending reset alone), then the committed stable mode.
    s_mode = AppMode::Entering;
    s_active_app = next_app;
    if (next_app != nullptr) {
        const esp_err_t err = next_app->app->enter();
        if (err != ESP_OK) {
            fail_closed("app enter failed");
        }
    }
    s_mode = next_app != nullptr ? AppMode::Running : AppMode::Launcher;

    // 6. Publish the run generation with the committed mode; lanes rebase and
    //    resume (Running) or park and draw the launcher (Launcher).
    const uint32_t run_gen = ++s_generation;
    s_active_selection.store(
        pack_selection(next_index, static_cast<uint32_t>(s_mode), run_gen),
        std::memory_order_release);

    // Publish the committed stable-mode name for status telemetry.
    s_mode_name.store(next_app != nullptr ? next_app->id : "launcher",
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

const char *runtime_mode_name()
{
    return s_mode_name.load(std::memory_order_acquire);
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

    // The console binds the shell's DisplayService (capture), MotionService
    // (motion/release/status) and InputService (synthetic `input` gestures);
    // the reset callback is the app's reset atomic via a trampoline bound
    // exactly once here, never rebound.
    err = s_console.start(&s_display, &s_motion, &s_input, app_reset_trampoline);
    if (err != ESP_OK) {
        fatal_startup("console service init", err);
    }

    // InputService terminal-action markers: emit and flush the console's wire
    // marker immediately before restart/power-off from the sensor lane. Each
    // callback is bound exactly once here, never rebound.
    s_input.set_reboot_marker(&emit_reboot_marker);
    s_input.set_power_off_marker(&emit_poweroff_marker);

    // Boot stable mode is Launcher: the Fluid app's setup_once() above ran
    // exactly once, but enter() is deliberately deferred — the first Launch
    // (PLUS from the launcher) performs it inside the full barrier. Publishing
    // the (Launcher, gen 0) word before the lanes start means each lane's
    // first acquire load sees the stable mode: sensor polls buttons, render
    // draws the launcher, physics parks. No lifecycle work runs at boot.
    s_active_app = nullptr;
    s_mode = AppMode::Launcher;
    s_generation = 0;
    s_mode_name.store("launcher", std::memory_order_release);
    s_active_selection.store(
        pack_selection(kNoAppIndex, static_cast<uint32_t>(AppMode::Launcher), 0),
        std::memory_order_release);
    emit_mode_line("launcher");

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
             "coordinator live: %u registered app(s), launcher boot stable, "
             "PLUS/touch launch / short PWR home",
             static_cast<unsigned>(kRegistryCount));

    // Coordinator loop on the ESP main task; transitions run synchronously,
    // one at a time, in queue order. Requests whose target equals the
    // committed side are no-ops inside commit_transition.
    for (;;) {
        RuntimeRequest request;
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) != pdTRUE) {
            continue;  // unreachable: portMAX_DELAY only returns on a send
        }
        commit_transition(request);
    }
}

}  // namespace fluid_demo
