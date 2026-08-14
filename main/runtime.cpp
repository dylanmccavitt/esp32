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

#include "attitude_app.hpp"
#include "board.hpp"
#include "console_service.hpp"
#include "display_service.hpp"
#include "fluid_app.hpp"
#include "input_service.hpp"
#include "launcher.hpp"
#include "motion_service.hpp"
#include "orient_cube_app.hpp"
#include "ragdoll_avalanche_app.hpp"
#include "tilt_maze_app.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "fluid_demo";

struct RegistryEntry {
    const char *id;
    const char *label;
    App *app;
};

constexpr uint32_t kAppIndexMask = 0x0Fu;
constexpr uint32_t kAppModeShift = 4;
constexpr uint32_t kAppModeMask = 0x3u << kAppModeShift;
constexpr uint32_t kAppGenerationShift = kAppModeShift + 2;
constexpr uint32_t kNoAppIndex = 0x0Fu;

constexpr RegistryEntry kRegistry[] = {
    {"fluid_box", "Fluid Box", &s_fluid_app},
    {"tilt_maze", "Task Maze", &s_tilt_maze_app},
    {"ragdoll_avalanche", "Avalanche", &s_ragdoll_avalanche_app},
    {"orient_cube", "Cube", &s_orient_cube_app},
    {"attitude", "Level", &s_attitude_app},
};
constexpr uint32_t kRegistryCount = sizeof(kRegistry) / sizeof(kRegistry[0]);

static_assert(kRegistryCount <= kNoAppIndex);

constexpr uint32_t kAppGenerationMask = ~(kAppIndexMask | kAppModeMask);

uint32_t pack_selection(uint32_t index, AppMode mode, uint32_t generation)
{
    return ((generation << kAppGenerationShift) & kAppGenerationMask) |
           ((static_cast<uint32_t>(mode) << kAppModeShift) & kAppModeMask) |
           (index & kAppIndexMask);
}

App *app_at_index(uint32_t index)
{
    return index < kRegistryCount ? kRegistry[index].app : nullptr;
}

AppMode selection_mode(uint32_t selection)
{
    return static_cast<AppMode>((selection & kAppModeMask) >> kAppModeShift);
}

static_assert(static_cast<uint32_t>(AppMode::Transition) < (1u << 2));

// Loaded once per lane iteration to keep app, mode, and generation coherent.
std::atomic<uint32_t> s_active_selection{
    pack_selection(kNoAppIndex, AppMode::Launcher, 0)};
portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
AppStats s_stats_snapshot{};
uint32_t s_stats_selection = pack_selection(kNoAppIndex, AppMode::Launcher, 0);

void publish_active_stats(uint32_t selection, const AppStats &stats)
{
    portENTER_CRITICAL(&s_stats_mux);
    s_stats_snapshot = stats;
    s_stats_selection = selection;
    portEXIT_CRITICAL(&s_stats_mux);
}

std::atomic<const char *> s_active_name{"launcher"};

std::atomic<uint32_t> s_launcher_index{0};

// Each lane sets its bit after its last old-generation callback.
constexpr uint32_t kSensorAcknowledgement = 1u << 0;
constexpr uint32_t kUpdateAcknowledgement = 1u << 1;
constexpr uint32_t kRenderAcknowledgement = 1u << 2;
constexpr uint32_t kAllLaneAcknowledgements =
    kSensorAcknowledgement | kUpdateAcknowledgement | kRenderAcknowledgement;
std::atomic<uint32_t> s_lane_acknowledgements{0};

AppMode s_mode = AppMode::Running;
const RegistryEntry *s_active_entry = nullptr;
uint32_t s_generation = 0;

constexpr uint32_t kQuiesceTimeoutMs = 500;

enum class RuntimeRequestKind : uint8_t {
    Launch = 0,
    Home = 1,
};

struct RuntimeRequest {
    RuntimeRequestKind kind;
    uint32_t app_index;
};

QueueHandle_t s_request_queue = nullptr;

RuntimeRequest selected_launch_request()
{
    return {RuntimeRequestKind::Launch,
            s_launcher_index.load(std::memory_order_acquire)};
}

constexpr RuntimeRequest home_request()
{
    return {RuntimeRequestKind::Home, kNoAppIndex};
}

esp_err_t enqueue_request(const RuntimeRequest &request)
{
    const QueueHandle_t queue = s_request_queue;
    if (queue == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueSend(queue, &request, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

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

void frame_latch_capture(void *transport)
{
    static_cast<DisplayService *>(transport)->latch_capture();
}

esp_err_t frame_submit(void *transport, int stripe_index, int stripe_y,
                       int stripe_rows, const uint16_t *pixels)
{
    return static_cast<DisplayService *>(transport)->submit_stripe(
        stripe_index, stripe_y, stripe_rows, pixels);
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

esp_err_t app_reset_trampoline()
{
    const uint32_t selection =
        s_active_selection.load(std::memory_order_acquire);
    if (selection_mode(selection) != AppMode::Running) {
        return ESP_ERR_INVALID_STATE;
    }
    App *app = app_at_index(selection & kAppIndexMask);
    if (app == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    app->on_plus_press();
    return ESP_OK;
}

void emit_poweroff_marker()
{
    s_console.emit_poweroff();
}

void emit_reboot_marker()
{
    s_console.emit_rebooting();
}

uint32_t s_last_frame_dma_us = 0;

std::atomic<TaskHandle_t> s_coordinator_task{nullptr};
std::atomic<TaskHandle_t> s_sensor_task{nullptr};
std::atomic<TaskHandle_t> s_update_task{nullptr};
std::atomic<TaskHandle_t> s_render_task{nullptr};
std::atomic<TaskHandle_t> s_console_task{nullptr};

void fill_task_telemetry(SystemTaskTelemetry &telemetry, SystemTaskKind kind,
                         std::atomic<TaskHandle_t> &task_handle)
{
    telemetry.kind = kind;
    const TaskHandle_t task = task_handle.load(std::memory_order_acquire);
    if (task == nullptr) {
        telemetry.state = SystemTaskState::Unknown;
        telemetry.core_id = -1;
        telemetry.stack_high_water_words = 0;
        telemetry.available = false;
        return;
    }

    telemetry.available = true;
    switch (eTaskGetState(task)) {
    case eRunning:
        telemetry.state = SystemTaskState::Running;
        break;
    case eReady:
        telemetry.state = SystemTaskState::Ready;
        break;
    case eBlocked:
        telemetry.state = SystemTaskState::Blocked;
        break;
    case eSuspended:
        telemetry.state = SystemTaskState::Suspended;
        break;
    default:
        telemetry.state = SystemTaskState::Unknown;
        break;
    }
    const BaseType_t core = xTaskGetCoreID(task);
    telemetry.core_id = core == tskNO_AFFINITY ? -1 : static_cast<int8_t>(core);
    telemetry.stack_high_water_words =
        static_cast<uint32_t>(uxTaskGetStackHighWaterMark2(task));
}

void sample_system_telemetry(App *app, uint32_t generation)
{
    SystemTelemetry telemetry{};
    telemetry.generation = generation;

    multi_heap_info_t heap_info{};
    heap_caps_get_info(&heap_info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    telemetry.internal_free_bytes =
        static_cast<uint32_t>(heap_info.total_free_bytes);
    telemetry.internal_largest_free_block =
        static_cast<uint32_t>(heap_info.largest_free_block);

    telemetry.psram_free_bytes = static_cast<uint32_t>(
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    fill_task_telemetry(telemetry.tasks[0], SystemTaskKind::Coordinator,
                        s_coordinator_task);
    fill_task_telemetry(telemetry.tasks[1], SystemTaskKind::Sensor,
                        s_sensor_task);
    fill_task_telemetry(telemetry.tasks[2], SystemTaskKind::Update,
                        s_update_task);
    fill_task_telemetry(telemetry.tasks[3], SystemTaskKind::Render,
                        s_render_task);
    fill_task_telemetry(telemetry.tasks[4], SystemTaskKind::Console,
                        s_console_task);

    app->on_system_telemetry(telemetry);
}

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
             static_cast<unsigned>(psram_mb),
             static_cast<unsigned>(configTICK_RATE_HZ));
    ESP_LOGI(
        kTag, "free heap %u B internal, %u B PSRAM",
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
        static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
}

[[noreturn]] void fatal_startup(const char *operation, esp_err_t error)
{
    ESP_LOGE(kTag, "FATAL: %s failed: %s - entering recovery idle", operation,
             esp_err_to_name(error));
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

[[noreturn]] void fail_closed(const char *reason)
{
    ESP_LOGE(kTag, "TRANSITION BARRIER: %s - fail-closed esp_restart()",
             reason);
    esp_restart();
}

constexpr uint32_t kSensorHz = 100;
constexpr uint32_t kPhysicsHz = 30;
constexpr uint32_t kRenderHz = 30;

constexpr TickType_t kSensorPeriodTicks = static_cast<TickType_t>(
    (1000000ULL / kSensorHz * configTICK_RATE_HZ + 999999ULL) / 1000000ULL);
constexpr TickType_t kPhysicsPeriodTicks = static_cast<TickType_t>(
    (1000000ULL / kPhysicsHz * configTICK_RATE_HZ + 999999ULL) / 1000000ULL);
constexpr TickType_t kRenderPeriodTicks = static_cast<TickType_t>(
    (1000000ULL / kRenderHz * configTICK_RATE_HZ + 999999ULL) / 1000000ULL);

constexpr uint32_t kSensorStackBytes = 4096;
constexpr uint32_t kPhysicsStackBytes = 4096;
constexpr uint32_t kRenderStackBytes = 4096;

// Return one coherent selection and acknowledge each generation once.
uint32_t synchronize_lane_generation(uint32_t &observed_generation,
                                     TickType_t &cadence_anchor,
                                     uint32_t acknowledgement_bit)
{
    const uint32_t selection =
        s_active_selection.load(std::memory_order_acquire);
    const uint32_t generation = selection >> kAppGenerationShift;
    if (generation != observed_generation) {
        const AppMode mode = selection_mode(selection);
        if (mode == AppMode::Transition) {
            s_lane_acknowledgements.fetch_or(acknowledgement_bit,
                                             std::memory_order_release);
        } else if (mode == AppMode::Running) {
            cadence_anchor = xTaskGetTickCount();
        }
        observed_generation = generation;
    }
    return selection;
}

void sensor_task(void *)
{
    s_sensor_task.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    uint32_t observed_generation = UINT32_MAX;
    uint32_t last_motion_error_log_second = UINT32_MAX;
    uint32_t last_touch_error_log_second = UINT32_MAX;
    RuntimeRequest pending_request{RuntimeRequestKind::Launch, 0};
    bool has_pending_request = false;
    auto enqueue_or_replace_pending = [&](RuntimeRequest request) {
        if (has_pending_request) {
            pending_request = request;
            return;
        }
        if (enqueue_request(request) != ESP_OK) {
            pending_request = request;
            has_pending_request = true;
        }
    };
    bool launcher_contact_active = false;
    uint16_t launcher_contact_start_x = 0;
    uint16_t launcher_contact_start_y = 0;
    TickType_t cadence_anchor = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&cadence_anchor, kSensorPeriodTicks);
        const uint32_t selection = synchronize_lane_generation(
            observed_generation, cadence_anchor, kSensorAcknowledgement);
        const AppMode mode = selection_mode(selection);
        if (mode != AppMode::Launcher) {
            launcher_contact_active = false;
        }
        App *app = app_at_index(selection & kAppIndexMask);
        if (has_pending_request && enqueue_request(pending_request) == ESP_OK) {
            has_pending_request = false;
        }

        if (mode == AppMode::Running && app != nullptr) {
            const MotionTick motion_tick = s_motion.poll();
            const esp_err_t motion_error = s_motion.last_read_error();
            if (motion_error != ESP_OK) {
                const uint32_t current_second =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
                if (current_second != last_motion_error_log_second &&
                    !s_console.protocol_output_active()) {
                    last_motion_error_log_second = current_second;
                    ESP_LOGW(kTag, "board_read_motion failed: %s",
                             esp_err_to_name(motion_error));
                }
            }
            s_motion.acknowledge_sample(app->on_motion(motion_tick));
        }

        TouchEvent touch_event;
        if (s_input.poll_touch(touch_event)) {
            if (mode == AppMode::Running && app != nullptr) {
                if (touch_event.phase == TouchPhase::Begin) {
                    app->on_touch_begin(touch_event);
                }
            } else if (mode == AppMode::Launcher) {
                if (touch_event.phase == TouchPhase::Begin) {
                    launcher_contact_active = true;
                    launcher_contact_start_x = touch_event.x;
                    launcher_contact_start_y = touch_event.y;
                } else if (touch_event.phase == TouchPhase::End) {
                    if (launcher_contact_active) {
                        launcher_contact_active = false;
                        const TouchGesture gesture = launcher_swipe_gesture(
                            touch_event.gesture, launcher_contact_start_x,
                            launcher_contact_start_y, touch_event.x,
                            touch_event.y);
                        if (gesture == TouchGesture::SwipeLeft) {
                            const uint32_t next_index =
                                (s_launcher_index.load(
                                     std::memory_order_relaxed) +
                                 1) %
                                kRegistryCount;
                            s_launcher_index.store(next_index,
                                                   std::memory_order_release);
                            ESP_LOGI(kTag,
                                     "swipe left - launcher selection %u: %s",
                                     static_cast<unsigned>(next_index),
                                     kRegistry[next_index].label);
                        } else if (gesture == TouchGesture::SwipeRight) {
                            const uint32_t previous_index =
                                (s_launcher_index.load(
                                     std::memory_order_relaxed) +
                                 kRegistryCount - 1u) %
                                kRegistryCount;
                            s_launcher_index.store(previous_index,
                                                   std::memory_order_release);
                            ESP_LOGI(kTag,
                                     "swipe right - launcher selection %u: %s",
                                     static_cast<unsigned>(previous_index),
                                     kRegistry[previous_index].label);
                        } else if (launcher_accepts_launch_touch(
                                       touch_event.x, touch_event.y)) {
                            ESP_LOGI(kTag, "touch launch x=%u y=%u",
                                     static_cast<unsigned>(touch_event.x),
                                     static_cast<unsigned>(touch_event.y));
                            enqueue_or_replace_pending(
                                selected_launch_request());
                        }
                    }
                }
            }
        } else {
            const esp_err_t touch_error = s_input.last_touch_error();
            if (touch_error != ESP_OK) {
                launcher_contact_active = false;
                const uint32_t current_second =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
                if (current_second != last_touch_error_log_second &&
                    !s_console.protocol_output_active()) {
                    last_touch_error_log_second = current_second;
                    ESP_LOGW(kTag, "board_read_touch failed: %s",
                             esp_err_to_name(touch_error));
                }
            }
        }
        const uint32_t current_milliseconds =
            static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
        ButtonEvent button_event;
        const bool have_button_event =
            s_input.poll(current_milliseconds, button_event);
        if (have_button_event) {
            if (button_event == ButtonEvent::PlusPress) {
                if (mode == AppMode::Running && app != nullptr) {
                    app->on_plus_press();
                } else if (mode == AppMode::Launcher) {
                    enqueue_or_replace_pending(selected_launch_request());
                }
            } else if (button_event == ButtonEvent::PowerShort) {
                if (mode == AppMode::Running) {
                    enqueue_or_replace_pending(home_request());
                }
            }
        }
        // Always yield for idle and watchdog service after overruns.
        vTaskDelay(1);
    }
}

void physics_task(void *)
{
    s_update_task.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    uint32_t observed_generation = UINT32_MAX;
    TickType_t cadence_anchor = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&cadence_anchor, kPhysicsPeriodTicks);
        const uint32_t selection = synchronize_lane_generation(
            observed_generation, cadence_anchor, kUpdateAcknowledgement);
        if (selection_mode(selection) != AppMode::Running) {
            vTaskDelay(1);
            continue;
        }
        App *app = app_at_index(selection & kAppIndexMask);
        if (app == nullptr) {
            vTaskDelay(1);
            continue;
        }

        static_cast<void>(app->update(fluid_demo::App::kPhysicsDt));

        vTaskDelay(1);
    }
}

void log_telemetry(App *app, uint32_t app_index)
{
    const AppStats stats = app->stats();
    const uint64_t current_checks = stats.candidate_checks;
    static uint64_t last_candidate_checks[kRegistryCount] = {};
    uint64_t &last_checks = last_candidate_checks[app_index];
    const uint64_t candidate_delta = current_checks >= last_checks
                                         ? current_checks - last_checks
                                         : current_checks;
    last_checks = current_checks;
    ESP_LOGI(
        kTag,
        "count=%u epoch=%u phys=%uus raster=%uus dma=%uus frame=%uus "
        "cand/s=%llu raw=(%.2f,%.2f,%.2f) sim=(%.2f,%.2f,%.2f) "
        "heap_int_min=%u heap_psram_min=%u missed=%u nonfinite=%u "
        "governed=%u",
        static_cast<unsigned>(stats.count), static_cast<unsigned>(stats.epoch),
        static_cast<unsigned>(stats.physics_us),
        static_cast<unsigned>(stats.raster_us),
        static_cast<unsigned>(s_last_frame_dma_us),
        static_cast<unsigned>(stats.frame_us),
        static_cast<unsigned long long>(candidate_delta),
        static_cast<double>(stats.raw[0]), static_cast<double>(stats.raw[1]),
        static_cast<double>(stats.raw[2]),
        static_cast<double>(stats.apparent[0]),
        static_cast<double>(stats.apparent[1]),
        static_cast<double>(stats.apparent[2]),
        static_cast<unsigned>(
            heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
        static_cast<unsigned>(
            heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM)),
        static_cast<unsigned>(s_display.missed_transfers()),
        static_cast<unsigned>(stats.nonfinite_resets),
        static_cast<unsigned>(stats.governor_hits));
}

void render_task(void *)
{
    s_render_task.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    uint32_t observed_generation = UINT32_MAX;
    uint32_t last_log_second = 0;
    TickType_t cadence_anchor = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&cadence_anchor, kRenderPeriodTicks);
        const uint32_t selection = synchronize_lane_generation(
            observed_generation, cadence_anchor, kRenderAcknowledgement);
        if (selection_mode(selection) == AppMode::Transition) {
            // The coordinator drains after every lane acknowledges.
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (selection_mode(selection) == AppMode::Launcher) {
            if (s_console.protocol_output_active()) {
                vTaskDelay(pdMS_TO_TICKS(10));
                cadence_anchor = xTaskGetTickCount();
                continue;
            }
            const uint32_t selected_index =
                s_launcher_index.load(std::memory_order_acquire);
            App *const app = app_at_index(selected_index);
            static_cast<void>(render_launcher(
                s_display_frame,
                app != nullptr ? app->launcher_visual() : nullptr,
                selected_index, kRegistryCount));
            vTaskDelay(1);
            continue;
        }
        if (s_console.protocol_output_active()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            cadence_anchor = xTaskGetTickCount();
            continue;
        }

        const uint32_t app_index = selection & kAppIndexMask;
        App *app = app_at_index(app_index);
        if (app == nullptr) {
            vTaskDelay(1);
            continue;
        }

        const uint32_t dma_wait_start_us = s_display.dma_wait_us();
        if (app->render(s_display_frame)) {
            s_last_frame_dma_us = s_display.dma_wait_us() - dma_wait_start_us;
            publish_active_stats(selection, app->stats());
        }

        const uint32_t current_second =
            static_cast<uint32_t>(esp_timer_get_time() / 1000000ULL);
        if (current_second != last_log_second &&
            !s_console.protocol_output_active()) {
            last_log_second = current_second;
            sample_system_telemetry(app, selection >> kAppGenerationShift);
            log_telemetry(app, app_index);
        }
        vTaskDelay(1);
    }
}

void emit_mode_line(const char *mode_name)
{
    std::printf("\r\n@DEV MODE %s\r\n", mode_name);
    std::fflush(stdout);
}

void commit_transition(RuntimeRequest request)
{
    const RegistryEntry *outgoing_entry = s_active_entry;
    const RegistryEntry *incoming_entry = nullptr;
    uint32_t incoming_index = kNoAppIndex;
    switch (request.kind) {
    case RuntimeRequestKind::Launch:
        if (s_mode != AppMode::Launcher) {
            return;
        }
        incoming_index = request.app_index;
        if (incoming_index >= kRegistryCount) {
            incoming_index = 0;
        }
        incoming_entry = &kRegistry[incoming_index];
        break;
    case RuntimeRequestKind::Home:
        if (s_mode != AppMode::Running) {
            return;
        }
        break;
    default:
        return;
    }

    s_mode = AppMode::Transition;
    s_lane_acknowledgements.store(0, std::memory_order_release);
    const uint32_t quiesce_generation = ++s_generation;
    s_active_selection.store(
        pack_selection(kNoAppIndex, AppMode::Transition, quiesce_generation),
        std::memory_order_release);

    const int64_t quiesce_deadline_us =
        esp_timer_get_time() + static_cast<int64_t>(kQuiesceTimeoutMs) * 1000;
    while ((s_lane_acknowledgements.load(std::memory_order_acquire) &
            kAllLaneAcknowledgements) != kAllLaneAcknowledgements) {
        if (esp_timer_get_time() >= quiesce_deadline_us) {
            fail_closed("quiesce ack timeout");
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (s_display.drain() != ESP_OK) {
        fail_closed("display drain failed");
    }

    if (outgoing_entry != nullptr) {
        outgoing_entry->app->leave();
    }

    s_mode = AppMode::Entering;
    s_active_entry = incoming_entry;
    if (incoming_entry != nullptr) {
        const esp_err_t enter_result = incoming_entry->app->enter();
        if (enter_result != ESP_OK) {
            fail_closed("app enter failed");
        }
    }
    s_mode = incoming_entry != nullptr ? AppMode::Running : AppMode::Launcher;

    const uint32_t active_generation = ++s_generation;
    s_active_selection.store(
        pack_selection(incoming_index, s_mode, active_generation),
        std::memory_order_release);

    const char *active_name =
        incoming_entry != nullptr ? incoming_entry->id : "launcher";
    s_active_name.store(active_name, std::memory_order_release);
    emit_mode_line(active_name);

    ESP_LOGV(kTag, "transition committed: mode=%u generation=%u",
             static_cast<unsigned>(s_mode),
             static_cast<unsigned>(s_generation));
}

}

const char *runtime_mode_name()
{
    return s_active_name.load(std::memory_order_acquire);
}

bool runtime_active_stats(AppStats &stats)
{
    const uint32_t selection_before =
        s_active_selection.load(std::memory_order_acquire);
    AppStats published_stats{};
    bool stats_available = false;
    if (selection_mode(selection_before) == AppMode::Running) {
        portENTER_CRITICAL(&s_stats_mux);
        if (s_stats_selection == selection_before) {
            published_stats = s_stats_snapshot;
            stats_available = true;
        }
        portEXIT_CRITICAL(&s_stats_mux);
    }
    const uint32_t selection_after =
        s_active_selection.load(std::memory_order_acquire);
    if (!stats_available || selection_after != selection_before) {
        stats = {};
        return false;
    }
    stats = published_stats;
    return true;
}

[[noreturn]] void runtime_run()
{
    s_request_queue = xQueueCreate(8, sizeof(RuntimeRequest));
    if (s_request_queue == nullptr) {
        fatal_startup("request queue create", ESP_ERR_NO_MEM);
    }

    s_coordinator_task.store(xTaskGetCurrentTaskHandle(),
                             std::memory_order_release);

    log_startup_info();

    ESP_LOGI(kTag, "board_init (ESP32-S3-Touch-LCD-1.54)");
    esp_err_t startup_result = board_init(s_board);
    if (startup_result != ESP_OK) {
        fatal_startup("board init", startup_result);
    }

    ESP_LOGI(kTag, "display init");
    startup_result = s_display.init(s_board.panel, s_board.io);
    if (startup_result != ESP_OK) {
        fatal_startup("display init", startup_result);
    }
    bind_display_frame();

    for (uint32_t app_index = 0; app_index < kRegistryCount; ++app_index) {
        ESP_LOGI(kTag, "app init: %s", kRegistry[app_index].id);
        startup_result = kRegistry[app_index].app->setup_once();
        if (startup_result != ESP_OK) {
            fatal_startup("app init", startup_result);
        }
    }

    startup_result =
        s_console.start(s_display, s_motion, s_input, app_reset_trampoline);
    if (startup_result != ESP_OK) {
        fatal_startup("console service init", startup_result);
    }
    s_console_task.store(xTaskGetHandle("console_repl"),
                         std::memory_order_release);

    s_input.set_reboot_marker(&emit_reboot_marker);
    s_input.set_power_off_marker(&emit_poweroff_marker);

    s_active_entry = nullptr;
    s_mode = AppMode::Launcher;
    s_generation = 0;
    s_active_name.store("launcher", std::memory_order_release);
    s_active_selection.store(pack_selection(kNoAppIndex, AppMode::Launcher, 0),
                             std::memory_order_release);
    emit_mode_line("launcher");

    BaseType_t task_creation_result = xTaskCreatePinnedToCore(
        sensor_task, "sensor", kSensorStackBytes, nullptr, 7, nullptr, 0);
    if (task_creation_result != pdPASS) {
        fatal_startup("sensor task create", ESP_ERR_NO_MEM);
    }
    task_creation_result = xTaskCreatePinnedToCore(
        physics_task, "physics", kPhysicsStackBytes, nullptr, 8, nullptr, 1);
    if (task_creation_result != pdPASS) {
        fatal_startup("physics task create", ESP_ERR_NO_MEM);
    }
    task_creation_result = xTaskCreatePinnedToCore(
        render_task, "render", kRenderStackBytes, nullptr, 5, nullptr, 0);
    if (task_creation_result != pdPASS) {
        fatal_startup("render task create", ESP_ERR_NO_MEM);
    }

    ESP_LOGI(kTag,
             "coordinator live: %u registered app(s), launcher boot stable, "
             "PLUS/tap launch selected, swipe cycle, short PWR home",
             static_cast<unsigned>(kRegistryCount));

    for (;;) {
        RuntimeRequest request;
        if (xQueueReceive(s_request_queue, &request, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        commit_transition(request);
    }
}

}
