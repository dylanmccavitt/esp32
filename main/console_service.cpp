// console_service.cpp — USB Serial/JTAG dev REPL (replaces dev_console.cpp).
//
// Commands, help text, @DEV/@FB wire format, the std::atomic<bool> s_dumping
// stdout dump gate, the 768-byte base64 line format and the REPL parameters
// (prompt "fluid> ", prio 2, core 0, 6144-byte stack) are preserved verbatim.
// State moved from dev_console's file statics: the override now lives in the
// shell's MotionService and the reset path uses a trampoline bound exactly
// once at start() to the Fluid app's reset atomic.

#include "console_service.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display_service.hpp"
#include "motion_service.hpp"
#include "runtime.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "console_service";
constexpr uint32_t kCaptureTimeoutMs = 2000;
constexpr size_t kBase64InputPerLine = 768;
constexpr char kBase64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

bool parse_float(const char *text, float *value)
{
    if (text == nullptr || value == nullptr) {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const float parsed = std::strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed) ||
        parsed < -18.0f || parsed > 18.0f) {
        return false;
    }
    *value = parsed;
    return true;
}

bool parse_duration_ms(const char *text, uint32_t *duration_ms)
{
    if (text == nullptr || duration_ms == nullptr || text[0] == '-') {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > 600000UL) {
        return false;
    }
    *duration_ms = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_sequence(const char *text, uint32_t *sequence)
{
    if (text == nullptr || sequence == nullptr || text[0] == '-') {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }
    *sequence = static_cast<uint32_t>(parsed);
    return true;
}

size_t encode_base64(const uint8_t *data, size_t length, char *output)
{
    size_t in = 0;
    size_t out = 0;
    while (in < length) {
        const size_t remaining = length - in;
        const uint32_t a = data[in++];
        const uint32_t b = remaining > 1 ? data[in++] : 0u;
        const uint32_t c = remaining > 2 ? data[in++] : 0u;
        const uint32_t triple = (a << 16) | (b << 8) | c;
        output[out++] = kBase64[(triple >> 18) & 0x3Fu];
        output[out++] = kBase64[(triple >> 12) & 0x3Fu];
        output[out++] = remaining > 1 ? kBase64[(triple >> 6) & 0x3Fu] : '=';
        output[out++] = remaining > 2 ? kBase64[triple & 0x3Fu] : '=';
    }
    output[out] = '\0';
    return out;
}

esp_err_t register_command(const char *name, const char *help, const char *hint,
                           esp_console_cmd_func_t function)
{
    esp_console_cmd_t command{};
    command.command = name;
    command.help = help;
    command.hint = hint;
    command.func = function;
    return esp_console_cmd_register(&command);
}

}  // namespace

ConsoleService *ConsoleService::s_active = nullptr;

esp_err_t ConsoleService::start(DisplayService *display, MotionService *motion,
                                ResetTrampoline reset_callback)
{
    if (display == nullptr || motion == nullptr || reset_callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (repl_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    display_ = display;
    motion_ = motion;
    reset_callback_ = reset_callback;
    s_active = this;

    esp_err_t err = esp_console_register_help_command();
    if (err != ESP_OK) return err;
    if ((err = register_command("ping", "Verify the firmware development link", nullptr,
                                command_ping)) != ESP_OK ||
        (err = register_command("status", "Report uptime and development-control state", nullptr,
                                command_status)) != ESP_OK ||
        (err = register_command("fb", "Capture one RGB565 framebuffer over USB",
                                "[request_id]", command_framebuffer)) != ESP_OK ||
        (err = register_command("motion", "Override box acceleration for deterministic driving",
                                "<ax> <ay> <az> [duration_ms]", command_motion)) != ESP_OK ||
        (err = register_command("release", "Return motion control to the physical IMU", nullptr,
                                command_release)) != ESP_OK ||
        (err = register_command("reset", "Reset only the in-memory fluid simulation", nullptr,
                                command_reset)) != ESP_OK ||
        (err = register_command("reboot", "Restart the firmware", nullptr,
                                command_reboot)) != ESP_OK ||
        (err = register_command("mode", "Queue a Fluid Box <-> launcher/idle transition (temporary S4 barrier exercise)", "next",
                                command_mode)) != ESP_OK) {
        return err;
    }

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "fluid> ";
    repl_config.max_cmdline_length = 128;
    repl_config.max_cmdline_args = 8;
    repl_config.task_stack_size = 6144;
    repl_config.task_priority = 2;
    repl_config.task_core_id = 0;
    esp_console_dev_usb_serial_jtag_config_t device_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_console_repl_t *repl = nullptr;
    err = esp_console_new_repl_usb_serial_jtag(&device_config, &repl_config, &repl);
    if (err != ESP_OK) return err;
    repl_ = repl;
    err = esp_console_start_repl(repl);
    if (err != ESP_OK) return err;
    ESP_LOGI(kTag, "USB dev console ready: ping/status/fb/motion/release/reset/reboot/mode");
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int ConsoleService::command_ping(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: ping\r\n");
        return 1;
    }
    std::printf("@DEV PONG\r\n");
    return 0;
}

int ConsoleService::command_status(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: status\r\n");
        return 1;
    }

    const MotionService::OverrideSnapshot snapshot = s_active->motion_->override_snapshot();
    const uint64_t uptime_ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
    std::printf("@DEV STATUS uptime_ms=%" PRIu64
                " override=%u accel=%.3f,%.3f,%.3f capture_ready=%u\r\n",
                uptime_ms, snapshot.active ? 1u : 0u,
                static_cast<double>(snapshot.acceleration.x),
                static_cast<double>(snapshot.acceleration.y),
                static_cast<double>(snapshot.acceleration.z),
                s_active->display_ != nullptr && s_active->display_->capture_ready() ? 1u : 0u);
    return 0;
}

int ConsoleService::command_motion(int argc, char **argv)
{
    if (argc != 4 && argc != 5) {
        std::printf("usage: motion <ax> <ay> <az> [duration_ms]\r\n");
        return 1;
    }
    Vec3 acceleration{};
    uint32_t duration_ms = 0;
    if (!parse_float(argv[1], &acceleration.x) ||
        !parse_float(argv[2], &acceleration.y) ||
        !parse_float(argv[3], &acceleration.z) ||
        (argc == 5 && !parse_duration_ms(argv[4], &duration_ms))) {
        std::printf("motion: finite components must be within [-18,18]; duration 0..600000 ms\r\n");
        return 1;
    }
    s_active->motion_->set_override(acceleration, duration_ms);
    std::printf("@DEV MOTION %.3f %.3f %.3f duration_ms=%" PRIu32 "\r\n",
                static_cast<double>(acceleration.x), static_cast<double>(acceleration.y),
                static_cast<double>(acceleration.z), duration_ms);
    return 0;
}

int ConsoleService::command_release(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: release\r\n");
        return 1;
    }
    s_active->motion_->clear_override();
    std::printf("@DEV MOTION_RELEASED\r\n");
    return 0;
}

int ConsoleService::command_reset(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1 || s_active->reset_callback_ == nullptr) {
        std::printf("usage: reset\r\n");
        return 1;
    }
    s_active->reset_callback_();
    std::printf("@DEV RESET_REQUESTED\r\n");
    return 0;
}

int ConsoleService::command_reboot(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: reboot\r\n");
        return 1;
    }
    std::printf("@DEV REBOOTING\r\n");
    std::fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
    return 0;
}

int ConsoleService::command_mode(int argc, char **argv)
{
    if (argc != 2 || std::strcmp(argv[1], "next") != 0) {
        std::printf("usage: mode next\r\n");
        return 1;
    }
    // Queue only, never perform synchronously: the coordinator on the main
    // task runs the full barrier (quiesce -> 3-lane acks -> DisplayService
    // drain -> leave/enter -> run publish) and emits the committed
    // "@DEV MODE fluid_box|launcher" line itself.
    const esp_err_t err = runtime_enqueue_request(RuntimeRequest::NextMode);
    if (err != ESP_OK) {
        std::printf("mode: transition queue full\r\n");
        return 1;
    }
    return 0;
}

int ConsoleService::command_framebuffer(int argc, char **argv)
{
    if ((argc != 1 && argc != 2) || s_active->display_ == nullptr) {
        std::printf("usage: fb [request_id]\r\n");
        return 1;
    }
    uint32_t sequence = 0;
    if (argc == 2) {
        if (!parse_sequence(argv[1], &sequence)) {
            std::printf("usage: fb [request_id]\r\n");
            return 1;
        }
    } else {
        sequence = ++s_active->capture_sequence_;
    }
    const esp_err_t request = s_active->display_->request_capture();
    if (request != ESP_OK) {
        std::printf("@FB ERROR %" PRIu32 " capture request failed: %s\r\n",
                    sequence, esp_err_to_name(request));
        return 1;
    }

    const int64_t deadline =
        esp_timer_get_time() + static_cast<int64_t>(kCaptureTimeoutMs) * 1000;
    while (!s_active->display_->capture_ready() && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_active->display_->capture_ready()) {
        std::printf("@FB ERROR %" PRIu32 " capture timed out\r\n", sequence);
        return 1;
    }

    const uint8_t *data = s_active->display_->capture_data();
    s_active->dumping_.store(true, std::memory_order_release);
    // The render task is CPU-bound on core 0. Give it one scheduling window
    // to observe s_dumping and park before this lower-priority REPL streams.
    vTaskDelay(pdMS_TO_TICKS(10));
    std::printf("@FB BEGIN %" PRIu32 " %d %d RGB565BE %zu\r\n",
                sequence, DisplayService::kCaptureWidth, DisplayService::kCaptureHeight,
                DisplayService::kCaptureBytes);

    char encoded[((kBase64InputPerLine + 2) / 3) * 4 + 1] = {};
    for (size_t offset = 0; offset < DisplayService::kCaptureBytes;
         offset += kBase64InputPerLine) {
        const size_t chunk =
            (DisplayService::kCaptureBytes - offset < kBase64InputPerLine)
                ? DisplayService::kCaptureBytes - offset
                : kBase64InputPerLine;
        encode_base64(data + offset, chunk, encoded);
        std::printf("@FB DATA %" PRIu32 " %s\r\n", sequence, encoded);
        // Keep IDLE0 runnable throughout a full framebuffer dump.
        vTaskDelay(1);
    }
    std::printf("@FB END %" PRIu32 "\r\n", sequence);
    std::fflush(stdout);
    s_active->dumping_.store(false, std::memory_order_release);
    return 0;
}

}  // namespace fluid_demo
