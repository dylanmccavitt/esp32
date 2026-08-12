#include "console_service.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <unistd.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.hpp"
#include "display_service.hpp"
#include "input_service.hpp"
#include "motion_service.hpp"
#include "runtime.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "console_service";
constexpr uint32_t kCaptureTimeoutMs = 2000;
constexpr size_t kBase64InputPerLine = 168;
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
                                InputService *input, ResetTrampoline reset_callback)
{
    if (display == nullptr || motion == nullptr || input == nullptr ||
        reset_callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (repl_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    display_ = display;
    motion_ = motion;
    input_ = input;
    reset_callback_ = reset_callback;
    s_active = this;

    esp_err_t err = esp_console_register_help_command();
    if (err != ESP_OK) {
        return err;
    }
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
        (err = register_command("reset", "Reset the currently running app", nullptr,
                                command_reset)) != ESP_OK ||
        (err = register_command("reboot", "Restart the firmware", nullptr,
                                command_reboot)) != ESP_OK ||
        (err = register_command("input", "Inject a debounced shell button gesture",
                                "<plus|pwr|boot> [hold_ms]", command_input)) != ESP_OK) {
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
    if (err != ESP_OK) {
        return err;
    }
    repl_ = repl;
    err = esp_console_start_repl(repl);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(kTag, "USB dev console ready: ping/status/fb/motion/release/reset/reboot/input");
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
void ConsoleService::begin_protocol_output()
{
    // The USB Serial/JTAG VFS has a 256-byte TX ring and may drop bytes after
    // 50 ms of backpressure. Park the render/telemetry lane, drain prior
    // output, then keep each bounded @DEV line wholly inside an empty ring.
    dumping_.store(true, std::memory_order_release);
    vTaskDelay(pdMS_TO_TICKS(10));
    drain_protocol_output();
}

void ConsoleService::drain_protocol_output()
{
    std::fflush(stdout);
    static_cast<void>(::fsync(STDOUT_FILENO));
}

void ConsoleService::end_protocol_output()
{
    drain_protocol_output();
    dumping_.store(false, std::memory_order_release);
}

void ConsoleService::emit_poweroff()
{
    begin_protocol_output();
    std::printf("\r\n@DEV POWEROFF\r\n");
    end_protocol_output();
}

void ConsoleService::emit_rebooting()
{
    begin_protocol_output();
    std::printf("\r\n@DEV REBOOTING\r\n");
    end_protocol_output();
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

    s_active->begin_protocol_output();
    const MotionService::OverrideSnapshot snapshot = s_active->motion_->override_snapshot();
    const uint64_t uptime_ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
    std::printf("@DEV STATUS uptime_ms=%" PRIu64
                " override=%u accel=%.3f,%.3f,%.3f capture_ready=%u"
                " battery_hold=%u mode=%s\r\n",
                uptime_ms, snapshot.active ? 1u : 0u,
                static_cast<double>(snapshot.acceleration.x),
                static_cast<double>(snapshot.acceleration.y),
                static_cast<double>(snapshot.acceleration.z),
                s_active->display_ != nullptr && s_active->display_->capture_ready() ? 1u : 0u,
                board_battery_hold_enabled() ? 1u : 0u,
                runtime_mode_name());
    s_active->end_protocol_output();
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
    const esp_err_t err = s_active->reset_callback_();
    if (err != ESP_OK) {
        std::printf("reset: no running app\r\n");
        return 1;
    }
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
    s_active->emit_rebooting();
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
    return 0;
}

int ConsoleService::command_input(int argc, char **argv)
{
    if ((argc != 2 && argc != 3) || s_active->input_ == nullptr) {
        std::printf("usage: input <plus|pwr|boot> [hold_ms]\r\n");
        return 1;
    }

    ButtonId button;
    if (std::strcmp(argv[1], "plus") == 0) {
        button = ButtonId::Plus;
    } else if (std::strcmp(argv[1], "pwr") == 0) {
        button = ButtonId::Pwr;
    } else if (std::strcmp(argv[1], "boot") == 0) {
        button = ButtonId::Boot;
    } else {
        std::printf("usage: input <plus|pwr|boot> [hold_ms]\r\n");
        return 1;
    }

    uint32_t hold_ms = InputService::kDefaultGestureHoldMs;
    if (argc == 3 &&
        (!parse_duration_ms(argv[2], &hold_ms) ||
         hold_ms < InputService::kMinimumGestureHoldMs)) {
        std::printf("input: hold_ms must be within [40,600000]\r\n");
        return 1;
    }
    const esp_err_t err = s_active->input_->enqueue_gesture(button, hold_ms);
    if (err != ESP_OK) {
        std::printf("input: gesture queue full\r\n");
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
    s_active->begin_protocol_output();
    std::printf("@FB BEGIN %" PRIu32 " %d %d RGB565BE %zu\r\n",
                sequence, DisplayService::kCaptureWidth, DisplayService::kCaptureHeight,
                DisplayService::kCaptureBytes);
    s_active->drain_protocol_output();

    char encoded[((kBase64InputPerLine + 2) / 3) * 4 + 1] = {};
    for (size_t offset = 0; offset < DisplayService::kCaptureBytes;
         offset += kBase64InputPerLine) {
        const size_t chunk =
            (DisplayService::kCaptureBytes - offset < kBase64InputPerLine)
                ? DisplayService::kCaptureBytes - offset
                : kBase64InputPerLine;
        encode_base64(data + offset, chunk, encoded);
        std::printf("@FB DATA %" PRIu32 " %s\r\n", sequence, encoded);
        // The complete worst-case wire line is 246 bytes, below the USB
        // Serial/JTAG VFS's 256-byte ring. Yield so USB can drain it before
        // the next bounded write without paying an fsync round trip per line.
        vTaskDelay(1);
    }
    std::printf("@FB END %" PRIu32 "\r\n", sequence);
    s_active->end_protocol_output();
    return 0;
}

}  // namespace fluid_demo
