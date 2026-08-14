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

#include "attitude.hpp"
#include "board.hpp"
#include "display_service.hpp"
#include "input_service.hpp"
#include "motion_service.hpp"
#include "runtime.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "console_service";
constexpr uint32_t kCaptureTimeoutMs = 2000;
constexpr size_t kBase64InputBytesPerLine = 168;
constexpr uint32_t kMaximumDurationMs = 600000;
constexpr char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
constexpr char kInputUsage[] = "usage: input "
                               "<plus|pwr|boot|swipe-left|swipe-right> "
                               "[hold_ms]\r\n";

bool parse_float(const char *text, float minimum, float maximum, float &value)
{
    if (text == nullptr) {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const float parsed = std::strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed) ||
        parsed < minimum || parsed > maximum) {
        return false;
    }
    value = parsed;
    return true;
}

bool parse_sign(const char *text, int &value)
{
    if (text == nullptr) {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        (parsed != 1 && parsed != -1)) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool parse_duration_ms(const char *text, uint32_t &duration_ms)
{
    if (text == nullptr || text[0] == '-') {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        parsed > kMaximumDurationMs) {
        return false;
    }
    duration_ms = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_sequence(const char *text, uint32_t &sequence)
{
    if (text == nullptr || text[0] == '-') {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }
    sequence = static_cast<uint32_t>(parsed);
    return true;
}

void encode_base64(const uint8_t *data, size_t length, char *output)
{
    size_t input_offset = 0;
    size_t output_offset = 0;
    while (input_offset < length) {
        const size_t remaining = length - input_offset;
        const uint32_t first_byte = data[input_offset++];
        const uint32_t second_byte = remaining > 1 ? data[input_offset++] : 0u;
        const uint32_t third_byte = remaining > 2 ? data[input_offset++] : 0u;
        const uint32_t triple =
            (first_byte << 16) | (second_byte << 8) | third_byte;
        output[output_offset++] = kBase64Alphabet[(triple >> 18) & 0x3Fu];
        output[output_offset++] = kBase64Alphabet[(triple >> 12) & 0x3Fu];
        output[output_offset++] =
            remaining > 1 ? kBase64Alphabet[(triple >> 6) & 0x3Fu] : '=';
        output[output_offset++] =
            remaining > 2 ? kBase64Alphabet[triple & 0x3Fu] : '=';
    }
    output[output_offset] = '\0';
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

}

ConsoleService *ConsoleService::s_active = nullptr;

esp_err_t ConsoleService::start(DisplayService &display, MotionService &motion,
                                InputService &input,
                                ResetCallback reset_callback)
{
    if (reset_callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (repl_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    display_ = &display;
    motion_ = &motion;
    input_ = &input;
    reset_callback_ = reset_callback;
    s_active = this;

    esp_err_t result = esp_console_register_help_command();
    if (result != ESP_OK) {
        return result;
    }

    struct CommandSpec {
        const char *name;
        const char *help;
        const char *hint;
        esp_console_cmd_func_t function;
    };
    const CommandSpec command_specs[] = {
        {"ping", "Verify the firmware development link", nullptr, command_ping},
        {"status", "Report uptime and development-control state", nullptr,
         command_status},
        {"fb", "Capture one RGB565 framebuffer over USB", "[request_id]",
         command_framebuffer},
        {"motion", "Override box acceleration for deterministic driving",
         "<ax> <ay> <az> [duration_ms]", command_motion},
        {"release", "Return motion control to the physical IMU", nullptr,
         command_release},
        {"reset", "Reset the currently running app", nullptr, command_reset},
        {"reboot", "Restart the firmware", nullptr, command_reboot},
        {"input", "Inject a debounced shell button or launcher swipe",
         "<plus|pwr|boot|swipe-left|swipe-right> [hold_ms]", command_input},
        {"yaw", "Apply a one-shot body yaw to the attitude filter", "<radians>",
         command_yaw},
        {"axes", "Set IMU axis signs for Cube/Level (each ±1)",
         "<sx> <sy> <sz>", command_axes},
        {"gain", "Scale Cube/Level relative rotation", "<scale>", command_gain},
        {"tau", "Set Cube/Level complementary-filter time constant",
         "<seconds>", command_tau},
    };
    for (const CommandSpec &spec : command_specs) {
        result =
            register_command(spec.name, spec.help, spec.hint, spec.function);
        if (result != ESP_OK) {
            return result;
        }
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
    result = esp_console_new_repl_usb_serial_jtag(&device_config, &repl_config,
                                                  &repl);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_console_start_repl(repl);
    if (result != ESP_OK) {
        return result;
    }
    repl_handle_ = repl;
    ESP_LOGI(
        kTag,
        "USB dev console ready: "
        "ping/status/fb/motion/release/reset/reboot/input/yaw/axes/gain/tau");
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

void ConsoleService::begin_protocol_output()
{
    // USB/JTAG drops bytes under backpressure; serialize protocol output.
    protocol_output_active_.store(true, std::memory_order_release);
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
    protocol_output_active_.store(false, std::memory_order_release);
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
    const MotionService::OverrideSnapshot override_snapshot =
        s_active->motion_->override_snapshot();
    AppStats stats{};
    static_cast<void>(runtime_active_stats(stats));
    const uint64_t uptime_ms =
        static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
    int axis_sign_x = 0;
    int axis_sign_y = 0;
    int axis_sign_z = 0;
    AttitudeFilter::axes(&axis_sign_x, &axis_sign_y, &axis_sign_z);
    std::printf(
        "@DEV STATUS uptime_ms=%" PRIu64 " override=%u accel=%.3f,%.3f,%.3f"
        " capture_ready=%u battery_hold=%u mode=%s"
        " raw=%.3f,%.3f,%.3f"
        " apparent=%.3f,%.3f,%.3f"
        " euler=%.3f,%.3f,%.3f"
        " axes=%d,%d,%d gain=%.3f tau=%.3f\r\n",
        uptime_ms, override_snapshot.active ? 1u : 0u,
        static_cast<double>(override_snapshot.acceleration.x),
        static_cast<double>(override_snapshot.acceleration.y),
        static_cast<double>(override_snapshot.acceleration.z),
        s_active->display_->capture_ready() ? 1u : 0u,
        board_battery_hold_enabled() ? 1u : 0u, runtime_mode_name(),
        static_cast<double>(stats.raw[0]), static_cast<double>(stats.raw[1]),
        static_cast<double>(stats.raw[2]),
        static_cast<double>(stats.apparent[0]),
        static_cast<double>(stats.apparent[1]),
        static_cast<double>(stats.apparent[2]),
        static_cast<double>(stats.pitch), static_cast<double>(stats.roll),
        static_cast<double>(stats.yaw), axis_sign_x, axis_sign_y, axis_sign_z,
        static_cast<double>(AttitudeFilter::gain()),
        static_cast<double>(AttitudeFilter::tau()));
    s_active->end_protocol_output();
    return 0;
}

int ConsoleService::command_motion(int argc, char **argv)
{
    if (argc != 4 && argc != 5) {
        std::printf("usage: motion "
                    "<ax> <ay> <az> [duration_ms]\r\n");
        return 1;
    }
    Vec3 acceleration{};
    uint32_t duration_ms = 0;
    if (!parse_float(argv[1], -18.0f, 18.0f, acceleration.x) ||
        !parse_float(argv[2], -18.0f, 18.0f, acceleration.y) ||
        !parse_float(argv[3], -18.0f, 18.0f, acceleration.z) ||
        (argc == 5 && !parse_duration_ms(argv[4], duration_ms))) {
        std::printf("motion: finite components must be within "
                    "[-18,18]; duration 0..600000 ms\r\n");
        return 1;
    }
    s_active->motion_->set_override(acceleration, duration_ms);
    std::printf("@DEV MOTION %.3f %.3f %.3f"
                " duration_ms=%" PRIu32 "\r\n",
                static_cast<double>(acceleration.x),
                static_cast<double>(acceleration.y),
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
    if (argc != 1) {
        std::printf("usage: reset\r\n");
        return 1;
    }
    const esp_err_t reset_result = s_active->reset_callback_();
    if (reset_result != ESP_OK) {
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
    const bool is_swipe =
        argc == 2 && (std::strcmp(argv[1], "swipe-left") == 0 ||
                      std::strcmp(argv[1], "swipe-right") == 0);
    if (is_swipe) {
        const bool swipe_left = std::strcmp(argv[1], "swipe-left") == 0;
        const TouchGesture gesture =
            swipe_left ? TouchGesture::SwipeLeft : TouchGesture::SwipeRight;
        const esp_err_t enqueue_result =
            s_active->input_->enqueue_swipe(gesture);
        if (enqueue_result != ESP_OK) {
            std::printf("input: swipe queue full\r\n");
            return 1;
        }
        return 0;
    }
    if (argc != 2 && argc != 3) {
        std::printf("%s", kInputUsage);
        return 1;
    }

    ButtonId button;
    if (std::strcmp(argv[1], "plus") == 0) {
        button = ButtonId::Plus;
    } else if (std::strcmp(argv[1], "pwr") == 0) {
        button = ButtonId::Power;
    } else if (std::strcmp(argv[1], "boot") == 0) {
        button = ButtonId::Boot;
    } else {
        std::printf("%s", kInputUsage);
        return 1;
    }

    uint32_t gesture_hold_ms = InputService::kDefaultGestureHoldMs;
    if (argc == 3 && (!parse_duration_ms(argv[2], gesture_hold_ms) ||
                      gesture_hold_ms < InputService::kMinimumGestureHoldMs)) {
        std::printf("input: hold_ms must be within "
                    "[40,600000]\r\n");
        return 1;
    }
    const esp_err_t enqueue_result =
        s_active->input_->enqueue_button_gesture(button, gesture_hold_ms);
    if (enqueue_result != ESP_OK) {
        std::printf("input: gesture queue full\r\n");
        return 1;
    }
    return 0;
}

int ConsoleService::command_yaw(int argc, char **argv)
{
    if (argc != 2) {
        std::printf("usage: yaw <radians>\r\n");
        return 1;
    }
    float radians = 0.0f;
    if (!parse_float(argv[1], -18.0f, 18.0f, radians)) {
        std::printf("yaw: finite radians must be within [-18,18]\r\n");
        return 1;
    }
    AttitudeFilter::request_yaw(radians);
    std::printf("@DEV YAW %.3f\r\n", static_cast<double>(radians));
    return 0;
}

int ConsoleService::command_axes(int argc, char **argv)
{
    if (argc != 4) {
        std::printf("usage: axes <sx> <sy> <sz>\r\n");
        return 1;
    }
    int x_sign = 0;
    int y_sign = 0;
    int z_sign = 0;
    if (!parse_sign(argv[1], x_sign) || !parse_sign(argv[2], y_sign) ||
        !parse_sign(argv[3], z_sign)) {
        std::printf("axes: each sign must be -1 or 1\r\n");
        return 1;
    }
    AttitudeFilter::set_axes(x_sign, y_sign, z_sign);
    std::printf("@DEV AXES %d %d %d\r\n", x_sign, y_sign, z_sign);
    return 0;
}

int ConsoleService::command_gain(int argc, char **argv)
{
    if (argc != 2) {
        std::printf("usage: gain <scale>\r\n");
        return 1;
    }
    float gain = 0.0f;
    if (!parse_float(argv[1], 0.0f, 4.0f, gain)) {
        std::printf("gain: finite scale must be within [0,4]\r\n");
        return 1;
    }
    AttitudeFilter::set_gain(gain);
    std::printf("@DEV GAIN %.3f\r\n",
                static_cast<double>(AttitudeFilter::gain()));
    return 0;
}

int ConsoleService::command_tau(int argc, char **argv)
{
    if (argc != 2) {
        std::printf("usage: tau <seconds>\r\n");
        return 1;
    }
    float seconds = 0.0f;
    if (!parse_float(argv[1], 0.05f, 2.0f, seconds)) {
        std::printf("tau: finite seconds must be within [0.05,2]\r\n");
        return 1;
    }
    AttitudeFilter::set_tau(seconds);
    std::printf("@DEV TAU %.3f\r\n",
                static_cast<double>(AttitudeFilter::tau()));
    return 0;
}

int ConsoleService::command_framebuffer(int argc, char **argv)
{
    if (argc != 1 && argc != 2) {
        std::printf("usage: fb [request_id]\r\n");
        return 1;
    }

    uint32_t sequence = 0;
    if (argc == 2) {
        if (!parse_sequence(argv[1], sequence)) {
            std::printf("usage: fb [request_id]\r\n");
            return 1;
        }
    } else {
        sequence = ++s_active->capture_sequence_;
    }

    const esp_err_t capture_result = s_active->display_->request_capture();
    if (capture_result != ESP_OK) {
        std::printf("@FB ERROR %" PRIu32 " capture request failed: %s\r\n",
                    sequence, esp_err_to_name(capture_result));
        return 1;
    }

    const int64_t capture_deadline_us =
        esp_timer_get_time() + static_cast<int64_t>(kCaptureTimeoutMs) * 1000;
    while (!s_active->display_->capture_ready() &&
           esp_timer_get_time() < capture_deadline_us) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_active->display_->capture_ready()) {
        std::printf("@FB ERROR %" PRIu32 " capture timed out\r\n", sequence);
        return 1;
    }

    const uint8_t *framebuffer_data = s_active->display_->capture_data();
    s_active->begin_protocol_output();
    std::printf("@FB BEGIN %" PRIu32 " %d %d RGB565BE %zu\r\n", sequence,
                DisplayService::kCaptureWidth, DisplayService::kCaptureHeight,
                DisplayService::kCaptureBytes);
    s_active->drain_protocol_output();

    char encoded_line[((kBase64InputBytesPerLine + 2) / 3) * 4 + 1] = {};
    for (size_t offset = 0; offset < DisplayService::kCaptureBytes;
         offset += kBase64InputBytesPerLine) {
        const size_t remaining_bytes = DisplayService::kCaptureBytes - offset;
        const size_t chunk_size = remaining_bytes < kBase64InputBytesPerLine
                                      ? remaining_bytes
                                      : kBase64InputBytesPerLine;
        encode_base64(framebuffer_data + offset, chunk_size, encoded_line);
        std::printf("@FB DATA %" PRIu32 " %s\r\n", sequence, encoded_line);
        // Let the 256-byte USB/JTAG TX ring drain.
        vTaskDelay(1);
    }
    std::printf("@FB END %" PRIu32 "\r\n", sequence);
    s_active->end_protocol_output();
    return 0;
}

}
