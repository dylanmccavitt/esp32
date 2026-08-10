#include "dev_console.hpp"

#include <atomic>
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

#include "renderer.hpp"

namespace fluid_demo {
namespace {

constexpr char kTag[] = "dev_console";
constexpr uint32_t kCaptureTimeoutMs = 2000;
constexpr size_t kBase64InputPerLine = 768;
constexpr char kBase64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

Renderer *s_renderer = nullptr;
DevResetCallback s_reset_callback = nullptr;
esp_console_repl_t *s_repl = nullptr;
std::atomic<bool> s_dumping{false};
uint32_t s_capture_sequence = 0;

struct MotionOverride {
    Vec3 acceleration{0.0f, 0.0f, 6.0f};
    int64_t until_us = 0;  // zero means no automatic expiry
    bool active = false;
};
portMUX_TYPE s_motion_mux = portMUX_INITIALIZER_UNLOCKED;
MotionOverride s_motion_override;

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


void set_motion_override(const Vec3 &acceleration, uint32_t duration_ms)
{
    const int64_t until_us =
        duration_ms == 0 ? 0 : esp_timer_get_time() + static_cast<int64_t>(duration_ms) * 1000;
    portENTER_CRITICAL(&s_motion_mux);
    s_motion_override.acceleration = acceleration;
    s_motion_override.until_us = until_us;
    s_motion_override.active = true;
    portEXIT_CRITICAL(&s_motion_mux);
}

void clear_motion_override()
{
    portENTER_CRITICAL(&s_motion_mux);
    s_motion_override.active = false;
    s_motion_override.until_us = 0;
    portEXIT_CRITICAL(&s_motion_mux);
}

int command_ping(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: ping\r\n");
        return 1;
    }
    std::printf("@DEV PONG\r\n");
    return 0;
}

int command_status(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: status\r\n");
        return 1;
    }

    MotionOverride snapshot;
    portENTER_CRITICAL(&s_motion_mux);
    snapshot = s_motion_override;
    if (snapshot.active && snapshot.until_us != 0 && esp_timer_get_time() >= snapshot.until_us) {
        s_motion_override.active = false;
        snapshot.active = false;
    }
    portEXIT_CRITICAL(&s_motion_mux);

    const uint64_t uptime_ms = static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
    std::printf("@DEV STATUS uptime_ms=%" PRIu64
                " override=%u accel=%.3f,%.3f,%.3f capture_ready=%u\r\n",
                uptime_ms, snapshot.active ? 1u : 0u,
                static_cast<double>(snapshot.acceleration.x),
                static_cast<double>(snapshot.acceleration.y),
                static_cast<double>(snapshot.acceleration.z),
                s_renderer != nullptr && s_renderer->capture_ready() ? 1u : 0u);
    return 0;
}

int command_motion(int argc, char **argv)
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
    set_motion_override(acceleration, duration_ms);
    std::printf("@DEV MOTION %.3f %.3f %.3f duration_ms=%" PRIu32 "\r\n",
                static_cast<double>(acceleration.x), static_cast<double>(acceleration.y),
                static_cast<double>(acceleration.z), duration_ms);
    return 0;
}

int command_release(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1) {
        std::printf("usage: release\r\n");
        return 1;
    }
    clear_motion_override();
    std::printf("@DEV MOTION_RELEASED\r\n");
    return 0;
}

int command_reset(int argc, char **argv)
{
    static_cast<void>(argv);
    if (argc != 1 || s_reset_callback == nullptr) {
        std::printf("usage: reset\r\n");
        return 1;
    }
    s_reset_callback();
    std::printf("@DEV RESET_REQUESTED\r\n");
    return 0;
}

int command_reboot(int argc, char **argv)
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

int command_framebuffer(int argc, char **argv)
{
    if ((argc != 1 && argc != 2) || s_renderer == nullptr) {
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
        sequence = ++s_capture_sequence;
    }
    const esp_err_t request = s_renderer->request_capture();
    if (request != ESP_OK) {
        std::printf("@FB ERROR %" PRIu32 " capture request failed: %s\r\n",
                    sequence, esp_err_to_name(request));
        return 1;
    }

    const int64_t deadline =
        esp_timer_get_time() + static_cast<int64_t>(kCaptureTimeoutMs) * 1000;
    while (!s_renderer->capture_ready() && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!s_renderer->capture_ready()) {
        std::printf("@FB ERROR %" PRIu32 " capture timed out\r\n", sequence);
        return 1;
    }

    const uint8_t *data = s_renderer->capture_data();
    s_dumping.store(true, std::memory_order_release);
    // The renderer is CPU-bound on core 0. Give it one scheduling window to
    // observe s_dumping and park before this lower-priority REPL task streams.
    vTaskDelay(pdMS_TO_TICKS(10));
    std::printf("@FB BEGIN %" PRIu32 " %d %d RGB565BE %zu\r\n",
                sequence, Renderer::kCaptureWidth, Renderer::kCaptureHeight,
                Renderer::kCaptureBytes);

    char encoded[((kBase64InputPerLine + 2) / 3) * 4 + 1] = {};
    for (size_t offset = 0; offset < Renderer::kCaptureBytes;
         offset += kBase64InputPerLine) {
        const size_t chunk =
            (Renderer::kCaptureBytes - offset < kBase64InputPerLine)
                ? Renderer::kCaptureBytes - offset
                : kBase64InputPerLine;
        encode_base64(data + offset, chunk, encoded);
        std::printf("@FB DATA %" PRIu32 " %s\r\n", sequence, encoded);
        // Keep IDLE0 runnable throughout a full framebuffer dump.
        vTaskDelay(1);
    }
    std::printf("@FB END %" PRIu32 "\r\n", sequence);
    std::fflush(stdout);
    s_dumping.store(false, std::memory_order_release);
    return 0;
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

esp_err_t dev_console_start(Renderer *renderer, DevResetCallback reset_callback)
{
    if (renderer == nullptr || reset_callback == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_repl != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    s_renderer = renderer;
    s_reset_callback = reset_callback;

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
                                command_reboot)) != ESP_OK) {
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
    err = esp_console_new_repl_usb_serial_jtag(&device_config, &repl_config, &s_repl);
    if (err != ESP_OK) return err;
    err = esp_console_start_repl(s_repl);
    if (err != ESP_OK) return err;
    ESP_LOGI(kTag, "USB dev console ready: ping/status/fb/motion/release/reset/reboot");
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool dev_console_motion_override(Vec3 *apparent_accel)
{
    if (apparent_accel == nullptr) {
        return false;
    }
    const int64_t now_us = esp_timer_get_time();
    bool active = false;
    portENTER_CRITICAL(&s_motion_mux);
    if (s_motion_override.active && s_motion_override.until_us != 0 &&
        now_us >= s_motion_override.until_us) {
        s_motion_override.active = false;
    }
    if (s_motion_override.active) {
        *apparent_accel = s_motion_override.acceleration;
        active = true;
    }
    portEXIT_CRITICAL(&s_motion_mux);
    return active;
}

bool dev_console_dump_active()
{
    return s_dumping.load(std::memory_order_acquire);
}

}  // namespace fluid_demo
