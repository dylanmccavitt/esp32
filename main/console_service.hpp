#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

namespace fluid_demo {

class DisplayService;
class MotionService;

/// USB Serial/JTAG development REPL owning all firmware debug commands.
/// Replaces dev_console.* with the identical command surface, @DEV/@FB
/// framing, s_dumping stdout dump gate, prompt and REPL parameters.
///
/// Bindings are fixed once for the process lifetime at start(): the shell's
/// DisplayService (capture), the shell's MotionService (motion/release/
/// status) and a ResetTrampoline — a one-time bound, non-lifecycle callback
/// into the Fluid app's reset atomic — never rebound.
class ConsoleService {
public:
    /// One-time bound, non-lifecycle callback into the active app's reset.
    using ResetTrampoline = void (*)();

    /// Start the REPL (priority 2, core 0, 6144-byte stack, USB Serial/JTAG).
    /// ESP_ERR_INVALID_STATE if already started, ESP_ERR_INVALID_ARG on a null
    /// binding, ESP_ERR_NOT_SUPPORTED without USB Serial/JTAG console.
    esp_err_t start(DisplayService *display, MotionService *motion,
                    ResetTrampoline reset_callback);

    /// True while a framebuffer dump owns stdout; telemetry should stay quiet.
    bool dump_active() const { return dumping_.load(std::memory_order_acquire); }

private:
    static int command_ping(int argc, char **argv);
    static int command_status(int argc, char **argv);
    static int command_framebuffer(int argc, char **argv);
    static int command_motion(int argc, char **argv);
    static int command_release(int argc, char **argv);
    static int command_reset(int argc, char **argv);
    static int command_reboot(int argc, char **argv);

    /// The single running instance (esp_console commands are plain function
    /// pointers without user context).
    static ConsoleService *s_active;

    DisplayService *display_ = nullptr;
    MotionService *motion_ = nullptr;
    ResetTrampoline reset_callback_ = nullptr;
    void *repl_ = nullptr;  // esp_console_repl_t*, owned by the console component
    std::atomic<bool> dumping_{false};
    uint32_t capture_sequence_ = 0;
};

}  // namespace fluid_demo
