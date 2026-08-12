#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

namespace fluid_demo {

class DisplayService;
class InputService;
class MotionService;

/// USB Serial/JTAG development console and framebuffer protocol endpoint.
class ConsoleService {
public:
    using ResetTrampoline = esp_err_t (*)();

    esp_err_t start(DisplayService *display, MotionService *motion,
                    InputService *input, ResetTrampoline reset_callback);

    bool dump_active() const { return dumping_.load(std::memory_order_acquire); }

    void emit_poweroff();
    void emit_rebooting();

private:
    void begin_protocol_output();
    void drain_protocol_output();
    void end_protocol_output();

    static int command_ping(int argc, char **argv);
    static int command_status(int argc, char **argv);
    static int command_framebuffer(int argc, char **argv);
    static int command_motion(int argc, char **argv);
    static int command_release(int argc, char **argv);
    static int command_reset(int argc, char **argv);
    static int command_reboot(int argc, char **argv);
    static int command_input(int argc, char **argv);
    static int command_yaw(int argc, char **argv);

    // esp_console callbacks do not carry user context.
    static ConsoleService *s_active;

    DisplayService *display_ = nullptr;
    MotionService *motion_ = nullptr;
    InputService *input_ = nullptr;
    ResetTrampoline reset_callback_ = nullptr;
    void *repl_ = nullptr;
    std::atomic<bool> dumping_{false};
    uint32_t capture_sequence_ = 0;
};

}  // namespace fluid_demo
