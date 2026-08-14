#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"

namespace fluid_demo {

class DisplayService;
class InputService;
class MotionService;

class ConsoleService {
public:
    using ResetCallback = esp_err_t (*)();

    esp_err_t start(DisplayService &display, MotionService &motion,
                    InputService &input, ResetCallback reset_callback);
    bool protocol_output_active() const
    {
        return protocol_output_active_.load(std::memory_order_acquire);
    }

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
    static int command_axes(int argc, char **argv);
    static int command_gain(int argc, char **argv);
    static int command_tau(int argc, char **argv);

    // esp_console callbacks do not carry user context.
    static ConsoleService *s_active;

    DisplayService *display_ = nullptr;
    MotionService *motion_ = nullptr;
    InputService *input_ = nullptr;
    ResetCallback reset_callback_ = nullptr;
    void *repl_handle_ = nullptr;
    std::atomic<bool> protocol_output_active_{false};
    uint32_t capture_sequence_ = 0;
};

}
