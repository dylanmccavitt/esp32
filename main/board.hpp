#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"

#include "app_shell.hpp"
#include "app_types.hpp"

namespace fluid_demo {

struct BoardHandles {
    esp_lcd_panel_handle_t panel = nullptr;
    esp_lcd_panel_io_handle_t io = nullptr;
};

/// Initialize the display, IMU, touch controller, buttons, and battery hold.
esp_err_t board_init(BoardHandles *out);

/// Read one QMI8658 sample in sensor coordinates.
esp_err_t board_read_motion(Vec3 *accel_mps2, Vec3 *gyro_rads, bool *fresh);

struct TouchSample {
    uint16_t x = 0;
    uint16_t y = 0;
    TouchGesture gesture = TouchGesture::None;
    bool pressed = false;
    bool fresh = false;
};

/// Consume one IRQ-gated CST816 report in display coordinates.
esp_err_t board_read_touch(TouchSample *out);

bool board_reset_pressed();
bool board_power_pressed();
esp_err_t board_power_off();
bool board_battery_hold_enabled();
bool board_boot_pressed();

}  // namespace fluid_demo
