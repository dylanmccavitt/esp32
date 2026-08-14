#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"

#include "app_shell.hpp"

namespace fluid_demo {

struct BoardHandles {
    esp_lcd_panel_handle_t panel = nullptr;
    esp_lcd_panel_io_handle_t io = nullptr;
};

esp_err_t board_init(BoardHandles &handles);
esp_err_t board_read_motion(Vec3 &acceleration_mps2, Vec3 &angular_rate_rads,
                            bool &fresh);

struct TouchSample {
    uint16_t x = 0;
    uint16_t y = 0;
    TouchGesture gesture = TouchGesture::None;
    bool pressed = false;
    bool fresh = false;
};

esp_err_t board_read_touch(TouchSample &sample);

bool board_reset_pressed();
bool board_power_pressed();
esp_err_t board_power_off();
bool board_battery_hold_enabled();
bool board_boot_pressed();

}
