#pragma once

#include "esp_err.h"

#include "app_types.hpp"

namespace fluid_demo {

class DisplayService;
using DevResetCallback = void (*)();

/** Start the USB Serial/JTAG development REPL and register fluid controls. */
esp_err_t dev_console_start(DisplayService *display, DevResetCallback reset_callback);

/** Override the live IMU with a box-space acceleration when a dev drive is active. */
bool dev_console_motion_override(Vec3 *apparent_accel);

/** True only while a framebuffer dump owns stdout; telemetry should stay quiet. */
bool dev_console_dump_active();

}  // namespace fluid_demo
