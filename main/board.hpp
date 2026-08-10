#pragma once

#include "esp_err.h"
#include "esp_lcd_types.h"

#include "app_types.hpp"

namespace fluid_demo {

/**
 * @brief Handles for the direct SPI LCD path (no LVGL or framebuffer).
 */
struct BoardHandles {
    esp_lcd_panel_handle_t panel = nullptr; /*!< ST7789 panel handle */
    esp_lcd_panel_io_handle_t io = nullptr; /*!< SPI panel I/O callback target */
};

/**
 * @brief Initialize the Waveshare ESP32-S3-Touch-LCD-1.54 hardware.
 *
 * Uses the official board pin map: QMI8658 on I2C0 GPIO42/41, 240x240 ST7789
 * on SPI2 (GPIO38/39/21/45/40/46), PLUS on GPIO4, PWR on GPIO5 and BOOT on
 * GPIO0. BAT_EN on GPIO2 is asserted during startup. The IMU is configured for
 * 4G/256 dps at 250 Hz.
 *
 * @param[out] out Receives initialized panel and panel-I/O handles.
 * @return ESP_OK on success, otherwise the first failing subsystem error.
 */
esp_err_t board_init(BoardHandles *out);

/**
 * @brief Read raw QMI8658 data in the sensor frame.
 *
 * Acceleration is in m/s^2 and gyro in rad/s. Both outputs are valid only when
 * `*fresh` is true; MotionFilter performs the sensor-to-box axis mapping.
 *
 * @param[out] accel_mps2 Sensor-frame acceleration.
 * @param[out] gyro_rads Sensor-frame angular rate.
 * @param[out] fresh True iff both reads succeeded in this call.
 * @return ESP_OK on success, otherwise the failing I2C read error.
 */
esp_err_t board_read_motion(Vec3 *accel_mps2, Vec3 *gyro_rads, bool *fresh);

/**
 * @brief Current PLUS (GPIO4) level.
 *
 * The custom PLUS button is active-low and input-only. The sensor task
 * debounces it and turns one press into one in-RAM fluid reset.
 *
 * @return true while PLUS is held.
 */
bool board_reset_pressed();

/**
 * @brief Current PWR (GPIO5) level and explicit software power-off.
 *
 * PWR is active-low. A validated long press may release BAT_EN (GPIO2); short
 * presses are left available and have no application action.
 */
bool board_power_pressed();
esp_err_t board_power_off();

/**
 * @brief Current BOOT (GPIO0) level. Active-low: true while the button is held down.
 *
 * Callers are responsible for debouncing and for interpreting a validated short
 * press-release (see app_main). The pin is input-only with pull-up; never drive it.
 *
 * @return true when GPIO0 reads low.
 */
bool board_boot_pressed();

} // namespace fluid_demo
