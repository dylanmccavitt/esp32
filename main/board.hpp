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
 * @brief One fresh capacitive-touch report, in the display-native orientation.
 *
 * The CST816-family controller reports 12-bit coordinates that natively
 * match the 240x240 ST7789: both x and y are in [0, 239]. `fresh` means this
 * call consumed a complete controller report; `pressed` carries its finger
 * count state. Coordinates are published only for a fresh pressed report.
 */
struct TouchSample {
    uint16_t x = 0;       /*!< 0..239 column, matching the ST7789 */
    uint16_t y = 0;       /*!< 0..239 row, matching the ST7789 */
    bool pressed = false; /*!< true iff the fresh report has one finger */
    bool fresh = false;   /*!< true iff this call consumed a report */
};

/**
 * @brief Consume one pending CST816 touch report (IRQ/level-gated).
 *
 * The shell's sensor lane polls this at its 100 Hz cadence. A read happens
 * only after the falling-edge INT interrupt latched a pending flag or while
 * the active-low line remains asserted; there is never an idle controller
 * read, and the shared I2C0 bus (QMI8658) is never reset or recreated from the
 * runtime touch path. A report whose finger count is zero returns ESP_OK with
 * `out->fresh` true and `out->pressed` false so InputService can re-arm only
 * after a physical release; no pending event returns both false. Transactions
 * retry immediately (no delay) inside the event window on transient failure —
 * some parts NACK a report read unless a touch interrupt just occurred — and
 * the final failure is returned without touching the bus.
 *
 * @param[out] out Receives the report when a touch was observed.
 * @return ESP_OK (with `out->fresh` set per the event), ESP_ERR_INVALID_ARG
 *         for a null out, the final failed I2C read error, or
 *         ESP_ERR_INVALID_RESPONSE for a coordinate outside [0, 239].
 */
esp_err_t board_read_touch(TouchSample *out);

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
 * @brief Current active-low PWR (GPIO5) level.
 *
 * @return true while the button is held.
 */
bool board_power_pressed();

/**
 * @brief Release the battery-only power hold on BAT_EN (GPIO2).
 *
 * USB VBUS can continue powering the board after this succeeds.
 *
 * @return ESP_OK on success, otherwise the GPIO failure.
 */
esp_err_t board_power_off();

/**
 * @brief Read whether the BAT_EN battery hold remains asserted.
 *
 * @return true while BAT_EN is high.
 */
bool board_battery_hold_enabled();

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
