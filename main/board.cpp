#include "board.hpp"

#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmi8658.h"

namespace fluid_demo {
namespace {

constexpr char kTag[] = "board";

// Waveshare ESP32-S3-Touch-LCD-1.54 pin map.
constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
constexpr gpio_num_t kI2cSda = GPIO_NUM_42;
constexpr gpio_num_t kI2cScl = GPIO_NUM_41;
constexpr int kI2cTimeoutMs = 100;
constexpr int kMotionReadTimeoutMs = 10;
constexpr int kMotionReadAttempts = 3;
constexpr uint32_t kMotionRetryDelayUs = 200;
constexpr uint32_t kMotionI2cHz = 100000;
constexpr uint32_t kMotionSclWaitUs = 10000;
constexpr float kAccelMps2PerLsbAt4G = 9.807f / 8192.0f;
constexpr float kGyroRadsPerLsbAt256Dps = 0.01745329252f / 128.0f;

constexpr spi_host_device_t kLcdHost = SPI2_HOST;
constexpr gpio_num_t kLcdSclk = GPIO_NUM_38;
constexpr gpio_num_t kLcdMosi = GPIO_NUM_39;
constexpr gpio_num_t kLcdCs = GPIO_NUM_21;
constexpr gpio_num_t kLcdDc = GPIO_NUM_45;
constexpr gpio_num_t kLcdReset = GPIO_NUM_40;
constexpr gpio_num_t kLcdBacklight = GPIO_NUM_46;
constexpr uint32_t kLcdPixelClockHz = 40U * 1000U * 1000U;
constexpr size_t kLcdMaxTransferBytes = 240U * 16U * sizeof(uint16_t);

constexpr gpio_num_t kBootButton = GPIO_NUM_0;
constexpr gpio_num_t kPowerButton = GPIO_NUM_5;
constexpr gpio_num_t kPlusButton = GPIO_NUM_4;
constexpr gpio_num_t kBatteryEnable = GPIO_NUM_2;

// CST816 reports are valid only after its falling-edge interrupt.
constexpr gpio_num_t kTouchReset = GPIO_NUM_47;
constexpr gpio_num_t kTouchInt = GPIO_NUM_48;
constexpr uint8_t kTouchI2cAddress = 0x15;
constexpr uint8_t kTouchReportRegister = 0x01;
constexpr uint8_t kTouchFingerCountMask = 0x0F;
constexpr uint8_t kTouchCoordinateHighMask = 0x0F;
constexpr uint16_t kTouchMaximumCoordinate = 239;
constexpr int kTouchResetLowMs = 200;
constexpr int kTouchResetHighMs = 200;
constexpr int kTouchReadAttempts = 3;
constexpr uint8_t kCst816SwipeLeftGestureId = 0x03;
constexpr uint8_t kCst816SwipeRightGestureId = 0x04;

struct Cst816Report {
    uint8_t gesture_id;
    uint8_t finger_count;
    uint8_t x_high;
    uint8_t x_low;
    uint8_t y_high;
    uint8_t y_low;
};
static_assert(sizeof(Cst816Report) == 6);

constexpr TouchGesture cst816_gesture(uint8_t gesture_id)
{
    switch (gesture_id) {
    case kCst816SwipeLeftGestureId:
        return TouchGesture::SwipeLeft;
    case kCst816SwipeRightGestureId:
        return TouchGesture::SwipeRight;
    default:
        return TouchGesture::None;
    }
}

i2c_master_bus_handle_t s_i2c_bus = nullptr;
qmi8658_dev_t s_motion_sensor{};
i2c_master_dev_handle_t s_touch_device = nullptr;
volatile bool s_touch_irq_pending = false;
portMUX_TYPE s_touch_irq_mux = portMUX_INITIALIZER_UNLOCKED;

esp_err_t init_power_hold()
{
    gpio_config_t power_hold_config{};
    power_hold_config.pin_bit_mask = 1ULL << kBatteryEnable;
    power_hold_config.mode = GPIO_MODE_OUTPUT;
    power_hold_config.pull_up_en = GPIO_PULLUP_DISABLE;
    power_hold_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    power_hold_config.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&power_hold_config), kTag,
                        "BAT_EN GPIO init failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kBatteryEnable, 1), kTag,
                        "BAT_EN assert failed");
    // Retain BAT_EN across esp_restart().
    return gpio_hold_en(kBatteryEnable);
}

esp_err_t init_buttons()
{
    gpio_config_t button_config{};
    button_config.pin_bit_mask =
        (1ULL << kBootButton) | (1ULL << kPowerButton) | (1ULL << kPlusButton);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&button_config);
}

esp_err_t init_i2c()
{
    i2c_master_bus_config_t bus_config{};
    bus_config.i2c_port = kI2cPort;
    bus_config.sda_io_num = kI2cSda;
    bus_config.scl_io_num = kI2cScl;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.intr_priority = 0;
    bus_config.trans_queue_depth = 0;
    bus_config.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&bus_config, &s_i2c_bus);
}

void IRAM_ATTR touch_interrupt_isr(void *)
{
    portENTER_CRITICAL_ISR(&s_touch_irq_mux);
    s_touch_irq_pending = true;
    portEXIT_CRITICAL_ISR(&s_touch_irq_mux);
}

bool consume_touch_interrupt()
{
    portENTER_CRITICAL(&s_touch_irq_mux);
    const bool interrupt_pending = s_touch_irq_pending;
    s_touch_irq_pending = false;
    portEXIT_CRITICAL(&s_touch_irq_mux);
    return interrupt_pending;
}

esp_err_t init_touch()
{
    gpio_config_t reset_config{};
    reset_config.pin_bit_mask = 1ULL << kTouchReset;
    reset_config.mode = GPIO_MODE_OUTPUT;
    reset_config.pull_up_en = GPIO_PULLUP_DISABLE;
    reset_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset_config.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&reset_config), kTag,
                        "touch RST GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kTouchReset, 0), kTag,
                        "touch RST low failed");
    vTaskDelay(pdMS_TO_TICKS(kTouchResetLowMs));
    ESP_RETURN_ON_ERROR(gpio_set_level(kTouchReset, 1), kTag,
                        "touch RST high failed");
    vTaskDelay(pdMS_TO_TICKS(kTouchResetHighMs));

    // Reset may leave the shared bus mid-cycle.
    ESP_RETURN_ON_ERROR(i2c_master_bus_reset(s_i2c_bus), kTag,
                        "I2C recovery after touch reset failed");

    gpio_config_t interrupt_config{};
    interrupt_config.pin_bit_mask = 1ULL << kTouchInt;
    interrupt_config.mode = GPIO_MODE_INPUT;
    interrupt_config.pull_up_en = GPIO_PULLUP_ENABLE;
    interrupt_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    interrupt_config.intr_type = GPIO_INTR_NEGEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&interrupt_config), kTag,
                        "touch INT GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_IRAM), kTag,
                        "touch ISR service install failed");
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(kTouchInt, touch_interrupt_isr, nullptr), kTag,
        "touch INT handler add failed");

    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = kTouchI2cAddress;
    device_config.scl_speed_hz = kMotionI2cHz;
    device_config.scl_wait_us = kMotionSclWaitUs;
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_touch_device),
        kTag, "touch device add failed");

    ESP_LOGI(kTag,
             "CST816 touch ready at 0x%02X "
             "(RST GPIO47, INT GPIO48)",
             kTouchI2cAddress);
    return ESP_OK;
}

esp_err_t init_imu()
{
    uint8_t i2c_address = 0;
    if (i2c_master_probe(s_i2c_bus, QMI8658_ADDRESS_HIGH, kI2cTimeoutMs) ==
        ESP_OK) {
        i2c_address = QMI8658_ADDRESS_HIGH;
    } else if (i2c_master_probe(s_i2c_bus, QMI8658_ADDRESS_LOW,
                                kI2cTimeoutMs) == ESP_OK) {
        i2c_address = QMI8658_ADDRESS_LOW;
    } else {
        ESP_LOGE(kTag, "QMI8658 not found at 0x%02X or 0x%02X",
                 QMI8658_ADDRESS_HIGH, QMI8658_ADDRESS_LOW);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(qmi8658_init(&s_motion_sensor, s_i2c_bus, i2c_address),
                        kTag, "QMI8658 init failed");
    ESP_RETURN_ON_ERROR(
        qmi8658_set_accel_range(&s_motion_sensor, QMI8658_ACCEL_RANGE_4G), kTag,
        "QMI8658 accel range failed");
    ESP_RETURN_ON_ERROR(
        qmi8658_set_accel_odr(&s_motion_sensor, QMI8658_ACCEL_ODR_250HZ), kTag,
        "QMI8658 accel ODR failed");
    ESP_RETURN_ON_ERROR(
        qmi8658_set_gyro_range(&s_motion_sensor, QMI8658_GYRO_RANGE_256DPS),
        kTag, "QMI8658 gyro range failed");
    ESP_RETURN_ON_ERROR(
        qmi8658_set_gyro_odr(&s_motion_sensor, QMI8658_GYRO_ODR_250HZ), kTag,
        "QMI8658 gyro ODR failed");
    qmi8658_set_accel_unit_mps2(&s_motion_sensor, true);
    qmi8658_set_gyro_unit_rads(&s_motion_sensor, true);

    // Use conservative bus timing while display DMA is active.
    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(s_motion_sensor.dev_handle),
                        kTag, "QMI8658 device rebind remove failed");
    s_motion_sensor.dev_handle = nullptr;
    i2c_device_config_t device_config{};
    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = i2c_address;
    device_config.scl_speed_hz = kMotionI2cHz;
    device_config.scl_wait_us = kMotionSclWaitUs;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &device_config,
                                                  &s_motion_sensor.dev_handle),
                        kTag, "QMI8658 device rebind add failed");

    ESP_LOGI(kTag,
             "QMI8658 ready at 0x%02X "
             "(4G/256dps, 250 Hz ODR, 100 kHz I2C)",
             i2c_address);
    return ESP_OK;
}

esp_err_t init_display(BoardHandles &handles)
{
    gpio_config_t backlight_config{};
    backlight_config.pin_bit_mask = 1ULL << kLcdBacklight;
    backlight_config.mode = GPIO_MODE_OUTPUT;
    backlight_config.pull_up_en = GPIO_PULLUP_DISABLE;
    backlight_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    backlight_config.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&backlight_config), kTag,
                        "LCD backlight GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kLcdBacklight, 0), kTag,
                        "LCD backlight off failed");

    spi_bus_config_t spi_bus_config{};
    spi_bus_config.sclk_io_num = kLcdSclk;
    spi_bus_config.mosi_io_num = kLcdMosi;
    spi_bus_config.miso_io_num = GPIO_NUM_NC;
    spi_bus_config.quadwp_io_num = GPIO_NUM_NC;
    spi_bus_config.quadhd_io_num = GPIO_NUM_NC;
    spi_bus_config.max_transfer_sz = kLcdMaxTransferBytes;
    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(kLcdHost, &spi_bus_config, SPI_DMA_CH_AUTO), kTag,
        "LCD SPI bus init failed");

    esp_lcd_panel_io_spi_config_t panel_io_config{};
    panel_io_config.cs_gpio_num = kLcdCs;
    panel_io_config.dc_gpio_num = kLcdDc;
    panel_io_config.spi_mode = 3;
    panel_io_config.pclk_hz = kLcdPixelClockHz;
    panel_io_config.trans_queue_depth = 2;
    panel_io_config.lcd_cmd_bits = 8;
    panel_io_config.lcd_param_bits = 8;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(
                            static_cast<esp_lcd_spi_bus_handle_t>(kLcdHost),
                            &panel_io_config, &handles.io),
                        kTag, "ST7789 panel I/O create failed");

    esp_lcd_panel_dev_config_t panel_config{};
    panel_config.reset_gpio_num = kLcdReset;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_st7789(handles.io, &panel_config, &handles.panel),
        kTag, "ST7789 panel create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(handles.panel), kTag,
                        "ST7789 reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(handles.panel), kTag,
                        "ST7789 init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(handles.panel, true), kTag,
                        "ST7789 invert failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(handles.panel, true), kTag,
                        "ST7789 display-on failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kLcdBacklight, 1), kTag,
                        "LCD backlight on failed");

    ESP_LOGI(kTag, "ST7789 ready: 240x240 RGB565, SPI 40 MHz");
    return ESP_OK;
}
}

esp_err_t board_init(BoardHandles &handles)
{
    handles = {};

    ESP_LOGI(kTag, "Waveshare ESP32-S3-Touch-LCD-1.54 "
                   "board init");
    ESP_RETURN_ON_ERROR(init_power_hold(), kTag, "power hold failed");
    ESP_RETURN_ON_ERROR(init_buttons(), kTag, "button GPIO init failed");
    ESP_RETURN_ON_ERROR(init_i2c(), kTag, "sensor I2C init failed");
    ESP_RETURN_ON_ERROR(init_touch(), kTag, "touch init failed");
    ESP_RETURN_ON_ERROR(init_imu(), kTag, "IMU init failed");
    ESP_RETURN_ON_ERROR(init_display(handles), kTag, "display init failed");
    return ESP_OK;
}

esp_err_t board_read_motion(Vec3 &acceleration_mps2, Vec3 &angular_rate_rads,
                            bool &fresh)
{
    fresh = false;

    // Read the contiguous accel/gyro registers coherently.
    uint8_t start_register = QMI8658_AX_L;
    uint8_t report[12]{};
    esp_err_t read_result = ESP_FAIL;
    for (int attempt = 0;
         attempt < kMotionReadAttempts && read_result != ESP_OK; ++attempt) {
        read_result = i2c_master_transmit_receive(
            s_motion_sensor.dev_handle, &start_register, sizeof(start_register),
            report, sizeof(report), kMotionReadTimeoutMs);
        if (read_result != ESP_OK) {
            esp_rom_delay_us(kMotionRetryDelayUs);
        }
    }
    if (read_result != ESP_OK) {
        ESP_RETURN_ON_ERROR(i2c_master_bus_reset(s_i2c_bus), kTag,
                            "sensor I2C reset failed");
        esp_rom_delay_us(kMotionRetryDelayUs);
        read_result = i2c_master_transmit_receive(
            s_motion_sensor.dev_handle, &start_register, sizeof(start_register),
            report, sizeof(report), kMotionReadTimeoutMs);
        if (read_result != ESP_OK) {
            return read_result;
        }
    }

    const auto read_i16_le = [&report](size_t offset) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(report[offset]) |
            (static_cast<uint16_t>(report[offset + 1]) << 8));
    };

    acceleration_mps2.x =
        static_cast<float>(read_i16_le(0)) * kAccelMps2PerLsbAt4G;
    acceleration_mps2.y =
        static_cast<float>(read_i16_le(2)) * kAccelMps2PerLsbAt4G;
    acceleration_mps2.z =
        static_cast<float>(read_i16_le(4)) * kAccelMps2PerLsbAt4G;
    angular_rate_rads.x =
        static_cast<float>(read_i16_le(6)) * kGyroRadsPerLsbAt256Dps;
    angular_rate_rads.y =
        static_cast<float>(read_i16_le(8)) * kGyroRadsPerLsbAt256Dps;
    angular_rate_rads.z =
        static_cast<float>(read_i16_le(10)) * kGyroRadsPerLsbAt256Dps;
    fresh = true;
    return ESP_OK;
}

esp_err_t board_read_touch(TouchSample &sample)
{
    sample.fresh = false;
    sample.pressed = false;
    sample.gesture = TouchGesture::None;

    const bool interrupt_pending = consume_touch_interrupt();
    if (!interrupt_pending && gpio_get_level(kTouchInt) != 0) {
        return ESP_OK;
    }

    uint8_t report_register = kTouchReportRegister;
    Cst816Report report{};
    esp_err_t read_result = ESP_FAIL;
    for (int attempt = 0; attempt < kTouchReadAttempts && read_result != ESP_OK;
         ++attempt) {
        // Retry while the controller's report window is open.
        read_result = i2c_master_transmit_receive(
            s_touch_device, &report_register, sizeof(report_register),
            reinterpret_cast<uint8_t *>(&report), sizeof(report),
            kMotionReadTimeoutMs);
    }
    if (read_result != ESP_OK) {
        return read_result;
    }

    sample.gesture = cst816_gesture(report.gesture_id);
    const uint8_t pressed_fingers = report.finger_count & kTouchFingerCountMask;
    if (pressed_fingers == 0) {
        sample.fresh = true;
        return ESP_OK;
    }

    const uint16_t x =
        (static_cast<uint16_t>(report.x_high & kTouchCoordinateHighMask) << 8) |
        static_cast<uint16_t>(report.x_low);
    const uint16_t y =
        (static_cast<uint16_t>(report.y_high & kTouchCoordinateHighMask) << 8) |
        static_cast<uint16_t>(report.y_low);
    if (x > kTouchMaximumCoordinate || y > kTouchMaximumCoordinate) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    sample.x = x;
    sample.y = y;
    sample.pressed = true;
    sample.fresh = true;
    return ESP_OK;
}

bool board_reset_pressed()
{
    return gpio_get_level(kPlusButton) == 0;
}

bool board_power_pressed()
{
    return gpio_get_level(kPowerButton) == 0;
}

esp_err_t board_power_off()
{
    ESP_LOGI(kTag, "PWR long press - releasing BAT_EN");
    ESP_RETURN_ON_ERROR(gpio_hold_dis(kBatteryEnable), kTag,
                        "BAT_EN hold release failed");
    return gpio_set_level(kBatteryEnable, 0);
}

bool board_battery_hold_enabled()
{
    return gpio_get_level(kBatteryEnable) != 0;
}

bool board_boot_pressed()
{
    return gpio_get_level(kBootButton) == 0;
}

}
