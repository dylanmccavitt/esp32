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

constexpr const char *kTag = "board";

// Waveshare ESP32-S3-Touch-LCD-1.54 pin map (official ESP-IDF examples).
constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
constexpr gpio_num_t kI2cSda = GPIO_NUM_42;
constexpr gpio_num_t kI2cScl = GPIO_NUM_41;
constexpr int kI2cTimeoutMs = 100;
constexpr int kMotionReadTimeoutMs = 10;
constexpr uint32_t kMotionI2cHz = 100000;
constexpr uint32_t kMotionSclWaitUs = 10000;
constexpr float kAccelScaleMps2 = 9.807f / 8192.0f;  // configured 4G range
constexpr float kGyroScaleRads = 0.01745329252f / 128.0f;  // configured 256 dps range

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

// CST816-family capacitive touch (official board evidence): 7-bit address
// 0x15 on the shared I2C0 bus, RST active-low on GPIO47, INT falling edge on
// GPIO48. One six-byte report transaction starts at register 0x01 and returns
// {gesture id, finger count, X high nibble, X low, Y high nibble, Y low}
// with 12-bit coordinates. The controller is read-only-on-INT: some parts
// NACK a report read unless a touch interrupt just occurred, so reads are
// IRQ-gated with immediate in-window retries and an idle NACK never resets
// the shared bus.
constexpr gpio_num_t kTouchReset = GPIO_NUM_47;
constexpr gpio_num_t kTouchInt = GPIO_NUM_48;
constexpr uint8_t kTouchI2cAddr = 0x15;
constexpr uint8_t kTouchReportReg = 0x01;
constexpr size_t kTouchReportLen = 6;          // registers 0x01..0x06
constexpr uint8_t kTouchFingerMask = 0x0F;     // finger count lives in the low nibble
constexpr uint8_t kTouchCoordHighMask = 0x0F;  // 12-bit coords: high byte low nibble
constexpr uint16_t kTouchMaxCoord = 239;       // display-native 240x240 bound
constexpr int kTouchResetLowMs = 200;
constexpr int kTouchResetHighMs = 200;
constexpr int kTouchReadAttempts = 3;          // initial + immediate retries

/// Map the controller's raw gesture id to the shell's logical swipe. CST816-
/// family ids: 0x03 = slide left, 0x04 = slide right; every other id (0 =
/// none, click, long-press, double-click) is not a swipe. Pure and constant
/// so the release report can be classified without any allocation.
constexpr TouchGesture cst816_gesture(uint8_t id)
{
    switch (id) {
        case 0x03:
            return TouchGesture::SwipeLeft;
        case 0x04:
            return TouchGesture::SwipeRight;
        default:
            return TouchGesture::None;
    }
}

static i2c_master_bus_handle_t s_i2c = nullptr;
static qmi8658_dev_t s_imu{};
static i2c_master_dev_handle_t s_touch_dev = nullptr;
// Bounded ISR -> sensor-task latch: the ISR only sets the flag under the mux,
// the sensor lane consumes it under the same mux. No allocation, no I2C, and
// the ISR never touches the shared bus.
static volatile bool s_touch_irq_pending = false;
static portMUX_TYPE s_touch_irq_mux = portMUX_INITIALIZER_UNLOCKED;

esp_err_t init_power_hold()
{
    gpio_config_t cfg{};
    cfg.pin_bit_mask = 1ULL << kBatteryEnable;
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), kTag, "BAT_EN GPIO init failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kBatteryEnable, 1), kTag, "BAT_EN assert failed");
    // Keep the latch asserted across esp_restart(); the next app instance
    // reconfigures the same high level before retaining it again.
    return gpio_hold_en(kBatteryEnable);
}

esp_err_t init_buttons()
{
    gpio_config_t cfg{};
    cfg.pin_bit_mask =
        (1ULL << kBootButton) | (1ULL << kPowerButton) | (1ULL << kPlusButton);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&cfg);
}

esp_err_t init_i2c()
{
    i2c_master_bus_config_t cfg{};
    cfg.i2c_port = kI2cPort;
    cfg.sda_io_num = kI2cSda;
    cfg.scl_io_num = kI2cScl;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.intr_priority = 0;
    cfg.trans_queue_depth = 0;
    cfg.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&cfg, &s_i2c);
}

static void IRAM_ATTR touch_intr_isr(void *arg)
{
    (void)arg;
    // Only latch the bounded flag; any I2C/GPIO work stays in the task lane.
    portENTER_CRITICAL_ISR(&s_touch_irq_mux);
    s_touch_irq_pending = true;
    portEXIT_CRITICAL_ISR(&s_touch_irq_mux);
}

static bool touch_irq_consume()
{
    bool pending = false;
    portENTER_CRITICAL(&s_touch_irq_mux);
    pending = s_touch_irq_pending;
    s_touch_irq_pending = false;
    portEXIT_CRITICAL(&s_touch_irq_mux);
    return pending;
}

esp_err_t init_touch()
{
    // RST: active-low output. Pulse low 200 ms then high 200 ms. The chip is
    // deliberately NOT probed or ID-read afterwards: the first touch arrives
    // via INT, and an init probe would consume the event window.
    gpio_config_t rst_cfg{};
    rst_cfg.pin_bit_mask = 1ULL << kTouchReset;
    rst_cfg.mode = GPIO_MODE_OUTPUT;
    rst_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    rst_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&rst_cfg), kTag, "touch RST GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kTouchReset, 0), kTag, "touch RST low failed");
    vTaskDelay(pdMS_TO_TICKS(kTouchResetLowMs));
    ESP_RETURN_ON_ERROR(gpio_set_level(kTouchReset, 1), kTag, "touch RST high failed");
    vTaskDelay(pdMS_TO_TICKS(kTouchResetHighMs));

    // The controller can leave the shared lines mid-cycle as reset releases.
    // Recover the idle bus once during board startup, before either device is
    // configured; touch report failures never reset this shared bus at runtime.
    ESP_RETURN_ON_ERROR(i2c_master_bus_reset(s_i2c), kTag,
                        "I2C recovery after touch reset failed");

    // INT: input, idle-high; a touch drives a falling edge that latches the
    // flag. The GPIO ISR service is installed exactly once, here, on the
    // board init path (no other component uses it in this firmware).
    gpio_config_t int_cfg{};
    int_cfg.pin_bit_mask = 1ULL << kTouchInt;
    int_cfg.mode = GPIO_MODE_INPUT;
    int_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    int_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    int_cfg.intr_type = GPIO_INTR_NEGEDGE;
    ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), kTag, "touch INT GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_IRAM), kTag,
                        "touch ISR service install failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(kTouchInt, touch_intr_isr, nullptr),
                        kTag, "touch INT handler add failed");

    // Second device handle on the existing 100 kHz bus; no probe or ID read.
    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = kTouchI2cAddr;
    dev_cfg.scl_speed_hz = kMotionI2cHz;
    dev_cfg.scl_wait_us = kMotionSclWaitUs;  // same stale-SCL guard as the IMU
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c, &dev_cfg, &s_touch_dev),
                        kTag, "touch device add failed");

    ESP_LOGI(kTag, "CST816 touch ready at 0x%02X (RST GPIO47, INT GPIO48)",
             kTouchI2cAddr);
    return ESP_OK;
}

esp_err_t init_imu()
{
    uint8_t address = 0;
    if (i2c_master_probe(s_i2c, QMI8658_ADDRESS_HIGH, kI2cTimeoutMs) == ESP_OK) {
        address = QMI8658_ADDRESS_HIGH;
    } else if (i2c_master_probe(s_i2c, QMI8658_ADDRESS_LOW, kI2cTimeoutMs) == ESP_OK) {
        address = QMI8658_ADDRESS_LOW;
    } else {
        ESP_LOGE(kTag, "QMI8658 not found at 0x%02X or 0x%02X",
                 QMI8658_ADDRESS_HIGH, QMI8658_ADDRESS_LOW);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(qmi8658_init(&s_imu, s_i2c, address), kTag,
                        "QMI8658 init failed");
    ESP_RETURN_ON_ERROR(qmi8658_set_accel_range(&s_imu, QMI8658_ACCEL_RANGE_4G), kTag,
                        "QMI8658 accel range failed");
    ESP_RETURN_ON_ERROR(qmi8658_set_accel_odr(&s_imu, QMI8658_ACCEL_ODR_250HZ), kTag,
                        "QMI8658 accel ODR failed");
    ESP_RETURN_ON_ERROR(qmi8658_set_gyro_range(&s_imu, QMI8658_GYRO_RANGE_256DPS), kTag,
                        "QMI8658 gyro range failed");
    ESP_RETURN_ON_ERROR(qmi8658_set_gyro_odr(&s_imu, QMI8658_GYRO_ODR_250HZ), kTag,
                        "QMI8658 gyro ODR failed");
    qmi8658_set_accel_unit_mps2(&s_imu, true);
    qmi8658_set_gyro_unit_rads(&s_imu, true);

    // Rebind with conservative timing for the enclosed board's shared I2C bus.
    // The extended clock-stretch guard prevents a peripheral-held SCL interval
    // from aborting a sample while the 40 MHz display DMA is active.
    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(s_imu.dev_handle), kTag,
                        "QMI8658 device rebind remove failed");
    s_imu.dev_handle = nullptr;
    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = kMotionI2cHz;
    dev_cfg.scl_wait_us = kMotionSclWaitUs;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c, &dev_cfg, &s_imu.dev_handle), kTag,
                        "QMI8658 device rebind add failed");

    ESP_LOGI(kTag,
             "QMI8658 ready at 0x%02X (4G/256dps, 250 Hz ODR, 100 kHz I2C, 10 ms SCL guard)",
             address);
    return ESP_OK;
}

esp_err_t init_display(BoardHandles *out)
{
    gpio_config_t bl_cfg{};
    bl_cfg.pin_bit_mask = 1ULL << kLcdBacklight;
    bl_cfg.mode = GPIO_MODE_OUTPUT;
    bl_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    bl_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    bl_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&bl_cfg), kTag, "LCD backlight GPIO failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kLcdBacklight, 0), kTag,
                        "LCD backlight off failed");

    spi_bus_config_t bus_cfg{};
    bus_cfg.sclk_io_num = kLcdSclk;
    bus_cfg.mosi_io_num = kLcdMosi;
    bus_cfg.miso_io_num = GPIO_NUM_NC;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;
    bus_cfg.max_transfer_sz = kLcdMaxTransferBytes;
    ESP_RETURN_ON_ERROR(spi_bus_initialize(kLcdHost, &bus_cfg, SPI_DMA_CH_AUTO), kTag,
                        "LCD SPI bus init failed");

    esp_lcd_panel_io_spi_config_t io_cfg{};
    io_cfg.cs_gpio_num = kLcdCs;
    io_cfg.dc_gpio_num = kLcdDc;
    io_cfg.spi_mode = 3;
    io_cfg.pclk_hz = kLcdPixelClockHz;
    io_cfg.trans_queue_depth = 2;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(kLcdHost),
                                 &io_cfg, &out->io),
        kTag, "ST7789 panel I/O create failed");

    esp_lcd_panel_dev_config_t panel_cfg{};
    panel_cfg.reset_gpio_num = kLcdReset;
    panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_cfg.bits_per_pixel = 16;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(out->io, &panel_cfg, &out->panel),
                        kTag, "ST7789 panel create failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(out->panel), kTag, "ST7789 reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(out->panel), kTag, "ST7789 init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(out->panel, true), kTag,
                        "ST7789 invert failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(out->panel, true), kTag,
                        "ST7789 display-on failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(kLcdBacklight, 1), kTag,
                        "LCD backlight on failed");

    ESP_LOGI(kTag, "ST7789 ready: 240x240 RGB565, SPI 40 MHz");
    return ESP_OK;
}

}  // namespace

esp_err_t board_init(BoardHandles *out)
{
    if (out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = BoardHandles{};

    ESP_LOGI(kTag, "Waveshare ESP32-S3-Touch-LCD-1.54 board init");
    ESP_RETURN_ON_ERROR(init_power_hold(), kTag, "power hold failed");
    ESP_RETURN_ON_ERROR(init_buttons(), kTag, "button GPIO init failed");
    ESP_RETURN_ON_ERROR(init_i2c(), kTag, "sensor I2C init failed");
    ESP_RETURN_ON_ERROR(init_touch(), kTag, "touch init failed");
    ESP_RETURN_ON_ERROR(init_imu(), kTag, "IMU init failed");
    ESP_RETURN_ON_ERROR(init_display(out), kTag, "display init failed");
    return ESP_OK;
}

esp_err_t board_read_motion(Vec3 *accel_mps2, Vec3 *gyro_rads, bool *fresh)
{
    if (accel_mps2 == nullptr || gyro_rads == nullptr || fresh == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *fresh = false;

    // AX_L through GZ_H are contiguous. One transaction is both cheaper and
    // more coherent than separate accel/gyro reads from the registry helper.
    // The enclosed board can transiently NACK while it is shaken; retry in
    // place, then clear the bus once before dropping the sample.
    uint8_t reg = QMI8658_AX_L;
    uint8_t raw[12]{};
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < 3 && ret != ESP_OK; ++attempt) {
        ret = i2c_master_transmit_receive(
            s_imu.dev_handle, &reg, 1, raw, sizeof(raw), kMotionReadTimeoutMs);
        if (ret != ESP_OK) {
            esp_rom_delay_us(200);
        }
    }
    if (ret != ESP_OK) {
        ESP_RETURN_ON_ERROR(i2c_master_bus_reset(s_i2c), kTag, "sensor I2C reset failed");
        esp_rom_delay_us(200);
        ret = i2c_master_transmit_receive(
            s_imu.dev_handle, &reg, 1, raw, sizeof(raw), kMotionReadTimeoutMs);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    const auto le_i16 = [&raw](size_t offset) {
        return static_cast<int16_t>(
            static_cast<uint16_t>(raw[offset]) |
            (static_cast<uint16_t>(raw[offset + 1]) << 8));
    };

    accel_mps2->x = static_cast<float>(le_i16(0)) * kAccelScaleMps2;
    accel_mps2->y = static_cast<float>(le_i16(2)) * kAccelScaleMps2;
    accel_mps2->z = static_cast<float>(le_i16(4)) * kAccelScaleMps2;
    gyro_rads->x = static_cast<float>(le_i16(6)) * kGyroScaleRads;
    gyro_rads->y = static_cast<float>(le_i16(8)) * kGyroScaleRads;
    gyro_rads->z = static_cast<float>(le_i16(10)) * kGyroScaleRads;
    *fresh = true;
    return ESP_OK;
}

esp_err_t board_read_touch(TouchSample *out)
{
    if (out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    out->fresh = false;
    out->pressed = false;
    out->gesture = TouchGesture::None;

    // Consume the falling-edge latch, with the active-low pin as a level
    // fallback when a short scheduler/critical-section window hid the edge.
    // A high idle line performs no I2C transaction.
    const bool interrupt_pending = touch_irq_consume();
    if (!interrupt_pending && gpio_get_level(kTouchInt) != 0) {
        return ESP_OK;
    }

    // Report registers 0x01..0x06 = {gesture id, finger count, X high nibble,
    // X low, Y high nibble, Y low}.
    uint8_t reg = kTouchReportReg;
    uint8_t raw[kTouchReportLen]{};
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < kTouchReadAttempts && ret != ESP_OK; ++attempt) {
        // No delay between attempts: the part may NACK once the event window
        // closes, so retry strictly immediately. The shared bus is never
        // reset or recreated from the touch path.
        ret = i2c_master_transmit_receive(s_touch_dev, &reg, 1, raw,
                                          sizeof(raw), kMotionReadTimeoutMs);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    // The gesture id accompanies every report; the finger-up report carries
    // the completed contact's swipe, which is what swipe classification uses.
    out->gesture = cst816_gesture(raw[0]);

    const uint8_t finger = raw[1] & kTouchFingerMask;
    if (finger == 0) {
        // Publish the release report (and its gesture) without coordinates so
        // the shell can re-arm exactly once for the next physical contact.
        out->fresh = true;
        return ESP_OK;
    }

    const uint16_t x = (static_cast<uint16_t>(raw[2] & kTouchCoordHighMask) << 8) |
                       static_cast<uint16_t>(raw[3]);
    const uint16_t y = (static_cast<uint16_t>(raw[4] & kTouchCoordHighMask) << 8) |
                       static_cast<uint16_t>(raw[5]);

    // Never publish partial data: reject out-of-range reports outright.
    if (x > kTouchMaxCoord || y > kTouchMaxCoord) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->x = x;
    out->y = y;
    out->pressed = true;
    out->fresh = true;
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
    ESP_RETURN_ON_ERROR(gpio_hold_dis(kBatteryEnable), kTag, "BAT_EN hold release failed");
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

}  // namespace fluid_demo
