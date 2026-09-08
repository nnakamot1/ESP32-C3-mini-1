// lab3_3.cpp
#include <stdio.h>
#include <math.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
}

#include "DFRobot_RGBLCD1602.h"

static const char *TAG = "SHTC3_LCD";

// SHTC3 (7-bit) address
#define SHTC3_SENSOR_ADDR        0x70
#define I2C_MASTER_TIMEOUT_MS    1000

/* -------------------------------------------------------------------------- */
/*   Software (bit-banged) I2C for the SHTC3, on its own GPIO8/GPIO10 wires. */
/*   The ESP32-C3 has only ONE hardware I2C peripheral, already used by the  */
/*   LCD on I2C_NUM_0 (GPIO0/GPIO1) - so the SHTC3 bus is driven manually,   */
/*   completely independent of the LCD's bus/pull-ups.                      */
/* -------------------------------------------------------------------------- */
#define SHTC3_I2C_SCL_IO         GPIO_NUM_8
#define SHTC3_I2C_SDA_IO         GPIO_NUM_10
#define SOFT_I2C_DELAY_US        5   // ~100 kHz

static inline void soft_i2c_sda_high(void) { gpio_set_level(SHTC3_I2C_SDA_IO, 1); }
static inline void soft_i2c_sda_low(void)  { gpio_set_level(SHTC3_I2C_SDA_IO, 0); }
static inline void soft_i2c_scl_high(void) { gpio_set_level(SHTC3_I2C_SCL_IO, 1); }
static inline void soft_i2c_scl_low(void)  { gpio_set_level(SHTC3_I2C_SCL_IO, 0); }
static inline int  soft_i2c_sda_read(void) { return gpio_get_level(SHTC3_I2C_SDA_IO); }

static void shtc3_i2c_bus_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SHTC3_I2C_SDA_IO) | (1ULL << SHTC3_I2C_SCL_IO);
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; // open-drain: write 1 = release/float high
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    soft_i2c_sda_high();
    soft_i2c_scl_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);

    ESP_LOGI(TAG, "SHTC3 software I2C bus initialized (SDA=%d, SCL=%d)",
             SHTC3_I2C_SDA_IO, SHTC3_I2C_SCL_IO);
}

static void soft_i2c_start(void)
{
    soft_i2c_sda_high();
    soft_i2c_scl_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_sda_low();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_scl_low();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
}

static void soft_i2c_stop(void)
{
    soft_i2c_sda_low();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_scl_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_sda_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
}

// Returns true if the slave ACKed.
static bool soft_i2c_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) soft_i2c_sda_high(); else soft_i2c_sda_low();
        byte <<= 1;
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
        soft_i2c_scl_high();
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
        soft_i2c_scl_low();
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
    }

    // Release SDA so the slave can pull it low for ACK.
    soft_i2c_sda_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_scl_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    bool ack = (soft_i2c_sda_read() == 0);
    soft_i2c_scl_low();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    return ack;
}

static uint8_t soft_i2c_read_byte(bool ack)
{
    uint8_t byte = 0;
    soft_i2c_sda_high(); // release so slave can drive

    for (int i = 0; i < 8; i++) {
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
        soft_i2c_scl_high();
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
        byte = (byte << 1) | (soft_i2c_sda_read() ? 1 : 0);
        soft_i2c_scl_low();
    }

    // Send ACK (0) to request more bytes, or NACK (1) after the last byte.
    if (ack) soft_i2c_sda_low(); else soft_i2c_sda_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_scl_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);
    soft_i2c_scl_low();
    soft_i2c_sda_high();
    esp_rom_delay_us(SOFT_I2C_DELAY_US);

    return byte;
}

static esp_err_t soft_i2c_write_to_device(uint8_t addr, const uint8_t *data, size_t len)
{
    soft_i2c_start();
    if (!soft_i2c_write_byte((uint8_t)(addr << 1))) { // write bit = 0
        soft_i2c_stop();
        return ESP_FAIL;
    }
    for (size_t i = 0; i < len; i++) {
        if (!soft_i2c_write_byte(data[i])) {
            soft_i2c_stop();
            return ESP_FAIL;
        }
    }
    soft_i2c_stop();
    return ESP_OK;
}

static esp_err_t soft_i2c_read_from_device(uint8_t addr, uint8_t *data, size_t len)
{
    soft_i2c_start();
    if (!soft_i2c_write_byte((uint8_t)((addr << 1) | 1))) { // read bit = 1
        soft_i2c_stop();
        return ESP_FAIL;
    }
    for (size_t i = 0; i < len; i++) {
        data[i] = soft_i2c_read_byte(i < len - 1); // ACK all but the last byte
    }
    soft_i2c_stop();
    return ESP_OK;
}

// SHTC3 commands
#define SHTC3_CMD_WAKEUP         0x3517
#define SHTC3_CMD_SLEEP          0xB098
// Measure T first, RH second, normal mode, clock stretching disabled
#define SHTC3_CMD_MEAS_T_RH      0x7CA2

// Forward declaration for CRC
static uint8_t shtc3_crc(const uint8_t *d, int length);

/* -------------------------------------------------------------------------- */
/*                         CRC (same as your code)                            */
/* -------------------------------------------------------------------------- */
static uint8_t shtc3_crc(const uint8_t *d, int length)
{
    uint8_t mark = 0xFF;

    for (int j = 0; j < length; j++) {
        mark ^= d[j];

        for (int bitsman = 0; bitsman < 8; bitsman++) {
            if (mark & 0x80) {
                mark = static_cast<uint8_t>(mark << 1) ^ 0x31;
		///////////
		////
		///
		///`
            } else {
                mark <<= static_cast<uint8_t>(1);
            }
        }
    }
    return mark;
}

/* -------------------------------------------------------------------------- */
/*                     Low-level SHTC3 I2C helpers (bit-banged)               */
/*   Own bus: software I2C on SDA=10, SCL=8 - separate from the LCD's bus.   */
/* -------------------------------------------------------------------------- */

static esp_err_t shtc3_write_cmd(uint16_t cmd)
{
    uint8_t buf[2] = {
        static_cast<uint8_t>(cmd >> 8),
        static_cast<uint8_t>(cmd & 0xFF)
    };

    return soft_i2c_write_to_device(SHTC3_SENSOR_ADDR, buf, sizeof(buf));
}

static esp_err_t shtc3_read_raw(uint16_t *temp_raw, uint16_t *hum_raw)
{
    uint8_t read_buffer[6] = {0};

    esp_err_t err = soft_i2c_read_from_device(SHTC3_SENSOR_ADDR, read_buffer,
                                               sizeof(read_buffer));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read error: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t temp_bytes[2] = { read_buffer[0], read_buffer[1] };
    uint8_t temp_crc      = read_buffer[2];
    uint8_t hum_bytes[2]  = { read_buffer[3], read_buffer[4] };
    uint8_t hum_crc       = read_buffer[5];

    if (shtc3_crc(temp_bytes, 2) != temp_crc) {
        ESP_LOGE(TAG, "Temperature CRC mismatch");
        return ESP_FAIL;
    }
    if (shtc3_crc(hum_bytes, 2) != hum_crc) {
        ESP_LOGE(TAG, "Humidity CRC mismatch");
        return ESP_FAIL;
    }

    *temp_raw = ((uint16_t)temp_bytes[0] << 8) | temp_bytes[1];
    *hum_raw  = ((uint16_t)hum_bytes[0] << 8) | hum_bytes[1];

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                      High-level measurement helper                         */
/* -------------------------------------------------------------------------- */

static esp_err_t shtc3_measure(float *temp_c, float *hum_pct)
{
    esp_err_t err;

    // Wake up sensor
    err = shtc3_write_cmd(SHTC3_CMD_WAKEUP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send WAKEUP: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    // Trigger measurement
    err = shtc3_write_cmd(SHTC3_CMD_MEAS_T_RH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send MEAS command: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(30));  // allow enough time (~12ms datasheet)

    uint16_t raw_temp = 0;
    uint16_t raw_hum  = 0;
    err = shtc3_read_raw(&raw_temp, &raw_hum);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "shtc3_read_raw failed: %s", esp_err_to_name(err));
        return err;
    }

    float t_c = -45.0f + 175.0f * ((float)raw_temp / 65536.0f);
    float h   = 100.0f * ((float)raw_hum  / 65536.0f);

    if (temp_c)  *temp_c  = t_c;
    if (hum_pct) *hum_pct = h;

    // Not sleeping between reads: polling every second on the bit-banged bus,
    // and the SLEEP/WAKEUP cycle was interfering with the next measurement.
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                               app_main                                     */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    // IMPORTANT: DFRobot_RGBLCD1602.cpp must be configured with:
    // #define I2C_MASTER_SCL_IO GPIO_NUM_0
    // #define I2C_MASTER_SDA_IO GPIO_NUM_1
    // and it calls i2c_param_config / i2c_driver_install on I2C_NUM_0.

    // RGBAddr = 0x2D (you said this works for your backlight)
    DFRobot_RGBLCD1602 lcd(/*RGBAddr*/ 0x2D, /*cols*/ 16, /*rows*/ 2,
                           I2C_NUM_0, LCD_ADDRESS);

    // Initialize LCD + I2C bus (this sets up I2C_NUM_0 on pins 0/1)
    lcd.init();
    lcd.setBacklight(true);
    lcd.setColorWhite();  // full white backlight

    // SHTC3 gets its own separate software (bit-banged) I2C bus on GPIO8/GPIO10 -
    // not shared with the LCD's hardware I2C_NUM_0 bus.
    shtc3_i2c_bus_init();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printstr("SHTC3 Monitor");
    lcd.setCursor(0, 1);
    lcd.printstr("Initializing...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // One-time bus scan to confirm what's actually answering on the SHTC3 bus
    ESP_LOGI(TAG, "Scanning SHTC3 I2C bus...");
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        uint8_t dummy = 0x00;
        esp_err_t probe = soft_i2c_write_to_device(addr, &dummy, 1);
        if (probe == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "Scan done.");

    while (true) {
        float temp_c = 0.0f;
        float hum    = 0.0f;

        esp_err_t err = shtc3_measure(&temp_c, &hum);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "T: %.2f C, H: %.2f %%", temp_c, hum);

            char line1[17];
            char line2[17];

            // 1st line: temperature in Celsius
            snprintf(line1, sizeof(line1), "T:%5.1f C", temp_c);
            // 2nd line: humidity
            snprintf(line2, sizeof(line2), "H:%5.1f %%RH", hum);

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.printstr(line1);
            lcd.setCursor(0, 1);
            lcd.printstr(line2);
        } else {
            ESP_LOGE(TAG, "SHTC3 read failed: %s", esp_err_to_name(err));
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.printstr("SHTC3 ERROR");
            lcd.setCursor(0, 1);
            lcd.printstr("Check sensor");
        }

        // Update once per second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
