// distance_lcd — combines lab6 (SHTC3 temp sensor + HC-SR04/RCWL-1601
// ultrasonic distance) and lab3 (DFRobot RGB LCD1602) into a distance meter
// that displays live readings on the LCD.
//
// The ESP32-C3 has only ONE hardware I2C peripheral, and the LCD needs it,
// so — same trick as lab3_3 — the LCD gets the hardware I2C bus and the
// SHTC3 gets its own bit-banged software I2C bus on separate pins.
//
// Wiring:
//   LCD (DFRobot RGB LCD1602): I2C_NUM_0, SCL=GPIO0, SDA=GPIO1
//   SHTC3: software I2C, SCL=GPIO8, SDA=GPIO10
//   Ultrasonic (HC-SR04 or RCWL-1601): TRIG=GPIO4, ECHO=GPIO5
//     - RCWL-1601: power from 3V3, ECHO connects directly to GPIO5.
//     - HC-SR04: power from 5V/VIN, ECHO needs a ~1k/2k divider down to
//       3.3V before GPIO5 (5V is not safe for the ESP32-C3's GPIOs).

#include <stdio.h>
#include <math.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
}

#include "DFRobot_RGBLCD1602.h"

static const char *TAG = "DISTANCE_LCD";

/* -------------------------------------------------------------------------- */
/*   Software (bit-banged) I2C for the SHTC3, on its own GPIO8/GPIO10 wires. */
/*   The ESP32-C3 has only ONE hardware I2C peripheral, already used by the  */
/*   LCD on I2C_NUM_0 (GPIO0/GPIO1) - so the SHTC3 bus is driven manually,   */
/*   completely independent of the LCD's bus/pull-ups.                      */
/* -------------------------------------------------------------------------- */

#define SHTC3_SENSOR_ADDR        0x70
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
    soft_i2c_sda_high();

    for (int i = 0; i < 8; i++) {
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
        soft_i2c_scl_high();
        esp_rom_delay_us(SOFT_I2C_DELAY_US);
        byte = (byte << 1) | (soft_i2c_sda_read() ? 1 : 0);
        soft_i2c_scl_low();
    }

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
    if (!soft_i2c_write_byte((uint8_t)(addr << 1))) {
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
    if (!soft_i2c_write_byte((uint8_t)((addr << 1) | 1))) {
        soft_i2c_stop();
        return ESP_FAIL;
    }
    for (size_t i = 0; i < len; i++) {
        data[i] = soft_i2c_read_byte(i < len - 1);
    }
    soft_i2c_stop();
    return ESP_OK;
}

#define SHTC3_CMD_WAKEUP         0x3517
#define SHTC3_CMD_SLEEP          0xB098
#define SHTC3_CMD_MEAS_T_RH      0x7CA2

static uint8_t shtc3_crc(const uint8_t *d, int length)
{
    uint8_t mark = 0xFF;
    for (int j = 0; j < length; j++) {
        mark ^= d[j];
        for (int bit = 0; bit < 8; bit++) {
            if (mark & 0x80) {
                mark = static_cast<uint8_t>(mark << 1) ^ 0x31;
            } else {
                mark <<= static_cast<uint8_t>(1);
            }
        }
    }
    return mark;
}

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
    esp_err_t err = soft_i2c_read_from_device(SHTC3_SENSOR_ADDR, read_buffer, sizeof(read_buffer));
    if (err != ESP_OK) {
        return err;
    }

    uint8_t temp_bytes[2] = { read_buffer[0], read_buffer[1] };
    uint8_t temp_crc      = read_buffer[2];
    uint8_t hum_bytes[2]  = { read_buffer[3], read_buffer[4] };
    uint8_t hum_crc       = read_buffer[5];

    if (shtc3_crc(temp_bytes, 2) != temp_crc) return ESP_FAIL;
    if (shtc3_crc(hum_bytes, 2) != hum_crc) return ESP_FAIL;

    *temp_raw = ((uint16_t)temp_bytes[0] << 8) | temp_bytes[1];
    *hum_raw  = ((uint16_t)hum_bytes[0] << 8) | hum_bytes[1];
    return ESP_OK;
}

static esp_err_t shtc3_measure(float *temp_c, float *hum_pct)
{
    esp_err_t err = shtc3_write_cmd(SHTC3_CMD_WAKEUP);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(1));

    err = shtc3_write_cmd(SHTC3_CMD_MEAS_T_RH);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(30));

    uint16_t raw_temp = 0, raw_hum = 0;
    err = shtc3_read_raw(&raw_temp, &raw_hum);
    if (err != ESP_OK) return err;

    if (temp_c)  *temp_c  = -45.0f + 175.0f * ((float)raw_temp / 65536.0f);
    if (hum_pct) *hum_pct = 100.0f * ((float)raw_hum / 65536.0f);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                    Ultrasonic distance (from lab6_1)                       */
/* -------------------------------------------------------------------------- */

#define ULTRA_TRIG_GPIO  GPIO_NUM_4
#define ULTRA_ECHO_GPIO  GPIO_NUM_5

static void ultrasonic_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRA_TRIG_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(ULTRA_TRIG_GPIO, 0);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRA_ECHO_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Ultrasonic pins initialized (TRIG=%d, ECHO=%d)",
             ULTRA_TRIG_GPIO, ULTRA_ECHO_GPIO);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static bool ultrasonic_measure_cm(float temp_c, float *distance_cm)
{
    gpio_set_level(ULTRA_TRIG_GPIO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(ULTRA_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRA_TRIG_GPIO, 0);

    int64_t t0 = esp_timer_get_time();
    const int64_t timeout_us = 30000;

    while (gpio_get_level(ULTRA_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - t0 > timeout_us) return false;
    }
    int64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(ULTRA_ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - echo_start > timeout_us) return false;
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t pulse_us = echo_end - echo_start;
    if (pulse_us <= 0) return false;

    // Speed of sound as a function of temperature: c(T) = 331.3 + 0.606*T [m/s]
    float c = 331.3f + 0.606f * temp_c;
    float d_cm = 0.00005f * c * (float)pulse_us;

    if (distance_cm) *distance_cm = d_cm;
    return true;
}

/* -------------------------------------------------------------------------- */
/*                                app_main                                    */
/* -------------------------------------------------------------------------- */

extern "C" void app_main(void)
{
    // RGBAddr = 0x2D, LCD on I2C_NUM_0 (SDA=1, SCL=0) — same wiring as lab3_3.
    DFRobot_RGBLCD1602 lcd(/*RGBAddr*/ 0x2D, /*cols*/ 16, /*rows*/ 2,
                           I2C_NUM_0, LCD_ADDRESS);
    lcd.init();
    lcd.setBacklight(true);
    lcd.setColorWhite();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printstr("Distance Meter");
    lcd.setCursor(0, 1);
    lcd.printstr("Initializing...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    shtc3_i2c_bus_init();
    ultrasonic_init();

    float last_temp_c = 25.0f;  // fallback if SHTC3 read fails

    while (true) {
        float temp_c = last_temp_c;
        float hum = 0.0f;

        esp_err_t err = shtc3_measure(&temp_c, &hum);
        if (err == ESP_OK) {
            last_temp_c = temp_c;
        } else {
            temp_c = last_temp_c;
        }

        float distance_cm = 0.0f;
        bool ok = ultrasonic_measure_cm(temp_c, &distance_cm);

        char line1[17];
        char line2[17];
        if (ok) {
            snprintf(line1, sizeof(line1), "Dist:%6.1fcm", distance_cm);
        } else {
            snprintf(line1, sizeof(line1), "Dist: ERROR");
        }
        snprintf(line2, sizeof(line2), "Temp:%5.1fC", temp_c);

        ESP_LOGI(TAG, "%s | %s", line1, line2);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.printstr(line1);
        lcd.setCursor(0, 1);
        lcd.printstr(line2);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
