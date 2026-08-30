// lab3_3.cpp
#include <stdio.h>
#include <math.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
}

#include "DFRobot_RGBLCD1602.h"

static const char *TAG = "SHTC3_LCD";

// SHTC3 (7-bit) address
#define SHTC3_SENSOR_ADDR        0x70
#define I2C_MASTER_TIMEOUT_MS    1000

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
/*                     Low-level SHTC3 I2C helpers (legacy API)               */
/*   Uses same bus as LCD: I2C_NUM_0 on SDA=1, SCL=0, configured by LCD      */
/* -------------------------------------------------------------------------- */

static esp_err_t shtc3_write_cmd(uint16_t cmd)
{
    uint8_t buf[2] = {
        static_cast<uint8_t>(cmd >> 8),
        static_cast<uint8_t>(cmd & 0xFF)
    };

    return i2c_master_write_to_device(
        I2C_NUM_0,
        SHTC3_SENSOR_ADDR,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}

static esp_err_t shtc3_read_raw(uint16_t *temp_raw, uint16_t *hum_raw)
{
    uint8_t read_buffer[6] = {0};

    esp_err_t err = i2c_master_read_from_device(
        I2C_NUM_0,
        SHTC3_SENSOR_ADDR,
        read_buffer,
        sizeof(read_buffer),
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
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
//        ESP_LOGE(TAG, "Failed to send MEAS command: %s", esp_err_to_name(err));
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

    // Put sensor to sleep (optional)
    (void)shtc3_write_cmd(SHTC3_CMD_SLEEP);

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

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printstr("SHTC3 Monitor");
    lcd.setCursor(0, 1);
    lcd.printstr("Initializing...");
    vTaskDelay(pdMS_TO_TICKS(1000));

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
  //          ESP_LOGE(TAG, "SHTC3 read failed: %s", esp_err_to_name(err));
            lcd.clear();
            lcd.setCursor(0, 0);
   //         lcd.printstr("SHTC3 ERROR");
            lcd.setCursor(0, 1);
     //       lcd.printstr("Check sensor");
        }

        // Update once per second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
