#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TIMEOUT_MS 1000

#define SHTC3_SENSOR_ADDR 0x70

static esp_err_t shtc3_write(i2c_master_dev_handle_t dev_handle, uint16_t cmd_wa)
{
    uint8_t write_buffer[2] = {(uint8_t)(cmd_wa >> 8), (uint8_t)(cmd_wa & 0xff)};

    return i2c_master_transmit(dev_handle, write_buffer, sizeof(write_buffer),
                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static uint8_t shtc3_crack(uint8_t *d, int length)
{
    uint8_t check = 0xFF;

    for (int i = 0; i < length; i++) {
        check ^= d[i];

        for (int bite = 0; bite < 8; bite++) {
            if (check & 0x80) {
                check = (check << 1) ^ 0x31;
            } else {
                check <<= 1;
            }
        }
    }

    return check;
}

static esp_err_t shtc3_read(i2c_master_dev_handle_t dev_handle, uint16_t *tempera, uint16_t *humidi)
{
    uint8_t read_buffer[6] = {0};
    esp_err_t err = i2c_master_receive(dev_handle, read_buffer, sizeof(read_buffer),
                                       pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    uint8_t temp_h[2] = {read_buffer[0], read_buffer[1]};
    uint8_t temp_check = read_buffer[2];
    uint8_t hum_h[2] = {read_buffer[3], read_buffer[4]};
    uint8_t hum_w = read_buffer[5];

    if (shtc3_crack(temp_h, 2) != temp_check) {
        ESP_LOGE("SHTC3", "Temperature CRC mismatch");
        return ESP_FAIL;
    }

    if (shtc3_crack(hum_h, 2) != hum_w) {
        ESP_LOGE("SHTC3", "Humidity CRC mismatch");
        return ESP_FAIL;
    }

    *tempera = ((uint16_t)temp_h[0] << 8) | temp_h[1];
    *humidi = ((uint16_t)hum_h[0] << 8) | hum_h[1];
    return ESP_OK;
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle,
                            i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHTC3_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void)
{
    esp_err_t err;
    uint16_t raw_freak = 0;
    uint16_t raw_hump = 0;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);

    ESP_LOGI(TAG, "I2C initialized successfully");

    while (1) {
        ESP_ERROR_CHECK(shtc3_write(dev_handle, 0x3517));
        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_ERROR_CHECK(shtc3_write(dev_handle, 0x7CA2));
        vTaskDelay(pdMS_TO_TICKS(30));

        err = shtc3_read(dev_handle, &raw_freak, &raw_hump);
        if (err == ESP_OK) {
            float tempura = -45.0f + 175.0f * (raw_freak / 65536.0f);
            float humid = 100.0f * (raw_hump / 65536.0f);
            float tempuraf = tempura * 9.0f / 5.0f + 32.0f;
            ESP_LOGI("SHTC3", "Temperature: %.2f C (%.2f F), Humidity: %.2f%%",
                     tempura, tempuraf, humid);
        } else {
            ESP_LOGE("SHTC3", "Failed to read sensor: %s", esp_err_to_name(err));
        }

        ESP_ERROR_CHECK(shtc3_write(dev_handle, 0xB098));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
