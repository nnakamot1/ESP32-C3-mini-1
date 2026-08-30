#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

// ===================== I2C / SHTC3 SECTION =====================

static const char *TAG = "LAB6";

// I2C pins
#define I2C_MASTER_SCL_IO      8
#define I2C_MASTER_SDA_IO      10
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     400000

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS     1000

// SHTC3 address
#define SHTC3_SENSOR_ADDR      0x70

static esp_err_t shtc3_write(i2c_master_dev_handle_t dev_handle, uint16_t cmd_wa) {
    uint8_t write_buffer[2] = { (uint8_t)(cmd_wa >> 8), (uint8_t)(cmd_wa & 0xFF) };
    return i2c_master_transmit(dev_handle,
                               write_buffer,
                               sizeof(write_buffer),
                               pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static uint8_t shtc3_crack(uint8_t *d, int length) {
    uint8_t check = 0xFF;

    for (int i = 0; i < length; i++) {
        check ^= d[i];
        for (int bit = 0; bit < 8; bit++) {
            if (check & 0x80) {
                check = (check << 1) ^ 0x31;
            } else {
                check <<= 1;
            }
        }
    }
    return check;
}

static esp_err_t shtc3_read(i2c_master_dev_handle_t dev_handle,
                            uint16_t *tempera, uint16_t *humidi) {
    esp_err_t err;
    uint8_t read_buffer[6] = {0};

    err = i2c_master_receive(dev_handle,
                             read_buffer,
                             sizeof(read_buffer),
                             pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (err != ESP_OK) {
        return err;
    }

    uint8_t temp_h[2] = { read_buffer[0], read_buffer[1] };
    uint8_t temp_check = read_buffer[2];
    uint8_t hum_h[2]  = { read_buffer[3], read_buffer[4] };
    uint8_t hum_w     = read_buffer[5];

    if (shtc3_crack(temp_h, 2) != temp_check) {
        // CRC mismatch – just fail quietly
        return ESP_FAIL;
    }
    if (shtc3_crack(hum_h, 2) != hum_w) {
        // CRC mismatch – just fail quietly
        return ESP_FAIL;
    }

    *tempera = ((uint16_t)temp_h[0] << 8) | temp_h[1];
    *humidi  = ((uint16_t)hum_h[0] << 8) | hum_h[1];
    return ESP_OK;
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle,
                            i2c_master_dev_handle_t *dev_handle) {
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
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle,
                                              &dev_config,
                                              dev_handle));
}

// Reads SHTC3 and returns temperature in °C (and humidity in %)
static esp_err_t read_temp_hum(i2c_master_dev_handle_t dev_handle,
                               float *temp_c, float *humid_percent) {
    esp_err_t err;
    uint16_t raw_temp = 0;
    uint16_t raw_hum  = 0;

    // Wake up
    err = shtc3_write(dev_handle, 0x3517);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2));

    // Measurement command (clock stretching disabled, T first, RH second)
    err = shtc3_write(dev_handle, 0x7CA2);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(30));

    err = shtc3_read(dev_handle, &raw_temp, &raw_hum);
    if (err != ESP_OK) return err;

    float t = -45.0f + 175.0f * (raw_temp / 65536.0f);
    float h = 100.0f * (raw_hum / 65536.0f);

    if (temp_c)        *temp_c = t;
    if (humid_percent) *humid_percent = h;

    // Go back to sleep (ignore result)
    shtc3_write(dev_handle, 0xB098);

    return ESP_OK;
}

// ===================== ULTRASONIC SECTION =====================

// Choose your SR04 pins here:
#define ULTRA_TRIG_GPIO  4
#define ULTRA_ECHO_GPIO  5

static void ultrasonic_init(void) {
    gpio_config_t io_conf = {0};

    // TRIG as output, start LOW
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRA_TRIG_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(ULTRA_TRIG_GPIO, 0);

    // ECHO as input with pulldown (keeps it at 0 when idle/floating)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRA_ECHO_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    ESP_LOGI("ULTRA", "Ultrasonic pins initialized (TRIG=%d, ECHO=%d)",
             ULTRA_TRIG_GPIO, ULTRA_ECHO_GPIO);

    // Give sensor a bit of time after power-up
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Returns true if measurement succeeded, fills distance in cm
static bool ultrasonic_measure_cm(float temp_c, float *distance_cm) {
    // Send 10us pulse on TRIG
    gpio_set_level(ULTRA_TRIG_GPIO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(ULTRA_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRA_TRIG_GPIO, 0);

    int64_t t0 = esp_timer_get_time();
    const int64_t timeout_us = 30000; // 30 ms

    // Wait for ECHO HIGH
    while (gpio_get_level(ULTRA_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - t0 > timeout_us) {
            return false;
        }
    }
    int64_t echo_start = esp_timer_get_time();

    // Wait for ECHO LOW
    while (gpio_get_level(ULTRA_ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - echo_start > timeout_us) {
            return false;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t pulse_us = echo_end - echo_start;
    if (pulse_us <= 0) {
        return false;
    }

    // Speed of sound as a function of temperature (approx):
    // c(T) = 331.3 + 0.606 * T  [m/s]
    float c = 331.3f + 0.606f * temp_c;

    // Correct distance formula:
    // distance_cm = c[m/s] * (pulse_us * 1e-6[s]) / 2 * 100[cm/m]
    //             = 0.00005 * c * pulse_us
    float d_cm = 0.00005f * c * (float)pulse_us;

    if (distance_cm) {
        *distance_cm = d_cm;
    }

    return true;
}

// ===================== MAIN APP =====================

void app_main(void) {
    esp_err_t err;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    // Init I2C + SHTC3
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Init ultrasonic sensor
    ultrasonic_init();

    static float last_temp_c = 25.0f;  // fallback if sensor read fails

    while (1) {
        float temp_c = last_temp_c;
        float humid  = 0.0f;

        // Read temperature/humidity; if it fails, keep last_temp_c
        err = read_temp_hum(dev_handle, &temp_c, &humid);
        if (err == ESP_OK) {
            last_temp_c = temp_c;
        } else {
            temp_c = last_temp_c;
        }

        // Read distance (cm) using temperature-adjusted speed of sound
        float distance_cm = 0.0f;
        bool ok = ultrasonic_measure_cm(temp_c, &distance_cm);

        if (ok) {
            // This is what the lab spec wants
            printf("Distance: %.2f cm at %.2fC\n", distance_cm, temp_c);
        } else {
            printf("Distance: (measurement failed) at %.2fC\n", temp_c);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // once per second
    }
}
