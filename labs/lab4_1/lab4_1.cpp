// lab4_1.cpp — Orientation-based direction printer (ESP-IDF, C++)
// Uses ICM-42670-P accelerometer over I2C (direct register reads, no FIFO).
// Direction mapping (from gravity vector):
//   +Y => RIGHT, -Y => LEFT
//   -X => UP,    +X => DOWN
//
// Wiring: set I2C_SCL_IO / I2C_SDA_IO to your board pins.
// Build: C++ file with extern "C" app_main; uses old I2C master driver.

#include <cstdio>
#include <cstring>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

// ---------- Logging tag ----------
static const char* TAG = "LAB4_1";

// ---------- App tuning ----------
static constexpr float kAlpha      = 0.25f;  // IIR smoothing for angles/inputs
static constexpr float kEnter     = 0.14f;  // enter threshold (in "g" after norm)
static constexpr float kExit      = 0.10f;  // exit  threshold (must be < enter)

// ---------- I2C pins/freq (adjust to your board) ----------
#define I2C_PORT    I2C_NUM_0
#define I2C_SCL_IO  (gpio_num_t)8
#define I2C_SDA_IO  (gpio_num_t)10
#define I2C_FREQ_HZ 100000

// ---------- ICM-42670-P (User Bank 0) ----------
#define ICM_ADDR           0x68   // AD0 = 0
#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT0      0x1F
#define REG_ACCEL_CONFIG0  0x21
#define REG_ACCEL_CONFIG1  0x24
#define REG_ACCEL_X1       0x0B   // then X0, Y1, Y0, Z1, Z0

// ==========================================================
//  Thread-safe "raw" motion store (we use only ax, ay for dirs)
// ==========================================================
static constexpr float kMaxAbsG = 4.0f;
static float g_raw_ax = 0.0f;   // +X => (we map to DOWN)
static float g_raw_ay = 0.0f;   // +Y => RIGHT
static float g_raw_az = 1.0f;   // not strictly required for this app
static SemaphoreHandle_t g_motion_mtx = nullptr;

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static void motion_store_init() {
    g_motion_mtx = xSemaphoreCreateMutex();
    if (!g_motion_mtx) {
        ESP_LOGW(TAG, "xSemaphoreCreateMutex() failed; continuing without locking");
    }
}

static inline void motion_lock()   { if (g_motion_mtx) xSemaphoreTake(g_motion_mtx, portMAX_DELAY); }
static inline void motion_unlock() { if (g_motion_mtx) xSemaphoreGive(g_motion_mtx); }

// Set absolute values (in g)
static void write_motion_raw(float ax_g, float ay_g, float az_g) {
    ax_g = clampf(ax_g, -kMaxAbsG, kMaxAbsG);
    ay_g = clampf(ay_g, -kMaxAbsG, kMaxAbsG);
    az_g = clampf(az_g, -kMaxAbsG, kMaxAbsG);
    motion_lock();
    g_raw_ax = ax_g; g_raw_ay = ay_g; g_raw_az = az_g;
    motion_unlock();
}

static void read_motion_raw(float& ax_g, float& ay_g, float& az_g) {
    motion_lock();
    ax_g = g_raw_ax; ay_g = g_raw_ay; az_g = g_raw_az;
    motion_unlock();
}

// ==========================================================
//  I2C helpers (non-fatal; they log errors and return)
// ==========================================================
static esp_err_t i2c_init_port() {
    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = I2C_SDA_IO;
    cfg.scl_io_num = I2C_SCL_IO;
    cfg.sda_pullup_en = true;
    cfg.scl_pullup_en = true;
    cfg.master.clk_speed = I2C_FREQ_HZ;
    esp_err_t err;
    if ((err = i2c_param_config(I2C_PORT, &cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config err=%d", err); return err;
    }
    if ((err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0)) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install err=%d", err); return err;
    }
    return ESP_OK;
}

static esp_err_t icm_write_u8(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (ICM_ADDR<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, reg, true);
    i2c_master_write_byte(c, val, true);
    i2c_master_stop(c);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    if (err != ESP_OK) ESP_LOGE(TAG, "I2C W reg=0x%02X val=0x%02X err=%d", reg, val, err);
    return err;
}

static esp_err_t icm_read(uint8_t start_reg, uint8_t* buf, size_t n) {
    i2c_cmd_handle_t c = i2c_cmd_link_create();
    i2c_master_start(c);
    i2c_master_write_byte(c, (ICM_ADDR<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(c, start_reg, true);
    i2c_master_start(c);
    i2c_master_write_byte(c, (ICM_ADDR<<1) | I2C_MASTER_READ, true);
    if (n > 1) i2c_master_read(c, buf, n-1, I2C_MASTER_ACK);
    i2c_master_read_byte(c, buf+n-1, I2C_MASTER_NACK);
    i2c_master_stop(c);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, c, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(c);
    //if (err != ESP_OK) ESP_LOGE(TAG, "I2C R reg=0x%02X n=%u err=%d", start_reg, (unsigned)n, err);
    return err;
}

// ==========================================================
//  ICM-42670-P: minimal accelerometer init + read (direct)
// ==========================================================
static esp_err_t icm_init_accel_direct() {
    uint8_t who=0;
    if (icm_read(REG_WHO_AM_I, &who, 1) != ESP_OK) return ESP_FAIL;
    if (who != 0x67) {
        ESP_LOGW(TAG, "WHO_AM_I=0x%02X (expected 0x67) — continuing", who);
    }

    // PWR_MGMT0: ACCEL_MODE = 0b11 (LN), GYRO off (0)
    if (icm_write_u8(REG_PWR_MGMT0, 0x03) != ESP_OK) return ESP_FAIL;

    // ACCEL_CONFIG0: example ODR=200 Hz (0x8), FS=±2g (device default fits orientation use)
    if (icm_write_u8(REG_ACCEL_CONFIG0, 0x08) != ESP_OK) return ESP_FAIL;

    // Optional LPF if needed (kept default here)
    // icm_write_u8(REG_ACCEL_CONFIG1, 0x04);

    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_LOGI(TAG, "ICM accel init done (WHO=0x%02X)", who);
    return ESP_OK;
}

// Read accel XYZ directly (no exact scale needed — we normalize)
static bool icm_read_accel_direct(float& ax_g, float& ay_g, float& az_g) {
    uint8_t b[6];
    if (icm_read(REG_ACCEL_X1, b, sizeof(b)) != ESP_OK) return false;
    int16_t rx = (int16_t)((b[0]<<8)|b[1]);
    int16_t ry = (int16_t)((b[2]<<8)|b[3]);
    int16_t rz = (int16_t)((b[4]<<8)|b[5]);

    float fx = (float)rx, fy = (float)ry, fz = (float)rz;
    float mag = std::sqrt(fx*fx + fy*fy + fz*fz);
    if (mag < 1e-3f) return false;

    // Normalize to ~unit vector; “g” scale not needed for direction
    ax_g = fx / mag;
    ay_g = fy / mag;
    az_g = fz / mag;
    return true;
}

// ==========================================================
//  Direction logic (keep your hysteresis API)
// ==========================================================
struct tilt_dir_t { bool up=false, down=false, left=false, right=false; };

static tilt_dir_t decide_dirs(float ax, float ay, float high, float low, tilt_dir_t prev) {
    // ax = +Y (RIGHT/LEFT), ay = -X (UP/DOWN) — see sensor task mapping
    tilt_dir_t d = prev;

    // X → LEFT/RIGHT (we feed +Y here)
    if      (ax >  high) d.right = true,  d.left = false;
    else if (ax < -high) d.left  = true,  d.right = false;
    else if (ax > -low && ax < low) d.left = d.right = false;

    // Y → UP/DOWN (we feed -X here)
    if      (ay >  high) d.up   = true,  d.down = false;
    else if (ay < -high) d.down = true,  d.up   = false;
    else if (ay > -low && ay < low) d.up = d.down = false;

    return d;
}

static void print_dirs(const tilt_dir_t& d) {
    char msg[32] = {0};
    bool first = true;
    if (d.up)    { std::strcat(msg, "UP");    first=false; }
    if (d.down)  { if(!first) std::strcat(msg, " "); std::strcat(msg, "DOWN");  first=false; }
    if (d.left)  { if(!first) std::strcat(msg, " "); std::strcat(msg, "LEFT");  first=false; }
    if (d.right) { if(!first) std::strcat(msg, " "); std::strcat(msg, "RIGHT"); first=false; }
    if (!first) ESP_LOGI(TAG, "%s", msg);
}

// ==========================================================
//  Producer task: read accel and feed the store (~100 Hz)
// ==========================================================
static void sensor_task(void*) {
    ESP_LOGI(TAG, "sensor_task: start");
    if (i2c_init_port() != ESP_OK)        { ESP_LOGE(TAG, "I2C init failed"); vTaskDelete(nullptr); }
    if (icm_init_accel_direct() != ESP_OK){ ESP_LOGE(TAG, "ICM accel init failed"); vTaskDelete(nullptr); }

    uint32_t last_dbg = xTaskGetTickCount();
    while (true) {
        float ax, ay, az;
        if (icm_read_accel_direct(ax, ay, az)) {
            // Map for direction logic:
            //   RIGHT/LEFT needs +Y / -Y  → feed ax = +Y
            //   UP/DOWN   needs -X / +X  → feed ay = -X
            write_motion_raw(/*ax*/ ay, /*ay*/ -ax, /*az*/ az);
        }

        // Optional 1 Hz debug of normalized gravity vector:
        uint32_t now = xTaskGetTickCount();
        if (now - last_dbg >= pdMS_TO_TICKS(1000)) {
            float rax, ray, raz; read_motion_raw(rax, ray, raz);
            ESP_LOGI(TAG, "ACC norm ax=%.3f ay=%.3f az=%.3f", rax, ray, raz);
            last_dbg = now;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz
    }
}

// ==========================================================
//  Consumer loop: smooth + hysteresis + print changes
// ==========================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "app_main: start");
    ESP_LOGI(TAG, "Free heap (bytes): %u", (unsigned) xPortGetFreeHeapSize());

    motion_store_init();

    // Start the producer (sensor) task
    xTaskCreate(sensor_task, "sensor_task", 4096, nullptr, 5, nullptr);

    float ax_f = 0.0f, ay_f = 0.0f;
    tilt_dir_t prev{};

    // Heartbeat to confirm loop activity
    uint32_t last_beat = xTaskGetTickCount();

    while (true) {
        float ax, ay, az;
        read_motion_raw(ax, ay, az);   // values already mapped: ax=+Y, ay=-X

        // IIR smoothing (operates on already-mapped axes)
        ax_f = kAlpha*ax + (1.0f - kAlpha)*ax_f;
        ay_f = kAlpha*ay + (1.0f - kAlpha)*ay_f;

        tilt_dir_t now = decide_dirs(ax_f, ay_f, kEnter, kExit, prev);
        if (std::memcmp(&now, &prev, sizeof(now)) != 0) {
            print_dirs(now);
            prev = now;
        }

        // heartbeat once per second
        uint32_t t = xTaskGetTickCount();
        if (t - last_beat >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "HB ax=%.3f ay=%.3f | ax_f=%.3f ay_f=%.3f", ax, ay, ax_f, ay_f);
            last_beat = t;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // ~20 Hz decision/print
    }
}
