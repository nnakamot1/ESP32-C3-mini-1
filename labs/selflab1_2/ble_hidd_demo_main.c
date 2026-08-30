#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "hid_dev.h"
#include "esp_timer.h"

/* ===== Renamed macros (from previous step) ===== */
#define APP_LOG_TAG               "HID_DEMO"

#define APP_I2C_PORT              I2C_NUM_0
#define APP_I2C_SDA_PIN           10
#define APP_I2C_SCL_PIN           8
#define APP_I2C_CLK_HZ            400000
#define APP_I2C_TIMEOUT_MS        1000

#define APP_ICM_ADDR              0x68
#define APP_ICM_REG_PWR_MGMT0     0x1F
#define APP_ICM_REG_ACCEL_CONFIG0 0x21
#define APP_ICM_REG_ACCEL_CONFIG1 0x24
#define APP_ICM_REG_ACCEL_DATA_X1 0x0B
#define APP_ICM_REG_ACCEL_DATA_X0 0x0C
#define APP_ICM_REG_ACCEL_DATA_Y1 0x0D
#define APP_ICM_REG_ACCEL_DATA_Y0 0x0E

#define APP_TILT_L                1000
#define APP_TILT_H                4000

#define APP_SPEED_SLOW            3
#define APP_SPEED_FAST            8

#define APP_ACCEL_MS_L1           0
#define APP_ACCEL_MS_L2           100
#define APP_ACCEL_MS_L3           500

#define APP_POLL_MS               10
#define APP_HOLDCLICK_MS          5000

/* Onboard BOOT button (active-low, has an external pull-up on most ESP32-C3 devkits) */
#define APP_BUTTON_GPIO           9
#define APP_BUTTON_DEBOUNCE_MS    200

/* size macro (unused, kept renamed) */
#define APP_CHAR_DECLARATION_SIZE (sizeof(uint8_t))

/* ===== Types (RENAMED) ===== */
typedef enum {
  TILT_NONE = 0,
  TILT_LEFT,
  TILT_RIGHT,
  TILT_UP,
  TILT_DOWN
} mouse_dir_t;

typedef struct {
  mouse_dir_t dir_x;
  mouse_dir_t dir_y;
  int64_t     x_tilt_start_time;
  int64_t     y_tilt_start_time;
  bool        is_x_tilted;
  bool        is_y_tilted;
  int64_t     stationary_start_time;
  bool        is_stationary;
  bool        click_triggered;
} mouse_pose_t;

/* ===== Globals (updated variable using new type name) ===== */
static uint16_t                g_conn_handle        = 0;
static bool                    g_is_secured         = false;
static i2c_master_bus_handle_t g_i2c_bus_handle     = NULL;
static i2c_master_dev_handle_t g_icm_device_handle  = NULL;
static mouse_pose_t            g_mouse_pose         = (mouse_pose_t){0};

#define APP_HIDD_DEVICE_NAME "SelfLab12-Mouse"

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

/* ===== Advertising data ===== */
static uint8_t g_hidd_uuid128[] = {
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
  0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t g_adv_data = {
  .set_scan_rsp        = false,
  .include_name        = true,
  .include_txpower     = true,
  .min_interval        = 0x0006,
  .max_interval        = 0x0010,
  .appearance          = 0x03c2,
  .manufacturer_len    = 0,
  .p_manufacturer_data = NULL,
  .service_data_len    = 0,
  .p_service_data      = NULL,
  .service_uuid_len    = sizeof(g_hidd_uuid128),
  .p_service_uuid      = g_hidd_uuid128,
  .flag                = 0x6,
};

static esp_ble_adv_params_t g_adv_params = {
  .adv_int_min       = 0x20,
  .adv_int_max       = 0x30,
  .adv_type          = ADV_TYPE_IND,
  .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
  .channel_map       = ADV_CHNL_ALL,
  .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ===== I2C helpers ===== */
static void i2c_bus_setup(void) {
  i2c_master_bus_config_t bus_cfg = {
    .i2c_port               = APP_I2C_PORT,
    .sda_io_num             = APP_I2C_SDA_PIN,
    .scl_io_num             = APP_I2C_SCL_PIN,
    .clk_source             = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt      = 7,
    .flags.enable_internal_pullup = true
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &g_i2c_bus_handle));

  i2c_device_config_t dev_cfg = {
    .device_address         = APP_ICM_ADDR,
    .scl_speed_hz           = APP_I2C_CLK_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &dev_cfg, &g_icm_device_handle));
}

static esp_err_t sensor_reg_write(uint8_t reg_addr, uint8_t data_byte) {
  uint8_t tx_bytes[2] = { reg_addr, data_byte };
  return i2c_master_transmit(g_icm_device_handle, tx_bytes, sizeof(tx_bytes),
                             pdMS_TO_TICKS(APP_I2C_TIMEOUT_MS));
}

static esp_err_t sensor_reg_read_burst(uint8_t start_reg_addr, uint8_t *rx_buf, size_t rx_len) {
  return i2c_master_transmit_receive(
    g_icm_device_handle,
    &start_reg_addr,
    1,
    rx_buf,
    rx_len,
    pdMS_TO_TICKS(APP_I2C_TIMEOUT_MS)
  );
}

static void accel_sensor_configure(void) {
  sensor_reg_write(APP_ICM_REG_PWR_MGMT0,     0x0B);
  sensor_reg_write(APP_ICM_REG_ACCEL_CONFIG0, 0x29);
  sensor_reg_write(APP_ICM_REG_ACCEL_CONFIG1, 0x03);
}

static int calculate_acceleration(int64_t tilt_duration_ms) {
  if (tilt_duration_ms >= APP_ACCEL_MS_L3) return 3;
  if (tilt_duration_ms >= APP_ACCEL_MS_L2) return 2;
  return 1;
}

/* ===== Button click ===== */
static void button_init(void) {
  gpio_config_t io_conf = {
    .intr_type    = GPIO_INTR_DISABLE,
    .mode         = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL << APP_BUTTON_GPIO),
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en   = GPIO_PULLUP_ENABLE,
  };
  gpio_config(&io_conf);
  ESP_LOGI(APP_LOG_TAG, "Button click on GPIO%d (BOOT button)", APP_BUTTON_GPIO);
}

/* Polls the button and returns true once per debounced press (falling edge). */
static bool button_poll_pressed(void) {
  static bool    was_pressed = false;
  static int64_t last_press_ms = 0;

  bool is_pressed = (gpio_get_level(APP_BUTTON_GPIO) == 0);
  bool fired = false;

  if (is_pressed && !was_pressed) {
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - last_press_ms >= APP_BUTTON_DEBOUNCE_MS) {
      fired = true;
      last_press_ms = now_ms;
    }
  }
  was_pressed = is_pressed;
  return fired;
}

/* ===== Motion -> mouse mapping ===== */
static esp_err_t read_accel_and_calculate_mouse(int8_t *mouse_dx, int8_t *mouse_dy, bool *click_out) {
  uint8_t raw_bytes[4];
  int16_t ax_raw = 0;
  int16_t ay_raw = 0;

  *click_out = false;

  esp_err_t err_read = sensor_reg_read_burst(APP_ICM_REG_ACCEL_DATA_X1, raw_bytes, 4);
  if (err_read != ESP_OK) {
    return err_read;
  }

  ax_raw = (int16_t)((raw_bytes[0] << 8) | raw_bytes[1]);
  ay_raw = (int16_t)((raw_bytes[2] << 8) | raw_bytes[3]);

  int64_t now_ms = esp_timer_get_time() / 1000;

  /* X axis */
  mouse_dir_t dir_next_x = TILT_NONE;
  int x_base = 0;

  if (ax_raw < -APP_TILT_H) {
    dir_next_x = TILT_LEFT;
    x_base     = -APP_SPEED_FAST;
  } else if (ax_raw < -APP_TILT_L) {
    dir_next_x = TILT_LEFT;
    x_base     = -APP_SPEED_SLOW;
  } else if (ax_raw > APP_TILT_H) {
    dir_next_x = TILT_RIGHT;
    x_base     =  APP_SPEED_FAST;
  } else if (ax_raw > APP_TILT_L) {
    dir_next_x = TILT_RIGHT;
    x_base     =  APP_SPEED_SLOW;
  }

  if (dir_next_x != TILT_NONE) {
    if (!g_mouse_pose.is_x_tilted || g_mouse_pose.dir_x != dir_next_x) {
      g_mouse_pose.dir_x            = dir_next_x;
      g_mouse_pose.x_tilt_start_time = now_ms;
      g_mouse_pose.is_x_tilted      = true;
    }
    int64_t held_x_ms = now_ms - g_mouse_pose.x_tilt_start_time;
    int      mult_x   = calculate_acceleration(held_x_ms);
    int      dx_tmp   = x_base * mult_x;
    if (dx_tmp > 127)  dx_tmp = 127;
    if (dx_tmp < -127) dx_tmp = -127;
    *mouse_dx = (int8_t)dx_tmp;
  } else {
    *mouse_dx = 0;
    g_mouse_pose.is_x_tilted = false;
  }

  /* Y axis */
  mouse_dir_t dir_next_y = TILT_NONE;
  int y_base = 0;

  if (ay_raw > APP_TILT_H) {
    dir_next_y = TILT_UP;
    y_base     = -APP_SPEED_FAST;
  } else if (ay_raw > APP_TILT_L) {
    dir_next_y = TILT_UP;
    y_base     = -APP_SPEED_SLOW;
  } else if (ay_raw < -APP_TILT_H) {
    dir_next_y = TILT_DOWN;
    y_base     =  APP_SPEED_FAST;
  } else if (ay_raw < -APP_TILT_L) {
    dir_next_y = TILT_DOWN;
    y_base     =  APP_SPEED_SLOW;
  }

  if (dir_next_y != TILT_NONE) {
    if (!g_mouse_pose.is_y_tilted || g_mouse_pose.dir_y != dir_next_y) {
      g_mouse_pose.dir_y            = dir_next_y;
      g_mouse_pose.y_tilt_start_time = now_ms;
      g_mouse_pose.is_y_tilted      = true;
    }
    int64_t held_y_ms = now_ms - g_mouse_pose.y_tilt_start_time;
    int      mult_y   = calculate_acceleration(held_y_ms);
    int      dy_tmp   = y_base * mult_y;
    if (dy_tmp > 127)  dy_tmp = 127;
    if (dy_tmp < -127) dy_tmp = -127;
    *mouse_dy = (int8_t)dy_tmp;
  } else {
    *mouse_dy = 0;
    g_mouse_pose.is_y_tilted = false;
  }

  /* Stationary -> click */
  bool is_still_now = (*mouse_dx == 0 && *mouse_dy == 0);

  if (is_still_now) {
    if (!g_mouse_pose.is_stationary) {
      g_mouse_pose.stationary_start_time = now_ms;
      g_mouse_pose.is_stationary         = true;
      g_mouse_pose.click_triggered       = false;
      ESP_LOGI(APP_LOG_TAG, "Board stationary - click timer started");
    } else {
      int64_t still_ms = now_ms - g_mouse_pose.stationary_start_time;
      if (still_ms >= APP_HOLDCLICK_MS && !g_mouse_pose.click_triggered) {
        *click_out                      = true;
        g_mouse_pose.click_triggered    = true;
        g_mouse_pose.stationary_start_time = now_ms;
        ESP_LOGI(APP_LOG_TAG, "Click triggered after %lld ms stationary", still_ms);
      }
    }
  } else {
    g_mouse_pose.is_stationary   = false;
    g_mouse_pose.click_triggered = false;
  }

  return ESP_OK;
}

/* ===== Event handlers ===== */
static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
  switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH: {
      if (param->init_finish.state == ESP_HIDD_INIT_OK) {
        esp_ble_gap_set_device_name(APP_HIDD_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&g_adv_data);
      }
      break;
    }
    case ESP_BAT_EVENT_REG: {
      break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
      break;
    case ESP_HIDD_EVENT_BLE_CONNECT: {
      ESP_LOGI(APP_LOG_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
      g_conn_handle = param->connect.conn_id;
      break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT: {
      g_is_secured = false;
      ESP_LOGI(APP_LOG_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
      esp_ble_gap_start_advertising(&g_adv_params);
      break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
      ESP_LOGI(APP_LOG_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
      ESP_LOG_BUFFER_HEX(APP_LOG_TAG, param->vendor_write.data, param->vendor_write.length);
      break;
    }
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
      ESP_LOGI(APP_LOG_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
      ESP_LOG_BUFFER_HEX(APP_LOG_TAG, param->led_write.data, param->led_write.length);
      break;
    }
    default:
      break;
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&g_adv_params);
      break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
      for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        ESP_LOGD(APP_LOG_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
      }
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      esp_bd_addr_t bd_addr_copy;
      memcpy(bd_addr_copy, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
      ESP_LOGI(APP_LOG_TAG, "remote BD_ADDR: %08x%04x",
               (bd_addr_copy[0] << 24) + (bd_addr_copy[1] << 16) +
               (bd_addr_copy[2] << 8) + bd_addr_copy[3],
               (bd_addr_copy[4] << 8) + bd_addr_copy[5]);
      ESP_LOGI(APP_LOG_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
      ESP_LOGI(APP_LOG_TAG, "pair status = %s",
               param->ble_security.auth_cmpl.success ? "success" : "fail");
      if (param->ble_security.auth_cmpl.success) {
        g_is_secured = true;
        ESP_LOGI(APP_LOG_TAG, "secure connection established.");
      } else {
          ESP_LOGE(APP_LOG_TAG, "pairing failed, reason = 0x%x",       param->ble_security.auth_cmpl.fail_reason);
      }
      break;
    }

    default:
      break;
  }
}

/* ===== Task ===== */
void hid_demo_task(void *pv_params) {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_LOGI(APP_LOG_TAG, "Tilt-controlled mouse task started");

  while (1) {
    if (g_is_secured) {
      int8_t dx_out   = 0;
      int8_t dy_out   = 0;
      bool   click_now = false;

      esp_err_t err_mv = read_accel_and_calculate_mouse(&dx_out, &dy_out, &click_now);
      if (err_mv == ESP_OK) {
        if (dx_out != 0 || dy_out != 0) {
          esp_hidd_send_mouse_value(g_conn_handle, 0, dx_out, dy_out);
          ESP_LOGD(APP_LOG_TAG, "Mouse: X=%d, Y=%d", dx_out, dy_out);
        }
        if (click_now) {
          ESP_LOGI(APP_LOG_TAG, "Sending mouse click! (stationary hold)");
          esp_hidd_send_mouse_value(g_conn_handle, 0x01, 0, 0);
          vTaskDelay(pdMS_TO_TICKS(50));
          esp_hidd_send_mouse_value(g_conn_handle, 0x00, 0, 0);
        }
      }

      if (button_poll_pressed()) {
        ESP_LOGI(APP_LOG_TAG, "Sending mouse click! (button)");
        esp_hidd_send_mouse_value(g_conn_handle, 0x01, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_hidd_send_mouse_value(g_conn_handle, 0x00, 0, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(APP_POLL_MS));
  }
}

/* ===== Main ===== */
void app_main(void) {
  esp_err_t err_main;

  err_main = nvs_flash_init();
  if (err_main == ESP_ERR_NVS_NO_FREE_PAGES || err_main == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err_main = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err_main);

  ESP_LOGI(APP_LOG_TAG, "Initializing I2C accelerometer...");
  i2c_bus_setup();
  vTaskDelay(pdMS_TO_TICKS(20));
  accel_sensor_configure();
  ESP_LOGI(APP_LOG_TAG, "Accelerometer initialized");

  button_init();

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t ctl_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  err_main = esp_bt_controller_init(&ctl_cfg);
  if (err_main) {
    ESP_LOGE(APP_LOG_TAG, "%s init controller failed", __func__);
    return;
  }

  err_main = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (err_main) {
    ESP_LOGE(APP_LOG_TAG, "%s enable controller failed", __func__);
    return;
  }

  esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  err_main = esp_bluedroid_init_with_cfg(&bd_cfg);
  if (err_main) {
    ESP_LOGE(APP_LOG_TAG, "%s init bluedroid failed", __func__);
    return;
  }

  err_main = esp_bluedroid_enable();
  if (err_main) {
    ESP_LOGE(APP_LOG_TAG, "%s enable bluedroid failed", __func__);
    return;
  }

  if ((err_main = esp_hidd_profile_init()) != ESP_OK) {
    ESP_LOGE(APP_LOG_TAG, "%s init hidd profile failed", __func__);
  }

  esp_ble_gap_register_callback(gap_event_handler);
  esp_hidd_register_callbacks(hidd_event_callback);

  /* Security params */
  esp_ble_auth_req_t sec_auth_mode = ESP_LE_AUTH_BOND;
  esp_ble_io_cap_t   io_capability = ESP_IO_CAP_NONE;
  uint8_t            sec_key_size  = 16;
  uint8_t            init_key_mask = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t            resp_key_mask = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &sec_auth_mode, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &io_capability, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &sec_key_size,  sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key_mask, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &resp_key_mask, sizeof(uint8_t));

  xTaskCreate(&hid_demo_task, "hid_task", 4096, NULL, 5, NULL);

  ESP_LOGI(APP_LOG_TAG, "app_main complete");
}
