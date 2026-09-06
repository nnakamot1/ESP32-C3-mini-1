// weather_lcd — combines lab7 (WiFi + wttr.in HTTP GET) and lab3 (DFRobot
// RGB LCD1602) into a small weather station.
//
// Connects to WiFi, then periodically GETs location + temperature +
// condition + humidity from wttr.in in a single request, and displays it on
// the LCD, alternating every few seconds between a "location" screen and a
// "weather" screen.
//
// City selection: cycles through a fixed list (Chicago, New York, Japan,
// Paris).
//   - BOOT button (GPIO9): polled live in the main loop, scrolls DOWN to the
//     previous city immediately, no reboot needed.
//   - RESET/EN button: the EN pin is wired straight to the chip's reset line,
//     not a GPIO, so it can't be polled like BOOT — pressing it restarts
//     app_main from scratch. To still make it act as "scroll UP", the
//     current city index is persisted in NVS flash; each boot (i.e. each
//     reset-button press) advances to the next city before starting.
//
// Set via build_flags in platformio.ini:
//   -DWIFI_SSID=\"your-ssid\"
//   -DWIFI_PASS=\"your-password\"

#include <stdio.h>
#include <string.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
}

#include "DFRobot_RGBLCD1602.h"

static const char *TAG = "WEATHER_LCD";

#ifndef WIFI_SSID
#define WIFI_SSID "CHANGE_ME_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "CHANGE_ME_PASSWORD"
#endif

// wttr.in location path segments (spaces encoded as '+').
static const char *CITIES[] = { "Chicago", "New+York", "Japan", "Paris" };
static const int NUM_CITIES = sizeof(CITIES) / sizeof(CITIES[0]);

#define BOOT_BUTTON_GPIO GPIO_NUM_9  // active-low, onboard pull-up

#define NVS_NAMESPACE "weather"
#define NVS_KEY_CITY_IDX "city_idx"

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;

/* -------------------------------------------------------------------------- */
/*                          WiFi (from lab7)                                  */
/* -------------------------------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_connect_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID \"%s\"...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

/* -------------------------------------------------------------------------- */
/*                       HTTP GET helper (from lab7)                          */
/* -------------------------------------------------------------------------- */

typedef struct {
    char *buf;
    int len;
    int max_len;
} http_resp_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        http_resp_ctx_t *ctx = (http_resp_ctx_t *) evt->user_data;
        if (ctx && !esp_http_client_is_chunked_response(evt->client)) {
            int copy_len = evt->data_len;
            if (copy_len > ctx->max_len - 1 - ctx->len) {
                copy_len = ctx->max_len - 1 - ctx->len;
            }
            if (copy_len > 0) {
                memcpy(ctx->buf + ctx->len, evt->data, copy_len);
                ctx->len += copy_len;
            }
        }
    }
    return ESP_OK;
}

// GETs url and copies up to out_len-1 bytes of the response body into out.
// Returns HTTP status code, or -1 on transport error.
static int http_get(const char *url, char *out, size_t out_len)
{
    http_resp_ctx_t ctx = { .buf = out, .len = 0, .max_len = (int) out_len };
    out[0] = '\0';

    esp_http_client_config_t config = {};
    config.url = url;
    config.method = HTTP_METHOD_GET;
    config.timeout_ms = 10000;
    config.event_handler = http_event_handler;
    config.user_data = &ctx;
    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t err = esp_http_client_perform(client);
    int status = -1;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
        out[ctx.len] = '\0';
    } else {
        ESP_LOGE(TAG, "HTTP GET failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return status;
}

/* -------------------------------------------------------------------------- */
/*                              Weather fetch                                 */
/* -------------------------------------------------------------------------- */

// wttr.in custom format: location name | temperature | condition | humidity,
// pipe-separated, metric units. One request gets everything we display.
#define WEATHER_FORMAT "%l|%t|%C|%h"

typedef struct {
    char location[48];
    char temp[16];
    char condition[32];
    char humidity[16];
    bool valid;
} weather_t;

static bool fetch_weather(weather_t *w, const char *location)
{
    char url[160];
    snprintf(url, sizeof(url), "http://wttr.in/%s?format=%s&m", location, WEATHER_FORMAT);

    char body[192];
    int status = http_get(url, body, sizeof(body));
    if (status != 200) {
        ESP_LOGW(TAG, "wttr.in request failed, HTTP status=%d", status);
        return false;
    }

    // Strip trailing newline wttr.in tacks onto the response.
    size_t len = strlen(body);
    while (len > 0 && (body[len - 1] == '\n' || body[len - 1] == '\r')) {
        body[--len] = '\0';
    }

    char *saveptr = NULL;
    char *tok_location  = strtok_r(body, "|", &saveptr);
    char *tok_temp       = strtok_r(NULL, "|", &saveptr);
    char *tok_condition  = strtok_r(NULL, "|", &saveptr);
    char *tok_humidity   = strtok_r(NULL, "|", &saveptr);

    if (!tok_location || !tok_temp) {
        ESP_LOGW(TAG, "Unexpected wttr.in response: %s", body);
        return false;
    }

    snprintf(w->location,  sizeof(w->location),  "%s", tok_location);
    snprintf(w->temp,      sizeof(w->temp),      "%s", tok_temp);
    snprintf(w->condition, sizeof(w->condition), "%s", tok_condition ? tok_condition : "");
    snprintf(w->humidity,  sizeof(w->humidity),  "%s", tok_humidity ? tok_humidity : "");
    w->valid = true;

    ESP_LOGI(TAG, "location=%s temp=%s condition=%s humidity=%s",
             w->location, w->temp, w->condition, w->humidity);
    return true;
}

/* -------------------------------------------------------------------------- */
/*                             LCD display helpers                            */
/* -------------------------------------------------------------------------- */

static void lcd_print_lines(DFRobot_RGBLCD1602 &lcd, const char *line1, const char *line2)
{
    char l1[17];
    char l2[17];
    snprintf(l1, sizeof(l1), "%s", line1);
    snprintf(l2, sizeof(l2), "%s", line2);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printstr(l1);
    lcd.setCursor(0, 1);
    lcd.printstr(l2);
}

static void show_location_screen(DFRobot_RGBLCD1602 &lcd, const weather_t &w)
{
    lcd_print_lines(lcd, "Location:", w.valid ? w.location : "Fetching...");
}

static void show_weather_screen(DFRobot_RGBLCD1602 &lcd, const weather_t &w)
{
    if (!w.valid) {
        lcd_print_lines(lcd, "Weather:", "Fetching...");
        return;
    }
    // Fields are copied into oversized scratch buffers first: the 16-char LCD
    // line is expected to truncate long conditions/humidity values, but that
    // would trip -Werror=format-truncation if snprintf'd directly into line1/line2.
    char scratch1[64];
    snprintf(scratch1, sizeof(scratch1), "%s %s", w.temp, w.condition);
    char scratch2[64];
    snprintf(scratch2, sizeof(scratch2), "Humidity: %s", w.humidity);
    lcd_print_lines(lcd, scratch1, scratch2);
}

/* -------------------------------------------------------------------------- */
/*                  City index persistence (NVS) + BOOT button                */
/* -------------------------------------------------------------------------- */

// Reads the persisted city index, advances it by one (wrapping) to act as a
// "scroll up" triggered by the RESET/EN button rebooting the chip, and saves
// it back. On the very first boot after flashing (no key yet), starts at
// city 0 without advancing.
static int nvs_load_and_advance_city_idx(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s, defaulting to city 0", esp_err_to_name(err));
        return 0;
    }

    uint8_t idx = 0;
    err = nvs_get_u8(handle, NVS_KEY_CITY_IDX, &idx);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        idx = 0;  // first boot ever
    } else {
        idx = (idx + 1) % NUM_CITIES;
    }

    nvs_set_u8(handle, NVS_KEY_CITY_IDX, idx);
    nvs_commit(handle);
    nvs_close(handle);
    return idx;
}

static void nvs_save_city_idx(int idx)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s, city selection won't persist", esp_err_to_name(err));
        return;
    }
    nvs_set_u8(handle, NVS_KEY_CITY_IDX, (uint8_t) idx);
    nvs_commit(handle);
    nvs_close(handle);
}

static void boot_button_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

// Edge-detects a BOOT button press (active low). Returns true once per press;
// requires the button to be released before it will fire again.
static bool boot_button_pressed_edge(void)
{
    static bool s_was_released = true;
    bool pressed_now = (gpio_get_level(BOOT_BUTTON_GPIO) == 0);

    bool edge = pressed_now && s_was_released;
    s_was_released = !pressed_now;
    return edge;
}

/* -------------------------------------------------------------------------- */
/*                                app_main                                    */
/* -------------------------------------------------------------------------- */

#define REFRESH_INTERVAL_MS  60000  // re-fetch weather every 60s
#define SCREEN_SWAP_MS       3000   // alternate location/weather screen every 3s
#define TICK_MS              150    // main loop tick — keeps the BOOT button responsive

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    boot_button_init();

    int city_idx = nvs_load_and_advance_city_idx();
    ESP_LOGI(TAG, "Starting on city: %s", CITIES[city_idx]);

    // RGBAddr = 0x2D, LCD on I2C_NUM_0 (SDA=1, SCL=0) — same wiring as lab3_3.
    DFRobot_RGBLCD1602 lcd(/*RGBAddr*/ 0x2D, /*cols*/ 16, /*rows*/ 2,
                           I2C_NUM_0, LCD_ADDRESS);
    lcd.init();
    lcd.setBacklight(true);
    lcd.setColorWhite();

    lcd_print_lines(lcd, "Weather Station", "Connecting...");

    wifi_connect_sta();

    weather_t weather = {};
    bool need_fetch = true;

    while (true) {
        if (need_fetch) {
            lcd_print_lines(lcd, CITIES[city_idx], "Updating...");
            weather = (weather_t){};
            fetch_weather(&weather, CITIES[city_idx]);
            need_fetch = false;
        }

        int elapsed_ms = 0;
        int swap_elapsed_ms = 0;
        bool show_location = true;

        if (show_location) {
            show_location_screen(lcd, weather);
        } else {
            show_weather_screen(lcd, weather);
        }

        while (elapsed_ms < REFRESH_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(TICK_MS));
            elapsed_ms += TICK_MS;
            swap_elapsed_ms += TICK_MS;

            if (boot_button_pressed_edge()) {
                city_idx = (city_idx - 1 + NUM_CITIES) % NUM_CITIES;
                nvs_save_city_idx(city_idx);
                ESP_LOGI(TAG, "BOOT pressed -> city: %s", CITIES[city_idx]);
                need_fetch = true;
                break;  // re-fetch immediately for the new city
            }

            if (swap_elapsed_ms >= SCREEN_SWAP_MS) {
                swap_elapsed_ms = 0;
                show_location = !show_location;
                if (show_location) {
                    show_location_screen(lcd, weather);
                } else {
                    show_weather_screen(lcd, weather);
                }
            }
        }

        if (!need_fetch) {
            need_fetch = true;  // refresh interval elapsed — re-fetch same city
        }
    }
}
