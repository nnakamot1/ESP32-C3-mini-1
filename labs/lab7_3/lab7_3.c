// lab7_3 — Integrate both
// Connects to WiFi, then in a loop:
//   1. GETs the server's configured location (GET /location)
//   2. GETs the outdoor temperature for that location from wttr.in
//   3. Reads the ESP32's onboard (die) temperature sensor
//   4. POSTs both readings + location back to the server
//
// Set via build_flags in platformio.ini:
//   -DWIFI_SSID=\"your-ssid\"
//   -DWIFI_PASS=\"your-password\"
//   -DSERVER_IP=\"192.168.1.100\"   (IP of the machine running server.py)

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "driver/temperature_sensor.h"

static const char *TAG = "LAB7_3";

#ifndef WIFI_SSID
#define WIFI_SSID "CHANGE_ME_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "CHANGE_ME_PASSWORD"
#endif
#ifndef SERVER_IP
#define SERVER_IP "CHANGE_ME_SERVER_IP"
#endif
#define SERVER_PORT 1234

#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_event_group;

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

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to SSID \"%s\"...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

// Accumulates the response body into a fixed caller-provided buffer as it
// streams in during esp_http_client_perform().
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

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 10000,
        .event_handler = http_event_handler,
        .user_data = &ctx,
    };
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

// POSTs a JSON body to url. Returns HTTP status code, or -1 on transport error.
static int http_post_json(const char *url, const char *json_body)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_body, strlen(json_body));

    esp_err_t err = esp_http_client_perform(client);
    int status = -1;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return status;
}

// Trims trailing whitespace/newlines and replaces internal spaces with '+'
// so the location can be dropped straight into a wttr.in URL path segment.
static void sanitize_for_url(char *s)
{
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r' || s[len - 1] == ' ')) {
        s[--len] = '\0';
    }
    for (size_t i = 0; i < len; i++) {
        if (s[i] == ' ') {
            s[i] = '+';
        }
    }
}

static temperature_sensor_handle_t s_tsens;

static void onboard_temp_sensor_init(void)
{
    temperature_sensor_config_t tsens_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&tsens_config, &s_tsens));
    ESP_ERROR_CHECK(temperature_sensor_enable(s_tsens));
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_connect_sta();
    onboard_temp_sensor_init();

    char location_url[64];
    snprintf(location_url, sizeof(location_url), "http://%s:%d/location", SERVER_IP, SERVER_PORT);
    char post_url[64];
    snprintf(post_url, sizeof(post_url), "http://%s:%d/", SERVER_IP, SERVER_PORT);

    while (1) {
        // 1. Ask the server what location to check
        char location[64];
        int loc_status = http_get(location_url, location, sizeof(location));
        if (loc_status != 200 || strlen(location) == 0) {
            ESP_LOGW(TAG, "Failed to get server location (status=%d), retrying later", loc_status);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        sanitize_for_url(location);
        ESP_LOGI(TAG, "Server location: %s", location);

        // 2. Query wttr.in for that location's outdoor temperature
        char wttr_url[128];
        snprintf(wttr_url, sizeof(wttr_url), "http://wttr.in/%s?format=%%t&m", location);
        char outdoor_temp[32];
        int wttr_status = http_get(wttr_url, outdoor_temp, sizeof(outdoor_temp));
        if (wttr_status != 200) {
            ESP_LOGW(TAG, "wttr.in request failed, HTTP status=%d", wttr_status);
            strncpy(outdoor_temp, "unknown", sizeof(outdoor_temp));
        }
        ESP_LOGI(TAG, "Outdoor temperature (wttr.in): %s", outdoor_temp);

        // 3. Read the onboard sensor
        float sensor_temp_c = 0.0f;
        esp_err_t err = temperature_sensor_get_celsius(s_tsens, &sensor_temp_c);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "temperature_sensor_get_celsius failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        ESP_LOGI(TAG, "ESP32 sensor temperature: %.2f C", sensor_temp_c);

        // 4. POST everything back to the server
        char body[256];
        snprintf(body, sizeof(body),
                 "{\"location\": \"%s\", \"outdoor_temp\": \"%s\", \"sensor_temp_c\": %.2f}",
                 location, outdoor_temp, sensor_temp_c);
        int post_status = http_post_json(post_url, body);
        if (post_status != 200) {
            ESP_LOGW(TAG, "POST failed, HTTP status=%d", post_status);
        }

        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
