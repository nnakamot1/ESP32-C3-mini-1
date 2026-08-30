// lab7_1 — Get the weather
// Connects to WiFi, then periodically GETs the current temperature (Celsius)
// from wttr.in and logs it.
//
// Set your WiFi credentials via build_flags in platformio.ini:
//   -DWIFI_SSID=\"your-ssid\"
//   -DWIFI_PASS=\"your-password\"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "nvs_flash.h"

static const char *TAG = "LAB7_1";

#ifndef WIFI_SSID
#define WIFI_SSID "CHANGE_ME_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "CHANGE_ME_PASSWORD"
#endif

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

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_connect_sta();

    char body[128];
    while (1) {
        int status = http_get("http://wttr.in/?format=%t&m", body, sizeof(body));
        if (status == 200) {
            ESP_LOGI(TAG, "Temperature: %s", body);
        } else {
            ESP_LOGW(TAG, "wttr.in request failed, HTTP status=%d", status);
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
