// lab7_2 — Post results
// Connects to WiFi, reads the ESP32-C3's onboard (die) temperature sensor,
// and POSTs it as JSON to a server on port 1234.
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

static const char *TAG = "LAB7_2";

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

    char url[64];
    snprintf(url, sizeof(url), "http://%s:%d/", SERVER_IP, SERVER_PORT);

    while (1) {
        float temp_c = 0.0f;
        esp_err_t err = temperature_sensor_get_celsius(s_tsens, &temp_c);
        if (err == ESP_OK) {
            char body[64];
            snprintf(body, sizeof(body), "{\"sensor_temp_c\": %.2f}", temp_c);

            ESP_LOGI(TAG, "Onboard sensor temp: %.2f C -> POST %s", temp_c, url);
            int status = http_post_json(url, body);
            if (status != 200) {
                ESP_LOGW(TAG, "POST failed, HTTP status=%d", status);
            }
        } else {
            ESP_LOGE(TAG, "temperature_sensor_get_celsius failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
