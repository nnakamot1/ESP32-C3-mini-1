idf_component_register(SRCS "main.cpp" "DFRobot_RGBLCD1602.cpp"
                    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_http_client
                    INCLUDE_DIRS ".")
