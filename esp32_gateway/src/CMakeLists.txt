idf_component_register(
    SRCS "main.cpp" "gw_prov.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        driver
        esp_http_server
        esp_timer
        led_strip
)
