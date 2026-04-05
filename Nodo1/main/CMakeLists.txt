idf_component_register(
    SRCS
        "main.c"
        "hal_sensors.c"
        "app_signal.c"
        "app_radio.c"
        "app_tasks.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_driver_gpio
        esp_adc
        esp_event
        esp_timer
        esp_wifi
        esp_netif
        nvs_flash
)