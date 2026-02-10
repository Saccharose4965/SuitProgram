idf_component_register(
    SRCS "led.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES hw driver esp_driver_rmt esp_timer
)
