idf_component_register(
    SRCS led_strip.c
    INCLUDE_DIRS include
    REQUIRES freertos esp_driver_rmt
    PRIV_REQUIRES esp_driver_gpio
)
