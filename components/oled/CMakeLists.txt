idf_component_register(
    SRCS oled.c
    INCLUDE_DIRS include
    REQUIRES freertos hw
    PRIV_REQUIRES esp_driver_gpio esp_driver_spi
)

