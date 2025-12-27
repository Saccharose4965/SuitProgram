idf_component_register(
    SRCS stick.c
    INCLUDE_DIRS include
    REQUIRES freertos hw oled esp_timer esp_driver_spi
)
