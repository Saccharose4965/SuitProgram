idf_component_register(
    SRCS hw.c
    INCLUDE_DIRS include
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_driver_uart
        esp_driver_i2s
        esp_adc
)

