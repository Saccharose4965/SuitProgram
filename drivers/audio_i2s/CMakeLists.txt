idf_component_register(
    SRCS audio_i2s.c
    INCLUDE_DIRS include
    REQUIRES freertos esp_driver_i2s
    PRIV_REQUIRES esp_driver_gpio
)
