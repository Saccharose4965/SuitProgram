idf_component_register(
    SRCS audio.c audio_wav.c
    INCLUDE_DIRS include
    REQUIRES freertos esp_driver_i2s esp_timer
    PRIV_REQUIRES esp_driver_gpio
)
