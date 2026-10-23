idf_component_register(
    SRCS audio.c audio_wav.c
    INCLUDE_DIRS include
    REQUIRES audio_i2s freertos esp_timer
)
