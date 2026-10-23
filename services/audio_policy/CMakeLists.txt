idf_component_register(
    SRCS audio_policy.c audio_bus.c
    INCLUDE_DIRS include
    REQUIRES fft led bt_audio audio audio_i2s audio_player microphone show_runtime hw esp_timer
)
