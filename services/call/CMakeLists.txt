idf_component_register(
    SRCS call_service.c
    INCLUDE_DIRS include
    REQUIRES audio_policy audio audio_i2s audio_player bt_audio microphone fft link
)
