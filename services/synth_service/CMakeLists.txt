idf_component_register(
    SRCS synth_service.c
    INCLUDE_DIRS include
    REQUIRES audio_policy orientation audio audio_i2s audio_player bt_audio
             microphone esp_timer
)
