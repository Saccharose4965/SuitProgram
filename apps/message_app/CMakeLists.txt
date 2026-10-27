idf_component_register(
    SRCS message_app.c
    INCLUDE_DIRS include
    REQUIRES ui_shell messages audio audio_player bt_audio fft microphone oled
             storage_sd telemetry esp_timer esp_hw_support
)
