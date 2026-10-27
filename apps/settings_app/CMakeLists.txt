idf_component_register(
    SRCS adc_debug_app.c preferences_app.c service_management_app.c volume_app.c
    INCLUDE_DIRS include
    REQUIRES ui_shell settings input audio audio_i2s audio_policy audio_rx
             bt_audio fft oled storage_sd
)
