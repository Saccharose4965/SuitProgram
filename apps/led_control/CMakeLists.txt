idf_component_register(
    SRCS
        "animations_app.c"
        "color_app.c"
        "fft_sync_app.c"
        "led_source_app.c"
        "manual_bpm_app.c"
    INCLUDE_DIRS "include"
    REQUIRES ui_shell led_source live_sync audio fft led led_modes audio_policy oled settings
)
