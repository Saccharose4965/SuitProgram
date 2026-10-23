idf_component_register(
    SRCS fft_app.c fft_display.c
    INCLUDE_DIRS include
    REQUIRES ui_shell audio_policy fft oled
)
