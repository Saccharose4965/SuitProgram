idf_component_register(
    SRCS bluetooth_app.c bluetooth_view.c
    INCLUDE_DIRS include
    REQUIRES ui_shell input oled audio_policy bt_audio
)
