idf_component_register(
    SRCS "shell_ui.c" "ui_draw.c" "startup_logo.c"
    INCLUDE_DIRS "include"
    REQUIRES oled state input freertos
)
