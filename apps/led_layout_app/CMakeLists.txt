idf_component_register(
    SRCS "led_layout_app.c"
    INCLUDE_DIRS "."
    REQUIRES ui_shell input oled led led_modes led_layout esp_timer
)
