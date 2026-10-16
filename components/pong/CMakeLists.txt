idf_component_register(
    SRCS "pong.c" "pong_physics.c"
    INCLUDE_DIRS "include"
    REQUIRES oled input link esp_timer esp_hw_support
)
