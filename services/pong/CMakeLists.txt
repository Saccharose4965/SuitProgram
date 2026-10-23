idf_component_register(
    SRCS "pong.c" "pong_physics.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES link esp_timer esp_hw_support
)
