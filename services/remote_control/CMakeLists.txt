idf_component_register(
    SRCS "remote_control.c"
    INCLUDE_DIRS "include"
    REQUIRES link input esp_timer esp_system
)
