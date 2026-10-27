idf_component_register(
    SRCS settings.c
    INCLUDE_DIRS include
    REQUIRES audio bt_audio storage_sd ui_shell led led_modes
)
