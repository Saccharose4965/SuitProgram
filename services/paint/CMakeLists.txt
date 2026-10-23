idf_component_register(
    SRCS paint_service.c
    INCLUDE_DIRS include
    REQUIRES orientation led led_layout suit_geometry link esp_timer esp_hw_support
)
