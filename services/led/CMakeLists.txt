idf_component_register(
    SRCS "led.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES hw driver esp_timer led_layout suit_geometry led_strip
)
