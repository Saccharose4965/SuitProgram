idf_component_register(
    SRCS fluid.c
    INCLUDE_DIRS include
    REQUIRES freertos oled mpu6500 hw input esp_timer orientation
)
