idf_component_register(
    SRCS threedee.c
    INCLUDE_DIRS include
    REQUIRES hw mpu6500 oled input log orientation
)
