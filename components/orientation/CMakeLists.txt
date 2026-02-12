idf_component_register(
    SRCS orientation.c orientation_service.c
    INCLUDE_DIRS include
    REQUIRES hw mpu6500
)
