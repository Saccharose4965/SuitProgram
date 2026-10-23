idf_component_register(
 SRCS fluid_service.c
    INCLUDE_DIRS include
 REQUIRES freertos mpu6500 hw orientation
)
