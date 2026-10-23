idf_component_register(
    SRCS orientation.c orientation_service.c arm_pose.c gyro_calibration.c
    INCLUDE_DIRS include
    REQUIRES hw mpu6500 storage_sd esp_timer
)
