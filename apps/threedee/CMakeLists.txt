idf_component_register(
    SRCS threedee.c
    REQUIRES hw mpu6500 oled input log orientation esp_timer ui_shell audio_policy
)
