idf_component_register(
    SRCS "performance_monitor.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_timer log
)
