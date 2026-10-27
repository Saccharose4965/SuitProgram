idf_component_register(
    SRCS message_direct.c message_notifications.c
    INCLUDE_DIRS include
    REQUIRES link telemetry audio storage_sd esp_hw_support
)
