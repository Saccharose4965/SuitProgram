idf_component_register(
    SRCS "comms.c"
    INCLUDE_DIRS "include"
    REQUIRES audio telemetry storage_sd oled hw microphone
)
