idf_component_register(
    SRCS "call.c"
    INCLUDE_DIRS "include"
    REQUIRES oled hw audio telemetry esp_wifi
)
