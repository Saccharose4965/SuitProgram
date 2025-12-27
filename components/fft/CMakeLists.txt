idf_component_register(
    SRCS fft.c fft_render.c
    INCLUDE_DIRS include
    REQUIRES freertos oled led esp_timer audio
)
