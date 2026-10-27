idf_component_register(
    SRCS "led_source.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_policy fft led led_modes state
)
