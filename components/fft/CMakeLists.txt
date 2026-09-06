idf_component_register(
    SRCS fft.c fft_detector.c fft_render.c
    INCLUDE_DIRS include
    REQUIRES freertos oled led esp_timer audio audio_player microphone
)

# Keep the math optimized even when the rest of the firmware is a debug build.
set_source_files_properties(fft.c fft_detector.c PROPERTIES COMPILE_OPTIONS "-O2")
