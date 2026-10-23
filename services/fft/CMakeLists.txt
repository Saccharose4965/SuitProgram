idf_component_register(
    SRCS fft.c fft_detector.c fft_phase.c fft_history.c fft_history_model.c
    INCLUDE_DIRS include
    REQUIRES freertos led esp_timer audio audio_i2s audio_player microphone
)

# Keep the math optimized even when the rest of the firmware is a debug build.
set_source_files_properties(fft.c fft_detector.c fft_phase.c fft_history_model.c PROPERTIES COMPILE_OPTIONS "-O2")
