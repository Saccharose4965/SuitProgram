idf_component_register(
    SRCS microphone.c recording_counter.c
    INCLUDE_DIRS include
    REQUIRES audio_i2s storage_sd driver vfs fatfs
)
