idf_component_register(
    SRCS "live_sync.c"
    INCLUDE_DIRS "include"
    REQUIRES fft led led_modes led_source link show_runtime
)
