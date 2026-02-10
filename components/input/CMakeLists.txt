idf_component_register(
    SRCS "input.c"
    INCLUDE_DIRS "include"
    REQUIRES hw freertos
    PRIV_REQUIRES fft
)
