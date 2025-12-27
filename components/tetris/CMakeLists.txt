idf_component_register(
    SRCS "tetris.c"
    INCLUDE_DIRS "include"
    REQUIRES oled hw freertos input
)
