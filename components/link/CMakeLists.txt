idf_component_register(
    SRCS "link.c"
    INCLUDE_DIRS "include"
    REQUIRES telemetry
    PRIV_REQUIRES state
)
