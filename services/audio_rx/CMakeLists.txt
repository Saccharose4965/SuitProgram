idf_component_register(
    SRCS "audio_rx.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_ringbuf esp_timer
    REQUIRES
        # needs sockets & SD already mounted, but no direct deps beside esp-idf core
        # if you have a "storage_sd" component that provides mount logic,
        # you don't need to require it here unless you call its APIs.
)
