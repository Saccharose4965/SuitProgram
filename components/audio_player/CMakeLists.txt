idf_component_register(
    SRCS "audio_player.c" "playback_diagnostics.c"
    INCLUDE_DIRS "include"
    REQUIRES bt_audio audio
)
