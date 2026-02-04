#pragma once
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Play a WAV (PCM16 mono) from filesystem.
// If a Bluetooth sink is connected/streaming, stream over A2DP.
// Otherwise, fall back to local I2S speaker via audio_play_wav_file().
esp_err_t audio_player_play(const char *path);
esp_err_t audio_player_set_current(const char *path);
esp_err_t audio_player_toggle_play_pause(void);
void      audio_player_stop(void);
bool      audio_player_should_abort(void);
bool      audio_player_is_active(void);
bool      audio_player_is_paused(void);

// Playlist helpers (in-RAM only).
void      audio_player_queue_clear(void);
int       audio_player_queue_count(void);
esp_err_t audio_player_queue_play_now(const char *path);
esp_err_t audio_player_queue_play_next(const char *path);
esp_err_t audio_player_queue_play_last(const char *path);

#ifdef __cplusplus
}
#endif
