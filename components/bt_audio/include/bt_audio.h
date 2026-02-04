#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BT_AUDIO_STATE_DISABLED = 0, 
    BT_AUDIO_STATE_IDLE,
    BT_AUDIO_STATE_STARTING,
    BT_AUDIO_STATE_DISCOVERING,
    BT_AUDIO_STATE_CONNECTING,
    BT_AUDIO_STATE_CONNECTED,
    BT_AUDIO_STATE_STREAMING,
    BT_AUDIO_STATE_FAILED,
} bt_audio_state_t;

typedef struct {
    bt_audio_state_t state;
    bool             has_peer;
    uint8_t          peer_bda[6];
    char             peer_name[32];
    bool             streaming;
    size_t           queued_frames;
} bt_audio_status_t;

typedef struct {
    uint8_t bda[6];
    char    name[32];
} bt_audio_device_t;

// Initialize Classic BT stack (A2DP Source) and start discovery+connect
// Set force_rescan=true to restart inquiry even if we already tried.
esp_err_t bt_audio_start(bool force_rescan);

// Volume control for A2DP source (0.0 = mute, 1.0 = unity, up to 2.0 boost).
void  bt_audio_set_volume(float vol);
float bt_audio_get_volume(void);

// Stop a running connection attempt/stream.
esp_err_t bt_audio_disconnect(void);

// Explicitly restart discovery for a new sink.
esp_err_t bt_audio_rescan(void);

// Start a scan (connectable+discoverable) and populate device list.
esp_err_t bt_audio_scan(void);

// Connect to a discovered device by index from bt_audio_get_devices().
esp_err_t bt_audio_connect_index(int idx);

// Control streaming once connected.
esp_err_t bt_audio_start_stream(void);
esp_err_t bt_audio_stop_stream(void);
esp_err_t bt_audio_pause_stream(void);

// Peer sample rate as reported by A2DP SBC config (Hz). Returns 0 if unknown.
int bt_audio_get_peer_sample_rate(void);

// Copy the current discovered devices into the caller buffer.
int bt_audio_get_devices(bt_audio_device_t *out, int max_out);

// Push interleaved stereo PCM16 frames into the A2DP source buffer.
// Frames dropped if the queue is full
esp_err_t bt_audio_feed_pcm(const int16_t *stereo, size_t frames);

// Quick check: true when stack is ready and a peer is connected/streamable.
bool bt_audio_can_stream(void);
bool bt_audio_media_cmd_pending(void);
// Playback progress for the current BT WAV stream (bytes in data chunk).
// Returns false if no BT file-backed playback is active.
bool bt_audio_get_playback_progress(uint32_t *out_total_bytes, uint32_t *out_bytes_left);

// Stream a WAV file directly from FILE* via the A2DP data callback (16-bit PCM).
esp_err_t bt_audio_play_wav(FILE *fp,
                            uint32_t data_offset,
                            uint32_t data_size,
                            uint32_t sample_rate,
                            uint16_t num_channels,
                            uint16_t bits_per_sample);

// Snapshot current state (safe from any task).
void bt_audio_get_status(bt_audio_status_t *out);

// Small helper for UI text.
const char *bt_audio_state_name(bt_audio_state_t s);
void bt_audio_volume_set_percent(int percent);
int  bt_audio_volume_get_percent(void);
void bt_audio_volume_up(void);
void bt_audio_volume_down(void);
typedef void (*bt_audio_disconnect_cb_t)(void);
void bt_audio_set_disconnect_cb(bt_audio_disconnect_cb_t cb);

#ifdef __cplusplus
}
#endif
