#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_err.h"
#include "driver/i2s_std.h"
#include "audio_wav.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int i2s_id;     // 0
    int bclk_gpio;  // 4
    int lrck_gpio;  // 22
    int dout_gpio;  // 25 (MAX98357A)
    int sd_en_pin;  // -1 if unused
    int din_gpio;   // 36 (INMP441), -1 to skip RX
} audio_i2s_config_t;

esp_err_t audio_init(const audio_i2s_config_t* cfg);
// Release I2S/DMA memory only after capture/playback users have stopped.
// Rate/enable calls restore the saved bus configuration on demand.
esp_err_t audio_release_idle_bus(void);
typedef struct {
    uint64_t frames;
    uint32_t rate;
    int64_t updated_us;
    bool bluetooth;
} audio_progress_t;
void audio_progress_reset(void);
void audio_progress_submit(size_t frames, uint32_t rate, bool bluetooth);
audio_progress_t audio_progress_get(void);
esp_err_t audio_set_rate(int hz);
// Capture diagnostics: rate is zero while clock reconfiguration is incomplete.
int       audio_get_rate(void);
uint32_t  audio_get_clock_revision(void);
uint32_t  audio_get_rx_overflows(void);
bool      audio_output_active(void);
esp_err_t audio_set_volume(float vol);
float     audio_get_volume(void);

esp_err_t audio_play_wav_mem(const void* data, size_t size);
esp_err_t audio_play_wav_file(const char* path);
esp_err_t audio_play_embedded(void);
esp_err_t audio_play_tone(int hz, int ms);
// Blocking local-speaker generator. Fill mono PCM16; return 0 to finish.
// Caller must stop other capture/playback users before entry.
typedef size_t (*audio_generator_t)(int16_t *samples, size_t capacity, void *user);
esp_err_t audio_play_generated(audio_generator_t generate, void *user, int rate);

// New streaming helper used by audio_player.c
esp_err_t audio_play_wav_local_stream(FILE *f, const audio_wav_info_t *info);

/* Shared-bus helpers (mutual exclusion) */
esp_err_t         audio_enable_tx(void);
esp_err_t         audio_enable_rx(void);
esp_err_t         audio_disable_all(void);
i2s_chan_handle_t audio_tx_handle(void);
i2s_chan_handle_t audio_rx_handle(void);

#ifdef __cplusplus
}
#endif
