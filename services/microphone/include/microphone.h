#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   sample_rate_hz;   // e.g. 44100 (default) or 48000
    int   bclk_gpio;        // e.g. 4
    int   ws_gpio;          // e.g. 22
    int   din_gpio;         // e.g. 36 (SENSOR_VP)
    int   i2s_id;           // 0 or 1, typically 0
} mic_rec_cfg_t;

/** Initialize the I²S RX channel (does not start recording). */
esp_err_t mic_rec_init(const mic_rec_cfg_t *cfg);

/** Start a new WAV recording without overwriting an existing WAV or .part file.
 *  If path == NULL, chooses the next free /sdcard/rec_XXXXX.wav name. */
esp_err_t mic_rec_start(const char *path);

/** Stop the current recording (finalizes WAV header). Safe to call if not recording. */
esp_err_t mic_rec_stop(void);

/** Query current state. */
bool      mic_rec_is_recording(void);

/** Optional: return the last path used (NULL if none yet). */
const char* mic_rec_last_path(void);
// 50 ms peak-amplitude columns, oldest first (up to 96), frozen after stop.
size_t mic_rec_envelope(uint8_t *out, size_t capacity);

#ifdef __cplusplus
}
#endif
