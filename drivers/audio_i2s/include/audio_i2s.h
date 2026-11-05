#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/i2s_std.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int i2s_id;
    int bclk_gpio;
    int lrck_gpio;
    int dout_gpio;
    int sd_en_pin;
    int din_gpio;
} audio_i2s_config_t;

esp_err_t audio_i2s_init(const audio_i2s_config_t *config);
esp_err_t audio_i2s_release(void);
esp_err_t audio_i2s_set_rate(int hz);
int audio_i2s_get_rate(void);
uint32_t audio_i2s_get_clock_revision(void);
uint32_t audio_i2s_get_rx_overflows(void);

// Read-only DMA receive clock observations, reset by clock revision.
// These describe produced stereo frames, not the age of a caller's read data.
typedef struct {
    uint64_t frames;
    uint32_t first_frames;
    int64_t first_us, latest_us;
    uint32_t revision;
} audio_i2s_rx_timing_t;
bool audio_i2s_rx_timing_snapshot(audio_i2s_rx_timing_t *out);

esp_err_t audio_i2s_enable_tx(void);
esp_err_t audio_i2s_enable_rx(void);
esp_err_t audio_i2s_disable_tx(void);
esp_err_t audio_i2s_disable_all(void);
i2s_chan_handle_t audio_i2s_tx_handle(void);
i2s_chan_handle_t audio_i2s_rx_handle(void);

#ifdef __cplusplus
}
#endif
