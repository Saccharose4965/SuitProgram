#pragma once

#include <stdint.h>

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

// The audio owner reads whole 256-frame stereo/32-bit DMA blocks through here.
// Optional timestamp: monotonic microseconds of the last returned sample;
// zero means unavailable. Check clock revision/overflows around the read too.
esp_err_t audio_i2s_read_rx(void *dest, size_t size, size_t *bytes_read,
                           uint32_t timeout_ms, int64_t *last_sample_us);

esp_err_t audio_i2s_enable_tx(void);
esp_err_t audio_i2s_enable_rx(void);
esp_err_t audio_i2s_disable_tx(void);
esp_err_t audio_i2s_disable_all(void);
i2s_chan_handle_t audio_i2s_tx_handle(void);
i2s_chan_handle_t audio_i2s_rx_handle(void);

#ifdef __cplusplus
}
#endif
