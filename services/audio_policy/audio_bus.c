#include "audio_bus.h"

#include "audio_i2s.h"
#include "hw.h"

static bool s_audio_inited = false;

bool audio_bus_init_if_needed(void)
{
    if (s_audio_inited && audio_i2s_tx_handle()) return true;
    s_audio_inited = false;
    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = PIN_I2S_DIN,
        .sd_en_pin = -1,
    };
    if (audio_i2s_init(&bus) == ESP_OK) {
        s_audio_inited = true;
    }
    return s_audio_inited;
}
