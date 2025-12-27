#include "shell_audio.h"

#include "audio.h"
#include "hw.h"

static bool s_audio_inited = false;

bool shell_audio_init_if_needed(void)
{
    if (s_audio_inited) return true;
    audio_i2s_config_t bus = {
        .i2s_id    = 0,
        .bclk_gpio = PIN_I2S_BCLK,
        .lrck_gpio = PIN_I2S_LRCK,
        .dout_gpio = PIN_I2S_DOUT,
        .din_gpio  = PIN_I2S_DIN,
        .sd_en_pin = -1,
    };
    if (audio_init(&bus) == ESP_OK) {
        s_audio_inited = true;
    }
    return s_audio_inited;
}
