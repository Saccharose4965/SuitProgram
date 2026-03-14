#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
    LED_UI_FFT_REQ_LED_APPS = 1u << 0,
    LED_UI_FFT_REQ_COLOR_APP = 1u << 1,
};

void led_ui_fft_request(uint32_t client_mask, bool needed);
