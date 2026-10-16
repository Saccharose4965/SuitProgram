#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
    LED_UI_FFT_REQ_LED_APPS = 1u << 0,
    LED_UI_FFT_REQ_COLOR_APP = 1u << 1,
    LED_UI_FFT_REQ_LED_SOURCE = 1u << 2,
};

void led_ui_fft_request(uint32_t client_mask, bool needed);
void led_ui_fft_set_bluetooth_app(bool active);
void led_ui_fft_set_show_app(bool active);
void led_ui_fft_set_call_app(bool active);
bool led_ui_fft_blocked_by_bluetooth(void);
