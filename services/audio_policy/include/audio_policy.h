#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
    AUDIO_POLICY_FFT_REQ_LED_APPS = 1u << 0,
    AUDIO_POLICY_FFT_REQ_COLOR_APP = 1u << 1,
    AUDIO_POLICY_FFT_REQ_LED_SOURCE = 1u << 2,
};

void audio_policy_fft_request(uint32_t client_mask, bool needed);
void audio_policy_fft_set_bluetooth_app(bool active);
void audio_policy_fft_set_show_app(bool active);
void audio_policy_fft_set_paint_app(bool active);
void audio_policy_fft_set_synth_app(bool active);
void audio_policy_fft_set_threedee_app(bool active);
void audio_policy_fft_set_armpose_app(bool active);
void audio_policy_fft_set_call_app(bool active);
bool audio_policy_fft_blocked_by_bluetooth(void);
