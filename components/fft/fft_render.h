#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "fft.h"
#include "fft_internal.h"

typedef struct {
    float logmag[FFT_SIZE / 2 + 1];
    float bpm_est;
    float bpm_conf;
    float bpm_phase;
} fft_render_packet_t;

// Initializes the rendering task/queue. Safe to call multiple times.
esp_err_t fft_render_init(void);

// Queue a frame for rendering (or render immediately if no queue exists).
void fft_render_submit(const fft_render_packet_t *pkt);

// Maintain render-only state that is updated on the producer side.
void fft_render_push_spectrogram(const float *logmag);
void fft_render_push_novelty_display(float raw, float local_mean, float cleaned);
void fft_render_update_tempo_spectrum(const float *bpm_spec_bc);
void fft_render_update_phase_curve(const float *vals, int count);
void fft_render_trigger_flash(uint32_t flash_frames);

void fft_render_set_view(fft_view_t view);
void fft_render_set_display_enabled(bool enabled);
