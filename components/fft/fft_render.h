#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "fft.h"

enum {
    FFT_RENDER_LOGMAG_BINS = 1024 / 2 + 1
};

typedef struct {
    float logmag[FFT_RENDER_LOGMAG_BINS];
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
// Called by fft.c when recent novelty history is suppressed.
void fft_render_suppress_recent_novelty(int frames);
void fft_render_update_tempo_spectrum(const float *bpm_spec_bc);
void fft_render_update_phase_curve(const float *vals, int count);
void fft_render_trigger_flash(uint32_t flash_frames);

void fft_render_set_view(fft_view_t view);
void fft_render_set_display_enabled(bool enabled);

// Copy the latest rendered FFT frame into dst_fb (PANEL_W*PANEL_H/8 bytes).
void fft_render_copy_frame(uint8_t *dst_fb, size_t dst_len);
