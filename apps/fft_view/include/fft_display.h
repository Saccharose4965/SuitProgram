#pragma once
#include "fft_history.h"

#define FFT_DISPLAY_WIDTH 128
#define FFT_DISPLAY_HEIGHT 64
#define FFT_DISPLAY_BYTES (FFT_DISPLAY_WIDTH * FFT_DISPLAY_HEIGHT / 8)

typedef enum {
    FFT_VIEW_SPECTRUM = 0,
    FFT_VIEW_SPECTROGRAM,
    FFT_VIEW_TEMPO_RAW,
    FFT_VIEW_CONFIDENCE,
    FFT_VIEW_PHASE_COMB,
    FFT_VIEW_COUNT
} fft_view_t;

// Pure drawing: neither consumes history nor changes detector/display state.
void fft_display_draw(const fft_history_snapshot_t *display, fft_view_t view,
                      uint8_t fb[FFT_DISPLAY_BYTES]);
// Absolute base-BPM phase -> actual output-clock coordinates (including ratio).
float fft_display_clock_phase(float absolute_phase, float origin, unsigned ratio);
