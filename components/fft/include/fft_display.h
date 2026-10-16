#pragma once
#include "fft.h"
#include "fft_detector.h"

#define FFT_DISPLAY_WIDTH 128
#define FFT_DISPLAY_HEIGHT 64
#define FFT_DISPLAY_HISTORY FFT_DISPLAY_WIDTH
#define FFT_DISPLAY_FRAMES_PER_COLUMN 2
#define FFT_DISPLAY_ROWS 32
#define FFT_DISPLAY_BYTES (FFT_DISPLAY_WIDTH * FFT_DISPLAY_HEIGHT / 8)

typedef struct {
    float logmag[FFT_CFG_SIZE / 2 + 1];
    float magnitudes[FFT_DISPLAY_ROWS]; // peak raw magnitude per frequency group
    fft_sync_state_t status;
    float decision; // uncapped 0..1.1 readiness, like localFFT
    float clock_origin; // absolute base-BPM phase of the actual output anchor
    double time;
} fft_display_packet_t;

typedef struct {
    uint8_t spectrogram[FFT_DISPLAY_HISTORY][FFT_DISPLAY_ROWS];
    float confidence[FFT_DISPLAY_HISTORY], evidence[FFT_DISPLAY_HISTORY];
    unsigned write, count;
    unsigned pending_frames;
    uint8_t pending[FFT_DISPLAY_ROWS];
    fft_display_packet_t latest;
    detector_result_t detector;
} fft_display_t;

void fft_display_reset(fft_display_t *display);
void fft_display_push(fft_display_t *display, const fft_display_packet_t *packet);
// Pure drawing: neither consumes history nor changes detector/display state.
void fft_display_draw(const fft_display_t *display, fft_view_t view,
                      uint8_t fb[FFT_DISPLAY_BYTES]);
// Absolute base-BPM phase -> actual output-clock coordinates (including ratio).
float fft_display_clock_phase(float absolute_phase, float origin, unsigned ratio);
