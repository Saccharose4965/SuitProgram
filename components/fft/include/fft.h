#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Starts a FreeRTOS task that continuously reads microphone samples from the
 * I²S RX channel, runs a small FFT, and renders a spectrum on the OLED.
 *
 * Prerequisites:
 * - audio_init() must have been called with a valid RX pin (din_gpio >= 0)
 * - oled_init() must have been called
 */
esp_err_t fft_visualizer_start(void);

typedef enum {
    FFT_VIEW_BPM_TEXT = 0,   // existing 3-line BPM/delta/flash view
    FFT_VIEW_SPECTRUM,       // per-frame log-magnitude spectrum (bars)
    FFT_VIEW_SPECTROGRAM,    // rolling spectrogram over time
    FFT_VIEW_NOVELTY_PEAKS,  // cleaned novelty (peaks) and local mean
    FFT_VIEW_FLUX,           // raw spectral flux (positive-only derivative) over time
    FFT_VIEW_TEMPO_RAW,      // temporal FFT spectrum (instantaneous)
    FFT_VIEW_TEMPO_SPECTRUM, // temporal FFT of novelty (capacitor-smoothed)
    FFT_VIEW_PHASE_COMB      // phase correlation vs comb filter aligned to BPM
} fft_view_t;

/**
 * Change what is rendered on the OLED. Safe to call from other tasks.
 */
void fft_visualizer_set_view(fft_view_t view);

typedef struct {
    float bpm;
    float confidence;
} fft_beat_event_t;

bool fft_receive_beat(fft_beat_event_t *evt, TickType_t timeout_ticks);

int fft_get_confident_bpms(float *bpm_out, float *conf_out, int max_out);

// Enable or disable OLED rendering; beat detection and sampling keep running.
void fft_set_display_enabled(bool enabled);

// Request novelty be zeroed for the next N frames (e.g., to ignore button voltage changes)
void fft_punch_novelty_hole(int frames);

#ifdef __cplusplus
}
#endif
