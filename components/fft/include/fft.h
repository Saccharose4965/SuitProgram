#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
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
 */
esp_err_t fft_visualizer_start(void);
void fft_visualizer_stop(void);
bool fft_visualizer_running(void);

// Shared FFT pipeline configuration (single source of truth for fft.c/fft_render.c).
enum {
    FFT_CFG_SAMPLE_RATE_HZ = 16000,
    FFT_CFG_HOP_SAMPLES    = 256,   // 16 ms hop
    FFT_CFG_SIZE           = 1024,  // 64 ms window
    FFT_CFG_NOVELTY_WIN    = 32,    // local mean window for novelty (~0.5 s)
    FFT_CFG_NOV_RING_FRAMES =
        (6 * FFT_CFG_SAMPLE_RATE_HZ + (FFT_CFG_HOP_SAMPLES / 2)) / FFT_CFG_HOP_SAMPLES, // ~6 s
    FFT_CFG_BPM_MIN        = 32,
    FFT_CFG_BPM_MAX        = 255,
    FFT_CFG_TARGET_BPM_MIN = 64,
    FFT_CFG_TARGET_BPM_MAX = 127
};

typedef enum {
    FFT_VIEW_BPM_TEXT = 0,   // existing 3-line BPM/delta/flash view
    FFT_VIEW_SPECTRUM,       // per-frame log-magnitude spectrum (bars)
    FFT_VIEW_SPECTROGRAM,    // rolling spectrogram over time
    FFT_VIEW_NOVELTY_PEAKS,  // cleaned novelty (peaks) and local mean
    FFT_VIEW_FLUX,           // raw spectral flux (positive-only derivative) over time
    FFT_VIEW_TEMPO_RAW,      // temporal FFT spectrum (instantaneous)
    FFT_VIEW_PHASE_COMB,     // phase correlation vs comb filter aligned to BPM
    FFT_VIEW_COUNT           // number of valid render views
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

typedef struct {
    float overall; // 0..1 normalized broadband movement
    float low;     // 0..1 normalized low-band movement
    float mid;     // 0..1 normalized mid-band movement
    float high;    // 0..1 normalized high-band movement
} fft_levels_t;

void fft_get_levels(fft_levels_t *out);

typedef struct {
    float bpm;           // effective BPM currently driving beat export
    float detected_bpm;  // raw BPM detected from novelty analysis
    float confidence;    // 0..1 detection confidence
    float beat_phase;    // 0..1 current beat cycle position
    float trigger_phase; // 0..1 phase where a beat is emitted
    float phase_offset;  // 0..1 user offset added to trigger phase
    bool  bpm_locked;
    bool  beat_enabled;
    bool  running;
} fft_sync_state_t;

void fft_get_sync_state(fft_sync_state_t *out);
void fft_beat_phase_offset_add(float delta_cycles);
void fft_beat_lock_set(bool locked);
void fft_beat_lock_toggle(void);
void fft_beat_enable_set(bool enabled);
void fft_beat_enable_toggle(void);

// Enable or disable OLED rendering; beat detection and sampling keep running.
void fft_set_display_enabled(bool enabled);

// Copy the latest FFT view framebuffer into dst_fb (PANEL_W*PANEL_H/8 bytes).
void fft_copy_frame(uint8_t *dst_fb, size_t dst_len);

// Request novelty suppression over the most recent N frames
// (e.g., to ignore button voltage changes).
void fft_suppress_novelty_frames(int frames);

// Request novelty suppression aligned to real-world latency:
// - backfill_ms: retroactively suppress recent novelty history covering this delay
// - future_frames: also suppress a few upcoming frames to absorb jitter/ringing
void fft_suppress_novelty_timed_ms(int backfill_ms, int future_frames);

#ifdef __cplusplus
}
#endif
