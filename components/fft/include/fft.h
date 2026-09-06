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
    FFT_CFG_SAMPLE_RATE_HZ = 44100,
    FFT_CFG_HOP_SAMPLES    = 1024,  // 23.22 ms; same non-overlapping frames as PC
    FFT_CFG_SIZE           = 1024,
    FFT_CFG_NOVELTY_WIN    = 32,    // local mean for flux display (~0.74 s)
    FFT_CFG_NOV_RING_FRAMES =
        (4 * FFT_CFG_SAMPLE_RATE_HZ + FFT_CFG_HOP_SAMPLES - 1) / FFT_CFG_HOP_SAMPLES,
    FFT_CFG_BPM_MIN        = 80,
    FFT_CFG_BPM_MAX        = 159,
    FFT_CFG_TARGET_BPM_MIN = 80,
    FFT_CFG_TARGET_BPM_MAX = 159
};

typedef enum {
    FFT_VIEW_BPM_TEXT = 0,   // held BPM, decision/evidence readiness, phase/flash
    FFT_VIEW_SPECTRUM,       // per-frame log-magnitude spectrum (bars)
    FFT_VIEW_SPECTROGRAM,    // rolling spectrogram over time
    FFT_VIEW_NOVELTY_PEAKS,  // raw spectral flux and local mean (display only)
    FFT_VIEW_FLUX,           // flux above local mean (display only)
    FFT_VIEW_TEMPO_RAW,      // normalized, averaged direct-comb BPM scores
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
    float detected_bpm;  // strongest smoothed BPM candidate
    float confidence;    // decision readiness, capped at 1 for existing consumers
    float held_bpm;      // base BPM before ratio
    float evidence;      // ungated evidence quality, threshold 1
    float lost_seconds;
    float analysis_ms;   // last worker wall time, including cooperative yields
    float capture_load;  // smoothed processing wall time / frame duration, percent
    uint32_t skipped_jobs;
    uint32_t capture_errors;
    float beat_phase;    // 0..1 current beat cycle position
    float trigger_phase; // 0..1 phase where a beat is emitted
    float phase_offset;  // 0..1 user offset added to trigger phase
    bool  bpm_locked;
    bool  phase_locked;  // also enables the independent BPM soft-lock
    unsigned ratio;      // output pulse multiplier: 1, 2, 3
    bool offbeat;
    bool  beat_enabled;
    bool  running;
} fft_sync_state_t;

void fft_get_sync_state(fft_sync_state_t *out);
void fft_beat_phase_offset_add(float delta_cycles);
void fft_beat_lock_set(bool locked);
void fft_beat_lock_toggle(void);
void fft_detector_reset(void);
void fft_phase_lock_toggle(void);
void fft_beat_ratio_cycle(void);
void fft_beat_offbeat_toggle(void);
void fft_beat_enable_set(bool enabled);
void fft_beat_enable_toggle(void);
void fft_led_export_enable(bool enabled);
bool fft_led_export_enabled(void);

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
