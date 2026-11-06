#pragma once

#include <stdbool.h>

#include "fft_detector.h"

#define FFT_PHASE_FAMILY_BINS (DETECTOR_PHASE_BINS / 2)

typedef struct {
    double score[DETECTOR_PHASE_BINS];
    double folded[FFT_PHASE_FAMILY_BINS];
    int best_family, best_side, raw_best_bin, teeth;
    double best_score, runner_score, prominence;
} fft_phase_evidence_t;

typedef struct {
    bool have;
    double bpm, period, anchor, last_update;
    double filtered_error;
    int family, side, challenger;
    double challenger_seconds;
} fft_phase_track_t;

typedef struct {
    fft_phase_track_t active, previous;
    double previous_saved;
    bool have_previous;
} fft_phase_tracker_t;

typedef struct {
    bool valid, acquired, restored, family_switched, corrected;
    int family, side, raw_best_bin;
    double anchor, precise_bpm, error, correction, prominence, readiness;
} fft_phase_decision_t;

typedef struct {
    bool initialized, pending;
    double bpm, target_bpm, pending_bpm;
    double period, target_period, pending_period, pending_anchor;
    double beat_start, next_beat;
    int beats_since_change;
    bool boundary_seen;
    double last_boundary, emitted_period;
} fft_beat_clock_t;

void fft_phase_tracker_reset(fft_phase_tracker_t *tracker);
void fft_phase_tracker_lose(fft_phase_tracker_t *tracker, double time);
bool fft_phase_evidence(const float *frames, int count, double frame_rate,
                        double bpm, double history_start, fft_phase_evidence_t *out);
fft_phase_decision_t fft_phase_update(fft_phase_tracker_t *tracker,
                                      const fft_phase_evidence_t *evidence,
                                      double bpm, double time,
                                      bool bpm_certain, bool family_locked);
void fft_beat_clock_reset(fft_beat_clock_t *clock);
// Initialize a zeroed clock or reanchor an existing clock; retain emission history.
void fft_beat_clock_init(fft_beat_clock_t *clock, double bpm,
                         double anchor, double now);
void fft_beat_clock_schedule(fft_beat_clock_t *clock, double bpm, double anchor);
void fft_beat_clock_update(fft_beat_clock_t *clock, double now);

// Advance the clock and accept the newest boundary at most once, with the
// same 75%-of-longer-period guard as the PC live harness. Rejected
// boundaries are consumed. Reanchoring preserves emission history; reset
// clears it. last_emit stores
// actual publication time, not the nominal boundary timestamp.
bool fft_beat_clock_emit(fft_beat_clock_t *clock, double now, double *last_emit);

// Output presentation belongs to one accepted tempo, including restored tempos.
bool fft_output_timing_on_hold_change(double previous, double next,
                                      bool *offbeat, unsigned *ratio, float *offset);
