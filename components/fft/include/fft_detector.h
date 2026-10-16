#pragma once

// FFT-internal portable detector: no hardware, allocation, UI, or RTOS dependencies.
#include <stdbool.h>
#include <stdint.h>

#define DETECTOR_SAMPLE_RATE 44100
#define DETECTOR_FFT_SIZE 1024
#define DETECTOR_HOP_FRAMES 88
#define DETECTOR_REFERENCE_HOP_FRAMES 22
#define DETECTOR_FRAMES 173
#define DETECTOR_BANDS 32
#define DETECTOR_BPM_MIN 80
#define DETECTOR_BPM_MAX 159
#define DETECTOR_BPM_COUNT 80
#define DETECTOR_HISTORY 32
#define DETECTOR_PHASE_BINS 128

typedef struct {
    bool have;
    float previous[DETECTOR_BANDS];
    float clean[DETECTOR_BANDS];
} detector_filter_t;

typedef struct {
    bool hard_lock;
    bool phase_lock;
    float forced_bpm;
    uint32_t force_revision;
    uint32_t reset_revision;
} detector_control_t;

typedef struct {
    float candidate_bpm, held_bpm;
    float confidence, evidence, variance, consistency, history_coverage, lost_seconds;
    bool trusted, phase_valid, phase_stable, stopped;
    double beat_time;
    float candidate_phase, accepted_phase; // absolute base-BPM cycle fractions
    float scores[DETECTOR_BPM_COUNT];
    float phase[DETECTOR_PHASE_BINS]; // absolute phase coordinates
} detector_result_t;

typedef struct {
    float history[DETECTOR_HISTORY][DETECTOR_BPM_COUNT];
    double history_start[DETECTOR_HISTORY];
    double history_end[DETECTOR_HISTORY];
    int count, write;
    float support[DETECTOR_BPM_COUNT];
    float held_bpm;
    int held_pair;
    double last_time, hold_last_time, loss_last_time;
    bool have_time, have_hold_time, have_loss_time, have_variance;
    float previous_variance, cooldown, lost_seconds;
    bool previous_trusted;
    int previous_candidate;
    uint32_t force_revision;
    uint32_t reset_revision;
    bool have_phase;
    float phase_bpm, phase_fraction;
} detector_t;

void detector_reset(detector_t *d);
// Input is log(1 + 10 * FFT magnitude), excludes the Nyquist bin.
bool detector_filter_bands(detector_filter_t *filter, const float *logmag,
                           bool suppress, float bands[DETECTOR_BANDS]);
// Each score can be computed independently; the caller schedules/yields batches.
float detector_comb_score(const float *frames, int count, int bpm);
typedef void (*detector_yield_fn)(void *ctx);
float detector_comb_score_cooperative(const float *frames, int count, int bpm,
                                      detector_yield_fn yield_fn, void *yield_ctx);
void detector_normalize(const float raw[DETECTOR_BPM_COUNT],
                        float scores[DETECTOR_BPM_COUNT]);
void detector_update(detector_t *d, const float scores[DETECTOR_BPM_COUNT],
                     double time, detector_control_t control, detector_result_t *out);
void detector_phase(detector_t *d, const float *frames, int count, double last_time,
                    bool locked, detector_result_t *out);
