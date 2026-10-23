#include "fft_phase.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PREVIOUS_EXPIRY 45.0
#define LOCAL_RADIUS 10
#define SWITCH_RATIO 1.20
#define SWITCH_SECONDS 6.0
#define MAX_BPM_DIFF 0.5
#define RESTORE_MAX_ERROR 0.10
#define MAX_PERIOD_OFFSET 0.02
#define MAX_PERIOD_STEP 0.002
#define PERIOD_GAIN 0.20
#define POSITION_GAIN 0.20
#define MAX_POSITION_STEP 0.030
#define ERROR_FILTER_SECONDS 0.80
#define ERROR_DEADBAND_BINS 0.75
#define MIN_PROMINENCE 1.05
#define BPM_UPDATE_SECONDS ((double)DETECTOR_HOP_FRAMES * DETECTOR_FFT_SIZE / DETECTOR_SAMPLE_RATE)
#define CLOCK_STEER_GAIN 0.20
#define CLOCK_STEER_MAX_RATIO 0.004

static double clampd(double x, double lo, double hi)
{ return x < lo ? lo : (x > hi ? hi : x); }

static double wrap01(double x)
{ x -= floor(x); return x < 0 ? x + 1 : x; }

static double wrap_half(double x)
{ x = fmod(x, .5); return x < 0 ? x + .5 : x; }

static double wrap_error(double x, double period)
{ return x - round(x / period) * period; }

static int distance(int a, int b, int count)
{ int d = abs(a - b) % count; return d < count - d ? d : count - d; }

static double time_for_phase(double phase, double bpm, double near)
{
    double period = 60 / bpm, origin = phase * period;
    return origin + round((near - origin) / period) * period;
}

static double predicted_family(const fft_phase_track_t *track, double time)
{
    double near = track->anchor + round((time - track->anchor) / (track->period * .5)) *
                                  (track->period * .5);
    return wrap_half(near * track->bpm / 60);
}

static int local_best(const fft_phase_evidence_t *e, int center)
{
    int best = center;
    for (int offset = -LOCAL_RADIUS; offset <= LOCAL_RADIUS; ++offset) {
        int bin = (center + offset + FFT_PHASE_FAMILY_BINS) % FFT_PHASE_FAMILY_BINS;
        if (e->folded[bin] > e->folded[best]) best = bin;
    }
    return best;
}

static void acquire(fft_phase_track_t *track, const fft_phase_evidence_t *e,
                    double bpm, double time, int family, int preserve_side)
{
    int side = preserve_side >= 0 ? preserve_side : e->best_side;
    double phase = (family + side * FFT_PHASE_FAMILY_BINS) / (double)DETECTOR_PHASE_BINS;
    memset(track, 0, sizeof(*track));
    track->have = true;
    track->bpm = bpm;
    track->period = 60 / bpm;
    track->anchor = time_for_phase(phase, bpm, time);
    track->last_update = time;
    track->family = family;
    track->side = side;
}

void fft_phase_tracker_reset(fft_phase_tracker_t *tracker)
{ memset(tracker, 0, sizeof(*tracker)); }

void fft_phase_tracker_lose(fft_phase_tracker_t *tracker, double time)
{
    if (!tracker->active.have) return;
    tracker->previous = tracker->active;
    tracker->previous_saved = time;
    tracker->have_previous = true;
    memset(&tracker->active, 0, sizeof(tracker->active));
}

bool fft_phase_evidence(const float *frames, int count, double frame_rate,
                        double bpm, double history_start, fft_phase_evidence_t *out)
{
    if (!frames || !out || count <= 2 || bpm <= 0 || frame_rate <= 0) return false;
    memset(out, 0, sizeof(*out));
    float step = (float)(frame_rate * 60 / bpm);
    double start_phase = wrap01(history_start * bpm / 60);
    for (int bin = 0; bin < DETECTOR_PHASE_BINS; ++bin) {
        float relative = (float)wrap01(bin / (double)DETECTOR_PHASE_BINS - start_phase);
        float sum = 0;
        int teeth = 0;
        for (float pos = relative * step; pos < count - 1 && teeth < 64; pos += step) {
            int index = (int)pos;
            float fraction = pos - index, energy = 0;
            for (int band = 0; band < DETECTOR_BANDS; ++band) {
                float a = frames[index * DETECTOR_BANDS + band];
                float b = frames[(index + 1) * DETECTOR_BANDS + band];
                energy += a + (b - a) * fraction;
            }
            sum += energy / DETECTOR_BANDS;
            ++teeth;
        }
        if (teeth > out->teeth) out->teeth = teeth;
        out->score[bin] = teeth ? sum / teeth : 0;
    }
    double floor_value = out->score[0];
    for (int i = 1; i < DETECTOR_PHASE_BINS; ++i)
        if (out->score[i] < floor_value) floor_value = out->score[i];
    for (int i = 0; i < DETECTOR_PHASE_BINS; ++i)
        out->score[i] = fmax(0, out->score[i] - floor_value);
    int best = 0;
    for (int i = 0; i < FFT_PHASE_FAMILY_BINS; ++i) {
        out->folded[i] = out->score[i] + out->score[i + FFT_PHASE_FAMILY_BINS];
        if (out->folded[i] > out->folded[best]) best = i;
    }
    double runner = 0;
    for (int i = 0; i < FFT_PHASE_FAMILY_BINS; ++i)
        if (distance(i, best, FFT_PHASE_FAMILY_BINS) > 3 && out->folded[i] > runner)
            runner = out->folded[i];
    out->best_family = best;
    out->best_side = out->score[best + FFT_PHASE_FAMILY_BINS] > out->score[best];
    out->raw_best_bin = best + out->best_side * FFT_PHASE_FAMILY_BINS;
    out->best_score = out->folded[best];
    out->runner_score = runner;
    out->prominence = runner > 1e-12 ? out->best_score / runner : HUGE_VAL;
    return out->teeth > 0;
}

fft_phase_decision_t fft_phase_update(fft_phase_tracker_t *tracker,
                                      const fft_phase_evidence_t *e,
                                      double bpm, double time,
                                      bool bpm_certain, bool family_locked)
{
    fft_phase_decision_t d = {0};
    if (!tracker || !e || !isfinite(bpm) || bpm <= 0 || !isfinite(time)) return d;
    if (tracker->have_previous && time - tracker->previous_saved > PREVIOUS_EXPIRY) {
        memset(&tracker->previous, 0, sizeof(tracker->previous));
        tracker->have_previous = false;
    }
    if (tracker->active.have && fabs(tracker->active.bpm - bpm) > MAX_BPM_DIFF)
        fft_phase_tracker_lose(tracker, time);
    if (!bpm_certain) {
        if (tracker->active.have) {
            d.valid = true; d.family = tracker->active.family; d.side = tracker->active.side;
            d.anchor = tracker->active.anchor; d.precise_bpm = 60 / tracker->active.period;
        }
        return d;
    }
    if (!tracker->active.have) {
        bool restored = false;
        if (tracker->have_previous && fabs(tracker->previous.bpm - bpm) <= MAX_BPM_DIFF) {
            int predicted = (int)lround(predicted_family(&tracker->previous, time) *
                                        DETECTOR_PHASE_BINS) % FFT_PHASE_FAMILY_BINS;
            int local = local_best(e, predicted);
            if (distance(local, predicted, FFT_PHASE_FAMILY_BINS) /
                    (double)DETECTOR_PHASE_BINS <= RESTORE_MAX_ERROR) {
                tracker->active = tracker->previous;
                tracker->have_previous = false;
                restored = true;
            }
        }
        if (!restored) acquire(&tracker->active, e, bpm, time, e->best_family, -1);
        d.acquired = !restored; d.restored = restored;
    }
    fft_phase_track_t *track = &tracker->active;
    double elapsed = fmax(1e-3, time - track->last_update);
    int predicted = (int)lround(predicted_family(track, time) * DETECTOR_PHASE_BINS) %
                    FFT_PHASE_FAMILY_BINS;
    int local = local_best(e, predicted), global = e->best_family;
    double local_score = e->folded[local], global_score = e->folded[global];
    bool distant = distance(global, local, FFT_PHASE_FAMILY_BINS) > LOCAL_RADIUS;
    if (!family_locked && distant && global_score >= local_score * SWITCH_RATIO) {
        if (track->challenger == global) track->challenger_seconds += elapsed;
        else { track->challenger = global; track->challenger_seconds = fmin(elapsed, BPM_UPDATE_SECONDS); }
    } else {
        track->challenger_seconds = fmax(0, track->challenger_seconds - elapsed);
        if (track->challenger_seconds <= 0) track->challenger = 0;
    }
    if (!family_locked && track->challenger_seconds >= SWITCH_SECONDS) {
        local = track->challenger;
        acquire(track, e, bpm, time, local, track->side);
        d.family_switched = true;
        elapsed = BPM_UPDATE_SECONDS;
    }
    double family_phase = local / (double)DETECTOR_PHASE_BINS;
    double a = time_for_phase(family_phase, bpm, time);
    double b = time_for_phase(family_phase + .5, bpm, time);
    double predicted_time = track->anchor + round((time - track->anchor) /
                            (track->period * .5)) * (track->period * .5);
    double observed = fabs(a - predicted_time) <= fabs(b - predicted_time) ? a : b;
    double raw_error = wrap_error(observed - predicted_time, track->period * .5);
    double alpha = 1 - exp(-elapsed / ERROR_FILTER_SECONDS);
    track->filtered_error += alpha * (raw_error - track->filtered_error);
    double error = track->filtered_error;
    if (fabs(error) <= (60 / bpm) * ERROR_DEADBAND_BINS / DETECTOR_PHASE_BINS) error = 0;
    double cadence = clampd(elapsed / BPM_UPDATE_SECONDS, 0, 1);
    double correction = clampd(error * POSITION_GAIN,
                               -MAX_POSITION_STEP * cadence, MAX_POSITION_STEP * cadence);
    bool strong = isfinite(e->prominence) && e->prominence >= MIN_PROMINENCE && local_score > 1e-12;
    if (strong && error != 0) {
        double nominal = 60 / bpm;
        double proposed = track->period + PERIOD_GAIN * error * track->period / elapsed;
        double step = nominal * MAX_PERIOD_STEP * cadence;
        track->period = clampd(proposed, track->period - step, track->period + step);
        track->period = clampd(track->period, nominal * (1 - MAX_PERIOD_OFFSET),
                              nominal * (1 + MAX_PERIOD_OFFSET));
        track->anchor += correction;
        d.corrected = true;
    } else correction = 0;
    track->bpm = bpm; track->family = local; track->last_update = time;
    d.valid = true; d.family = local; d.side = track->side; d.raw_best_bin = e->raw_best_bin;
    d.anchor = track->anchor; d.precise_bpm = 60 / track->period;
    d.error = error; d.correction = correction; d.prominence = e->prominence;
    d.readiness = clampd(track->challenger_seconds / SWITCH_SECONDS, 0, 1);
    return d;
}

void fft_beat_clock_reset(fft_beat_clock_t *clock) { memset(clock, 0, sizeof(*clock)); }

void fft_beat_clock_init(fft_beat_clock_t *c, double bpm, double anchor, double now)
{
    double period = 60 / bpm, start = anchor;
    while (start > now) start -= period;
    while (start + period <= now) start += period;
    *c = (fft_beat_clock_t){ .initialized = true, .bpm = bpm, .target_bpm = bpm,
        .pending_bpm = bpm, .period = period, .target_period = period,
        .pending_period = period, .pending_anchor = anchor,
        .beat_start = start, .next_beat = start + period, .beats_since_change = 1000000 };
}

void fft_beat_clock_schedule(fft_beat_clock_t *c, double bpm, double anchor)
{
    if (!c->initialized || !isfinite(bpm) || bpm <= 0) return;
    c->pending_bpm = bpm; c->pending_period = 60 / bpm;
    c->pending_anchor = anchor; c->pending = true;
}

void fft_beat_clock_update(fft_beat_clock_t *c, double now)
{
    if (!c->initialized) return;
    int guard = 0;
    while (now >= c->next_beat && guard++ < 8) {
        double boundary = c->next_beat;
        if (c->pending) {
            c->target_bpm = c->pending_bpm; c->target_period = c->pending_period;
            c->beat_start = boundary;
            double desired = c->pending_anchor + round((boundary + c->target_period -
                             c->pending_anchor) / c->target_period) * c->target_period;
            double steer = clampd((desired - boundary - c->target_period) * CLOCK_STEER_GAIN,
                                  -c->target_period * CLOCK_STEER_MAX_RATIO,
                                   c->target_period * CLOCK_STEER_MAX_RATIO);
            c->period = c->target_period + steer; c->pending = false; c->beats_since_change = 0;
        } else {
            c->beat_start = boundary; c->period = c->target_period;
            if (c->beats_since_change < 1000000) ++c->beats_since_change;
        }
        c->bpm = 60 / c->period; c->next_beat = c->beat_start + c->period;
    }
    if (guard >= 8) { c->beat_start = now; c->next_beat = now + c->period; }
}
