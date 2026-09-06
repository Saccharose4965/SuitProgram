#include "fft_detector.h"

#include <math.h>
#include <string.h>

#define FRAME_RATE ((float)DETECTOR_SAMPLE_RATE / DETECTOR_FFT_SIZE)
#define UPDATE_SECONDS ((float)DETECTOR_HOP_FRAMES / FRAME_RATE)
#define STABILITY_LIMIT 0.014f
#define CONSISTENCY_LIMIT 0.95f
#define RHYTHM_MIN 0.05f
#define HELD_VISIBLE_MIN 0.03f
#define SUPPORT_HALF_LIFE 6.0f
#define SHOCK_LIMIT 0.004f
#define SHOCK_CALM 3.0f
#define LOST_TIMEOUT 30.0f

static float wrap(float x) { return x - floorf(x); }
static float display_value(float x) { return isfinite(x) ? fminf(1.1f, fmaxf(0, x)) : 0; }

void detector_reset(detector_t *d) { memset(d, 0, sizeof(*d)); }

bool detector_filter_bands(detector_filter_t *f, const float *logmag,
                           bool suppress, float bands[DETECTOR_BANDS])
{
    float raw[DETECTOR_BANDS];
    const int first = (100 * DETECTOR_FFT_SIZE + DETECTOR_SAMPLE_RATE - 1) / DETECTOR_SAMPLE_RATE;
    const int bins = DETECTOR_FFT_SIZE / 2 - first;
    float upper_rise = 0;
    int rising = 0;
    for (int b = 0; b < DETECTOR_BANDS; ++b) {
        int lo = first + b * bins / DETECTOR_BANDS;
        int hi = first + (b + 1) * bins / DETECTOR_BANDS;
        float sum = 0;
        for (int k = lo; k < hi; ++k) sum += logmag[k];
        raw[b] = sum / (hi - lo);
        float rise = fmaxf(0, raw[b] - f->previous[b]);
        if (b >= 16) upper_rise += rise / 16;
        if (rise >= 0.75f) ++rising;
    }
    bool impulse = f->have && upper_rise >= 1.0f && rising >= 8;
    memcpy(f->previous, raw, sizeof(raw));
    if (!suppress && !impulse) {
        memcpy(f->clean, raw, sizeof(raw));
        f->have = true;
    }
    for (int b = 0; b < DETECTOR_BANDS; ++b)
        bands[b] = f->clean[b] * ((b >= 16 && b < 24) ? 1.41421356237f : 1.0f);
    return impulse || suppress;
}

float detector_comb_score(const float *frames, int count, int bpm)
{
    if (count < 3 || bpm < DETECTOR_BPM_MIN || bpm > DETECTOR_BPM_MAX) return 0;
    float step = FRAME_RATE * 60 / bpm;
    int phases = (int)ceilf(step * 8);
    if (phases > 256) phases = 256;
    float total = 0;
    int evaluations = 0;
    for (int p = 0; p < phases; ++p) {
        float sums[DETECTOR_BANDS] = {0}, squares[DETECTOR_BANDS] = {0};
        int teeth = 0;
        // Compute tooth positions once for all bands; contiguous loads suit PSRAM.
        for (float pos = step * p / phases; pos < count - 1; pos += step) {
            int i = (int)pos;
            float t = pos - i;
            const float *a = frames + i * DETECTOR_BANDS;
            const float *b = a + DETECTOR_BANDS;
            for (int band = 0; band < DETECTOR_BANDS; ++band) {
                // Subtract a fixed origin: same variance, no cancellation of a
                // large DC level when converting the PC's doubles to floats.
                float x = (a[band] - frames[band]) + (b[band] - a[band]) * t;
                sums[band] += x;
                squares[band] += x * x;
            }
            ++teeth;
        }
        if (teeth < 2) continue;
        float inv_n = 1.0f / teeth, inv_dof = 1.0f / (teeth - 1);
        for (int b = 0; b < DETECTOR_BANDS; ++b)
            total += fmaxf(0, (squares[b] - sums[b] * sums[b] * inv_n) * inv_dof);
        evaluations += DETECTOR_BANDS;
    }
    return evaluations ? total / evaluations : 0;
}

void detector_normalize(const float raw[DETECTOR_BPM_COUNT], float score[DETECTOR_BPM_COUNT])
{
    float maximum = 0, low = 0, high = 0;
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i)
        if (isfinite(raw[i])) maximum = fmaxf(maximum, raw[i]);
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
        score[i] = isfinite(raw[i]) ? fmaxf(0, maximum - raw[i]) : 0;
        if (i < 40) low += score[i]; else high += score[i];
    }
    float slope = (high - low) / (40 * 40), intercept = low / 40 - slope * 20;
    float sum = 0;
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
        score[i] = fmaxf(0, score[i] - (intercept + slope * i));
        sum += score[i];
    }
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i)
        score[i] = sum > 1e-12f ? score[i] / sum : 1.0f / DETECTOR_BPM_COUNT;
}

static int best_pair(const float *scores)
{
    int best = 0;
    for (int i = 1; i < DETECTOR_BPM_COUNT - 1; ++i)
        if (scores[i] + scores[i + 1] > scores[best] + scores[best + 1]) best = i;
    return best;
}

static float support_at(const detector_t *d, int center)
{
    float sum = 0;
    for (int i = center - 1; i <= center + 1; ++i)
        if (i >= 0 && i < DETECTOR_BPM_COUNT) sum += d->support[i];
    return sum;
}

void detector_update(detector_t *d, const float scores[DETECTOR_BPM_COUNT], double time,
                     detector_control_t control, detector_result_t *out)
{
    memset(out, 0, sizeof(*out));
    float elapsed = d->have_time ? fmaxf(0, (float)(time - d->last_time)) : UPDATE_SECONDS;
    d->last_time = time;
    d->have_time = true;
    // Keep a four-second score memory even if jobs finish slower than requested.
    while (d->count && time - d->history_time[(d->write - d->count + DETECTOR_HISTORY) % DETECTOR_HISTORY]
           >= DETECTOR_HISTORY * UPDATE_SECONDS + 0.001) --d->count;
    memcpy(d->history[d->write], scores, sizeof(d->history[0]));
    d->history_time[d->write] = time;
    d->write = (d->write + 1) % DETECTOR_HISTORY;
    if (d->count < DETECTOR_HISTORY) ++d->count;
    for (int h = 0; h < d->count; ++h) {
        int slot = (d->write - 1 - h + DETECTOR_HISTORY) % DETECTOR_HISTORY;
        for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) out->scores[i] += d->history[slot][i] / d->count;
    }
    int pair = best_pair(out->scores), wins = 0;
    for (int h = 0; h < d->count; ++h) {
        int slot = (d->write - 1 - h + DETECTOR_HISTORY) % DETECTOR_HISTORY;
        int winner = best_pair(d->history[slot]);
        if (winner >= pair - 2 && winner <= pair + 2) ++wins;
        for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
            float delta = d->history[slot][i] - out->scores[i];
            out->variance += delta * delta / d->count;
        }
    }
    out->consistency = (float)wins / d->count;
    int candidate = pair + (out->scores[pair + 1] > out->scores[pair]);
    out->candidate_bpm = DETECTOR_BPM_MIN + candidate;
    float pair_score = out->scores[pair] + out->scores[pair + 1];

    if (control.force_revision != d->force_revision) {
        memset(d->support, 0, sizeof(d->support));
        if (control.hard_lock && control.forced_bpm >= DETECTOR_BPM_MIN &&
            control.forced_bpm <= DETECTOR_BPM_MAX) {
            d->held_bpm = control.forced_bpm;
            int bin = (int)control.forced_bpm - DETECTOR_BPM_MIN;
            d->held_pair = bin < DETECTOR_BPM_COUNT - 1 ? bin : bin - 1;
        }
        d->force_revision = control.force_revision;
    }
    float decay = exp2f(-elapsed / SUPPORT_HALF_LIFE);
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) d->support[i] *= decay;
    bool shock = d->have_variance && fabsf(out->variance - d->previous_variance) >= SHOCK_LIMIT;
    d->cooldown = shock ? SHOCK_CALM : fmaxf(0, d->cooldown - elapsed);
    d->previous_variance = out->variance;
    d->have_variance = true;
    float quality = fminf(pair_score / RHYTHM_MIN,
                         fminf(STABILITY_LIMIT / fmaxf(out->variance, 1e-12f),
                               out->consistency / CONSISTENCY_LIMIT));
    out->evidence = display_value(quality);
    // A delayed job is one observation, never seconds of uninterrupted support.
    out->trusted = quality >= 1 && d->cooldown <= 0 && d->count >= 2;
    if (out->trusted) d->support[candidate] += fminf(elapsed, UPDATE_SECONDS);
    quality = out->trusted ? quality : 0;
    float challenger = support_at(d, candidate);
    float required = control.phase_lock ? 2 : 1;
    float release = control.phase_lock ? 0.25f : 0.5f;
    if (!d->held_bpm) {
        out->confidence = fminf(quality, challenger);
        if (out->trusted && challenger >= 1) { d->held_bpm = out->candidate_bpm; d->held_pair = pair; }
    } else {
        int held_bin = (int)d->held_bpm - DETECTOR_BPM_MIN;
        int distance = candidate - held_bin;
        if (distance < 0) distance = -distance;
        float held = support_at(d, held_bin);
        bool max_slide = distance == 1 && candidate == DETECTOR_BPM_COUNT - 1;
        if (!distance) out->confidence = fminf(quality, held);
        else if (max_slide) out->confidence = fminf(quality, d->support[candidate] / required);
        else if (distance > 1)
            out->confidence = fminf(quality, fminf(challenger / required, release / fmaxf(held, 1e-12f)));
        if (!control.hard_lock && out->confidence >= 1 && distance) {
            d->held_bpm = out->candidate_bpm;
            d->held_pair = pair;
        }
    }
    out->confidence = display_value(out->confidence);
    float held_score = d->held_bpm ? out->scores[d->held_pair] + out->scores[d->held_pair + 1] : 0;
    if (control.hard_lock || !d->held_bpm) d->lost_seconds = 0;
    else if (out->trusted || held_score >= HELD_VISIBLE_MIN) d->lost_seconds = fmaxf(0, d->lost_seconds - 4 * elapsed);
    else d->lost_seconds += elapsed;
    out->lost_seconds = d->lost_seconds;
    out->held_bpm = d->held_bpm;
    int oldest = (d->write - d->count + DETECTOR_HISTORY) % DETECTOR_HISTORY;
    out->phase_stable = d->count >= 2 &&
        time - d->history_time[oldest] + elapsed >= DETECTOR_HISTORY * UPDATE_SECONDS - 0.001 &&
        out->variance <= STABILITY_LIMIT;
    if (d->lost_seconds >= LOST_TIMEOUT) {
        detector_reset(d);
        out->held_bpm = 0;
        out->stopped = true;
        out->confidence = 0;
    }
}

void detector_phase(detector_t *d, const float *frames, int count, double last_time,
                    bool locked, detector_result_t *out)
{
    if (out->held_bpm <= 0 || count < 3 || count > DETECTOR_FRAMES) return;
    float energy[DETECTOR_FRAMES], curve[DETECTOR_PHASE_BINS];
    for (int i = 0; i < count; ++i) {
        energy[i] = 0;
        for (int b = 0; b < DETECTOR_BANDS; ++b) energy[i] += frames[i * DETECTOR_BANDS + b] / DETECTOR_BANDS;
    }
    float step = FRAME_RATE * 60 / out->held_bpm;
    int best = 0;
    for (int p = 0; p < DETECTOR_PHASE_BINS; ++p) {
        float sum = 0;
        int n = 0;
        for (float pos = step * p / DETECTOR_PHASE_BINS; pos < count - 1; pos += step) {
            int i = (int)pos;
            sum += energy[i] + (energy[i + 1] - energy[i]) * (pos - i);
            ++n;
        }
        curve[p] = n ? sum / n : 0;
        if (curve[p] > curve[best]) best = p;
    }
    // Window-local bins must be mapped to the same absolute phase as the hold.
    double origin = (last_time - (count - 1) / (double)FRAME_RATE) * out->held_bpm / 60;
    float origin_phase = (float)(origin - floor(origin));
    float candidate = wrap(origin_phase + (float)best / DETECTOR_PHASE_BINS);
    if (!d->have_phase || d->phase_bpm != out->held_bpm) {
        d->phase_fraction = candidate;
        d->have_phase = true;
    } else if (!locked) {
        int held_index = (int)(wrap(d->phase_fraction - origin_phase) * DETECTOR_PHASE_BINS);
        int delta = best - held_index;
        if (delta < 0) delta = -delta;
        if (delta <= 1 || delta >= DETECTOR_PHASE_BINS - 1 || curve[best] >= curve[held_index] * 1.2f)
            d->phase_fraction = candidate;
    }
    d->phase_bpm = out->held_bpm;
    double period = 60.0 / out->held_bpm;
    out->beat_time = (floor(last_time / period - d->phase_fraction) + d->phase_fraction) * period;
    out->phase_valid = true;
    for (int p = 0; p < DETECTOR_PHASE_BINS; ++p) {
        float index = wrap((float)p / DETECTOR_PHASE_BINS - origin_phase) * DETECTOR_PHASE_BINS;
        int a = (int)index;
        out->phase[p] = curve[a] + (curve[(a + 1) % DETECTOR_PHASE_BINS] - curve[a]) * (index - a);
    }
}
