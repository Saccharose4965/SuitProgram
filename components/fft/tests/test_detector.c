// Host regressions: compile with fft_detector.c and -lm; no ESP-IDF required.
#include "fft_detector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define HOP ((double)DETECTOR_HOP_FRAMES * DETECTOR_FFT_SIZE / DETECTOR_SAMPLE_RATE)
#define RATE ((double)DETECTOR_SAMPLE_RATE / DETECTOR_FFT_SIZE)
static float frames[DETECTOR_FRAMES * DETECTOR_BANDS];

// Independent, double-precision PC formula, with the original loop order.
static double reference_score(int bpm)
{
    double step = RATE * 60 / bpm, total = 0;
    int phases = (int)ceil(step * 8), evals = 0;
    if (phases > 256) phases = 256;
    for (int p = 0; p < phases; ++p) for (int b = 0; b < DETECTOR_BANDS; ++b) {
        double sum = 0, square = 0;
        int n = 0;
        for (double pos = step * p / phases; pos < DETECTOR_FRAMES - 1; pos += step) {
            int i = (int)pos;
            double a = frames[i * DETECTOR_BANDS + b];
            double x = a + (frames[(i + 1) * DETECTOR_BANDS + b] - a) * (pos - i);
            sum += x; square += x * x; ++n;
        }
        if (n >= 2) { total += (square - sum * sum / n) / (n - 1); ++evals; }
    }
    return total / evals;
}

static void make_frames(int bpm, double last_time, double phase, float baseline)
{
    for (int i = 0; i < DETECTOR_FRAMES; ++i) {
        double time = last_time - (DETECTOR_FRAMES - 1 - i) / RATE;
        double cycle = time * bpm / 60 - phase;
        cycle -= round(cycle);
        float pulse = (float)exp(-cycle * cycle / 0.008);
        for (int b = 0; b < DETECTOR_BANDS; ++b)
            frames[i * DETECTOR_BANDS + b] = baseline + (1 + b / 32.0f) * pulse;
    }
}

static void test_comb(void)
{
    float raw[DETECTOR_BPM_COUNT], score[DETECTOR_BPM_COUNT];
    float reference_raw[DETECTOR_BPM_COUNT], reference_normalized[DETECTOR_BPM_COUNT];
    const int tempos[] = {80, 105, 120, 138, 148, 159};
    double worst = 0;
    for (unsigned t = 0; t < sizeof(tempos) / sizeof(tempos[0]); ++t) {
        make_frames(tempos[t], 123.14, .17, 10);
        int best = 0, reference_best = 0;
        for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
            raw[i] = detector_comb_score(frames, DETECTOR_FRAMES, DETECTOR_BPM_MIN + i);
            double reference = reference_score(DETECTOR_BPM_MIN + i);
            reference_raw[i] = (float)reference;
            double error = fabs(raw[i] - reference);
            if (error > worst) worst = error;
            assert(error < 0.0001);
        }
        detector_normalize(raw, score);
        detector_normalize(reference_raw, reference_normalized);
        float sum = 0;
        for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
            assert(isfinite(score[i]) && score[i] >= 0);
            sum += score[i];
            if (score[i] > score[best]) best = i;
            if (reference_normalized[i] > reference_normalized[reference_best]) reference_best = i;
        }
        assert(fabsf(sum - 1) < 0.00001f);
        assert(best == reference_best);
        // Existing range-edge octave ambiguity: a uniform 159 pulse can prefer
        // 80 (~159/2). Record parity; do not disguise it as a newly solved case.
        assert(best + DETECTOR_BPM_MIN == (tempos[t] == 159 ? 80 : tempos[t]));
    }
    for (unsigned i = 0; i < sizeof(frames) / sizeof(frames[0]); ++i) frames[i] = 9.123f;
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
        raw[i] = detector_comb_score(frames, DETECTOR_FRAMES, DETECTOR_BPM_MIN + i);
        assert(raw[i] == 0);
    }
    detector_normalize(raw, score);
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) assert(score[i] == 1.0f / DETECTOR_BPM_COUNT);
    printf("comb: 480 float/double comparisons, max absolute error %.8g\n", worst);
}

static void test_filter(void)
{
    detector_filter_t f = {0};
    float spectrum[DETECTOR_FFT_SIZE / 2], bands[DETECTOR_BANDS], previous[DETECTOR_BANDS];
    for (unsigned k = 0; k < sizeof(spectrum) / sizeof(spectrum[0]); ++k) spectrum[k] = 1;
    assert(!detector_filter_bands(&f, spectrum, false, bands));
    assert(bands[15] == 1 && fabsf(bands[16] - sqrtf(2)) < 1e-6f && bands[24] == 1);
    memcpy(previous, bands, sizeof(bands));
    for (unsigned k = 0; k < sizeof(spectrum) / sizeof(spectrum[0]); ++k) spectrum[k] = 3;
    assert(detector_filter_bands(&f, spectrum, false, bands));
    assert(memcmp(bands, previous, sizeof(bands)) == 0);
    assert(detector_filter_bands(&f, spectrum, true, bands));
    assert(memcmp(bands, previous, sizeof(bands)) == 0);
    assert(!detector_filter_bands(&f, spectrum, false, bands));
    assert(bands[0] == 3);
}

static void make_scores(float *score, int bpm)
{
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) score[i] = bpm ? 0.65f / 78 : 1.0f / 80;
    if (bpm) {
        int bin = bpm - DETECTOR_BPM_MIN;
        score[bin] = 0.25f;
        score[bin == 79 ? 78 : bin + 1] = 0.10f;
    }
}

static detector_result_t feed(detector_t *d, detector_control_t c, int bpm, int updates, double interval)
{
    float scores[DETECTOR_BPM_COUNT];
    detector_result_t out = {0};
    make_scores(scores, bpm);
    for (int i = 0; i < updates; ++i) detector_update(d, scores, d->last_time + interval, c, &out);
    return out;
}

static detector_result_t feed_unstable_loss(detector_t *d, detector_control_t c,
                                             int updates, double interval)
{
    float scores[DETECTOR_BPM_COUNT];
    detector_result_t out = {0};
    for (int i = 0; i < updates; ++i) {
        int sequence = (int)lround(d->last_time / interval);
        make_scores(scores, (sequence & 1) ? 100 : 145);
        detector_update(d, scores, d->last_time + interval, c, &out);
    }
    return out;
}

static void test_hold(void)
{
    detector_t d = {0};
    detector_control_t c = {0};
    detector_result_t out = feed(&d, c, 0, 80, HOP);
    assert(out.held_bpm == 0 && out.confidence < 1 && !out.trusted);
    detector_reset(&d);
    out = feed(&d, c, 120, 8, HOP);
    assert(out.held_bpm == 120 && out.phase_stable && out.confidence >= 1);
    out = feed(&d, c, 145, 2, HOP);
    assert(out.held_bpm == 120 && out.confidence < 1);
    out = feed(&d, c, 120, 20, HOP);
    assert(out.held_bpm == 120);
    detector_t soft = d;
    int normal_switch = 0, soft_switch = 0;
    for (int i = 1; i <= 130; ++i) {
        out = feed(&d, c, 145, 1, HOP);
        if (out.held_bpm == 145 && !normal_switch) normal_switch = i;
        detector_control_t locked = {.phase_lock = true};
        out = feed(&soft, locked, 145, 1, HOP);
        if (out.held_bpm == 145 && !soft_switch) soft_switch = i;
    }
    assert(normal_switch > 10 && soft_switch > normal_switch);
    out = feed(&d, c, 146, 130, HOP);
    assert(out.held_bpm == 145); // adjacent sliding remains disabled
    detector_reset(&d);
    out = feed(&d, c, 158, 20, HOP);
    assert(out.held_bpm == 158);
    out = feed(&d, c, 159, 40, HOP);
    assert(out.held_bpm == 159); // only the existing upper-bound exception
    out = feed(&d, c, 158, 80, HOP);
    assert(out.held_bpm == 159);

    c.hard_lock = true; c.forced_bpm = 105; c.force_revision = 1;
    out = feed(&d, c, 138, 100, HOP);
    assert(out.held_bpm == 105);
    out = feed(&d, c, 0, 100, HOP);
    assert(out.held_bpm == 105 && out.lost_seconds == 0);
    c.hard_lock = false; ++c.force_revision;
    out = feed(&d, c, 138, 100, HOP);
    assert(out.held_bpm == 138);
    printf("hold: transient rejected; sustained switch %.1fs, phase-soft-lock %.1fs\n",
           normal_switch * HOP, soft_switch * HOP);
}

static void test_loss_and_scheduling(void)
{
    detector_t d = {0};
    detector_control_t c = {0};
    detector_result_t out = feed(&d, c, 120, 30, HOP);
    assert(out.held_bpm == 120);
    out = feed_unstable_loss(&d, c, 20, HOP);
    assert(out.held_bpm == 120 && !out.stopped); // ten-second breakdown
    out = feed(&d, c, 120, 40, HOP);
    assert(out.held_bpm == 120 && out.lost_seconds == 0);
    bool stopped = false;
    for (int i = 0; i < 100; ++i) {
        out = feed_unstable_loss(&d, c, 1, HOP);
        if (out.stopped) { stopped = true; break; }
    }
    assert(stopped && out.held_bpm == 0);
    out = feed(&d, c, 120, 40, HOP);
    assert(out.held_bpm == 120);
    detector_reset(&d);
    out = feed(&d, c, 120, 2, 10);
    assert(out.held_bpm == 0 && out.trusted); // isolated delayed evidence never accumulates enough support
    detector_reset(&d);
    out = feed(&d, c, 120, 20, 2 * HOP);
    assert(out.held_bpm == 120 && d.count < DETECTOR_HISTORY);
}

static double phase_distance(double beat_time, int bpm, double desired)
{
    double error = beat_time * bpm / 60 - desired;
    return fabs(error - round(error));
}

static void test_phase(void)
{
    detector_t d = {0};
    detector_result_t out = {.held_bpm = 120};
    make_frames(120, 123.14, .17, 1);
    detector_phase(&d, frames, DETECTOR_FRAMES, 123.14, false, &out);
    assert(out.phase_valid && phase_distance(out.beat_time, 120, .17) < .03);
    make_frames(120, 124.377, .17, 1);
    detector_phase(&d, frames, DETECTOR_FRAMES, 124.377, false, &out);
    assert(phase_distance(out.beat_time, 120, .17) < .03);
    double held = d.phase_fraction;
    make_frames(120, 128.191, .67, 1);
    detector_phase(&d, frames, DETECTOR_FRAMES, 128.191, true, &out);
    assert(d.phase_fraction == held);
    assert(fabsf(out.accepted_phase - (float)held) < 1e-6f);
    assert(fabsf(out.candidate_phase - .67f) < .03f);
    detector_phase(&d, frames, DETECTOR_FRAMES, 128.191, false, &out);
    assert(phase_distance(out.beat_time, 120, .67) < .03);
    out.held_bpm = 105;
    make_frames(105, 132.711, .32, 1);
    detector_phase(&d, frames, DETECTOR_FRAMES, 132.711, true, &out);
    assert(phase_distance(out.beat_time, 105, .32) < .03); // re-anchor on a genuine tempo change
}

int main(void)
{
    test_comb(); test_filter(); test_hold(); test_loss_and_scheduling(); test_phase();
    puts("All detector regressions passed.");
    return 0;
}
