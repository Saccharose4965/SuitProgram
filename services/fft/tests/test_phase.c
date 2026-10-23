#include "fft_phase.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define RATE ((double)DETECTOR_SAMPLE_RATE / DETECTOR_FFT_SIZE)
static float frames[DETECTOR_FRAMES * DETECTOR_BANDS];

static void make_frames(int bpm, double last, double phase)
{
    for (int i = 0; i < DETECTOR_FRAMES; ++i) {
        double time = last - (DETECTOR_FRAMES - 1 - i) / RATE;
        double cycle = time * bpm / 60 - phase;
        cycle -= round(cycle);
        float pulse = (float)exp(-cycle * cycle / .008);
        for (int b = 0; b < DETECTOR_BANDS; ++b)
            frames[i * DETECTOR_BANDS + b] = 1 + (1 + b / 32.0f) * pulse;
    }
}

static double phase_error(double anchor, double bpm, double phase)
{
    double error = anchor * bpm / 60 - phase;
    return fabs(error - round(error));
}

int main(void)
{
    fft_phase_tracker_t tracker = {0};
    fft_phase_evidence_t evidence;
    make_frames(120, 123.14, .17);
    double start = 123.14 - (DETECTOR_FRAMES - 1) / RATE;
    assert(fft_phase_evidence(frames, DETECTOR_FRAMES, RATE, 120, start, &evidence));
    fft_phase_decision_t decision =
        fft_phase_update(&tracker, &evidence, 120, 123.14, true, false);
    assert(decision.valid && decision.acquired);
    assert(phase_error(decision.anchor, 120, .17) < .03);

    double held = decision.anchor;
    make_frames(120, 124.50, .67);
    start = 124.50 - (DETECTOR_FRAMES - 1) / RATE;
    assert(fft_phase_evidence(frames, DETECTOR_FRAMES, RATE, 120, start, &evidence));
    decision = fft_phase_update(&tracker, &evidence, 120, 124.50, true, true);
    assert(decision.valid && !decision.family_switched);
    assert(fabs(decision.anchor - held) < .04);

    fft_phase_tracker_lose(&tracker, 125);
    make_frames(105, 130, .32);
    start = 130 - (DETECTOR_FRAMES - 1) / RATE;
    assert(fft_phase_evidence(frames, DETECTOR_FRAMES, RATE, 105, start, &evidence));
    decision = fft_phase_update(&tracker, &evidence, 105, 130, true, false);
    assert(decision.acquired && phase_error(decision.anchor, 105, .32) < .03);

    fft_beat_clock_t clock = {0};
    fft_beat_clock_init(&clock, decision.precise_bpm, decision.anchor, 130);
    double boundary = clock.next_beat;
    fft_beat_clock_schedule(&clock, decision.precise_bpm, decision.anchor + .02);
    fft_beat_clock_update(&clock, boundary - .001);
    assert(clock.pending && clock.next_beat == boundary);
    fft_beat_clock_update(&clock, boundary + .001);
    assert(!clock.pending && clock.beat_start == boundary);
    puts("Phase tracker regressions passed (folded lock, tempo reacquire, boundary steering).");
    return 0;
}
