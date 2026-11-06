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

    int held_side = decision.side;
    make_frames(105, 130.75, .35);
    start = 130.75 - (DETECTOR_FRAMES - 1) / RATE;
    assert(fft_phase_evidence(frames, DETECTOR_FRAMES, RATE, 105, start, &evidence));
    decision = fft_phase_update(&tracker, &evidence, 105, 130.75, false, false);
    assert(decision.valid && !decision.corrected && !decision.family_switched);
    assert(fabs(decision.precise_bpm - 105) < 1e-9);
    assert(decision.side == held_side); // confidence loss cannot select the offbeat side

    fft_beat_clock_t clock = {0};
    fft_beat_clock_init(&clock, decision.precise_bpm, decision.anchor, 130);
    double boundary = clock.next_beat;
    fft_beat_clock_schedule(&clock, decision.precise_bpm, decision.anchor + .02);
    fft_beat_clock_update(&clock, boundary - .001);
    assert(clock.pending && clock.next_beat == boundary);
    fft_beat_clock_update(&clock, boundary + .001);
    assert(!clock.pending && clock.beat_start == boundary);
    assert(fabs(clock.period - clock.target_period) <= clock.target_period * .006001);

    // Task stalls consume missed beats; a resume at half-phase cannot flash or
    // redefine phase zero. Even a long suspension keeps the original grid.
    fft_beat_clock_reset(&clock);
    double last_emit = 0;
    fft_beat_clock_init(&clock, 150, 10, 10);
    assert(fft_beat_clock_emit(&clock, 10, &last_emit));
    assert(!fft_beat_clock_emit(&clock, 10.6, &last_emit));
    assert(fabs(clock.beat_start - 10.4) < 1e-8);
    assert(fft_beat_clock_emit(&clock, 10.8, &last_emit));
    assert(!fft_beat_clock_emit(&clock, 20.2, &last_emit));
    assert(fabs(clock.beat_start - 20.0) < 1e-8);
    assert(fft_beat_clock_emit(&clock, 20.4 + 1e-8, &last_emit));
    puts("Phase tracker regressions passed (folded lock, tempo reacquire, boundary steering).");
    return 0;
}
