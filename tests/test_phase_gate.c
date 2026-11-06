#define main localfft_live_main
#include "../LatestFFT.c"
#undef main
#include <assert.h>

static void test_lost_pulse(void) {
    double scores[BPM_COUNT] = {0};
    scores[60] = 1.0;
    scores[40] = 0.59;
    assert(fabs(held_score_ratio(scores, 120) - .59) < 1e-12);
    double lost = update_lost_pulse(5, 2, 0, 1, held_score_ratio(scores, 120), 0);
    assert(lost == 7);
    scores[40] = .6;
    assert(update_lost_pulse(lost, 2, 0, 1, held_score_ratio(scores, 120), 0) == 0);
    scores[40] = .9;
    assert(update_lost_pulse(19, 20, 0, 1, held_score_ratio(scores, 120), 0) == 0);
    // Either neighbor contributes support; both range edges remain valid.
    memset(scores, 0, sizeof(scores));
    scores[39] = 1;
    assert(held_score_ratio(scores, 120) == 1);
    scores[39] = 0; scores[41] = 1;
    assert(held_score_ratio(scores, 120) == 1);
    memset(scores, 0, sizeof(scores)); scores[0] = 1;
    assert(held_score_ratio(scores, 80) == 1);
    memset(scores, 0, sizeof(scores)); scores[BPM_COUNT - 1] = 1;
    assert(held_score_ratio(scores, 159) == 1);
    assert(held_score_ratio(scores, NAN) == 0);
    memset(scores, 0, sizeof(scores));
    assert(held_score_ratio(scores, 120) == 0);
    assert(update_lost_pulse(AUTO_STOP_LOST_SECONDS - .5, .5, 0, 1, 0, 0) == AUTO_STOP_LOST_SECONDS);
    assert(update_lost_pulse(AUTO_STOP_LOST_SECONDS - 1, 1, 0, 1, NAN, 0) == AUTO_STOP_LOST_SECONDS);
    assert(update_lost_pulse(19, 1, 0, 1, 0, 1) == 0);
    assert(update_lost_pulse(19, 1, 1, 1, 0, 0) == 0);
    assert(update_lost_pulse(19, 1, 0, 0, 0, 0) == 0);
    assert(update_lost_pulse(5, NAN, 0, 1, 0, 0) == 5);
    puts("Lost-pulse tests passed: ratio threshold, immediate reset, missing evidence, range edges and timeout.");
}

static void test_phase_correction_waits_for_boundary(void) {
    BeatClock clock = {0};
    beat_clock_init_from_detected_beat(&clock, 120, 10.0, 10.0);
    double oldStart = clock.beatStartTime;
    double oldNext = clock.nextBeatTime;
    beat_clock_schedule_from_detected_beat(&clock, 119.5, 10.52);
    assert(clock.beatStartTime == oldStart && clock.nextBeatTime == oldNext);
    beat_clock_update(&clock, 10.49);
    assert(clock.beatStartTime == oldStart && clock.nextBeatTime == oldNext);
    beat_clock_update(&clock, 10.50);
    assert(!clock.hasPendingBpm);
    assert(clock.beatStartTime == oldNext); // no boundary snap
    assert(fabs(clock.periodCurrent - clock.periodTarget) <=
           clock.periodTarget * BEAT_PHASE_STEER_MAX_RATIO + 1e-12);
}

static void test_suppressed_boundary_stays_suppressed(void) {
    BeatClock clock = {0}; double last = 0;
    beat_clock_init_from_detected_beat(&clock, 120, 10, 10);
    assert(beat_clock_emit(&clock, 10, &last));
    beat_clock_init_from_detected_beat(&clock, 120, 10.1, 10.1);
    assert(!beat_clock_emit(&clock, 10.1, &last));
    assert(!beat_clock_emit(&clock, 10.3, &last));
    assert(!beat_clock_emit(&clock, 10.49, &last));
    assert(beat_clock_flash_phase(&clock, 10.3, last) > .59);
    assert(beat_clock_emit(&clock, 10.6, &last));
    assert(beat_clock_flash_phase(&clock, 10.6, last) == 0);
    // Reinitialization between beats waits for a future boundary.
    memset(&clock, 0, sizeof(clock)); last = 0;
    beat_clock_init_from_detected_beat(&clock, 120, 10, 10.2);
    assert(!beat_clock_emit(&clock, 10.2, &last));
    assert(beat_clock_emit(&clock, 10.5, &last));
    puts("Emission gate: rejected boundaries stay rejected; flash brightness follows accepted emissions.");
}

int main(void) {
    BeatClock view_clock = {.initialized = 1, .beatStartTime = 10.125};
    assert(fabs(phase_graph_origin(&view_clock, 120) - .25) < 1e-12);
    // A new boundary changes the graph origin; phase targets and ratio do not.
    view_clock.beatStartTime += .003;
    assert(fabs(phase_graph_origin(&view_clock, 120) - .256) < 1e-12);
    assert(fabs(phase_wrap_error(phase_relative_to_clock(.256, &view_clock, 120), 1)) < 1e-12);
    assert(fabs(phase_relative_to_clock(.756, &view_clock, 120) - .5) < 1e-12);
    // Sample zero is the emitted boundary, without a half-cycle offset.
    assert(phase_graph_sample_bin(.25, 0, 128, 1) == 32);
    assert(phase_graph_sample_bin(.25, 64, 128, 1) == 96);
    for (int column = 0; column < 128; ++column) {
        double unshifted = .25 + (double)column / 128 / 2;
        unshifted -= floor(unshifted);
        int bin = (int)floor(unshifted * 128 + 1e-4) % 64;
        assert(phase_graph_sample_bin(.25, column, 128, 2) == bin);
    }
    Uint64 last = 0; unsigned presses = 0;
    assert(timing_button_next(0, 1000, &last, &presses) == 1);
    assert(timing_button_next(1, 1200, &last, &presses) == 2);
    assert(timing_button_next(2, 1400, &last, &presses) == 0);
    assert(timing_button_next(0, 1600, &last, &presses) == 1);
    assert(timing_button_next(1, 3000, &last, &presses) == 0);
    // A spectral peak at phase .25 and a clock anchored there share phase zero
    // at every emitted beat, regardless of the wall-time epoch or capture lag.
    BeatClock aligned = {0};
    beat_clock_init_from_detected_beat(&aligned, 120, 10.125, 10.125);
    for (int i = 1; i <= 20; ++i) {
        double sampleTime = 10.125 + i * .5;
        beat_clock_update(&aligned, sampleTime);
        assert(fabs((sampleTime - aligned.beatStartTime) / aligned.periodCurrent) < 1e-10);
        assert(fabs(phase_wrap_error(phase_relative_to_clock(.25, &aligned, 120), 1)) < 1e-10);
    }
    BeatClock emission = {0};
    double lastEmit = 0;
    beat_clock_init_from_detected_beat(&emission, 120, 10, 10);
    assert(beat_clock_emit(&emission, 10, &lastEmit));
    assert(!beat_clock_emit(&emission, 10.1, &lastEmit));
    // A poll late in a cycle consumes the missed beat without flashing.
    assert(!beat_clock_emit(&emission, 10.9, &lastEmit));
    assert(!beat_clock_emit(&emission, 10.91, &lastEmit));
    assert(beat_clock_emit(&emission, 11.01, &lastEmit));
    // A control reinitialization cannot emit twice within half a period.
    beat_clock_init_from_detected_beat(&emission, 120, 11.05, 11.05);
    assert(!beat_clock_emit(&emission, 11.05, &lastEmit));
    assert(!beat_clock_emit(&emission, 20.3, &lastEmit));
    assert(fabs(emission.beatStartTime - 20.05) < 1e-8);
    test_suppressed_boundary_stays_suppressed();
    test_lost_pulse();
    test_phase_correction_waits_for_boundary();
    assert(bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS, 1, 1, 120, 120));
    assert(!bpm_confident_for_phase(0, STABILITY_WINDOW_SECONDS, 1, 1, 120, 120));
    assert(!bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS - 1, 1, 1, 120, 120));
    assert(!bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS, 0, 1.1, 120, 120));
    assert(!bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS, 1, .99, 120, 120));
    assert(!bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS, 1, 1.1, 121, 120));
    assert(!bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS, 1, 1.1, 140, 120));
    assert(!bpm_confident_for_phase(1, STABILITY_WINDOW_SECONDS, 1, NAN, 120, 120));
    puts("Phase gate passed: confidence, trust, history and BPM agreement.");
}
