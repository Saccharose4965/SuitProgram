// Compile the actual desktop source into this test; no sound/window is opened.
// Requires the reference's ALSA/SDL headers and libraries, not microphone access.
#define BPM_UPDATE_EVERY_FRAMES 88
#define main localfft_desktop_main
#include "../../../../localFFT/LatestFFT.c"
#undef main
#include "fft_detector.h"
#include "fft_phase.h"
#include <assert.h>

static float embedded_frames[DETECTOR_FRAMES * DETECTOR_BANDS];
static double reference_frames[DETECTOR_FRAMES * DETECTOR_BANDS];
static void test_scores_and_phase(void)
{
    const int tempos[] = {80, 105, 120, 138, 148, 159};
    double worst = 0;
    for (unsigned t = 0; t < sizeof(tempos) / sizeof(tempos[0]); ++t) {
        for (int i = 0; i < DETECTOR_FRAMES; ++i) {
            double cycle = i * tempos[t] * FFT_SIZE / (60.0 * SAMPLE_RATE) - .17;
            cycle -= round(cycle);
            for (int b = 0; b < COMB_BANDS; ++b) {
                float v = 4 + (1 + b / 32.0) * exp(-cycle * cycle / .008);
                embedded_frames[i * COMB_BANDS + b] = v;
                reference_frames[i * COMB_BANDS + b] = v;
            }
        }
        double raw[BPM_COUNT], positive[BPM_COUNT], corrected[BPM_COUNT], norm[BPM_COUNT];
        float embedded[BPM_COUNT], normalized[BPM_COUNT];
        compute_raw_comb_scores(reference_frames, DETECTOR_FRAMES, COMB_BANDS,
                                (double)SAMPLE_RATE / FFT_SIZE, raw);
        raw_scores_to_positive(raw, BPM_COUNT, positive);
        bpm_baseline_correct(positive, BPM_COUNT, corrected);
        normalize_scores_by_sum(corrected, BPM_COUNT, norm);
        for (int b = 0; b < BPM_COUNT; ++b) {
            embedded[b] = detector_comb_score(embedded_frames, DETECTOR_FRAMES, BPM_MIN + b);
            worst = fmax(worst, fabs(embedded[b] - raw[b]));
            assert(fabs(embedded[b] - raw[b]) < 1e-4);
        }
        detector_normalize(embedded, normalized);
        int desktop_best = 0, port_best = 0;
        for (int b = 0; b < BPM_COUNT; ++b) {
            assert(fabs(normalized[b] - norm[b]) < 1e-4);
            if (norm[b] > norm[desktop_best]) desktop_best = b;
            if (normalized[b] > normalized[port_best]) port_best = b;
        }
        assert(desktop_best == port_best);
        PhaseEvidence evidence;
        double frame_rate = (double)SAMPLE_RATE / FFT_SIZE;
        double last = (DETECTOR_FRAMES - 1) / frame_rate;
        double history_start = last - (DETECTOR_FRAMES - 1) / frame_rate;
        assert(compute_phase_evidence_mean(reference_frames, DETECTOR_FRAMES, COMB_BANDS,
                                           frame_rate, tempos[t], history_start, &evidence));
        fft_phase_evidence_t port_evidence;
        assert(fft_phase_evidence(embedded_frames, DETECTOR_FRAMES, frame_rate,
                                  tempos[t], history_start, &port_evidence));
        for (int i = 0; i < PHASE_GRAPH_BINS; ++i)
            assert(fabs(port_evidence.score[i] - evidence.score[PHASE_SCORE_MEAN][i]) < 1e-4);
        assert(port_evidence.best_family == evidence.bestFamily[PHASE_SCORE_MEAN]);
        assert(port_evidence.best_side == evidence.bestSide[PHASE_SCORE_MEAN]);

        PhaseTracker pc_tracker = {0};
        fft_phase_tracker_t port_tracker = {0};
        PhaseDecision pc_decision = phase_tracker_update(&pc_tracker, &evidence,
            PHASE_SCORE_MEAN, tempos[t], last, 1, 0);
        fft_phase_decision_t port_decision = fft_phase_update(&port_tracker,
            &port_evidence, tempos[t], last, true, false);
        assert(pc_decision.valid == port_decision.valid);
        assert(pc_decision.familyBin == port_decision.family);
        assert(pc_decision.selectedSide == port_decision.side);
        assert(fabs(pc_decision.flashAnchorTime - port_decision.anchor) < 1e-9);
        assert(fabs(pc_decision.preciseBpm - port_decision.precise_bpm) < 1e-9);
    }
    printf("Actual LatestFFT.c: scores, phase evidence, and initial tracker decisions match; max raw error %.8g\n", worst);
}

static void test_filter_reference(void)
{
    detector_filter_t port = {0}; ImpulseFilterState pc = {0};
    float logmag[FFT_SIZE / 2], bands[COMB_BANDS];
    for (int frame = 0; frame < 50; ++frame) {
        double reference[COMB_BANDS];
        for (int k = 0; k < FFT_SIZE / 2; ++k)
            logmag[k] = 1 + .2 * sin(k * .07 + frame * .03) + (frame == 25 ? 3 : 0);
        int first = (int)ceil(MIN_INT_FREQ_HZ * FFT_SIZE / SAMPLE_RATE);
        int bins = FFT_SIZE / 2 - first;
        for (int b = 0; b < COMB_BANDS; ++b) {
            int lo = first + b * bins / COMB_BANDS, hi = first + (b + 1) * bins / COMB_BANDS;
            reference[b] = 0;
            for (int k = lo; k < hi; ++k) reference[b] += logmag[k] / (double)(hi - lo);
        }
        int suppressed = filter_and_weight_band_frame(reference, &pc);
        assert(detector_filter_bands(&port, logmag, false, bands) == suppressed);
        for (int b = 0; b < COMB_BANDS; ++b) assert(fabs(bands[b] - reference[b]) < 1e-5);
    }
}

static void test_readiness_reference(void)
{
    // Compare the same established support state at sub-threshold evidence.
    // The port used to incorrectly force confidence to zero whenever untrusted.
    detector_t d = {0}; detector_result_t out;
    float scores[BPM_COUNT];
    for (int i = 0; i < BPM_COUNT; ++i) scores[i] = .96f / 78;
    scores[40] = .023f; scores[41] = .017f;
    double hop = DETECTOR_HOP_FRAMES * FFT_SIZE / (double)SAMPLE_RATE;
    detector_update(&d, scores, hop, (detector_control_t){0}, &out);
    d.held_bpm = 120; d.held_pair = 40; d.support[40] = 2;
    BpmHoldState pc = {.have=1, .bpm=120, .pairIdx=40, .previousEvalTime=hop,
        .havePreviousTime=1, .previousVariance=d.previous_variance, .havePreviousVariance=1};
    pc.support[40] = 2;
    detector_update(&d, scores, 2 * hop, (detector_control_t){0}, &out);
    double bpm, confidence, evidence; int pair, trusted;
    update_final_bpm_from_accum(120, 40, out.scores[40] + out.scores[41], out.consistency,
        out.variance, 2 * hop, 1, 0, &pc, &bpm, &pair, &confidence, &evidence, &trusted);
    assert(!trusted && !out.trusted);
    assert(out.confidence > .79f && out.confidence < .81f);
    assert(fabs(out.confidence - confidence) < 1e-5);
    assert(fabs(out.evidence - evidence) < 1e-5);
    assert(out.held_bpm == bpm);
}

static void test_phase_replay(void)
{
    PhaseTracker pc = {0};
    fft_phase_tracker_t port = {0};
    BeatClock pc_clock = {0};
    fft_beat_clock_t clock = {0};
    for (int n = 0; n < 1200; ++n) {
        double time = 100 + n * 8.0 * FFT_SIZE / SAMPLE_RATE;
        double bpm = n >= 500 && n < 580 ? 105 : 120;
        int certain = !(n >= 300 && n < 350);
        int locked = n >= 150 && n < 280;
        if (n == 480 || n == 900) {
            phase_tracker_lose_active(&pc, time);
            fft_phase_tracker_lose(&port, time);
        }
        PhaseEvidence evidence = {0};
        fft_phase_evidence_t e = {0};
        for (int i = 0; i < PHASE_GRAPH_BINS; ++i) {
            double phase = i / (double)PHASE_GRAPH_BINS;
            double shift = .12 + .025 * sin(n * .012);
            double a = phase_wrap_error(phase - shift, 1);
            double b = phase_wrap_error(phase - shift - .5, 1);
            double c = phase_wrap_error(phase - .38, 1);
            evidence.score[0][i] = exp(-a*a/.001) +
                (n % 200 < 100 ? .8 : 1.4) * exp(-b*b/.001) +
                (n >= 180 && n < 290 ? 3 : .2) * exp(-c*c/.001);
        }
        phase_finalize_evidence_mode(&evidence, PHASE_SCORE_MEAN);
        memcpy(e.score, evidence.score[0], sizeof(e.score));
        memcpy(e.folded, evidence.folded[0], sizeof(e.folded));
        e.best_family = evidence.bestFamily[0]; e.best_side = evidence.bestSide[0];
        e.raw_best_bin = e.best_family + e.best_side * PHASE_FAMILY_BINS;
        e.prominence = evidence.familyProminence[0];
        PhaseDecision a = phase_tracker_update(&pc, &evidence, PHASE_SCORE_MEAN,
                                               bpm, time, certain, locked);
        fft_phase_decision_t b = fft_phase_update(&port, &e, bpm, time, certain, locked);
        assert(a.valid == b.valid && a.acquired == b.acquired && a.restored == b.restored);
        assert(a.familySwitched == b.family_switched && a.selectedSide == b.side);
        assert(fabs(a.flashAnchorTime - b.anchor) < 1e-9);
        assert(fabs(a.preciseBpm - b.precise_bpm) < 1e-9);
        if (a.valid && certain) {
            if (!pc_clock.initialized || a.acquired || a.restored) {
                beat_clock_init_from_detected_beat(&pc_clock, a.preciseBpm, a.flashAnchorTime, time);
                fft_beat_clock_init(&clock, b.precise_bpm, b.anchor, time);
            } else if (a.correctionApplied || a.familySwitched) {
                beat_clock_schedule_from_detected_beat(&pc_clock, a.preciseBpm, a.flashAnchorTime);
                fft_beat_clock_schedule(&clock, b.precise_bpm, b.anchor);
            }
        }
        beat_clock_update(&pc_clock, time);
        fft_beat_clock_update(&clock, time);
        assert(fabs(pc_clock.nextBeatTime - clock.next_beat) < 1e-8);
    }
    puts("1200-step PC/ESP phase and beat-clock replay passed.");
}

static void test_bpm_replay(void)
{
    detector_t port = {0};
    BpmHoldState pc = {0};
    ScoreHistory history = {0};
    double time = 0, lost = 0, loss_time = 0;
    bool have_loss_time = false;
    const double hop = DETECTOR_HOP_FRAMES * FFT_SIZE / (double)SAMPLE_RATE;
    int stops = 0, switches = 0;
    for (int n = 0; n < 900; ++n) {
        int stage = n % 300;
        int bpm = stage < 70 ? 120 : stage < 140 ? 145 :
                  stage < 220 ? ((n & 1) ? 100 : 130) : 105;
        if (stage == 35 || stage == 36) bpm = 138; // short challenger
        float scores[BPM_COUNT]; double reference[BPM_COUNT], mean[BPM_COUNT];
        for (int i = 0; i < BPM_COUNT; ++i) scores[i] = .65f / 78;
        scores[bpm - BPM_MIN] = .25f;
        scores[bpm - BPM_MIN + 1] = .10f;
        for (int i = 0; i < BPM_COUNT; ++i) reference[i] = scores[i];
        time += (n % 67 == 66) ? 2 * hop : hop; // overdue worker
        score_history_push(&history, reference, time, hop);
        double variance, consistency, confidence, evidence, final_bpm;
        double coverage = score_history_stats(&history, mean, &variance, &consistency);
        int pair = strongest_pair(mean), final_pair, trusted;
        int candidate = pair + (mean[pair + 1] > mean[pair]);
        double previous = pc.have ? pc.bpm : 0;
        bool soft = n >= 300 && n < 600;
        update_final_bpm_from_accum(BPM_MIN + candidate, pair, mean[pair] + mean[pair+1],
            consistency, variance, time, 1, soft, &pc, &final_bpm, &final_pair,
            &confidence, &evidence, &trusted);
        bool changed = pc.have && pc.bpm != previous;
        switches += changed;
        lost = update_lost_pulse(lost, have_loss_time ? time - loss_time : .5,
            0, pc.have, held_score_ratio(mean, final_bpm), changed);
        loss_time = time; have_loss_time = true;
        bool stopped = pc.have && lost >= AUTO_STOP_LOST_SECONDS;
        if (stopped) {
            ++stops; bpm_hold_reset(&pc); lost = 0; have_loss_time = false;
        }
        detector_result_t out;
        detector_update(&port, scores, time, (detector_control_t){.phase_lock=soft}, &out);
        assert(out.held_bpm == (pc.have ? pc.bpm : 0));
        assert(out.stopped == stopped);
        assert(fabs(out.history_coverage - coverage) < 1e-5);
        assert(fabs(out.variance - variance) < 1e-6);
        assert(fabs(out.consistency - consistency) < 1e-5);
        if (!stopped) {
            if (out.trusted != trusted) fprintf(stderr, "step=%d trusted=%d/%d variance=%.9g/%.9g consistency=%.9g/%.9g cooldown=%.9g/%.9g evidence=%.9g/%.9g\n", n, out.trusted, trusted, out.variance, variance, out.consistency, consistency, port.cooldown, pc.shockCooldown, out.evidence, evidence);
            assert(out.trusted == trusted);
            assert(fabs(out.confidence - confidence) < 1e-4);
            assert(fabs(out.evidence - evidence) < 1e-4);
            assert(fabs(out.lost_seconds - lost) < 1e-4);
        }
    }
    assert(stops > 0 && switches > 5);
    puts("900-step PC/ESP BPM replay passed: acquisition, rejection, switching, soft lock, delayed jobs, loss/reacquisition.");
}

int main(void)
{
    test_scores_and_phase(); test_filter_reference(); test_readiness_reference();
    test_phase_replay();
    test_bpm_replay();
    puts("Actual localFFT reference regressions passed.");
}
