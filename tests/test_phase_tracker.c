#include "../include/bpm_core.h"
#include "../include/phase_tracker.h"
#include <assert.h>
#include <stdio.h>

static PhaseEvidence evidence_at(int family, int side, double peak, double runner) {
    PhaseEvidence evidence = {0};
    for (int mode = 0; mode < PHASE_SCORE_MODE_COUNT; ++mode) {
        evidence.folded[mode][family] = peak;
        evidence.score[mode][family + side * PHASE_FAMILY_BINS] = peak * .7;
        evidence.score[mode][family + (1 - side) * PHASE_FAMILY_BINS] = peak * .3;
        int other = (family + PHASE_FAMILY_BINS / 2) % PHASE_FAMILY_BINS;
        evidence.folded[mode][other] = runner;
        evidence.bestFamily[mode] = family;
        evidence.bestSide[mode] = side;
        evidence.bestFoldedScore[mode] = peak;
        evidence.runnerFoldedScore[mode] = runner;
        evidence.familyProminence[mode] = peak / runner;
    }
    return evidence;
}

static void test_equal_folding_and_robust_aggregation(void) {
    double frames[24 * 2];
    for (int i = 0; i < 24; ++i) {
        frames[i * 2] = 1;
        frames[i * 2 + 1] = 2;
    }
    frames[4] = frames[5] = 1000; // isolated event affects mean most
    PhaseEvidence evidence;
    assert(compute_phase_evidence(frames, 24, 2, 8, 120, 0, &evidence));
    PhaseEvidence fastMean;
    assert(compute_phase_evidence_mean(frames, 24, 2, 8, 120, 0, &fastMean));
    for (int bin = 0; bin < PHASE_GRAPH_BINS; ++bin) {
        assert(fabs(fastMean.score[PHASE_SCORE_MEAN][bin] -
                    evidence.score[PHASE_SCORE_MEAN][bin]) < 1e-12);
    }
    int family = evidence.bestFamily[PHASE_SCORE_MEAN];
    assert(fabs(evidence.folded[PHASE_SCORE_MEAN][family] -
                (evidence.score[PHASE_SCORE_MEAN][family] +
                 evidence.score[PHASE_SCORE_MEAN][family + PHASE_FAMILY_BINS])) < 1e-12);
    double values[] = {1, 2, 3, 4, 1000};
    assert(phase_aggregate(values, 5, PHASE_SCORE_MEAN) == 202);
    assert(phase_aggregate(values, 5, PHASE_SCORE_MEDIAN) == 3);
    assert(phase_aggregate(values, 5, PHASE_SCORE_TRIMMED) == 3);
}

static void test_family_side_lock_and_previous(void) {
    PhaseTracker tracker = {0};
    PhaseEvidence first = evidence_at(10, 1, 10, 2);
    PhaseDecision d = phase_tracker_update(&tracker, &first, PHASE_SCORE_MEAN, 120, 10, 1, 0);
    assert(d.valid && d.acquired && tracker.active.side == 1 && d.familyBin == 10);

    // Opposite side dominance is retained as information but cannot flip selection.
    PhaseEvidence opposite = evidence_at(10, 0, 11, 2);
    d = phase_tracker_update(&tracker, &opposite, PHASE_SCORE_MEAN, 120, 12, 1, 0);
    assert(d.valid && tracker.active.side == 1);
    phase_tracker_toggle_side(&tracker);
    assert(tracker.active.side == 0);

    // A locked family cannot follow a persistent distant winner.
    PhaseEvidence distant = evidence_at(35, 0, 20, 1);
    for (int i = 0; i < 6; ++i)
        d = phase_tracker_update(&tracker, &distant, PHASE_SCORE_MEAN, 120, 14 + i * 2, 1, 1);
    assert(!d.familySwitched && phase_circular_distance(d.familyBin, 10, PHASE_FAMILY_BINS) <= PHASE_LOCAL_RADIUS_BINS);

    phase_tracker_lose_active(&tracker, 26);
    assert(!tracker.active.have && tracker.havePrevious);
    d = phase_tracker_update(&tracker, &first, PHASE_SCORE_MEAN, 120, 30, 1, 0);
    assert(d.restored && tracker.active.side == 0);
    phase_tracker_lose_active(&tracker, 31);
    d = phase_tracker_update(&tracker, &first, PHASE_SCORE_MEAN, 120,
                             31 + PHASE_PREVIOUS_EXPIRY_SECONDS + 1, 1, 0);
    assert(d.acquired && !d.restored && tracker.active.side == 1);
}

static void test_drift_correction_is_bounded(void) {
    PhaseTracker tracker = {0};
    PhaseEvidence evidence = evidence_at(0, 0, 10, 2);
    PhaseDecision d = phase_tracker_update(&tracker, &evidence, PHASE_SCORE_MEAN, 120, 10, 1, 0);
    double nominal = 60.0 / 120;
    for (int i = 1; i <= 20; ++i) {
        int family = (i / 2) % PHASE_FAMILY_BINS;
        evidence = evidence_at(family, 0, 10, 2);
        d = phase_tracker_update(&tracker, &evidence, PHASE_SCORE_MEAN, 120, 10 + 2 * i, 1, 1);
        assert(fabs(tracker.active.period - nominal) < 1e-12);
        assert(fabs(d.correctionSeconds) <= PHASE_MAX_POSITION_STEP_SECONDS + 1e-12);
    }
    assert(d.valid && d.preciseBpm > 0);
}

int main(void) {
    test_equal_folding_and_robust_aggregation();
    test_family_side_lock_and_previous();
    test_drift_correction_is_bounded();
    puts("Phase tracker: fast-mean parity, aggregation, folding, stable side/family and drift passed.");
    return 0;
}
