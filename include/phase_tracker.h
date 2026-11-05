#ifndef SUIT_PHASE_TRACKER_H
#define SUIT_PHASE_TRACKER_H

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef PHASE_GRAPH_BINS
#define PHASE_GRAPH_BINS 128
#endif
#define PHASE_FAMILY_BINS (PHASE_GRAPH_BINS / 2)
#define PHASE_MAX_TEETH 64
#define PHASE_PREVIOUS_EXPIRY_SECONDS 45.0
#define PHASE_LOCAL_RADIUS_BINS 10
#define PHASE_FAMILY_SWITCH_RATIO 1.20
#define PHASE_FAMILY_SWITCH_SECONDS 6.0
#define PHASE_MAX_BPM_DIFF 0.5
#define PHASE_RESTORE_MAX_CYCLE_ERROR 0.10
#define PHASE_POSITION_GAIN 0.20
#define PHASE_MAX_POSITION_STEP_SECONDS 0.030
#define PHASE_ERROR_FILTER_SECONDS 0.80
#define PHASE_ERROR_DEADBAND_BINS 0.10
#define PHASE_MIN_TRACK_PROMINENCE 1.05

typedef enum {
    PHASE_SCORE_MEAN = 0,
    PHASE_SCORE_MEDIAN = 1,
    PHASE_SCORE_TRIMMED = 2,
    PHASE_SCORE_MODE_COUNT = 3
} PhaseScoreMode;

typedef struct {
    double score[PHASE_SCORE_MODE_COUNT][PHASE_GRAPH_BINS];
    double folded[PHASE_SCORE_MODE_COUNT][PHASE_FAMILY_BINS];
    int bestFamily[PHASE_SCORE_MODE_COUNT];
    int bestSide[PHASE_SCORE_MODE_COUNT];
    double bestFoldedScore[PHASE_SCORE_MODE_COUNT];
    double runnerFoldedScore[PHASE_SCORE_MODE_COUNT];
    double familyProminence[PHASE_SCORE_MODE_COUNT];
    int teeth;
} PhaseEvidence;

typedef struct {
    int have;
    double bpm;
    double period;
    double anchorTime;
    double lastUpdateTime;
    double confidence;
    double lastErrorSeconds;
    double filteredErrorSeconds;
    int familyBin;
    int side;
    int challengerBin;
    double challengerSeconds;
} PhaseTrack;

typedef struct {
    PhaseTrack active;
    PhaseTrack previous;
    double previousSavedTime;
    int havePrevious;
} PhaseTracker;

typedef struct {
    int valid;
    int acquired;
    int restored;
    int familySwitched;
    int correctionApplied;
    int selectedSide;
    int familyBin;
    int rawBestBin;
    double flashAnchorTime;
    double preciseBpm;
    double phaseErrorSeconds;
    double correctionSeconds;
    double familyScore;
    double familyProminence;
    double challengerReadiness;
} PhaseDecision;

static inline double phase_wrap01(double value) {
    value -= floor(value);
    return value < 0.0 ? value + 1.0 : value;
}

static inline double phase_wrap_half(double value) {
    value = fmod(value, 0.5);
    return value < 0.0 ? value + 0.5 : value;
}

static inline double phase_wrap_error(double value, double period) {
    if (!isfinite(value) || !isfinite(period) || period <= 0.0) return 0.0;
    value -= round(value / period) * period;
    return value;
}

static inline double phase_clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
}

static inline int phase_circular_distance(int a, int b, int count) {
    int delta = abs(a - b) % count;
    return delta < count - delta ? delta : count - delta;
}

static inline int phase_double_compare(const void *left, const void *right) {
    double a = *(const double *)left, b = *(const double *)right;
    return (a > b) - (a < b);
}

static inline double phase_aggregate(double *values, int count, PhaseScoreMode mode) {
    if (!values || count <= 0) return 0.0;
    if (mode == PHASE_SCORE_MEAN) {
        double sum = 0.0;
        for (int i = 0; i < count; ++i) sum += values[i];
        return sum / count;
    }
    double ordered[PHASE_MAX_TEETH];
    memcpy(ordered, values, sizeof(double) * (size_t)count);
    qsort(ordered, (size_t)count, sizeof(double), phase_double_compare);
    if (mode == PHASE_SCORE_MEDIAN) {
        int middle = count / 2;
        return count & 1 ? ordered[middle] : 0.5 * (ordered[middle - 1] + ordered[middle]);
    }
    int first = count >= 5 ? 1 : 0;
    int end = count >= 5 ? count - 1 : count;
    double sum = 0.0;
    for (int i = first; i < end; ++i) sum += ordered[i];
    return sum / (double)(end - first);
}

static inline void phase_finalize_evidence_mode(PhaseEvidence *out, PhaseScoreMode mode) {
    double floorValue = out->score[mode][0];
    for (int bin = 1; bin < PHASE_GRAPH_BINS; ++bin)
        if (out->score[mode][bin] < floorValue) floorValue = out->score[mode][bin];
    for (int bin = 0; bin < PHASE_GRAPH_BINS; ++bin)
        out->score[mode][bin] = fmax(0.0, out->score[mode][bin] - floorValue);
    int best = 0;
    for (int family = 0; family < PHASE_FAMILY_BINS; ++family) {
        double a = out->score[mode][family];
        double b = out->score[mode][family + PHASE_FAMILY_BINS];
        out->folded[mode][family] = a + b;
        if (out->folded[mode][family] > out->folded[mode][best]) best = family;
    }
    double runner = 0.0;
    for (int family = 0; family < PHASE_FAMILY_BINS; ++family)
        if (phase_circular_distance(family, best, PHASE_FAMILY_BINS) > 3 &&
            out->folded[mode][family] > runner)
            runner = out->folded[mode][family];
    out->bestFamily[mode] = best;
    out->bestSide[mode] = out->score[mode][best + PHASE_FAMILY_BINS] > out->score[mode][best];
    out->bestFoldedScore[mode] = out->folded[mode][best];
    out->runnerFoldedScore[mode] = runner;
    out->familyProminence[mode] = runner > 1e-12 ? out->folded[mode][best] / runner : HUGE_VAL;
}

// Embedded/live path: compute only the temporal mean. This avoids retaining and
// sorting tooth samples for the benchmark-only median and trimmed alternatives.
static inline int compute_phase_evidence_mean(const double *frames, int frameCount, int bands,
                                              double frameRate, double bpm,
                                              double historyStartTime, PhaseEvidence *out) {
    if (!frames || !out || frameCount <= 2 || bands <= 0 || frameRate <= 0.0 || bpm <= 0.0)
        return 0;
    memset(out, 0, sizeof(*out));
    double step = frameRate * 60.0 / bpm;
    double startPhase = phase_wrap01(historyStartTime * bpm / 60.0);
    int maximumTeeth = 0;
    for (int absoluteBin = 0; absoluteBin < PHASE_GRAPH_BINS; ++absoluteBin) {
        double relativePhase = phase_wrap01((double)absoluteBin / PHASE_GRAPH_BINS - startPhase);
        double sum = 0.0;
        int toothCount = 0;
        for (double position = relativePhase * step;
             position < frameCount - 1 && toothCount < PHASE_MAX_TEETH;
             position += step) {
            int index = (int)floor(position);
            double fraction = position - index;
            double energy = 0.0;
            int validBands = 0;
            for (int band = 0; band < bands; ++band) {
                double a = frames[index * bands + band];
                double b = frames[(index + 1) * bands + band];
                double value = a + (b - a) * fraction;
                if (isfinite(value)) { energy += value; ++validBands; }
            }
            if (validBands) { sum += energy / validBands; ++toothCount; }
        }
        if (toothCount > maximumTeeth) maximumTeeth = toothCount;
        out->score[PHASE_SCORE_MEAN][absoluteBin] = toothCount ? sum / toothCount : 0.0;
    }
    out->teeth = maximumTeeth;
    phase_finalize_evidence_mode(out, PHASE_SCORE_MEAN);
    return maximumTeeth > 0;
}

// Compute phase curves in absolute nominal-BPM phase coordinates. Each tooth first
// averages all bands; temporal mean/median/trimmed aggregation then compares phases.
static inline int compute_phase_evidence(const double *frames, int frameCount, int bands,
                                         double frameRate, double bpm,
                                         double historyStartTime, PhaseEvidence *out) {
    if (!frames || !out || frameCount <= 2 || bands <= 0 || frameRate <= 0.0 || bpm <= 0.0)
        return 0;
    memset(out, 0, sizeof(*out));
    double step = frameRate * 60.0 / bpm;
    double startPhase = phase_wrap01(historyStartTime * bpm / 60.0);
    int maximumTeeth = 0;
    for (int absoluteBin = 0; absoluteBin < PHASE_GRAPH_BINS; ++absoluteBin) {
        double absolutePhase = (double)absoluteBin / PHASE_GRAPH_BINS;
        double relativePhase = phase_wrap01(absolutePhase - startPhase);
        double firstPosition = relativePhase * step;
        double teeth[PHASE_MAX_TEETH];
        int toothCount = 0;
        for (double position = firstPosition;
             position < frameCount - 1 && toothCount < PHASE_MAX_TEETH;
             position += step) {
            int index = (int)floor(position);
            double fraction = position - index;
            double energy = 0.0;
            int validBands = 0;
            for (int band = 0; band < bands; ++band) {
                double a = frames[index * bands + band];
                double b = frames[(index + 1) * bands + band];
                double value = a + (b - a) * fraction;
                if (isfinite(value)) {
                    energy += value;
                    ++validBands;
                }
            }
            if (validBands) teeth[toothCount++] = energy / validBands;
        }
        if (toothCount > maximumTeeth) maximumTeeth = toothCount;
        for (int mode = 0; mode < PHASE_SCORE_MODE_COUNT; ++mode)
            out->score[mode][absoluteBin] = phase_aggregate(teeth, toothCount, (PhaseScoreMode)mode);
    }
    out->teeth = maximumTeeth;
    for (int mode = 0; mode < PHASE_SCORE_MODE_COUNT; ++mode)
        phase_finalize_evidence_mode(out, (PhaseScoreMode)mode);
    return maximumTeeth > 0;
}

static inline double phase_time_for_nominal_phase(double phase, double bpm, double nearTime) {
    double period = 60.0 / bpm;
    double origin = phase * period;
    return origin + round((nearTime - origin) / period) * period;
}

static inline double phase_track_predicted_family(const PhaseTrack *track, double time) {
    double nearest = track->anchorTime + round((time - track->anchorTime) / (track->period * 0.5)) *
                                             (track->period * 0.5);
    return phase_wrap_half(nearest * track->bpm / 60.0);
}

static inline int phase_best_local_family(const PhaseEvidence *evidence, PhaseScoreMode mode,
                                          int center, int radius) {
    int best = center;
    for (int offset = -radius; offset <= radius; ++offset) {
        int bin = (center + offset + PHASE_FAMILY_BINS) % PHASE_FAMILY_BINS;
        if (evidence->folded[mode][bin] > evidence->folded[mode][best]) best = bin;
    }
    return best;
}

static inline void phase_tracker_reset(PhaseTracker *tracker) {
    if (tracker) memset(tracker, 0, sizeof(*tracker));
}

static inline void phase_tracker_toggle_side(PhaseTracker *tracker) {
    if (!tracker || !tracker->active.have) return;
    tracker->active.side ^= 1;
    tracker->active.anchorTime += tracker->active.period * 0.5;
}

static inline void phase_tracker_lose_active(PhaseTracker *tracker, double time) {
    if (!tracker || !tracker->active.have) return;
    tracker->previous = tracker->active;
    tracker->previousSavedTime = time;
    tracker->havePrevious = 1;
    memset(&tracker->active, 0, sizeof(tracker->active));
}

// Side is a coordinate within the folded family, not a persistent 0/1
// identity. Preserve the occurrence nearest the existing full-cycle anchor.
static inline int phase_side_nearest_anchor(int family, double bpm, double anchor) {
    double phase = (double)family / PHASE_GRAPH_BINS;
    double a = phase_time_for_nominal_phase(phase, bpm, anchor);
    double b = phase_time_for_nominal_phase(phase + 0.5, bpm, anchor);
    return fabs(b - anchor) < fabs(a - anchor);
}

static inline void phase_track_acquire(PhaseTrack *track, const PhaseEvidence *evidence,
                                       PhaseScoreMode mode, double bpm, double time, int family,
                                       int preserveSide) {
    int side = preserveSide >= 0 ? preserveSide : evidence->bestSide[mode];
    double phase = ((double)family + side * PHASE_FAMILY_BINS) / PHASE_GRAPH_BINS;
    memset(track, 0, sizeof(*track));
    track->have = 1;
    track->bpm = bpm;
    track->period = 60.0 / bpm;
    track->anchorTime = phase_time_for_nominal_phase(phase, bpm, time);
    track->lastUpdateTime = time;
    track->familyBin = family;
    track->side = side;
    track->confidence = evidence->familyProminence[mode];
}

static inline PhaseDecision phase_tracker_update(PhaseTracker *tracker,
                                                 const PhaseEvidence *evidence,
                                                 PhaseScoreMode mode, double bpm, double time,
                                                 int bpmCertain, int familyLocked) {
    PhaseDecision decision = {0};
    if (!tracker || !evidence || mode < 0 || mode >= PHASE_SCORE_MODE_COUNT ||
        !isfinite(bpm) || bpm <= 0.0 || !isfinite(time)) return decision;
    if (tracker->havePrevious && time - tracker->previousSavedTime > PHASE_PREVIOUS_EXPIRY_SECONDS) {
        memset(&tracker->previous, 0, sizeof(tracker->previous));
        tracker->havePrevious = 0;
    }
    if (tracker->active.have && fabs(tracker->active.bpm - bpm) > PHASE_MAX_BPM_DIFF)
        phase_tracker_lose_active(tracker, time);
    if (!bpmCertain) {
        if (tracker->active.have) {
            tracker->active.filteredErrorSeconds = 0.0;
            tracker->active.lastUpdateTime = time;
            decision.valid = 1;
            decision.selectedSide = tracker->active.side;
            decision.familyBin = tracker->active.familyBin;
            decision.flashAnchorTime = tracker->active.anchorTime;
            decision.preciseBpm = bpm;
        }
        return decision;
    }
    if (!tracker->active.have) {
        int restored = 0;
        if (tracker->havePrevious && fabs(tracker->previous.bpm - bpm) <= PHASE_MAX_BPM_DIFF) {
            double predicted = phase_track_predicted_family(&tracker->previous, time);
            int predictedBin = (int)lround(predicted * PHASE_GRAPH_BINS) % PHASE_FAMILY_BINS;
            int bestLocal = phase_best_local_family(evidence, mode, predictedBin, PHASE_LOCAL_RADIUS_BINS);
            double errorCycles = phase_circular_distance(bestLocal, predictedBin, PHASE_FAMILY_BINS) /
                                 (double)PHASE_GRAPH_BINS;
            if (errorCycles <= PHASE_RESTORE_MAX_CYCLE_ERROR) {
                tracker->active = tracker->previous;
                tracker->havePrevious = 0;
                restored = 1;
            }
        }
        if (!restored)
            phase_track_acquire(&tracker->active, evidence, mode, bpm, time,
                                evidence->bestFamily[mode], -1);
        decision.acquired = !restored;
        decision.restored = restored;
    }
    PhaseTrack *track = &tracker->active;
    double elapsed = fmax(1e-3, time - track->lastUpdateTime);
    double predictedFamily = phase_track_predicted_family(track, time);
    int predictedBin = (int)lround(predictedFamily * PHASE_GRAPH_BINS) % PHASE_FAMILY_BINS;
    int local = phase_best_local_family(evidence, mode, predictedBin, PHASE_LOCAL_RADIUS_BINS);
    int global = evidence->bestFamily[mode];
    double localScore = evidence->folded[mode][local];
    double globalScore = evidence->folded[mode][global];
    int distant = phase_circular_distance(global, local, PHASE_FAMILY_BINS) > PHASE_LOCAL_RADIUS_BINS;
    if (bpmCertain && !familyLocked && distant &&
        globalScore >= localScore * PHASE_FAMILY_SWITCH_RATIO) {
        if (track->challengerBin == global) track->challengerSeconds += elapsed;
        else { track->challengerBin = global; track->challengerSeconds = fmin(elapsed, BPM_UPDATE_SECONDS); }
    } else {
        track->challengerSeconds = fmax(0.0, track->challengerSeconds - elapsed);
        if (track->challengerSeconds <= 0.0) track->challengerBin = 0;
    }
    if (bpmCertain && !familyLocked &&
        track->challengerSeconds >= PHASE_FAMILY_SWITCH_SECONDS) {
        local = track->challengerBin;
        int side = phase_side_nearest_anchor(local, bpm, track->anchorTime);
        phase_track_acquire(track, evidence, mode, bpm, time, local, side);
        decision.familySwitched = 1;
        elapsed = BPM_UPDATE_SECONDS;
    }
    double familyPhase = (double)local / PHASE_GRAPH_BINS;
    double memberA = phase_time_for_nominal_phase(familyPhase, bpm, time);
    double memberB = phase_time_for_nominal_phase(familyPhase + 0.5, bpm, time);
    double predictedOccurrence = track->anchorTime +
        round((time - track->anchorTime) / (track->period * 0.5)) * (track->period * 0.5);
    double observed = fabs(memberA - predictedOccurrence) <= fabs(memberB - predictedOccurrence) ? memberA : memberB;
    double rawError = phase_wrap_error(observed - predictedOccurrence, track->period * 0.5);
    double deadband = (60.0 / bpm) * PHASE_ERROR_DEADBAND_BINS / PHASE_GRAPH_BINS;
    double filterAlpha = 1.0 - exp(-elapsed / PHASE_ERROR_FILTER_SECONDS);
    if (fabs(rawError) <= deadband || track->filteredErrorSeconds * rawError < 0.0)
        track->filteredErrorSeconds = 0.0;
    else
        track->filteredErrorSeconds += filterAlpha *
            (rawError - track->filteredErrorSeconds);
    double error = track->filteredErrorSeconds;
    double cadenceScale = phase_clamp(elapsed / BPM_UPDATE_SECONDS, 0.0, 1.0);
    double positionStep = PHASE_MAX_POSITION_STEP_SECONDS * cadenceScale;
    double correction = phase_clamp(error * PHASE_POSITION_GAIN,
                                    -positionStep, positionStep);
    correction = phase_clamp(correction, -fabs(rawError), fabs(rawError));
    double prominence = evidence->familyProminence[mode];
    int strong = isfinite(prominence) && prominence >= PHASE_MIN_TRACK_PROMINENCE &&
                 localScore > 1e-12;
    if (strong && error != 0.0) {
        track->anchorTime += correction;
        decision.correctionApplied = 1;
    } else {
        correction = 0.0;
    }
    track->bpm = bpm;
    track->familyBin = local;
    track->side = phase_side_nearest_anchor(local, bpm, track->anchorTime);
    track->period = 60.0 / bpm;
    track->lastUpdateTime = time;
    track->lastErrorSeconds = rawError;
    track->confidence = prominence;
    decision.valid = 1;
    decision.selectedSide = track->side;
    decision.familyBin = local;
    decision.rawBestBin = global + evidence->bestSide[mode] * PHASE_FAMILY_BINS;
    decision.flashAnchorTime = track->anchorTime;
    decision.preciseBpm = 60.0 / track->period;
    decision.phaseErrorSeconds = error;
    decision.correctionSeconds = correction;
    decision.familyScore = localScore;
    decision.familyProminence = prominence;
    decision.challengerReadiness = phase_clamp(track->challengerSeconds /
                                               PHASE_FAMILY_SWITCH_SECONDS, 0.0, 1.0);
    return decision;
}

#endif
