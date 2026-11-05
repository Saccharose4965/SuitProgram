#ifndef SUIT_BPM_CORE_H
#define SUIT_BPM_CORE_H

// Shared live/offline mathematics. No UI, microphone, clock or dataset dependencies.
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef BPM_UPDATE_EVERY_FRAMES
#define BPM_UPDATE_EVERY_FRAMES 88 // ~2.04 s: four times less frequent than reference
#endif
#define BPM_WORK_SPREAD_FRACTION 0.75
#define SAMPLE_RATE 44100
#define FFT_SIZE 1024
#define RECORD_SECONDS 4.0
#define COMB_BANDS 32       // still direct spectral-band comb, just fewer bands
#define BPM_REFERENCE_HOP_FRAMES 22
#define BPM_UPDATE_SECONDS (BPM_UPDATE_EVERY_FRAMES * (double)FFT_SIZE / SAMPLE_RATE)
#define BPM_REFERENCE_SECONDS (BPM_REFERENCE_HOP_FRAMES * (double)FFT_SIZE / SAMPLE_RATE)
#define BPM_MIN 80
#define BPM_MAX 159
#define BPM_COUNT (BPM_MAX - BPM_MIN + 1)
#define MAX_PHASE_EVAL 256
#define PHASE_SAMPLES_PER_FRAME 8
#define MIN_INT_FREQ_HZ 100.0
#define FAST_COMB_NEAREST_SAMPLE 0  // 1 = nearest-frame comb teeth; 0 = linear interpolation
#define IMPULSE_UPPER_START_BAND 16
#define IMPULSE_UPPER_RISE_THRESHOLD 1.0
#define IMPULSE_BAND_RISE_THRESHOLD 0.75
#define IMPULSE_MIN_RISING_BANDS 8
#define TREBLE_WEIGHT_START_BAND 16
#define TREBLE_WEIGHT_END_BAND 24
#define TREBLE_AMPLITUDE_SCALE 1.4142135623730951
#define STABILITY_FRAMES 8 // reference-rate window, not the number of slow results
// Frozen contestant bank: time-weighted score memory, independent decaying support.
#define STABILITY_WINDOW_SECONDS (STABILITY_FRAMES * BPM_REFERENCE_SECONDS)
#define SCORE_HISTORY_CAPACITY 32
#define STABILITY_VARIANCE_THRESHOLD 0.014
#define MIN_CANDIDATE_CONSISTENCY 0.95
#define SUPPORT_HALF_LIFE_SECONDS 6.0
#define ACQUISITION_SUPPORT_SECONDS 1.0
#define CHALLENGER_SUPPORT_SECONDS 1.0
#define HELD_RELEASE_SECONDS 0.5
#define PHASE_BPM_SOFT_LOCK_SUPPORT_MULTIPLIER 2.0
#define PHASE_BPM_SOFT_LOCK_RELEASE_MULTIPLIER 0.5
#define VARIANCE_SHOCK_THRESHOLD 0.004
#define SHOCK_COOLDOWN_SECONDS 3.0
#define MIN_RHYTHMIC_PAIR_SCORE 0.05
// A flat/no-evidence normalized spectrum has adjacent-pair prominence 2/BPM_COUNT.
#define AUTO_STOP_LOST_SECONDS 40.0
#define AUTO_STOP_MIN_HELD_RATIO 0.60
#define DECISION_CONFIDENCE_DISPLAY_MAX 1.1

typedef struct {
    int have;
    double previousRaw[COMB_BANDS];
    double lastClean[COMB_BANDS];
} ImpulseFilterState;

static inline int filter_and_weight_band_frame(double *frame, ImpulseFilterState *state) {
    int suppressed = 0;
    if (!state->have) {
        memcpy(state->previousRaw, frame, sizeof(state->previousRaw));
        memcpy(state->lastClean, frame, sizeof(state->lastClean));
        state->have = 1;
    } else {
        double raw[COMB_BANDS];
        double upperRise = 0.0;
        int risingBands = 0;
        memcpy(raw, frame, sizeof(raw));
        for (int band = 0; band < COMB_BANDS; ++band) {
            double rise = fmax(0.0, raw[band] - state->previousRaw[band]);
            if (band >= IMPULSE_UPPER_START_BAND) upperRise += rise;
            if (rise >= IMPULSE_BAND_RISE_THRESHOLD) risingBands++;
        }
        upperRise /= (double)(COMB_BANDS - IMPULSE_UPPER_START_BAND);
        memcpy(state->previousRaw, raw, sizeof(state->previousRaw));
        if (upperRise >= IMPULSE_UPPER_RISE_THRESHOLD &&
            risingBands >= IMPULSE_MIN_RISING_BANDS) {
            memcpy(frame, state->lastClean, sizeof(state->lastClean));
            suppressed = 1;
        } else {
            memcpy(state->lastClean, raw, sizeof(state->lastClean));
        }
    }
    for (int band = TREBLE_WEIGHT_START_BAND;
         band < TREBLE_WEIGHT_END_BAND;
         ++band) {
        frame[band] *= TREBLE_AMPLITUDE_SCALE;
    }
    return suppressed;
}

static inline void fft(double *real, double *imag, int n, int inverse) {
    int i, j;

    j = 0;
    for (i = 0; i < n; ++i) {
        if (i < j) {
            double tr = real[i], ti = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = tr;
            imag[j] = ti;
        }

        int m = n >> 1;
        while (m > 0 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    double angSign = inverse ? 2.0 * M_PI : -2.0 * M_PI;

    for (int len = 2; len <= n; len <<= 1) {
        double ang = angSign / (double)len;
        double wlen_r = cos(ang);
        double wlen_i = sin(ang);

        for (i = 0; i < n; i += len) {
            double wr = 1.0, wi = 0.0;

            for (j = 0; j < len / 2; ++j) {
                int u = i + j;
                int v = i + j + len / 2;

                double tr = wr * real[v] - wi * imag[v];
                double ti = wr * imag[v] + wi * real[v];

                real[v] = real[u] - tr;
                imag[v] = imag[u] - ti;
                real[u] += tr;
                imag[u] += ti;

                double nwr = wr * wlen_r - wi * wlen_i;
                double nwi = wr * wlen_i + wi * wlen_r;
                wr = nwr;
                wi = nwi;
            }
        }
    }

    if (inverse) {
        for (i = 0; i < n; ++i) {
            real[i] /= (double)n;
            imag[i] /= (double)n;
        }
    }
}

static inline void compute_raw_comb_scores_range(const double *specFrames,
                                    int numFrames,
                                    int bands,
                                    double frameRate,
                                    double *bpmScoreOut, int first, int end) {

    if (!specFrames || numFrames <= 2 || bands <= 0 || frameRate <= 0.0) {
        for (int i = first; i < end; ++i) {
            bpmScoreOut[i] = 0.0;
        }
        return;
    }

    // Lighter direct-comb score. This deliberately keeps the original idea:
    // score every BPM directly against the spectral-band history, not an onset
    // envelope. The savings come from fewer phase probes, fewer bands, nearest
    // frame sampling by default, and a one-pass sum/sumsq variance calculation.
    for (int i = first; i < end; ++i) {
        int bpm = BPM_MIN + i;
        double step = frameRate * 60.0 / (double)bpm;

        int phaseCount = (int)ceil(step * (double)PHASE_SAMPLES_PER_FRAME);
        if (phaseCount < 1) phaseCount = 1;
        if (phaseCount > MAX_PHASE_EVAL) phaseCount = MAX_PHASE_EVAL;

        double totalVariance = 0.0;
        int validEvalCount = 0;

        for (int phaseIdx = 0; phaseIdx < phaseCount; ++phaseIdx) {
            double phase = step * ((double)phaseIdx / (double)phaseCount);

            for (int b = 0; b < bands; ++b) {
                int n = 0;
                double sum = 0.0;
                double sumSq = 0.0;

                for (double pos = phase;
                     pos < (double)(numFrames - 1);
                     pos += step) {
#if FAST_COMB_NEAREST_SAMPLE
                    int ii = (int)lround(pos);
                    if (ii < 0) ii = 0;
                    if (ii >= numFrames) ii = numFrames - 1;
                    double x = specFrames[ii * bands + b];
#else
                    int i0 = (int)floor(pos);
                    double t = pos - (double)i0;
                    double a = specFrames[i0 * bands + b];
                    double x = a + (specFrames[(i0 + 1) * bands + b] - a) * t;
#endif
                    n++;
                    sum += x;
                    sumSq += x * x;
                }

                if (n >= 2) {
                    double var = (sumSq - (sum * sum) / (double)n) / (double)(n - 1);
                    if (var < 0.0 && var > -1e-12) var = 0.0;
                    totalVariance += var;
                    validEvalCount++;
                }
            }
        }

        bpmScoreOut[i] = (validEvalCount > 0)
            ? totalVariance / (double)validEvalCount
            : 0.0;
    }
}


static inline void compute_raw_comb_scores(const double *frames, int count, int bands,
                                    double rate, double *scores) {
    compute_raw_comb_scores_range(frames, count, bands, rate, scores, 0, BPM_COUNT);
}

static inline void raw_scores_to_positive(const double *rawScore, int bpmCount, double *positiveOut) {
    double maxV = 0.0;

    for (int i = 0; i < bpmCount; ++i) {
        double v = rawScore[i];
        if (isfinite(v) && v > maxV) {
            maxV = v;
        }
    }

    for (int i = 0; i < bpmCount; ++i) {
        double v = rawScore[i];
        if (!isfinite(v) || v < 0.0) v = maxV;
        positiveOut[i] = maxV - v;
        if (positiveOut[i] < 0.0) positiveOut[i] = 0.0;
    }
}

static inline void bpm_baseline_correct(const double *bpmSpec, int bpmCount, double *bcOut) {
    int midIdx = bpmCount / 2;

    double lowSum = 0.0;
    for (int i = 0; i < midIdx; ++i) lowSum += bpmSpec[i];

    double highSum = 0.0;
    for (int i = midIdx; i < bpmCount; ++i) highSum += bpmSpec[i];

    double lowAvg = (midIdx > 0) ? (lowSum / (double)midIdx) : 0.0;
    double highAvg = (bpmCount - midIdx > 0) ? (highSum / (double)(bpmCount - midIdx)) : 0.0;

    int x1 = midIdx / 2;
    int x2 = midIdx + (bpmCount - midIdx) / 2;
    if (x2 == x1) {
        x1 = 0;
        x2 = bpmCount - 1;
    }

    double slope = (x2 != x1) ? ((highAvg - lowAvg) / (double)(x2 - x1)) : 0.0;
    double intercept = lowAvg - slope * (double)x1;

    for (int i = 0; i < bpmCount; ++i) {
        double base = intercept + slope * (double)i;
        double v = bpmSpec[i] - base;
        if (v < 0.0) v = 0.0;
        bcOut[i] = v;
    }
}

static inline void normalize_scores_by_sum(const double *score, int count, double *normOut) {
    double sum = 0.0;

    if (!score || !normOut || count <= 0) return;

    for (int i = 0; i < count; ++i) {
        double v = score[i];
        if (isfinite(v) && v > 0.0) {
            sum += v;
        }
    }

    if (sum <= 1e-12) {
        double u = 1.0 / (double)count;
        for (int i = 0; i < count; ++i) {
            normOut[i] = u;
        }
        return;
    }

    for (int i = 0; i < count; ++i) {
        double v = score[i];
        normOut[i] = (isfinite(v) && v > 0.0) ? (v / sum) : 0.0;
    }
}


typedef struct {
    double scores[SCORE_HISTORY_CAPACITY][BPM_COUNT];
    double start[SCORE_HISTORY_CAPACITY], end[SCORE_HISTORY_CAPACITY];
    double lastTime;
    int count, write;
} ScoreHistory;

// Raw 32-band frame, before causal impulse filtering and treble weighting.
static inline void magnitudes_to_bands(const double *mags, double *bands) {
    const int halfBins = FFT_SIZE / 2;
    int minBin = (int)ceil(MIN_INT_FREQ_HZ * (double)FFT_SIZE / SAMPLE_RATE);
    if (minBin < 0) minBin = 0;
    if (minBin >= halfBins) minBin = halfBins - 1;
    int usableBins = halfBins - minBin;
    if (usableBins < 1) usableBins = 1;
    for (int b = 0; b < COMB_BANDS; ++b) {
        int k0 = minBin + (b * usableBins) / COMB_BANDS;
        int k1 = minBin + ((b + 1) * usableBins) / COMB_BANDS;
        if (k1 <= k0) k1 = k0 + 1;
        if (k1 > halfBins) k1 = halfBins;
        double sum = 0.0;
        for (int k = k0; k < k1; ++k) sum += log(1.0 + 10.0 * mags[k]);
        bands[b] = sum / (double)(k1 - k0);
    }
}

static inline int strongest_pair(const double *scores) {
    int best = 0;
    for (int i = 1; i < BPM_COUNT - 1; ++i)
        if (scores[i] + scores[i + 1] > scores[best] + scores[best + 1]) best = i;
    return best;
}

static inline void score_history_push(ScoreHistory *h, const double *scores,
                               double time, double nominalStep) {
    double duration = h->count ? time - h->lastTime : nominalStep;
    if (!isfinite(duration) || duration <= 0 || nominalStep <= 0) return;
    // Allow ordinary scheduling jitter, but never treat a long worker stall
    // as several seconds of continuously observed evidence.
    duration = fmin(duration, nominalStep + (double)FFT_SIZE / SAMPLE_RATE);
    double cutoff = time - STABILITY_WINDOW_SECONDS;
    while (h->count && h->end[(h->write - h->count + SCORE_HISTORY_CAPACITY) % SCORE_HISTORY_CAPACITY] <= cutoff + 1e-9)
        --h->count;
    memcpy(h->scores[h->write], scores, sizeof(h->scores[0]));
    h->start[h->write] = time - duration;
    h->end[h->write] = time;
    h->write = (h->write + 1) % SCORE_HISTORY_CAPACITY;
    if (h->count < SCORE_HISTORY_CAPACITY) ++h->count;
    h->lastTime = time;
}

static inline double score_history_stats(const ScoreHistory *h, double *mean,
                                  double *variance, double *consistency) {
    double weight[SCORE_HISTORY_CAPACITY] = {0}, coverage = 0;
    memset(mean, 0, sizeof(double) * BPM_COUNT);
    *variance = *consistency = 0;
    double cutoff = h->lastTime - STABILITY_WINDOW_SECONDS;
    for (int i = 0; i < h->count; ++i) {
        int slot = (h->write - 1 - i + SCORE_HISTORY_CAPACITY) % SCORE_HISTORY_CAPACITY;
        weight[slot] = fmax(0.0, h->end[slot] - fmax(cutoff, h->start[slot]));
        coverage += weight[slot];
        for (int b = 0; b < BPM_COUNT; ++b) mean[b] += weight[slot] * h->scores[slot][b];
    }
    if (coverage <= 0) return 0;
    for (int b = 0; b < BPM_COUNT; ++b) mean[b] /= coverage;
    int winner = strongest_pair(mean);
    for (int i = 0; i < h->count; ++i) {
        int slot = (h->write - 1 - i + SCORE_HISTORY_CAPACITY) % SCORE_HISTORY_CAPACITY;
        double w = weight[slot] / coverage;
        for (int b = 0; b < BPM_COUNT; ++b) {
            double delta = h->scores[slot][b] - mean[b];
            *variance += w * delta * delta;
        }
        if (abs(strongest_pair(h->scores[slot]) - winner) <= 2) *consistency += w;
    }
    return coverage;
}

// Exact elapsed-time equivalent of adding one reference-hop credit each hop.
static inline double support_credit(double elapsed, int sustained) {
    double creditTime = sustained && elapsed <= 1.5 * BPM_UPDATE_SECONDS
        ? elapsed : fmin(elapsed, BPM_REFERENCE_SECONDS);
    double lambda = log(2.0) / SUPPORT_HALF_LIFE_SECONDS;
    return BPM_REFERENCE_SECONDS * (-expm1(-lambda * creditTime)) /
                                  (-expm1(-lambda * BPM_REFERENCE_SECONDS));
}

typedef struct {
    int have;
    double bpm;
    int pairIdx;
    double support[BPM_COUNT];
    double previousEvalTime;
    double previousVariance;
    double shockCooldown;
    int havePreviousTime;
    int havePreviousVariance;
    int previousTrusted, previousCandidate;
} BpmHoldState;


static inline void bpm_hold_reset(BpmHoldState *state) {
    if (state) memset(state, 0, sizeof(*state));
}


static inline void bpm_hold_force(BpmHoldState *state, double bpm, int pairIdx) {
    bpm_hold_reset(state);
    state->have = 1;
    state->bpm = bpm;
    state->pairIdx = pairIdx;
}


static inline double bpm_neighborhood_support(const BpmHoldState *state, double bpm) {
    int center = (int)lround(bpm) - BPM_MIN;
    double support = 0.0;
    for (int index = center - 1; index <= center + 1; ++index) {
        if (index >= 0 && index < BPM_COUNT) support += state->support[index];
    }
    return support;
}


static inline int update_final_bpm_from_accum(double accumBpm,
                                       int accumPairIdx,
                                       double rhythmicPairScore,
                                       double candidateConsistency,
                                       double stabilityVariance,
                                       double evaluationTime,
                                       int transitionsEnabled,
                                       int softLocked,
                                       BpmHoldState *state,
                                       double *finalBpmOut,
                                       int *finalPairIdxOut,
                                       double *decisionConfidenceOut,
                                       double *evidenceQualityOut,
                                       int *candidateTrustedOut) {
    double finalBpm = accumBpm;
    int finalPairIdx = accumPairIdx;
    int switched = 0;
    double decisionConfidence = 0.0;
    if (!state) {
        if (decisionConfidenceOut) *decisionConfidenceOut = 0.0;
        if (evidenceQualityOut) *evidenceQualityOut = 0.0;
        if (candidateTrustedOut) *candidateTrustedOut = 0;
        return 0;
    }

    double elapsed = state->havePreviousTime
        ? fmax(1e-3, evaluationTime - state->previousEvalTime)
        : BPM_REFERENCE_SECONDS;
    state->previousEvalTime = evaluationTime;
    state->havePreviousTime = 1;
    double decay = exp(log(0.5) * elapsed / SUPPORT_HALF_LIFE_SECONDS);
    for (int index = 0; index < BPM_COUNT; ++index) state->support[index] *= decay;

    int varianceShock = state->havePreviousVariance &&
        fabs(stabilityVariance - state->previousVariance) *
        BPM_REFERENCE_SECONDS / fmax(BPM_REFERENCE_SECONDS, elapsed) >= VARIANCE_SHOCK_THRESHOLD;
    state->previousVariance = stabilityVariance;
    state->havePreviousVariance = 1;
    state->shockCooldown = varianceShock
        ? SHOCK_COOLDOWN_SECONDS
        : fmax(0.0, state->shockCooldown - elapsed);

    int evidenceTrusted = rhythmicPairScore >= MIN_RHYTHMIC_PAIR_SCORE &&
        stabilityVariance <= STABILITY_VARIANCE_THRESHOLD &&
        candidateConsistency >= MIN_CANDIDATE_CONSISTENCY;
    int trusted = evidenceTrusted && state->shockCooldown <= 0.0;
    double rhythmReadiness = rhythmicPairScore / MIN_RHYTHMIC_PAIR_SCORE;
    double stabilityReadiness = STABILITY_VARIANCE_THRESHOLD /
        fmax(stabilityVariance, 1e-12);
    double consistencyReadiness = candidateConsistency /
        MIN_CANDIDATE_CONSISTENCY;
    double evidenceQuality = fmin(
        rhythmReadiness,
        fmin(stabilityReadiness, consistencyReadiness)
    );
    double qualityReadiness = state->shockCooldown <= 0.0
        ? evidenceQuality
        : 0.0;
    int candidateBin = (int)lround(accumBpm) - BPM_MIN;
    if (trusted && candidateBin >= 0 && candidateBin < BPM_COUNT) {
        state->support[candidateBin] += support_credit(elapsed,
            state->previousTrusted && state->previousCandidate == candidateBin);
    }
    state->previousTrusted = trusted;
    state->previousCandidate = candidateBin;

    double challengerSupport = bpm_neighborhood_support(state, accumBpm);
    if (!state->have) {
        decisionConfidence = fmin(
            qualityReadiness,
            challengerSupport / ACQUISITION_SUPPORT_SECONDS
        );
        if (transitionsEnabled &&
            trusted &&
            challengerSupport >= ACQUISITION_SUPPORT_SECONDS) {
            state->have = 1;
            state->bpm = accumBpm;
            state->pairIdx = accumPairIdx;
        }
    } else {
        double heldSupport = bpm_neighborhood_support(state, state->bpm);
        double bpmDistance = fabs(accumBpm - state->bpm);
        if (bpmDistance >= 0.5 && bpmDistance <= 1.0) {
            // Adjacent alternatives must earn separate persistence: shared
            // neighbourhoods would credit the challenger to the held bin too.
            challengerSupport = state->support[candidateBin];
            heldSupport = state->support[(int)lround(state->bpm) - BPM_MIN];
        }
        double requiredSupport = CHALLENGER_SUPPORT_SECONDS *
            (softLocked ? PHASE_BPM_SOFT_LOCK_SUPPORT_MULTIPLIER : 1.0);
        double releaseLimit = HELD_RELEASE_SECONDS *
            (softLocked ? PHASE_BPM_SOFT_LOCK_RELEASE_MULTIPLIER : 1.0);

        if (bpmDistance < 0.5) {
            // Shock is a transition guard, not a reason to invalidate a held BPM
            // that the current clean evidence still agrees with. Acquisition has
            // already paid the persistence requirement.
            decisionConfidence = evidenceQuality;
        } else if (bpmDistance >= 0.5) {
            double releaseReadiness = releaseLimit /
                fmax(heldSupport, 1e-12);
            decisionConfidence = fmin(
                qualityReadiness,
                fmin(challengerSupport / requiredSupport, releaseReadiness)
            );
        }

        if (transitionsEnabled &&
            bpmDistance >= 0.5 &&
            trusted &&
            challengerSupport >= requiredSupport &&
            heldSupport <= releaseLimit) {
            state->bpm = accumBpm;
            state->pairIdx = accumPairIdx;
            switched = 1;
        }
    }

    if (state->have) {
        finalBpm = state->bpm;
        finalPairIdx = state->pairIdx;
    }

    if (finalBpmOut) *finalBpmOut = finalBpm;
    if (finalPairIdxOut) *finalPairIdxOut = finalPairIdx;
    if (!isfinite(decisionConfidence) || decisionConfidence < 0.0) {
        decisionConfidence = 0.0;
    }
    if (decisionConfidence > DECISION_CONFIDENCE_DISPLAY_MAX) {
        decisionConfidence = DECISION_CONFIDENCE_DISPLAY_MAX;
    }
    if (decisionConfidenceOut) *decisionConfidenceOut = decisionConfidence;
    if (!isfinite(evidenceQuality) || evidenceQuality < 0.0) {
        evidenceQuality = 0.0;
    }
    if (evidenceQuality > DECISION_CONFIDENCE_DISPLAY_MAX) {
        evidenceQuality = DECISION_CONFIDENCE_DISPLAY_MAX;
    }
    if (evidenceQualityOut) *evidenceQualityOut = evidenceQuality;
    if (candidateTrustedOut) {
        int agreesWithHeld = state->have && fabs(accumBpm - state->bpm) < 0.5;
        *candidateTrustedOut = agreesWithHeld ? evidenceTrusted : trusted;
    }
    return switched;
}

// Compare the best adjacent pair containing the held BPM with the global
// winning pair. This tolerates one-bin jitter without following a challenger.
static inline double held_score_ratio(const double *scores, double heldBpm) {
    if (!isfinite(heldBpm) || heldBpm < BPM_MIN || heldBpm > BPM_MAX) return 0.0;
    int held = (int)lround(heldBpm) - BPM_MIN;
    double maximum = 0.0, support = 0.0;
    for (int i = 0; i < BPM_COUNT - 1; ++i) {
        double pair = scores[i] + scores[i + 1];
        if (!isfinite(pair) || pair < 0.0) return 0.0;
        maximum = fmax(maximum, pair);
        if (i == held || i + 1 == held) support = fmax(support, pair);
    }
    return maximum > 0.0 ? support / maximum : 0.0;
}

// Loss is consecutive time below the score ratio threshold. A supporting
// result immediately resets it, including after a delayed computation.
static inline double update_lost_pulse(double lost, double elapsed, int forced,
                                int haveBpm, double heldRatio, int newLock) {
    if (newLock || forced || !haveBpm) return 0.0;
    if (isfinite(heldRatio) && heldRatio >= AUTO_STOP_MIN_HELD_RATIO) return 0.0;
    if (!isfinite(elapsed) || elapsed <= 0.0) return lost;
    return lost + elapsed;
}

#endif
