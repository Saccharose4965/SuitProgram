// Live PC microphone harness for the benchmarked BPM detector and beat clock.
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <alsa/asoundlib.h>
#include <SDL2/SDL.h>
#include "include/bpm_core.h"
#include "include/phase_tracker.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CHANNELS 1


#define WIN_WIDTH 2560
#define WIN_HEIGHT 1522

#define SPEC_SCROLL_SCALE 50.0
#define SPEC_MIN_LEVEL 0.05

#define BPM_WORK_BATCH 8
#define PHASE_UPDATE_EVERY_FRAMES 8 // ~5.4 Hz; independent of the slow BPM sweep

#define TARGET_BPM_MIN 80
#define TARGET_BPM_MAX 159



#define SPEC_WIDTH 320
#define SPEC_HEIGHT (FFT_SIZE / 2)

#define BPM_SCALE_SMOOTH_UP   0.25
#define BPM_SCALE_SMOOTH_DOWN 0.05
#define BPM_HEADROOM          1.10

// Beat-clock display layer, driven by the emitted clock boundary.
#define BLINK_DECAY                10.0
#define BEAT_PHASE_STEER_GAIN       0.30
#define BEAT_PHASE_STEER_MAX_RATIO  0.006


static double g_bpm_display_scale = 1.0;
static double g_bpm_display_floor = 1e-9;
static double g_last_bpm_draw_time = 0.0;

static Uint32 *specPixels = NULL;
static SDL_Texture *specTex = NULL;

// One confidence sample per spectrogram column keeps both timelines aligned.
#define CONFIDENCE_HISTORY_LEN SPEC_WIDTH
#define DECISION_CONFIDENCE_THRESHOLD 1.0
static double g_confidenceHistory[CONFIDENCE_HISTORY_LEN];
static double g_evidenceHistory[CONFIDENCE_HISTORY_LEN];
static int g_confidenceHistoryWrite = 0;
static int g_confidenceHistoryCount = 0;

typedef struct {
    SDL_Thread *thread;
    SDL_mutex *mutex;
    SDL_cond *cond;
    int stop;
    int busy;
    int hasJob;
    int resultReady;
    unsigned epoch, jobEpoch, resultEpoch;
    int jobNumFrames;
    int resultNumFrames;
    double jobFrameRate;
    double resultFrameRate;
    double jobLastFrameTime;
    double resultLastFrameTime;
    double resultWorkMs, resultWallMs;
    double *jobFrames;
    double *jobScores;
    double *resultScores;
    int bpmCount;
} BPMWorker;

static void copy_band_history(const double *bandRing,
                              int maxFrames,
                              int ringWrite,
                              int numFrames,
                              double *bandOrdered) {
    int start = ringWrite - numFrames;

    while (start < 0) {
        start += maxFrames;
    }

    for (int i = 0; i < numFrames; ++i) {
        int src = (start + i) % maxFrames;

        memcpy(&bandOrdered[i * COMB_BANDS],
               &bandRing[src * COMB_BANDS],
               sizeof(double) * (size_t)COMB_BANDS);
    }
}

static int read_exact_frames(snd_pcm_t *handle, int16_t *buf, snd_pcm_uframes_t frames_wanted) {
    snd_pcm_uframes_t done = 0;
    int discontinuity = 0;

    while (done < frames_wanted) {
        snd_pcm_uframes_t to_read = frames_wanted - done;
        int rc = snd_pcm_readi(handle, buf + done, to_read);

        if (rc == -EPIPE) {
            fprintf(stderr, "overrun occurred\n");
            int recovered = snd_pcm_prepare(handle);
            if (recovered < 0) return recovered;
            discontinuity = 1;
            done = 0;
            continue;
        } else if (rc < 0) {
            fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
            return rc;
        } else if (rc == 0) {
            usleep(1000);
            continue;
        }

        done += (snd_pcm_uframes_t)rc;
    }

    return discontinuity;
}

static int setup_alsa_capture(snd_pcm_t **handle) {
    snd_pcm_t *h = NULL;
    snd_pcm_hw_params_t *params;
    int rc;
    unsigned int rate = SAMPLE_RATE;
    int dir = 0;
    snd_pcm_uframes_t frames = FFT_SIZE;

    rc = snd_pcm_open(&h, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
        fprintf(stderr, "unable to open capture device: %s\n", snd_strerror(rc));
        return rc;
    }

    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(h, params);
    snd_pcm_hw_params_set_access(h, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(h, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(h, params, CHANNELS);
    snd_pcm_hw_params_set_rate_near(h, params, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(h, params, &frames, &dir);

    rc = snd_pcm_hw_params(h, params);
    if (rc < 0) {
        fprintf(stderr, "unable to set capture hw parameters: %s\n", snd_strerror(rc));
        snd_pcm_close(h);
        return rc;
    }

    if (rate != SAMPLE_RATE) {
        fprintf(stderr, "Capture requires %d Hz; device selected %u Hz.\n", SAMPLE_RATE, rate);
        snd_pcm_close(h);
        return -EINVAL;
    }
    *handle = h;
    return 0;
}

static void init_scrolling_spectrogram(SDL_Renderer *ren) {
    specPixels = (Uint32 *)malloc(sizeof(Uint32) * SPEC_WIDTH * SPEC_HEIGHT);
    if (!specPixels) {
        fprintf(stderr, "Failed to allocate specPixels\n");
        return;
    }

    for (int i = 0; i < SPEC_WIDTH * SPEC_HEIGHT; ++i) {
        specPixels[i] = 0xFF000000u;
    }

    specTex = SDL_CreateTexture(
        ren,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        SPEC_WIDTH,
        SPEC_HEIGHT
    );

    if (!specTex) {
        fprintf(stderr, "SDL_CreateTexture error: %s\n", SDL_GetError());
    }
}

static void update_scrolling_spectrogram(const double *mags) {
    if (!specPixels || !mags) return;

    const int halfBins = FFT_SIZE / 2;

    for (int y = 0; y < SPEC_HEIGHT; ++y) {
        Uint32 *row = &specPixels[y * SPEC_WIDTH];
        memmove(row, row + 1, (SPEC_WIDTH - 1) * sizeof(Uint32));
    }

    int xNew = SPEC_WIDTH - 1;

    for (int y = 0; y < SPEC_HEIGHT; ++y) {
        int k = (y * halfBins) / SPEC_HEIGHT;
        if (k < 0) k = 0;
        if (k >= halfBins) k = halfBins - 1;

        double norm = mags[k] / SPEC_SCROLL_SCALE;
        if (norm < 0.0) norm = 0.0;
        if (norm > 1.0) norm = 1.0;

        double m;
        if (norm <= SPEC_MIN_LEVEL) {
            m = norm * 0.3 / (SPEC_MIN_LEVEL > 0.0 ? SPEC_MIN_LEVEL : 1.0);
        } else {
            m = 0.3 + (norm - SPEC_MIN_LEVEL) * 0.7 / (1.0 - SPEC_MIN_LEVEL);
        }

        m = log10(1.0 + 9.0 * m);
        if (m < 0.0) m = 0.0;
        if (m > 1.0) m = 1.0;

        Uint8 c = (Uint8)(m * 255.0);
        int yy = SPEC_HEIGHT - 1 - y;
        specPixels[yy * SPEC_WIDTH + xNew] = 0xFF000000u | ((Uint32)c << 16) | ((Uint32)c << 8) | (Uint32)c;
    }
}

// Raw score. Lower is better.
// No phase picking.
// For each BPM, sweep the comb across the whole phase interval [0, step).
// For each phase/band evaluation, compute the exact sample variance with
// Welford's online update while sweeping the comb teeth once, then average
// those variances equally across evaluations. This keeps the full-phase sweep
// while avoiding a second pass over the same interpolated samples.
static int bpm_worker_main(void *userdata) {
    BPMWorker *worker = (BPMWorker *)userdata;

    for (;;) {
        int numFrames = 0;
        double frameRate = 0.0;

        SDL_LockMutex(worker->mutex);

        while (!worker->stop && !worker->hasJob) {
            SDL_CondWait(worker->cond, worker->mutex);
        }

        if (worker->stop) {
            SDL_UnlockMutex(worker->mutex);
            break;
        }

        numFrames = worker->jobNumFrames;
        frameRate = worker->jobFrameRate;
        worker->hasJob = 0;

        SDL_UnlockMutex(worker->mutex);

        Uint64 start = SDL_GetPerformanceCounter();
        double frequency = (double)SDL_GetPerformanceFrequency();
        double workMs = 0;
        int cancelled = 0;
        if (BPM_WORK_SPREAD_FRACTION <= 0) {
            compute_raw_comb_scores(worker->jobFrames, numFrames, COMB_BANDS,
                                    frameRate, worker->jobScores);
            workMs = (SDL_GetPerformanceCounter() - start) * 1000.0 / frequency;
        } else for (int first = 0; first < BPM_COUNT; first += BPM_WORK_BATCH) {
            int end = first + BPM_WORK_BATCH;
            if (end > BPM_COUNT) end = BPM_COUNT;
            Uint64 workStart = SDL_GetPerformanceCounter();
            compute_raw_comb_scores_range(worker->jobFrames, numFrames, COMB_BANDS,
                                          frameRate, worker->jobScores, first, end);
            workMs += (SDL_GetPerformanceCounter() - workStart) * 1000.0 / frequency;
            double deadline = BPM_UPDATE_SECONDS * BPM_WORK_SPREAD_FRACTION * end / BPM_COUNT;
            SDL_LockMutex(worker->mutex);
            while (!worker->stop) {
                double remaining = deadline - (SDL_GetPerformanceCounter() - start) / frequency;
                if (remaining <= 0) break;
                SDL_CondWaitTimeout(worker->cond, worker->mutex, (Uint32)ceil(remaining * 1000));
            }
            cancelled = worker->stop;
            SDL_UnlockMutex(worker->mutex);
            if (cancelled) break;
        }
        if (cancelled) break;

        SDL_LockMutex(worker->mutex);
        memcpy(worker->resultScores,
               worker->jobScores,
               sizeof(double) * (size_t)worker->bpmCount);
        worker->resultNumFrames = worker->jobNumFrames;
        worker->resultFrameRate = worker->jobFrameRate;
        worker->resultLastFrameTime = worker->jobLastFrameTime;
        worker->resultWorkMs = workMs;
        worker->resultWallMs = (SDL_GetPerformanceCounter() - start) * 1000.0 / frequency;
        worker->resultEpoch = worker->jobEpoch;
        worker->resultReady = worker->resultEpoch == worker->epoch;
        worker->busy = 0;
        SDL_UnlockMutex(worker->mutex);
    }

    return 0;
}



typedef struct {
    int initialized;
    double bpmCurrent;
    double bpmTarget;
    double bpmPending;
    int hasPendingBpm;
    double periodCurrent;
    double periodTarget;
    double periodPending;
    double phasePendingAnchor;
    int beatsSincePeriodChange;
    double beatStartTime;
    double nextBeatTime;
    int boundarySeen;
    double lastBoundary, emittedPeriod;
} BeatClock;

typedef struct {
    int phaseValid;
    int phaseLockAllowed;
    int nudgeApplied;
    int phaseSwitched;
    double detectedBeatTime;
    double detectedPhase;
    double phaseErrorSec;
    double nudgeSec;
    double bestPhaseFrames;
    double bestPhaseScore;
    double heldPhaseFrames;
    double heldPhaseScore;
    double candidatePhaseFrames;
    double candidatePhaseScore;
    double candidateBeatTime;
    double acceptedBeatTime;
    double preciseBpm;
    double familyProminence;
    double nextBeatInSec;

    double targetBpm;
    double targetFrameRate;
    double targetLastFrameTime;
    int targetNumFrames;
    int targetPhaseCount;
    double targetPhaseScore[PHASE_GRAPH_BINS];
} BeatDebug;

static void beat_clock_init_from_detected_beat(BeatClock *clk,
                                               double bpm,
                                               double detectedBeatTime,
                                               double now) {
    double period = 60.0 / bpm;
    double start = detectedBeatTime;

    while (start > now) start -= period;
    while (start + period <= now) start += period;

    if (start < now - 1e-9) {
        if (!clk->boundarySeen || start > clk->lastBoundary) clk->lastBoundary = start;
        clk->boundarySeen = 1;
    }
    clk->initialized = 1;
    clk->bpmCurrent = bpm;
    clk->bpmTarget = bpm;
    clk->bpmPending = bpm;
    clk->hasPendingBpm = 0;
    clk->periodCurrent = period;
    clk->periodTarget = period;
    clk->periodPending = period;
    clk->phasePendingAnchor = detectedBeatTime;
    clk->beatsSincePeriodChange = 1000000;
    clk->beatStartTime = start;
    clk->nextBeatTime = start + period;
}

static void beat_clock_schedule_from_detected_beat(BeatClock *clk,
                                                   double bpm,
                                                   double detectedBeatTime) {
    if (!clk || !clk->initialized || !isfinite(bpm) || bpm <= 0.0) return;
    clk->bpmPending = bpm;
    clk->periodPending = 60.0 / bpm;
    clk->phasePendingAnchor = detectedBeatTime;
    clk->hasPendingBpm = 1;
}

static void beat_clock_update(BeatClock *clk, double now) {
    if (!clk->initialized) return;

    int guard = 0;
    while (now >= clk->nextBeatTime && guard++ < 8) {
        double boundary = clk->nextBeatTime;

        if (clk->hasPendingBpm) {
            clk->bpmTarget = clk->bpmPending;
            clk->periodTarget = clk->periodPending;
            clk->beatStartTime = boundary;
            // Preserve this boundary, then approach the corrected phase by
            // gently stretching the following cycle instead of snapping.
            double desiredNext = clk->phasePendingAnchor +
                round((boundary + clk->periodTarget - clk->phasePendingAnchor) /
                      clk->periodTarget) * clk->periodTarget;
            double phaseDelta = desiredNext - (boundary + clk->periodTarget);
            double maximumSteer = clk->periodTarget * BEAT_PHASE_STEER_MAX_RATIO;
            double steer = phase_clamp(phaseDelta * BEAT_PHASE_STEER_GAIN,
                                       -maximumSteer, maximumSteer);
            clk->periodCurrent = clk->periodTarget + steer;
            clk->hasPendingBpm = 0;
            clk->beatsSincePeriodChange = 0;
        } else if (clk->beatsSincePeriodChange < 1000000) {
            clk->beatStartTime = boundary;
            clk->beatsSincePeriodChange++;
            clk->periodCurrent = clk->periodTarget;
        } else {
            clk->beatStartTime = boundary;
            clk->periodCurrent = clk->periodTarget;
        }

        // Period/phase changes happen only here; the current cycle never jumps.
        clk->bpmCurrent = 60.0 / clk->periodCurrent;

        clk->nextBeatTime = clk->beatStartTime + clk->periodCurrent;
    }

    if (now >= clk->nextBeatTime) {
        double missed = floor((now - clk->beatStartTime) / clk->periodCurrent);
        clk->beatStartTime += missed * clk->periodCurrent;
        clk->nextBeatTime = clk->beatStartTime + clk->periodCurrent;
    }
}

// Consume each boundary once. Suppressed beats stay suppressed; emission
// spacing spans reanchors, BPM and ratio changes.
static int beat_clock_emit(BeatClock *clock, double now, double *lastEmit) {
    beat_clock_update(clock, now);
    if (!clock->initialized || clock->periodCurrent <= 0) return 0;
    if (clock->boundarySeen && clock->beatStartTime <= clock->lastBoundary) return 0;
    clock->boundarySeen = 1;
    clock->lastBoundary = clock->beatStartTime;
    if (now - clock->beatStartTime > fmin(.050, clock->periodCurrent * .125) + 1e-9) return 0;
    double minimumGap = .75 * fmax(clock->emittedPeriod, clock->periodCurrent);
    if (clock->emittedPeriod > 0 && now - *lastEmit + 1e-9 < minimumGap) return 0;
    *lastEmit = now;
    clock->emittedPeriod = clock->periodCurrent;
    return 1;
}

static double beat_clock_flash_phase(const BeatClock *clock, double now, double lastEmit) {
    return clock->emittedPeriod > 0 ? (now - lastEmit) / clock->emittedPeriod : -1;
}

static int bpm_confident_for_phase(int haveBpm, double historyCoverage,
                                   int candidateTrusted, double confidence,
                                   double candidateBpm, double heldBpm) {
    return haveBpm && historyCoverage >= STABILITY_WINDOW_SECONDS - 1e-6 && candidateTrusted &&
           isfinite(confidence) && confidence >= DECISION_CONFIDENCE_THRESHOLD &&
           isfinite(candidateBpm) && isfinite(heldBpm) &&
           heldBpm >= BPM_MIN && heldBpm <= BPM_MAX &&
           fabs(candidateBpm - heldBpm) < 0.5;
}

static void draw_bpm_scores(SDL_Renderer *ren,
                            int x0,
                            int y0,
                            int w,
                            int h,
                            const double *score,
                            int heldBpm,
                            double blinkPhase) {
    SDL_Rect rect = { x0, y0, w, h };
    double pulse = 0.0;
    if (blinkPhase >= 0.0 && blinkPhase < 1.0) {
        pulse = exp(-BLINK_DECAY * blinkPhase);
    }
    int bgInt = (int)lround(8.0 + 38.0 * pulse);
    if (bgInt < 0) bgInt = 0;
    if (bgInt > 255) bgInt = 255;
    Uint8 bg = (Uint8)bgInt;
    SDL_SetRenderDrawColor(ren, bg, bg, bg, 255);
    SDL_RenderFillRect(ren, &rect);

    if (!score) return;

    int dispBpmMin = TARGET_BPM_MIN;
    int dispBpmMax = TARGET_BPM_MAX;

    if (dispBpmMin < BPM_MIN) dispBpmMin = BPM_MIN;
    if (dispBpmMax > BPM_MAX) dispBpmMax = BPM_MAX;

    int dispCount = dispBpmMax - dispBpmMin + 1;
    if (dispCount <= 0) return;

    int barHeights[128];
    if (dispCount > (int)(sizeof(barHeights) / sizeof(barHeights[0]))) return;

    int plotY = y0;
    int plotH = h;

    double maxV = 0.0;
    for (int i = 0; i < dispCount; ++i) {
        double v = score[dispBpmMin + i - BPM_MIN];
        if (isfinite(v) && v >= 0.0 && v > maxV) maxV = v;
    }
    if (maxV < g_bpm_display_floor) maxV = g_bpm_display_floor;
    double target = maxV * BPM_HEADROOM;
    double baseAlpha = target > g_bpm_display_scale ? BPM_SCALE_SMOOTH_UP : BPM_SCALE_SMOOTH_DOWN;
    double now = SDL_GetTicks64() / 1000.0;
    double dt = g_last_bpm_draw_time > 0 ? fmax(0.0, now - g_last_bpm_draw_time) : (double)FFT_SIZE / SAMPLE_RATE;
    g_last_bpm_draw_time = now;
    double alpha = 1.0 - pow(1.0 - baseAlpha, dt * SAMPLE_RATE / FFT_SIZE);
    g_bpm_display_scale = (1.0 - alpha) * g_bpm_display_scale + alpha * target;
    double scaleMax = fmax(g_bpm_display_floor, g_bpm_display_scale);

    double step = (double)w / (double)dispCount;

    for (int i = 0; i < dispCount; ++i) {
        int bpm = dispBpmMin + i;
        int bin = bpm - BPM_MIN;
        double v = score[bin];
        if (!isfinite(v) || v < 0.0) v = 0.0;

        double m = v / scaleMax;
        if (m < 0.0) m = 0.0;
        if (m > 1.0) m = 1.0;

        barHeights[i] = (int)lround(m * (double)(plotH - 1));
    }

    // Mark the accepted integer BPM, independent of neighbouring score changes.
    int heldBin = heldBpm - dispBpmMin;
    if (heldBin >= 0 && heldBin < dispCount) {
        int xStart = x0 + (int)lround(heldBin * step);
        int xEnd = x0 + (int)lround((heldBin + 1) * step) - 1;
        if (xEnd < xStart) xEnd = xStart;

        SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        SDL_Rect hi = {
            xStart,
            plotY,
            xEnd - xStart + 1,
            plotH
        };
        SDL_RenderFillRect(ren, &hi);
    }

    for (int i = 0; i < dispCount; ++i) {
        int bpm = dispBpmMin + i;
        int r = 255, g = 255, b = 255;
        int bar = barHeights[i];

        int xStart = x0 + (int)lround(i * step);
        int xEnd = x0 + (int)lround((i + 1) * step) - 1;
        if (xEnd < xStart) xEnd = xStart;

        switch ((bpm / 10) % 10) {
            case 0: r = 255; g = 255; b = 0;   break;
            case 1: r = 255; g = 128; b = 0;   break;
            case 2: r = 255; g = 0;   b = 0;   break;
            case 3: r = 200; g = 0;   b = 200; break;
            case 4: r = 255; g = 0;   b = 255; break;
            case 5: r = 128; g = 0;   b = 255; break;
            case 6: r = 0;   g = 0;   b = 255; break;
            case 7: r = 0;   g = 128; b = 255; break;
            case 8: r = 0;   g = 255; b = 255; break;
            case 9: r = 128; g = 255; b = 0;   break;
        }

        SDL_SetRenderDrawColor(ren, (Uint8)r, (Uint8)g, (Uint8)b, 255);

        // Leave a one-pixel horizontal inset when possible, so the white
        // accepted-BPM backing remains visible behind the selected bar.
        int drawX0 = xStart;
        int drawX1 = xEnd;
        if (xEnd - xStart >= 2) {
            drawX0 = xStart + 1;
            drawX1 = xEnd - 1;
        }

        for (int x = drawX0; x <= drawX1; ++x) {
            SDL_RenderDrawLine(ren, x, plotY + plotH - 1, x, plotY + plotH - 1 - bar);
        }
    }

}


static double phase_graph_origin(const BeatClock *clock, double bpm) {
    double origin = clock->beatStartTime * bpm / 60.0;
    return origin - floor(origin);
}

// Coordinates in the full-cycle graph, measured from the emitted flash clock.
static double phase_relative_to_clock(double absolute, const BeatClock *clock, double bpm) {
    double relative = absolute - phase_graph_origin(clock, bpm);
    return relative - floor(relative);
}

static int phase_graph_sample_bin(double origin, int column, int width, int folds) {
    // Sample the evidence at the emitted clock boundary, then move forward
    // through one full cycle (or one folded half-cycle).
    double absolute = origin + (double)column / width / folds;
    absolute -= floor(absolute);
    return (int)floor(absolute * PHASE_GRAPH_BINS + 1e-4) % (PHASE_GRAPH_BINS / folds);
}

static int reset_output_timing_for_bpm(double previous, double next,
                                      int *offbeat, int *ratio, double *offset,
                                      Uint64 *lastPress, unsigned *presses) {
    if (!isfinite(previous) || !isfinite(next) || previous <= 0 || next <= 0 ||
        fabs(next - previous) < .5) return 0;
    *offbeat = 0;
    *ratio = 1;
    *offset = 0;
    *lastPress = 0;
    *presses = 0;
    return 1;
}

static const char *output_timing_name(int offbeat, int ratio) {
    return ratio == 2 ? "2x" : offbeat ? "offbeat" : "auto";
}

static int timing_button_next(int mode, Uint64 now, Uint64 *last, unsigned *presses) {
    int next;
    if (!*last || now - *last > 700) {
        next = mode == 0 ? 1 : 0;
        *presses = 1;
    } else {
        next = ++*presses == 2 ? 2 : (mode + 1) % 3;
    }
    *last = now;
    return next;
}

static void draw_target_choice_panel(SDL_Renderer *ren,
                                     int x0,
                                     int y0,
                                     int w,
                                     int h,
                                     const BeatDebug *dbg,
                                     const BeatClock *clk,
                                     int phaseMultiplier,
                                     double now) {
    if (!ren || !dbg || !clk || w <= 20 || h <= 20) return;

    SDL_Rect panel = { x0, y0, w, h };
    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderFillRect(ren, &panel);
    SDL_SetRenderDrawColor(ren, 80, 80, 80, 255);
    SDL_RenderDrawRect(ren, &panel);

    int confidenceW = w / 2;
    int confidenceX = x0;
    int confidenceY = y0;
    int confidenceH = h;

    int phaseX = x0 + confidenceW;
    int phaseY = y0;
    int phaseW = w - confidenceW;
    int phaseH = h;
    if (phaseW < 120) return;

    SDL_SetRenderDrawColor(ren, 18, 18, 18, 255);
    SDL_Rect confidenceRect = { confidenceX, confidenceY, confidenceW, confidenceH };
    SDL_Rect phaseRect = { phaseX, phaseY, phaseW, phaseH };
    SDL_RenderFillRect(ren, &confidenceRect);
    SDL_RenderFillRect(ren, &phaseRect);
    SDL_SetRenderDrawColor(ren, 90, 90, 90, 255);
    SDL_RenderDrawRect(ren, &confidenceRect);
    SDL_RenderDrawRect(ren, &phaseRect);

    SDL_Rect previousClip;
    SDL_bool hadClip = SDL_RenderIsClipEnabled(ren);
    SDL_RenderGetClipRect(ren, &previousClip);
    SDL_RenderSetClipRect(ren, &confidenceRect);

    /* Confidence strip: actual decision readiness. One is the threshold. */
    double thresholdNorm = DECISION_CONFIDENCE_THRESHOLD /
        DECISION_CONFIDENCE_DISPLAY_MAX;
    int thresholdY = confidenceY + confidenceH - 1 -
        (int)lround(thresholdNorm * (double)(confidenceH - 1));
    SDL_SetRenderDrawColor(ren, 0, 120, 0, 255);
    SDL_RenderDrawLine(
        ren,
        confidenceX,
        thresholdY,
        confidenceX + confidenceW - 1,
        thresholdY
    );

    for (int i = 1; i < 4; ++i) {
        int xx = confidenceX + (confidenceW * i) / 4;
        int yy = confidenceY + (confidenceH * i) / 4;
        SDL_SetRenderDrawColor(ren, 36, 36, 36, 255);
        SDL_RenderDrawLine(ren, xx, confidenceY, xx, confidenceY + confidenceH - 1);
        SDL_RenderDrawLine(ren, confidenceX, yy, confidenceX + confidenceW - 1, yy);
    }

    if (g_confidenceHistoryCount > 1) {
        int prevX = 0;
        int prevConfidenceY = 0;
        int prevEvidenceY = 0;
        for (int k = 0; k < g_confidenceHistoryCount; ++k) {
            int idx = g_confidenceHistoryWrite - g_confidenceHistoryCount + k;
            while (idx < 0) idx += CONFIDENCE_HISTORY_LEN;
            idx %= CONFIDENCE_HISTORY_LEN;
            double confidence = g_confidenceHistory[idx];
            double evidence = g_evidenceHistory[idx];
            double confidenceNorm = confidence / DECISION_CONFIDENCE_DISPLAY_MAX;
            double evidenceNorm = evidence / DECISION_CONFIDENCE_DISPLAY_MAX;
            if (confidenceNorm < 0.0) confidenceNorm = 0.0;
            if (confidenceNorm > 1.0) confidenceNorm = 1.0;
            if (evidenceNorm < 0.0) evidenceNorm = 0.0;
            if (evidenceNorm > 1.0) evidenceNorm = 1.0;
            int x = confidenceX + (int)lround(
                (double)(CONFIDENCE_HISTORY_LEN - g_confidenceHistoryCount + k) *
                (double)(confidenceW - 1) /
                (double)(CONFIDENCE_HISTORY_LEN - 1)
            );
            int confidenceYNow = confidenceY + confidenceH - 1 -
                (int)lround(confidenceNorm * (double)(confidenceH - 1));
            int evidenceYNow = confidenceY + confidenceH - 1 -
                (int)lround(evidenceNorm * (double)(confidenceH - 1));
            int good = confidence >= DECISION_CONFIDENCE_THRESHOLD;
            if (k > 0) {
                SDL_SetRenderDrawColor(ren, 0, 180, 255, 255);
                SDL_RenderDrawLine(ren, prevX, prevEvidenceY, x, evidenceYNow);
                if (good) SDL_SetRenderDrawColor(ren, 0, 255, 0, 255);
                else SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
                SDL_RenderDrawLine(ren, prevX, prevConfidenceY, x, confidenceYNow);
            }
            prevX = x;
            prevConfidenceY = confidenceYNow;
            prevEvidenceY = evidenceYNow;
        }
        int lastIdx = g_confidenceHistoryWrite - 1;
        if (lastIdx < 0) lastIdx += CONFIDENCE_HISTORY_LEN;
        SDL_SetRenderDrawColor(ren, 0, 180, 255, 255);
        SDL_Rect evidenceCurrent = { prevX - 2, prevEvidenceY - 2, 5, 5 };
        SDL_RenderFillRect(ren, &evidenceCurrent);
        int currentGood = g_confidenceHistory[lastIdx] >= DECISION_CONFIDENCE_THRESHOLD;
        SDL_SetRenderDrawColor(ren, currentGood ? 0 : 255, 255, currentGood ? 0 : 255, 255);
        SDL_Rect cur = { prevX - 3, prevConfidenceY - 3, 7, 7 };
        SDL_RenderFillRect(ren, &cur);
    }

    SDL_RenderSetClipRect(ren, hadClip ? &previousClip : NULL);

    /* Folded above, full cycle below, both referenced to the output clock. */
    (void)phaseMultiplier;
    (void)now;
    int n = dbg->targetPhaseCount;
    if (n != PHASE_GRAPH_BINS || !clk->initialized || !dbg->phaseValid ||
        dbg->targetBpm <= 0) return;
    double origin = phase_graph_origin(clk, dbg->targetBpm);
    for (int band = 0; band < 2; ++band) {
        int folds = band == 0 ? 2 : 1;
        int count = n / folds;
        int top = phaseY + band * (phaseH / 2);
        int height = band == 0 ? phaseH / 2 : phaseH - phaseH / 2;
        double values[PHASE_GRAPH_BINS], low = HUGE_VAL, high = 0;
        for (int i = 0; i < count; ++i) {
            values[i] = dbg->targetPhaseScore[i];
            if (folds == 2) values[i] += dbg->targetPhaseScore[i + count];
            low = fmin(low, values[i]);
            high = fmax(high, values[i]);
        }
        SDL_SetRenderDrawColor(ren, 210, 210, 210, 255);
        for (int col = 0; col < phaseW; ++col) {
            int bin = phase_graph_sample_bin(origin, col, phaseW, folds);
            double level = high > low ? (values[bin] - low) / (high - low) : 0;
            int bar = (int)lround(level * (height - 1));
            if (bar > 0) SDL_RenderDrawLine(ren, phaseX + col, top + height - 1,
                                           phaseX + col, top + height - 1 - bar);
        }
        SDL_SetRenderDrawColor(ren, 70, 70, 70, 255);
        SDL_RenderDrawLine(ren, phaseX, top, phaseX + phaseW - 1, top);
    }

}


static void push_confidence_timeline(double confidence, double evidenceQuality) {
    if (!isfinite(confidence) || !isfinite(evidenceQuality)) return;
    g_confidenceHistory[g_confidenceHistoryWrite] = confidence;
    g_evidenceHistory[g_confidenceHistoryWrite] = evidenceQuality;
    g_confidenceHistoryWrite =
        (g_confidenceHistoryWrite + 1) % CONFIDENCE_HISTORY_LEN;
    if (g_confidenceHistoryCount < CONFIDENCE_HISTORY_LEN) {
        g_confidenceHistoryCount++;
    }
}

int main(void) {
    int rc = 0;

    const int halfBins = FFT_SIZE / 2;
    const double frameRate = (double)SAMPLE_RATE / (double)FFT_SIZE;

    int maxFrames = (int)floor(RECORD_SECONDS * frameRate + 0.5);
    if (maxFrames < 4) maxFrames = 4;

    printf("Using ~%d frames (~%.2f seconds) at %.2f fps\n",
           maxFrames,
           (double)maxFrames / frameRate,
           frameRate);

    snd_pcm_t *cap_handle = NULL;
    int16_t *captureBuf = NULL;
    SDL_Window *win = NULL;
    SDL_Renderer *ren = NULL;

    double *hann = NULL;
    double *specReal = NULL;
    double *specImag = NULL;
    double *specMags = NULL;
    double *bandRing = NULL;
    double *bandOrdered = NULL;
    double *phaseOrdered = NULL;
    double *bpmScore = NULL;
    double *bpmJobScore = NULL;
    double *bpmResultScore = NULL;
    double *bpmPositive = NULL;
    double *bpmBcScore = NULL;
    double *bpmNormScore = NULL;
    double *bpmMeanNormScore = NULL;
    BPMWorker bpmWorker = {0};
    BeatDebug beatDebug = {0};

    int accumPairIdx = 0;
    int finalPairIdx = 0;
    double accumBpm = TARGET_BPM_MIN;
    double finalBpm = TARGET_BPM_MIN;
    double stabilityVariance = 0.0;
    double decisionConfidence = 0.0;
    double evidenceQuality = 0.0;

    int candidateTrusted = 0;
    int historyCount = 0;
    BpmHoldState bpmHold = {0};
    BeatClock beatClock = {0};
    PhaseTracker phaseTracker = {0};
    PhaseDecision phaseDecision = {0};
    PhaseEvidence phaseEvidence = {0};
    ImpulseFilterState impulseFilter = {0};
    int manualBpmForced = 0;
    int phaseForcedLocked = 0;
    int phaseMultiplier = 1;
    int offbeatEnabled = 0;
    double phaseOffset = 0;
    uint64_t suppressedFrames = 0;

    const int bpmCount = BPM_COUNT;
    ScoreHistory scoreHistory = {0};
    double historyCoverage = 0.0;
    double lastAnalysisWorkMs = 0, lastAnalysisWallMs = 0;
    double lastAnalysisTime = 0;

    do {
#define MALLOC_ARRAY(p, n) ((p) = malloc(sizeof *(p) * (size_t)(n)))
#define CALLOC_ARRAY(p, n) ((p) = calloc((size_t)(n), sizeof *(p)))

    int allocFailed =
        !MALLOC_ARRAY(hann, FFT_SIZE) || !MALLOC_ARRAY(specReal, FFT_SIZE) ||
        !MALLOC_ARRAY(specImag, FFT_SIZE) || !MALLOC_ARRAY(specMags, halfBins) ||
        !CALLOC_ARRAY(bandRing, maxFrames * COMB_BANDS) || !CALLOC_ARRAY(bandOrdered, maxFrames * COMB_BANDS) ||
        !CALLOC_ARRAY(phaseOrdered, maxFrames * COMB_BANDS) ||
        !CALLOC_ARRAY(bpmScore, bpmCount) || !CALLOC_ARRAY(bpmJobScore, bpmCount) ||
        !CALLOC_ARRAY(bpmResultScore, bpmCount) || !CALLOC_ARRAY(bpmPositive, bpmCount) ||
        !CALLOC_ARRAY(bpmBcScore, bpmCount) || !CALLOC_ARRAY(bpmNormScore, bpmCount) ||
        !CALLOC_ARRAY(bpmMeanNormScore, bpmCount);

    if (allocFailed) {
        fprintf(stderr, "Failed to allocate buffers\n");
        rc = 1;
        break;
    }

    for (int i = 0; i < FFT_SIZE; ++i) {
        hann[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (FFT_SIZE - 1)));
    }

    rc = setup_alsa_capture(&cap_handle);
    if (rc < 0) {
        rc = 1;
        break;
    }

    if (!MALLOC_ARRAY(captureBuf, FFT_SIZE)) {
        fprintf(stderr, "Failed to allocate capture buffer\n");
        rc = 1;
        break;
    }

#undef MALLOC_ARRAY
#undef CALLOC_ARRAY

    printf("Warming up microphone (2 seconds)...\n");
    {
        int warmSamples = SAMPLE_RATE * 2;
        int16_t dummyBuf[FFT_SIZE];
        int warmRemaining = warmSamples;

        while (warmRemaining > 0) {
            int toRead = warmRemaining < FFT_SIZE ? warmRemaining : FFT_SIZE;
            int rrc = snd_pcm_readi(cap_handle, dummyBuf, toRead);

            if (rrc == -EPIPE) {
                snd_pcm_prepare(cap_handle);
                continue;
            }

            if (rrc < 0) {
                fprintf(stderr, "Warm-up read error: %s\n", snd_strerror(rrc));
                break;
            }

            if (rrc == 0) {
                usleep(1000);
                continue;
            }

            warmRemaining -= rrc;
        }
    }
    printf("Microphone ready. BPM hop %.2fs; %d-bin batches over %.2fs; display follows audio frames.\n",
           BPM_UPDATE_SECONDS, BPM_WORK_BATCH,
           BPM_UPDATE_SECONDS * BPM_WORK_SPREAD_FRACTION);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        rc = 1;
        break;
    }

    int initialWidth = WIN_WIDTH, initialHeight = WIN_HEIGHT;
    SDL_Rect usable;
    if (SDL_GetDisplayUsableBounds(0, &usable) == 0) {
        if (initialWidth > usable.w - 40) initialWidth = usable.w - 40;
        if (initialHeight > usable.h - 80) initialHeight = usable.h - 80;
    }
    if (initialWidth < 320) initialWidth = 320;
    if (initialHeight < 240) initialHeight = 240;
    win = SDL_CreateWindow(
        "Live FFT",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        initialWidth,
        initialHeight,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!win) {
        fprintf(stderr, "SDL_CreateWindow error: %s\n", SDL_GetError());
        rc = 1;
        break;
    }

    SDL_SetWindowMinimumSize(win, 320, 240);
    ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    if (!ren) {
        fprintf(stderr, "SDL_CreateRenderer error: %s\n", SDL_GetError());
        rc = 1;
        break;
    }

    bpmWorker.mutex = SDL_CreateMutex();
    bpmWorker.cond = SDL_CreateCond();
    bpmWorker.jobFrames = bandOrdered;
    bpmWorker.jobScores = bpmJobScore;
    bpmWorker.resultScores = bpmResultScore;
    bpmWorker.bpmCount = bpmCount;

    if (!bpmWorker.mutex || !bpmWorker.cond) {
        fprintf(stderr, "Failed to create BPM worker sync objects\n");
        rc = 1;
        break;
    }

    bpmWorker.thread = SDL_CreateThread(bpm_worker_main, "bpm-worker", &bpmWorker);
    if (!bpmWorker.thread) {
        fprintf(stderr, "SDL_CreateThread error: %s\n", SDL_GetError());
        rc = 1;
        break;
    }

    init_scrolling_spectrogram(ren);

    printf("Controls: R clear accepted state | F force/unforce BPM | Space lock/unlock phase | D set output | O set offbeat | 1/2 ratio | [/] phase | Shift coarse | backslash zero | Esc quit.\n");

    Uint64 lastTimingPress = 0;
    unsigned timingPresses = 0;
    int running = 1;
    int ringWrite = 0;
    int frameCount = 0;
    uint64_t totalFrames = 0, lastAnalysisFrame = 0, lastPhaseAnalysisFrame = 0;
    double audioFrameTime = 0.0;
    double lastFlashEmit = 0.0;

    // Capture was started by warm-up and kept running during SDL setup. Its
    // queued samples predate the first timestamp below. Start a fresh stream
    // only after UI/worker initialization so that backlog is not labelled now.
    snd_pcm_sframes_t queued = snd_pcm_avail_update(cap_handle);
    printf("Capture ready: discarding startup backlog (%ld available frames).\n", (long)queued);
    rc = snd_pcm_drop(cap_handle);
    if (rc >= 0) rc = snd_pcm_prepare(cap_handle);
    if (rc < 0) {
        fprintf(stderr, "Cannot restart capture after UI setup: %s\n", snd_strerror(rc));
        break;
    }

    while (running) {
        SDL_Event e;

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running = 0;
            } else if (e.type == SDL_KEYDOWN && !e.key.repeat) {
                SDL_Keycode key = e.key.keysym.sym;
                if (key == SDLK_ESCAPE) {
                    running = 0;
                } else if (key == SDLK_r) {
                    // Song boundary: forget decisions, not evidence. The audio ring,
                    // score history, candidate and any in-flight analysis stay valid.
                    finalPairIdx = accumPairIdx;
                    finalBpm = accumBpm;
                    decisionConfidence = 0.0;
                    candidateTrusted = 0;
                    manualBpmForced = 0;
                    phaseForcedLocked = 0;
                    phaseMultiplier = 1;
                    offbeatEnabled = 0;
                    phaseOffset = 0;
                    lastTimingPress = 0;
                    timingPresses = 0;
                    lastFlashEmit = 0.0;
                    bpm_hold_reset(&bpmHold);
                    memset(&beatClock, 0, sizeof(beatClock));
                    phase_tracker_reset(&phaseTracker);
                    memset(&phaseDecision, 0, sizeof(phaseDecision));
                    memset(&beatDebug, 0, sizeof(beatDebug));
                    printf("Accepted BPM/phase cleared; audio evidence retained.\n");
                } else if (key == SDLK_f) {
                    if (!historyCount) {
                        printf("Force ignored: no BPM evidence yet.\n");
                    } else if (!manualBpmForced) {
                        double previousBpm = bpmHold.have ? bpmHold.bpm : 0;
                        manualBpmForced = 1;
                        bpm_hold_force(&bpmHold, accumBpm,
                            (int)accumBpm < BPM_MAX ? (int)accumBpm - BPM_MIN : BPM_COUNT - 2);
                        finalBpm = bpmHold.bpm;
                        finalPairIdx = bpmHold.pairIdx;
                        reset_output_timing_for_bpm(previousBpm, finalBpm,
                            &offbeatEnabled, &phaseMultiplier, &phaseOffset, &lastTimingPress, &timingPresses);
                        printf("BPM forced at %.0f.\n", finalBpm);
                    } else {
                        manualBpmForced = 0;
                        bpm_hold_force(&bpmHold, finalBpm, finalPairIdx);
                        printf("BPM force released at %.0f; automatic challengers restart empty.\n", finalBpm);
                    }
                } else if (key == SDLK_SPACE) {
                    if (!phaseTracker.active.have || !beatClock.initialized) {
                        printf("Phase lock ignored: automatic phase is not ready yet.\n");
                    } else {
                        phaseForcedLocked = !phaseForcedLocked;
                        printf(
                            "Automatic phase %s; BPM soft-lock %s.\n",
                            phaseForcedLocked ? "locked" : "released",
                            phaseForcedLocked ? "enabled" : "disabled"
                        );
                    }
                } else if (key == SDLK_LEFTBRACKET || key == SDLK_RIGHTBRACKET || key == SDLK_BACKSLASH) {
                    double step = (e.key.keysym.mod & KMOD_SHIFT) ? .05 : .01;
                    phaseOffset = key == SDLK_BACKSLASH ? 0 :
                        phase_wrap01(phaseOffset + (key == SDLK_LEFTBRACKET ? -step : step));
                    if (beatClock.initialized && phaseDecision.valid) {
                        double basePeriod = 60 / phaseDecision.preciseBpm;
                        double anchor = phaseDecision.flashAnchorTime + phaseOffset * basePeriod;
                        if (offbeatEnabled && phaseMultiplier == 1) anchor += .5 * basePeriod;
                        beatDebug.acceptedBeatTime = anchor;
                        beat_clock_init_from_detected_beat(&beatClock,
                            phaseDecision.preciseBpm * phaseMultiplier, anchor,
                            SDL_GetTicks64() / 1000.0);
                    }
                } else if (key == SDLK_o || key == SDLK_d) {
                    if (key == SDLK_d) {
                        int mode = phaseMultiplier == 2 ? 2 : offbeatEnabled;
                        mode = timing_button_next(mode, SDL_GetTicks64(),
                                                  &lastTimingPress, &timingPresses);
                        phaseMultiplier = mode == 2 ? 2 : 1;
                        offbeatEnabled = mode == 1;
                    } else {
                        offbeatEnabled = !offbeatEnabled;
                    }
                    if (beatClock.initialized && phaseDecision.valid) {
                        double now = SDL_GetTicks64() / 1000.0;
                        double basePeriod = 60.0 / phaseDecision.preciseBpm;
                        double anchor = phaseDecision.flashAnchorTime + phaseOffset * basePeriod;
                        if (offbeatEnabled && phaseMultiplier == 1) anchor += 0.5 * basePeriod;
                        beatDebug.acceptedBeatTime = anchor;
                        beat_clock_init_from_detected_beat(&beatClock,
                            phaseDecision.preciseBpm * phaseMultiplier, anchor, now);
                    }
                    printf("Output: %s.\n", output_timing_name(offbeatEnabled, phaseMultiplier));
                } else if (key == SDLK_1 || key == SDLK_2) {
                    int nextMultiplier = key == SDLK_1 ? 1 : 2;
                    if (nextMultiplier != phaseMultiplier) {
                        double now = SDL_GetTicks64() / 1000.0;
                        phaseMultiplier = nextMultiplier;
                        if (bpmHold.have && beatClock.initialized && phaseDecision.valid) {
                            double basePeriod = 60.0 / phaseDecision.preciseBpm;
                            double anchor = phaseDecision.flashAnchorTime + phaseOffset * basePeriod;
                            if (offbeatEnabled && phaseMultiplier == 1) anchor += 0.5 * basePeriod;
                            beatDebug.acceptedBeatTime = anchor;
                            beat_clock_init_from_detected_beat(
                                &beatClock,
                                phaseDecision.preciseBpm * phaseMultiplier,
                                anchor,
                                now
                            );
                        }
                        printf("Output: %s.\n", output_timing_name(offbeatEnabled, phaseMultiplier));
                    }
                }
            }
        }

        rc = read_exact_frames(cap_handle, captureBuf, FFT_SIZE);
        double captureReadTime = SDL_GetTicks64() / 1000.0;
        if (rc < 0) {
            fprintf(stderr, "read_exact_frames failed: %d\n", rc);
            break;
        }

        if (rc > 0) {
            // ALSA overrun: retain the clock/hold, rebuild contiguous evidence.
            SDL_LockMutex(bpmWorker.mutex);
            ++bpmWorker.epoch;
            bpmWorker.resultReady = 0;
            SDL_UnlockMutex(bpmWorker.mutex);
            frameCount = ringWrite = 0;
            memset(&scoreHistory, 0, sizeof(scoreHistory));
            memset(&impulseFilter, 0, sizeof(impulseFilter));
            historyCount = 0;
            historyCoverage = decisionConfidence = evidenceQuality = 0;
            candidateTrusted = 0;
            bpmHold.havePreviousVariance = bpmHold.previousTrusted = 0;
            beatClock.hasPendingBpm = 0;
            phaseTracker.active.filteredErrorSeconds = 0;
            audioFrameTime = 0;
            fprintf(stderr, "Capture gap: rebuilding evidence; retaining BPM %.0f\n",
                    bpmHold.have ? bpmHold.bpm : 0);
        }

        for (int i = 0; i < FFT_SIZE; ++i) {
            double s = fmax(-1.0, fmin(1.0, captureBuf[i] / 8192.0));
            specReal[i] = s * hann[i];
            specImag[i] = 0.0;
        }

        fft(specReal, specImag, FFT_SIZE, 0);

        for (int k = 0; k < halfBins; ++k) {
            double re = specReal[k];
            double im = specImag[k];
            specMags[k] = sqrt(re * re + im * im);
        }

        update_scrolling_spectrogram(specMags);

        double *dst = &bandRing[ringWrite * COMB_BANDS];

        magnitudes_to_bands(specMags, dst);
        suppressedFrames += filter_and_weight_band_frame(dst, &impulseFilter);

        ringWrite = (ringWrite + 1) % maxFrames;

        if (frameCount < maxFrames) {
            frameCount++;
        }

        totalFrames++;
        if (audioFrameTime <= 0.0)
            // Suit capture origin: end of the first successful read minus one
            // sample; subsequent frame times advance by sample count.
            audioFrameTime = captureReadTime - 1.0 / SAMPLE_RATE;
        else
            audioFrameTime += (double)FFT_SIZE / SAMPLE_RATE;

        int numFrames = frameCount;


        SDL_LockMutex(bpmWorker.mutex);
        if (bpmWorker.resultReady) {
            memcpy(bpmScore,
                   bpmWorker.resultScores,
                   sizeof(double) * (size_t)bpmCount);
            raw_scores_to_positive(bpmScore, bpmCount, bpmPositive);
            bpm_baseline_correct(bpmPositive, bpmCount, bpmBcScore);
            normalize_scores_by_sum(bpmBcScore, bpmCount, bpmNormScore);

            score_history_push(&scoreHistory, bpmNormScore,
                               bpmWorker.resultLastFrameTime, BPM_UPDATE_SECONDS);
            double candidateConsistency = 0;
            historyCoverage = score_history_stats(&scoreHistory, bpmMeanNormScore,
                                                  &stabilityVariance, &candidateConsistency);
            historyCount = scoreHistory.count;
            accumPairIdx = strongest_pair(bpmMeanNormScore);
            double accumPairScore = bpmMeanNormScore[accumPairIdx] + bpmMeanNormScore[accumPairIdx + 1];
            lastAnalysisWorkMs = bpmWorker.resultWorkMs;
            lastAnalysisWallMs = bpmWorker.resultWallMs;
            lastAnalysisTime = bpmWorker.resultLastFrameTime;

            int accumNext = accumPairIdx + 1;
            int bestAccumBin = (bpmMeanNormScore[accumNext] > bpmMeanNormScore[accumPairIdx])
                ? accumNext
                : accumPairIdx;
            accumBpm = (double)(BPM_MIN + bestAccumBin);
            double previousBpm = bpmHold.have ? bpmHold.bpm : 0;
            update_final_bpm_from_accum(
                accumBpm,
                accumPairIdx,
                accumPairScore,
                candidateConsistency,
                stabilityVariance,
                bpmWorker.resultLastFrameTime,
                !manualBpmForced,
                phaseForcedLocked,
                &bpmHold,
                &finalBpm,
                &finalPairIdx,
                &decisionConfidence,
                &evidenceQuality,
                &candidateTrusted
            );
            reset_output_timing_for_bpm(previousBpm, bpmHold.have ? bpmHold.bpm : 0,
                &offbeatEnabled, &phaseMultiplier, &phaseOffset, &lastTimingPress, &timingPresses);

            bpmWorker.resultReady = 0;
        }
        SDL_UnlockMutex(bpmWorker.mutex);

        // Phase is cheap and latency-sensitive, so refresh it independently of the
        // expensive BPM sweep. The live path computes only the temporal mean.
        if (numFrames >= 3 && bpmHold.have &&
            (totalFrames - lastPhaseAnalysisFrame) >= PHASE_UPDATE_EVERY_FRAMES) {
            // Use sample progression rather than analysis completion time. This
            // removes UI/thread scheduling jitter from phase coordinates.
            double phaseNow = audioFrameTime;
            int strictPhaseAcquisition = bpm_confident_for_phase(
                historyCount >= 2, historyCoverage, candidateTrusted,
                decisionConfidence, accumBpm, finalBpm
            );
            copy_band_history(bandRing, maxFrames, ringWrite, numFrames, phaseOrdered);
            double historyStartTime = phaseNow - (double)(numFrames - 1) / frameRate;
            int phaseOk = compute_phase_evidence_mean(
                phaseOrdered, numFrames, COMB_BANDS, frameRate, finalBpm,
                historyStartTime, &phaseEvidence
            );
            lastPhaseAnalysisFrame = totalFrames;
            beatDebug.targetBpm = finalBpm;
            beatDebug.targetFrameRate = frameRate;
            beatDebug.targetLastFrameTime = phaseNow;
            beatDebug.targetNumFrames = numFrames;
            beatDebug.targetPhaseCount = PHASE_GRAPH_BINS;

            if (phaseOk) {
                const PhaseScoreMode liveMode = PHASE_SCORE_MEAN;
                int phaseUpdateAllowed = strictPhaseAcquisition;
                memcpy(beatDebug.targetPhaseScore, phaseEvidence.score[liveMode],
                       sizeof(beatDebug.targetPhaseScore));
                phaseDecision = phase_tracker_update(
                    &phaseTracker, &phaseEvidence, liveMode, finalBpm, phaseNow,
                    phaseUpdateAllowed, phaseForcedLocked
                );
                if (phaseDecision.valid) {
                    double basePeriod = 60.0 / phaseDecision.preciseBpm;
                    double clockBeatTime = phaseDecision.flashAnchorTime + phaseOffset * basePeriod;
                    if (offbeatEnabled && phaseMultiplier == 1)
                        clockBeatTime += 0.5 * basePeriod;
                    double visualBpm = phaseDecision.preciseBpm * phaseMultiplier;
                    int selectedBin = phaseDecision.familyBin +
                        phaseDecision.selectedSide * PHASE_FAMILY_BINS;
                    double selectedPhase = (double)selectedBin / PHASE_GRAPH_BINS;
                    double rawPhase = (double)phaseDecision.rawBestBin / PHASE_GRAPH_BINS;

                    beatDebug.phaseValid = 1;
                    beatDebug.phaseSwitched = phaseDecision.familySwitched;
                    beatDebug.detectedBeatTime = phaseDecision.flashAnchorTime;
                    beatDebug.acceptedBeatTime = clockBeatTime;
                    beatDebug.candidateBeatTime = phase_time_for_nominal_phase(
                        rawPhase, finalBpm, phaseNow);
                    beatDebug.candidatePhaseFrames = rawPhase;
                    beatDebug.candidatePhaseScore =
                        phaseEvidence.score[liveMode][phaseDecision.rawBestBin];
                    beatDebug.bestPhaseFrames = selectedPhase;
                    beatDebug.bestPhaseScore = phaseDecision.familyScore;
                    beatDebug.heldPhaseFrames = selectedPhase;
                    beatDebug.heldPhaseScore = phaseDecision.familyScore;
                    beatDebug.preciseBpm = phaseDecision.preciseBpm;
                    beatDebug.familyProminence = phaseDecision.familyProminence;
                    beatDebug.detectedPhase = selectedPhase;
                    beatDebug.phaseErrorSec = phaseDecision.phaseErrorSeconds;
                    beatDebug.nudgeSec = phaseDecision.correctionSeconds;
                    beatDebug.nudgeApplied = phaseDecision.correctionApplied;
                    beatDebug.phaseLockAllowed = phaseUpdateAllowed;
                    if (!beatClock.initialized || phaseDecision.acquired ||
                        phaseDecision.restored) {
                        if (phaseUpdateAllowed) {
                            beat_clock_init_from_detected_beat(
                                &beatClock, visualBpm, clockBeatTime, SDL_GetTicks64() / 1000.0);
                        }
                    } else if (phaseUpdateAllowed) {
                        beat_clock_schedule_from_detected_beat(
                            &beatClock, visualBpm, clockBeatTime);
                    }
                    if (!phaseUpdateAllowed) beatClock.hasPendingBpm = 0;
                }
            } else {
                beatDebug.phaseValid = phaseTracker.active.have;
                beatDebug.phaseLockAllowed = 0;
                beatDebug.nudgeApplied = 0;
            }
        }
        if (numFrames >= 3 && (totalFrames - lastAnalysisFrame) >= BPM_UPDATE_EVERY_FRAMES) {
            SDL_LockMutex(bpmWorker.mutex);
            if (!bpmWorker.busy && !bpmWorker.resultReady) {
                copy_band_history(
                    bandRing,
                    maxFrames,
                    ringWrite,
                    numFrames,
                    bandOrdered
                );
                bpmWorker.jobNumFrames = numFrames;
                bpmWorker.jobFrameRate = frameRate;
                bpmWorker.jobLastFrameTime = audioFrameTime;
                bpmWorker.jobEpoch = bpmWorker.epoch;
                lastAnalysisFrame = totalFrames;
                bpmWorker.busy = 1;
                bpmWorker.hasJob = 1;
                SDL_CondSignal(bpmWorker.cond);
            }
            SDL_UnlockMutex(bpmWorker.mutex);
        }

        push_confidence_timeline(decisionConfidence, evidenceQuality);

        double drawNowSec = SDL_GetTicks64() / 1000.0;
        // Suit behavior: sample-derived evidence is anchored to monotonic
        // time; emitted beats advance on that monotonic clock.
        double beatNowSec = drawNowSec;
        int flashPeakFrame = beat_clock_emit(&beatClock, beatNowSec, &lastFlashEmit);
        if (beatClock.initialized) {
            beatDebug.nextBeatInSec = beatClock.nextBeatTime - beatNowSec;
            if (beatDebug.nextBeatInSec < 0.0) beatDebug.nextBeatInSec = 0.0;
            if (beatDebug.phaseValid) {
                beatDebug.detectedPhase = beatDebug.bestPhaseFrames;
            }
        }
        double blinkPhase = beat_clock_flash_phase(&beatClock, beatNowSec, lastFlashEmit);

        char windowTitle[384];
        if (bpmHold.have) {
            snprintf(
                windowTitle,
                sizeof(windowTitle),
                "LatestFFT | candidate %.0f | held %.0f %s | clock %.2f | confidence %.2f evidence %.2f | phase %s err %+.0fms prom %.2f | output %s phase %+.2f",
                accumBpm,
                finalBpm,
                manualBpmForced ? "FORCED" : (phaseForcedLocked ? "SOFT" : "AUTO"),
                phaseDecision.valid ? phaseDecision.preciseBpm : finalBpm,
                decisionConfidence,
                evidenceQuality,
                phaseForcedLocked ? "LOCKED" : "TRACKING",
                phaseDecision.valid ? phaseDecision.phaseErrorSeconds * 1000.0 : 0.0,
                phaseDecision.valid ? phaseDecision.familyProminence : 0.0,
                output_timing_name(offbeatEnabled, phaseMultiplier),
                phaseOffset - floor(phaseOffset + .5)
            );
        } else {
            snprintf(
                windowTitle,
                sizeof(windowTitle),
                "LatestFFT | candidate %.0f | seeking lock | confidence %.2f evidence %.2f | output %s phase %+.2f",
                accumBpm,
                decisionConfidence,
                evidenceQuality,
                output_timing_name(offbeatEnabled, phaseMultiplier),
                phaseOffset - floor(phaseOffset + .5)
            );
        }
        size_t titleLength = strlen(windowTitle);
        snprintf(windowTitle + titleLength, sizeof(windowTitle) - titleLength,
                 " | hop %.2fs work %.0fms wall %.0fms age %.1fs",
                 BPM_UPDATE_SECONDS, lastAnalysisWorkMs, lastAnalysisWallMs,
                 lastAnalysisTime > 0 ? fmax(0.0, drawNowSec - lastAnalysisTime) : 0.0);
        if (beatClock.initialized && beatDebug.phaseValid && beatDebug.targetBpm > 0) {
            int peak = 0;
            for (int i = 1; i < beatDebug.targetPhaseCount; ++i)
                if (beatDebug.targetPhaseScore[i] > beatDebug.targetPhaseScore[peak]) peak = i;
            double peakRelative = phase_relative_to_clock(
                peak / (double)PHASE_GRAPH_BINS, &beatClock, beatDebug.targetBpm);
            double targetAbsolute = beatDebug.acceptedBeatTime * beatDebug.targetBpm / 60.0;
            double targetRelative = phase_relative_to_clock(
                targetAbsolute, &beatClock, beatDebug.targetBpm);
            // Signed target error makes a tiny negative offset readable as -0.01,
            // rather than 0.99 on the circular graph.
            if (targetRelative > .5) targetRelative -= 1;
            titleLength = strlen(windowTitle);
            snprintf(windowTitle + titleLength, sizeof(windowTitle) - titleLength,
                     " | rawpeak %.2f rawtarget %+.2f capture %+.0fms",
                     peakRelative, targetRelative, (drawNowSec - audioFrameTime) * 1000);
        }
        SDL_SetWindowTitle(win, windowTitle);

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        int renderWidth = 0, renderHeight = 0;
        if (SDL_GetRendererOutputSize(ren, &renderWidth, &renderHeight) != 0 ||
            renderWidth < 2 || renderHeight < 2) continue;
        int panelW = renderWidth / 2;
        int topH = renderHeight / 2;
        int bottomY = topH;
        int bottomH = renderHeight - topH;

        if (specTex && specPixels) {
            SDL_Rect src = { 0, 0, SPEC_WIDTH, SPEC_HEIGHT };
            SDL_Rect out = { 0, 0, panelW, topH };

            SDL_UpdateTexture(specTex, NULL, specPixels, SPEC_WIDTH * (int)sizeof(Uint32));
            SDL_RenderCopy(ren, specTex, &src, &out);
            SDL_SetRenderDrawColor(ren, 80, 80, 80, 255);
            SDL_Rect specBorder = { 0, 0, panelW, topH };
            SDL_RenderDrawRect(ren, &specBorder);
        }

        draw_bpm_scores(
            ren,
            panelW,
            0,
            renderWidth - panelW,
            topH,
            bpmMeanNormScore,
            bpmHold.have ? (int)lround(bpmHold.bpm) : -1,
            flashPeakFrame ? 0.0 : blinkPhase
        );

        draw_target_choice_panel(
            ren,
            0,
            bottomY,
            renderWidth,
            bottomH,
            &beatDebug,
            &beatClock,
            phaseMultiplier,
            drawNowSec
        );

        SDL_RenderPresent(ren);
    }

    } while (0);

    if (bpmWorker.thread) {
        SDL_LockMutex(bpmWorker.mutex);
        bpmWorker.stop = 1;
        SDL_CondSignal(bpmWorker.cond);
        SDL_UnlockMutex(bpmWorker.mutex);
        SDL_WaitThread(bpmWorker.thread, NULL);
    }
    if (bpmWorker.cond) SDL_DestroyCond(bpmWorker.cond);
    if (bpmWorker.mutex) SDL_DestroyMutex(bpmWorker.mutex);

    if (captureBuf) free(captureBuf);
    if (cap_handle) snd_pcm_close(cap_handle);

    if (specTex) SDL_DestroyTexture(specTex);
    if (specPixels) free(specPixels);
    if (ren) SDL_DestroyRenderer(ren);
    if (win) SDL_DestroyWindow(win);
    if (SDL_WasInit(SDL_INIT_VIDEO)) SDL_Quit();

    free(hann);
    free(specReal);
    free(specImag);
    free(specMags);
    free(bandRing);
    free(bandOrdered);
    free(phaseOrdered);
    free(bpmScore);
    free(bpmJobScore);
    free(bpmResultScore);
    free(bpmPositive);
    free(bpmBcScore);
    free(bpmNormScore);
    free(bpmMeanNormScore);

    printf("Suppressed %llu impulse frames.\n", (unsigned long long)suppressedFrames);

    return rc ? 1 : 0;
}
