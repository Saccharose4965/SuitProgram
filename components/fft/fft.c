#include "fft.h"
#include "fft_internal.h"
#include "fft_render.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "audio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "led.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "fft_vis";

// ------------------- Config -------------------
static const int   kFlashFrames   = 5;      // frames to hold beat flash
static const float kNoveltyMeanOffset = 0.02f; // lifts rolling mean slightly to gate quiet peaks
static const float kHopRate_nominal = (float)SAMPLE_RATE_HZ / (float)HOP_SAMPLES; // nominal hop rate (62.5 Hz)
static const int   kTempoUpdateIntervalFrames = FPS/2; // update tempo estimate twice per second
#define BPM_SCORE_WINDOW_UPDATES 8 // ~4s at 2 tempo updates/sec
#define PHASE_SLOTS     32     // downsampled phase bins for display

// ------------------- DSP tuning -------------------
static const float kGlobalCombMin        = 0.01f;
static const float kNoveltyZeroThreshold = 400.0f;
static const float kOverallMinConf       = 0.25f;
static const float kFamilyTauUpSec       = 0.35f;
static const float kFamilyTauDownSec     = 1.20f;
#define MAX_PHASE_EVAL 256

static inline float wrap_phase01(float phase){
    phase -= floorf(phase);
    if (phase < 0.0f) phase += 1.0f;
    return phase;
}

static void shift_phase_curve_circular(float *vals, uint8_t count, float shift_norm){
    if (!vals || count < 2) return;

    shift_norm = wrap_phase01(shift_norm);
    float shift = shift_norm * (float)count;
    float tmp[PHASE_SLOTS];

    for (int i = 0; i < count; ++i){
        float src = (float)i - shift;
        while (src < 0.0f) src += (float)count;
        while (src >= (float)count) src -= (float)count;

        int i0 = (int)floorf(src);
        int i1 = i0 + 1;
        if (i1 >= count) i1 = 0;
        float frac = src - (float)i0;

        float v0 = vals[i0];
        float v1 = vals[i1];
        tmp[i] = v0 * (1.0f - frac) + v1 * frac;
    }

    memcpy(vals, tmp, (size_t)count * sizeof(float));
}

// ------------------- FFT core -------------------
static void fft_radix2(float re[], float im[], int n){
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j &= ~bit;
        j |= bit;
        if (i < j) {
            float tr = re[i]; re[i]=re[j]; re[j]=tr;
            float ti = im[i]; im[i]=im[j]; im[j]=ti;
        }
    }
    for (int len=2; len<=n; len<<=1) {
        float ang = -2.0f*(float)M_PI/(float)len;
        float wlen_cos=cosf(ang), wlen_sin=sinf(ang);
        for (int i=0;i<n;i+=len){
            float wcos=1.0f, wsin=0.0f;
            for (int j=0;j<len/2;++j){
                int u=i+j, v=u+len/2;
                float tr=wcos*re[v]-wsin*im[v];
                float ti=wcos*im[v]+wsin*re[v];
                re[v]=re[u]-tr; im[v]=im[u]-ti;
                re[u]+=tr; im[u]+=ti;
                float nwcos=wcos*wlen_cos - wsin*wlen_sin;
                float nwsin=wcos*wlen_sin + wsin*wlen_cos;
                wcos=nwcos; wsin=nwsin;
            }
        }
    }
}

// ------------------- State -------------------
static EXT_RAM_BSS_ATTR float   g_hop_buf[HOP_SAMPLES];
static size_t  g_hop_fill=0;
static EXT_RAM_BSS_ATTR float   g_fft_buf[FFT_SIZE];
static size_t  g_fft_fill=0;
static EXT_RAM_BSS_ATTR float   g_window[FFT_SIZE];
static EXT_RAM_BSS_ATTR float   g_fft_re[FFT_SIZE];
static EXT_RAM_BSS_ATTR float   g_fft_im[FFT_SIZE];
static EXT_RAM_BSS_ATTR float   g_prev_logmag[FFT_SIZE/2+1];
static bool    g_prev_logmag_valid=false;
static float   g_last_cand_bpm     = 0.0f; // last candidate BPM from spectrum
static int     g_bpm_stable_frames = 0;    // how many consecutive frames it's stayed similar

// novelty history (cleaned novelty stored in g_novelty_ring)
static EXT_RAM_BSS_ATTR float   g_novelty_ring[NOV_RING_FRAMES];
static int     g_nov_write_pos=0; // next slot to overwrite (also oldest sample)
static uint64_t g_nov_total_frames=0;

static EXT_RAM_BSS_ATTR float   g_local_mem[NOVELTY_WIN];
static int     g_local_idx=0;
static int     g_local_count=0;
static float   g_local_sum=0.0f;

static uint64_t g_prof_accum_us = 0;     // accumulated processing time in microseconds
static int      g_prof_frame_count = 0;  // frames counted in the accumulator
static float    g_last_load_pct = 0.0f;
static float    g_hop_rate_hz_est = kHopRate_nominal;   // immediate hop-rate estimate from timer
static int64_t  g_last_frame_time_us = 0;
static float    g_bpm_hz = 0.0f;
static float    g_bpm_conf = 0.0f;
static float    g_bpm_phase = 0.0f;
static float    g_blink_bpm_hz = 0.0f;  // latched BPM that drives the blinker until next beat
static float    g_blink_rate_hz = 0.0f; // actual blink rate used for display
static float    g_beat_phase = 0.0f;    // phase accumulator for main beat blinker
static int      g_tempo_update_countdown = 0;
static QueueHandle_t s_beat_queue = NULL;
static volatile int g_nov_hole_backfill_frames = 0;  // recent frames to retroactively clear
static volatile int g_nov_hole_future_frames = 0;    // upcoming frames to clear (timing jitter guard)

// ------------------- BPM export and smoothing -------------------
#define MAX_BPM_EXPORT  3      // export at most 3 BPMs (fundamental, double, special)
#define BPM_AVG_FRAMES  6      // rolling average length for main BPM
static float g_bpm_list[MAX_BPM_EXPORT];
static float g_bpm_conf_list[MAX_BPM_EXPORT];
static int   g_bpm_list_count = 0;

// history of the fundamental BPM per tempo-update frame
static EXT_RAM_BSS_ATTR float g_fund_hist[BPM_AVG_FRAMES];
static int   g_fund_hist_pos   = 0;
static int   g_fund_hist_count = 0;
static EXT_RAM_BSS_ATTR float   g_phase_curve[PHASE_SLOTS];
static uint8_t g_phase_curve_len = 0;
static EXT_RAM_BSS_ATTR float g_family_spec_smooth[BPM_MAX - BPM_MIN + 1];
static EXT_RAM_BSS_ATTR float g_bpm_score_hist[BPM_SCORE_WINDOW_UPDATES][BPM_MAX - BPM_MIN + 1];
static EXT_RAM_BSS_ATTR float g_bpm_score_sum[BPM_MAX - BPM_MIN + 1];
static int g_bpm_score_hist_idx = 0;
static int g_bpm_score_hist_count = 0;

// Rolling average of recent fundamental estimates
static float update_fund_history(float bpm_inst){
    if (bpm_inst <= 0.0f) return 0.0f;

    g_fund_hist[g_fund_hist_pos] = bpm_inst;
    g_fund_hist_pos = (g_fund_hist_pos + 1) % BPM_AVG_FRAMES;
    if (g_fund_hist_count < BPM_AVG_FRAMES) g_fund_hist_count++;

    float sum = 0.0f;
    for (int f = 0; f < g_fund_hist_count; ++f){
        int idx = (g_fund_hist_pos - 1 - f + BPM_AVG_FRAMES) % BPM_AVG_FRAMES;
        sum += g_fund_hist[idx];
    }
    return sum / (float)g_fund_hist_count;
}

static void bpm_soft_decay_and_reset(void){
    g_bpm_conf *= 0.9f;
    g_last_cand_bpm = 0.0f;
    g_bpm_stable_frames = 0;
    g_fund_hist_count = 0;
    g_fund_hist_pos   = 0;
    g_bpm_list_count = 0;
    g_bpm_hz    = 0.0f;
    g_bpm_phase = 0.0f;
    g_phase_curve_len = 0;
    g_bpm_score_hist_idx = 0;
    g_bpm_score_hist_count = 0;
    memset(g_bpm_score_hist, 0, sizeof(g_bpm_score_hist));
    memset(g_bpm_score_sum, 0, sizeof(g_bpm_score_sum));
}

static void apply_novelty_backfill_hole(void){
    int frames = g_nov_hole_backfill_frames;
    if (frames <= 0) return;
    g_nov_hole_backfill_frames = 0;

    int history_len = (g_nov_total_frames < (uint64_t)NOV_RING_FRAMES)
                        ? (int)g_nov_total_frames
                        : NOV_RING_FRAMES;
    if (history_len <= 0) return;
    int shift = frames / 2; // move hole back by ~half its width
    int max_erasable = history_len - shift;
    if (max_erasable <= 0) return;
    if (frames > max_erasable) frames = max_erasable;

    // Clear a same-width window shifted older by `shift`.
    for (int i = 0; i < frames; ++i){
        int idx = g_nov_write_pos - 1 - (shift + i);
        while (idx < 0) idx += NOV_RING_FRAMES;
        g_novelty_ring[idx] = 0.0f;
    }
}

// Public hook: clear a recent novelty window (retroactive) for button-induced spikes.
void fft_punch_novelty_hole(int frames){
    if (frames < 1) frames = 1;
    if (frames > NOV_RING_FRAMES) frames = NOV_RING_FRAMES;
    if (frames > g_nov_hole_backfill_frames){
        g_nov_hole_backfill_frames = frames;
    }
    if (frames > g_nov_hole_future_frames){
        g_nov_hole_future_frames = frames;
    }
}

// ------------------- Novelty computation -------------------

static float compute_novelty(const float *logmag){
    float sum = 0.0f;

    if (!g_prev_logmag_valid){
        memcpy(g_prev_logmag, logmag, sizeof(g_prev_logmag));
        g_prev_logmag_valid = true;
        return 0.0f;
    }

    // use band ~100 Hz .. Nyquist
    const float fs   = (float)SAMPLE_RATE_HZ;
    const int   nfft = FFT_SIZE;
    int kLow  = (int)ceilf(100.0f * (float)nfft / fs);
    int kHigh = (int)floorf((fs * 0.5f) * (float)nfft / fs); // ≈ Nyquist

    if (kLow  < 0)            kLow  = 0;
    if (kLow  > nfft/2)       kLow  = nfft/2;
    if (kHigh < 0)            kHigh = 0;
    if (kHigh > nfft/2)       kHigh = nfft/2;
    if (kHigh < kLow)         kHigh = kLow;

    for (int i = kLow; i <= kHigh; ++i){
        float d = logmag[i] - g_prev_logmag[i];
        if (d > 0.0f) sum += d;
        g_prev_logmag[i] = logmag[i];
    }
    return sum;
}

static float novelty_local_mean(float v){
    if (g_local_count < NOVELTY_WIN){
        g_local_mem[g_local_count++] = v;
        g_local_sum += v;
        return g_local_sum / (float)g_local_count + kNoveltyMeanOffset;
    }
    g_local_sum -= g_local_mem[g_local_idx];
    g_local_mem[g_local_idx] = v;
    g_local_sum += v;
    g_local_idx = (g_local_idx + 1) % NOVELTY_WIN;
    return g_local_sum / (float)NOVELTY_WIN + kNoveltyMeanOffset;
}

// ------------------- Novelty history + display -------------------
static void push_novelty(float raw, float local_mean){
    // Canonical cleaned novelty: raw - mean, clamped to zero
    float cleaned = raw - local_mean;
    if (cleaned < 0.0f) cleaned = 0.0f;

    // Full-resolution history (up to NOV_RING_FRAMES, ~8 s)
    g_novelty_ring[g_nov_write_pos] = cleaned;
    g_nov_write_pos = (g_nov_write_pos + 1) % NOV_RING_FRAMES;

    // Display history (kept in render module)
    fft_render_push_novelty_display(raw, local_mean, cleaned);

    g_nov_total_frames++;
}

// ------------------- Comb-based BPM spectrum from novelty -------------------
static void comb_bpm_from_novelty(const float *nov, int N, float frameRate,
                                  float *bpmSpec){
    const int bpmCount = BPM_MAX - BPM_MIN + 1;
    const float minSamplesPerBpm = 2.0f;

    if (N <= 1){
        for (int i = 0; i < bpmCount; ++i){
            bpmSpec[i]  = 0.0f;
        }
        return;
    }

    for (int idx = 0; idx < bpmCount; ++idx){
        int bpm = BPM_MIN + idx;
        float step = frameRate * 60.0f / (float)bpm;

        int maxPhase = (int)floorf(step);
        if (maxPhase < 1) maxPhase = 1;

        float bestScore = 0.0f;

        for (int phase = 0; phase < maxPhase; ++phase){
            float pos = (float)phase;
            float sum = 0.0f;
            int   count = 0;

            while (pos < (float)(N - 1)){
                int j = (int)floorf(pos);
                float t = pos - (float)j;

                float v;
                if (j >= 0 && j + 1 < N){
                    v = nov[j] * (1.0f - t) + nov[j + 1] * t;
                } else if (j >= 0 && j < N){
                    v = nov[j];
                } else {
                    v = 0.0f;
                }

                sum   += v;
                count += 1;
                pos   += step;
            }

            if ((float)count < minSamplesPerBpm) continue;

            float avg   = sum / (float)count;
            float score = avg * (float)count;

            if (score > bestScore){
                bestScore = score;
            }
        }

        bpmSpec[idx] = bestScore;
    }
}

// Recompute comb phase using the exact selected BPM spacing.
static void recompute_phase_for_bpm(const float *nov, int N, float frameRate, float bpm,
                                    float *phaseNormOut, float *phaseCurveOut, uint8_t *phaseCurveLenOut){
    if (phaseNormOut) *phaseNormOut = 0.0f;
    if (phaseCurveLenOut) *phaseCurveLenOut = 0;
    if (!nov || N <= 1 || frameRate <= 0.0f || bpm <= 0.0f){
        return;
    }

    float step = frameRate * 60.0f / bpm;
    int phaseCount = (int)floorf(step);
    if (phaseCount < 1) phaseCount = 1;
    if (phaseCount > MAX_PHASE_EVAL) phaseCount = MAX_PHASE_EVAL;

    const float minSamplesPerPhase = 2.0f;
    float phaseScores[MAX_PHASE_EVAL];
    for (int i = 0; i < phaseCount; ++i){
        phaseScores[i] = 0.0f;
    }

    float bestScore = 0.0f;
    int bestPhaseIdx = 0;

    for (int phase = 0; phase < phaseCount; ++phase){
        float pos = (float)phase;
        float sum = 0.0f;
        int count = 0;

        while (pos < (float)(N - 1)){
            int j = (int)floorf(pos);
            float t = pos - (float)j;
            float v;
            if (j >= 0 && j + 1 < N){
                v = nov[j] * (1.0f - t) + nov[j + 1] * t;
            } else if (j >= 0 && j < N){
                v = nov[j];
            } else {
                v = 0.0f;
            }
            sum += v;
            count += 1;
            pos += step;
        }

        if ((float)count < minSamplesPerPhase){
            phaseScores[phase] = 0.0f;
            continue;
        }

        phaseScores[phase] = sum;
        if (sum > bestScore){
            bestScore = sum;
            bestPhaseIdx = phase;
        }
    }

    if (phaseNormOut && bestScore > 0.0f && phaseCount > 1){
        *phaseNormOut = (float)bestPhaseIdx / (float)(phaseCount - 1);
    }

    if (!phaseCurveOut || !phaseCurveLenOut || bestScore <= 0.0f){
        return;
    }

    if (phaseCount == 1){
        for (int s = 0; s < PHASE_SLOTS; ++s){
            phaseCurveOut[s] = phaseScores[0];
        }
        *phaseCurveLenOut = PHASE_SLOTS;
        return;
    }

    for (int s = 0; s < PHASE_SLOTS; ++s){
        float p = ((float)s / (float)PHASE_SLOTS) * (float)phaseCount;
        while (p >= (float)phaseCount) p -= (float)phaseCount;

        int i0 = (int)floorf(p);
        int i1 = i0 + 1;
        if (i1 >= phaseCount) i1 = 0;
        float frac = p - (float)i0;

        float v0 = phaseScores[i0];
        float v1 = phaseScores[i1];
        phaseCurveOut[s] = v0 * (1.0f - frac) + v1 * frac;
    }
    *phaseCurveLenOut = PHASE_SLOTS;
}

static inline float bpm_bc_at(const float *bc, float bpm){
    if (bpm < (float)BPM_MIN || bpm > (float)BPM_MAX) return 0.0f;

    float idx = bpm - (float)BPM_MIN;
    int i0 = (int)floorf(idx);
    int i1 = i0 + 1;
    int maxIdx = BPM_MAX - BPM_MIN;

    if (i0 < 0) i0 = 0;
    if (i0 > maxIdx) i0 = maxIdx;
    if (i1 < 0) i1 = 0;
    if (i1 > maxIdx) i1 = maxIdx;

    float t = idx - (float)i0;
    return bc[i0] * (1.0f - t) + bc[i1] * t;
}

static void bpm_family_spectrum_from_bc(const float *bc, int bpmCount, float *famOut){
    const float wHalf = 0.50f; // requested 1/2 BPM gather term
    const float w1    = 1.00f;
    const float w2    = 0.65f;
    const float w4    = 0.45f;

    for (int i = 0; i < bpmCount; ++i){
        float F = (float)(BPM_MIN + i);
        float s = 0.0f;

        s += wHalf * bpm_bc_at(bc, 0.5f * F);
        s += w1    * bpm_bc_at(bc, F);
        s += w2    * bpm_bc_at(bc, 2.0f * F);
        s += w4    * bpm_bc_at(bc, 4.0f * F);

        famOut[i] = s;
    }
}

static inline float alpha_from_tau(float dt, float tau){
    if (tau <= 1e-6f) return 1.0f;
    return 1.0f - expf(-dt / tau);
}

static void smooth_spectrum_asym_tau(const float *in, float *state, int n,
                                     float dt, float tauUp, float tauDown){
    float aUp = alpha_from_tau(dt, tauUp);
    float aDown = alpha_from_tau(dt, tauDown);

    for (int i = 0; i < n; ++i){
        float x = in[i];
        float y = state[i];
        float a = (x > y) ? aUp : aDown;
        state[i] = (1.0f - a) * y + a * x;
    }
}

// ------------------- BPM update from novelty -------------------
static void update_bpm_from_novelty(void){
    const int bpmCount = BPM_MAX - BPM_MIN + 1;
    if (g_nov_total_frames == 0){
        g_bpm_conf *= 0.98f;
        return;
    }

    // 1) How many novelty frames do we have?
    int history_len = (g_nov_total_frames < (uint64_t)NOV_RING_FRAMES)
                        ? (int)g_nov_total_frames
                        : NOV_RING_FRAMES;

    // 2) Build contiguous novelty history oldest→newest
    static EXT_RAM_BSS_ATTR float nov_history[NOV_RING_FRAMES];
    int start = g_nov_write_pos - history_len;
    while (start < 0) start += NOV_RING_FRAMES;
    for (int i = 0; i < history_len; ++i){
        int src = (start + i) % NOV_RING_FRAMES;
        nov_history[i] = g_novelty_ring[src];
    }

    // 3) Run comb BPM search (use nominal hop rate to match desktop logic)
    const float frameRate = kHopRate_nominal;
    static EXT_RAM_BSS_ATTR float bpmSpec[BPM_MAX - BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmSpecBC[BPM_MAX - BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmFamily[BPM_MAX - BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmWindowAvg[BPM_MAX - BPM_MIN + 1];

    comb_bpm_from_novelty(nov_history, history_len, frameRate, bpmSpec);

    // 4) Baseline-correct comb spectrum (like desktop bpm panel)
    int mid = bpmCount / 2;
    float lowSum = 0.0f, highSum = 0.0f;
    for (int i = 0; i < mid; ++i)        lowSum  += bpmSpec[i];
    for (int i = mid; i < bpmCount; ++i) highSum += bpmSpec[i];

    float lowAvg  = (mid > 0) ? (lowSum  / (float)mid)           : 0.0f;
    float highAvg = (bpmCount - mid > 0) ? (highSum / (float)(bpmCount - mid)) : 0.0f;

    int x1 = mid / 2;
    int x2 = mid + (bpmCount - mid) / 2;
    if (x2 == x1){ x1 = 0; x2 = bpmCount - 1; }

    float slope     = (x2 != x1) ? ((highAvg - lowAvg) / (float)(x2 - x1)) : 0.0f;
    float intercept = lowAvg - slope * (float)x1;

    for (int i = 0; i < bpmCount; ++i){
        float base = intercept + slope * (float)i;
        float v    = bpmSpec[i] - base;
        if (v < 0.0f) v = 0.0f;
        bpmSpecBC[i] = v;
    }

    bpm_family_spectrum_from_bc(bpmSpecBC, bpmCount, bpmFamily);

    float dt = 1.0f / frameRate;
    smooth_spectrum_asym_tau(bpmFamily, g_family_spec_smooth, bpmCount,
                             dt, kFamilyTauUpSec, kFamilyTauDownSec);

    int next_count = (g_bpm_score_hist_count < BPM_SCORE_WINDOW_UPDATES)
                        ? (g_bpm_score_hist_count + 1)
                        : BPM_SCORE_WINDOW_UPDATES;
    for (int i = 0; i < bpmCount; ++i){
        float v = bpmFamily[i];
        if (v < 0.0f) v = 0.0f;
        float old = (g_bpm_score_hist_count < BPM_SCORE_WINDOW_UPDATES)
                    ? 0.0f
                    : g_bpm_score_hist[g_bpm_score_hist_idx][i];
        g_bpm_score_hist[g_bpm_score_hist_idx][i] = v;
        g_bpm_score_sum[i] += v - old;
        bpmWindowAvg[i] = g_bpm_score_sum[i] / (float)next_count;
    }
    g_bpm_score_hist_idx = (g_bpm_score_hist_idx + 1) % BPM_SCORE_WINDOW_UPDATES;
    g_bpm_score_hist_count = next_count;

    // Render the same windowed score map used by BPM picking.
    fft_render_update_tempo_spectrum(bpmWindowAvg);

    int targetMinIdx = TARGET_BPM_MIN - BPM_MIN;
    int targetMaxIdx = TARGET_BPM_MAX - BPM_MIN;
    if (targetMinIdx < 0) targetMinIdx = 0;
    if (targetMaxIdx >= bpmCount) targetMaxIdx = bpmCount - 1;
    if (targetMaxIdx < targetMinIdx){
        bpm_soft_decay_and_reset();
        return;
    }

    float target_energy = 0.0f;
    for (int i = targetMinIdx; i <= targetMaxIdx; ++i){
        float v = g_bpm_score_sum[i];
        if (v > 0.0f) target_energy += v;
    }
    if (target_energy <= 1e-6f){
        bpm_soft_decay_and_reset();
        return;
    }

    // Pick BPM from maximum adjacent-bin pair to handle true tempo between bins.
    int pair_best_idx = targetMinIdx;
    float pair_best_score = 0.0f;

    if (targetMaxIdx > targetMinIdx){
        for (int i = targetMinIdx; i < targetMaxIdx; ++i){
            float s0 = g_bpm_score_sum[i];
            float s1 = g_bpm_score_sum[i + 1];
            if (s0 < 0.0f) s0 = 0.0f;
            if (s1 < 0.0f) s1 = 0.0f;
            float pair_score = s0 + s1;
            if (pair_score > pair_best_score){
                pair_best_score = pair_score;
                pair_best_idx = i;
            }
        }
    } else {
        float s0 = g_bpm_score_sum[targetMinIdx];
        if (s0 < 0.0f) s0 = 0.0f;
        pair_best_score = s0;
    }

    if (pair_best_score < kGlobalCombMin){
        bpm_soft_decay_and_reset();
        return;
    }

    float s_pair_0 = g_bpm_score_sum[pair_best_idx];
    float s_pair_1 = (pair_best_idx + 1 <= targetMaxIdx) ? g_bpm_score_sum[pair_best_idx + 1] : 0.0f;
    if (s_pair_0 < 0.0f) s_pair_0 = 0.0f;
    if (s_pair_1 < 0.0f) s_pair_1 = 0.0f;
    float pair_mass = s_pair_0 + s_pair_1;

    float bpm0 = (float)(BPM_MIN + pair_best_idx);
    float bpm1 = (float)(BPM_MIN + pair_best_idx + 1);
    float fund_inst = bpm0;
    if (pair_best_idx + 1 <= targetMaxIdx && pair_mass > 1e-6f){
        fund_inst = (bpm0 * s_pair_0 + bpm1 * s_pair_1) / pair_mass;
    }

    // Cluster energy: expand from winning pair while scores strictly decrease outward.
    int cluster_l = pair_best_idx;
    int cluster_r = (pair_best_idx + 1 <= targetMaxIdx) ? (pair_best_idx + 1) : pair_best_idx;

    while (cluster_l > targetMinIdx){
        float inner = g_bpm_score_sum[cluster_l];
        float outer = g_bpm_score_sum[cluster_l - 1];
        if (inner < 0.0f) inner = 0.0f;
        if (outer < 0.0f) outer = 0.0f;
        if (outer < inner){
            cluster_l--;
        } else {
            break;
        }
    }
    while (cluster_r < targetMaxIdx){
        float inner = g_bpm_score_sum[cluster_r];
        float outer = g_bpm_score_sum[cluster_r + 1];
        if (inner < 0.0f) inner = 0.0f;
        if (outer < 0.0f) outer = 0.0f;
        if (outer < inner){
            cluster_r++;
        } else {
            break;
        }
    }

    float cluster_energy = 0.0f;
    for (int i = cluster_l; i <= cluster_r; ++i){
        float v = g_bpm_score_sum[i];
        if (v > 0.0f) cluster_energy += v;
    }

    float fund_avg  = update_fund_history(fund_inst);
    if (fund_avg <= 0.0f){
        bpm_soft_decay_and_reset();
        return;
    }

    const float kBpmTol = 3.0f;
    float diff = fabsf(fund_avg - g_last_cand_bpm);
    if (g_last_cand_bpm > 0.0f && diff < kBpmTol){
        g_bpm_stable_frames++;
    } else {
        g_bpm_stable_frames = 1;
    }
    g_last_cand_bpm = fund_avg;

    // Confidence is mass ratio of winning monotonic cluster vs all target-bin energy.
    float combined_raw_conf = cluster_energy / (target_energy + 1e-6f);
    if (combined_raw_conf < 0.0f) combined_raw_conf = 0.0f;
    if (combined_raw_conf > 1.0f) combined_raw_conf = 1.0f;
    g_bpm_conf = 0.9f * g_bpm_conf + 0.1f * combined_raw_conf;

    g_bpm_hz = fund_avg / 60.0f;

    // Recompute phase from comb offsets at the exact BPM selected for blink driving.
    float base_phase = 0.0f;
    g_phase_curve_len = 0;
    recompute_phase_for_bpm(nov_history, history_len, frameRate, fund_avg,
                            &base_phase, g_phase_curve, &g_phase_curve_len);

    double elapsed_sec = (frameRate > 0.0f)
                            ? ((double)g_nov_total_frames / (double)frameRate)
                            : 0.0;
    float phase_shift = (float)fmod(elapsed_sec * (double)g_bpm_hz, 1.0);
    g_bpm_phase = wrap_phase01(base_phase + phase_shift);

    g_bpm_list_count = 0;
    if (g_bpm_conf >= kOverallMinConf){
        g_bpm_list[0] = fund_avg;
        g_bpm_conf_list[0] = g_bpm_conf;
        g_bpm_list_count = 1;
    }

    if (g_phase_curve_len > 0){
        // Keep phase-comb view aligned with the exported compensated phase.
        shift_phase_curve_circular(g_phase_curve, g_phase_curve_len, phase_shift);
    }
    fft_render_update_phase_curve(
        (g_phase_curve_len > 0) ? g_phase_curve : NULL,
        g_phase_curve_len);
}

// ------------------- Processing per FFT frame -------------------
static void process_fft_frame(void){
    int64_t t0 = esp_timer_get_time();
    float frame_dt_sec = 1.0f / kHopRate_nominal;
    // Update hop rate estimate using high-resolution timer
    if (g_last_frame_time_us != 0){
        int64_t dt_us = t0 - g_last_frame_time_us;
        if (dt_us > 500 && dt_us < 500000){ // clamp to reasonable range
            float inst_rate = 1e6f / (float)dt_us;
            g_hop_rate_hz_est = 0.9f * g_hop_rate_hz_est + 0.1f * inst_rate;
            frame_dt_sec = (float)dt_us * 1e-6f;
        }
    }
    g_last_frame_time_us = t0;

    // --------- FFT magnitude (full-resolution, used for all DSP) ---------
    float logmag[FFT_SIZE/2 + 1];

    for (int i = 0; i < FFT_SIZE; ++i){
        g_fft_re[i] = g_fft_buf[i] * g_window[i];
        g_fft_im[i] = 0.0f;
    }
    fft_radix2(g_fft_re, g_fft_im, FFT_SIZE);

    for (int k = 0; k <= FFT_SIZE/2; ++k){
        float r  = g_fft_re[k];
        float im = g_fft_im[k];
        float mag = sqrtf(r*r + im*im);
        logmag[k] = logf(1.0f + 10.0f * mag);
    }

    fft_render_push_spectrogram(logmag);

    float nov   = compute_novelty(logmag);

    // Spike protection: drop absurd novelty bursts from physical bumps/taps
    if (nov > kNoveltyZeroThreshold) {
        nov = 0.0f;
    }
    apply_novelty_backfill_hole();
    if (g_nov_hole_future_frames > 0){
        nov = 0.0f;
        g_nov_hole_future_frames--;
    }

    float local = novelty_local_mean(nov);
    push_novelty(nov, local);

    if (g_tempo_update_countdown <= 0){
        update_bpm_from_novelty();
        g_tempo_update_countdown = kTempoUpdateIntervalFrames;
    } else {
        g_tempo_update_countdown--;
    }

    // --------- Beat blinker ---------
    // Match blinker to the currently found BPM shown to the user.
    float target_bpm_hz = (g_last_cand_bpm > 0.5f) ? (g_last_cand_bpm / 60.0f) : 0.0f;
    bool beat_triggered = false;

    if (target_bpm_hz > 0.5f) {
        g_blink_bpm_hz = target_bpm_hz;
        g_blink_rate_hz = target_bpm_hz;
        if (g_blink_rate_hz < 0.1f) g_blink_rate_hz = 0.1f;

        g_beat_phase += g_blink_rate_hz * frame_dt_sec;
        if (g_beat_phase >= 1.0f) {
            g_beat_phase -= floorf(g_beat_phase);
            fft_render_trigger_flash(kFlashFrames);
            beat_triggered = true;
        }
    } else {
        g_blink_bpm_hz  = 0.0f;
        g_blink_rate_hz = 0.0f;
        g_beat_phase    = 0.0f;
    }

    if (beat_triggered && s_beat_queue){
        fft_beat_event_t evt = {
            .bpm = g_blink_bpm_hz * 60.0f,
            .confidence = g_bpm_conf
        };
        xQueueSendToBack(s_beat_queue, &evt, 0);
    }
    if (beat_triggered){
        // Fixed orange pulse avoids hue flicker when confidence jitters.
        uint8_t red   = 255;
        uint8_t green = 96;
        led_trigger_beat(red, green, 0);
    }

    fft_render_packet_t pkt = {
        .bpm_est   = g_last_cand_bpm,
        .bpm_conf  = g_bpm_conf,
        .bpm_phase = g_bpm_phase
    };
    memcpy(pkt.logmag, logmag, sizeof(pkt.logmag));
    fft_render_submit(&pkt);

    int64_t dt_us = esp_timer_get_time() - t0;
    g_prof_accum_us += (uint64_t)dt_us;
    g_prof_frame_count++;
    if (g_prof_frame_count >= (int)(kHopRate_nominal)) {
        float avg_us = (float)g_prof_accum_us / (float)g_prof_frame_count;
        float hop_us = (g_hop_rate_hz_est > 1.0f) ? (1e6f / g_hop_rate_hz_est) : (1e6f / kHopRate_nominal);
        g_last_load_pct = (hop_us > 0.0f) ? (avg_us / hop_us * 100.0f) : 0.0f;
        ESP_LOGI(TAG, "FFT frame avg %.2f ms (%.1f%% of hop), hop_est=%.1f Hz",
                 avg_us / 1000.0f, g_last_load_pct, (double)g_hop_rate_hz_est);
        g_prof_accum_us = 0;
        g_prof_frame_count = 0;
    }
}

// ------------------- Task -------------------
static TaskHandle_t s_task=NULL;
static volatile bool s_stop_requested = false;

#if CONFIG_FREERTOS_UNICORE
#define FFT_TASK_CORE 0
#else
#define FFT_TASK_CORE 1
#endif
#define FFT_TASK_PRIO 4

static void fft_task(void *arg){
    i2s_chan_handle_t rx = audio_rx_handle();
    if (!rx){
        ESP_LOGE(TAG,"No RX handle");
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    for(int n=0;n<FFT_SIZE;++n){
        g_window[n]=0.5f*(1.0f - cosf((2.0f*(float)M_PI*n)/(float)(FFT_SIZE-1)));
    }
    const size_t in_bytes = HOP_SAMPLES * sizeof(int32_t) * 2;
    int32_t *rx32 = (int32_t*)malloc(in_bytes);
    if (!rx32){
        ESP_LOGE(TAG,"OOM");
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    int slot_index=0; bool slot_decided=false;

    while(!s_stop_requested){
        size_t nread=0;
        esp_err_t e = i2s_channel_read(rx, rx32, in_bytes, &nread, pdMS_TO_TICKS(200));
        if (e!=ESP_OK || nread==0){ vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        size_t frames_rd = nread / (sizeof(int32_t)*2);
        if (!slot_decided && frames_rd>=8){
            uint64_t sumL=0,sumR=0;
            for(size_t i=0;i<frames_rd;++i){
                sumL += llabs((long long)rx32[2*i+0]);
                sumR += llabs((long long)rx32[2*i+1]);
            }
            slot_index = (sumR>sumL)?1:0;
            slot_decided=true;
            ESP_LOGI(TAG,"Mic slot: %s", slot_index?"RIGHT":"LEFT");
        }
        for(size_t i=0;i<frames_rd;++i){
            int32_t v32 = rx32[2*i + slot_index];
            int16_t v16 = (int16_t)(v32 >> 14);
            g_hop_buf[g_hop_fill++] = (float)v16 / 32768.0f;
            if (g_hop_fill >= HOP_SAMPLES){
                if (g_fft_fill + HOP_SAMPLES > FFT_SIZE){
                    memmove(g_fft_buf, g_fft_buf + HOP_SAMPLES, (FFT_SIZE - HOP_SAMPLES)*sizeof(float));
                    g_fft_fill -= HOP_SAMPLES;
                }
                memcpy(g_fft_buf + g_fft_fill, g_hop_buf, HOP_SAMPLES*sizeof(float));
                g_fft_fill += HOP_SAMPLES;
                g_hop_fill = 0;
                if (g_fft_fill >= FFT_SIZE){
                    process_fft_frame();
                    taskYIELD();
                }
            }
        }
    }
    free(rx32);
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t fft_visualizer_start(void){
    if (s_task) return ESP_OK;
    s_stop_requested = false;
    ESP_RETURN_ON_ERROR(audio_set_rate(SAMPLE_RATE_HZ), TAG, "set fs");
    ESP_RETURN_ON_ERROR(audio_enable_rx(), TAG, "enable rx");
    ESP_RETURN_ON_ERROR(led_init(), TAG, "led init");
    ESP_RETURN_ON_ERROR(fft_render_init(), TAG, "render init");
    if (!s_beat_queue){
        s_beat_queue = xQueueCreate(4, sizeof(fft_beat_event_t));
        if (!s_beat_queue) return ESP_ERR_NO_MEM;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(fft_task, "fft_vis", 12288, NULL,
                                            FFT_TASK_PRIO, &s_task, FFT_TASK_CORE);
#if CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    if (ok != pdPASS) {
        ok = xTaskCreatePinnedToCoreWithCaps(fft_task, "fft_vis", 12288, NULL,
                                             FFT_TASK_PRIO, &s_task, FFT_TASK_CORE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
#endif
    return ok==pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void fft_visualizer_stop(void){
    if (!s_task) return;
    s_stop_requested = true;
    for (int i = 0; i < 60 && s_task; ++i){
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_task){
        ESP_LOGW(TAG, "forcing fft task stop");
        vTaskDelete(s_task);
        s_task = NULL;
    }
    s_stop_requested = false;
}

void fft_visualizer_set_view(fft_view_t view){
    fft_render_set_view(view);
}

bool fft_receive_beat(fft_beat_event_t *evt, TickType_t timeout_ticks){
    if (!evt || !s_beat_queue) return false;
    return xQueueReceive(s_beat_queue, evt, timeout_ticks) == pdTRUE;
}

void fft_set_display_enabled(bool enabled){
    fft_render_set_display_enabled(enabled);
}

// ------------------- Public API: get list of confident BPMs -------------------
int fft_get_confident_bpms(float *bpm_out, float *conf_out, int max_out){
    if (!bpm_out || !conf_out || max_out <= 0) return 0;

    int n = g_bpm_list_count;
    if (n > max_out) n = max_out;

    for (int i = 0; i < n; ++i){
        bpm_out[i]  = g_bpm_list[i];
        conf_out[i] = g_bpm_conf_list[i];
    }
    return n;
}
