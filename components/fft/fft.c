#include "fft.h"
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
static const float kHopRateNominalHz = (float)FFT_CFG_SAMPLE_RATE_HZ / (float)FFT_CFG_HOP_SAMPLES; // 62.5 Hz
// Recompute tempo roughly twice/second (ceil keeps update rate <= 2 Hz).
static const int   kTempoUpdateIntervalFrames = (FFT_CFG_SAMPLE_RATE_HZ + (2 * FFT_CFG_HOP_SAMPLES) - 1) / (2 * FFT_CFG_HOP_SAMPLES);
static const int   kLoadReportWindowFrames = (FFT_CFG_SAMPLE_RATE_HZ + (FFT_CFG_HOP_SAMPLES / 2)) / FFT_CFG_HOP_SAMPLES; // ~1 sec
static const float kBlinkPhaseTarget = 0.85f; // trigger blink when beat phase reaches this point
#define BPM_SCORE_WINDOW_UPDATES 8 // ~4s at 2 tempo updates/sec
#define PHASE_SLOTS     32     // downsampled phase bins for display

#if ((FFT_CFG_SIZE & (FFT_CFG_SIZE - 1)) != 0)
#error "FFT_CFG_SIZE must be a power of two for ring-buffer indexing"
#endif

#if ((FFT_CFG_HOP_SAMPLES & (FFT_CFG_HOP_SAMPLES - 1)) != 0)
#error "FFT_CFG_HOP_SAMPLES must be a power of two for hop-boundary indexing"
#endif

// ------------------- DSP tuning -------------------
static const float kGlobalCombMin        = 0.01f;
static const float kNoveltyZeroThreshold = 400.0f;
static const float kOverallMinConf       = 0.25f;
static const float kBlinkMinConf         = 0.25f;
static const float kFamilyTauUpSec       = 0.35f;
static const float kFamilyTauDownSec     = 1.20f;
#define MAX_PHASE_EVAL 256

// ------------------- State -------------------
static size_t  ring_write_pos = 0; // next sample slot to write
static EXT_RAM_BSS_ATTR float sample_ring[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float window[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float fft_re[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float fft_im[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float prev_logmag[FFT_CFG_SIZE/2+1];
static bool    prev_logmag_valid=false;
static float   last_cand_bpm     = 0.0f; // last candidate BPM from spectrum
static int     bpm_stable_frames = 0;    // how many consecutive frames it's stayed similar

// novelty history (cleaned novelty stored in novelty_ring)
static EXT_RAM_BSS_ATTR float   novelty_ring[FFT_CFG_NOV_RING_FRAMES];
static int     nov_write_pos=0; // next slot to overwrite (also oldest sample)
static uint64_t nov_total_frames=0;

static EXT_RAM_BSS_ATTR float   local_mem[FFT_CFG_NOVELTY_WIN];
static int     local_idx=0;
static int     local_count=0;
static float   local_sum=0.0f;

static uint64_t prof_accum_us = 0;     // accumulated processing time in microseconds
static int      prof_frame_count = 0;  // frames counted in the accumulator
static float    last_load_pct = 0.0f;
static float    hop_rate_hz_est = kHopRateNominalHz;   // immediate hop-rate estimate from timer
static int64_t  last_frame_time_us = 0;
static float    bpm_hz = 0.0f;
static float    bpm_conf = 0.0f;
static float    bpm_phase = 0.0f;
static float    phase_shift_accum = 0.0f; // continuous phase advance (prevents retroactive jumps)
static float    blink_bpm_hz = 0.0f;  // latched BPM that drives the blinker until next beat
static float    blink_rate_hz = 0.0f; // actual blink rate used for display
static float    beat_phase = 0.0f;    // phase accumulator for main beat blinker
static int      tempo_update_countdown = 0;
static int      tempo_update_hold = 0; // tempo update cycles to skip after novelty suppression
static QueueHandle_t s_beat_queue = NULL;
static volatile int nov_suppress_backfill_frames = 0;  // recent frames to suppress
static volatile int nov_suppress_future_frames = 0;    // upcoming frames to suppress

// ------------------- BPM export and smoothing -------------------
#define MAX_BPM_EXPORT  3      // export at most 3 BPMs (fundamental, double, special)
#define BPM_AVG_FRAMES  6      // rolling average length for main BPM
static float bpm_list[MAX_BPM_EXPORT];
static float bpm_conf_list[MAX_BPM_EXPORT];
static int   bpm_list_count = 0;

// history of the fundamental BPM per tempo-update frame
static EXT_RAM_BSS_ATTR float fund_hist[BPM_AVG_FRAMES];
static int   fund_hist_pos   = 0;
static int   fund_hist_count = 0;
static EXT_RAM_BSS_ATTR float   phase_curve[PHASE_SLOTS];
static uint8_t phase_curve_len = 0;
static EXT_RAM_BSS_ATTR float family_spec_smooth[FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];
static EXT_RAM_BSS_ATTR float bpm_score_hist[BPM_SCORE_WINDOW_UPDATES][FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];
static EXT_RAM_BSS_ATTR float bpm_score_sum[FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];
static int bpm_score_hist_idx = 0;
static int bpm_score_hist_count = 0;

static void fft_reset_runtime_state(void){
    ring_write_pos = 0;
    memset(sample_ring, 0, sizeof(sample_ring));

    memset(prev_logmag, 0, sizeof(prev_logmag));
    prev_logmag_valid = false;
    last_cand_bpm = 0.0f;
    bpm_stable_frames = 0;

    memset(novelty_ring, 0, sizeof(novelty_ring));
    nov_write_pos = 0;
    nov_total_frames = 0;

    memset(local_mem, 0, sizeof(local_mem));
    local_idx = 0;
    local_count = 0;
    local_sum = 0.0f;

    prof_accum_us = 0;
    prof_frame_count = 0;
    last_load_pct = 0.0f;
    hop_rate_hz_est = kHopRateNominalHz;
    last_frame_time_us = 0;

    bpm_hz = 0.0f;
    bpm_conf = 0.0f;
    bpm_phase = 0.0f;
    phase_shift_accum = 0.0f;
    blink_bpm_hz = 0.0f;
    blink_rate_hz = 0.0f;
    beat_phase = 0.0f;
    tempo_update_countdown = 0;
    tempo_update_hold = 0;
    nov_suppress_backfill_frames = 0;
    nov_suppress_future_frames = 0;

    memset(bpm_list, 0, sizeof(bpm_list));
    memset(bpm_conf_list, 0, sizeof(bpm_conf_list));
    bpm_list_count = 0;

    memset(fund_hist, 0, sizeof(fund_hist));
    fund_hist_pos = 0;
    fund_hist_count = 0;

    memset(phase_curve, 0, sizeof(phase_curve));
    phase_curve_len = 0;

    memset(family_spec_smooth, 0, sizeof(family_spec_smooth));
    memset(bpm_score_hist, 0, sizeof(bpm_score_hist));
    memset(bpm_score_sum, 0, sizeof(bpm_score_sum));
    bpm_score_hist_idx = 0;
    bpm_score_hist_count = 0;
}

static inline float wrap_phase01(float phase){ // Wrap phase to [0,1)
    phase -= floorf(phase);
    if (phase < 0.0f) phase += 1.0f;
    return phase;
}

static inline bool phase_crossed_forward(float prev, float curr, float target){ // Check if phase crossed target in forward direction (handles wraparound)
    prev = wrap_phase01(prev);
    curr = wrap_phase01(curr);
    target = wrap_phase01(target);

    if (curr >= prev){
        return (target > prev && target <= curr);
    }
    return (target > prev || target <= curr);
}

static void shift_phase_curve_circular(float *vals, uint8_t count, float shift_norm){ // Shift a phase curve by shift_norm (0..1) with circular wraparound and linear interpolation. Modifies vals in-place.
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

// Rolling average of recent fundamental estimates
static float update_fund_history(float bpm_inst){
    if (bpm_inst <= 0.0f) return 0.0f;

    fund_hist[fund_hist_pos] = bpm_inst;
    fund_hist_pos = (fund_hist_pos + 1) % BPM_AVG_FRAMES;
    if (fund_hist_count < BPM_AVG_FRAMES) fund_hist_count++;

    float sum = 0.0f;
    for (int f = 0; f < fund_hist_count; ++f){
        int idx = (fund_hist_pos - 1 - f + BPM_AVG_FRAMES) % BPM_AVG_FRAMES;
        sum += fund_hist[idx];
    }
    return sum / (float)fund_hist_count;
}

static void bpm_soft_decay_and_reset(void){
    bpm_conf *= 0.9f;
    last_cand_bpm = 0.0f;
    bpm_stable_frames = 0;
    fund_hist_count = 0;
    fund_hist_pos   = 0;
    bpm_list_count = 0;
    bpm_hz    = 0.0f;
    bpm_phase = 0.0f;
    phase_shift_accum = 0.0f;
    phase_curve_len = 0;
    bpm_score_hist_idx = 0;
    bpm_score_hist_count = 0;
    blink_bpm_hz = 0.0f;
    blink_rate_hz = 0.0f;
    beat_phase = 0.0f;
    tempo_update_hold = 0;
    memset(family_spec_smooth, 0, sizeof(family_spec_smooth));
    memset(bpm_score_hist, 0, sizeof(bpm_score_hist));
    memset(bpm_score_sum, 0, sizeof(bpm_score_sum));
}

static void reset_novelty_baseline_state(void){
    local_idx = 0;
    local_count = 0;
    local_sum = 0.0f;
    memset(local_mem, 0, sizeof(local_mem));
    prev_logmag_valid = false;
}

static void apply_novelty_suppression_to_recent_history(void){
    int frames = nov_suppress_backfill_frames;
    if (frames <= 0) return;
    nov_suppress_backfill_frames = 0;

    int history_len = (nov_total_frames < (uint64_t)FFT_CFG_NOV_RING_FRAMES)
                        ? (int)nov_total_frames
                        : FFT_CFG_NOV_RING_FRAMES;
    if (history_len <= 0) return;
    if (frames > history_len) frames = history_len;

    // Clear the most recent history so suppression reaches back to the
    // voltage-change time reported by input debounce logic.
    for (int i = 0; i < frames; ++i){
        int idx = nov_write_pos - 1 - i;
        while (idx < 0) idx += FFT_CFG_NOV_RING_FRAMES;
        novelty_ring[idx] = 0.0f;
    }
    fft_render_suppress_recent_novelty(frames);

    // Suppression must affect downstream novelty logic too, not only history arrays.
    reset_novelty_baseline_state();

    // Do not reset BPM/phase on button-hole punching: keep the current phase lock.
    // Instead, skip a few tempo re-estimates so damaged novelty frames don't jerk phase.
    int total_supp_frames = frames + nov_suppress_future_frames;
    int hold_updates = (total_supp_frames + kTempoUpdateIntervalFrames - 1) / kTempoUpdateIntervalFrames;
    hold_updates += 1; // extra guard update after suppression window
    if (hold_updates > 10) hold_updates = 10;
    if (hold_updates > tempo_update_hold){
        tempo_update_hold = hold_updates;
    }
    if (tempo_update_countdown <= 0){
        tempo_update_countdown = kTempoUpdateIntervalFrames;
    }
}

void fft_suppress_novelty_frames(int frames){
    if (frames < 1) frames = 1;
    if (frames > FFT_CFG_NOV_RING_FRAMES) frames = FFT_CFG_NOV_RING_FRAMES;
    if (frames > nov_suppress_backfill_frames){
        nov_suppress_backfill_frames = frames;
    }
    if (frames > nov_suppress_future_frames){
        nov_suppress_future_frames = frames;
    }
}

static int novelty_frames_from_ms(int backfill_ms)
{
    if (backfill_ms <= 0) return 1;
    const int denom = FFT_CFG_HOP_SAMPLES * 1000;
    int64_t num = (int64_t)backfill_ms * (int64_t)FFT_CFG_SAMPLE_RATE_HZ;
    int frames = (int)((num + denom - 1) / denom); // ceil(ms / hop_ms)
    if (frames < 1) frames = 1;
    if (frames > FFT_CFG_NOV_RING_FRAMES) frames = FFT_CFG_NOV_RING_FRAMES;
    return frames;
}

void fft_suppress_novelty_timed_ms(int backfill_ms, int future_frames)
{
    int backfill_frames = novelty_frames_from_ms(backfill_ms);
    if (future_frames < 0) future_frames = 0;
    if (future_frames > FFT_CFG_NOV_RING_FRAMES) future_frames = FFT_CFG_NOV_RING_FRAMES;

    if (backfill_frames > nov_suppress_backfill_frames){
        nov_suppress_backfill_frames = backfill_frames;
    }
    if (future_frames > nov_suppress_future_frames){
        nov_suppress_future_frames = future_frames;
    }
}

// ------------------- Novelty computation -------------------

static float compute_novelty(const float *logmag){
    float sum = 0.0f;

    if (!prev_logmag_valid){
        memcpy(prev_logmag, logmag, sizeof(prev_logmag));
        prev_logmag_valid = true;
        return 0.0f;
    }

    // use band ~100 Hz .. Nyquist
    const float fs   = (float)FFT_CFG_SAMPLE_RATE_HZ;
    const int   nfft = FFT_CFG_SIZE;
    int kLow  = (int)ceilf(100.0f * (float)nfft / fs);
    int kHigh = (int)floorf((fs * 0.5f) * (float)nfft / fs); // ≈ Nyquist

    if (kLow  < 0)            kLow  = 0;
    if (kLow  > nfft/2)       kLow  = nfft/2;
    if (kHigh < 0)            kHigh = 0;
    if (kHigh > nfft/2)       kHigh = nfft/2;
    if (kHigh < kLow)         kHigh = kLow;

    for (int i = kLow; i <= kHigh; ++i){
        float d = logmag[i] - prev_logmag[i];
        if (d > 0.0f) sum += d;
        prev_logmag[i] = logmag[i];
    }
    return sum;
}

static float novelty_local_mean(float v){
    if (local_count < FFT_CFG_NOVELTY_WIN){
        local_mem[local_count++] = v;
        local_sum += v;
        return local_sum / (float)local_count + kNoveltyMeanOffset;
    }
    local_sum -= local_mem[local_idx];
    local_mem[local_idx] = v;
    local_sum += v;
    local_idx = (local_idx + 1) % FFT_CFG_NOVELTY_WIN;
    return local_sum / (float)FFT_CFG_NOVELTY_WIN + kNoveltyMeanOffset;
}

// ------------------- Novelty history + display -------------------
static void push_novelty(float raw, float local_mean){
    // Canonical cleaned novelty: raw - mean, clamped to zero
    float cleaned = raw - local_mean;
    if (cleaned < 0.0f) cleaned = 0.0f;

    // Full-resolution history (up to FFT_CFG_NOV_RING_FRAMES, ~6 s)
    novelty_ring[nov_write_pos] = cleaned;
    nov_write_pos = (nov_write_pos + 1) % FFT_CFG_NOV_RING_FRAMES;

    // Display history (kept in render module)
    fft_render_push_novelty_display(raw, local_mean, cleaned);

    nov_total_frames++;
}

// ------------------- Comb-based BPM spectrum from novelty -------------------
static void comb_bpm_from_novelty(const float *nov, int N, float frameRate,
                                  float *bpmSpec){
    const int bpmCount = FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1;
    const float minSamplesPerBpm = 2.0f;

    if (N <= 1){
        for (int i = 0; i < bpmCount; ++i){
            bpmSpec[i]  = 0.0f;
        }
        return;
    }

    for (int idx = 0; idx < bpmCount; ++idx){
        int bpm = FFT_CFG_BPM_MIN + idx;
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
    if (bpm < (float)FFT_CFG_BPM_MIN || bpm > (float)FFT_CFG_BPM_MAX) return 0.0f;

    float idx = bpm - (float)FFT_CFG_BPM_MIN;
    int i0 = (int)floorf(idx);
    int i1 = i0 + 1;
    int maxIdx = FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN;

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
        float F = (float)(FFT_CFG_BPM_MIN + i);
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
    const int bpmCount = FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1;
    if (nov_total_frames == 0){
        bpm_conf *= 0.98f;
        return;
    }

    // 1) How many novelty frames do we have?
    int history_len = (nov_total_frames < (uint64_t)FFT_CFG_NOV_RING_FRAMES)
                        ? (int)nov_total_frames
                        : FFT_CFG_NOV_RING_FRAMES;

    // 2) Build contiguous novelty history oldest→newest
    static EXT_RAM_BSS_ATTR float nov_history[FFT_CFG_NOV_RING_FRAMES];
    int start = nov_write_pos - history_len;
    while (start < 0) start += FFT_CFG_NOV_RING_FRAMES;
    for (int i = 0; i < history_len; ++i){
        int src = (start + i) % FFT_CFG_NOV_RING_FRAMES;
        nov_history[i] = novelty_ring[src];
    }

    // 3) Run comb BPM search (use nominal hop rate to match desktop logic)
    const float frameRate = kHopRateNominalHz;
    static EXT_RAM_BSS_ATTR float bpmSpec[FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmSpecBC[FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmFamily[FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmWindowAvg[FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1];

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
    smooth_spectrum_asym_tau(bpmFamily, family_spec_smooth, bpmCount,
                             dt, kFamilyTauUpSec, kFamilyTauDownSec);

    int next_count = (bpm_score_hist_count < BPM_SCORE_WINDOW_UPDATES)
                        ? (bpm_score_hist_count + 1)
                        : BPM_SCORE_WINDOW_UPDATES;
    for (int i = 0; i < bpmCount; ++i){
        float v = bpmFamily[i];
        if (v < 0.0f) v = 0.0f;
        float old = (bpm_score_hist_count < BPM_SCORE_WINDOW_UPDATES)
                    ? 0.0f
                    : bpm_score_hist[bpm_score_hist_idx][i];
        bpm_score_hist[bpm_score_hist_idx][i] = v;
        bpm_score_sum[i] += v - old;
        bpmWindowAvg[i] = bpm_score_sum[i] / (float)next_count;
    }
    bpm_score_hist_idx = (bpm_score_hist_idx + 1) % BPM_SCORE_WINDOW_UPDATES;
    bpm_score_hist_count = next_count;

    // Render windowed score map for display; BPM/phase picking uses score_map below.
    fft_render_update_tempo_spectrum(bpmWindowAvg);

    int targetMinIdx = FFT_CFG_TARGET_BPM_MIN - FFT_CFG_BPM_MIN;
    int targetMaxIdx = FFT_CFG_TARGET_BPM_MAX - FFT_CFG_BPM_MIN;
    const float *score_map = family_spec_smooth; // match temp.c BPM/phase selection map
    if (targetMinIdx < 0) targetMinIdx = 0;
    if (targetMaxIdx >= bpmCount) targetMaxIdx = bpmCount - 1;
    if (targetMaxIdx < targetMinIdx){
        bpm_soft_decay_and_reset();
        return;
    }

    float target_energy = 0.0f;
    for (int i = targetMinIdx; i <= targetMaxIdx; ++i){
        float v = score_map[i];
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
            float s0 = score_map[i];
            float s1 = score_map[i + 1];
            if (s0 < 0.0f) s0 = 0.0f;
            if (s1 < 0.0f) s1 = 0.0f;
            float pair_score = s0 + s1;
            if (pair_score > pair_best_score){
                pair_best_score = pair_score;
                pair_best_idx = i;
            }
        }
    } else {
        float s0 = score_map[targetMinIdx];
        if (s0 < 0.0f) s0 = 0.0f;
        pair_best_score = s0;
    }

    if (pair_best_score < kGlobalCombMin){
        bpm_soft_decay_and_reset();
        return;
    }

    float s_pair_0 = score_map[pair_best_idx];
    float s_pair_1 = (pair_best_idx + 1 <= targetMaxIdx) ? score_map[pair_best_idx + 1] : 0.0f;
    if (s_pair_0 < 0.0f) s_pair_0 = 0.0f;
    if (s_pair_1 < 0.0f) s_pair_1 = 0.0f;
    float pair_mass = s_pair_0 + s_pair_1;

    float bpm0 = (float)(FFT_CFG_BPM_MIN + pair_best_idx);
    float bpm1 = (float)(FFT_CFG_BPM_MIN + pair_best_idx + 1);
    float fund_inst = bpm0;
    if (pair_best_idx + 1 <= targetMaxIdx && pair_mass > 1e-6f){
        fund_inst = (bpm0 * s_pair_0 + bpm1 * s_pair_1) / pair_mass;
    }

    // Cluster energy: expand from winning pair while scores strictly decrease outward.
    int cluster_l = pair_best_idx;
    int cluster_r = (pair_best_idx + 1 <= targetMaxIdx) ? (pair_best_idx + 1) : pair_best_idx;

    while (cluster_l > targetMinIdx){
        float inner = score_map[cluster_l];
        float outer = score_map[cluster_l - 1];
        if (inner < 0.0f) inner = 0.0f;
        if (outer < 0.0f) outer = 0.0f;
        if (outer < inner){
            cluster_l--;
        } else {
            break;
        }
    }
    while (cluster_r < targetMaxIdx){
        float inner = score_map[cluster_r];
        float outer = score_map[cluster_r + 1];
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
        float v = score_map[i];
        if (v > 0.0f) cluster_energy += v;
    }

    float fund_avg  = update_fund_history(fund_inst);
    if (fund_avg <= 0.0f){
        bpm_soft_decay_and_reset();
        return;
    }

    const float kBpmTol = 3.0f;
    float diff = fabsf(fund_avg - last_cand_bpm);
    if (last_cand_bpm > 0.0f && diff < kBpmTol){
        bpm_stable_frames++;
    } else {
        bpm_stable_frames = 1;
    }
    last_cand_bpm = fund_avg;

    // Confidence is mass ratio of winning monotonic cluster vs all target-bin energy.
    float combined_raw_conf = cluster_energy / (target_energy + 1e-6f);
    if (combined_raw_conf < 0.0f) combined_raw_conf = 0.0f;
    if (combined_raw_conf > 1.0f) combined_raw_conf = 1.0f;
    bpm_conf = 0.9f * bpm_conf + 0.1f * combined_raw_conf;

    bpm_hz = fund_avg / 60.0f;

    // Recompute phase from comb offsets at the exact BPM selected for blink driving.
    float base_phase = 0.0f;
    phase_curve_len = 0;
    recompute_phase_for_bpm(nov_history, history_len, frameRate, fund_avg,
                            &base_phase, phase_curve, &phase_curve_len);

    // Use continuous phase integration instead of elapsed_time*bpm:
    // this avoids large visual jumps when BPM estimate nudges slightly.
    float phase_shift = phase_shift_accum;
    bpm_phase = wrap_phase01(base_phase + phase_shift);

    bpm_list_count = 0;
    if (bpm_conf >= kOverallMinConf){
        bpm_list[0] = fund_avg;
        bpm_conf_list[0] = bpm_conf;
        bpm_list_count = 1;
    }

    if (phase_curve_len > 0){
        // Keep phase-comb view aligned with the exported compensated phase.
        shift_phase_curve_circular(phase_curve, phase_curve_len, phase_shift);
    }
    fft_render_update_phase_curve(
        (phase_curve_len > 0) ? phase_curve : NULL,
        phase_curve_len);
}

// ------------------- Processing per FFT frame -------------------
static void process_fft_frame(void){
    int64_t t0 = esp_timer_get_time();
    float frame_dt_sec = 1.0f / kHopRateNominalHz;
    // Update hop rate estimate using high-resolution timer
    if (last_frame_time_us != 0){
        int64_t dt_us = t0 - last_frame_time_us;
        if (dt_us > 500 && dt_us < 500000){ // clamp to reasonable range
            float inst_rate = 1e6f / (float)dt_us;
            hop_rate_hz_est = 0.9f * hop_rate_hz_est + 0.1f * inst_rate;
            frame_dt_sec = (float)dt_us * 1e-6f;
        }
    }
    last_frame_time_us = t0;

    // Advance phase continuously with current BPM estimate.
    if (bpm_hz > 0.0f && frame_dt_sec > 0.0f){
        phase_shift_accum = wrap_phase01(phase_shift_accum + bpm_hz * frame_dt_sec);
    }

    // --------- FFT magnitude (full-resolution, used for all DSP) ---------
    float logmag[FFT_CFG_SIZE/2 + 1];
    const size_t ring_mask = FFT_CFG_SIZE - 1;
    const size_t ring_start = ring_write_pos; // oldest sample in the full ring

    for (int i = 0; i < FFT_CFG_SIZE; ++i){
        size_t ring_idx = (ring_start + (size_t)i) & ring_mask;
        fft_re[i] = sample_ring[ring_idx] * window[i];
        fft_im[i] = 0.0f;
    }
    fft_radix2(fft_re, fft_im, FFT_CFG_SIZE);

    for (int k = 0; k <= FFT_CFG_SIZE/2; ++k){
        float r  = fft_re[k];
        float im = fft_im[k];
        float mag = sqrtf(r*r + im*im);
        logmag[k] = logf(1.0f + 10.0f * mag);
    }

    fft_render_push_spectrogram(logmag);

    float nov   = compute_novelty(logmag);

    // Spike protection: drop absurd novelty bursts from physical bumps/taps
    if (nov > kNoveltyZeroThreshold) {
        nov = 0.0f;
    }
    apply_novelty_suppression_to_recent_history();
    if (nov_suppress_future_frames > 0){
        nov = 0.0f;
        nov_suppress_future_frames--;
    }

    float local = novelty_local_mean(nov);
    push_novelty(nov, local);

    if (tempo_update_countdown <= 0){
        if (tempo_update_hold > 0){
            tempo_update_hold--;
        } else {
            update_bpm_from_novelty();
        }
        tempo_update_countdown = kTempoUpdateIntervalFrames;
    } else {
        tempo_update_countdown--;
    }

    // --------- Beat blinker ---------
    // Match blinker to the currently found BPM shown to the user.
    float target_bpm_hz = (last_cand_bpm > 0.5f) ? (last_cand_bpm / 60.0f) : 0.0f;
    bool beat_triggered = false;
    bool beat_confident = (bpm_conf >= kBlinkMinConf);

    if (target_bpm_hz > 0.5f && beat_confident) {
        blink_bpm_hz = target_bpm_hz;
        blink_rate_hz = target_bpm_hz;
        if (blink_rate_hz < 0.1f) blink_rate_hz = 0.1f;

        float prev_phase = beat_phase;
        beat_phase = wrap_phase01(beat_phase + blink_rate_hz * frame_dt_sec);
        if (phase_crossed_forward(prev_phase, beat_phase, kBlinkPhaseTarget)) {
            fft_render_trigger_flash(kFlashFrames);
            beat_triggered = true;
        }
    } else {
        blink_bpm_hz  = 0.0f;
        blink_rate_hz = 0.0f;
        beat_phase    = 0.0f;
    }

    if (beat_triggered && s_beat_queue){
        fft_beat_event_t evt = {
            .bpm = blink_bpm_hz * 60.0f,
            .confidence = bpm_conf
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
        .bpm_est   = last_cand_bpm,
        .bpm_conf  = bpm_conf,
        .bpm_phase = bpm_phase
    };
    memcpy(pkt.logmag, logmag, sizeof(pkt.logmag));
    fft_render_submit(&pkt);

    int64_t dt_us = esp_timer_get_time() - t0;
    prof_accum_us += (uint64_t)dt_us;
    prof_frame_count++;
    if (prof_frame_count >= kLoadReportWindowFrames) {
        float avg_us = (float)prof_accum_us / (float)prof_frame_count;
        float hop_us = (hop_rate_hz_est > 1.0f) ? (1e6f / hop_rate_hz_est) : (1e6f / kHopRateNominalHz);
        last_load_pct = (hop_us > 0.0f) ? (avg_us / hop_us * 100.0f) : 0.0f;
        prof_accum_us = 0;
        prof_frame_count = 0;
    }
}

// ------------------- Task -------------------
static TaskHandle_t s_task=NULL;
static volatile bool s_stop_requested = false;

#define FFT_TASK_CORE 1
#define FFT_TASK_PRIO 4

static void fft_task(void *arg){
    i2s_chan_handle_t rx = audio_rx_handle();
    if (!rx){
        ESP_LOGE(TAG,"No RX handle");
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    for(int n=0;n<FFT_CFG_SIZE;++n){
        window[n]=0.5f*(1.0f - cosf((2.0f*(float)M_PI*n)/(float)(FFT_CFG_SIZE-1)));
    }
    const size_t in_bytes = FFT_CFG_HOP_SAMPLES * sizeof(int32_t) * 2;
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
        // IDF 6.x API expects timeout in milliseconds (not RTOS ticks).
        esp_err_t e = i2s_channel_read(rx, rx32, in_bytes, &nread, 200);
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
            sample_ring[ring_write_pos] = (float)v16 / 32768.0f;
            ring_write_pos = (ring_write_pos + 1) & (FFT_CFG_SIZE - 1);
            if ((ring_write_pos & (FFT_CFG_HOP_SAMPLES - 1)) == 0){
                process_fft_frame();
                taskYIELD();
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
    fft_reset_runtime_state();
    ESP_RETURN_ON_ERROR(audio_set_rate(FFT_CFG_SAMPLE_RATE_HZ), TAG, "set fs");
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

void fft_copy_frame(uint8_t *dst_fb, size_t dst_len){
    fft_render_copy_frame(dst_fb, dst_len);
}

// ------------------- Public API: get list of confident BPMs -------------------
int fft_get_confident_bpms(float *bpm_out, float *conf_out, int max_out){
    if (!bpm_out || !conf_out || max_out <= 0) return 0;

    int n = bpm_list_count;
    if (n > max_out) n = max_out;

    for (int i = 0; i < n; ++i){
        bpm_out[i] = bpm_list[i];
        conf_out[i] = bpm_conf_list[i];
    }
    return n;
}
