#include "fft.h"
#include "fft_internal.h"
#include "fft_render.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
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

// ------------------- DSP tuning -------------------
static const float kGlobalCombMin        = 0.01f;
static const float kPeakFracOfGlobal     = 0.25f;
static const float kHarmonicRatioTol     = 0.08f;
static const float kSpecialRatioTol      = 0.15f;
static const float kNoveltyZeroThreshold = 400.0f;
static const float kEnterConf            = 0.30f;
static const float kExitConf             = 0.18f;
static const float kOverallMinConf       = 0.25f;
static const float kPeakAvgMultiplier    = 10.0f; // peaks must be >= this × average comb energy
//static const float kPhaseSteerGain       = 0.01f; // small nudge toward phase=0.5

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
static int      g_nov_hole_frames = 0;  // number of upcoming frames to zero novelty

// ------------------- BPM export and smoothing -------------------
#define MAX_BPM_PEAKS   8      // internal peaks we consider from comb
#define MAX_BPM_EXPORT  3      // export at most 3 BPMs (fundamental, double, special)
#define BPM_AVG_FRAMES  6      // rolling average length for main BPM
#define PHASE_SLOTS     32     // downsampled phase bins for display
static float g_bpm_list[MAX_BPM_EXPORT];
static float g_bpm_conf_list[MAX_BPM_EXPORT];
static int   g_bpm_list_count = 0;
static float g_prev_export_bpm[MAX_BPM_EXPORT];
static int   g_prev_export_count = 0;

// history of the fundamental BPM per tempo-update frame
static EXT_RAM_BSS_ATTR float g_fund_hist[BPM_AVG_FRAMES];
static int   g_fund_hist_pos   = 0;
static int   g_fund_hist_count = 0;
static EXT_RAM_BSS_ATTR float   g_phase_curve[PHASE_SLOTS];
static uint8_t g_phase_curve_len = 0;

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
    g_prev_export_count = 0;
    g_bpm_list_count = 0;
    g_bpm_hz    = 0.0f;
    g_bpm_phase = 0.0f;
    g_phase_curve_len = 0;
}

// Public hook: zero novelty for the next `frames` updates to ignore button-induced spikes
void fft_punch_novelty_hole(int frames){
    if (frames < 1) frames = 1;
    g_nov_hole_frames = frames;
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

typedef struct {
    float bpm;
    float val;   // raw comb value
} bpm_peak_t;

typedef struct {
    float base_bpm;     // fundamental candidate (lowest in family)
    float energy;       // accumulated comb energy
    float best_double;  // best 2× bpm found, 0 if none
    float best_special; // best special-ratio bpm found, 0 if none
} bpm_family_t;

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
static void comb_bpm_from_novelty(const float *nov, int N, float frameRate, float *bpmSpec, float *bpmPhase){
    const int bpmCount = BPM_MAX - BPM_MIN + 1;
    const float minSamplesPerBpm = 2.0f;

    if (N <= 1){
        for (int i = 0; i < bpmCount; ++i){
            bpmSpec[i]  = 0.0f;
            bpmPhase[i] = 0.0f;
        }
        return;
    }

    for (int idx = 0; idx < bpmCount; ++idx){
        int bpm = BPM_MIN + idx;
        float step = frameRate * 60.0f / (float)bpm;

        int maxPhase = (int)floorf(step);
        if (maxPhase < 1) maxPhase = 1;

        float bestScore = 0.0f;
        int   bestPhaseIdx = 0;
        float phaseSlots[PHASE_SLOTS];
        for (int s = 0; s < PHASE_SLOTS; ++s) phaseSlots[s] = 0.0f;

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
                bestPhaseIdx = phase;
            }

            int slot = (maxPhase > 1) ? (int)((float)phase * (float)(PHASE_SLOTS - 1) / (float)(maxPhase - 1)) : 0;
            if (slot < 0) slot = 0;
            if (slot >= PHASE_SLOTS) slot = PHASE_SLOTS - 1;
            if (score > phaseSlots[slot]) phaseSlots[slot] = score;
        }

        bpmSpec[idx] = bestScore;

        float phaseNorm = 0.0f;
        if (bestScore > 0.0f && maxPhase > 1){
            phaseNorm = (float)bestPhaseIdx / (float)(maxPhase - 1);
        }
        bpmPhase[idx] = phaseNorm;

    }
}

static void compute_phase_curve_for_bpm(const float *nov, int N,
                                        float frameRate, float bpm,
                                        float *outSlots, uint8_t *outCount)
{
    if (!outSlots || !outCount || N <= 1 || bpm <= 0.0f) {
        if (outCount) *outCount = 0;
        return;
    }

    float step = frameRate * 60.0f / bpm;
    int   maxPhase = (int)floorf(step);
    if (maxPhase < 1) maxPhase = 1;

    for (int s = 0; s < PHASE_SLOTS; ++s) {
        outSlots[s] = 0.0f;
    }

    const float minSamplesPerBpm = 2.0f;

    for (int phase = 0; phase < maxPhase; ++phase){
        float pos = (float)phase;
        float sum = 0.0f;
        int   count = 0;

        while (pos < (float)(N - 1)){
            int   j = (int)floorf(pos);
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

        int slot = (maxPhase > 1)
                    ? (int)((float)phase * (float)(PHASE_SLOTS - 1) /
                            (float)(maxPhase - 1))
                    : 0;
        if (slot < 0) slot = 0;
        if (slot >= PHASE_SLOTS) slot = PHASE_SLOTS - 1;

        if (score > outSlots[slot]) {
            outSlots[slot] = score;
        }
    }

    *outCount = PHASE_SLOTS;
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
    static EXT_RAM_BSS_ATTR float bpmPhase[BPM_MAX - BPM_MIN + 1];
    static EXT_RAM_BSS_ATTR float bpmSpecBC[BPM_MAX - BPM_MIN + 1];

    comb_bpm_from_novelty(nov_history, history_len, frameRate,
                              bpmSpec, bpmPhase);

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

    // 5) Map baseline-corrected BPM spectrum → display arrays (30..286 BPM across PANEL_W)
    fft_render_update_tempo_spectrum(bpmSpecBC);

    // ------------------- Peak extraction from comb spectrum -------------------
    float globalMax = 0.0f;
    float sumSpec   = 0.0f;
    for (int i = 0; i < bpmCount; ++i){
        float v = bpmSpecBC[i];
        sumSpec += v;
        if (v > globalMax) globalMax = v;
    }
    float avgSpec = (bpmCount > 0) ? (sumSpec / (float)bpmCount) : 0.0f;

    g_bpm_list_count = 0;

    // If there's basically no energy, bail out early (too quiet / no beat)
    if (globalMax < kGlobalCombMin){
        bpm_soft_decay_and_reset();
        return;
    }

    // 1) Extract local peaks in baseline-corrected comb
    bpm_peak_t peaks[MAX_BPM_PEAKS];
    int        peakCount = 0;

    for (int i = 1; i < bpmCount - 1; ++i){
        float v = bpmSpecBC[i];
        if (v <= 0.0f) continue;
        if (v < kPeakFracOfGlobal * globalMax) continue;
        if (avgSpec > 0.0f && v < kPeakAvgMultiplier * avgSpec) continue;
        if (v <= bpmSpecBC[i - 1]) continue;
        if (v <= bpmSpecBC[i + 1]) continue;

        if (peakCount < MAX_BPM_PEAKS){
            peaks[peakCount].bpm = (float)(BPM_MIN + i);
            peaks[peakCount].val = v;
            peakCount++;
        }
    }

    if (peakCount == 0){
        bpm_soft_decay_and_reset();
        return;
    }

    // 2) Sort peaks by BPM ascending
    for (int i = 0; i < peakCount; ++i){
        int best = i;
        for (int j = i + 1; j < peakCount; ++j){
            if (peaks[j].bpm < peaks[best].bpm) best = j;
        }
        if (best != i){
            bpm_peak_t tmp = peaks[i];
            peaks[i] = peaks[best];
            peaks[best] = tmp;
        }
    }

    // 3) Choose fundamental by halving the strongest peak until <=130 BPM
    int strongest = 0;
    for (int i = 1; i < peakCount; ++i){
        if (peaks[i].val > peaks[strongest].val) strongest = i;
    }
    float fund_inst = peaks[strongest].bpm;
    while (fund_inst > 130.0f) fund_inst *= 0.5f;
    if (fund_inst < (float)BPM_MIN) fund_inst = (float)BPM_MIN;

    // 4) Attach peaks as harmonics/specials of the chosen fundamental
    bpm_family_t fam;
    memset(&fam, 0, sizeof(fam));
    fam.base_bpm = fund_inst;

    const float ratios[] = {1.5f, 4.0f/3.0f, 5.0f/4.0f, 3.0f};
    const int   numRatios = sizeof(ratios)/sizeof(ratios[0]);

    for (int p = 0; p < peakCount; ++p){
        float bpm = peaks[p].bpm;
        float val = peaks[p].val;

        float r = bpm / fam.base_bpm;
        float r_abs = (r < 0.0f) ? -r : r;
        if (r_abs < 1.0f) r_abs = 1.0f / r_abs; // allow subharmonics treated as multiples

        bool attached = false;
        float n = floorf(r_abs + 0.5f);
        if (n >= 1.0f && n <= 8.0f && fabsf(r_abs - n) <= kHarmonicRatioTol){
            attached = true;
        } else {
            for (int k = 0; k < numRatios; ++k){
                if (fabsf(r_abs - ratios[k]) <= kSpecialRatioTol){
                    attached = true;
                    if (fam.best_special == 0.0f) fam.best_special = bpm;
                    break;
                }
            }
        }

        if (!attached) continue;

        fam.energy += val;

        if (fabsf(r_abs - 2.0f) < 0.25f){
            if (fam.best_double == 0.0f || val > fam.energy * 0.5f) {
                fam.best_double = bpm;
            }
        }

        for (int k = 0; k < numRatios; ++k){
            if (fabsf(r_abs - ratios[k]) <= kSpecialRatioTol){
                if (fam.best_special == 0.0f) {
                    fam.best_special = bpm;
                }
                break;
            }
        }
    }
    float fund_avg  = update_fund_history(fund_inst);
    if (fund_avg <= 0.0f){
        bpm_soft_decay_and_reset();
        return;
    }

    // 5) Fill export list (max 3 entries) with explicit ordering:
    // [0] fundamental, [1..] harmonics, [last] special ratios.
    int exportCount = 0;

    // Fundamental always first
    float fund_conf = fam.energy / (globalMax + 1e-6f);
    if (fund_conf > 1.0f) fund_conf = 1.0f;

    if (exportCount < MAX_BPM_EXPORT) {
        g_bpm_list[exportCount]      = fund_avg;
        g_bpm_conf_list[exportCount] = fund_conf;
        exportCount++;
    }

    // Higher-frequency harmonics (currently just best_double)
    if (fam.best_double > 0.0f && exportCount < MAX_BPM_EXPORT) {
        g_bpm_list[exportCount]      = fam.best_double;
        g_bpm_conf_list[exportCount] = 0.8f * fund_conf;
        exportCount++;
    }

    // Special-ratio BPMs always come after harmonics
    if (fam.best_special > 0.0f && exportCount < MAX_BPM_EXPORT) {
        g_bpm_list[exportCount]      = fam.best_special;
        g_bpm_conf_list[exportCount] = 0.7f * fund_conf;
        exportCount++;
    }


    float new_bpm[MAX_BPM_EXPORT];
    float new_conf[MAX_BPM_EXPORT];
    int   new_count = 0;

    for (int i = 0; i < exportCount; ++i){
        float bpm  = g_bpm_list[i];
        float conf = g_bpm_conf_list[i];

        bool wasActive = false;
        for (int j = 0; j < g_prev_export_count; ++j){
            if (fabsf(g_prev_export_bpm[j] - bpm) < 1.0f){
                wasActive = true;
                break;
            }
        }

        if ((!wasActive && conf >= kEnterConf) || (wasActive && conf >= kExitConf)){
            if (new_count < MAX_BPM_EXPORT){
                new_bpm[new_count]  = bpm;
                new_conf[new_count] = conf;
                new_count++;
            }
        }
    }

    memcpy(g_bpm_list, new_bpm, new_count * sizeof(float));
    memcpy(g_bpm_conf_list, new_conf, new_count * sizeof(float));
    g_bpm_list_count = new_count;

    memcpy(g_prev_export_bpm, g_bpm_list, new_count * sizeof(float));
    g_prev_export_count = g_bpm_list_count;

    if (g_bpm_list_count == 0){
        bpm_soft_decay_and_reset();
        return;
    }

    float maxConf = 0.0f;
    for (int i = 0; i < g_bpm_list_count; ++i){
        if (g_bpm_conf_list[i] > maxConf) maxConf = g_bpm_conf_list[i];
    }

    if (maxConf < kOverallMinConf){
        bpm_soft_decay_and_reset();
        return;
    }

    // Confidence and stability tracking for headline BPM
    int bestIdx = (int)lroundf(fund_inst - (float)BPM_MIN);
    if (bestIdx < 0) bestIdx = 0;
    if (bestIdx >= bpmCount) bestIdx = bpmCount - 1;

    float bestScore = bpmSpecBC[bestIdx];
    float avgSpecForConf = (bpmCount > 0) ? (sumSpec / (float)bpmCount) : 0.0f;
    float spectral_conf = 0.0f;
    if (avgSpecForConf > 0.0f){
        spectral_conf = bestScore / (avgSpecForConf + 1e-6f);
        if (spectral_conf > 4.0f) spectral_conf = 4.0f;
        spectral_conf /= 4.0f;
    }

    const float kBpmTol = 3.0f;
    float diff = fabsf(fund_avg - g_last_cand_bpm);
    if (g_last_cand_bpm > 0.0f && diff < kBpmTol) {
        g_bpm_stable_frames++;
    } else {
        g_bpm_stable_frames = 1;
        g_last_cand_bpm     = fund_avg;
    }

    float target_stable_frames = kHopRate_nominal * 4.0f;
    if (target_stable_frames < 1.0f) target_stable_frames = 1.0f;
    float stable_conf = (float)g_bpm_stable_frames / target_stable_frames;
    if (stable_conf > 1.0f) stable_conf = 1.0f;

    float combined_raw_conf = 0.7f * spectral_conf + 0.3f * stable_conf;
    g_bpm_conf = 0.9f * g_bpm_conf + 0.1f * combined_raw_conf;

    g_bpm_hz    = fund_avg / 60.0f;
    g_bpm_phase = bpmPhase[bestIdx];

    // Phase curve display for the chosen fundamental (compute only for it)
    if (fund_avg > 0.0f) {
        compute_phase_curve_for_bpm(nov_history, history_len,
                                    frameRate, fund_avg,
                                    g_phase_curve, &g_phase_curve_len);
        fft_render_update_phase_curve(
            (g_phase_curve_len > 0) ? g_phase_curve : NULL,
            g_phase_curve_len);
    } else {
        g_phase_curve_len = 0;
        fft_render_update_phase_curve(NULL, 0);
    }
}

// ------------------- Processing per FFT frame -------------------
static void process_fft_frame(void){
    int64_t t0 = esp_timer_get_time();
    // Update hop rate estimate using high-resolution timer
    if (g_last_frame_time_us != 0){
        int64_t dt_us = t0 - g_last_frame_time_us;
        if (dt_us > 500 && dt_us < 500000){ // clamp to reasonable range
            float inst_rate = 1e6f / (float)dt_us;
            g_hop_rate_hz_est = 0.9f * g_hop_rate_hz_est + 0.1f * inst_rate;
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
    // Requested hole: zero novelty for N frames after button voltage changes
    if (g_nov_hole_frames > 0){
        nov = 0.0f;
        g_nov_hole_frames--;
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
    float fps_used = kHopRate_nominal;

    const float kConfUse = 0.65f;
    float target_bpm_hz = (g_bpm_conf > kConfUse && g_bpm_hz > 0.5f) ? g_bpm_hz : 0.0f;
    bool beat_triggered = false;

    if (target_bpm_hz > 0.5f) {
        if (g_blink_bpm_hz <= 0.0f) {
            g_blink_bpm_hz = target_bpm_hz;
        }

        g_blink_rate_hz = g_blink_bpm_hz;
        if (g_blink_rate_hz < 0.1f) g_blink_rate_hz = 0.1f;

        g_beat_phase += g_blink_rate_hz / fps_used;
        if (g_beat_phase >= 1.0f) {
            g_beat_phase -= 1.0f;
            fft_render_trigger_flash(kFlashFrames);

            g_blink_bpm_hz = target_bpm_hz;
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

static void fft_task(void *arg){
    i2s_chan_handle_t rx = audio_rx_handle();
    if (!rx){ ESP_LOGE(TAG,"No RX handle"); vTaskDelete(NULL); return; }

    for(int n=0;n<FFT_SIZE;++n){
        g_window[n]=0.5f*(1.0f - cosf((2.0f*(float)M_PI*n)/(float)(FFT_SIZE-1)));
    }
    const size_t in_bytes = HOP_SAMPLES * sizeof(int32_t) * 2;
    int32_t *rx32 = (int32_t*)malloc(in_bytes);
    if (!rx32){ ESP_LOGE(TAG,"OOM"); vTaskDelete(NULL); return; }

    int slot_index=0; bool slot_decided=false;

    while(1){
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
}

esp_err_t fft_visualizer_start(void){
    if (s_task) return ESP_OK;
    ESP_RETURN_ON_ERROR(audio_set_rate(SAMPLE_RATE_HZ), TAG, "set fs");
    ESP_RETURN_ON_ERROR(audio_enable_rx(), TAG, "enable rx");
    led_init();
    ESP_RETURN_ON_ERROR(fft_render_init(), TAG, "render init");
    if (!s_beat_queue){
        s_beat_queue = xQueueCreate(4, sizeof(fft_beat_event_t));
        if (!s_beat_queue) return ESP_ERR_NO_MEM;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(fft_task, "fft_vis", 12288, NULL, 5, &s_task, 1);
    return ok==pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
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
