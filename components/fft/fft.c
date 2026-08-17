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
#include "portmacro.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "fft_vis";

// ------------------- Config -------------------
static const int   kFlashFrames   = 5;      // frames to hold beat flash
static const float kNoveltyMeanOffset = 0.02f; // display-only flux mean offset
static const float kHopRateNominalHz = (float)FFT_CFG_SAMPLE_RATE_HZ / (float)FFT_CFG_HOP_SAMPLES; // 62.5 Hz
static const int   kTempoUpdateIntervalFrames = (FFT_CFG_SAMPLE_RATE_HZ + (2 * FFT_CFG_HOP_SAMPLES) - 1) / (2 * FFT_CFG_HOP_SAMPLES);
static const int   kLoadReportWindowFrames = (FFT_CFG_SAMPLE_RATE_HZ + (FFT_CFG_HOP_SAMPLES / 2)) / FFT_CFG_HOP_SAMPLES; // ~1 sec
static const float kBlinkPhaseTarget = 0.0f; // variance phase clock emits at cycle wrap
#define PHASE_SLOTS     32     // downsampled phase bins for display
#define PHASE_GRAPH_BINS 128
#define COMB_BANDS      32
#define PHASE_SAMPLES_PER_FRAME 8
#define BPM_COUNT      (FFT_CFG_BPM_MAX - FFT_CFG_BPM_MIN + 1)
#define FFT_TASK_CORE 1
#define FFT_TASK_PRIO 4
#define BPM_WORKER_PRIO 2

#if ((FFT_CFG_SIZE & (FFT_CFG_SIZE - 1)) != 0)
#error "FFT_CFG_SIZE must be a power of two for ring-buffer indexing"
#endif

#if ((FFT_CFG_HOP_SAMPLES & (FFT_CFG_HOP_SAMPLES - 1)) != 0)
#error "FFT_CFG_HOP_SAMPLES must be a power of two for hop-boundary indexing"
#endif

// ------------------- DSP tuning -------------------
static const float kBlinkMinConf         = 0.25f;
static const float kLevelFastAlpha       = 0.36f;
static const float kLevelSlowAlpha       = 0.045f;
static const float kLevelPeakDecay       = 0.992f;
static const float kRawVolumeGain        = 12.0f;
static const float kMinInterestHz        = 100.0f;
#define STABILITY_FRAMES 4
#define STABILITY_VARIANCE_THRESHOLD 0.0010f
#define SWITCH_OVERTAKE_RATIO 1.20f
#define PHASE_LOCK_MAX_BPM_DIFF 2.5f
#define PHASE_SWITCH_OVERTAKE_RATIO 1.20f
#define MAX_PHASE_EVAL 256

typedef struct {
    float fast;
    float slow;
    float peak;
    float out;
} level_track_t;

typedef struct {
    int have;
    float bpm;
    int pair_idx;
} bpm_hold_state_t;

typedef struct {
    int have;
    float bpm;
    float phase_frac;
    int phase_idx;
    float phase_score;
} phase_hold_state_t;

typedef struct {
    bool initialized;
    float bpm_current;
    float bpm_target;
    float bpm_pending;
    bool has_pending_bpm;
    int beats_since_period_change;
    int64_t period_current_us;
    int64_t period_target_us;
    int64_t period_pending_us;
    int64_t beat_start_us;
    int64_t next_beat_us;
} beat_clock_t;

typedef struct {
    int num_frames;
    float frame_rate;
    int64_t last_frame_time_us;
} bpm_worker_job_t;

typedef struct {
    bool valid;
    bool stable;
    bool phase_valid;
    float final_bpm;
    float accum_bpm;
    float confidence;
    float stability_variance;
    float accepted_phase_frac;
    int final_pair_idx;
    int64_t detected_beat_time_us;
    float mean_norm_scores[BPM_COUNT];
    float phase_curve[PHASE_SLOTS];
} bpm_worker_result_t;

// ------------------- State -------------------
static size_t  ring_write_pos = 0; // next sample slot to write
static EXT_RAM_BSS_ATTR float sample[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float window[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float fft_re[FFT_CFG_SIZE];
static EXT_RAM_BSS_ATTR float fft_im[FFT_CFG_SIZE];
static float   last_cand_bpm     = 0.0f; // effective BPM after user lock
static float   raw_cand_bpm      = 0.0f; // detector BPM before user locking

static int     band_write_pos = 0; // next slot to overwrite
static int     band_frame_count = 0;
static EXT_RAM_BSS_ATTR float band_history[FFT_CFG_NOV_RING_FRAMES][COMB_BANDS];
static float   prev_band_frame[COMB_BANDS];
static bool    prev_band_frame_valid = false;
static float   band_flux_local_sum = 0.0f;

static uint64_t prof_accum_us = 0;     // accumulated processing time in microseconds
static int      prof_frame_count = 0;  // frames counted in the accumulator
static float    last_load_pct = 0.0f;
static float    hop_rate_hz_est = kHopRateNominalHz;   // immediate hop-rate estimate from timer
static int64_t  last_frame_time_us = 0;
static float    bpm_hz = 0.0f;
static float    bpm_conf = 0.0f;
static float    bpm_phase = 0.0f;
static float    blink_bpm_hz = 0.0f;  // latched BPM that drives the blinker until next beat
static float    blink_rate_hz = 0.0f; // actual blink rate used for display
static float    beat_phase = 0.0f;    // phase accumulator for main beat blinker
static int      tempo_update_countdown = 0;
static int      tempo_update_hold = 0; // tempo update cycles to skip after suppression
static QueueHandle_t s_beat_queue = NULL;
static volatile int band_suppress_backfill_frames = 0;  // recent frames to suppress
static volatile int band_suppress_future_frames = 0;    // upcoming frames to suppress
static portMUX_TYPE s_ctrl_mux = portMUX_INITIALIZER_UNLOCKED;
static bool     s_user_bpm_locked = false;
static bool     s_user_beat_enabled = true;
static bool     s_led_export_enabled = true;
static float    s_user_locked_bpm = 0.0f;
static float    s_user_phase_offset = 0.0f;

static EXT_RAM_BSS_ATTR float phase_curve[PHASE_SLOTS];
static EXT_RAM_BSS_ATTR float bpm_scores[BPM_COUNT];
static beat_clock_t s_beat_clock = {0};

static TaskHandle_t s_bpm_task = NULL;
static QueueHandle_t s_bpm_job_queue = NULL;
static QueueHandle_t s_bpm_result_queue = NULL;
static volatile bool s_bpm_worker_busy = false;
static volatile bool s_bpm_worker_stop = false;
static EXT_RAM_BSS_ATTR float s_bpm_job_frames[FFT_CFG_NOV_RING_FRAMES * COMB_BANDS];
static EXT_RAM_BSS_ATTR float s_worker_raw_score[BPM_COUNT];
static EXT_RAM_BSS_ATTR float s_worker_norm_score[BPM_COUNT];
static EXT_RAM_BSS_ATTR float s_worker_mean_norm_score[BPM_COUNT];
static EXT_RAM_BSS_ATTR float s_worker_norm_history[STABILITY_FRAMES][BPM_COUNT];
static EXT_RAM_BSS_ATTR float s_worker_phase_score[MAX_PHASE_EVAL];
static int s_worker_norm_history_write = 0;
static int s_worker_norm_history_count = 0;
static bpm_hold_state_t s_worker_bpm_hold = {0};
static phase_hold_state_t s_worker_phase_hold = {0};

static level_track_t s_level_overall = {0};
static level_track_t s_level_low = {0};
static level_track_t s_level_mid = {0};
static level_track_t s_level_high = {0};

static inline float wrap_phase01(float phase){ // Wrap phase to [0,1)
    phase -= floorf(phase);
    if (phase < 0.0f) phase += 1.0f;
    return phase;
}

static int64_t period_us_from_bpm(float bpm)
{
    if (!isfinite(bpm) || bpm <= 0.5f) return 0;
    int64_t period = (int64_t)llround(60000000.0 / (double)bpm);
    return (period > 0) ? period : 1;
}

static void beat_clock_reset(beat_clock_t *clk)
{
    if (!clk) return;
    memset(clk, 0, sizeof(*clk));
}

static float beat_clock_phase(const beat_clock_t *clk, int64_t now_us)
{
    if (!clk || !clk->initialized || clk->period_current_us <= 0) return 0.0f;
    double phase = (double)(now_us - clk->beat_start_us) / (double)clk->period_current_us;
    phase -= floor(phase);
    if (phase < 0.0) phase += 1.0;
    return (float)phase;
}

static void beat_clock_init_from_detected_beat(beat_clock_t *clk,
                                               float bpm,
                                               int64_t detected_beat_us,
                                               int64_t now_us)
{
    if (!clk) return;
    int64_t period = period_us_from_bpm(bpm);
    if (period <= 0) return;

    int64_t start = detected_beat_us;
    int guard = 0;
    while (start > now_us && guard++ < 256) start -= period;
    guard = 0;
    while (start + period <= now_us && guard++ < 256) start += period;

    clk->initialized = true;
    clk->bpm_current = bpm;
    clk->bpm_target = bpm;
    clk->bpm_pending = bpm;
    clk->has_pending_bpm = false;
    clk->period_current_us = period;
    clk->period_target_us = period;
    clk->period_pending_us = period;
    clk->beats_since_period_change = 1000000;
    clk->beat_start_us = start;
    clk->next_beat_us = start + period;
}

static void beat_clock_queue_bpm(beat_clock_t *clk, float bpm)
{
    if (!clk || !clk->initialized) return;
    int64_t period = period_us_from_bpm(bpm);
    if (period <= 0) return;

    clk->bpm_pending = bpm;
    clk->period_pending_us = period;
    if (llabs(period - clk->period_target_us) > 1) {
        clk->has_pending_bpm = true;
    }
}

static void beat_clock_update(beat_clock_t *clk, int64_t now_us)
{
    if (!clk || !clk->initialized || clk->period_current_us <= 0) return;

    int guard = 0;
    while (now_us >= clk->next_beat_us && guard++ < 8) {
        clk->beat_start_us = clk->next_beat_us;

        if (clk->has_pending_bpm) {
            clk->bpm_target = clk->bpm_pending;
            clk->period_target_us = clk->period_pending_us;
            clk->has_pending_bpm = false;
            clk->beats_since_period_change = 0;
        } else if (clk->beats_since_period_change < 1000000) {
            clk->beats_since_period_change++;
        }

        clk->period_current_us = clk->period_target_us;
        clk->bpm_current = (clk->period_current_us > 0)
            ? (float)(60000000.0 / (double)clk->period_current_us)
            : 0.0f;
        clk->next_beat_us = clk->beat_start_us + clk->period_current_us;
    }

    if (guard >= 8) {
        clk->beat_start_us = now_us;
        clk->next_beat_us = now_us + clk->period_current_us;
    }
}

static void bpm_worker_reset_state(void)
{
    memset(s_worker_raw_score, 0, sizeof(s_worker_raw_score));
    memset(s_worker_norm_score, 0, sizeof(s_worker_norm_score));
    memset(s_worker_mean_norm_score, 0, sizeof(s_worker_mean_norm_score));
    memset(s_worker_norm_history, 0, sizeof(s_worker_norm_history));
    memset(s_worker_phase_score, 0, sizeof(s_worker_phase_score));
    s_worker_norm_history_write = 0;
    s_worker_norm_history_count = 0;
    memset(&s_worker_bpm_hold, 0, sizeof(s_worker_bpm_hold));
    memset(&s_worker_phase_hold, 0, sizeof(s_worker_phase_hold));
}

static void fft_reset_runtime_state(void){
    ring_write_pos = 0;
    memset(sample, 0, sizeof(sample));
    last_cand_bpm = 0.0f;
    raw_cand_bpm = 0.0f;

    band_write_pos = 0;
    band_frame_count = 0;
    memset(band_history, 0, sizeof(band_history));
    memset(prev_band_frame, 0, sizeof(prev_band_frame));
    prev_band_frame_valid = false;
    band_flux_local_sum = 0.0f;

    prof_accum_us = 0;
    prof_frame_count = 0;
    last_load_pct = 0.0f;
    hop_rate_hz_est = kHopRateNominalHz;
    last_frame_time_us = 0;

    bpm_hz = 0.0f;
    bpm_conf = 0.0f;
    bpm_phase = 0.0f;
    blink_bpm_hz = 0.0f;
    blink_rate_hz = 0.0f;
    beat_phase = 0.0f;
    tempo_update_countdown = 0;
    tempo_update_hold = 0;
    band_suppress_backfill_frames = 0;
    band_suppress_future_frames = 0;
    beat_clock_reset(&s_beat_clock);

    memset(phase_curve, 0, sizeof(phase_curve));
    memset(bpm_scores, 0, sizeof(bpm_scores));
    memset(&s_level_overall, 0, sizeof(s_level_overall));
    memset(&s_level_low, 0, sizeof(s_level_low));
    memset(&s_level_mid, 0, sizeof(s_level_mid));
    memset(&s_level_high, 0, sizeof(s_level_high));
    led_audio_raw_volume_set(0.0f);
    s_bpm_worker_busy = false;
    bpm_worker_reset_state();
    if (s_bpm_job_queue) xQueueReset(s_bpm_job_queue);
    if (s_bpm_result_queue) xQueueReset(s_bpm_result_queue);
}

static int bin_from_hz(float hz)
{
    int bin = (int)lroundf(hz * (float)FFT_CFG_SIZE / (float)FFT_CFG_SAMPLE_RATE_HZ);
    if (bin < 0) bin = 0;
    if (bin > FFT_CFG_SIZE / 2) bin = FFT_CFG_SIZE / 2;
    return bin;
}

static float band_mean_logmag(const float *logmag, float hz_lo, float hz_hi)
{
    if (!logmag) return 0.0f;
    int lo = bin_from_hz(hz_lo);
    int hi = bin_from_hz(hz_hi);
    if (hi < lo) {
        int tmp = lo;
        lo = hi;
        hi = tmp;
    }
    if (hi <= lo) hi = lo + 1;
    if (hi > FFT_CFG_SIZE / 2) hi = FFT_CFG_SIZE / 2;

    float sum = 0.0f;
    int count = 0;
    for (int i = lo; i <= hi; ++i) {
        sum += logmag[i];
        count++;
    }
    return (count > 0) ? (sum / (float)count) : 0.0f;
}

static float recent_hop_rms(void)
{
    const size_t ring_mask = FFT_CFG_SIZE - 1;
    size_t base = (ring_write_pos - (size_t)FFT_CFG_HOP_SAMPLES) & ring_mask;
    float sum_sq = 0.0f;
    for (size_t i = 0; i < (size_t)FFT_CFG_HOP_SAMPLES; ++i) {
        float v = sample[(base + i) & ring_mask];
        sum_sq += v * v;
    }
    float rms = sqrtf(sum_sq / (float)FFT_CFG_HOP_SAMPLES);
    float scaled = rms * kRawVolumeGain;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 1.0f) scaled = 1.0f;
    return scaled;
}

static void update_level_track(level_track_t *track, float raw)
{
    if (!track || !isfinite(raw)) return;
    track->fast += kLevelFastAlpha * (raw - track->fast);
    track->slow += kLevelSlowAlpha * (raw - track->slow);
    float delta = track->fast - track->slow;
    if (delta < 0.0f) delta = 0.0f;
    if (delta > track->peak) {
        track->peak = delta;
    } else {
        track->peak = track->peak * kLevelPeakDecay + delta * (1.0f - kLevelPeakDecay);
    }
    if (track->peak < 0.05f) track->peak = 0.05f;
    track->out = delta / track->peak;
    if (track->out < 0.0f) track->out = 0.0f;
    if (track->out > 1.0f) track->out = 1.0f;
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

static void fft_get_user_ctrl_locked(bool *bpm_locked, bool *beat_enabled,
                                     float *locked_bpm, float *phase_offset)
{
    portENTER_CRITICAL(&s_ctrl_mux);
    if (bpm_locked) *bpm_locked = s_user_bpm_locked;
    if (beat_enabled) *beat_enabled = s_user_beat_enabled;
    if (locked_bpm) *locked_bpm = s_user_locked_bpm;
    if (phase_offset) *phase_offset = s_user_phase_offset;
    portEXIT_CRITICAL(&s_ctrl_mux);
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

// ------------------- Spectral-band history + display flux -------------------
static float band_flux_local_mean(float v)
{
    const float win = (float)FFT_CFG_NOVELTY_WIN;
    band_flux_local_sum += v - (band_flux_local_sum / win);
    if (band_flux_local_sum < 0.0f) band_flux_local_sum = 0.0f;
    return band_flux_local_sum / win + kNoveltyMeanOffset;
}

static float push_band_frame_from_logmag(const float *logmag, bool suppress_frame)
{
    float *dst = band_history[band_write_pos];
    float raw_flux = 0.0f;

    if (!logmag || suppress_frame) {
        memset(dst, 0, COMB_BANDS * sizeof(float));
        prev_band_frame_valid = false;
    } else {
        int min_bin = bin_from_hz(kMinInterestHz);
        int max_bin = FFT_CFG_SIZE / 2;
        if (min_bin < 0) min_bin = 0;
        if (min_bin > max_bin) min_bin = max_bin;

        int usable_bins = max_bin - min_bin + 1;
        if (usable_bins < 1) usable_bins = 1;

        for (int b = 0; b < COMB_BANDS; ++b) {
            int k0 = min_bin + (b * usable_bins) / COMB_BANDS;
            int k1 = min_bin + ((b + 1) * usable_bins) / COMB_BANDS;
            if (k1 <= k0) k1 = k0 + 1;
            if (k1 > max_bin + 1) k1 = max_bin + 1;

            float sum = 0.0f;
            int count = 0;
            for (int k = k0; k < k1; ++k) {
                sum += logmag[k];
                count++;
            }

            float v = (count > 0) ? (sum / (float)count) : 0.0f;
            dst[b] = v;
            if (prev_band_frame_valid && v > prev_band_frame[b]) {
                raw_flux += v - prev_band_frame[b];
            }
        }
        memcpy(prev_band_frame, dst, sizeof(prev_band_frame));
        prev_band_frame_valid = true;
    }

    band_write_pos = (band_write_pos + 1) % FFT_CFG_NOV_RING_FRAMES;
    if (band_frame_count < FFT_CFG_NOV_RING_FRAMES) band_frame_count++;
    return raw_flux;
}

static void bpm_soft_decay_and_reset(void){
    bpm_conf *= 0.9f;
    last_cand_bpm  = 0.0f;
    raw_cand_bpm   = 0.0f;
    bpm_hz         = 0.0f;
    bpm_phase      = 0.0f;
    blink_bpm_hz   = 0.0f;
    blink_rate_hz  = 0.0f;
    beat_phase     = 0.0f;
    beat_clock_reset(&s_beat_clock);
    memset(phase_curve, 0, sizeof(phase_curve));
    memset(bpm_scores, 0, sizeof(bpm_scores));
    if (fft_render_is_display_enabled()) {
        fft_render_update_tempo_spectrum(NULL);
        fft_render_update_phase_curve(NULL, 0);
    }
}

static void apply_band_suppression_to_recent_history(void){
    int frames = band_suppress_backfill_frames;
    if (frames <= 0) return;
    band_suppress_backfill_frames = 0;

    int history_len = FFT_CFG_NOV_RING_FRAMES;
    if (frames > history_len) frames = history_len;

    for (int i = 0; i < frames; ++i){
        int idx = band_write_pos - 1 - i;
        while (idx < 0) idx += FFT_CFG_NOV_RING_FRAMES;
        memset(band_history[idx], 0, COMB_BANDS * sizeof(float));
    }
    if (fft_render_is_display_enabled()){
        fft_render_suppress_recent_novelty(frames);
    }

    int total_supp_frames = frames + band_suppress_future_frames;
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

static int novelty_frames_from_ms(int backfill_ms){
    if (backfill_ms <= 0) return 1;
    const int denom = FFT_CFG_HOP_SAMPLES * 1000;
    int64_t num = (int64_t)backfill_ms * (int64_t)FFT_CFG_SAMPLE_RATE_HZ;
    int frames = (int)((num + denom - 1) / denom); // ceil(ms / hop_ms)
    if (frames < 1) frames = 1;
    if (frames > FFT_CFG_NOV_RING_FRAMES) frames = FFT_CFG_NOV_RING_FRAMES;
    return frames;
}

// ------------------- Variance-comb BPM worker -------------------
static void copy_band_history_ordered(float *dst, int num_frames)
{
    if (!dst || num_frames <= 0) return;
    int start = band_write_pos - num_frames;
    while (start < 0) start += FFT_CFG_NOV_RING_FRAMES;

    for (int i = 0; i < num_frames; ++i) {
        int src = (start + i) % FFT_CFG_NOV_RING_FRAMES;
        memcpy(&dst[i * COMB_BANDS], band_history[src],
               COMB_BANDS * sizeof(float));
    }
}

static inline float ordered_band_at(const float *frames, int num_frames,
                                    float pos, int band)
{
    if (!frames || num_frames <= 0) return 0.0f;
    if (band < 0) band = 0;
    if (band >= COMB_BANDS) band = COMB_BANDS - 1;

    if (pos <= 0.0f) return frames[band];
    if (pos >= (float)(num_frames - 1)) {
        return frames[(num_frames - 1) * COMB_BANDS + band];
    }

    int i0 = (int)floorf(pos);
    float t = pos - (float)i0;
    float a = frames[i0 * COMB_BANDS + band];
    float b = frames[(i0 + 1) * COMB_BANDS + band];
    return a + (b - a) * t;
}

static void compute_raw_comb_scores(const float *frames, int num_frames,
                                    float frame_rate, float *score_out)
{
    if (!score_out) return;
    if (!frames || num_frames <= 2 || frame_rate <= 0.0f) {
        memset(score_out, 0, BPM_COUNT * sizeof(float));
        return;
    }

    for (int bpm_idx = 0; bpm_idx < BPM_COUNT; ++bpm_idx) {
        if (s_bpm_worker_stop) return;

        int bpm = FFT_CFG_BPM_MIN + bpm_idx;
        float step = frame_rate * 60.0f / (float)bpm;
        int phase_count = (int)ceilf(step * (float)PHASE_SAMPLES_PER_FRAME);
        if (phase_count < 1) phase_count = 1;
        if (phase_count > MAX_PHASE_EVAL) phase_count = MAX_PHASE_EVAL;

        float total_variance = 0.0f;
        int valid_eval_count = 0;

        for (int phase_idx = 0; phase_idx < phase_count; ++phase_idx) {
            float phase = step * ((float)phase_idx / (float)phase_count);

            for (int b = 0; b < COMB_BANDS; ++b) {
                int n = 0;
                float sum = 0.0f;
                float sum_sq = 0.0f;

                for (float pos = phase; pos < (float)(num_frames - 1); pos += step) {
                    float x = ordered_band_at(frames, num_frames, pos, b);
                    n++;
                    sum += x;
                    sum_sq += x * x;
                }

                if (n >= 2) {
                    float var = (sum_sq - (sum * sum) / (float)n) / (float)(n - 1);
                    if (var < 0.0f && var > -1e-6f) var = 0.0f;
                    if (var > 0.0f && isfinite(var)) {
                        total_variance += var;
                    }
                    valid_eval_count++;
                }
            }
        }

        score_out[bpm_idx] = (valid_eval_count > 0)
            ? (total_variance / (float)valid_eval_count)
            : 0.0f;

        if ((bpm_idx & 7) == 7) {
            taskYIELD();
        }
        if ((bpm_idx & 15) == 15) {
            vTaskDelay(1);
        }
    }
}

static void raw_scores_to_positive_inplace(float *score)
{
    if (!score) return;
    float max_v = 0.0f;
    for (int i = 0; i < BPM_COUNT; ++i) {
        float v = score[i];
        if (isfinite(v) && v > max_v) max_v = v;
    }
    for (int i = 0; i < BPM_COUNT; ++i) {
        float v = score[i];
        if (!isfinite(v) || v < 0.0f) v = max_v;
        score[i] = max_v - v;
        if (score[i] < 0.0f) score[i] = 0.0f;
    }
}

static void bpm_baseline_correct(float *score)
{
    if (!score) return;
    int mid = BPM_COUNT / 2;
    float low_sum = 0.0f;
    float high_sum = 0.0f;
    for (int i = 0; i < mid; ++i) low_sum += score[i];
    for (int i = mid; i < BPM_COUNT; ++i) high_sum += score[i];

    float low_avg = (mid > 0) ? (low_sum / (float)mid) : 0.0f;
    float high_avg = (BPM_COUNT - mid > 0) ? (high_sum / (float)(BPM_COUNT - mid)) : 0.0f;
    int x1 = mid / 2;
    int x2 = mid + (BPM_COUNT - mid) / 2;
    if (x2 == x1) {
        x1 = 0;
        x2 = BPM_COUNT - 1;
    }

    float slope = (x2 != x1) ? ((high_avg - low_avg) / (float)(x2 - x1)) : 0.0f;
    float intercept = low_avg - slope * (float)x1;
    for (int i = 0; i < BPM_COUNT; ++i) {
        float base = intercept + slope * (float)i;
        float v = score[i] - base;
        score[i] = (v > 0.0f && isfinite(v)) ? v : 0.0f;
    }
}

static bool normalize_scores_by_sum(const float *score, float *norm_out)
{
    if (!score || !norm_out) return false;
    float sum = 0.0f;
    for (int i = 0; i < BPM_COUNT; ++i) {
        float v = score[i];
        if (isfinite(v) && v > 0.0f) sum += v;
    }
    if (sum <= 1e-12f) {
        memset(norm_out, 0, BPM_COUNT * sizeof(float));
        return false;
    }
    for (int i = 0; i < BPM_COUNT; ++i) {
        float v = score[i];
        norm_out[i] = (isfinite(v) && v > 0.0f) ? (v / sum) : 0.0f;
    }
    return true;
}

static void update_final_bpm_from_accum(float accum_bpm,
                                        int accum_pair_idx,
                                        float accum_pair_score,
                                        const float *mean_scores,
                                        float stability_variance,
                                        int history_count,
                                        bpm_hold_state_t *state,
                                        float *final_bpm_out,
                                        int *final_pair_idx_out)
{
    float final_bpm = accum_bpm;
    int final_pair_idx = accum_pair_idx;
    bool stable_enough = (history_count >= STABILITY_FRAMES &&
                          stability_variance <= STABILITY_VARIANCE_THRESHOLD);

    if (state && state->have) {
        final_bpm = state->bpm;
        final_pair_idx = state->pair_idx;

        if (stable_enough) {
            int prior_next = state->pair_idx + 1;
            if (prior_next >= BPM_COUNT) prior_next = 0;
            float prior_pair_score = mean_scores[state->pair_idx] + mean_scores[prior_next];
            bool overtook_prior = (prior_pair_score <= 1e-12f) ||
                                  (accum_pair_score >= prior_pair_score * SWITCH_OVERTAKE_RATIO);
            if (overtook_prior) {
                state->bpm = accum_bpm;
                state->pair_idx = accum_pair_idx;
                final_bpm = state->bpm;
                final_pair_idx = state->pair_idx;
            }
        }
    } else if (state) {
        state->bpm = accum_bpm;
        state->pair_idx = accum_pair_idx;
        state->have = 1;
        final_bpm = state->bpm;
        final_pair_idx = state->pair_idx;
    }

    if (final_bpm_out) *final_bpm_out = final_bpm;
    if (final_pair_idx_out) *final_pair_idx_out = final_pair_idx;
}

static int estimate_best_phase_for_bpm(const float *frames,
                                       int num_frames,
                                       float frame_rate,
                                       float bpm,
                                       float *best_phase_out,
                                       float *best_score_out,
                                       float *phase_score_out,
                                       int *phase_count_out,
                                       int *best_phase_idx_out,
                                       float *step_frames_out)
{
    if (best_phase_out) *best_phase_out = 0.0f;
    if (best_score_out) *best_score_out = 0.0f;
    if (phase_count_out) *phase_count_out = 0;
    if (best_phase_idx_out) *best_phase_idx_out = -1;
    if (step_frames_out) *step_frames_out = 0.0f;
    if (phase_score_out) memset(phase_score_out, 0, MAX_PHASE_EVAL * sizeof(float));

    if (!frames || num_frames <= 2 || frame_rate <= 0.0f || bpm <= 1e-6f) {
        return 0;
    }

    float step = frame_rate * 60.0f / bpm;
    if (step <= 1e-6f || !isfinite(step)) return 0;

    int phase_count = PHASE_GRAPH_BINS;
    if (phase_count > MAX_PHASE_EVAL) phase_count = MAX_PHASE_EVAL;
    if (phase_count_out) *phase_count_out = phase_count;
    if (step_frames_out) *step_frames_out = step;

    float best_phase = 0.0f;
    float best_score = -HUGE_VALF;
    int best_phase_idx = -1;
    bool have_best = false;

    for (int phase_idx = 0; phase_idx < phase_count; ++phase_idx) {
        if (s_bpm_worker_stop) return 0;

        float phase = step * ((float)phase_idx / (float)phase_count);
        float total_energy = 0.0f;
        int valid_eval_count = 0;

        for (int b = 0; b < COMB_BANDS; ++b) {
            int n = 0;
            float sum = 0.0f;

            for (float pos = phase; pos < (float)(num_frames - 1); pos += step) {
                float x = ordered_band_at(frames, num_frames, pos, b);
                if (isfinite(x)) {
                    n++;
                    sum += x;
                }
            }

            if (n >= 1) {
                total_energy += sum / (float)n;
                valid_eval_count++;
            }
        }

        float avg_energy = (valid_eval_count > 0)
            ? (total_energy / (float)valid_eval_count)
            : 0.0f;
        if (phase_score_out) phase_score_out[phase_idx] = avg_energy;

        if (valid_eval_count > 0 && (!have_best || avg_energy > best_score)) {
            best_score = avg_energy;
            best_phase = phase;
            best_phase_idx = phase_idx;
            have_best = true;
        }

        if ((phase_idx & 15) == 15) taskYIELD();
        if ((phase_idx & 31) == 31) vTaskDelay(1);
    }

    if (best_phase_out) *best_phase_out = best_phase;
    if (best_score_out) *best_score_out = have_best ? best_score : 0.0f;
    if (best_phase_idx_out) *best_phase_idx_out = best_phase_idx;
    return have_best ? 1 : 0;
}

static void update_phase_hold_from_candidate(phase_hold_state_t *hold,
                                             float final_bpm,
                                             float candidate_phase_frac,
                                             int candidate_phase_idx,
                                             float candidate_score,
                                             const float *phase_score,
                                             int phase_count,
                                             float *accepted_phase_frac_out,
                                             int *accepted_phase_idx_out,
                                             float *accepted_score_out)
{
    if (!hold || !phase_score || phase_count <= 0) {
        if (accepted_phase_frac_out) *accepted_phase_frac_out = candidate_phase_frac;
        if (accepted_phase_idx_out) *accepted_phase_idx_out = candidate_phase_idx;
        if (accepted_score_out) *accepted_score_out = candidate_score;
        return;
    }

    if (!hold->have || fabsf(final_bpm - hold->bpm) > PHASE_LOCK_MAX_BPM_DIFF) {
        hold->have = 1;
        hold->bpm = final_bpm;
        hold->phase_frac = candidate_phase_frac;
        hold->phase_idx = candidate_phase_idx;
        hold->phase_score = candidate_score;
    } else {
        int held_idx_now = (int)floorf(hold->phase_frac * (float)phase_count);
        if (held_idx_now >= phase_count) held_idx_now = phase_count - 1;
        if (held_idx_now < 0) held_idx_now = 0;

        float held_score_now = phase_score[held_idx_now];
        if (!isfinite(held_score_now) || held_score_now <= 1e-12f) held_score_now = hold->phase_score;
        if (held_score_now <= 1e-12f) held_score_now = candidate_score;

        int phase_delta = abs(candidate_phase_idx - held_idx_now) % phase_count;
        bool close_phase = (phase_delta <= 1 || phase_count - phase_delta <= 1);
        if (close_phase || candidate_score >= held_score_now * PHASE_SWITCH_OVERTAKE_RATIO) {
            hold->phase_frac = candidate_phase_frac;
            hold->phase_idx = candidate_phase_idx;
            hold->phase_score = candidate_score;
        } else {
            hold->phase_idx = held_idx_now;
            hold->phase_score = held_score_now;
        }
        hold->bpm = final_bpm;
    }

    if (accepted_phase_frac_out) *accepted_phase_frac_out = hold->phase_frac;
    if (accepted_phase_idx_out) *accepted_phase_idx_out = hold->phase_idx;
    if (accepted_score_out) *accepted_score_out = hold->phase_score;
}

static void build_phase_curve(const float *phase_score, int phase_count, float *curve_out)
{
    if (!curve_out) return;
    if (!phase_score || phase_count <= 0) {
        memset(curve_out, 0, PHASE_SLOTS * sizeof(float));
        return;
    }

    if (phase_count == 1) {
        for (int i = 0; i < PHASE_SLOTS; ++i) curve_out[i] = phase_score[0];
        return;
    }

    for (int s = 0; s < PHASE_SLOTS; ++s) {
        float p = ((float)s / (float)PHASE_SLOTS) * (float)phase_count;
        while (p >= (float)phase_count) p -= (float)phase_count;
        int i0 = (int)floorf(p);
        int i1 = i0 + 1;
        if (i1 >= phase_count) i1 = 0;
        float frac = p - (float)i0;
        curve_out[s] = phase_score[i0] * (1.0f - frac) + phase_score[i1] * frac;
    }
}

static void handle_bpm_result(const bpm_worker_result_t *result)
{
    if (!result || !result->valid) {
        bpm_soft_decay_and_reset();
        return;
    }

    memcpy(bpm_scores, result->mean_norm_scores, sizeof(bpm_scores));
    memcpy(phase_curve, result->phase_curve, sizeof(phase_curve));
    if (fft_render_is_display_enabled()) {
        fft_render_update_tempo_spectrum(bpm_scores);
        fft_render_update_phase_curve(phase_curve, PHASE_SLOTS);
    }

    raw_cand_bpm = result->final_bpm;

    bool bpm_locked = false;
    float locked_bpm = 0.0f;
    fft_get_user_ctrl_locked(&bpm_locked, NULL, &locked_bpm, NULL);

    float selected_bpm = result->final_bpm;
    if (bpm_locked && locked_bpm > 0.5f) {
        selected_bpm = locked_bpm;
    }

    last_cand_bpm = selected_bpm;
    bpm_hz = selected_bpm / 60.0f;
    bpm_conf = result->confidence;
    bpm_phase = result->accepted_phase_frac;

    bool may_lock_phase = result->stable || bpm_locked;
    if (result->phase_valid && may_lock_phase && selected_bpm > 0.5f) {
        int64_t now_us = esp_timer_get_time();
        beat_clock_init_from_detected_beat(&s_beat_clock,
                                           selected_bpm,
                                           result->detected_beat_time_us,
                                           now_us);
        beat_phase = beat_clock_phase(&s_beat_clock, now_us);
    } else if (s_beat_clock.initialized && selected_bpm > 0.5f) {
        beat_clock_queue_bpm(&s_beat_clock, selected_bpm);
    }
}

static void poll_bpm_results(void)
{
    if (!s_bpm_result_queue) return;

    bpm_worker_result_t result;
    while (xQueueReceive(s_bpm_result_queue, &result, 0) == pdTRUE) {
        handle_bpm_result(&result);
    }
}

static void bpm_worker_main(void *arg)
{
    (void)arg;

    while (!s_bpm_worker_stop) {
        bpm_worker_job_t job = {0};
        if (xQueueReceive(s_bpm_job_queue, &job, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (s_bpm_worker_stop) break;

        bpm_worker_result_t result;
        memset(&result, 0, sizeof(result));

        compute_raw_comb_scores(s_bpm_job_frames,
                                job.num_frames,
                                job.frame_rate,
                                s_worker_raw_score);
        if (s_bpm_worker_stop) break;

        raw_scores_to_positive_inplace(s_worker_raw_score);
        bpm_baseline_correct(s_worker_raw_score);
        if (!normalize_scores_by_sum(s_worker_raw_score, s_worker_norm_score)) {
            s_bpm_worker_busy = false;
            xQueueOverwrite(s_bpm_result_queue, &result);
            continue;
        }

        memcpy(s_worker_norm_history[s_worker_norm_history_write],
               s_worker_norm_score,
               BPM_COUNT * sizeof(float));
        s_worker_norm_history_write = (s_worker_norm_history_write + 1) % STABILITY_FRAMES;
        if (s_worker_norm_history_count < STABILITY_FRAMES) s_worker_norm_history_count++;

        for (int idx = 0; idx < BPM_COUNT; ++idx) {
            float mean = 0.0f;
            for (int h = 0; h < s_worker_norm_history_count; ++h) {
                mean += s_worker_norm_history[h][idx];
            }
            s_worker_mean_norm_score[idx] = mean / (float)s_worker_norm_history_count;
        }

        float stability_variance = 0.0f;
        for (int idx = 0; idx < BPM_COUNT; ++idx) {
            float var = 0.0f;
            for (int h = 0; h < s_worker_norm_history_count; ++h) {
                float d = s_worker_norm_history[h][idx] - s_worker_mean_norm_score[idx];
                var += d * d;
            }
            stability_variance += var / (float)s_worker_norm_history_count;
        }

        float stability_confidence =
            STABILITY_VARIANCE_THRESHOLD /
            (STABILITY_VARIANCE_THRESHOLD + fmaxf(0.0f, stability_variance));
        if (stability_confidence < 0.0f) stability_confidence = 0.0f;
        if (stability_confidence > 1.0f) stability_confidence = 1.0f;

        int accum_pair_idx = 0;
        float accum_pair_score = -1.0f;
        for (int idx = 0; idx < BPM_COUNT; ++idx) {
            int next = idx + 1;
            if (next >= BPM_COUNT) next = 0;
            float pair_score = s_worker_mean_norm_score[idx] + s_worker_mean_norm_score[next];
            if (pair_score > accum_pair_score) {
                accum_pair_score = pair_score;
                accum_pair_idx = idx;
            }
        }

        if (accum_pair_score <= 1e-12f) {
            s_bpm_worker_busy = false;
            xQueueOverwrite(s_bpm_result_queue, &result);
            continue;
        }

        int accum_next = accum_pair_idx + 1;
        if (accum_next >= BPM_COUNT) accum_next = 0;
        int best_accum_bin = (s_worker_mean_norm_score[accum_next] >
                              s_worker_mean_norm_score[accum_pair_idx])
            ? accum_next
            : accum_pair_idx;
        float accum_bpm = (float)(FFT_CFG_BPM_MIN + best_accum_bin);

        float final_bpm = accum_bpm;
        int final_pair_idx = accum_pair_idx;
        update_final_bpm_from_accum(accum_bpm,
                                    accum_pair_idx,
                                    accum_pair_score,
                                    s_worker_mean_norm_score,
                                    stability_variance,
                                    s_worker_norm_history_count,
                                    &s_worker_bpm_hold,
                                    &final_bpm,
                                    &final_pair_idx);

        float best_phase_frames = 0.0f;
        float best_phase_score = 0.0f;
        int phase_count = 0;
        int best_phase_idx = -1;
        float step_frames = 0.0f;
        int phase_ok = estimate_best_phase_for_bpm(s_bpm_job_frames,
                                                   job.num_frames,
                                                   job.frame_rate,
                                                   final_bpm,
                                                   &best_phase_frames,
                                                   &best_phase_score,
                                                   s_worker_phase_score,
                                                   &phase_count,
                                                   &best_phase_idx,
                                                   &step_frames);

        result.valid = true;
        result.final_bpm = final_bpm;
        result.accum_bpm = accum_bpm;
        result.confidence = stability_confidence;
        result.stability_variance = stability_variance;
        result.stable = (s_worker_norm_history_count >= STABILITY_FRAMES &&
                         stability_variance <= STABILITY_VARIANCE_THRESHOLD);
        result.final_pair_idx = final_pair_idx;
        memcpy(result.mean_norm_scores,
               s_worker_mean_norm_score,
               BPM_COUNT * sizeof(float));

        if (phase_ok && phase_count > 0) {
            double history_start_us = (double)job.last_frame_time_us -
                                      (((double)job.num_frames - 1.0) /
                                       (double)job.frame_rate) * 1000000.0;
            double candidate_time_us = history_start_us +
                                       ((double)best_phase_frames /
                                        (double)job.frame_rate) * 1000000.0;
            double candidate_time_sec = candidate_time_us / 1000000.0;
            double candidate_beat = candidate_time_sec * (double)final_bpm / 60.0;
            float candidate_phase_frac = (float)(candidate_beat - floor(candidate_beat));

            float accepted_phase_frac = candidate_phase_frac;
            int accepted_phase_idx = best_phase_idx;
            float accepted_phase_score = best_phase_score;
            update_phase_hold_from_candidate(&s_worker_phase_hold,
                                             final_bpm,
                                             candidate_phase_frac,
                                             best_phase_idx,
                                             best_phase_score,
                                             s_worker_phase_score,
                                             phase_count,
                                             &accepted_phase_frac,
                                             &accepted_phase_idx,
                                             &accepted_phase_score);
            (void)accepted_phase_idx;
            (void)accepted_phase_score;

            double beat_period_us = 60000000.0 / (double)final_bpm;
            double phase_offset_us = (double)wrap_phase01(accepted_phase_frac) * beat_period_us;
            double now_us = (double)esp_timer_get_time();
            double nearest_beat = floor((now_us - phase_offset_us) / beat_period_us + 0.5);
            double detected_beat_us = phase_offset_us + nearest_beat * beat_period_us;

            result.phase_valid = true;
            result.accepted_phase_frac = wrap_phase01(accepted_phase_frac);
            result.detected_beat_time_us = (int64_t)llround(detected_beat_us);
            build_phase_curve(s_worker_phase_score, phase_count, result.phase_curve);
        }

        xQueueOverwrite(s_bpm_result_queue, &result);
        s_bpm_worker_busy = false;
    }

    s_bpm_worker_busy = false;
    s_bpm_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t bpm_worker_start(void)
{
    if (!s_bpm_job_queue) {
        s_bpm_job_queue = xQueueCreate(1, sizeof(bpm_worker_job_t));
        if (!s_bpm_job_queue) return ESP_ERR_NO_MEM;
    }
    if (!s_bpm_result_queue) {
        s_bpm_result_queue = xQueueCreate(1, sizeof(bpm_worker_result_t));
        if (!s_bpm_result_queue) return ESP_ERR_NO_MEM;
    }
    if (s_bpm_task) return ESP_OK;

    xQueueReset(s_bpm_job_queue);
    xQueueReset(s_bpm_result_queue);
    bpm_worker_reset_state();
    s_bpm_worker_busy = false;
    s_bpm_worker_stop = false;

    BaseType_t ok = xTaskCreatePinnedToCore(bpm_worker_main, "fft_bpm", 6144, NULL,
                                            BPM_WORKER_PRIO, &s_bpm_task, FFT_TASK_CORE);
#if CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    if (ok != pdPASS) {
        ok = xTaskCreatePinnedToCoreWithCaps(bpm_worker_main, "fft_bpm", 6144, NULL,
                                             BPM_WORKER_PRIO, &s_bpm_task, FFT_TASK_CORE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
#endif
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

static void bpm_worker_stop(void)
{
    if (!s_bpm_task) {
        s_bpm_worker_busy = false;
        return;
    }

    s_bpm_worker_stop = true;
    if (s_bpm_job_queue) {
        bpm_worker_job_t wake = {0};
        xQueueOverwrite(s_bpm_job_queue, &wake);
    }

    for (int i = 0; i < 50 && s_bpm_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_bpm_task) {
        vTaskDelete(s_bpm_task);
        s_bpm_task = NULL;
    }
    s_bpm_worker_busy = false;
    s_bpm_worker_stop = false;
    if (s_bpm_job_queue) xQueueReset(s_bpm_job_queue);
    if (s_bpm_result_queue) xQueueReset(s_bpm_result_queue);
}

static void submit_bpm_job(int64_t last_frame_time_us)
{
    if (!s_bpm_task || !s_bpm_job_queue || s_bpm_worker_busy) return;
    if (band_frame_count < 3) return;

    int num_frames = band_frame_count;
    if (num_frames > FFT_CFG_NOV_RING_FRAMES) num_frames = FFT_CFG_NOV_RING_FRAMES;
    copy_band_history_ordered(s_bpm_job_frames, num_frames);

    float frame_rate = (hop_rate_hz_est > 1.0f && isfinite(hop_rate_hz_est))
        ? hop_rate_hz_est
        : kHopRateNominalHz;
    bpm_worker_job_t job = {
        .num_frames = num_frames,
        .frame_rate = frame_rate,
        .last_frame_time_us = last_frame_time_us,
    };

    s_bpm_worker_busy = true;
    if (xQueueOverwrite(s_bpm_job_queue, &job) != pdTRUE) {
        s_bpm_worker_busy = false;
    }
}

// ------------------- Processing per FFT frame -------------------
static void process_fft_frame(void){
    int64_t t0 = esp_timer_get_time();
    // Update hop rate estimate using high-resolution timer
    if (last_frame_time_us != 0){
        int64_t dt_us = t0 - last_frame_time_us;
        if (dt_us > 500 && dt_us < 500000){ // clamp to reasonable range
            float inst_rate = 1e6f / (float)dt_us;
            hop_rate_hz_est = 0.9f * hop_rate_hz_est + 0.1f * inst_rate;
        }
    }
    last_frame_time_us = t0;

    // --------- FFT magnitude (full-resolution, used for all DSP) ---------
    float logmag[FFT_CFG_SIZE/2 + 1];
    const size_t ring_mask = FFT_CFG_SIZE - 1;
    const size_t ring_start = ring_write_pos; // oldest sample in the full ring

    for (int i = 0; i < FFT_CFG_SIZE; ++i){
        size_t ring_idx = (ring_start + (size_t)i) & ring_mask;
        fft_re[i] = sample[ring_idx] * window[i];
        fft_im[i] = 0.0f;
    }
    fft_radix2(fft_re, fft_im, FFT_CFG_SIZE);

    for (int k = 0; k <= FFT_CFG_SIZE/2; ++k){
        float r  = fft_re[k];
        float im = fft_im[k];
        float mag = sqrtf(r*r + im*im);
        logmag[k] = logf(1.0f + 10.0f * mag);
    }

    led_audio_raw_volume_set(recent_hop_rms());
    update_level_track(&s_level_overall, band_mean_logmag(logmag, 80.0f, 4800.0f));
    update_level_track(&s_level_low,     band_mean_logmag(logmag, 80.0f, 220.0f));
    update_level_track(&s_level_mid,     band_mean_logmag(logmag, 220.0f, 1400.0f));
    update_level_track(&s_level_high,    band_mean_logmag(logmag, 1400.0f, 4800.0f));
    led_audio_levels_set(s_level_overall.out,
                         s_level_low.out,
                         s_level_mid.out,
                         s_level_high.out);

    if (fft_render_is_display_enabled()){
        fft_render_push_spectrogram(logmag);
    }

    apply_band_suppression_to_recent_history();
    bool suppress_frame = false;
    if (band_suppress_future_frames > 0){
        suppress_frame = true;
        band_suppress_future_frames--;
    }

    float flux = push_band_frame_from_logmag(logmag, suppress_frame);
    float local = band_flux_local_mean(suppress_frame ? 0.0f : flux);
    float cleaned = fmaxf(0.0f, (suppress_frame ? 0.0f : flux) - local);
    if (fft_render_is_display_enabled()){
        fft_render_push_novelty_display(suppress_frame ? 0.0f : flux, local, cleaned);
    }

    if (tempo_update_countdown <= 0){
        if (tempo_update_hold > 0){
            tempo_update_hold--;
        } else {
            submit_bpm_job(t0);
        }
        tempo_update_countdown = kTempoUpdateIntervalFrames;
    } else {
        tempo_update_countdown--;
    }
    poll_bpm_results();

    // --------- Beat blinker ---------
    // Match blinker to the currently found BPM shown to the user.
    float target_bpm_hz = (last_cand_bpm > 0.5f) ? (last_cand_bpm / 60.0f) : 0.0f;
    bool beat_triggered = false;
    bool bpm_locked = false;
    bool beat_enabled = true;
    float phase_offset = 0.0f;
    fft_get_user_ctrl_locked(&bpm_locked, &beat_enabled, NULL, &phase_offset);
    bool beat_confident = bpm_locked ? (target_bpm_hz > 0.5f) : (bpm_conf >= kBlinkMinConf);
    float beat_target_phase = wrap_phase01(kBlinkPhaseTarget + phase_offset);

    int64_t now_us = esp_timer_get_time();
    beat_clock_update(&s_beat_clock, now_us);
    if (s_beat_clock.initialized && target_bpm_hz > 0.5f && beat_confident) {
        blink_bpm_hz = (s_beat_clock.bpm_current > 0.5f)
            ? (s_beat_clock.bpm_current / 60.0f)
            : target_bpm_hz;
        blink_rate_hz = blink_bpm_hz;
        if (blink_rate_hz < 0.1f) blink_rate_hz = 0.1f;

        float prev_phase = beat_phase;
        beat_phase = beat_clock_phase(&s_beat_clock, now_us);
        bpm_phase = beat_phase;
        if (phase_crossed_forward(prev_phase, beat_phase, beat_target_phase)) {
            if (beat_enabled && fft_render_is_display_enabled()){
                fft_render_trigger_flash(kFlashFrames);
            }
            beat_triggered = true;
        }
    } else {
        blink_bpm_hz  = 0.0f;
        blink_rate_hz = 0.0f;
        beat_phase    = 0.0f;
    }

    if (beat_triggered && beat_enabled && s_beat_queue){
        fft_beat_event_t evt = {
            .bpm = blink_bpm_hz * 60.0f,
            .confidence = bpm_conf
        };
        xQueueSendToBack(s_beat_queue, &evt, 0);
    }
    if (beat_triggered && beat_enabled && fft_led_export_enabled()){
        // Fixed orange pulse avoids hue flicker when confidence jitters.
        uint8_t red   = 255;
        uint8_t green = 96;
        led_trigger_beat(red, green, 0);
    }

    if (fft_render_is_display_enabled()){
        fft_render_packet_t pkt = {
            .bpm_est   = last_cand_bpm,
            .bpm_conf  = bpm_conf,
            .bpm_phase = bpm_phase
        };
        memcpy(pkt.logmag, logmag, sizeof(pkt.logmag));
        fft_render_submit(&pkt);
    }

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

static void fft_task(void *arg){
    (void)arg;
    i2s_chan_handle_t rx = audio_rx_handle();
    if (!rx){
        ESP_LOGE(TAG,"No RX handle");
        bpm_worker_stop();
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
        bpm_worker_stop();
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
            sample[ring_write_pos] = (float)v16 / 32768.0f;
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

// ------------------- Public API -------------------
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
    ESP_RETURN_ON_ERROR(bpm_worker_start(), TAG, "bpm worker");
    BaseType_t ok = xTaskCreatePinnedToCore(fft_task, "fft_vis", 12288, NULL,
                                            FFT_TASK_PRIO, &s_task, FFT_TASK_CORE);
#if CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    if (ok != pdPASS) {
        ok = xTaskCreatePinnedToCoreWithCaps(fft_task, "fft_vis", 12288, NULL,
                                             FFT_TASK_PRIO, &s_task, FFT_TASK_CORE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
#endif
    if (ok != pdPASS) {
        bpm_worker_stop();
    }
    return ok==pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void fft_visualizer_stop(void){
    if (!s_task) {
        bpm_worker_stop();
        return;
    }
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
    bpm_worker_stop();
    led_audio_raw_volume_set(0.0f);
}

bool fft_visualizer_running(void){
    return s_task != NULL;
}

void fft_visualizer_set_view(fft_view_t view){
    fft_render_set_view(view);
}

bool fft_receive_beat(fft_beat_event_t *evt, TickType_t timeout_ticks){
    if (!evt || !s_beat_queue) return false;
    return xQueueReceive(s_beat_queue, evt, timeout_ticks) == pdTRUE;
}

void fft_get_levels(fft_levels_t *out)
{
    if (!out) return;
    out->overall = s_level_overall.out;
    out->low = s_level_low.out;
    out->mid = s_level_mid.out;
    out->high = s_level_high.out;
}

void fft_get_sync_state(fft_sync_state_t *out)
{
    if (!out) return;
    bool bpm_locked = false;
    bool beat_enabled = true;
    float phase_offset = 0.0f;
    fft_get_user_ctrl_locked(&bpm_locked, &beat_enabled, NULL, &phase_offset);

    out->bpm = s_beat_clock.initialized ? last_cand_bpm : 0.0f;
    out->detected_bpm = raw_cand_bpm;
    out->confidence = bpm_conf;
    out->beat_phase = s_beat_clock.initialized ? beat_phase : 0.0f;
    out->trigger_phase = wrap_phase01(kBlinkPhaseTarget + phase_offset);
    out->phase_offset = phase_offset;
    out->bpm_locked = bpm_locked;
    out->beat_enabled = beat_enabled;
    out->running = fft_visualizer_running();
}

void fft_beat_phase_offset_add(float delta_cycles)
{
    portENTER_CRITICAL(&s_ctrl_mux);
    s_user_phase_offset = wrap_phase01(s_user_phase_offset + delta_cycles);
    portEXIT_CRITICAL(&s_ctrl_mux);
}

void fft_beat_lock_set(bool locked)
{
    portENTER_CRITICAL(&s_ctrl_mux);
    if (locked) {
        float bpm = (last_cand_bpm > 0.5f) ? last_cand_bpm : raw_cand_bpm;
        if (bpm > 0.5f) {
            s_user_locked_bpm = bpm;
            s_user_bpm_locked = true;
        }
    } else {
        s_user_bpm_locked = false;
    }
    portEXIT_CRITICAL(&s_ctrl_mux);
}

void fft_beat_lock_toggle(void)
{
    bool locked = false;
    portENTER_CRITICAL(&s_ctrl_mux);
    locked = s_user_bpm_locked;
    portEXIT_CRITICAL(&s_ctrl_mux);
    fft_beat_lock_set(!locked);
}

void fft_beat_enable_set(bool enabled)
{
    portENTER_CRITICAL(&s_ctrl_mux);
    s_user_beat_enabled = enabled;
    portEXIT_CRITICAL(&s_ctrl_mux);
}

void fft_beat_enable_toggle(void)
{
    portENTER_CRITICAL(&s_ctrl_mux);
    s_user_beat_enabled = !s_user_beat_enabled;
    portEXIT_CRITICAL(&s_ctrl_mux);
}

void fft_led_export_enable(bool enabled)
{
    portENTER_CRITICAL(&s_ctrl_mux);
    s_led_export_enabled = enabled;
    portEXIT_CRITICAL(&s_ctrl_mux);
}

bool fft_led_export_enabled(void)
{
    bool enabled = false;
    portENTER_CRITICAL(&s_ctrl_mux);
    enabled = s_led_export_enabled;
    portEXIT_CRITICAL(&s_ctrl_mux);
    return enabled;
}

void fft_set_display_enabled(bool enabled){
    fft_render_set_display_enabled(enabled);
}

void fft_copy_frame(uint8_t *dst_fb, size_t dst_len){
    fft_render_copy_frame(dst_fb, dst_len);
}

void fft_suppress_novelty_frames(int frames){
    if (frames < 1) frames = 1;
    if (frames > FFT_CFG_NOV_RING_FRAMES) frames = FFT_CFG_NOV_RING_FRAMES;
    if (frames > band_suppress_backfill_frames){
        band_suppress_backfill_frames = frames;
    }
    if (frames > band_suppress_future_frames){
        band_suppress_future_frames = frames;
    }
}

void fft_suppress_novelty_timed_ms(int backfill_ms, int future_frames){
    int backfill_frames = novelty_frames_from_ms(backfill_ms);
    if (future_frames < 0) future_frames = 0;
    if (future_frames > FFT_CFG_NOV_RING_FRAMES) future_frames = FFT_CFG_NOV_RING_FRAMES;

    if (backfill_frames > band_suppress_backfill_frames){
        band_suppress_backfill_frames = backfill_frames;
    }
    if (future_frames > band_suppress_future_frames){
        band_suppress_future_frames = future_frames;
    }
}
