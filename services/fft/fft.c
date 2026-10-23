#include "fft.h"
#include "fft_detector.h"
#include "fft_phase.h"
#include "fft_history.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "audio.h"
#include "audio_i2s.h"
#include "audio_player.h"
#include "microphone.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led.h"

static const char *TAG = "fft_vis";
#define CAPTURE_PRIORITY 4
#define ANALYSIS_PRIORITY 2
#define PHASE_PRIORITY 2
#define FFT_CORE 1
#define ANALYSIS_CORE 0
#define PHASE_CORE 0
#define FRAME_SECONDS ((double)DETECTOR_FFT_SIZE / DETECTOR_SAMPLE_RATE)
#define PHASE_HOP_FRAMES 8
_Static_assert(FFT_CFG_SAMPLE_RATE_HZ == DETECTOR_SAMPLE_RATE, "sample rate mismatch");
_Static_assert(FFT_CFG_SIZE == DETECTOR_FFT_SIZE && FFT_CFG_HOP_SAMPLES == DETECTOR_FFT_SIZE, "FFT mismatch");
_Static_assert(FFT_CFG_NOV_RING_FRAMES == DETECTOR_FRAMES, "history mismatch");
_Static_assert(FFT_CFG_BPM_MIN == DETECTOR_BPM_MIN && FFT_CFG_BPM_MAX == DETECTOR_BPM_MAX, "BPM range mismatch");

typedef struct {
    detector_control_t detector;
    uint32_t epoch, revision;
    bool beat_enabled, export_leds, offbeat;
    unsigned ratio;
    float offset;
} controls_t;

typedef struct {
    int count;
    double time;
    controls_t controls;
} analysis_job_t;

typedef struct {
    detector_result_t detector;
    uint32_t epoch, revision;
    double time;
    float milliseconds;
} analysis_result_t;

typedef struct {
    int count;
    double time;
    float bpm;
    uint32_t epoch, reset_revision;
} phase_job_t;

typedef struct {
    fft_phase_evidence_t evidence;
    double time;
    float bpm;
    float milliseconds;
    uint32_t epoch, reset_revision;
    bool valid;
} phase_result_t;

typedef struct { float fast, slow, peak, out; } level_track_t;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static controls_t s_controls = { .beat_enabled = true, .export_leds = true, .ratio = 1 };
static fft_sync_state_t s_status;
static fft_levels_t s_levels;
static TaskHandle_t s_capture_task, s_analysis_task, s_phase_task;
static QueueHandle_t s_jobs, s_results, s_phase_jobs, s_phase_results, s_beats;
static volatile bool s_stop_capture, s_stop_analysis, s_stop_phase;
// The capture task owns a binary job credit until it consumes a result.
static bool s_job_busy;
static bool s_phase_busy;
static EXT_RAM_BSS_ATTR float s_frames[DETECTOR_FRAMES][DETECTOR_BANDS];
static EXT_RAM_BSS_ATTR float s_snapshot[DETECTOR_FRAMES][DETECTOR_BANDS];
static EXT_RAM_BSS_ATTR float s_phase_frames[DETECTOR_FRAMES][DETECTOR_BANDS];
static EXT_RAM_BSS_ATTR detector_t s_detector;
static EXT_RAM_BSS_ATTR float s_re[DETECTOR_FFT_SIZE], s_im[DETECTOR_FFT_SIZE];
static EXT_RAM_BSS_ATTR float s_window[DETECTOR_FFT_SIZE];
static EXT_RAM_BSS_ATTR analysis_result_t s_current;
static detector_filter_t s_filter;
static int s_frame_count, s_frame_write, s_hop_count, s_phase_hop_count;
static int s_suppress_back, s_suppress_future;
static uint32_t s_capture_epoch;
static fft_phase_tracker_t s_phase_tracker;
static fft_phase_decision_t s_phase_decision;
static fft_beat_clock_t s_clock;
static uint32_t s_phase_reset_revision;
static unsigned s_clock_ratio = 1;
static bool s_clock_offbeat;
static float s_clock_offset;
static double s_last_emit;
static uint32_t s_skipped, s_capture_errors;
static double s_analysis_interval;
static level_track_t s_level[4];

static float wrap(float p) { return p - floorf(p); }

static controls_t controls_get(void)
{
    portENTER_CRITICAL(&s_mux);
    controls_t c = s_controls;
    portEXIT_CRITICAL(&s_mux);
    return c;
}

static void clear_capture(void)
{
    s_frame_count = s_frame_write = s_hop_count = s_phase_hop_count = 0;
    memset(&s_filter, 0, sizeof(s_filter));
    memset(&s_current, 0, sizeof(s_current));
    fft_phase_tracker_reset(&s_phase_tracker);
    memset(&s_phase_decision, 0, sizeof(s_phase_decision));
    fft_beat_clock_reset(&s_clock);
    s_last_emit = 0;
    if (s_beats) xQueueReset(s_beats);
    fft_history_reset();
    portENTER_CRITICAL(&s_mux);
    s_status.bpm = s_status.held_bpm = s_status.detected_bpm = 0;
    s_status.confidence = s_status.decision_confidence = s_status.evidence = s_status.lost_seconds = 0;
    s_status.beat_phase = 0;
    portEXIT_CRITICAL(&s_mux);
}

static void invalidate_capture(void)
{
    portENTER_CRITICAL(&s_mux);
    ++s_controls.epoch;
    ++s_controls.revision;
    s_capture_epoch = s_controls.epoch;
    portEXIT_CRITICAL(&s_mux);
    // A DMA gap invalidates comb evidence, not the song. Keep the accepted
    // tempo/clock running while rebuilding contiguous audio, like the PC hold.
    s_frame_count = s_frame_write = s_hop_count = s_phase_hop_count = 0;
    memset(&s_filter, 0, sizeof(s_filter));
    s_current.detector.trusted = false;
    s_current.detector.history_coverage = 0;
    s_current.detector.confidence = s_current.detector.evidence = 0;
    fft_history_update_detector(&s_current.detector);
    ESP_LOGW(TAG, "Capture gap: rebuilding audio history; retaining BPM %.0f", s_current.detector.held_bpm);
    led_audio_raw_volume_set(0);
    led_audio_levels_set(0, 0, 0, 0);
    portENTER_CRITICAL(&s_mux);
    memset(&s_levels, 0, sizeof(s_levels));
    portEXIT_CRITICAL(&s_mux);
}

static void fft_radix2(float *re, float *im)
{
    const int n = DETECTOR_FFT_SIZE;
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j &= ~bit;
        j |= bit;
        if (i < j) {
            float r = re[i], v = im[i];
            re[i] = re[j]; im[i] = im[j]; re[j] = r; im[j] = v;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float angle = -6.28318530718f / len;
        float dc = cosf(angle), ds = sinf(angle);
        for (int i = 0; i < n; i += len) {
            float c = 1, s = 0;
            for (int j = 0; j < len / 2; ++j) {
                int u = i + j, v = u + len / 2;
                float r = c * re[v] - s * im[v], m = c * im[v] + s * re[v];
                re[v] = re[u] - r; im[v] = im[u] - m;
                re[u] += r; im[u] += m;
                float nc = c * dc - s * ds;
                s = c * ds + s * dc; c = nc;
            }
        }
    }
}

static float band_level(const float *logmag, float low, float high)
{
    int lo = (int)lroundf(low * DETECTOR_FFT_SIZE / DETECTOR_SAMPLE_RATE);
    int hi = (int)lroundf(high * DETECTOR_FFT_SIZE / DETECTOR_SAMPLE_RATE);
    float sum = 0;
    for (int i = lo; i <= hi; ++i) sum += logmag[i];
    return sum / (hi - lo + 1);
}

static float level_update(level_track_t *t, float raw)
{
    // Original 62.5 Hz envelope constants retimed to the 43.07 Hz capture hop:
    // alpha_new = 1 - (1 - alpha_old)^(62.5 / 43.07).
    t->fast += 0.47673795f * (raw - t->fast);
    t->slow += 0.06463751f * (raw - t->slow);
    float delta = fmaxf(0, t->fast - t->slow);
    t->peak = fmaxf(0.05f, fmaxf(delta, t->peak * 0.98841101f + delta * 0.01158899f));
    return t->out = fminf(1, delta / t->peak);
}

static void analysis_cooperate(void *ctx)
{
    int64_t *slice_start = ctx;
    if (esp_timer_get_time() - *slice_start >= 40000) {
        vTaskDelay(1);
        *slice_start = esp_timer_get_time();
    }
}

static void analysis_task(void *arg)
{
    (void)arg;
    uint32_t epoch = UINT32_MAX;
    analysis_job_t job;
    while (!s_stop_analysis && xQueueReceive(s_jobs, &job, portMAX_DELAY) == pdTRUE) {
        if (s_stop_analysis) break;
        if (epoch != job.controls.epoch) {
            if (epoch == UINT32_MAX) detector_reset(&s_detector);
            else detector_discontinuity(&s_detector);
            epoch = job.controls.epoch;
        }
        analysis_result_t result = { .epoch = job.controls.epoch, .revision = job.controls.revision,
                                     .time = job.time };
        float raw[DETECTOR_BPM_COUNT], normalized[DETECTOR_BPM_COUNT];
        int64_t start = esp_timer_get_time(), slice = start;
        bool cancelled = false;
        for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
            if (s_stop_analysis || controls_get().epoch != epoch) { cancelled = true; break; }
            raw[i] = detector_comb_score_cooperative(&s_snapshot[0][0], job.count,
                         DETECTOR_BPM_MIN + i, analysis_cooperate, &slice);
        }
        if (s_stop_analysis) break;
        if (!cancelled) {
            detector_normalize(raw, normalized);
            detector_update(&s_detector, normalized, job.time, job.controls.detector, &result.detector);
        }
        result.milliseconds = (float)(esp_timer_get_time() - start) / 1000;
        xQueueOverwrite(s_results, &result);
    }
    s_analysis_task = NULL;
    vTaskDelete(NULL);
}

static void poll_analysis(controls_t c)
{
    analysis_result_t result;
    if (xQueueReceive(s_results, &result, 0) != pdTRUE) return;
    s_job_busy = false;
    if (result.epoch != c.epoch || result.revision != c.revision) return;
    float live_phase[DETECTOR_PHASE_BINS];
    bool had_phase = s_current.detector.phase_valid &&
        s_current.detector.held_bpm == result.detector.held_bpm && !result.detector.stopped;
    float candidate_phase = s_current.detector.candidate_phase;
    float accepted_phase = s_current.detector.accepted_phase;
    double beat_time = s_current.detector.beat_time;
    if (had_phase) memcpy(live_phase, s_current.detector.phase, sizeof(live_phase));
    s_analysis_interval = s_current.time > 0 ? result.time - s_current.time : 0;
    s_current = result;
    detector_result_t *d = &s_current.detector;
    if (had_phase) {
        memcpy(d->phase, live_phase, sizeof(live_phase));
        d->phase_valid = true;
        d->candidate_phase = candidate_phase;
        d->accepted_phase = accepted_phase;
        d->beat_time = beat_time;
    }
    if (d->stopped) {
        ESP_LOGI(TAG, "No credible pulse for 40 s; seeking a new BPM");
        // Match localFFT: stop the accepted clock, but retain audio/score history
        // so reacquisition does not start from an empty four-second window.
        fft_phase_tracker_lose(&s_phase_tracker, result.time);
        fft_beat_clock_reset(&s_clock);
        memset(&s_phase_decision, 0, sizeof(s_phase_decision));
        s_last_emit = 0;
        fft_history_update_detector(d);
        return;
    }
    fft_history_update_detector(d);
}

static double phase_clock_anchor(controls_t c)
{
    double period = 60.0 / s_phase_decision.precise_bpm;
    double shift = c.offset + ((c.offbeat && c.ratio == 1) ? 0.5 : 0.0);
    return s_phase_decision.anchor + shift * period;
}

static void phase_task(void *arg)
{
    (void)arg;
    phase_job_t job;
    while (!s_stop_phase && xQueueReceive(s_phase_jobs, &job, portMAX_DELAY) == pdTRUE) {
        if (s_stop_phase) break;
        phase_result_t result = { .time = job.time, .bpm = job.bpm, .epoch = job.epoch,
                                 .reset_revision = job.reset_revision };
        double rate = (double)DETECTOR_SAMPLE_RATE / DETECTOR_FFT_SIZE;
        double start_time = job.time - (job.count - 1) / rate;
        int64_t start = esp_timer_get_time();
        result.valid = fft_phase_evidence(&s_phase_frames[0][0], job.count, rate,
                                          job.bpm, start_time, &result.evidence);
        result.milliseconds = (float)(esp_timer_get_time() - start) / 1000;
        xQueueOverwrite(s_phase_results, &result);
    }
    s_phase_task = NULL;
    vTaskDelete(NULL);
}

static void poll_phase(controls_t c)
{
    phase_result_t result;
    if (xQueueReceive(s_phase_results, &result, 0) != pdTRUE) return;
    s_phase_busy = false;
    detector_result_t *d = &s_current.detector;
    if (!result.valid || result.epoch != c.epoch ||
        result.reset_revision != c.detector.reset_revision || d->held_bpm <= 0 ||
        fabsf(result.bpm - d->held_bpm) >= 0.5f) return;
    double frame_rate = (double)DETECTOR_SAMPLE_RATE / DETECTOR_FFT_SIZE;
    double required_coverage = 8.0 * DETECTOR_REFERENCE_HOP_FRAMES / frame_rate;
    bool certain = d->history_coverage >= required_coverage - 1e-6 && d->trusted &&
                   d->confidence >= 1.0f && fabs(d->candidate_bpm - d->held_bpm) < 0.5;
    s_phase_decision = fft_phase_update(&s_phase_tracker, &result.evidence, d->held_bpm,
                                        result.time, certain, c.detector.phase_lock);
    for (int i = 0; i < DETECTOR_PHASE_BINS; ++i)
        d->phase[i] = (float)result.evidence.score[i];
    d->phase_valid = s_phase_decision.valid;
    if (!s_phase_decision.valid) return;
    d->candidate_phase = result.evidence.raw_best_bin / (float)DETECTOR_PHASE_BINS;
    d->accepted_phase = (s_phase_decision.family +
        s_phase_decision.side * FFT_PHASE_FAMILY_BINS) / (float)DETECTOR_PHASE_BINS;
    d->beat_time = s_phase_decision.anchor;
    double visual_bpm = s_phase_decision.precise_bpm * c.ratio;
    double anchor = phase_clock_anchor(c);
    bool configuration_changed = s_clock_ratio != c.ratio ||
        s_clock_offbeat != c.offbeat || s_clock_offset != c.offset;
    if (configuration_changed && s_clock.initialized) {
        fft_beat_clock_init(&s_clock, visual_bpm, anchor, esp_timer_get_time() / 1e6);
    } else if (certain) {
        if (!s_clock.initialized || s_phase_decision.acquired ||
            s_phase_decision.restored || configuration_changed)
            fft_beat_clock_init(&s_clock, visual_bpm, anchor, result.time);
        else if (s_phase_decision.family_switched || s_phase_decision.corrected)
            fft_beat_clock_schedule(&s_clock, visual_bpm, anchor);
    }
    s_clock_ratio = c.ratio;
    s_clock_offbeat = c.offbeat;
    s_clock_offset = c.offset;
    fft_history_update_detector(d);
}

static void submit_phase(double time, controls_t c)
{
    detector_result_t *d = &s_current.detector;
    if (s_phase_busy || d->held_bpm <= 0 || s_frame_count < 3) return;
    int oldest = (s_frame_write - s_frame_count + DETECTOR_FRAMES) % DETECTOR_FRAMES;
    for (int i = 0; i < s_frame_count; ++i)
        memcpy(s_phase_frames[i], s_frames[(oldest + i) % DETECTOR_FRAMES],
               sizeof(s_phase_frames[i]));
    phase_job_t job = { .count = s_frame_count, .time = time,
                        .bpm = d->held_bpm, .epoch = c.epoch,
                        .reset_revision = c.detector.reset_revision };
    s_phase_busy = xQueueSend(s_phase_jobs, &job, 0) == pdTRUE;
}

static bool submit_analysis(double time, controls_t c)
{
    if (s_job_busy || s_frame_count < 3) return false;
    int oldest = (s_frame_write - s_frame_count + DETECTOR_FRAMES) % DETECTOR_FRAMES;
    for (int i = 0; i < s_frame_count; ++i)
        memcpy(s_snapshot[i], s_frames[(oldest + i) % DETECTOR_FRAMES], sizeof(s_snapshot[i]));
    analysis_job_t job = { .count = s_frame_count, .time = time, .controls = c };
    s_job_busy = xQueueSend(s_jobs, &job, 0) == pdTRUE;
    return s_job_busy;
}

static void emit_and_publish(double now, controls_t c)
{
    detector_result_t *d = &s_current.detector;
    float base = c.detector.hard_lock ? c.detector.forced_bpm : d->held_bpm;
    fft_beat_clock_update(&s_clock, now);
    float bpm = s_clock.initialized ? (float)s_clock.bpm : 0;
    float phase = 0, target = 0;
    if (bpm > 0) {
        double period = s_clock.period;
        double cycle = (now - s_clock.beat_start) / period;
        phase = (float)(cycle - floor(cycle));
        double trigger = s_clock.beat_start;
        // Emit the newest crossed boundary once. A delayed capture iteration may
        // make a flash late, but must not silently delete the beat.
        if (trigger > s_last_emit && now - s_last_emit >= period * 0.5) {
            s_last_emit = now;
            if (c.beat_enabled) {
                fft_beat_event_t evt = { .bpm = bpm, .confidence = fminf(1, d->confidence) };
                xQueueSend(s_beats, &evt, 0);
                if (c.export_leds) led_trigger_beat(255, 96, 0);
            }
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_status.bpm = bpm;
    s_status.held_bpm = base;
    s_status.detected_bpm = d->candidate_bpm;
    s_status.confidence = fminf(1, d->confidence);
    s_status.decision_confidence = d->confidence;
    s_status.evidence = d->evidence;
    s_status.lost_seconds = d->lost_seconds;
    s_status.analysis_ms = s_current.milliseconds;
    s_status.skipped_jobs = s_skipped;
    s_status.capture_errors = s_capture_errors;
    s_status.beat_phase = phase;
    s_status.trigger_phase = target;
    portEXIT_CRITICAL(&s_mux);
}

static void process_frame(double time)
{
    int64_t start = esp_timer_get_time();
    controls_t c = controls_get();
    if (c.epoch != s_capture_epoch) { s_capture_epoch = c.epoch; clear_capture(); }
    if (c.detector.reset_revision != s_phase_reset_revision) {
        fft_phase_tracker_reset(&s_phase_tracker);
        fft_beat_clock_reset(&s_clock);
        memset(&s_phase_decision, 0, sizeof(s_phase_decision));
        s_phase_reset_revision = c.detector.reset_revision;
        s_last_emit = 0;
        s_current.detector.held_bpm = 0;
        s_current.detector.phase_valid = false;
        s_current.detector.trusted = false;
        s_current.detector.confidence = 0;
        if (s_beats) xQueueReset(s_beats);
    }
    float power = 0;
    for (int i = 0; i < DETECTOR_FFT_SIZE; ++i) {
        power += s_re[i] * s_re[i];
        s_re[i] *= s_window[i]; s_im[i] = 0;
    }
    fft_radix2(s_re, s_im);
    fft_history_packet_t packet = {0};
    for (int k = 0; k <= DETECTOR_FFT_SIZE / 2; ++k) {
        float magnitude = sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]);
        packet.logmag[k] = log1pf(10 * magnitude);
        if (k < DETECTOR_FFT_SIZE / 2)
        {
            int row = k * FFT_HISTORY_ROWS / (DETECTOR_FFT_SIZE / 2);
            if (magnitude > packet.magnitudes[row]) packet.magnitudes[row] = magnitude;
        }
    }
    led_audio_raw_volume_set(fminf(1, sqrtf(power / DETECTOR_FFT_SIZE) * 12));
    fft_levels_t levels = {
        .overall = level_update(&s_level[0], band_level(packet.logmag, 80, 4800)),
        .low = level_update(&s_level[1], band_level(packet.logmag, 80, 220)),
        .mid = level_update(&s_level[2], band_level(packet.logmag, 220, 1400)),
        .high = level_update(&s_level[3], band_level(packet.logmag, 1400, 4800)),
    };
    portENTER_CRITICAL(&s_mux);
    s_levels = levels;
    int back = s_suppress_back;
    s_suppress_back = 0;
    bool suppress = s_suppress_future > 0;
    if (suppress) --s_suppress_future;
    portEXIT_CRITICAL(&s_mux);
    led_audio_levels_set(levels.overall, levels.low, levels.mid, levels.high);
    if (back > s_frame_count) back = s_frame_count;
    if (back > 0) {
        int source = (s_frame_write - back - 1 + DETECTOR_FRAMES) % DETECTOR_FRAMES;
        for (int i = 1; i <= back; ++i) {
            int slot = (s_frame_write - i + DETECTOR_FRAMES) % DETECTOR_FRAMES;
            if (back < s_frame_count) memcpy(s_frames[slot], s_frames[source], sizeof(s_frames[slot]));
            else memset(s_frames[slot], 0, sizeof(s_frames[slot]));
        }
    }
    detector_filter_bands(&s_filter, packet.logmag, suppress, s_frames[s_frame_write]);
    s_frame_write = (s_frame_write + 1) % DETECTOR_FRAMES;
    if (s_frame_count < DETECTOR_FRAMES) ++s_frame_count;
    poll_analysis(c);
    poll_phase(c);
    c = controls_get();
    if (c.epoch != s_capture_epoch) { s_capture_epoch = c.epoch; clear_capture(); }
    if (s_hop_count < DETECTOR_HOP_FRAMES) ++s_hop_count;
    if (s_hop_count >= DETECTOR_HOP_FRAMES) {
        if (submit_analysis(time, c)) s_hop_count = 0;
        else ++s_skipped;
    }
    if (++s_phase_hop_count >= PHASE_HOP_FRAMES) {
        s_phase_hop_count = 0;
        submit_phase(time, c);
    }
    emit_and_publish(esp_timer_get_time() / 1e6, c);
    // Keep both histories on the audio clock, including while another app is open.
    fft_get_sync_state(&packet.status);
    packet.decision = s_current.detector.confidence;
    // Phase evidence uses absolute nominal-BPM coordinates. Using the steered
    // clock BPM here makes the graph crawl even when its peaks are stationary.
    double origin_phase = s_clock.initialized && s_phase_decision.valid &&
                          s_current.detector.held_bpm > 0
        ? s_clock.beat_start * s_current.detector.held_bpm / 60.0 : 0;
    packet.clock_origin = (float)(origin_phase - floor(origin_phase));
    packet.time = time;
    fft_history_submit(&packet);
    float load = (float)(esp_timer_get_time() - start) / (FRAME_SECONDS * 10000);
    portENTER_CRITICAL(&s_mux);
    s_status.capture_load += 0.05f * (load - s_status.capture_load);
    portEXIT_CRITICAL(&s_mux);
}

static void capture_task(void *arg)
{
    int32_t *input = arg;
    i2s_chan_handle_t rx = audio_i2s_rx_handle();
    uint32_t clock_revision = audio_i2s_get_clock_revision();
    uint32_t overflows = audio_i2s_get_rx_overflows();
    uint64_t samples = 0;
    double origin = 0;
    int fill = 0, slot = -1;
    bool suspended = false;
    int64_t report_time = esp_timer_get_time();
    while (!s_stop_capture) {
        uint32_t revision = audio_i2s_get_clock_revision();
        bool unavailable = audio_i2s_get_rate() != DETECTOR_SAMPLE_RATE || mic_rec_is_recording();
        if (unavailable || revision != clock_revision) {
            if (!suspended) { invalidate_capture(); suspended = true; }
            clock_revision = revision;
            fill = 0; samples = 0;
            if (unavailable) {
                // Playback/recording own the shared clock while active. Restore
                // mic capture only after they finish, never during re-clocking.
                if (audio_i2s_get_rate() > 0 && !audio_output_active() &&
                    !audio_player_is_active() && !mic_rec_is_recording()) {
                    if (audio_i2s_set_rate(DETECTOR_SAMPLE_RATE) == ESP_OK) (void)audio_i2s_enable_rx();
                }
                emit_and_publish(esp_timer_get_time() / 1e6, controls_get());
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
        }
        suspended = false;
        size_t bytes = 0;
        esp_err_t err = i2s_channel_read(rx, input, DETECTOR_FFT_SIZE * 2 * sizeof(*input), &bytes, 100);
        // Never concatenate partial frames across a failed read or clock change.
        uint32_t current_overflows = audio_i2s_get_rx_overflows();
        if ((err != ESP_OK && err != ESP_ERR_TIMEOUT) || !bytes ||
            current_overflows != overflows ||
            revision != audio_i2s_get_clock_revision()) {
            ++s_capture_errors;
            overflows = current_overflows;
            if (fill || samples) invalidate_capture();
            fill = 0; samples = 0;
            if ((err == ESP_ERR_INVALID_STATE || !bytes) && audio_i2s_get_rate() > 0 && !audio_output_active() &&
                !audio_player_is_active() && !mic_rec_is_recording()) {
                if (audio_i2s_set_rate(DETECTOR_SAMPLE_RATE) == ESP_OK) (void)audio_i2s_enable_rx();
            }
            emit_and_publish(esp_timer_get_time() / 1e6, controls_get());
            vTaskDelay(1);
            continue;
        }
        int n = bytes / (2 * sizeof(*input));
        if (!samples) origin = esp_timer_get_time() / 1e6 - n / (double)DETECTOR_SAMPLE_RATE;
        if (slot < 0) {
            double left = 0, right = 0;
            for (int i = 0; i < n; ++i) { left += fabs((double)input[2*i]); right += fabs((double)input[2*i+1]); }
            slot = right > left;
            ESP_LOGI(TAG, "Mic %s; 44.1 kHz, 4 s treble comb, 88-frame BPM / 8-frame phase hops",
                     slot ? "RIGHT" : "LEFT");
        }
        for (int i = 0; i < n; ++i) {
            // INMP441 signed left-aligned PCM. Keep old 4x gain, saturate instead of wrapping int16.
            s_re[fill++] = fmaxf(-1, fminf(1, input[2*i + slot] / 536870912.0f));
            ++samples;
            if (fill == DETECTOR_FFT_SIZE) {
                process_frame(origin + (samples - 1) / (double)DETECTOR_SAMPLE_RATE);
                fill = 0;
            }
        }
        if (esp_timer_get_time() - report_time >= 5000000) {
            fft_sync_state_t status;
            fft_get_sync_state(&status);
            ESP_LOGI(TAG, "capture %.0f%% worker %.0fms cadence %.2fs age %.2fs overdue-frames %lu errors %lu held %.0f evidence %.2f lost %.1fs",
                     status.capture_load, status.analysis_ms, s_analysis_interval,
                     s_current.time > 0 ? esp_timer_get_time() / 1e6 - s_current.time : 0,
                     (unsigned long)s_skipped,
                     (unsigned long)s_capture_errors, status.held_bpm, status.evidence, status.lost_seconds);
            ESP_LOGI(TAG, "internal heap %u; stack headroom capture %u worker %u bytes",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)uxTaskGetStackHighWaterMark(NULL),
                     s_analysis_task ? (unsigned)uxTaskGetStackHighWaterMark(s_analysis_task) : 0);
            report_time = esp_timer_get_time();
        }
    }
    free(input);
    s_capture_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t fft_visualizer_start(void)
{
    if (s_capture_task) return ESP_OK;
    if (mic_rec_is_recording() || audio_player_is_active() || audio_output_active()) return ESP_ERR_INVALID_STATE;
    if (!s_jobs) s_jobs = xQueueCreate(1, sizeof(analysis_job_t));
    if (!s_results) s_results = xQueueCreate(1, sizeof(analysis_result_t));
    if (!s_phase_jobs) s_phase_jobs = xQueueCreate(1, sizeof(phase_job_t));
    if (!s_phase_results) s_phase_results = xQueueCreate(1, sizeof(phase_result_t));
    if (!s_beats) s_beats = xQueueCreate(4, sizeof(fft_beat_event_t));
    if (!s_jobs || !s_results || !s_phase_jobs || !s_phase_results || !s_beats)
        return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(audio_i2s_set_rate(DETECTOR_SAMPLE_RATE), TAG, "set rate");
    ESP_RETURN_ON_ERROR(audio_i2s_enable_rx(), TAG, "enable rx");
    ESP_RETURN_ON_ERROR(led_init(), TAG, "LED init");
    ESP_RETURN_ON_ERROR(fft_history_init(), TAG, "history init");
    // i2s_channel_read copies from its own DMA buffers into this staging buffer.
    // Keep it in PSRAM so internal RAM remains available for task stacks.
    int32_t *input = heap_caps_malloc(DETECTOR_FFT_SIZE * 2 * sizeof(*input),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!input) return ESP_ERR_NO_MEM;
    for (int i = 0; i < DETECTOR_FFT_SIZE; ++i)
        s_window[i] = 0.5f * (1 - cosf(6.28318530718f * i / (DETECTOR_FFT_SIZE - 1)));
    xQueueReset(s_jobs); xQueueReset(s_results);
    xQueueReset(s_phase_jobs); xQueueReset(s_phase_results);
    s_job_busy = s_phase_busy = false;
    s_stop_capture = s_stop_analysis = s_stop_phase = false;
    s_skipped = s_capture_errors = 0;
    s_analysis_interval = 0;
    memset(s_level, 0, sizeof(s_level));
    fft_detector_reset();
    s_capture_epoch = controls_get().epoch;
    clear_capture();
    if (xTaskCreatePinnedToCore(analysis_task, "fft_bpm", 6144, NULL,
                               ANALYSIS_PRIORITY, &s_analysis_task, ANALYSIS_CORE) != pdPASS) {
        free(input); return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(phase_task, "fft_phase", 4096, NULL,
                               PHASE_PRIORITY, &s_phase_task, PHASE_CORE) != pdPASS) {
        free(input); fft_visualizer_stop(); return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(capture_task, "fft_vis", 8192, input,
                               CAPTURE_PRIORITY, &s_capture_task, FFT_CORE) != pdPASS) {
        free(input); fft_visualizer_stop(); return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void fft_visualizer_stop(void)
{
    s_stop_capture = s_stop_analysis = s_stop_phase = true;
    if (s_jobs) { analysis_job_t wake = {0}; xQueueOverwrite(s_jobs, &wake); }
    if (s_phase_jobs) { phase_job_t wake = {0}; xQueueOverwrite(s_phase_jobs, &wake); }
    // Bounded I2S reads and per-BPM cancellation make cooperative shutdown safe.
    while (s_capture_task || s_analysis_task || s_phase_task) vTaskDelay(1);
    clear_capture();
    led_audio_raw_volume_set(0);
    led_audio_levels_set(0, 0, 0, 0);
    portENTER_CRITICAL(&s_mux);
    memset(&s_levels, 0, sizeof(s_levels));
    portEXIT_CRITICAL(&s_mux);
}

bool fft_visualizer_running(void) { return s_capture_task != NULL; }
bool fft_receive_beat(fft_beat_event_t *evt, TickType_t timeout)
{ return evt && s_beats && xQueueReceive(s_beats, evt, timeout) == pdTRUE; }

void fft_get_levels(fft_levels_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_mux); *out = s_levels; portEXIT_CRITICAL(&s_mux);
}

void fft_get_sync_state(fft_sync_state_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_mux);
    *out = s_status;
    out->bpm_locked = s_controls.detector.hard_lock;
    out->phase_locked = s_controls.detector.phase_lock;
    out->beat_enabled = s_controls.beat_enabled;
    out->phase_offset = s_controls.offset;
    out->ratio = s_controls.ratio;
    out->offbeat = s_controls.offbeat;
    out->running = s_capture_task != NULL;
    portEXIT_CRITICAL(&s_mux);
}

void fft_detector_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    ++s_controls.detector.reset_revision;
    ++s_controls.detector.force_revision;
    ++s_controls.revision;
    s_controls.detector.hard_lock = s_controls.detector.phase_lock = false;
    s_controls.ratio = 1;
    s_controls.offbeat = false;
    s_controls.offset = 0;
    s_status.bpm = s_status.held_bpm = s_status.detected_bpm = 0;
    s_status.confidence = s_status.decision_confidence = s_status.evidence = 0;
    s_status.lost_seconds = 0;
    portEXIT_CRITICAL(&s_mux);
}

void fft_beat_lock_set(bool locked)
{
    portENTER_CRITICAL(&s_mux);
    if (!locked || s_status.detected_bpm >= DETECTOR_BPM_MIN) {
        s_controls.detector.hard_lock = locked;
        if (locked) s_controls.detector.forced_bpm = s_status.detected_bpm;
        ++s_controls.detector.force_revision; ++s_controls.revision;
    }
    portEXIT_CRITICAL(&s_mux);
}

void fft_beat_lock_toggle(void) { fft_beat_lock_set(!controls_get().detector.hard_lock); }
void fft_phase_lock_toggle(void)
{
    portENTER_CRITICAL(&s_mux);
    if (s_status.bpm > 0 || s_controls.detector.phase_lock) {
        s_controls.detector.phase_lock = !s_controls.detector.phase_lock;
        ++s_controls.revision;
    }
    portEXIT_CRITICAL(&s_mux);
}
void fft_beat_ratio_set(unsigned ratio)
{
    if (ratio < 1) ratio = 1;
    if (ratio > 3) ratio = 3;
    portENTER_CRITICAL(&s_mux); s_controls.ratio = ratio; portEXIT_CRITICAL(&s_mux);
}
void fft_beat_ratio_cycle(void) { fft_beat_ratio_set(controls_get().ratio % 3 + 1); }
void fft_beat_offbeat_set(bool offbeat)
{ portENTER_CRITICAL(&s_mux); s_controls.offbeat = offbeat; portEXIT_CRITICAL(&s_mux); }
void fft_beat_offbeat_toggle(void) { fft_beat_offbeat_set(!controls_get().offbeat); }
void fft_beat_phase_offset_add(float delta)
{ portENTER_CRITICAL(&s_mux); s_controls.offset = wrap(s_controls.offset + delta); portEXIT_CRITICAL(&s_mux); }
void fft_beat_enable_set(bool enabled)
{ portENTER_CRITICAL(&s_mux); s_controls.beat_enabled = enabled; portEXIT_CRITICAL(&s_mux); }
void fft_beat_enable_toggle(void) { fft_beat_enable_set(!controls_get().beat_enabled); }
void fft_led_export_enable(bool enabled)
{ portENTER_CRITICAL(&s_mux); s_controls.export_leds = enabled; portEXIT_CRITICAL(&s_mux); }
bool fft_led_export_enabled(void) { return controls_get().export_leds; }

void fft_suppress_novelty_frames(int frames)
{
    if (frames < 1) frames = 1;
    if (frames > DETECTOR_FRAMES) frames = DETECTOR_FRAMES;
    portENTER_CRITICAL(&s_mux);
    if (frames > s_suppress_back) s_suppress_back = frames;
    if (frames > s_suppress_future) s_suppress_future = frames;
    portEXIT_CRITICAL(&s_mux);
}

void fft_suppress_novelty_timed_ms(int backfill_ms, int future_frames)
{
    int frames = (int)ceilf(fmaxf(0, backfill_ms) / (FRAME_SECONDS * 1000));
    if (frames > DETECTOR_FRAMES) frames = DETECTOR_FRAMES;
    if (future_frames > DETECTOR_FRAMES) future_frames = DETECTOR_FRAMES;
    portENTER_CRITICAL(&s_mux);
    if (frames > s_suppress_back) s_suppress_back = frames;
    if (future_frames > s_suppress_future) s_suppress_future = future_frames;
    portEXIT_CRITICAL(&s_mux);
}
