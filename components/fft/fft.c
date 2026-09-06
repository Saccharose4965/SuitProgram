#include "fft.h"
#include "fft_detector.h"
#include "fft_render.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "audio.h"
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
#define FFT_CORE 1
#define FRAME_SECONDS ((double)DETECTOR_FFT_SIZE / DETECTOR_SAMPLE_RATE)
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
    float milliseconds;
} analysis_result_t;

typedef struct { float fast, slow, peak, out; } level_track_t;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static controls_t s_controls = { .beat_enabled = true, .export_leds = true, .ratio = 1 };
static fft_sync_state_t s_status;
static fft_levels_t s_levels;
static TaskHandle_t s_capture_task, s_analysis_task;
static QueueHandle_t s_jobs, s_results, s_beats;
static volatile bool s_stop_capture, s_stop_analysis;
// The capture task owns a binary job credit until it consumes a result.
static bool s_job_busy;
static EXT_RAM_BSS_ATTR float s_frames[DETECTOR_FRAMES][DETECTOR_BANDS];
static EXT_RAM_BSS_ATTR float s_snapshot[DETECTOR_FRAMES][DETECTOR_BANDS];
static EXT_RAM_BSS_ATTR detector_t s_detector;
static EXT_RAM_BSS_ATTR float s_re[DETECTOR_FFT_SIZE], s_im[DETECTOR_FFT_SIZE];
static EXT_RAM_BSS_ATTR float s_window[DETECTOR_FFT_SIZE];
static EXT_RAM_BSS_ATTR analysis_result_t s_current;
static detector_filter_t s_filter;
static int s_frame_count, s_frame_write, s_hop_count;
static int s_suppress_back, s_suppress_future;
static uint32_t s_capture_epoch;
static double s_clock_anchor;
static float s_clock_bpm;
static bool s_clock_valid;
static double s_last_emit;
static uint32_t s_skipped, s_capture_errors;
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
    s_frame_count = s_frame_write = s_hop_count = 0;
    memset(&s_filter, 0, sizeof(s_filter));
    memset(&s_current, 0, sizeof(s_current));
    s_clock_valid = false;
    s_clock_bpm = 0;
    s_last_emit = 0;
    if (s_beats) xQueueReset(s_beats);
    fft_render_update_tempo_spectrum(NULL);
    fft_render_update_phase_curve(NULL, 0);
    portENTER_CRITICAL(&s_mux);
    s_status.bpm = s_status.held_bpm = s_status.detected_bpm = 0;
    s_status.confidence = s_status.evidence = s_status.lost_seconds = 0;
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
    clear_capture();
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

static void analysis_task(void *arg)
{
    (void)arg;
    uint32_t epoch = UINT32_MAX;
    analysis_job_t job;
    while (!s_stop_analysis && xQueueReceive(s_jobs, &job, portMAX_DELAY) == pdTRUE) {
        if (s_stop_analysis) break;
        if (epoch != job.controls.epoch) {
            detector_reset(&s_detector);
            epoch = job.controls.epoch;
        }
        analysis_result_t result = { .epoch = job.controls.epoch, .revision = job.controls.revision };
        float raw[DETECTOR_BPM_COUNT], normalized[DETECTOR_BPM_COUNT];
        int64_t start = esp_timer_get_time(), slice = start;
        bool cancelled = false;
        for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) {
            if (s_stop_analysis || controls_get().epoch != epoch) { cancelled = true; break; }
            raw[i] = detector_comb_score(&s_snapshot[0][0], job.count, DETECTOR_BPM_MIN + i);
            if (esp_timer_get_time() - slice >= 2000) {
                vTaskDelay(1); // actual block lets idle, capture, LEDs and radio run
                slice = esp_timer_get_time();
            }
        }
        if (s_stop_analysis) break;
        if (!cancelled) {
            detector_normalize(raw, normalized);
            detector_update(&s_detector, normalized, job.time, job.controls.detector, &result.detector);
            detector_phase(&s_detector, &s_snapshot[0][0], job.count, job.time,
                           job.controls.detector.phase_lock, &result.detector);
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
    s_current = result;
    detector_result_t *d = &s_current.detector;
    if (d->stopped) {
        ESP_LOGI(TAG, "No credible pulse for 30 s; seeking a new BPM");
        portENTER_CRITICAL(&s_mux);
        // Do not undo a reset or force request made since the job snapshot.
        if (s_controls.revision == c.revision) {
            s_controls.detector.phase_lock = false;
            ++s_controls.revision;
            ++s_controls.epoch;
        }
        portEXIT_CRITICAL(&s_mux);
        return;
    }
    if (d->phase_valid && d->held_bpm > 0 &&
        (d->phase_stable || !s_clock_valid || s_clock_bpm != d->held_bpm)) {
        s_clock_anchor = d->beat_time;
        s_clock_bpm = d->held_bpm;
        s_clock_valid = true;
    }
    if (fft_render_is_display_enabled()) {
        fft_render_update_tempo_spectrum(d->scores);
        fft_render_update_phase_curve(d->phase, DETECTOR_PHASE_BINS);
    }
}

static void submit_analysis(double time, controls_t c)
{
    if (s_job_busy) { ++s_skipped; return; }
    if (s_frame_count < 3) return;
    int oldest = (s_frame_write - s_frame_count + DETECTOR_FRAMES) % DETECTOR_FRAMES;
    for (int i = 0; i < s_frame_count; ++i)
        memcpy(s_snapshot[i], s_frames[(oldest + i) % DETECTOR_FRAMES], sizeof(s_snapshot[i]));
    analysis_job_t job = { .count = s_frame_count, .time = time, .controls = c };
    s_job_busy = xQueueSend(s_jobs, &job, 0) == pdTRUE;
}

static void emit_and_publish(double now, controls_t c)
{
    detector_result_t *d = &s_current.detector;
    float base = c.detector.hard_lock ? c.detector.forced_bpm : d->held_bpm;
    float bpm = s_clock_valid ? base * c.ratio : 0;
    float phase = 0, target = wrap(c.offset + (c.offbeat ? 0.5f : 0));
    if (bpm > 0) {
        double period = 60.0 / bpm;
        double cycle = (now - s_clock_anchor) / period;
        phase = (float)(cycle - floor(cycle));
        double trigger = s_clock_anchor + (floor(cycle - target) + target) * period;
        // A phase snap cannot produce a burst of duplicate beats.
        if (trigger > s_last_emit && now - trigger < FRAME_SECONDS * 1.5 &&
            now - s_last_emit >= period * 0.5) {
            s_last_emit = now;
            if (c.beat_enabled) {
                fft_beat_event_t evt = { .bpm = bpm, .confidence = fminf(1, d->confidence) };
                xQueueSend(s_beats, &evt, 0);
                if (c.export_leds) led_trigger_beat(255, 96, 0);
                if (fft_render_is_display_enabled()) fft_render_trigger_flash(5);
            }
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_status.bpm = bpm;
    s_status.held_bpm = base;
    s_status.detected_bpm = d->candidate_bpm;
    s_status.confidence = fminf(1, d->confidence);
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
    float power = 0;
    for (int i = 0; i < DETECTOR_FFT_SIZE; ++i) {
        power += s_re[i] * s_re[i];
        s_re[i] *= s_window[i]; s_im[i] = 0;
    }
    fft_radix2(s_re, s_im);
    fft_render_packet_t packet = {0};
    for (int k = 0; k <= DETECTOR_FFT_SIZE / 2; ++k)
        packet.logmag[k] = log1pf(10 * sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]));
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
        fft_render_suppress_recent_novelty(back);
    }
    float previous[DETECTOR_BANDS];
    memcpy(previous, s_frames[(s_frame_write - 1 + DETECTOR_FRAMES) % DETECTOR_FRAMES], sizeof(previous));
    detector_filter_bands(&s_filter, packet.logmag, suppress, s_frames[s_frame_write]);
    float flux = 0;
    if (s_frame_count) for (int i = 0; i < DETECTOR_BANDS; ++i) flux += fmaxf(0, s_frames[s_frame_write][i] - previous[i]);
    s_frame_write = (s_frame_write + 1) % DETECTOR_FRAMES;
    if (s_frame_count < DETECTOR_FRAMES) ++s_frame_count;
    poll_analysis(c);
    c = controls_get();
    if (c.epoch != s_capture_epoch) { s_capture_epoch = c.epoch; clear_capture(); }
    if (++s_hop_count >= DETECTOR_HOP_FRAMES) { s_hop_count = 0; submit_analysis(time, c); }
    emit_and_publish(esp_timer_get_time() / 1e6, c);
    if (fft_render_is_display_enabled()) {
        static float flux_mean;
        flux_mean += (flux - flux_mean) / FFT_CFG_NOVELTY_WIN;
        fft_render_push_spectrogram(packet.logmag);
        fft_render_push_novelty_display(flux, flux_mean, fmaxf(0, flux - flux_mean));
        fft_sync_state_t status;
        fft_get_sync_state(&status);
        packet.bpm_est = status.bpm;
        packet.bpm_conf = status.confidence;
        packet.bpm_phase = status.beat_phase;
        packet.evidence = status.evidence;
        fft_render_submit(&packet);
    }
    float load = (float)(esp_timer_get_time() - start) / (FRAME_SECONDS * 10000);
    portENTER_CRITICAL(&s_mux);
    s_status.capture_load += 0.05f * (load - s_status.capture_load);
    portEXIT_CRITICAL(&s_mux);
}

static void capture_task(void *arg)
{
    int32_t *input = arg;
    i2s_chan_handle_t rx = audio_rx_handle();
    uint32_t clock_revision = audio_get_clock_revision();
    uint32_t overflows = audio_get_rx_overflows();
    uint64_t samples = 0;
    double origin = 0;
    int fill = 0, slot = -1;
    bool suspended = false;
    int64_t report_time = esp_timer_get_time();
    while (!s_stop_capture) {
        uint32_t revision = audio_get_clock_revision();
        bool unavailable = audio_get_rate() != DETECTOR_SAMPLE_RATE || mic_rec_is_recording();
        if (unavailable || revision != clock_revision) {
            if (!suspended) { invalidate_capture(); suspended = true; }
            clock_revision = revision;
            fill = 0; samples = 0;
            if (unavailable) {
                // Playback/recording own the shared clock while active. Restore
                // mic capture only after they finish, never during re-clocking.
                if (audio_get_rate() > 0 && !audio_output_active() &&
                    !audio_player_is_active() && !mic_rec_is_recording()) {
                    if (audio_set_rate(DETECTOR_SAMPLE_RATE) == ESP_OK) (void)audio_enable_rx();
                }
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
        }
        suspended = false;
        size_t bytes = 0;
        esp_err_t err = i2s_channel_read(rx, input, DETECTOR_FFT_SIZE * 2 * sizeof(*input), &bytes, 100);
        // Never concatenate partial frames across a failed read or clock change.
        uint32_t current_overflows = audio_get_rx_overflows();
        if ((err != ESP_OK && err != ESP_ERR_TIMEOUT) || !bytes ||
            current_overflows != overflows ||
            revision != audio_get_clock_revision()) {
            ++s_capture_errors;
            overflows = current_overflows;
            if (fill || samples) invalidate_capture();
            fill = 0; samples = 0;
            if ((err == ESP_ERR_INVALID_STATE || !bytes) && audio_get_rate() > 0 && !audio_output_active() &&
                !audio_player_is_active() && !mic_rec_is_recording()) {
                if (audio_set_rate(DETECTOR_SAMPLE_RATE) == ESP_OK) (void)audio_enable_rx();
            }
            vTaskDelay(1);
            continue;
        }
        int n = bytes / (2 * sizeof(*input));
        if (!samples) origin = esp_timer_get_time() / 1e6 - n / (double)DETECTOR_SAMPLE_RATE;
        if (slot < 0) {
            double left = 0, right = 0;
            for (int i = 0; i < n; ++i) { left += fabs((double)input[2*i]); right += fabs((double)input[2*i+1]); }
            slot = right > left;
            ESP_LOGI(TAG, "Mic %s; 44.1 kHz, 4 s treble comb, 22-frame analysis hop", slot ? "RIGHT" : "LEFT");
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
            ESP_LOGI(TAG, "capture %.0f%% worker %.0fms skipped %lu errors %lu held %.0f evidence %.2f lost %.1fs",
                     status.capture_load, status.analysis_ms, (unsigned long)s_skipped,
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
    if (!s_beats) s_beats = xQueueCreate(4, sizeof(fft_beat_event_t));
    if (!s_jobs || !s_results || !s_beats) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(audio_set_rate(DETECTOR_SAMPLE_RATE), TAG, "set rate");
    ESP_RETURN_ON_ERROR(audio_enable_rx(), TAG, "enable rx");
    ESP_RETURN_ON_ERROR(led_init(), TAG, "LED init");
    ESP_RETURN_ON_ERROR(fft_render_init(), TAG, "render init");
    int32_t *input = malloc(DETECTOR_FFT_SIZE * 2 * sizeof(*input));
    if (!input) return ESP_ERR_NO_MEM;
    for (int i = 0; i < DETECTOR_FFT_SIZE; ++i)
        s_window[i] = 0.5f * (1 - cosf(6.28318530718f * i / (DETECTOR_FFT_SIZE - 1)));
    xQueueReset(s_jobs); xQueueReset(s_results);
    s_job_busy = s_stop_capture = s_stop_analysis = false;
    s_skipped = s_capture_errors = 0;
    memset(s_level, 0, sizeof(s_level));
    fft_detector_reset();
    s_capture_epoch = controls_get().epoch;
    clear_capture();
    if (xTaskCreatePinnedToCore(analysis_task, "fft_bpm", 6144, NULL,
                               ANALYSIS_PRIORITY, &s_analysis_task, FFT_CORE) != pdPASS) {
        free(input); return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(capture_task, "fft_vis", 8192, input,
                               CAPTURE_PRIORITY, &s_capture_task, FFT_CORE) != pdPASS) {
        free(input); fft_visualizer_stop(); return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void fft_visualizer_stop(void)
{
    s_stop_capture = s_stop_analysis = true;
    if (s_jobs) { analysis_job_t wake = {0}; xQueueOverwrite(s_jobs, &wake); }
    // Bounded I2S reads and per-BPM cancellation make cooperative shutdown safe.
    while (s_capture_task || s_analysis_task) vTaskDelay(1);
    clear_capture();
    led_audio_raw_volume_set(0);
    led_audio_levels_set(0, 0, 0, 0);
    portENTER_CRITICAL(&s_mux);
    memset(&s_levels, 0, sizeof(s_levels));
    portEXIT_CRITICAL(&s_mux);
}

bool fft_visualizer_running(void) { return s_capture_task != NULL; }
void fft_visualizer_set_view(fft_view_t view) { fft_render_set_view(view); }
void fft_set_display_enabled(bool enabled) { fft_render_set_display_enabled(enabled); }
void fft_copy_frame(uint8_t *dst, size_t len) { fft_render_copy_frame(dst, len); }
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
    ++s_controls.epoch; ++s_controls.revision;
    s_controls.detector.hard_lock = s_controls.detector.phase_lock = false;
    s_status.bpm = s_status.held_bpm = s_status.detected_bpm = 0;
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
void fft_beat_ratio_cycle(void)
{ portENTER_CRITICAL(&s_mux); s_controls.ratio = s_controls.ratio % 3 + 1; portEXIT_CRITICAL(&s_mux); }
void fft_beat_offbeat_toggle(void)
{ portENTER_CRITICAL(&s_mux); s_controls.offbeat = !s_controls.offbeat; portEXIT_CRITICAL(&s_mux); }
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
