#include "synth_service.h"
#include "synth_voice.h"
#include "audio_policy.h"
#include "audio_bus.h"
#include "arm_pose.h"
#include "orientation_service.h"
#include "audio.h"
#include "audio_player.h"
#include "bt_audio.h"
#include "microphone.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static arm_pose_t s_pose;
static synth_voice_t s_voice;
static bool s_muted, s_worker, s_stop, s_bluetooth;
static float s_hz, s_gain;
static int64_t s_updated;
static esp_err_t s_error;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t s_done_storage;
static SemaphoreHandle_t s_done;

static bool sample(imu_quat_t *q)
{
    return orientation_service_fresh_snapshot(q, 250000);
}

static size_t generate(int16_t *out, size_t n, void *user)
{
    (void)user;
    portENTER_CRITICAL(&s_mux);
    float hz = s_hz, gain = s_gain;
    bool stop = s_stop;
    int64_t updated = s_updated;
    portEXIT_CRITICAL(&s_mux);
    if (stop || esp_timer_get_time() - updated > 250000) gain = 0;
    if (stop && s_voice.gain < .0001f) return 0;
    synth_voice_render(&s_voice, out, n, hz, gain);
    return n;
}

static bool stopping(void)
{
    portENTER_CRITICAL(&s_mux);
    bool stop = s_stop;
    portEXIT_CRITICAL(&s_mux);
    return stop;
}

static size_t generate_bt(int16_t *out, size_t n, int rate, void *user)
{
    s_voice.rate = rate;
    return generate(out, n, user);
}

static void worker(void *unused)
{
    (void)unused;
    esp_err_t err = ESP_OK;
    if (bt_audio_can_stream()) {
        int64_t deadline = esp_timer_get_time() + 2000000;
        while (bt_audio_media_cmd_pending() && !stopping() &&
               esp_timer_get_time() < deadline) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        bool attached = false, started = false;
        int rate = bt_audio_get_peer_sample_rate();
        if (!stopping() && !bt_audio_media_cmd_pending() &&
            (rate == 44100 || rate == 48000)) {
            attached = bt_audio_set_generator(generate_bt, NULL) == ESP_OK;
            if (attached) started = bt_audio_start_stream() == ESP_OK;
        }
        if (started) {
            portENTER_CRITICAL(&s_mux);
            s_bluetooth = true;
            portEXIT_CRITICAL(&s_mux);
            while (!stopping() && bt_audio_can_stream()) {
                bt_audio_status_t status;
                bt_audio_get_status(&status);
                if (!status.streaming && !bt_audio_media_cmd_pending()) break;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        if (attached) (void)bt_audio_set_generator(NULL, NULL);
        if (started) {
            deadline = esp_timer_get_time() + 2000000;
            while (bt_audio_media_cmd_pending() &&
                   esp_timer_get_time() < deadline) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            (void)bt_audio_stop_stream();
        }
    }

    portENTER_CRITICAL(&s_mux);
    s_bluetooth = false;
    portEXIT_CRITICAL(&s_mux);
    if (!stopping()) {
        s_voice.rate = SYNTH_RATE;
        s_voice.gain = 0;
        if (audio_bus_init_if_needed()) {
            err = audio_play_generated(generate, NULL, SYNTH_RATE);
        } else {
            err = ESP_FAIL;
        }
    }
    portENTER_CRITICAL(&s_mux);
    s_error = err;
    portEXIT_CRITICAL(&s_mux);
    xSemaphoreGive(s_done);
    vTaskDelete(NULL);
}

void synth_service_open(void)
{
    audio_policy_fft_set_synth_app(true);
    orientation_reference_reset(&s_pose);
    s_muted = true;
    s_worker = false;
    s_stop = false;
    s_bluetooth = false;
    s_hz = 220;
    s_gain = 0;
    s_updated = esp_timer_get_time();
    s_error = ESP_OK;

    bt_audio_status_t status;
    bt_audio_get_status(&status);
    if (mic_rec_is_recording() || audio_player_is_active() ||
        audio_output_active() || status.streaming) {
        s_error = ESP_ERR_INVALID_STATE;
        return;
    }
    if (!s_done) s_done = xSemaphoreCreateBinaryStatic(&s_done_storage);
    while (xSemaphoreTake(s_done, 0) == pdTRUE) {}
    synth_voice_init(&s_voice);
    s_worker = xTaskCreate(worker, "synth", 6144, NULL, 3, NULL) == pdPASS;
    if (!s_worker) s_error = ESP_ERR_NO_MEM;
}

void synth_service_close(void)
{
    portENTER_CRITICAL(&s_mux);
    s_stop = true;
    portEXIT_CRITICAL(&s_mux);
    if (s_worker) xSemaphoreTake(s_done, portMAX_DELAY);
    s_worker = false;
    audio_policy_fft_set_synth_app(false);
}

void synth_service_calibrate(void)
{
    imu_quat_t q;
    if (sample(&q)) {
        (void)arm_pose_calibrate(&s_pose, q);
        s_muted = true;
    }
}

void synth_service_toggle_mute(void)
{
    if (s_pose.calibrated) s_muted = !s_muted;
}

void synth_service_tick(void)
{
    imu_quat_t q;
    arm_vec3_t direction;
    float hz = 220, gain = 0;
    if (sample(&q) && arm_pose_direction(&s_pose, q, true, &direction)) {
        hz = 220 * exp2f(1.5f * fmaxf(-1, fminf(1, direction.y)));
        if (!s_muted) gain = .5f * (1 + fmaxf(-1, fminf(1, direction.x)));
    }
    portENTER_CRITICAL(&s_mux);
    s_hz = hz;
    s_gain = gain;
    s_updated = esp_timer_get_time();
    portEXIT_CRITICAL(&s_mux);
}

synth_status_t synth_service_status(void)
{
    portENTER_CRITICAL(&s_mux);
    synth_status_t status = {
        .calibrated = s_pose.calibrated,
        .muted = s_muted,
        .bluetooth = s_bluetooth,
        .hz = s_hz,
        .gain = s_gain,
        .error = s_error,
    };
    portEXIT_CRITICAL(&s_mux);
    return status;
}
