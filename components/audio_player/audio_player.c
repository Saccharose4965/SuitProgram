#include "audio_player.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include "bt_audio.h"
#include "audio.h"
#include "audio_wav.h"

static const char *TAG = "audio_player";

static volatile bool s_abort = false;
static volatile bool s_paused = false;
static TaskHandle_t s_player_task = NULL;
static char s_current_path[128] = {0};

#define AUDIO_QUEUE_MAX 8

typedef enum {
    AUDIO_PLAYER_MODE_NONE = 0,
    AUDIO_PLAYER_MODE_BT,
    AUDIO_PLAYER_MODE_LOCAL,
} audio_player_mode_t;

static volatile audio_player_mode_t s_play_mode = AUDIO_PLAYER_MODE_NONE;

typedef struct {
    char path[128];
} audio_task_args_t;

static char s_queue_paths[AUDIO_QUEUE_MAX][128];
static int  s_queue_count = 0;

static bool     s_resume_valid = false;
static char     s_resume_path[128] = {0};
static uint32_t s_resume_offset = 0; // bytes into WAV data chunk

static bool s_bt_cb_registered = false;

#define AUDIO_TASK_STACK (8 * 1024)
#define AUDIO_TASK_PRIO  2

static void audio_play_task(void *arg);
static esp_err_t audio_player_start_task(const char *path);
static void audio_player_store_resume_from_bt(void);
static void audio_player_clear_resume(void);
static bool audio_player_get_resume_for_path(const char *path, uint32_t *out_offset);
static bool audio_player_queue_pop_front(char *out, size_t outsz);
static esp_err_t audio_player_queue_insert(int idx, const char *path);
static void audio_player_bt_disconnect_cb(void);

// ------------------- Worker task -------------------

static void audio_play_task(void *arg)
{
    audio_task_args_t *a = (audio_task_args_t *)arg;

    char path[128];
    strncpy(path, a->path, sizeof(path) - 1);
    path[sizeof(path) - 1] = '\0';
    free(a);

    // Remember current track (for toggle)
    strncpy(s_current_path, path, sizeof(s_current_path) - 1);
    s_current_path[sizeof(s_current_path) - 1] = '\0';

    s_abort = false;
    s_paused = false;
    s_play_mode = AUDIO_PLAYER_MODE_NONE;

    ESP_LOGI(TAG, "audio_play_task started for '%s'", path);
    ESP_LOGI(TAG, "audio_play_task stack high watermark: %u words",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open '%s', playing via local speaker", path);
        audio_play_wav_file(path); // this path cannot be aborted mid-play
        s_player_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    audio_wav_info_t info = (audio_wav_info_t){0};
    esp_err_t err = audio_wav_parse_file(f, &info);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "audio_wav_parse_file failed (%s), falling back to speaker",
                 esp_err_to_name(err));
        fclose(f);
        audio_play_wav_file(path); // re-opens and re-parses
        s_player_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG,
             "WAV: fmt=%u ch=%u bits=%u rate=%u data=%u",
             info.fmt.audio_format, info.fmt.num_channels,
             info.fmt.bits_per_sample, info.fmt.sample_rate, info.data_size);

    bool bt_used = false;
    bool skip_local = false;
    uint32_t resume_offset = 0;
    if (audio_player_get_resume_for_path(path, &resume_offset)) {
        if (resume_offset >= info.data_size) {
            audio_player_clear_resume();
            resume_offset = 0;
        }
    }

#if CONFIG_BT_ENABLED && CONFIG_BT_A2DP_ENABLE && CONFIG_BT_CLASSIC_ENABLED
    if (bt_audio_can_stream()) {
        const int wait_step_ms = 50;
        const int wait_limit_ms = 800;
        int waited_ms = 0;
        while (bt_audio_media_cmd_pending() && waited_ms < wait_limit_ms) {
            vTaskDelay(pdMS_TO_TICKS(wait_step_ms));
            waited_ms += wait_step_ms;
        }

        if (bt_audio_media_cmd_pending()) {
            ESP_LOGW(TAG, "bt_audio_play_wav skipped (media ctrl pending)");
            skip_local = true;
        } else {
            uint32_t data_offset = info.data_offset + resume_offset;
            uint32_t data_size = info.data_size - resume_offset;
            err = bt_audio_play_wav(f,
                                    data_offset,
                                    data_size,
                                    info.fmt.sample_rate,
                                    info.fmt.num_channels,
                                    info.fmt.bits_per_sample);
            if (err == ESP_OK) {
                bt_used = true;
                s_play_mode = AUDIO_PLAYER_MODE_BT;
                ESP_LOGI(TAG, "BT playback started");

                while (!s_abort) {
                    if (s_paused) {
                        vTaskDelay(pdMS_TO_TICKS(50));
                        continue;
                    }
                    bt_audio_status_t st = {0};
                    bt_audio_get_status(&st);
                    if (!st.streaming && st.queued_frames == 0) {
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                if (s_abort) {
                    ESP_LOGW(TAG, "BT playback aborted via audio_player_stop");
                    if (bt_audio_can_stream()) {
                        bt_audio_status_t st = {0};
                        bt_audio_get_status(&st);
                        if (st.streaming) {
                            (void)bt_audio_stop_stream();
                        }
                    }
                } else {
                    // Natural end: clear resume for this track
                    audio_player_clear_resume();
                }

            } else if (err == ESP_ERR_INVALID_STATE && bt_audio_can_stream()) {
                ESP_LOGW(TAG,
                         "bt_audio_play_wav failed (%s), BT busy; skipping speaker fallback",
                         esp_err_to_name(err));
                skip_local = true;
            } else {
                ESP_LOGW(TAG,
                         "bt_audio_play_wav failed (%s), falling back to speaker",
                         esp_err_to_name(err));
            }
        }
    }
#endif

    if (!bt_used) {
        if (skip_local) {
            fclose(f);
        } else {
            s_play_mode = AUDIO_PLAYER_MODE_LOCAL;
            audio_wav_info_t info_local = info;
            if (resume_offset > 0 && resume_offset < info.data_size) {
                info_local.data_offset = info.data_offset + resume_offset;
                info_local.data_size = info.data_size - resume_offset;
            }
            audio_play_wav_local_stream(f, &info_local);
            fclose(f);
            if (!s_abort) {
                audio_player_clear_resume();
            }
        }
    }

    s_player_task = NULL;
    s_play_mode = AUDIO_PLAYER_MODE_NONE;
    s_paused = false;
    if (!s_resume_valid) {
        s_current_path[0] = '\0';
    }

    if (!s_abort && audio_player_queue_pop_front(path, sizeof(path))) {
        audio_player_clear_resume();
        (void)audio_player_start_task(path);
    }
    vTaskDelete(NULL);
}

// ------------------- Public control API -------------------

esp_err_t audio_player_play(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;

#if CONFIG_BT_ENABLED && CONFIG_BT_A2DP_ENABLE && CONFIG_BT_CLASSIC_ENABLED
    if (!s_bt_cb_registered) {
        bt_audio_set_disconnect_cb(audio_player_bt_disconnect_cb);
        s_bt_cb_registered = true;
    }
#endif

    if (s_resume_valid && strncmp(path, s_resume_path, sizeof(s_resume_path)) != 0) {
        audio_player_clear_resume();
    }

    // If something is already playing, stop it first
    if (s_player_task) {
        audio_player_stop();
        for (int i = 0; i < 200 && s_player_task; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_player_task) {
            ESP_LOGW(TAG, "audio_player_play: previous task still running, forcing stop");
            if (s_player_task == xTaskGetCurrentTaskHandle()) {
                return ESP_ERR_INVALID_STATE;
            }
            vTaskDelete(s_player_task);
            s_player_task = NULL;
            s_play_mode = AUDIO_PLAYER_MODE_NONE;
            s_paused = false;
            s_current_path[0] = '\0';
        }
    }

    esp_err_t start_err = audio_player_start_task(path);
    if (start_err != ESP_OK) {
        return start_err;
    }

    strncpy(s_current_path, path, sizeof(s_current_path) - 1);
    s_current_path[sizeof(s_current_path) - 1] = '\0';

    return ESP_OK;
}

esp_err_t audio_player_set_current(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;

    strncpy(s_current_path, path, sizeof(s_current_path) - 1);
    s_current_path[sizeof(s_current_path) - 1] = '\0';

    ESP_LOGI(TAG, "Current track set to '%s'", s_current_path);
    return ESP_OK;
}

esp_err_t audio_player_toggle_play_pause(void)
{
    if (s_player_task) {
        if (s_paused) {
            s_paused = false;
            if (s_play_mode == AUDIO_PLAYER_MODE_BT && bt_audio_can_stream()) {
                esp_err_t err = bt_audio_start_stream();
                if (err != ESP_OK) {
                    s_paused = true;
                    return err;
                }
            }
            ESP_LOGI(TAG, "Toggle: resume playback");
            return ESP_OK;
        }
        s_paused = true;
        if (s_play_mode == AUDIO_PLAYER_MODE_BT && bt_audio_can_stream()) {
            audio_player_store_resume_from_bt();
            esp_err_t err = bt_audio_pause_stream();
            if (err != ESP_OK) {
                s_paused = false;
                return err;
            }
        }
        ESP_LOGI(TAG, "Toggle: pause playback");
        return ESP_OK;
    }

    // Nothing is playing; start the currently selected track
    if (s_current_path[0] == '\0') {
        if (s_resume_valid) {
            strncpy(s_current_path, s_resume_path, sizeof(s_current_path) - 1);
            s_current_path[sizeof(s_current_path) - 1] = '\0';
        }
    }
    if (s_current_path[0] == '\0') {
        ESP_LOGW(TAG, "Toggle: no current track selected");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Toggle: starting '%s'", s_current_path);
    return audio_player_play(s_current_path);
}

void audio_player_stop(void)
{
    s_abort = true;
    s_paused = false;
    if (bt_audio_can_stream()) {
        bt_audio_status_t st = {0};
        bt_audio_get_status(&st);
        if (st.streaming || s_play_mode == AUDIO_PLAYER_MODE_BT) {
            (void)bt_audio_stop_stream();
        }
    }
}

bool audio_player_should_abort(void)
{
    return s_abort;
}

bool audio_player_should_pause(void)
{
    return s_paused;
}

bool audio_player_is_active(void)
{
    return s_player_task != NULL;
}

bool audio_player_is_paused(void)
{
    return s_paused;
}

void audio_player_queue_clear(void)
{
    s_queue_count = 0;
}

int audio_player_queue_count(void)
{
    return s_queue_count;
}

esp_err_t audio_player_queue_play_now(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;
    return audio_player_play(path);
}

esp_err_t audio_player_queue_play_next(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;
    if (!audio_player_is_active() && s_current_path[0] == '\0') {
        return audio_player_play(path);
    }
    return audio_player_queue_insert(0, path);
}

esp_err_t audio_player_queue_play_last(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;
    if (!audio_player_is_active() && s_current_path[0] == '\0') {
        return audio_player_play(path);
    }
    return audio_player_queue_insert(s_queue_count, path);
}

static esp_err_t audio_player_start_task(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;
    audio_task_args_t *a = (audio_task_args_t*)malloc(sizeof(audio_task_args_t));
    if (!a) return ESP_ERR_NO_MEM;
    strncpy(a->path, path, sizeof(a->path) - 1);
    a->path[sizeof(a->path) - 1] = '\0';

    s_abort = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        audio_play_task,
        "audio_play_task",
        AUDIO_TASK_STACK,
        a,
        AUDIO_TASK_PRIO,
        &s_player_task,
        tskNO_AFFINITY
    );
    if (ok != pdPASS){
        free(a);
        s_player_task = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void audio_player_store_resume_from_bt(void)
{
    if (s_play_mode != AUDIO_PLAYER_MODE_BT || s_current_path[0] == '\0') {
        return;
    }
    uint32_t total = 0;
    uint32_t left = 0;
    if (!bt_audio_get_playback_progress(&total, &left) || total == 0) {
        return;
    }
    uint32_t played = total > left ? (total - left) : 0;
    if (played >= total) {
        audio_player_clear_resume();
        return;
    }
    strncpy(s_resume_path, s_current_path, sizeof(s_resume_path) - 1);
    s_resume_path[sizeof(s_resume_path) - 1] = '\0';
    s_resume_offset = played;
    s_resume_valid = true;
}

static void audio_player_clear_resume(void)
{
    s_resume_valid = false;
    s_resume_offset = 0;
    s_resume_path[0] = '\0';
}

static bool audio_player_get_resume_for_path(const char *path, uint32_t *out_offset)
{
    if (!path || !out_offset || !s_resume_valid) return false;
    if (strncmp(path, s_resume_path, sizeof(s_resume_path)) != 0) {
        return false;
    }
    *out_offset = s_resume_offset;
    return true;
}

static bool audio_player_queue_pop_front(char *out, size_t outsz)
{
    if (!out || outsz == 0 || s_queue_count <= 0) {
        return false;
    }
    strncpy(out, s_queue_paths[0], outsz - 1);
    out[outsz - 1] = '\0';
    for (int i = 1; i < s_queue_count; ++i) {
        strncpy(s_queue_paths[i - 1], s_queue_paths[i], sizeof(s_queue_paths[i - 1]) - 1);
        s_queue_paths[i - 1][sizeof(s_queue_paths[i - 1]) - 1] = '\0';
    }
    s_queue_count--;
    return true;
}

static esp_err_t audio_player_queue_insert(int idx, const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;
    if (s_queue_count >= AUDIO_QUEUE_MAX) return ESP_ERR_NO_MEM;
    if (idx < 0) idx = 0;
    if (idx > s_queue_count) idx = s_queue_count;
    for (int i = s_queue_count; i > idx; --i) {
        strncpy(s_queue_paths[i], s_queue_paths[i - 1], sizeof(s_queue_paths[i]) - 1);
        s_queue_paths[i][sizeof(s_queue_paths[i]) - 1] = '\0';
    }
    strncpy(s_queue_paths[idx], path, sizeof(s_queue_paths[idx]) - 1);
    s_queue_paths[idx][sizeof(s_queue_paths[idx]) - 1] = '\0';
    s_queue_count++;
    return ESP_OK;
}

static void audio_player_bt_disconnect_cb(void)
{
    audio_player_store_resume_from_bt();
    s_abort = true;
    s_paused = false;
}
