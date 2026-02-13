#include "audio_player.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#include "bt_audio.h"
#include "audio.h"
#include "audio_wav.h"

static const char *TAG = "audio_player";

static volatile bool s_abort = false;
static volatile bool s_paused = false;
static TaskHandle_t s_player_task = NULL;
static char s_current_path[128] = {0};

typedef enum {
    AUDIO_PLAYER_MODE_NONE = 0,
    AUDIO_PLAYER_MODE_BT,
    AUDIO_PLAYER_MODE_LOCAL,
} audio_player_mode_t;

static volatile audio_player_mode_t s_play_mode = AUDIO_PLAYER_MODE_NONE;

typedef struct {
    char path[128];
} audio_task_args_t;

#define AUDIO_TASK_STACK (4 * 1024)
#define AUDIO_TASK_STACK_RETRY (3 * 1024)
#define AUDIO_TASK_PRIO  2
#if configNUMBER_OF_CORES > 1
#define AUDIO_TASK_CORE  1
#else
#define AUDIO_TASK_CORE  0
#endif

static void audio_play_task(void *arg);

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
    ESP_LOGI(TAG, "audio_play_task stack high watermark: %u bytes",
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
            err = bt_audio_play_wav(f,
                                    info.data_offset,
                                    info.data_size,
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
                }

            } else if (bt_audio_can_stream()) {
                ESP_LOGW(TAG,
                         "bt_audio_play_wav failed (%s), skipping speaker fallback",
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
            audio_play_wav_local_stream(f, &info);
            fclose(f);
        }
    }

    s_player_task = NULL;
    s_play_mode = AUDIO_PLAYER_MODE_NONE;
    s_paused = false;
    s_current_path[0] = '\0';
    vTaskDelete(NULL);
}

// ------------------- Public control API -------------------

esp_err_t audio_player_play(const char *path)
{
    if (!path) return ESP_ERR_INVALID_ARG;

    // If something is already playing, stop it first
    if (s_player_task) {
        audio_player_stop();
        for (int i = 0; i < 200 && s_player_task; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_player_task) {
            ESP_LOGW(TAG, "audio_player_play: previous task still running, drop request");
            return ESP_ERR_INVALID_STATE;
        }
    }

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
        AUDIO_TASK_CORE
    );

#if CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    if (ok != pdPASS) {
        // Fallback when internal heap is fragmented/low under BT+Wi-Fi load.
        ok = xTaskCreatePinnedToCoreWithCaps(
            audio_play_task,
            "audio_play_task",
            AUDIO_TASK_STACK,
            a,
            AUDIO_TASK_PRIO,
            &s_player_task,
            AUDIO_TASK_CORE,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        );
        if (ok == pdPASS) {
            ESP_LOGW(TAG, "audio task allocated in PSRAM");
        }
    }
#endif

    if (ok != pdPASS && AUDIO_TASK_STACK_RETRY < AUDIO_TASK_STACK) {
        ESP_LOGW(TAG, "audio task create retry with smaller stack (%u -> %u bytes)",
                 (unsigned)AUDIO_TASK_STACK, (unsigned)AUDIO_TASK_STACK_RETRY);
        ok = xTaskCreatePinnedToCore(
            audio_play_task,
            "audio_play_task",
            AUDIO_TASK_STACK_RETRY,
            a,
            AUDIO_TASK_PRIO,
            &s_player_task,
            AUDIO_TASK_CORE
        );
#if CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
        if (ok != pdPASS) {
            ok = xTaskCreatePinnedToCoreWithCaps(
                audio_play_task,
                "audio_play_task",
                AUDIO_TASK_STACK_RETRY,
                a,
                AUDIO_TASK_PRIO,
                &s_player_task,
                AUDIO_TASK_CORE,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );
            if (ok == pdPASS) {
                ESP_LOGW(TAG, "audio task allocated in PSRAM (retry stack)");
            }
        }
#endif
    }

    if (ok != pdPASS){
        ESP_LOGE(TAG,
                 "audio task create failed: internal=%u largest_internal=%u spiram=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        free(a);
        s_player_task = NULL;
        return ESP_FAIL;
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
