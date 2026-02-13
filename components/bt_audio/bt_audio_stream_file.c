#include "bt_audio.h"
#include "bt_audio_internal.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "bt_audio";

// ============================ File-backed source ============================
static FILE    *s_bt_fp             = NULL;
static uint32_t s_bt_bytes_left     = 0;
static uint16_t s_bt_channels       = 2;
static SemaphoreHandle_t s_bt_fp_lock = NULL; // protects FILE access across callback/control paths

#define BT_STREAM_BUF_BYTES_MAX   (256 * 1024)
#define BT_STREAM_BUF_BYTES_MIN   (64 * 1024)
#define BT_STREAM_START_WATERMARK (12 * 1024)
#define BT_FEEDER_CHUNK_BYTES     2048
#define BT_FEEDER_POLL_MS         8
#define BT_FEEDER_TASK_STACK      3072
#define BT_FEEDER_TASK_PRIO       4
#if configNUMBER_OF_CORES > 1
#define BT_FEEDER_TASK_CORE       1
#else
#define BT_FEEDER_TASK_CORE       0
#endif

static uint8_t *s_bt_buf        = NULL;
static size_t   s_bt_buf_size   = 0;
static size_t   s_bt_buf_r      = 0;
static size_t   s_bt_buf_w      = 0;
static size_t   s_bt_buf_count  = 0;
static uint8_t  s_bt_feeder_chunk[BT_FEEDER_CHUNK_BYTES];
static volatile bool s_bt_file_eof    = false;
static volatile bool s_bt_feeder_stop = false;
static volatile bool s_bt_direct_stream = false;
static TaskHandle_t  s_bt_feeder_task = NULL;
static portMUX_TYPE  s_bt_buf_mux     = portMUX_INITIALIZER_UNLOCKED;

// Volume control: 0-100 % mapped to Q15 gain (0-32767)
static int32_t s_bt_vol_q15 = 32767; // unity gain (Q15)
static int     s_bt_volume_percent = 100;

static void bt_audio_update_vol_q15_from_percent(void)
{
    int v = s_bt_volume_percent;
    if (v <= 0) {
        s_bt_vol_q15 = 0;
    } else if (v >= 100) {
        s_bt_vol_q15 = 32767;
    } else {
        // rounded linear mapping 0-100% -> 0-32767
        s_bt_vol_q15 = (int32_t)(v * 32767 + 50) / 100;
    }
}

static bool lock_fp(TickType_t wait_ticks)
{
    if (!s_bt_fp_lock) {
        s_bt_fp_lock = xSemaphoreCreateMutex();
    }
    if (!s_bt_fp_lock) return false;
    return xSemaphoreTake(s_bt_fp_lock, wait_ticks) == pdTRUE;
}

static void unlock_fp(void)
{
    if (s_bt_fp_lock) {
        xSemaphoreGive(s_bt_fp_lock);
    }
}

static void bt_buf_reset_locked(void)
{
    s_bt_buf_r = 0;
    s_bt_buf_w = 0;
    s_bt_buf_count = 0;
}

static void bt_buf_reset(void)
{
    taskENTER_CRITICAL(&s_bt_buf_mux);
    bt_buf_reset_locked();
    taskEXIT_CRITICAL(&s_bt_buf_mux);
}

static size_t bt_buf_level(void)
{
    size_t level = 0;
    taskENTER_CRITICAL(&s_bt_buf_mux);
    level = s_bt_buf_count;
    taskEXIT_CRITICAL(&s_bt_buf_mux);
    return level;
}

static size_t bt_buf_space(void)
{
    size_t space = 0;
    taskENTER_CRITICAL(&s_bt_buf_mux);
    space = (s_bt_buf_size > s_bt_buf_count) ? (s_bt_buf_size - s_bt_buf_count) : 0;
    taskEXIT_CRITICAL(&s_bt_buf_mux);
    return space;
}

static size_t bt_buf_write(const uint8_t *src, size_t n)
{
    if (!src || n == 0 || !s_bt_buf || s_bt_buf_size == 0) return 0;

    taskENTER_CRITICAL(&s_bt_buf_mux);

    size_t space = (s_bt_buf_size > s_bt_buf_count) ? (s_bt_buf_size - s_bt_buf_count) : 0;
    if (n > space) n = space;

    size_t first = s_bt_buf_size - s_bt_buf_w;
    if (first > n) first = n;
    memcpy(s_bt_buf + s_bt_buf_w, src, first);

    size_t second = n - first;
    if (second > 0) {
        memcpy(s_bt_buf, src + first, second);
    }

    s_bt_buf_w = (s_bt_buf_w + n) % s_bt_buf_size;
    s_bt_buf_count += n;

    taskEXIT_CRITICAL(&s_bt_buf_mux);
    return n;
}

static size_t bt_buf_read(uint8_t *dst, size_t n)
{
    if (!dst || n == 0 || !s_bt_buf || s_bt_buf_size == 0) return 0;

    taskENTER_CRITICAL(&s_bt_buf_mux);

    if (n > s_bt_buf_count) n = s_bt_buf_count;

    size_t first = s_bt_buf_size - s_bt_buf_r;
    if (first > n) first = n;
    memcpy(dst, s_bt_buf + s_bt_buf_r, first);

    size_t second = n - first;
    if (second > 0) {
        memcpy(dst + first, s_bt_buf, second);
    }

    s_bt_buf_r = (s_bt_buf_r + n) % s_bt_buf_size;
    s_bt_buf_count -= n;

    taskEXIT_CRITICAL(&s_bt_buf_mux);
    return n;
}

static bool bt_ensure_buffer(void)
{
    if (s_bt_buf && s_bt_buf_size >= BT_STREAM_BUF_BYTES_MIN) {
        return true;
    }

    const size_t candidates[] = {
        BT_STREAM_BUF_BYTES_MAX,
        128 * 1024,
        BT_STREAM_BUF_BYTES_MIN,
    };

    uint8_t *buf = NULL;
    size_t chosen = 0;
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        size_t want = candidates[i];
        buf = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!buf) {
            buf = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_8BIT);
        }
        if (buf) {
            chosen = want;
            break;
        }
    }

    if (!buf || chosen == 0) {
        ESP_LOGE(TAG,
                 "Failed to allocate BT stream buffer (max=%u min=%u)",
                 (unsigned)BT_STREAM_BUF_BYTES_MAX,
                 (unsigned)BT_STREAM_BUF_BYTES_MIN);
        return false;
    }

    if (s_bt_buf) {
        free(s_bt_buf);
    }
    s_bt_buf = buf;
    s_bt_buf_size = chosen;
    bt_buf_reset();
    ESP_LOGI(TAG, "BT stream buffer: %u bytes", (unsigned)s_bt_buf_size);
    return true;
}

static void bt_release_buffer(void)
{
    taskENTER_CRITICAL(&s_bt_buf_mux);
    s_bt_buf_r = 0;
    s_bt_buf_w = 0;
    s_bt_buf_count = 0;
    taskEXIT_CRITICAL(&s_bt_buf_mux);

    if (s_bt_buf) {
        free(s_bt_buf);
        s_bt_buf = NULL;
    }
    s_bt_buf_size = 0;
}

// Pull bytes for the A2DP callback:
// from RAM ring buffer (normal buffered path).
static size_t bt_stream_pull(uint8_t *dst, size_t n)
{
    if (!dst || n == 0) return 0;

    size_t rd = bt_buf_read(dst, n);
    if (rd > 0 || !s_bt_direct_stream) {
        return rd;
    }

    if (!s_bt_fp || s_bt_bytes_left == 0 || s_bt_file_eof) {
        return 0;
    }

    size_t want = n;
    if (want > s_bt_bytes_left) want = s_bt_bytes_left;
    if (want == 0) {
        s_bt_file_eof = true;
        return 0;
    }

    bool last_chunk = (want == s_bt_bytes_left);
    if (!lock_fp(0)) {
        return 0;
    }

    if (s_bt_fp) {
        rd = fread(dst, 1, want, s_bt_fp);
    } else {
        rd = 0;
    }
    unlock_fp();

    if (rd == 0) {
        s_bt_file_eof = true;
        return 0;
    }
    if (rd < want || last_chunk) {
        s_bt_file_eof = true;
    }

    return rd;
}

static bool bt_stop_feeder(TickType_t wait_ticks)
{
    if (!s_bt_feeder_task) return true;

    s_bt_feeder_stop = true;
    if (wait_ticks == 0) {
        return false;
    }

    TickType_t start = xTaskGetTickCount();
    while (s_bt_feeder_task) {
        if (wait_ticks > 0 && (xTaskGetTickCount() - start) >= wait_ticks) {
            ESP_LOGW(TAG, "Timeout stopping BT feeder task");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

static void bt_file_feeder_task(void *arg)
{
    (void)arg;

    while (!s_bt_feeder_stop) {
        FILE *fp_local = s_bt_fp;
        if (!fp_local) break;

        if (s_bt_bytes_left == 0) {
            s_bt_file_eof = true;
            break;
        }

        size_t space = bt_buf_space();
        if (space < (BT_FEEDER_CHUNK_BYTES / 2)) {
            vTaskDelay(pdMS_TO_TICKS(BT_FEEDER_POLL_MS));
            continue;
        }

        size_t want = BT_FEEDER_CHUNK_BYTES;
        if (want > space) want = space;
        if (want > s_bt_bytes_left) want = s_bt_bytes_left;
        if (want == 0) {
            vTaskDelay(pdMS_TO_TICKS(BT_FEEDER_POLL_MS));
            continue;
        }

        size_t rd = 0;
        if (lock_fp(pdMS_TO_TICKS(20))) {
            if (s_bt_fp) {
                rd = fread(s_bt_feeder_chunk, 1, want, s_bt_fp);
            }
            unlock_fp();
        } else {
            // Contended by control path; retry instead of forcing EOF.
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        if (rd > 0) {
            (void)bt_buf_write(s_bt_feeder_chunk, rd);
            if (rd < want) {
                // Reached EOF earlier than expected data_size; clamp remaining
                // bytes so playback can end cleanly.
                s_bt_file_eof = true;
                size_t level = bt_buf_level();
                if (s_bt_bytes_left > level) {
                    s_bt_bytes_left = (uint32_t)level;
                }
                break;
            }
            continue;
        }

        s_bt_file_eof = true;
        size_t level = bt_buf_level();
        if (s_bt_bytes_left > level) {
            s_bt_bytes_left = (uint32_t)level;
        }
        break;
    }

    // Best-effort file close owned by feeder context. Control path may already
    // have closed/nullified it; keep this idempotent.
    if (lock_fp(pdMS_TO_TICKS(50))) {
        if (s_bt_fp) {
            fclose(s_bt_fp);
            s_bt_fp = NULL;
        }
        unlock_fp();
    }

    s_bt_feeder_task = NULL;
    vTaskDelete(NULL);
}

static void close_bt_file_locked(void)
{
    if (s_bt_fp) {
        fclose(s_bt_fp);
        s_bt_fp = NULL;
    }
    s_bt_bytes_left = 0;
    s_bt_file_eof = false;
    s_bt_direct_stream = false;
    bt_buf_reset();
}

static bool close_bt_file(TickType_t wait_ticks)
{
    (void)bt_stop_feeder(wait_ticks);
    if (lock_fp(wait_ticks)) {
        close_bt_file_locked();
        unlock_fp();
        return true;
    }

    s_bt_bytes_left = 0;
    // Mark EOF so callback-side close path can finish once feeder exits.
    s_bt_file_eof = true;
    bt_buf_reset();
    ESP_LOGW(TAG, "Failed to lock BT file for close");
    return false;
}

void bt_audio_stream_close(void)
{
    (void)close_bt_file(pdMS_TO_TICKS(100));
}

size_t bt_audio_stream_queued_frames(void)
{
    size_t bytes_per_frame = (s_bt_channels == 2) ? 4 : 2;
    return bytes_per_frame ? (s_bt_bytes_left / bytes_per_frame) : 0;
}

int32_t bt_audio_a2dp_data_cb(uint8_t *data, int32_t len)
{
    if (!data || len <= 0) return 0;

    if (s_bt_bytes_left == 0 || (s_bt_file_eof && bt_buf_level() == 0)) {
        memset(data, 0, len);
        return len;
    }

    // Stereo fast path (16-bit): consume bytes from RAM buffer.
    if (s_bt_channels == 2) {
        size_t to_read = (size_t)len;
        if (to_read > s_bt_bytes_left) to_read = s_bt_bytes_left;
        size_t rd = bt_stream_pull(data, to_read);
        if (rd < (size_t)len) {
            memset(data + rd, 0, (size_t)len - rd);
        }
        if (s_bt_bytes_left >= rd) s_bt_bytes_left -= (uint32_t)rd;
        else s_bt_bytes_left = 0;
    } else { // mono -> duplicate to stereo
        size_t frames_rem = (size_t)len / 4; // stereo frames requested
        uint8_t mono_buf[1024];
        size_t copied = 0;
        uint8_t *out_bytes = data;

        while (frames_rem > 0 && s_bt_bytes_left > 0) {
            size_t chunk_frames = frames_rem;
            size_t chunk_mono_bytes = chunk_frames * sizeof(int16_t);
            if (chunk_mono_bytes > sizeof(mono_buf)) {
                chunk_mono_bytes = sizeof(mono_buf);
                chunk_frames = chunk_mono_bytes / sizeof(int16_t);
            }
            if (chunk_frames == 0) break;

            if (chunk_mono_bytes > s_bt_bytes_left) {
                chunk_mono_bytes = s_bt_bytes_left;
                chunk_frames = chunk_mono_bytes / sizeof(int16_t);
            }

            size_t rd = bt_stream_pull(mono_buf, chunk_mono_bytes);
            if (rd == 0) {
                break;
            }
            if (s_bt_bytes_left >= rd) s_bt_bytes_left -= (uint32_t)rd;
            else s_bt_bytes_left = 0;
            size_t got_frames = rd / sizeof(int16_t);
            int16_t *mono = (int16_t*)mono_buf;
            int16_t *out = (int16_t*)out_bytes;
            for (size_t i = 0; i < got_frames; ++i) {
                int16_t s = mono[i];
                out[2 * i + 0] = s;
                out[2 * i + 1] = s;
            }
            size_t written_bytes = got_frames * 4;
            out_bytes += written_bytes;
            copied += written_bytes;
            frames_rem -= got_frames;
            if (got_frames < chunk_frames) {
                break;
            }
        }
        if (copied < (size_t)len) {
            memset(out_bytes, 0, (size_t)len - copied);
        }
    }

    // Apply volume gain (Q15) to the PCM buffer if not unity.
    if (s_bt_vol_q15 != 32767) {
        int16_t *pcm = (int16_t*)data;
        size_t samples = (size_t)len / sizeof(int16_t);
        int32_t gain = s_bt_vol_q15;
        for (size_t i = 0; i < samples; ++i) {
            int32_t v = (int32_t)pcm[i] * gain;
            v >>= 15;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            pcm[i] = (int16_t)v;
        }
    }

    if (s_bt_bytes_left == 0 && s_bt_file_eof && s_bt_feeder_task == NULL && s_bt_fp) {
        if (lock_fp(0)) {
            if (s_bt_fp) {
                fclose(s_bt_fp);
                s_bt_fp = NULL;
            }
            unlock_fp();
        }
    }
    return len;
}

esp_err_t bt_audio_play_wav(FILE *fp,
                            uint32_t data_offset,
                            uint32_t data_size,
                            uint32_t sample_rate,
                            uint16_t num_channels,
                            uint16_t bits_per_sample)
{
    if (!fp || data_size == 0) return ESP_ERR_INVALID_ARG;
    if (bits_per_sample != 16) return ESP_ERR_NOT_SUPPORTED;
    if (num_channels != 1 && num_channels != 2) return ESP_ERR_NOT_SUPPORTED;
    if (bt_audio_media_cmd_pending()) return ESP_ERR_INVALID_STATE;

    int peer_rate = bt_audio_get_peer_sample_rate();
    if (peer_rate <= 0) peer_rate = (int)sample_rate;
    if (peer_rate != (int)sample_rate) {
        ESP_LOGW(TAG, "Rate mismatch: wav=%u peer=%d, playing anyway (pitch may be off)",
                 sample_rate, peer_rate);
    }

    if (fseek(fp, (long)data_offset, SEEK_SET) != 0) {
        ESP_LOGW(TAG, "bt_audio_play_wav: fseek failed");
        return ESP_FAIL;
    }

    bool use_buffered_stream = bt_ensure_buffer();
    if (!use_buffered_stream) {
        ESP_LOGW(TAG, "bt_audio_play_wav: stream buffer alloc failed, using direct stream fallback");
    }

    setvbuf(fp, NULL, _IOFBF, 32 * 1024);

    esp_err_t err = bt_audio_start(false);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "bt_audio_play_wav: bt_audio_start failed (%s)", esp_err_to_name(err));
        return err;
    }

    (void)bt_stop_feeder(pdMS_TO_TICKS(500));

    // Atomically replace the stream source so callback/control paths never
    // observe partially-updated FILE state across track switches.
    if (!lock_fp(pdMS_TO_TICKS(500))) {
        ESP_LOGW(TAG, "bt_audio_play_wav: stream lock timeout");
        return ESP_ERR_TIMEOUT;
    }
    close_bt_file_locked();
    s_bt_fp              = fp;
    s_bt_bytes_left      = data_size;
    s_bt_channels        = num_channels;
    bt_buf_reset();
    s_bt_file_eof = false;
    s_bt_feeder_stop = false;
    s_bt_direct_stream = !use_buffered_stream;
    unlock_fp();

    if (use_buffered_stream && !s_bt_feeder_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            bt_file_feeder_task,
            "bt_feed",
            BT_FEEDER_TASK_STACK,
            NULL,
            BT_FEEDER_TASK_PRIO,
            &s_bt_feeder_task,
            BT_FEEDER_TASK_CORE
        );
#if CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
        if (ok != pdPASS) {
            ok = xTaskCreatePinnedToCoreWithCaps(
                bt_file_feeder_task,
                "bt_feed",
                BT_FEEDER_TASK_STACK,
                NULL,
                BT_FEEDER_TASK_PRIO,
                &s_bt_feeder_task,
                BT_FEEDER_TASK_CORE,
                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
            );
            if (ok == pdPASS) {
                ESP_LOGW(TAG, "BT feeder task allocated in PSRAM");
            }
        }
#endif
        if (ok != pdPASS) {
            s_bt_feeder_task = NULL;
            s_bt_direct_stream = true;
            bt_release_buffer();
            ESP_LOGW(TAG,
                     "BT feeder task create failed, using direct stream fallback "
                     "(internal free=%u largest=%u)",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        }
    }

    if (!s_bt_direct_stream) {
        TickType_t start_tick = xTaskGetTickCount();
        while (!s_bt_file_eof && bt_buf_level() < BT_STREAM_START_WATERMARK) {
            if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(250)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    err = bt_audio_start_stream();
    if (err != ESP_OK) {
        s_bt_feeder_stop = true;
        (void)bt_stop_feeder(pdMS_TO_TICKS(300));
        if (lock_fp(pdMS_TO_TICKS(200))) {
            if (s_bt_fp == fp) {
                // Start failed; caller retains FILE ownership on failure paths.
                s_bt_fp = NULL;
                s_bt_bytes_left = 0;
                bt_buf_reset();
            }
            unlock_fp();
        }
        ESP_LOGW(TAG, "bt_audio_play_wav: start_stream failed (%s)", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BT WAV playback started: %u bytes @ %u Hz, ch=%u",
             data_size, sample_rate, num_channels);
    return ESP_OK;
}

void bt_audio_volume_set_percent(int percent)
{
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }

    s_bt_volume_percent = percent;
    bt_audio_update_vol_q15_from_percent();
    ESP_LOGI(TAG, "Volume set to %d%% (Q15=%ld)",
             s_bt_volume_percent, (long)s_bt_vol_q15);
}

int bt_audio_volume_get_percent(void)
{
    return s_bt_volume_percent;
}

void bt_audio_volume_up(void)
{
    // step size: 5%, saturating at 100.
    bt_audio_volume_set_percent(s_bt_volume_percent + 5);
}

void bt_audio_volume_down(void)
{
    // step size: 5%, saturating at 0.
    bt_audio_volume_set_percent(s_bt_volume_percent - 5);
}
