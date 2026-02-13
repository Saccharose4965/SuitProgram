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
static FILE    *s_bt_fp               = NULL;
static uint32_t s_bt_src_bytes_left   = 0; // bytes left in source WAV payload
static uint32_t s_bt_bytes_left       = 0; // bytes left to output to A2DP (stereo)
static uint16_t s_bt_channels         = 2; // source channels: 1 or 2
static SemaphoreHandle_t s_bt_fp_lock = NULL; // protects FILE access

#define BT_STREAM_BUF_BYTES_MAX        (256 * 1024)
#define BT_STREAM_BUF_BYTES_MIN        (64 * 1024)
#define BT_STREAM_START_WATERMARK      (12 * 1024)
#define BT_FEEDER_OUT_CHUNK_BYTES      (8 * 1024)
#define BT_FEEDER_MONO_IN_CHUNK_BYTES  (BT_FEEDER_OUT_CHUNK_BYTES / 2)
#define BT_DIRECT_MONO_CHUNK_BYTES     1024
#define BT_FEEDER_WAIT_MS              100
#define BT_FEEDER_TASK_STACK           3072
#define BT_FEEDER_TASK_PRIO            4
#define BT_FEEDER_TASK_CORE            1

static uint8_t *s_bt_buf                 = NULL;
static size_t   s_bt_buf_size            = 0;
static size_t   s_bt_buf_r               = 0;
static size_t   s_bt_buf_w               = 0;
static size_t   s_bt_buf_count           = 0;
static uint8_t *s_bt_feeder_chunk         = NULL;
static uint8_t *s_bt_feeder_mono_chunk    = NULL;
static uint8_t *s_bt_direct_mono_chunk    = NULL;
static volatile bool s_bt_file_eof       = false;
static volatile bool s_bt_feeder_stop    = false;
static volatile bool s_bt_stream_paused  = false;
static volatile bool s_bt_direct_stream  = false;
static TaskHandle_t  s_bt_feeder_task    = NULL;
static portMUX_TYPE  s_bt_buf_mux        = portMUX_INITIALIZER_UNLOCKED;

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

static size_t bt_buf_low_watermark(void)
{
    if (s_bt_buf_size == 0) return BT_STREAM_START_WATERMARK;
    size_t low = s_bt_buf_size / 4;
    if (low < BT_STREAM_START_WATERMARK) low = BT_STREAM_START_WATERMARK;
    return low;
}

static size_t bt_buf_high_watermark(void)
{
    if (s_bt_buf_size == 0) return BT_STREAM_START_WATERMARK;
    size_t high = (s_bt_buf_size * 3) / 4;
    size_t min_high = bt_buf_low_watermark() + BT_FEEDER_OUT_CHUNK_BYTES;
    if (high < min_high) high = min_high;
    if (high > s_bt_buf_size) high = s_bt_buf_size;
    return high;
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

static void *bt_alloc_work_buffer(size_t bytes)
{
    void *buf = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    }
    return buf;
}

static bool bt_ensure_feeder_buffers(bool mono_source)
{
    if (!s_bt_feeder_chunk) {
        s_bt_feeder_chunk = (uint8_t*)bt_alloc_work_buffer(BT_FEEDER_OUT_CHUNK_BYTES);
        if (!s_bt_feeder_chunk) {
            ESP_LOGE(TAG, "Failed to alloc BT feeder chunk (%u bytes)",
                     (unsigned)BT_FEEDER_OUT_CHUNK_BYTES);
            return false;
        }
    }
    if (mono_source && !s_bt_feeder_mono_chunk) {
        s_bt_feeder_mono_chunk = (uint8_t*)bt_alloc_work_buffer(BT_FEEDER_MONO_IN_CHUNK_BYTES);
        if (!s_bt_feeder_mono_chunk) {
            ESP_LOGE(TAG, "Failed to alloc BT feeder mono chunk (%u bytes)",
                     (unsigned)BT_FEEDER_MONO_IN_CHUNK_BYTES);
            return false;
        }
    }
    return true;
}

static bool bt_ensure_direct_mono_buffer(void)
{
    if (s_bt_direct_mono_chunk) return true;
    s_bt_direct_mono_chunk = (uint8_t*)bt_alloc_work_buffer(BT_DIRECT_MONO_CHUNK_BYTES);
    if (!s_bt_direct_mono_chunk) {
        ESP_LOGE(TAG, "Failed to alloc BT direct mono chunk (%u bytes)",
                 (unsigned)BT_DIRECT_MONO_CHUNK_BYTES);
        return false;
    }
    return true;
}

static inline void bt_feeder_notify(void)
{
    TaskHandle_t t = s_bt_feeder_task;
    if (t) {
        xTaskNotifyGive(t);
    }
}

static void bt_apply_gain_pcm16(uint8_t *buf, size_t bytes)
{
    if (!buf || bytes < 2 || s_bt_vol_q15 == 32767) return;

    size_t samples = (bytes & ~(size_t)1) / sizeof(int16_t);
    int16_t *pcm = (int16_t*)buf;
    int32_t gain = s_bt_vol_q15;
    for (size_t i = 0; i < samples; ++i) {
        int32_t v = (int32_t)pcm[i] * gain;
        v >>= 15;
        if (v > 32767) v = 32767;
        else if (v < -32768) v = -32768;
        pcm[i] = (int16_t)v;
    }
}

static size_t bt_read_stereo_chunk_locked(size_t want_out_bytes)
{
    if (!s_bt_feeder_chunk || !s_bt_fp || s_bt_src_bytes_left == 0 || want_out_bytes == 0) return 0;

    size_t want = want_out_bytes;
    if (want > s_bt_src_bytes_left) want = s_bt_src_bytes_left;
    size_t rd = fread(s_bt_feeder_chunk, 1, want, s_bt_fp);
    if (rd > 0) {
        s_bt_src_bytes_left -= (uint32_t)rd;
        bt_apply_gain_pcm16(s_bt_feeder_chunk, rd);
    }
    return rd;
}

static size_t bt_read_mono_to_stereo_chunk_locked(size_t want_out_bytes)
{
    if (!s_bt_feeder_chunk || !s_bt_feeder_mono_chunk ||
        !s_bt_fp || s_bt_src_bytes_left < sizeof(int16_t) || want_out_bytes < 4) return 0;

    size_t frame_cap = want_out_bytes / 4; // output stereo frames
    size_t want_in = frame_cap * sizeof(int16_t);
    if (want_in > BT_FEEDER_MONO_IN_CHUNK_BYTES) want_in = BT_FEEDER_MONO_IN_CHUNK_BYTES;
    if (want_in > s_bt_src_bytes_left) want_in = s_bt_src_bytes_left;
    want_in &= ~(size_t)1;
    if (want_in == 0) return 0;

    size_t rd = fread(s_bt_feeder_mono_chunk, 1, want_in, s_bt_fp);
    rd &= ~(size_t)1;
    if (rd == 0) return 0;

    s_bt_src_bytes_left -= (uint32_t)rd;

    size_t frames = rd / sizeof(int16_t);
    int16_t *mono = (int16_t*)s_bt_feeder_mono_chunk;
    int16_t *out = (int16_t*)s_bt_feeder_chunk;
    int32_t gain = s_bt_vol_q15;

    for (size_t i = 0; i < frames; ++i) {
        int16_t s = mono[i];
        if (gain != 32767) {
            int32_t v = ((int32_t)s * gain) >> 15;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            s = (int16_t)v;
        }
        out[2 * i + 0] = s;
        out[2 * i + 1] = s;
    }

    return frames * 4;
}

static size_t bt_direct_read_stereo_locked(uint8_t *dst, size_t want_out_bytes)
{
    if (!s_bt_fp || s_bt_src_bytes_left == 0 || !dst || want_out_bytes == 0) return 0;

    size_t want = want_out_bytes;
    if (want > s_bt_src_bytes_left) want = s_bt_src_bytes_left;

    size_t rd = fread(dst, 1, want, s_bt_fp);
    if (rd > 0) {
        s_bt_src_bytes_left -= (uint32_t)rd;
        bt_apply_gain_pcm16(dst, rd);
    }
    return rd;
}

static size_t bt_direct_read_mono_locked(uint8_t *dst, size_t want_out_bytes)
{
    if (!s_bt_direct_mono_chunk || !s_bt_fp || s_bt_src_bytes_left < sizeof(int16_t) ||
        !dst || want_out_bytes < 4) return 0;

    size_t frame_cap = want_out_bytes / 4;
    size_t want_in = frame_cap * sizeof(int16_t);
    if (want_in > BT_DIRECT_MONO_CHUNK_BYTES) want_in = BT_DIRECT_MONO_CHUNK_BYTES;
    if (want_in > s_bt_src_bytes_left) want_in = s_bt_src_bytes_left;
    want_in &= ~(size_t)1;
    if (want_in == 0) return 0;

    size_t rd = fread(s_bt_direct_mono_chunk, 1, want_in, s_bt_fp);
    rd &= ~(size_t)1;
    if (rd == 0) return 0;

    s_bt_src_bytes_left -= (uint32_t)rd;

    size_t frames = rd / sizeof(int16_t);
    int16_t *mono = (int16_t*)s_bt_direct_mono_chunk;
    int16_t *out = (int16_t*)dst;
    int32_t gain = s_bt_vol_q15;

    for (size_t i = 0; i < frames; ++i) {
        int16_t s = mono[i];
        if (gain != 32767) {
            int32_t v = ((int32_t)s * gain) >> 15;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            s = (int16_t)v;
        }
        out[2 * i + 0] = s;
        out[2 * i + 1] = s;
    }

    return frames * 4;
}

static bool bt_stop_feeder(TickType_t wait_ticks)
{
    if (!s_bt_feeder_task) return true;

    s_bt_feeder_stop = true;
    bt_feeder_notify();
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
        if (s_bt_stream_paused) {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BT_FEEDER_WAIT_MS));
            continue;
        }

        FILE *fp_local = s_bt_fp;
        if (!fp_local) break;

        if (s_bt_src_bytes_left == 0) {
            s_bt_file_eof = true;
            break;
        }

        size_t level = bt_buf_level();
        if (level >= bt_buf_high_watermark()) {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BT_FEEDER_WAIT_MS));
            continue;
        }

        size_t space = bt_buf_space();
        if (space < (BT_FEEDER_OUT_CHUNK_BYTES / 2)) {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BT_FEEDER_WAIT_MS));
            continue;
        }

        size_t want_out = BT_FEEDER_OUT_CHUNK_BYTES;
        if (want_out > space) want_out = space;

        size_t produced = 0;
        if (lock_fp(pdMS_TO_TICKS(20))) {
            if (s_bt_fp) {
                if (s_bt_channels == 2) {
                    produced = bt_read_stereo_chunk_locked(want_out);
                } else {
                    produced = bt_read_mono_to_stereo_chunk_locked(want_out);
                }
            }
            unlock_fp();
        } else {
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            continue;
        }

        if (produced > 0) {
            (void)bt_buf_write(s_bt_feeder_chunk, produced);
            continue;
        }

        s_bt_file_eof = true;
        size_t level_now = bt_buf_level();
        if (s_bt_bytes_left > level_now) {
            s_bt_bytes_left = (uint32_t)level_now;
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
    s_bt_src_bytes_left = 0;
    s_bt_bytes_left = 0;
    s_bt_file_eof = false;
    s_bt_stream_paused = false;
    s_bt_direct_stream = false;
    bt_buf_reset();
}

static bool close_bt_file(TickType_t wait_ticks)
{
    bool feeder_stopped = bt_stop_feeder(wait_ticks);
    if (!feeder_stopped && s_bt_feeder_task) {
        s_bt_src_bytes_left = 0;
        s_bt_bytes_left = 0;
        s_bt_file_eof = true;
        s_bt_stream_paused = false;
        s_bt_direct_stream = false;
        bt_buf_reset();
        return false;
    }
    if (lock_fp(wait_ticks)) {
        close_bt_file_locked();
        unlock_fp();
        return true;
    }

    s_bt_src_bytes_left = 0;
    s_bt_bytes_left = 0;
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
    return s_bt_bytes_left / 4; // always output stereo frames
}

int32_t bt_audio_a2dp_data_cb(uint8_t *data, int32_t len)
{
    if (!data || len <= 0) return 0;

    if (s_bt_stream_paused) {
        memset(data, 0, len);
        return len;
    }

    if (s_bt_bytes_left == 0 || (s_bt_file_eof && bt_buf_level() == 0)) {
        memset(data, 0, len);
        return len;
    }

    if (!s_bt_direct_stream) {
        size_t to_read = (size_t)len;
        if (to_read > s_bt_bytes_left) to_read = s_bt_bytes_left;

        size_t rd = bt_buf_read(data, to_read);
        if (rd < (size_t)len) {
            memset(data + rd, 0, (size_t)len - rd);
        }

        if (s_bt_bytes_left >= rd) s_bt_bytes_left -= (uint32_t)rd;
        else s_bt_bytes_left = 0;

        if (rd > 0 && s_bt_feeder_task && !s_bt_stream_paused) {
            if (bt_buf_level() <= bt_buf_low_watermark()) {
                bt_feeder_notify();
            }
        }

        return len;
    }

    // Direct fallback mode (no ring buffer/feeder): pull from file in callback.
    size_t copied = 0;
    while (copied < (size_t)len && s_bt_bytes_left > 0) {
        size_t want_out = (size_t)len - copied;
        if (want_out > s_bt_bytes_left) want_out = s_bt_bytes_left;

        size_t got = 0;
        if (lock_fp(0)) {
            if (s_bt_fp) {
                if (s_bt_channels == 2) {
                    got = bt_direct_read_stereo_locked(data + copied, want_out);
                } else {
                    got = bt_direct_read_mono_locked(data + copied, want_out);
                }
            }
            unlock_fp();
        }

        if (got == 0) {
            s_bt_file_eof = true;
            break;
        }

        copied += got;
        if (s_bt_bytes_left >= got) s_bt_bytes_left -= (uint32_t)got;
        else s_bt_bytes_left = 0;
    }

    if (copied < (size_t)len) {
        memset(data + copied, 0, (size_t)len - copied);
    }

    if ((s_bt_bytes_left == 0 || s_bt_src_bytes_left == 0) && s_bt_fp) {
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
    if (use_buffered_stream && !bt_ensure_feeder_buffers(num_channels == 1)) {
        ESP_LOGW(TAG, "bt_audio_play_wav: feeder buffer alloc failed, using direct stream fallback");
        use_buffered_stream = false;
    }

    setvbuf(fp, NULL, _IOFBF, 32 * 1024);

    esp_err_t err = bt_audio_start(false);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "bt_audio_play_wav: bt_audio_start failed (%s)", esp_err_to_name(err));
        return err;
    }

    if (!bt_stop_feeder(pdMS_TO_TICKS(500)) && s_bt_feeder_task) {
        ESP_LOGW(TAG, "bt_audio_play_wav: previous feeder still stopping; drop request");
        return ESP_ERR_TIMEOUT;
    }

    uint64_t out_total = (num_channels == 1)
        ? ((uint64_t)data_size * 2ULL)
        : (uint64_t)data_size;
    if (out_total > UINT32_MAX) {
        ESP_LOGW(TAG, "bt_audio_play_wav: stream too large (%llu bytes)",
                 (unsigned long long)out_total);
        return ESP_ERR_INVALID_SIZE;
    }

    // Atomically replace the stream source so callback/control paths never
    // observe partially-updated FILE state across track switches.
    if (!lock_fp(pdMS_TO_TICKS(500))) {
        ESP_LOGW(TAG, "bt_audio_play_wav: stream lock timeout");
        return ESP_ERR_TIMEOUT;
    }
    close_bt_file_locked();
    s_bt_fp               = fp;
    s_bt_src_bytes_left   = data_size;
    s_bt_bytes_left       = (uint32_t)out_total;
    s_bt_channels         = num_channels;
    bt_buf_reset();
    s_bt_file_eof = false;
    s_bt_feeder_stop = false;
    s_bt_stream_paused = false;
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

    if (s_bt_direct_stream && num_channels == 1 && !bt_ensure_direct_mono_buffer()) {
        ESP_LOGW(TAG, "bt_audio_play_wav: direct mono buffer alloc failed");
        if (lock_fp(pdMS_TO_TICKS(100))) {
            if (s_bt_fp == fp) {
                s_bt_fp = NULL;
                s_bt_src_bytes_left = 0;
                s_bt_bytes_left = 0;
                s_bt_file_eof = false;
                s_bt_stream_paused = false;
                s_bt_direct_stream = false;
                bt_buf_reset();
            }
            unlock_fp();
        }
        return ESP_ERR_NO_MEM;
    }

    if (!s_bt_direct_stream && s_bt_feeder_task) {
        bt_feeder_notify();
        TickType_t start_tick = xTaskGetTickCount();
        while (!s_bt_file_eof && bt_buf_level() < BT_STREAM_START_WATERMARK) {
            if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(250)) {
                break;
            }
            bt_feeder_notify();
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    err = bt_audio_start_stream();
    if (err != ESP_OK) {
        s_bt_feeder_stop = true;
        bt_feeder_notify();
        (void)bt_stop_feeder(pdMS_TO_TICKS(300));
        if (lock_fp(pdMS_TO_TICKS(200))) {
            if (s_bt_fp == fp) {
                // Start failed; caller retains FILE ownership on failure paths.
                s_bt_fp = NULL;
                s_bt_src_bytes_left = 0;
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

void bt_audio_stream_set_paused(bool paused)
{
    s_bt_stream_paused = paused;
    if (!paused) {
        bt_feeder_notify();
    }
}
