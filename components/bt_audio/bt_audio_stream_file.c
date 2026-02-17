#include "bt_audio.h"
#include "bt_audio_internal.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

static const char *TAG = "bt_audio";

// ============================ File-backed source ============================
static FILE    *s_bt_fp               = NULL;
static uint32_t s_bt_src_bytes_left   = 0; // bytes left in source WAV payload
static uint32_t s_bt_bytes_left       = 0; // bytes left to output to A2DP (stereo)
static SemaphoreHandle_t s_bt_fp_lock = NULL; // protects FILE access

#define BT_STREAM_BUF_BYTES_MAX        (256 * 1024)
#define BT_STREAM_BUF_BYTES_MIN        (128 * 1024)
#define BT_STREAM_START_WATERMARK      (12 * 1024)
#define BT_STREAM_LOW_WATERMARK_MAX    (32 * 1024)
#define BT_STREAM_REFILL_WINDOW_BYTES  (32 * 1024)
#define BT_FEEDER_OUT_CHUNK_BYTES      (16 * 1024)
#define BT_FEEDER_MONO_IN_CHUNK_BYTES  (BT_FEEDER_OUT_CHUNK_BYTES / 2)
#define BT_FEEDER_MONO_READ_MAX_BYTES  (4 * 1024)
#define BT_FEEDER_TASK_STACK           4096
#define BT_FEEDER_TASK_PRIO            3
#define BT_FEEDER_TASK_CORE            1
#define BT_FEEDER_YIELD_EVERY          4
#define BT_AUDIO_DIAG_LOG_ENABLE       0
#define BT_DIAG_LOG_MS                 5000

static uint8_t *s_bt_buf                 = NULL;
static size_t   s_bt_buf_size            = 0;
static volatile size_t s_bt_buf_r        = 0; // consumer-owned
static volatile size_t s_bt_buf_w        = 0; // producer-owned
static uint8_t *s_bt_feeder_chunk         = NULL;
static uint8_t *s_bt_feeder_mono_chunk    = NULL;
static volatile bool s_bt_file_eof       = false;
static volatile bool s_bt_feeder_stop    = false;
static volatile bool s_bt_stream_paused  = false;
static TaskHandle_t  s_bt_feeder_task    = NULL;

typedef struct {
    volatile uint32_t cb_calls;
    volatile uint32_t cb_partial_reads;
    volatile uint32_t cb_underruns;
    volatile uint32_t cb_req_bytes;
    volatile uint32_t cb_read_bytes;
    volatile uint32_t cb_zero_fill_bytes;
    volatile uint32_t cb_last_us;
    volatile uint32_t cb_max_us;
    volatile uint32_t cb_win_max_us;

    volatile uint32_t ring_level_min;
    volatile uint32_t ring_level_max;

    volatile uint32_t feeder_loops;
    volatile uint32_t feeder_bytes;
    volatile uint32_t feeder_wait_pause;
    volatile uint32_t feeder_wait_high;
    volatile uint32_t feeder_wait_space;
    volatile uint32_t feeder_wait_lock;
    volatile uint32_t feeder_yields;
} bt_diag_t;

static bt_diag_t s_bt_diag = {0};
static int64_t s_bt_diag_last_log_us = 0;
static bt_diag_t s_bt_diag_prev = {0};

static size_t bt_buf_level(void);

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

static inline void bt_diag_note_level(size_t level)
{
    uint32_t lvl = (uint32_t)level;
    if (lvl > s_bt_diag.ring_level_max) {
        s_bt_diag.ring_level_max = lvl;
    }
    if (s_bt_diag.ring_level_min == UINT32_MAX || lvl < s_bt_diag.ring_level_min) {
        s_bt_diag.ring_level_min = lvl;
    }
}

static void bt_diag_reset(void)
{
    memset((void *)&s_bt_diag, 0, sizeof(s_bt_diag));
    memset((void *)&s_bt_diag_prev, 0, sizeof(s_bt_diag_prev));
    s_bt_diag.ring_level_min = UINT32_MAX;
    s_bt_diag_prev.ring_level_min = UINT32_MAX;
    s_bt_diag_last_log_us = esp_timer_get_time();
}

static void bt_diag_log_periodic(void)
{
#if !BT_AUDIO_DIAG_LOG_ENABLE
    return;
#else
    int64_t now_us = esp_timer_get_time();
    if (s_bt_diag_last_log_us != 0 &&
        (now_us - s_bt_diag_last_log_us) < ((int64_t)BT_DIAG_LOG_MS * 1000LL)) {
        return;
    }

    bt_diag_t cur = s_bt_diag;
    bt_diag_t prev = s_bt_diag_prev;
    s_bt_diag_prev = cur;
    int64_t dt_us = now_us - s_bt_diag_last_log_us;
    s_bt_diag_last_log_us = now_us;

    if (dt_us <= 0) dt_us = 1;
    uint32_t cb_calls = cur.cb_calls - prev.cb_calls;
    uint32_t cb_part = cur.cb_partial_reads - prev.cb_partial_reads;
    uint32_t cb_und = cur.cb_underruns - prev.cb_underruns;
    uint32_t cb_req = cur.cb_req_bytes - prev.cb_req_bytes;
    uint32_t cb_rd = cur.cb_read_bytes - prev.cb_read_bytes;
    uint32_t cb_zf = cur.cb_zero_fill_bytes - prev.cb_zero_fill_bytes;
    uint32_t feed_bytes = cur.feeder_bytes - prev.feeder_bytes;
    uint32_t feed_loops = cur.feeder_loops - prev.feeder_loops;
    uint32_t wp = cur.feeder_wait_pause - prev.feeder_wait_pause;
    uint32_t wh = cur.feeder_wait_high - prev.feeder_wait_high;
    uint32_t ws = cur.feeder_wait_space - prev.feeder_wait_space;
    uint32_t wl = cur.feeder_wait_lock - prev.feeder_wait_lock;
    uint32_t wy = cur.feeder_yields - prev.feeder_yields;

    uint32_t cb_rate = (uint32_t)((uint64_t)cb_calls * 1000000ULL / (uint64_t)dt_us);
    uint32_t feed_kb_s = (uint32_t)(((uint64_t)feed_bytes * 1000ULL / 1024ULL) /
                                    (uint64_t)BT_DIAG_LOG_MS);
    uint32_t ring_min = (cur.ring_level_min == UINT32_MAX) ? 0 : cur.ring_level_min;
    uint32_t ring_max = cur.ring_level_max;
    uint32_t ring_now = (uint32_t)bt_buf_level();
    uint32_t cb_win_max = cur.cb_win_max_us;

    ESP_LOGI(TAG,
             "diag: cb=%u/s part=%u und=%u req=%u rd=%u zf=%u maxcb(win/tot)=%u/%uus "
             "ring(now/min/max)=%u/%u/%u feed=%uKB/s loops=%u waits[p/h/s/l]=%u/%u/%u/%u yld=%u",
             cb_rate, cb_part, cb_und, cb_req, cb_rd, cb_zf, cb_win_max, cur.cb_max_us,
             ring_now, ring_min, ring_max, feed_kb_s, feed_loops, wp, wh, ws, wl, wy);

    s_bt_diag.cb_win_max_us = 0;
#endif
}

static void unlock_fp(void)
{
    if (s_bt_fp_lock) {
        xSemaphoreGive(s_bt_fp_lock);
    }
}

static inline size_t bt_buf_load_r(void)
{
    return __atomic_load_n(&s_bt_buf_r, __ATOMIC_ACQUIRE);
}

static inline size_t bt_buf_load_w(void)
{
    return __atomic_load_n(&s_bt_buf_w, __ATOMIC_ACQUIRE);
}

static inline void bt_buf_store_r(size_t r)
{
    __atomic_store_n(&s_bt_buf_r, r, __ATOMIC_RELEASE);
}

static inline void bt_buf_store_w(size_t w)
{
    __atomic_store_n(&s_bt_buf_w, w, __ATOMIC_RELEASE);
}

static inline size_t bt_buf_level_from_rw(size_t r, size_t w, size_t size)
{
    if (w >= r) return w - r;
    return size - (r - w);
}

static void bt_buf_reset(void)
{
    bt_buf_store_r(0);
    bt_buf_store_w(0);
}

static size_t bt_buf_level(void)
{
    size_t size = s_bt_buf_size;
    if (!s_bt_buf || size == 0) return 0;

    size_t r = bt_buf_load_r();
    size_t w = bt_buf_load_w();
    return bt_buf_level_from_rw(r, w, size);
}

static size_t bt_buf_space(void)
{
    size_t size = s_bt_buf_size;
    if (!s_bt_buf || size == 0) return 0;

    size_t r = bt_buf_load_r();
    size_t w = bt_buf_load_w();
    size_t level = bt_buf_level_from_rw(r, w, size);
    return (size > level) ? (size - level) : 0;
}

static size_t bt_buf_write(const uint8_t *src, size_t n)
{
    if (!src || n == 0 || !s_bt_buf || s_bt_buf_size == 0) return 0;

    size_t size = s_bt_buf_size;
    size_t r = bt_buf_load_r();
    size_t w = bt_buf_load_w();
    size_t level = bt_buf_level_from_rw(r, w, size);
    size_t space = (size > level) ? (size - level) : 0;
    if (n > space) n = space;
    if (n == 0) return 0;

    size_t first = size - w;
    if (first > n) first = n;
    memcpy(s_bt_buf + w, src, first);

    size_t second = n - first;
    if (second > 0) {
        memcpy(s_bt_buf, src + first, second);
    }

    w += n;
    if (w >= size) w -= size;
    bt_buf_store_w(w);
    return n;
}

static size_t bt_buf_read(uint8_t *dst, size_t n)
{
    if (!dst || n == 0 || !s_bt_buf || s_bt_buf_size == 0) return 0;

    size_t size = s_bt_buf_size;
    size_t r = bt_buf_load_r();
    size_t w = bt_buf_load_w();
    size_t level = bt_buf_level_from_rw(r, w, size);
    if (n > level) n = level;
    if (n == 0) return 0;

    size_t first = size - r;
    if (first > n) first = n;
    memcpy(dst, s_bt_buf + r, first);

    size_t second = n - first;
    if (second > 0) {
        memcpy(dst + first, s_bt_buf, second);
    }

    r += n;
    if (r >= size) r -= size;
    bt_buf_store_r(r);
    return n;
}

static size_t bt_buf_low_watermark(void)
{
    if (s_bt_buf_size == 0) return BT_STREAM_START_WATERMARK;
    size_t low = s_bt_buf_size / 4;
    if (low > BT_STREAM_LOW_WATERMARK_MAX) low = BT_STREAM_LOW_WATERMARK_MAX;
    if (low < BT_STREAM_START_WATERMARK) low = BT_STREAM_START_WATERMARK;
    return low;
}

static size_t bt_buf_high_watermark(void)
{
    if (s_bt_buf_size == 0) return BT_STREAM_START_WATERMARK;
    size_t high = bt_buf_low_watermark() + BT_STREAM_REFILL_WINDOW_BYTES;
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

static inline void bt_feeder_notify(void)
{
    TaskHandle_t t = s_bt_feeder_task;
    if (t) {
        xTaskNotifyGive(t);
    }
}

static inline void bt_feeder_wait_for_signal(void)
{
    // Use a pure notify-driven feeder wakeup: callback/start/stop/resume paths
    // are responsible for poking this task when more work is needed.
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static size_t bt_read_mono_to_stereo_chunk_locked(size_t want_out_bytes)
{
    if (!s_bt_feeder_chunk || !s_bt_feeder_mono_chunk ||
        !s_bt_fp || s_bt_src_bytes_left < sizeof(int16_t) || want_out_bytes < 4) return 0;

    size_t frame_cap = want_out_bytes / 4; // output stereo frames
    size_t want_in = frame_cap * sizeof(int16_t);
    if (want_in > BT_FEEDER_MONO_IN_CHUNK_BYTES) want_in = BT_FEEDER_MONO_IN_CHUNK_BYTES;
    // Keep SD SPI transactions short so other SPI users (OLED) get bus time.
    if (want_in > BT_FEEDER_MONO_READ_MAX_BYTES) want_in = BT_FEEDER_MONO_READ_MAX_BYTES;
    if (want_in > s_bt_src_bytes_left) want_in = s_bt_src_bytes_left;
    want_in &= ~(size_t)1;
    if (want_in == 0) return 0;

    size_t rd = fread(s_bt_feeder_mono_chunk, 1, want_in, s_bt_fp);
    rd &= ~(size_t)1;
    if (rd == 0) return 0;

    s_bt_src_bytes_left -= (uint32_t)rd;

    size_t frames = rd / sizeof(int16_t);
    const uint16_t *mono_u16 = (const uint16_t*)s_bt_feeder_mono_chunk;
    uint32_t *out_u32 = (uint32_t*)s_bt_feeder_chunk;
    int32_t gain = s_bt_vol_q15;

    if (gain == 0) {
        memset(out_u32, 0, frames * sizeof(uint32_t));
    } else if (gain == 32767) {
        for (size_t i = 0; i < frames; ++i) {
            uint32_t s = (uint32_t)mono_u16[i];
            out_u32[i] = s | (s << 16);
        }
    } else {
        for (size_t i = 0; i < frames; ++i) {
            int16_t s = (int16_t)mono_u16[i];
            int32_t v = ((int32_t)s * gain) >> 15;
            if (v > 32767) v = 32767;
            else if (v < -32768) v = -32768;
            s = (int16_t)v;
            uint32_t dup = (uint32_t)(uint16_t)s;
            out_u32[i] = dup | (dup << 16);
        }
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
    unsigned int yield_ctr = 0;

    while (!s_bt_feeder_stop) {
        s_bt_diag.feeder_loops++;
        if (s_bt_stream_paused) {
            s_bt_diag.feeder_wait_pause++;
            bt_feeder_wait_for_signal();
            bt_diag_log_periodic();
            continue;
        }

        FILE *fp_local = s_bt_fp;
        if (!fp_local) break;

        if (s_bt_src_bytes_left == 0) {
            s_bt_file_eof = true;
            break;
        }

        size_t level = bt_buf_level();
        bt_diag_note_level(level);
        if (level >= bt_buf_high_watermark()) {
            s_bt_diag.feeder_wait_high++;
            bt_feeder_wait_for_signal();
            bt_diag_log_periodic();
            continue;
        }

        size_t space = bt_buf_space();
        if (space < (BT_FEEDER_OUT_CHUNK_BYTES / 2)) {
            s_bt_diag.feeder_wait_space++;
            bt_feeder_wait_for_signal();
            bt_diag_log_periodic();
            continue;
        }

        size_t want_out = BT_FEEDER_OUT_CHUNK_BYTES;
        if (want_out > space) want_out = space;

        size_t produced = 0;
        if (lock_fp(pdMS_TO_TICKS(20))) {
            if (s_bt_fp) {
                produced = bt_read_mono_to_stereo_chunk_locked(want_out);
            }
            unlock_fp();
        } else {
            s_bt_diag.feeder_wait_lock++;
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            bt_diag_log_periodic();
            continue;
        }

        if (produced > 0) {
            (void)bt_buf_write(s_bt_feeder_chunk, produced);
            s_bt_diag.feeder_bytes += (uint32_t)produced;
            bt_diag_note_level(bt_buf_level());
            if (++yield_ctr >= BT_FEEDER_YIELD_EVERY) {
                yield_ctr = 0;
                s_bt_diag.feeder_yields++;
                vTaskDelay(1);
            }
            bt_diag_log_periodic();
            continue;
        }

        s_bt_file_eof = true;
        size_t level_now = bt_buf_level();
        if (s_bt_bytes_left > level_now) {
            s_bt_bytes_left = (uint32_t)level_now;
        }
        bt_diag_log_periodic();
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
    int64_t cb_start_us = esp_timer_get_time();
    if (!data || len <= 0) return 0;

    if (s_bt_stream_paused) {
        memset(data, 0, len);
        return len;
    }

    if (s_bt_bytes_left == 0 || (s_bt_file_eof && bt_buf_level() == 0)) {
        memset(data, 0, len);
        return len;
    }

    size_t to_read = (size_t)len;
    if (to_read > s_bt_bytes_left) to_read = s_bt_bytes_left;

    size_t rd = bt_buf_read(data, to_read);
    if (rd < (size_t)len) {
        memset(data + rd, 0, (size_t)len - rd);
    }

    if (s_bt_bytes_left >= rd) s_bt_bytes_left -= (uint32_t)rd;
    else s_bt_bytes_left = 0;

    if (s_bt_feeder_task && !s_bt_stream_paused) {
        if (rd < to_read || bt_buf_level() <= bt_buf_low_watermark()) {
            bt_feeder_notify();
        }
    }

    s_bt_diag.cb_calls++;
    s_bt_diag.cb_req_bytes += (uint32_t)to_read;
    s_bt_diag.cb_read_bytes += (uint32_t)rd;
    if (rd < to_read) s_bt_diag.cb_partial_reads++;
    if (rd < (size_t)len) {
        s_bt_diag.cb_underruns++;
        s_bt_diag.cb_zero_fill_bytes += (uint32_t)((size_t)len - rd);
    }
    bt_diag_note_level(bt_buf_level());

    uint32_t cb_us = (uint32_t)(esp_timer_get_time() - cb_start_us);
    s_bt_diag.cb_last_us = cb_us;
    if (cb_us > s_bt_diag.cb_max_us) s_bt_diag.cb_max_us = cb_us;
    if (cb_us > s_bt_diag.cb_win_max_us) s_bt_diag.cb_win_max_us = cb_us;

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
    if (num_channels != 1) {
        ESP_LOGW(TAG, "bt_audio_play_wav: mono-only, got ch=%u", num_channels);
        return ESP_ERR_NOT_SUPPORTED;
    }
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

    if (!bt_ensure_buffer()) {
        ESP_LOGE(TAG, "bt_audio_play_wav: stream buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }
    if (!bt_ensure_feeder_buffers(true)) {
        ESP_LOGE(TAG, "bt_audio_play_wav: feeder buffer alloc failed");
        return ESP_ERR_NO_MEM;
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

    uint64_t out_total = (uint64_t)data_size * 2ULL;
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
    bt_buf_reset();
    s_bt_file_eof = false;
    s_bt_feeder_stop = false;
    s_bt_stream_paused = false;
    bt_diag_reset();
    unlock_fp();

    if (!s_bt_feeder_task) {
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
            if (lock_fp(pdMS_TO_TICKS(100))) {
                if (s_bt_fp == fp) {
                    s_bt_fp = NULL;
                    s_bt_src_bytes_left = 0;
                    s_bt_bytes_left = 0;
                    s_bt_file_eof = false;
                    s_bt_stream_paused = false;
                    bt_buf_reset();
                }
                unlock_fp();
            }
            ESP_LOGE(TAG,
                     "BT feeder task create failed "
                     "(internal free=%u largest=%u)",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_bt_feeder_task) {
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
