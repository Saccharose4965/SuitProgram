#include "bt_audio.h"
#include "audio.h"
#include "audio_player.h"
#include "bt_audio_app.h"
#include "input.h"
#include "oled.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/idf_additions.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "system_state.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "bta/bta_api.h"
#include "bta_dm_int.h"

// The public ESP-IDF API has no helper to toggle sniff for A2DP source.
// Use the Bluedroid primitives directly to force sniff off (avoids the
// unsniff deadlock seen on some sinks).
#ifndef HCI_ENABLE_MASTER_SLAVE_SWITCH
#define HCI_ENABLE_MASTER_SLAVE_SWITCH 0x0001
#endif
#ifndef HCI_ENABLE_HOLD_MODE
#define HCI_ENABLE_HOLD_MODE 0x0002
#endif
#ifndef HCI_ENABLE_SNIFF_MODE
#define HCI_ENABLE_SNIFF_MODE 0x0004
#endif
#ifndef HCI_ENABLE_PARK_MODE
#define HCI_ENABLE_PARK_MODE 0x0008
#endif
extern void BTM_SetDefaultLinkPolicy(uint16_t settings);
extern uint8_t BTM_SetLinkPolicy(uint8_t bd_addr[6], uint16_t *settings);

static const char *TAG = "bt_audio";

// ===== AVRCP hooks (weak, overridden by higher-level code like audio_player) =====
void __attribute__((weak)) bt_audio_on_play(void)   {}
void __attribute__((weak)) bt_audio_on_pause(void)  {}
void __attribute__((weak)) bt_audio_on_next(void)   {}
void __attribute__((weak)) bt_audio_on_prev(void)   {}

// ============================ State ============================
static bt_audio_state_t s_state         = BT_AUDIO_STATE_IDLE;
static bool             s_stack_ready   = false;
static bool             s_streaming     = false;
static bool             s_user_streaming = false;
static bool             s_media_cmd_pending = false;
static esp_a2d_media_ctrl_t s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
static esp_bd_addr_t    s_peer_bda      = {0};
static char             s_peer_name[32] = {0};
static int              s_peer_sample_rate = 44100;
static bt_audio_device_t s_devices[8];
static int               s_device_count = 0;
static bool              s_name_req_active = false;
static esp_bd_addr_t     s_name_req_bda = {0};
static bool              s_scanning = false;
static bt_audio_disconnect_cb_t s_disconnect_cb = NULL;
static bool              s_pending_connect = false;
static esp_bd_addr_t     s_pending_bda = {0};
static int               s_pending_attempts = 0;
static TimerHandle_t     s_pending_timer = NULL;
static bool bda_is_zero(const esp_bd_addr_t bda);
static esp_err_t start_connect(esp_bd_addr_t bda);
static esp_err_t start_discovery(void);
static void clear_pending_connect(void)
{
    s_pending_connect = false;
    memset(s_pending_bda, 0, sizeof(s_pending_bda));
    s_pending_attempts = 0;
    if (s_pending_timer) {
        xTimerStop(s_pending_timer, 0);
    }
}

static void pending_connect_timer_cb(TimerHandle_t tmr)
{
    (void)tmr;
    if (!s_pending_connect || bda_is_zero(s_pending_bda)) {
        clear_pending_connect();
        return;
    }
    if (s_state == BT_AUDIO_STATE_CONNECTING ||
        s_state == BT_AUDIO_STATE_CONNECTED ||
        s_state == BT_AUDIO_STATE_STREAMING) {
        return; // already in progress or satisfied
    }
    if (s_pending_attempts >= 3) {
        ESP_LOGW(TAG, "Pending connect retries exhausted");
        clear_pending_connect();
        (void)start_discovery();
        return;
    }
    s_pending_attempts++;
    ESP_LOGI(TAG, "Retry pending connect (%d/3)", s_pending_attempts);
    (void)start_connect(s_pending_bda);
}

static void schedule_pending_connect_retry(uint32_t delay_ms)
{
    if (!s_pending_timer) {
        s_pending_timer = xTimerCreate("bt_pend",
                                       pdMS_TO_TICKS(delay_ms),
                                       pdFALSE,
                                       NULL,
                                       pending_connect_timer_cb);
    }
    if (s_pending_timer) {
        xTimerStop(s_pending_timer, 0);
        xTimerChangePeriod(s_pending_timer, pdMS_TO_TICKS(delay_ms), 0);
        xTimerStart(s_pending_timer, 0);
    }
}

typedef struct {
    bt_audio_device_t devs[8];
    int count;
    int sel;
    int start;
    bool peer_extra;
    int  peer_index;
} bt_ui_state_t;
static bt_ui_state_t s_bt_ui = {0};
static bt_audio_state_t s_bt_prev_state = BT_AUDIO_STATE_DISABLED;

// Forward declarations for helper routines used before their definitions.
static void add_device(const esp_bd_addr_t bda, const char *name);
static void maybe_request_name(esp_bd_addr_t bda);
static void reset_devices(void);
static void avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                       esp_avrc_tg_cb_param_t *param);

// ============================ File-backed source ============================
static FILE    *s_bt_fp             = NULL;
static uint32_t s_bt_bytes_left     = 0;
static uint16_t s_bt_channels       = 2;
static uint16_t s_bt_bits_per_sample= 16;
static uint32_t s_bt_sample_rate    = 44100;
static SemaphoreHandle_t s_bt_fp_lock = NULL; // protects FILE access across callback/control paths

#define BT_STREAM_BUF_BYTES_MAX   (256 * 1024)
#define BT_STREAM_BUF_BYTES_MIN   (64 * 1024)
#define BT_STREAM_START_WATERMARK (12 * 1024)
#define BT_FEEDER_CHUNK_BYTES     2048
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
static TaskHandle_t  s_bt_feeder_task = NULL;
static portMUX_TYPE  s_bt_buf_mux     = portMUX_INITIALIZER_UNLOCKED;

// Volume control: 0–100 % mapped to Q15 gain (0–32767)
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
        // rounded linear mapping 0–100% → 0–32767
        s_bt_vol_q15 = (int32_t)(v * 32767 + 50) / 100;
    }
}

static void set_state(bt_audio_state_t st){
    s_state = st;
}

static void set_connection(system_connection_t conn){
    system_state_set_connection(conn);
}

void bt_audio_set_disconnect_cb(bt_audio_disconnect_cb_t cb)
{
    s_disconnect_cb = cb;
}

void bt_audio_invoke_disconnect_cb(void)
{
    if (s_disconnect_cb) {
        s_disconnect_cb();
    }
}

static void set_pending_connect(const esp_bd_addr_t bda)
{
    memcpy(s_pending_bda, bda, sizeof(s_pending_bda));
    s_pending_connect = !bda_is_zero(s_pending_bda);
    s_pending_attempts = 0;
}

// ============================ Helpers ============================
static esp_err_t ensure_nvs(void){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_LOGW(TAG, "NVS full; erasing");
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase nvs");
        err = nvs_flash_init();
    }
    return err;
}

static bool cod_is_audio(uint32_t cod){
    uint32_t major_service = (cod & 0x00FF0000) >> 16;
    uint32_t major_device  = (cod & 0x00001F00) >> 8;
    return (major_service & (ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO)) ||
           (major_device == ESP_BT_COD_MAJOR_DEV_AV);
}

static bool lock_fp(TickType_t wait_ticks){
    if (!s_bt_fp_lock) {
        s_bt_fp_lock = xSemaphoreCreateMutex();
    }
    if (!s_bt_fp_lock) return false;
    return xSemaphoreTake(s_bt_fp_lock, wait_ticks) == pdTRUE;
}

static void unlock_fp(void){
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

// Pull bytes for the A2DP callback:
// from RAM ring buffer (normal buffered path).
static size_t bt_stream_pull(uint8_t *dst, size_t n)
{
    if (!dst || n == 0) return 0;
    return bt_buf_read(dst, n);
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
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        size_t want = BT_FEEDER_CHUNK_BYTES;
        if (want > space) want = space;
        if (want > s_bt_bytes_left) want = s_bt_bytes_left;
        if (want == 0) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        size_t rd = 0;
        if (lock_fp(pdMS_TO_TICKS(20))) {
            if (s_bt_fp) {
                rd = fread(s_bt_feeder_chunk, 1, want, s_bt_fp);
            }
            unlock_fp();
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

static void close_bt_file_locked(void){
    if (s_bt_fp) {
        fclose(s_bt_fp);
        s_bt_fp = NULL;
    }
    s_bt_bytes_left = 0;
    s_bt_file_eof = false;
    bt_buf_reset();
}

static bool close_bt_file(TickType_t wait_ticks){
    (void)bt_stop_feeder(wait_ticks);
    if (lock_fp(wait_ticks)) {
        close_bt_file_locked();
        unlock_fp();
        return true;
    } else {
        s_bt_bytes_left = 0;
        // Mark EOF so callback-side close path can finish once feeder exits.
        s_bt_file_eof = true;
        bt_buf_reset();
        ESP_LOGW(TAG, "Failed to lock BT file for close");
        return false;
    }
}

// Device list helpers
static int find_device(const esp_bd_addr_t bda){
    for (int i = 0; i < s_device_count; ++i){
        if (memcmp(s_devices[i].bda, bda, sizeof(esp_bd_addr_t)) == 0) return i;
    }
    return -1;
}

static void add_device(const esp_bd_addr_t bda, const char *name){
    int idx = find_device(bda);
    if (idx < 0 && s_device_count < (int)(sizeof(s_devices)/sizeof(s_devices[0]))){
        idx = s_device_count++;
    }
    if (idx >= 0){
        memcpy(s_devices[idx].bda, bda, sizeof(esp_bd_addr_t));
        if (name && name[0]) {
            strncpy(s_devices[idx].name, name, sizeof(s_devices[idx].name) - 1);
            s_devices[idx].name[sizeof(s_devices[idx].name) - 1] = '\0';
        }
    }
}

static void reset_devices(void){
    s_device_count = 0;
    memset(s_devices, 0, sizeof(s_devices));
}

static bool bda_is_zero(const esp_bd_addr_t bda)
{
    if (!bda) return true;
    for (int i = 0; i < 6; ++i) {
        if (bda[i] != 0) return false;
    }
    return true;
}

static void disconnect_all_peers_except(const esp_bd_addr_t keep, bool use_keep)
{
    if (!s_stack_ready) {
        return;
    }

    for (int i = 0; i < s_device_count; ++i) {
        if (bda_is_zero(s_devices[i].bda)) {
            continue;
        }
        if (use_keep &&
            memcmp(keep, s_devices[i].bda, sizeof(esp_bd_addr_t)) == 0) {
            continue;
        }

        esp_err_t e = esp_a2d_source_disconnect(s_devices[i].bda);
        if (e != ESP_OK &&
            e != ESP_ERR_INVALID_STATE &&
            e != ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "disconnect dev %d failed: %s",
                     i, esp_err_to_name(e));
        }
    }

    // Also try the current peer directly if it's not the "keep" one
    if (!bda_is_zero(s_peer_bda) && (!use_keep ||
        memcmp(keep, s_peer_bda, sizeof(s_peer_bda)) != 0)) {
        esp_err_t e = esp_a2d_source_disconnect(s_peer_bda);
        if (e != ESP_OK &&
            e != ESP_ERR_INVALID_STATE &&
            e != ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "disconnect current peer failed: %s",
                     esp_err_to_name(e));
        }
    }
}

static void maybe_request_name(esp_bd_addr_t bda){
    if (s_name_req_active) return;
    int idx = find_device(bda);
    if (idx >= 0 && s_devices[idx].name[0]) return;
    esp_err_t e = esp_bt_gap_read_remote_name(bda);
    if (e == ESP_OK){
        s_name_req_active = true;
        memcpy(s_name_req_bda, bda, sizeof(s_name_req_bda));
    }
}

// ============================ A2DP callbacks ============================
static int32_t a2dp_data_cb(uint8_t *data, int32_t len){
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

    // Apply volume gain (Q15) to the PCM buffer if not unity
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

static void a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    switch (event){
        case ESP_A2D_CONNECTION_STATE_EVT:{
            esp_a2d_connection_state_t st = param->conn_stat.state;
            ESP_LOGI(TAG, "A2DP conn state %d", st);
            if (st == ESP_A2D_CONNECTION_STATE_CONNECTED){
                memcpy(s_peer_bda, param->conn_stat.remote_bda, sizeof(s_peer_bda));
                clear_pending_connect(); // satisfied
                uint16_t nosniff_policy = HCI_ENABLE_MASTER_SLAVE_SWITCH | HCI_ENABLE_HOLD_MODE | HCI_ENABLE_PARK_MODE;
                (void)BTM_SetLinkPolicy(s_peer_bda, &nosniff_policy);
                ESP_LOGI(TAG, "Applied no-sniff link policy to peer");
                add_device(s_peer_bda, NULL);
                maybe_request_name(s_peer_bda);
                set_state(BT_AUDIO_STATE_CONNECTED);
                s_user_streaming = false;
                s_streaming = false;
                set_connection(SYS_CONN_CONNECTED);
            } else if (st == ESP_A2D_CONNECTION_STATE_CONNECTING){
                set_state(BT_AUDIO_STATE_CONNECTING);
                set_connection(SYS_CONN_CONNECTING);
            } else if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTED ||
                       st == ESP_A2D_CONNECTION_STATE_DISCONNECTING){
                s_user_streaming = false;
                s_streaming = false;
                s_media_cmd_pending = false;
                s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
                set_state(BT_AUDIO_STATE_IDLE);
                set_connection(SYS_CONN_DISCONNECTED);
                memset(s_peer_bda, 0, sizeof(s_peer_bda));
                s_peer_name[0] = '\0';
                if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTING) {
                    break; // wait for DISCONNECTED to handle pending/restart
                }
                bool tried_pending = false;
                if (s_pending_connect && !bda_is_zero(s_pending_bda)) {
                    s_pending_attempts++;
                    if (s_pending_attempts > 3) {
                        ESP_LOGW(TAG, "Pending connect failed %d times, giving up", s_pending_attempts - 1);
                        clear_pending_connect();
                    } else {
                        ESP_LOGI(TAG, "Attempt pending connect after disconnect (%d/3)", s_pending_attempts);
                        tried_pending = (start_connect(s_pending_bda) == ESP_OK);
                        if (!tried_pending) {
                            schedule_pending_connect_retry(400);
                        }
                    }
                }
                if (!tried_pending) {
                    (void)start_discovery();
                }
            }

            break;
        }
        case ESP_A2D_AUDIO_STATE_EVT:{
            esp_a2d_audio_state_t st = param->audio_stat.state;
            ESP_LOGI(TAG, "A2DP audio state %d", st);
            if (st == ESP_A2D_AUDIO_STATE_STARTED){
                s_streaming = true;
                set_state(BT_AUDIO_STATE_STREAMING);
            } else {
                s_streaming = false;
                if (s_state == BT_AUDIO_STATE_STREAMING){
                    set_state(BT_AUDIO_STATE_CONNECTED);
                }
            }
            break;
        }
        case ESP_A2D_AUDIO_CFG_EVT:{
            uint8_t sf = param->audio_cfg.mcc.cie.sbc_info.samp_freq;
            int sample_rate = 16000;
            if (sf & ESP_A2D_SBC_CIE_SF_48K) sample_rate = 48000;
            else if (sf & ESP_A2D_SBC_CIE_SF_44K) sample_rate = 44100;
            else if (sf & ESP_A2D_SBC_CIE_SF_32K) sample_rate = 32000;
            s_peer_sample_rate = sample_rate;
            ESP_LOGI(TAG, "A2DP cfg: %d Hz", sample_rate);
            break;
        }
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:{
            esp_a2d_media_ctrl_t cmd = param->media_ctrl_stat.cmd;
            esp_a2d_media_ctrl_ack_t status = param->media_ctrl_stat.status;
            s_media_cmd_pending = false;
            s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
            if (status != ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                ESP_LOGW(TAG, "A2DP media ctrl ack %d for cmd %d", status, cmd);
                if (cmd == ESP_A2D_MEDIA_CTRL_START) {
                    s_streaming = false;
                    if (s_state == BT_AUDIO_STATE_STREAMING) {
                        set_state(BT_AUDIO_STATE_CONNECTED);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}

// ============================ GAP callback ============================
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event){
        case ESP_BT_GAP_DISC_RES_EVT:{
            bool audio_dev = false;
            char name[32] = {0};
            uint32_t cod = 0;
            for (int i = 0; i < param->disc_res.num_prop; ++i){
                esp_bt_gap_dev_prop_t *p = param->disc_res.prop + i;
                if (p->type == ESP_BT_GAP_DEV_PROP_COD && p->val){
                    cod = *(uint32_t*)p->val;
                    audio_dev = cod_is_audio(cod);
                } else if (p->type == ESP_BT_GAP_DEV_PROP_BDNAME && p->val){
                    size_t n = p->len < sizeof(name) - 1 ? p->len : sizeof(name) - 1;
                    memcpy(name, p->val, n);
                    name[n] = 0;
                }
            }
            if (audio_dev){
                add_device(param->disc_res.bda, name[0] ? name : NULL);
                ESP_LOGI(TAG, "Found audio dev %s", name[0] ? name : "(unnamed)");
                if (!name[0]) {
                    maybe_request_name(param->disc_res.bda);
                }
                if (memcmp(param->disc_res.bda, s_peer_bda, sizeof(s_peer_bda)) == 0 && name[0]) {
                    strncpy(s_peer_name, name, sizeof(s_peer_name) - 1);
                    s_peer_name[sizeof(s_peer_name) - 1] = '\0';
                }
            }
            break;
        }
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:{
            s_name_req_active = false;
            if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS){
                add_device(param->read_rmt_name.bda, (const char*)param->read_rmt_name.rmt_name);
                ESP_LOGI(TAG, "Name resolved: %s", param->read_rmt_name.rmt_name);
                if (memcmp(param->read_rmt_name.bda, s_peer_bda, sizeof(s_peer_bda)) == 0) {
                    strncpy(s_peer_name, (const char*)param->read_rmt_name.rmt_name, sizeof(s_peer_name) - 1);
                    s_peer_name[sizeof(s_peer_name) - 1] = '\0';
                }
            }
            break;
        }
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:{
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                s_scanning = true;
                // Keep state/UI synchronized even if another callback briefly
                // forced us back to IDLE while inquiry is still active.
                set_state(BT_AUDIO_STATE_DISCOVERING);
                set_connection(SYS_CONN_CONNECTING);
            } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED){
                s_scanning = false;
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

                if (s_state == BT_AUDIO_STATE_DISCOVERING) {
                    set_state(BT_AUDIO_STATE_IDLE);
                    set_connection(SYS_CONN_DISCONNECTED);
                }
            }
            break;
        }
        default:
            break;
    }
}

// ============================ Start/Connect ============================
static esp_err_t start_connect(esp_bd_addr_t bda){
    if (!bda) return ESP_ERR_INVALID_ARG;
    // Stop any active discovery before initiating a link; otherwise the
    // controller can get congested and fail the new connection.
    (void)esp_bt_gap_cancel_discovery();
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    s_scanning = false;
    set_state(BT_AUDIO_STATE_CONNECTING);
    set_connection(SYS_CONN_CONNECTING);
    memcpy(s_peer_bda, bda, sizeof(s_peer_bda));
    esp_err_t err = esp_a2d_source_connect(bda);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "connect err %s", esp_err_to_name(err));
        set_state(BT_AUDIO_STATE_FAILED);
        set_connection(SYS_CONN_DISCONNECTED);
        if (s_pending_connect) {
            schedule_pending_connect_retry(400);
        } else {
            (void)start_discovery(); // recover by restarting inquiry
        }
    }
    return err;
}

static esp_err_t start_discovery(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    if (s_state == BT_AUDIO_STATE_CONNECTED ||
        s_state == BT_AUDIO_STATE_STREAMING ||
        s_state == BT_AUDIO_STATE_CONNECTING) {
        ESP_LOGI(TAG, "start_discovery: busy (state=%s), skipping", bt_audio_state_name(s_state));
        return ESP_ERR_INVALID_STATE;
    }
    if (s_scanning) {
        ESP_LOGI(TAG, "start_discovery: already scanning");
        // Re-assert discovery state so UI/HUD do not remain in IDLE while
        // inquiry is active.
        set_state(BT_AUDIO_STATE_DISCOVERING);
        set_connection(SYS_CONN_CONNECTING);
        return ESP_OK;
    }
    s_scanning = true;
    set_state(BT_AUDIO_STATE_DISCOVERING);
    set_connection(SYS_CONN_CONNECTING);
    (void)esp_bt_gap_cancel_discovery();
    // Keep inquiry short to reduce controller airtime (less impact on streaming).
    esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 5, 0);
    if (err != ESP_OK){
        ESP_LOGE(TAG, "discovery start err %s", esp_err_to_name(err));
        s_scanning = false;
        set_state(BT_AUDIO_STATE_FAILED);
        set_connection(SYS_CONN_DISCONNECTED);
    }
    return err;
}

// ============================ Public file-backed playback API ============================
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
    if (s_media_cmd_pending) return ESP_ERR_INVALID_STATE;

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
        ESP_LOGW(TAG, "bt_audio_play_wav: stream buffer alloc failed");
        return ESP_ERR_NO_MEM;
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
    s_bt_bits_per_sample = bits_per_sample;
    s_bt_sample_rate     = sample_rate;
    bt_buf_reset();
    s_bt_file_eof = false;
    s_bt_feeder_stop = false;
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
            if (lock_fp(pdMS_TO_TICKS(200))) {
                if (s_bt_fp == fp) {
                    s_bt_fp = NULL;
                    s_bt_bytes_left = 0;
                    s_bt_file_eof = false;
                    bt_buf_reset();
                }
                unlock_fp();
            }
            ESP_LOGW(TAG, "BT feeder task create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    TickType_t start_tick = xTaskGetTickCount();
    while (!s_bt_file_eof && bt_buf_level() < BT_STREAM_START_WATERMARK) {
        if ((xTaskGetTickCount() - start_tick) > pdMS_TO_TICKS(250)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
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

// ============================ Public API ============================
esp_err_t bt_audio_start(bool force_rescan){
    ESP_RETURN_ON_ERROR(ensure_nvs(), TAG, "nvs");

    if (!s_stack_ready){
        esp_err_t rel = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
        if (rel != ESP_OK && rel != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "release BLE %s", esp_err_to_name(rel));
            return rel;
        }
        esp_bt_controller_status_t ctrl_st = esp_bt_controller_get_status();
        if (ctrl_st == ESP_BT_CONTROLLER_STATUS_IDLE) {
            esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
            esp_err_t e = esp_bt_controller_init(&bt_cfg);
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "ctrl init %s", esp_err_to_name(e));
                return e;
            }
        }
        if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
            esp_bt_mode_t mode = ESP_BT_MODE_CLASSIC_BT;
            esp_err_t e = esp_bt_controller_enable(mode);
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "ctrl enable %s", esp_err_to_name(e));
                return e;
            }
        }

        esp_bluedroid_status_t bd_st = esp_bluedroid_get_status();
        if (bd_st == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
            ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "bd init");
            bd_st = esp_bluedroid_get_status();
        }
        if (bd_st != ESP_BLUEDROID_STATUS_ENABLED) {
            esp_err_t e = esp_bluedroid_enable();
            if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "bd enable %s", esp_err_to_name(e));
                return e;
            }
        }
        uint16_t nosniff_policy = HCI_ENABLE_MASTER_SLAVE_SWITCH | HCI_ENABLE_HOLD_MODE | HCI_ENABLE_PARK_MODE;
        BTM_SetDefaultLinkPolicy(nosniff_policy);
        ESP_LOGI(TAG, "Sniff mode disabled (policy=0x%04x)", nosniff_policy);
        
        ESP_RETURN_ON_ERROR(esp_avrc_tg_init(), TAG, "avrc tg init");
        ESP_RETURN_ON_ERROR(esp_avrc_tg_register_callback(avrc_tg_cb), TAG, "avrc tg cb");

        ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(gap_cb), TAG, "gap cb");
        ESP_RETURN_ON_ERROR(esp_a2d_register_callback(a2dp_cb), TAG, "a2dp cb");
        esp_err_t a2d_init = esp_a2d_source_init();
        if (a2d_init != ESP_OK && a2d_init != ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(a2d_init, TAG, "a2dp init");
        }
        esp_err_t data_cb_err = esp_a2d_source_register_data_callback(a2dp_data_cb);
        if (data_cb_err != ESP_OK && data_cb_err != ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(data_cb_err, TAG, "data cb");
        }
        ESP_RETURN_ON_ERROR(esp_bt_gap_set_device_name("Suit-BT"), TAG, "set name");
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_bt_pin_code_t pin_code = { '0', '0', '0', '0' };
        esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
        s_stack_ready = true;
    }

    if (force_rescan){
        reset_devices();
        memset(s_peer_bda, 0, sizeof(s_peer_bda));
        s_peer_name[0] = '\0';
        return start_discovery();
    }
    return ESP_OK;
}

esp_err_t bt_audio_disconnect(void){
    if (!s_stack_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "bt_audio_disconnect: disconnect current peer");

    // Stop playback if needed
    if (s_streaming || s_user_streaming) {
        esp_err_t m = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        if (m != ESP_OK && m != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "media suspend before disconnect failed: %s", esp_err_to_name(m));
        }
        s_streaming = false;
        s_user_streaming = false;
    }

    bool have_peer = false;
    for (int i = 0; i < 6; ++i) {
        if (s_peer_bda[i] != 0) {
            have_peer = true;
            break;
        }
    }

    if (have_peer) {
        esp_err_t e = esp_a2d_source_disconnect(s_peer_bda);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE && e != ESP_ERR_INVALID_ARG) {
            ESP_LOGW(TAG, "a2dp disconnect err: %s", esp_err_to_name(e));
        }
        // Let a2dp_cb() handle state when disconnect completes
        return e;
    }

    // No active link
    set_state(BT_AUDIO_STATE_IDLE);
    set_connection(SYS_CONN_DISCONNECTED);
    memset(s_peer_bda, 0, sizeof(s_peer_bda));
    s_peer_name[0] = '\0';
    return ESP_OK;
}

static void avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                       esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        // NOTE: in this ESP-IDF version the member is psth_cmd, not psth
        uint8_t key_code  = param->psth_cmd.key_code;
        uint8_t key_state = param->psth_cmd.key_state;

        ESP_LOGI(TAG, "AVRCP passthrough: code=%d state=%d", key_code, key_state);

        // Only act on key press, ignore key release/repeat
        if (key_state != ESP_AVRC_PT_CMD_STATE_PRESSED) {
            break;
        }

        switch (key_code) {
        case ESP_AVRC_PT_CMD_PLAY:
        case ESP_AVRC_PT_CMD_PAUSE:
            ESP_LOGI(TAG, "AVRCP: toggle play/pause");
            (void)audio_player_toggle_play_pause();
            break;

        case ESP_AVRC_PT_CMD_FORWARD:
            ESP_LOGI(TAG, "AVRCP: next track (unimplemented)");
            break;

        case ESP_AVRC_PT_CMD_BACKWARD:
            ESP_LOGI(TAG, "AVRCP: prev track (unimplemented)");
            break;

        case ESP_AVRC_PT_CMD_VOL_UP:
            ESP_LOGI(TAG, "AVRCP: vol up");
            bt_audio_volume_up();
            break;

        case ESP_AVRC_PT_CMD_VOL_DOWN:
            ESP_LOGI(TAG, "AVRCP: vol down");
            bt_audio_volume_down();
            break;

        default:
            ESP_LOGI(TAG, "AVRCP: unhandled key_code=%d", key_code);
            break;
        }
        break;
    }

    default:
        break;
    }
}

esp_err_t bt_audio_rescan(void){
    reset_devices();
    memset(s_peer_bda, 0, sizeof(s_peer_bda));
    s_peer_name[0] = '\0';
    s_user_streaming = false;
    return bt_audio_start(true);
}

esp_err_t bt_audio_scan(void){
    // Avoid scans while connected/streaming to prevent L2CAP congestion and audio glitches.
    if (s_state == BT_AUDIO_STATE_CONNECTED ||
        s_state == BT_AUDIO_STATE_STREAMING ||
        s_state == BT_AUDIO_STATE_CONNECTING) {
        ESP_LOGI(TAG, "skip scan: busy (state=%s)", bt_audio_state_name(s_state));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(bt_audio_start(false), TAG, "start");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    return start_discovery();
}

esp_err_t bt_audio_connect_index(int idx){
    if (idx < 0 || idx >= s_device_count) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_bd_addr_t target;
    memcpy(target, s_devices[idx].bda, sizeof(target));

    bool same_as_current =
        (memcmp(target, s_peer_bda, sizeof(target)) == 0) &&
        (s_state == BT_AUDIO_STATE_CONNECTED ||
         s_state == BT_AUDIO_STATE_STREAMING ||
         s_state == BT_AUDIO_STATE_CONNECTING);

    if (same_as_current) {
        ESP_LOGI(TAG, "Selected current device; disconnecting current peer");
        return bt_audio_disconnect();
    }

    bool busy = (s_state == BT_AUDIO_STATE_CONNECTED ||
                 s_state == BT_AUDIO_STATE_STREAMING ||
                 s_state == BT_AUDIO_STATE_CONNECTING);

    if (busy) {
        ESP_LOGI(TAG, "Switching peers: queue connect after disconnect");
        set_pending_connect(target);
        // Stop current stream before tearing down link to avoid congestion.
        (void)bt_audio_stop_stream();
        return bt_audio_disconnect();
    }

    ESP_LOGI(TAG, "Disconnecting other peers before connecting new one");
    disconnect_all_peers_except(target, true);

    s_user_streaming = false;
    s_streaming      = false;
    set_state(BT_AUDIO_STATE_IDLE);
    set_connection(SYS_CONN_DISCONNECTED);

    memcpy(s_peer_bda, target, sizeof(s_peer_bda));
    if (s_devices[idx].name[0]) {
        strncpy(s_peer_name, s_devices[idx].name, sizeof(s_peer_name) - 1);
        s_peer_name[sizeof(s_peer_name) - 1] = '\0';
    } else {
        s_peer_name[0] = '\0';
    }

    ESP_LOGI(TAG, "Connecting to device idx=%d", idx);
    return start_connect(target);
}

int bt_audio_get_devices(bt_audio_device_t *out, int max_out){
    if (!out || max_out <= 0) return 0;
    int n = s_device_count;
    if (n > max_out) n = max_out;
    memcpy(out, s_devices, n * sizeof(bt_audio_device_t));
    return n;
}

esp_err_t bt_audio_start_stream(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    if (s_state != BT_AUDIO_STATE_CONNECTED && s_state != BT_AUDIO_STATE_STREAMING){
        return ESP_ERR_INVALID_STATE;
    }
    s_user_streaming = true;
    if (s_streaming) {
        set_state(BT_AUDIO_STATE_STREAMING);
        return ESP_OK;
    }
    if (s_media_cmd_pending) {
        ESP_LOGW(TAG, "media ctrl pending (%d), start skipped", s_media_cmd_inflight);
        return ESP_ERR_INVALID_STATE;
    }
    s_media_cmd_pending = true;
    s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_START;
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
    if (err == ESP_OK){
        s_streaming = true;
        set_state(BT_AUDIO_STATE_STREAMING);
    } else {
        s_media_cmd_pending = false;
        s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
    }
    return err;
}

esp_err_t bt_audio_stop_stream(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    // Keep stop path responsive for shell/input context; feeder exits
    // asynchronously if SD read is momentarily blocked.
    (void)close_bt_file(pdMS_TO_TICKS(100));
    if (s_state != BT_AUDIO_STATE_CONNECTED && s_state != BT_AUDIO_STATE_STREAMING){
        return ESP_OK;
    }
    s_user_streaming = false;
    if (!s_streaming) {
        set_state(BT_AUDIO_STATE_CONNECTED);
        return ESP_OK;
    }
    if (s_media_cmd_pending) {
        ESP_LOGW(TAG, "media ctrl pending (%d), suspend skipped", s_media_cmd_inflight);
        return ESP_ERR_INVALID_STATE;
    }
    s_media_cmd_pending = true;
    s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_SUSPEND;
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    if (err != ESP_OK) {
        s_media_cmd_pending = false;
        s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
        return err;
    }
    s_streaming = false;
    set_state(BT_AUDIO_STATE_CONNECTED);
    return err;
}

esp_err_t bt_audio_pause_stream(void){
    if (!s_stack_ready) return ESP_ERR_INVALID_STATE;
    if (s_state != BT_AUDIO_STATE_CONNECTED && s_state != BT_AUDIO_STATE_STREAMING){
        return ESP_OK;
    }
    s_user_streaming = false;
    if (!s_streaming) {
        set_state(BT_AUDIO_STATE_CONNECTED);
        return ESP_OK;
    }
    if (s_media_cmd_pending) {
        ESP_LOGW(TAG, "media ctrl pending (%d), pause skipped", s_media_cmd_inflight);
        return ESP_ERR_INVALID_STATE;
    }
    s_media_cmd_pending = true;
    s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_SUSPEND;
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    if (err != ESP_OK) {
        s_media_cmd_pending = false;
        s_media_cmd_inflight = ESP_A2D_MEDIA_CTRL_NONE;
        return err;
    }
    s_streaming = false;
    set_state(BT_AUDIO_STATE_CONNECTED);
    return err;
}

int bt_audio_get_peer_sample_rate(void){
    return s_peer_sample_rate;
}

esp_err_t bt_audio_feed_pcm(const int16_t *stereo, size_t frames){
    (void)stereo; (void)frames;
    return ESP_ERR_NOT_SUPPORTED; // legacy API not used in file-backed mode
}

void bt_audio_get_status(bt_audio_status_t *out){
    if (!out) return;
    out->state = s_state;
    bool have_peer = false;
    for (int i = 0; i < 6; ++i) {
        if (s_peer_bda[i] != 0) { have_peer = true; break; }
    }
    out->has_peer = have_peer || (s_device_count > 0);
    memcpy(out->peer_bda, s_peer_bda, sizeof(out->peer_bda));
    strncpy(out->peer_name, s_peer_name, sizeof(out->peer_name) - 1);
    out->peer_name[sizeof(out->peer_name) - 1] = '\0';
    out->streaming = s_streaming;
    size_t bytes_per_frame = (s_bt_channels == 2) ? 4 : 2;
    out->queued_frames = bytes_per_frame ? (s_bt_bytes_left / bytes_per_frame) : 0;
}

bool bt_audio_can_stream(void){
    return s_stack_ready &&
           (s_state == BT_AUDIO_STATE_CONNECTED || s_state == BT_AUDIO_STATE_STREAMING);
}

bool bt_audio_media_cmd_pending(void)
{
    return s_media_cmd_pending;
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
    // step size: 5%, saturating at 100
    bt_audio_volume_set_percent(s_bt_volume_percent + 5);
}

void bt_audio_volume_down(void)
{
    // step size: 5%, saturating at 0
    bt_audio_volume_set_percent(s_bt_volume_percent - 5);
}

const char *bt_audio_state_name(bt_audio_state_t s){
    switch (s){
        case BT_AUDIO_STATE_DISABLED:     return "disabled";
        case BT_AUDIO_STATE_IDLE:         return "idle";
        case BT_AUDIO_STATE_STARTING:     return "starting";
        case BT_AUDIO_STATE_DISCOVERING:  return "scan";
        case BT_AUDIO_STATE_CONNECTING:   return "connecting";
        case BT_AUDIO_STATE_CONNECTED:    return "connected";
        case BT_AUDIO_STATE_STREAMING:    return "streaming";
        case BT_AUDIO_STATE_FAILED:       return "failed";
        default:                          return "?";
    }
}

// ============================ Shell UI ============================
static const int k_bt_list_max_vis = 4;

static int bt_find_dev_idx(const bt_audio_device_t *list, int count, const uint8_t bda[6]){
    for (int i = 0; i < count; ++i){
        if (memcmp(list[i].bda, bda, 6) == 0) return i;
    }
    return -1;
}

static void bt_update_viewport(void)
{
    const int max_vis = k_bt_list_max_vis;
    if (s_bt_ui.sel < 0) s_bt_ui.sel = 0;
    if (s_bt_ui.sel >= s_bt_ui.count) {
        s_bt_ui.sel = s_bt_ui.count ? s_bt_ui.count - 1 : 0;
    }

    int max_start = (s_bt_ui.count > max_vis) ? (s_bt_ui.count - max_vis) : 0;
    if (s_bt_ui.start < 0) s_bt_ui.start = 0;
    if (s_bt_ui.start > max_start) s_bt_ui.start = max_start;

    if (s_bt_ui.sel < s_bt_ui.start) {
        s_bt_ui.start = s_bt_ui.sel;
    } else if (s_bt_ui.sel >= s_bt_ui.start + max_vis) {
        s_bt_ui.start = s_bt_ui.sel - (max_vis - 1);
    }
}

static void bt_refresh_list(void){
    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);

    s_bt_ui.count = bt_audio_get_devices(s_bt_ui.devs, 8);
    s_bt_ui.peer_extra = false;
    s_bt_ui.peer_index = -1;

    if ((st.state == BT_AUDIO_STATE_CONNECTED || st.state == BT_AUDIO_STATE_STREAMING) && st.peer_bda[0] != 0) {
        int idx = bt_find_dev_idx(s_bt_ui.devs, s_bt_ui.count, st.peer_bda);
        if (idx < 0 && s_bt_ui.count < 8){
            idx = s_bt_ui.count;
            memcpy(s_bt_ui.devs[idx].bda, st.peer_bda, 6);
            if (st.peer_name[0]) {
                strncpy(s_bt_ui.devs[idx].name, st.peer_name, sizeof(s_bt_ui.devs[idx].name) - 1);
                s_bt_ui.devs[idx].name[sizeof(s_bt_ui.devs[idx].name) - 1] = '\0';
            } else {
                s_bt_ui.devs[idx].name[0] = '\0';
            }
            s_bt_ui.peer_extra = true;
            s_bt_ui.peer_index = idx;
            s_bt_ui.count++;
        } else if (idx >= 0) {
            s_bt_ui.peer_index = idx;
        }
    }

    bt_update_viewport();
}

static void bt_make_label(const bt_audio_device_t *dev, char *out, size_t out_sz){
    if (!dev || !out || out_sz == 0) return;
    if (dev->name[0]) {
        strncpy(out, dev->name, out_sz - 1);
        out[out_sz - 1] = '\0';
    } else {
        snprintf(out, out_sz, "%02X:%02X:%02X", dev->bda[3], dev->bda[4], dev->bda[5]);
    }
}

void bt_audio_app_init(void)
{
    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    // Only kick off a scan if we're not already connected/connecting/streaming.
    if (!(st.state == BT_AUDIO_STATE_CONNECTED ||
          st.state == BT_AUDIO_STATE_STREAMING ||
          st.state == BT_AUDIO_STATE_CONNECTING ||
          st.state == BT_AUDIO_STATE_DISCOVERING)) {
        (void)bt_audio_scan(); // show list immediately, no auto-connect
    }
}

void bt_audio_app_handle_input(const input_event_t *ev)
{
    if (!ev) return;
    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_A) {           // up
            if (s_bt_ui.sel > 0) s_bt_ui.sel--;
        } else if (ev->button == INPUT_BTN_B) {    // down
            if (s_bt_ui.sel + 1 < s_bt_ui.count) s_bt_ui.sel++;
        } else if (ev->button == INPUT_BTN_C) {    // scan
            (void)bt_audio_scan();
            s_bt_ui.sel = 0;
            bt_refresh_list();
        } else if (ev->button == INPUT_BTN_D) {    // select/connect/disconnect
            bt_audio_status_t st = {0};
            bt_audio_get_status(&st);
            bool connected = (st.state == BT_AUDIO_STATE_CONNECTED || st.state == BT_AUDIO_STATE_STREAMING);
            bool selecting_peer = (s_bt_ui.peer_index >= 0 && s_bt_ui.sel == s_bt_ui.peer_index);
            if (connected && selecting_peer) {
                (void)bt_audio_disconnect();
            } else {
                if (connected && !selecting_peer) {
                    (void)bt_audio_disconnect();
                }
                (void)bt_audio_connect_index(s_bt_ui.sel);
            }
        }
    }

    bt_update_viewport();
}

void bt_audio_app_draw(uint8_t *fb, int x, int y, int w, int h)
{
    (void)w; (void)h;
    bt_refresh_list();
    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    if ((s_bt_prev_state == BT_AUDIO_STATE_CONNECTED || s_bt_prev_state == BT_AUDIO_STATE_STREAMING) &&
        !(st.state == BT_AUDIO_STATE_CONNECTED || st.state == BT_AUDIO_STATE_STREAMING)) {
        bt_audio_invoke_disconnect_cb();
    }
    s_bt_prev_state = st.state;

    char line[48];
    snprintf(line, sizeof(line), "BT:%s", bt_audio_state_name(st.state));
    oled_draw_text3x5(fb, x + 2, y + 2, line);
    if (st.peer_name[0]) {
        snprintf(line, sizeof(line), "Conn:%s", st.peer_name);
        oled_draw_text3x5(fb, x + 2, y + 10, line);
    }

    const int max_vis = k_bt_list_max_vis;
    int start = s_bt_ui.start;
    int visible = s_bt_ui.count - start;
    if (visible > max_vis) visible = max_vis;
    if (visible < 0) visible = 0;

    int list_y = y + 18;
    if (s_bt_ui.count == 0) {
        oled_draw_text3x5(fb, x + 2, list_y, "Scanning...");
    } else {
        for (int i = 0; i < visible; ++i) {
            int idx = start + i;
            int yy = list_y + i * 8;
            char dev_label[24];
            bt_make_label(&s_bt_ui.devs[idx], dev_label, sizeof(dev_label));
            bool is_current = (memcmp(s_bt_ui.devs[idx].bda, st.peer_bda, sizeof(st.peer_bda)) == 0) &&
                              (st.state == BT_AUDIO_STATE_CONNECTED ||
                               st.state == BT_AUDIO_STATE_STREAMING ||
                               st.state == BT_AUDIO_STATE_CONNECTING);
            snprintf(line, sizeof(line), "%c%s %s",
                     (idx == s_bt_ui.sel) ? '>' : ' ',
                     is_current ? "[*]" : "[ ]",
                     dev_label);
            oled_draw_text3x5(fb, x + 2, yy, line);
        }
    }
}
