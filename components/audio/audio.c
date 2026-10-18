// __cplusplus // components/audio/audio.c
#include "audio.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "audio_wav.h"

// Optional hook provided by audio_player to abort playback when switching tracks.
bool audio_player_should_abort(void) __attribute__((weak));
bool audio_player_should_abort(void){ return false; }
bool audio_player_should_pause(void) __attribute__((weak));
bool audio_player_should_pause(void){ return false; }

static const char* TAG = "audio";

// ====================== I2S state ======================
static i2s_chan_handle_t   s_tx = NULL;     // speaker (MAX98357A)
static i2s_chan_handle_t   s_rx = NULL;     // microphone (INMP441)
static audio_i2s_config_t  g_last_cfg = {0};
static volatile int        s_curr_rate = 48000; // default boot rate
static volatile bool       s_rate_changing;
static volatile uint32_t   s_clock_revision = 0;
static volatile uint32_t   s_rx_overflows = 0;
static portMUX_TYPE        s_output_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned            s_output_users;
static int32_t             s_vol_q15   = 32767; // 1.0 in Q1.15
static bool                s_tx_enabled = false;
static bool                s_rx_enabled = false;
static bool                s_bus_configured;
static portMUX_TYPE s_progress_mux = portMUX_INITIALIZER_UNLOCKED;
static audio_progress_t s_progress;

void audio_progress_reset(void)
{
    portENTER_CRITICAL(&s_progress_mux);
    memset(&s_progress, 0, sizeof(s_progress));
    portEXIT_CRITICAL(&s_progress_mux);
}

void audio_progress_submit(size_t frames, uint32_t rate, bool bluetooth)
{
    if (!frames || !rate) return;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_progress_mux);
    s_progress.frames += frames;
    s_progress.rate = rate;
    s_progress.bluetooth = bluetooth;
    s_progress.updated_us = now;
    portEXIT_CRITICAL(&s_progress_mux);
}

audio_progress_t audio_progress_get(void)
{
    portENTER_CRITICAL(&s_progress_mux);
    audio_progress_t p = s_progress;
    portEXIT_CRITICAL(&s_progress_mux);
    return p;
}

// Yield occasionally during long streaming loops to keep UI responsive.
#define AUDIO_PLAY_YIELD_EVERY    4
#define AUDIO_PLAY_YIELD_DELAY_MS 1
// Keep TX writes <= one DMA frame worth of bytes to avoid all-or-nothing stalls.
#define AUDIO_TX_WRITE_SLICE_BYTES 2048

// Small static zero blocks to avoid stack usage
// I2S write copies these task-context buffers into the driver's internal DMA.
static EXT_RAM_BSS_ATTR int32_t s_zero256_stereo[256 * 2];
static EXT_RAM_BSS_ATTR int32_t s_zero512_stereo[512 * 2];

// Forward
static esp_err_t audio_quiet_tail(int fade_ms, int zero_ms);

// ---------------- Minimal WAV structs for memory variant ----------------
typedef struct __attribute__((packed)) {
    char     riff[4];       // "RIFF"
    uint32_t riff_size;
    char     wave[4];       // "WAVE"
} riff_t;

typedef struct __attribute__((packed)) {
    char     id[4];         // "fmt " / "data" / ...
    uint32_t size;
} chunk_hdr_t;

typedef struct __attribute__((packed)) {
    uint16_t audio_format;      // 1 = PCM
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;   // 16
} fmt_t;

// ====================== helpers ======================
static inline uint32_t fcc4(const char s[4]){
    return ((uint32_t)(uint8_t)s[0]) | ((uint32_t)(uint8_t)s[1]<<8) |
           ((uint32_t)(uint8_t)s[2]<<16) | ((uint32_t)(uint8_t)s[3]<<24);
}
#define FCC(a,b,c,d) ( ((uint32_t)(a)) | ((uint32_t)(b)<<8) | ((uint32_t)(c)<<16) | ((uint32_t)(d)<<24) )

static inline i2s_clock_src_t clk_src_for_rate(int hz) {
    // 44.1k family → APLL, 8k/16k/32k/48k family → DEFAULT (PLL160M)
    return (hz % 11025) == 0 ? I2S_CLK_SRC_APLL : I2S_CLK_SRC_DEFAULT;
}

// Tolerant fread (handles transient SD CRC hiccups)
static size_t fread_retry(void *ptr, size_t sz, size_t cnt, FILE *f) {
    for (int i = 0; i < 3; ++i) {
        size_t got = fread(ptr, sz, cnt, f);
        if (got == cnt) return got;
        clearerr(f);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return 0;
}

// ====================== Public API ======================

static bool IRAM_ATTR audio_rx_overflow(i2s_chan_handle_t ch, i2s_event_data_t *event, void *user)
{
    (void)ch; (void)event; (void)user;
    ++s_rx_overflows;
    return false;
}

esp_err_t audio_init(const audio_i2s_config_t* cfg){
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_tx || s_rx) return ESP_ERR_INVALID_STATE;
    g_last_cfg  = *cfg;
    s_bus_configured = true;
    s_curr_rate = 48000;
    s_tx_enabled = s_rx_enabled = false;

    if (cfg->sd_en_pin >= 0) {
        gpio_reset_pin(cfg->sd_en_pin);
        gpio_set_direction(cfg->sd_en_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(cfg->sd_en_pin, 0); // start muted
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(cfg->i2s_id, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 256; // <= 511 to avoid clamp warning
    chan_cfg.auto_clear    = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_tx, &s_rx), TAG, "new_channel(tx+rx)");

    i2s_std_config_t tx_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(s_curr_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = cfg->bclk_gpio,
            .ws   = cfg->lrck_gpio,
            .dout = cfg->dout_gpio,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    tx_cfg.clk_cfg.clk_src         = clk_src_for_rate(s_curr_rate);
    tx_cfg.clk_cfg.mclk_multiple   = I2S_MCLK_MULTIPLE_256;
    tx_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    tx_cfg.slot_cfg.bit_shift      = true;
    tx_cfg.slot_cfg.slot_mask      = I2S_STD_SLOT_BOTH;

    i2s_std_config_t rx_cfg = tx_cfg;
    rx_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT; // keep RX aligned like TX; scale handled in FFT
    rx_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    rx_cfg.gpio_cfg.din  = cfg->din_gpio;

    esp_err_t err = i2s_channel_init_std_mode(s_tx, &tx_cfg);
    if (err != ESP_OK) goto fail;
    if (cfg->din_gpio >= 0) {
        err = i2s_channel_init_std_mode(s_rx, &rx_cfg);
        if (err != ESP_OK) goto fail;
        i2s_event_callbacks_t callbacks = { .on_recv_q_ovf = audio_rx_overflow };
        err = i2s_channel_register_event_callback(s_rx, &callbacks, NULL);
        if (err != ESP_OK) goto fail;
    } else {
        i2s_del_channel(s_rx);
        s_rx = NULL;
    }

    ESP_LOGI(TAG, "I2S ready: fs=%d, BCLK=%d, LRCK=%d, DOUT=%d, DIN=%d",
             s_curr_rate, cfg->bclk_gpio, cfg->lrck_gpio, cfg->dout_gpio, cfg->din_gpio);
    return ESP_OK;
fail:
    if (s_rx) { i2s_del_channel(s_rx); s_rx = NULL; }
    if (s_tx) { i2s_del_channel(s_tx); s_tx = NULL; }
    return err;
}

esp_err_t audio_release_idle_bus(void)
{
    if (audio_output_active()) return ESP_ERR_INVALID_STATE;
    if (s_tx_enabled) {
        ESP_RETURN_ON_ERROR(i2s_channel_disable(s_tx), TAG, "stop tx");
        s_tx_enabled = false;
    }
    if (s_rx_enabled) {
        ESP_RETURN_ON_ERROR(i2s_channel_disable(s_rx), TAG, "stop rx");
        s_rx_enabled = false;
    }
    if (g_last_cfg.sd_en_pin >= 0 && s_bus_configured)
        gpio_set_level(g_last_cfg.sd_en_pin, 0);
    if (s_rx) {
        ESP_RETURN_ON_ERROR(i2s_del_channel(s_rx), TAG, "release rx");
        s_rx = NULL;
    }
    if (s_tx) {
        ESP_RETURN_ON_ERROR(i2s_del_channel(s_tx), TAG, "release tx");
        s_tx = NULL;
    }
    ++s_clock_revision;
    return ESP_OK;
}

static esp_err_t audio_restore_bus(void)
{
    if (s_tx) return ESP_OK;
    if (!s_bus_configured) return ESP_ERR_INVALID_STATE;
    int rate = s_curr_rate;
    ESP_RETURN_ON_ERROR(audio_init(&g_last_cfg), TAG, "restore bus");
    return audio_set_rate(rate);
}

static esp_err_t audio_reconfig_clock(i2s_chan_handle_t ch, int hz){
    if (!ch) return ESP_OK;
    if ((ch == s_tx && s_tx_enabled) || (ch == s_rx && s_rx_enabled)) {
        (void)i2s_channel_disable(ch);
        if (ch == s_tx) s_tx_enabled = false; else s_rx_enabled = false;
    }

    i2s_std_clk_config_t clk = {
        .clk_src        = clk_src_for_rate(hz),
        .sample_rate_hz = hz,
        .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
    };
    return i2s_channel_reconfig_std_clock(ch, &clk);
}

esp_err_t audio_set_rate(int hz){
    if (hz <= 0) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(audio_restore_bus(), TAG, "restore I2S");
    s_rate_changing = true; // capture must discard windows spanning reconfiguration
    ++s_clock_revision;
    ESP_RETURN_ON_ERROR(audio_reconfig_clock(s_tx, hz), TAG, "tx re-clock");
    ESP_RETURN_ON_ERROR(audio_reconfig_clock(s_rx, hz), TAG, "rx re-clock");
    s_curr_rate = hz;
    s_rate_changing = false;
    return ESP_OK;
}

int audio_get_rate(void) { return s_rate_changing ? 0 : s_curr_rate; }
uint32_t audio_get_clock_revision(void) { return s_clock_revision; }
uint32_t audio_get_rx_overflows(void) { return s_rx_overflows; }

bool audio_output_active(void)
{
    portENTER_CRITICAL(&s_output_mux);
    bool active = s_output_users > 0;
    portEXIT_CRITICAL(&s_output_mux);
    return active;
}

static void audio_output_enter(void)
{ portENTER_CRITICAL(&s_output_mux); ++s_output_users; portEXIT_CRITICAL(&s_output_mux); }
static esp_err_t audio_output_leave(esp_err_t err)
{ portENTER_CRITICAL(&s_output_mux); --s_output_users; portEXIT_CRITICAL(&s_output_mux); return err; }

esp_err_t audio_enable_tx(void){
    ESP_RETURN_ON_ERROR(audio_restore_bus(), TAG, "restore I2S");
    if (!s_tx) return ESP_ERR_INVALID_STATE;
    if (!s_tx_enabled) {
        if (g_last_cfg.sd_en_pin >= 0) gpio_set_level(g_last_cfg.sd_en_pin, 1); // unmute
        esp_err_t e = i2s_channel_enable(s_tx);
        if (e == ESP_ERR_TIMEOUT) {
            // Recover from occasional stalled TX state by re-arming the channel.
            ESP_LOGW(TAG, "TX enable timeout; retrying");
            (void)i2s_channel_disable(s_tx);
            vTaskDelay(pdMS_TO_TICKS(1));
            e = i2s_channel_enable(s_tx);
        }
        if (e == ESP_ERR_TIMEOUT && s_rx && !s_rx_enabled) {
            // Some full-duplex driver states need RX armed before TX can drain.
            ESP_LOGW(TAG, "TX enable still timed out; arming RX and retrying TX");
            esp_err_t re = i2s_channel_enable(s_rx);
            if (re == ESP_OK || re == ESP_ERR_INVALID_STATE) {
                s_rx_enabled = true;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            e = i2s_channel_enable(s_tx);
        }
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        s_tx_enabled = true;
    }
    // In full-duplex allocation mode, some ESP32 driver states only progress TX
    // writes when RX is also enabled on the paired channel.
    if (s_rx && !s_rx_enabled) {
        esp_err_t re = i2s_channel_enable(s_rx);
        if (re == ESP_OK || re == ESP_ERR_INVALID_STATE) {
            s_rx_enabled = true;
        } else {
            ESP_LOGW(TAG, "RX companion enable failed while enabling TX: %s",
                     esp_err_to_name(re));
        }
    }
    // Prime TX DMA once after enable; if this stalls, surface it immediately.
    size_t just = 0;
    // IDF 6.x API expects timeout in milliseconds (not RTOS ticks).
    esp_err_t pe = i2s_channel_write(
        s_tx, s_zero256_stereo, sizeof(s_zero256_stereo), &just, 40);
    if (pe == ESP_ERR_TIMEOUT || just == 0) {
        ESP_LOGW(TAG, "TX prime failed: err=%s wrote=%u",
                 esp_err_to_name(pe), (unsigned)just);
        return ESP_ERR_TIMEOUT;
    }
    if (pe != ESP_OK) {
        ESP_LOGW(TAG, "TX prime write error: %s", esp_err_to_name(pe));
        return pe;
    }
    return ESP_OK;
}

esp_err_t audio_enable_rx(void){
    ESP_RETURN_ON_ERROR(audio_restore_bus(), TAG, "restore I2S");
    if (!s_rx) return ESP_ERR_INVALID_STATE;

    // Mute amp while recording
    if (g_last_cfg.sd_en_pin >= 0) gpio_set_level(g_last_cfg.sd_en_pin, 0);

    // Provide clocks to mic: enable TX once and push a tiny zero block
    if (!s_tx_enabled) {
        esp_err_t e = i2s_channel_enable(s_tx);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        s_tx_enabled = true;
        size_t just = 0;
        (void)i2s_channel_write(s_tx, s_zero256_stereo, sizeof(s_zero256_stereo), &just, 10);
    }

    if (!s_rx_enabled) {
        esp_err_t e = i2s_channel_enable(s_rx);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        s_rx_enabled = true;
    }
    return ESP_OK;
}

esp_err_t audio_disable_all(void){
    if (s_tx_enabled) { (void)i2s_channel_disable(s_tx); s_tx_enabled = false; }
    if (s_rx_enabled) { (void)i2s_channel_disable(s_rx); s_rx_enabled = false; }
    if (g_last_cfg.sd_en_pin >= 0) gpio_set_level(g_last_cfg.sd_en_pin, 0);
    return ESP_OK;
}

i2s_chan_handle_t audio_tx_handle(void){ return s_tx; }
i2s_chan_handle_t audio_rx_handle(void){ return s_rx; }

// ====================== Volume ======================
esp_err_t audio_set_volume(float vol){
    if (vol < 0.f) vol = 0.f;
    if (vol > 2.f) vol = 2.f; // mild boost cap
    s_vol_q15 = (int32_t)(vol * 32767.f + 0.5f);
    return ESP_OK;
}
float audio_get_volume(void) { return (float)s_vol_q15 / 32767.f; }

// ====================== Playback: WAV from file ======================
//
// Uses audio_wav_parse_file() so BT + local speaker share one parser.
// Supports mono PCM16 only. For the embedded speaker we duplicate mono
// into both L/R I2S slots.
//
// ====================== Local playback using parsed WAV header ======================
//
// Uses audio_wav_info_t (parsed by audio_wav_parse_file) to play mono PCM16
// data via the speaker path (duplicated to stereo I2S slots).
//
#define AUDIO_RING_BYTES (32 * 1024) // PSRAM-backed ring (must hold even number of samples)

typedef struct {
    uint8_t *buf;
    size_t   size;
    size_t   rd;
    size_t   wr;
    size_t   fill;
} audio_ring_t;

static audio_ring_t ring_alloc(size_t bytes){
    audio_ring_t r = {0};
    if (bytes == 0) return r;
    uint8_t *p = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) return r;
    r.buf = p;
    r.size = bytes;
    r.rd = r.wr = r.fill = 0;
    return r;
}

static void ring_free(audio_ring_t *r){
    if (!r) return;
    if (r->buf) free(r->buf);
    r->buf = NULL;
    r->size = r->rd = r->wr = r->fill = 0;
}

//unused
//static size_t ring_write(audio_ring_t *r, const uint8_t *src, size_t n){
//    if (!r || !r->buf || n == 0) return 0;
//    size_t wrote = 0;
//    while (wrote < n && r->fill < r->size){
//        size_t chunk = r->size - r->wr;
//        if (chunk > (n - wrote)) chunk = n - wrote;
//        if (chunk > (r->size - r->fill)) chunk = r->size - r->fill;
//        if (chunk == 0) break;
//        memcpy(r->buf + r->wr, src + wrote, chunk);
//        r->wr = (r->wr + chunk) % r->size;
//        r->fill += chunk;
//        wrote += chunk;
//    }
//    return wrote;
//}

static size_t ring_read(audio_ring_t *r, uint8_t *dst, size_t n){
    if (!r || !r->buf || n == 0) return 0;
    size_t rd = 0;
    while (rd < n && r->fill > 0){
        size_t chunk = r->size - r->rd;
        if (chunk > (n - rd)) chunk = n - rd;
        if (chunk > r->fill) chunk = r->fill;
        if (chunk == 0) break;
        memcpy(dst + rd, r->buf + r->rd, chunk);
        r->rd = (r->rd + chunk) % r->size;
        r->fill -= chunk;
        rd += chunk;
    }
    return rd;
}

// Local WAV playback using I2S speaker (mono PCM16 only)
static esp_err_t play_wav_local_stream(FILE *f, const audio_wav_info_t *info)
{
    if (!f || !info) {
        return ESP_ERR_INVALID_ARG;
    }

    const audio_wav_fmt_t *fmt = &info->fmt;

    if (fmt->audio_format != 1 || fmt->bits_per_sample != 16 ||
        (fmt->num_channels != 1 && fmt->num_channels != 2)) {
        ESP_LOGW(TAG, "local_stream: unsupported fmt=%u ch=%u bits=%u",
                 fmt->audio_format, fmt->num_channels, fmt->bits_per_sample);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "local_stream: %u Hz, ch=%u, bits=%u, data=%u",
             fmt->sample_rate, fmt->num_channels, fmt->bits_per_sample,
             (unsigned)info->data_size);

    // 1) Set I2S sample rate and enable TX
    ESP_RETURN_ON_ERROR(audio_set_rate((int)fmt->sample_rate), TAG, "set fs");
    ESP_RETURN_ON_ERROR(audio_enable_tx(),                    TAG, "enable tx");

    // 2) Seek to data offset in the same FILE already opened by caller
    if (fseek(f, (long)info->data_offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "local_stream: fseek to data_offset failed");
        return ESP_FAIL;
    }

    // 3) Streaming loop: read PCM16 frames (with PSRAM ring), convert to stereo s32, write to I2S

    const size_t FRAMES_PER_CHUNK = 512;  // audio frames per chunk
    const size_t IN_SAMPLES       = FRAMES_PER_CHUNK * fmt->num_channels;

    // i2s_channel_write copies into its own DMA buffers; staging can use PSRAM.
    int16_t *in = heap_caps_malloc(IN_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int32_t *out = heap_caps_malloc(FRAMES_PER_CHUNK * 2 * sizeof(int32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    audio_ring_t ring = ring_alloc(AUDIO_RING_BYTES);

    if (!in || !out) {
        ESP_LOGE(TAG, "local_stream: malloc failed");
        free(in);
        free(out);
        ring_free(&ring);
        return ESP_ERR_NO_MEM;
    }

    uint32_t bytes_left = info->data_size;
    unsigned int yield_count = 0;

    while (!audio_player_should_abort()) {
        if (audio_player_should_pause()) {
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        size_t rd = 0;

        if (ring.buf) {
            // Top up ring if running low
            if (ring.fill < (ring.size / 2) && bytes_left > 0) {
                size_t to_read = ring.size - ring.fill;
                if (to_read > bytes_left) to_read = bytes_left;
                if (to_read > (8 * 1024)) to_read = 8 * 1024; // moderate burst

                size_t space_to_end = ring.size - ring.wr;
                size_t first = to_read > space_to_end ? space_to_end : to_read;
                size_t got1 = fread_retry(ring.buf + ring.wr, 1, first, f);
                size_t got2 = 0;
                if (got1 > 0) {
                    ring.wr = (ring.wr + got1) % ring.size;
                    ring.fill += got1;
                }
                if (got1 == first && (to_read - got1) > 0) {
                    size_t second = to_read - got1;
                    got2 = fread_retry(ring.buf + ring.wr, 1, second, f);
                    if (got2 > 0) {
                        ring.wr = (ring.wr + got2) % ring.size;
                        ring.fill += got2;
                    }
                }
                size_t got = got1 + got2;
                if (got > 0 && bytes_left >= got) bytes_left -= (uint32_t)got;
            }

            if (ring.fill == 0 && bytes_left == 0) break;

            size_t want_bytes = IN_SAMPLES * sizeof(int16_t);
            if (want_bytes > ring.fill) want_bytes = ring.fill;
            rd = ring_read(&ring, (uint8_t*)in, want_bytes);
        } else {
            if (bytes_left == 0) break;
            size_t want_bytes = IN_SAMPLES * sizeof(int16_t);
            if (want_bytes > bytes_left) want_bytes = bytes_left;
            rd = fread_retry(in, 1, want_bytes, f);
            if (rd > bytes_left) rd = bytes_left;
            if (bytes_left >= rd) bytes_left -= (uint32_t)rd;
        }

        if (rd == 0) {
            break;  // EOF or read error
        }

        // Convert bytes -> samples -> frames (mono source)
        size_t samples = rd / sizeof(int16_t);
        size_t frames  = samples / fmt->num_channels;

        if (frames == 0) {
            break;
        }

        // Build stereo s32 from mono PCM16
        for (size_t i = 0; i < frames; ++i) {
            int32_t s = in[i * fmt->num_channels];
            if (fmt->num_channels == 2) s = (s + in[i * 2 + 1]) / 2;

            // Apply volume (Q15), then convert s16 → s32 MSB-justified
            int32_t vs  = (int32_t)((s * (int64_t)s_vol_q15) >> 15);
            if (vs > 32767) vs = 32767;
            if (vs < -32768) vs = -32768;
            int32_t v32 = vs * 65536;

            out[2 * i + 0] = v32;  // left
            out[2 * i + 1] = v32;  // right
        }

        size_t bytes_out = frames * 2 * sizeof(int32_t);
        size_t written   = 0;

        while (written < bytes_out && !audio_player_should_abort()) {
            if (audio_player_should_pause()) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            size_t just = 0;
            size_t req = bytes_out - written;
            if (req > AUDIO_TX_WRITE_SLICE_BYTES) req = AUDIO_TX_WRITE_SLICE_BYTES;
            esp_err_t e = i2s_channel_write(
                s_tx,
                ((uint8_t *)out) + written,
                req,
                &just,
                200
            );

            written += just;
            audio_progress_submit(just / 8, fmt->sample_rate, false);
            if (e == ESP_ERR_TIMEOUT) {
                // Retry instead of dropping samples; avoid underruns that speed playback up.
                continue;
            }
            if (e != ESP_OK) {
                ESP_LOGE(TAG, "local_stream: i2s write %s", esp_err_to_name(e));
                free(in);
                free(out);
                ring_free(&ring);
                return e;
            }
            if (just == 0) {
                // No progress – bail out of loop
                break;
            }
        }

        if (++yield_count >= AUDIO_PLAY_YIELD_EVERY) {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_PLAY_YIELD_DELAY_MS));
            yield_count = 0;
        }
    }

    free(in);
    free(out);
    ring_free(&ring);

    // Send a short silence tail to avoid pops, then mute/disable TX
    (void)audio_quiet_tail(12, 8);
    return ESP_OK;
}


static esp_err_t play_wav_file(const char* path){
    if (!path) return ESP_ERR_INVALID_ARG;

    FILE* f = fopen(path, "rb");
    if (!f) return ESP_ERR_NOT_FOUND;

    audio_wav_info_t info = {0};
    esp_err_t err = audio_wav_parse_file(f, &info);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "audio_wav_parse_file failed (%s)", esp_err_to_name(err));
        fclose(f);
        return err;
    }

    if (info.fmt.audio_format != 1 || info.fmt.bits_per_sample != 16) {
        ESP_LOGW(TAG, "Unsupported WAV format: fmt=%u bits=%u",
                 info.fmt.audio_format, info.fmt.bits_per_sample);
        fclose(f);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (info.fmt.num_channels != 1) {
        ESP_LOGW(TAG, "Unsupported channel count: %u", info.fmt.num_channels);
        fclose(f);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "WAV: %u Hz, %u ch, %u bits, data=%u",
             info.fmt.sample_rate,
             info.fmt.num_channels,
             info.fmt.bits_per_sample,
             (unsigned)info.data_size);

    ESP_RETURN_ON_ERROR(audio_set_rate((int)info.fmt.sample_rate), TAG, "set fs");
    ESP_RETURN_ON_ERROR(audio_enable_tx(),                        TAG, "enable tx");

    if (fseek(f, (long)info.data_offset, SEEK_SET) != 0) {
        fclose(f);
        return ESP_FAIL;
    }

    const size_t FRAMES   = 1024; // frames per chunk
    int16_t *in  = (int16_t*)malloc(FRAMES * sizeof(int16_t));
    int32_t *out = (int32_t*)malloc(FRAMES * 2 * sizeof(int32_t)); // stereo s32 out
    if (!in || !out) {
        free(in); free(out);
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    uint32_t bytes_left = info.data_size;
    unsigned int yield_count = 0;

    while (bytes_left > 0 && !audio_player_should_abort()) {
        if (audio_player_should_pause()) {
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
        size_t frames_to_read = FRAMES;
        size_t bytes_to_read  = frames_to_read * sizeof(int16_t);
        if (bytes_left < bytes_to_read) {
            frames_to_read = bytes_left / sizeof(int16_t);
            bytes_to_read  = frames_to_read * sizeof(int16_t);
        }
        if (frames_to_read == 0) break;

        size_t rd = fread_retry(in, 1, bytes_to_read, f);
        if (rd == 0) break;

        size_t frames = rd / sizeof(int16_t);
        // Mono in -> mono out (duplicated into both I2S slots)
        for (size_t i = 0; i < frames; ++i) {
            int32_t v  = in[i];
            int32_t vs = (int32_t)((v * (int64_t)s_vol_q15) >> 15); // s16
            int32_t v32 = vs << 16;                                 // s16->s32 MSB
            out[2*i + 0] = v32;
            out[2*i + 1] = v32;
        }

        size_t bytes_out = frames * 2 * sizeof(int32_t);
        size_t written = 0;
        while (written < bytes_out && !audio_player_should_abort()) {
            if (audio_player_should_pause()) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            size_t just = 0;
            size_t req = bytes_out - written;
            if (req > AUDIO_TX_WRITE_SLICE_BYTES) req = AUDIO_TX_WRITE_SLICE_BYTES;
            esp_err_t e = i2s_channel_write(
                s_tx,
                ((uint8_t*)out) + written,
                req,
                &just,
                200
            );
            if (e == ESP_ERR_TIMEOUT) continue;   // retry to avoid dropping samples
            if (e != ESP_OK) {
                ESP_LOGE(TAG, "i2s write %s", esp_err_to_name(e));
                free(in); free(out); fclose(f);
                return e;
            }
            written += just;
        }

        if (++yield_count >= AUDIO_PLAY_YIELD_EVERY) {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_PLAY_YIELD_DELAY_MS));
            yield_count = 0;
        }
        bytes_left -= (uint32_t)rd;
    }

    free(in);
    free(out);
    fclose(f);

    (void)audio_quiet_tail(12, 8);
    return ESP_OK;
}

// ====================== Memory variant (same format, mono only) ======================
static esp_err_t play_wav_mem(const void* data, size_t size){
    if (!data || size < sizeof(riff_t)) return ESP_ERR_INVALID_ARG;

    const uint8_t *p   = (const uint8_t*)data;
    const uint8_t *end = p + size;

    if ((size_t)(end - p) < sizeof(riff_t)) return ESP_ERR_INVALID_RESPONSE;
    const riff_t* rh = (const riff_t*)p; p += sizeof(riff_t);
    if (memcmp(rh->riff,"RIFF",4) || memcmp(rh->wave,"WAVE",4)) return ESP_ERR_INVALID_RESPONSE;

    fmt_t fmt = {0};
    const uint8_t *data_ptr = NULL;
    uint32_t data_size = 0;

    while (p + sizeof(chunk_hdr_t) <= end) {
        const chunk_hdr_t* ch = (const chunk_hdr_t*)p; p += sizeof(chunk_hdr_t);
        if (p + ch->size > end) break;

        uint32_t id = fcc4(ch->id);
        if (id == FCC('f','m','t',' ')) {
            size_t n = ch->size < sizeof(fmt) ? ch->size : sizeof(fmt);
            memcpy(&fmt, p, n);
        } else if (id == FCC('d','a','t','a')) {
            data_ptr  = p;
            data_size = ch->size;
            break;
        }
        p += ch->size;
        if (ch->size & 1) p++; // pad
    }

    if (!data_ptr || data_size == 0) return ESP_ERR_INVALID_RESPONSE;
    if (fmt.audio_format != 1 || fmt.num_channels != 1 || fmt.bits_per_sample != 16) return ESP_ERR_NOT_SUPPORTED;

    ESP_RETURN_ON_ERROR(audio_set_rate((int)fmt.sample_rate), TAG, "set fs");
    ESP_RETURN_ON_ERROR(audio_enable_tx(),                  TAG, "enable tx");

    const int16_t *in = (const int16_t*)data_ptr;
    size_t samples = data_size / sizeof(int16_t);

    const size_t CHUNK = 1024;
    int32_t *st = (int32_t*)malloc(CHUNK * 2 * sizeof(int32_t));
    if (!st) return ESP_ERR_NO_MEM;

    size_t idx = 0;
    while (idx < samples) {
        size_t n = (samples - idx) > CHUNK ? CHUNK : (samples - idx);
        for (size_t i=0;i<n;i++) {
            int32_t v  = in[idx+i];
            int32_t vs = (int32_t)((v * (int64_t)s_vol_q15) >> 15);
            int32_t v32 = vs << 16;
            st[2*i+0] = v32; st[2*i+1] = v32;
        }
        size_t bytes = n * 2 * sizeof(int32_t);
        size_t written = 0, just = 0;
        while (written < bytes) {
            size_t req = bytes - written;
            if (req > AUDIO_TX_WRITE_SLICE_BYTES) req = AUDIO_TX_WRITE_SLICE_BYTES;
            esp_err_t e = i2s_channel_write(s_tx, ((uint8_t*)st)+written, req, &just, 60);
            if (e == ESP_ERR_TIMEOUT) break;
            if (e != ESP_OK)          { free(st); return e; }
            written += just;
        }
        idx += n;
    }
    free(st);

    (void)audio_quiet_tail(12, 8);
    return ESP_OK;
}

// ====================== Quiet tail (heap/static, not stack) ======================
static esp_err_t audio_quiet_tail(int fade_ms, int zero_ms){
    if (!s_tx) return ESP_OK;

    // Make sure TX is enabled for the tail
    if (!s_tx_enabled) {
        (void)i2s_channel_enable(s_tx);
        s_tx_enabled = true;
    }

    const int Fs = s_curr_rate > 0 ? s_curr_rate : 48000;
    int frames = (fade_ms * Fs) / 1000 + (zero_ms * Fs) / 1000;
    if (frames < 1) frames = 1;

    int left = frames;
    while (left > 0) {
        int n = left > 512 ? 512 : left;
        size_t bytes = (size_t)n * 2 * sizeof(int32_t);
        size_t wr = 0;
        while (wr < bytes) {
            size_t just = 0;
            size_t req = bytes - wr;
            if (req > AUDIO_TX_WRITE_SLICE_BYTES) req = AUDIO_TX_WRITE_SLICE_BYTES;
            esp_err_t e = i2s_channel_write(s_tx, ((uint8_t*)s_zero512_stereo) + wr, req, &just, 10);
            if (e != ESP_OK || just == 0) {
                wr = bytes;
                break;
            }
            wr += just;
        }
        left -= n;
    }

    if (g_last_cfg.sd_en_pin >= 0) gpio_set_level(g_last_cfg.sd_en_pin, 0); // mute
    vTaskDelay(pdMS_TO_TICKS(2));
    (void)i2s_channel_disable(s_tx);
    s_tx_enabled = false;
    return ESP_OK;
}

// =================== Embedded asset & tone ===================
extern const uint8_t _binary_assets_test_wav_start[] __attribute__((weak));
extern const uint8_t _binary_assets_test_wav_end[]   __attribute__((weak));
extern const uint8_t _binary_test_wav_start[]        __attribute__((weak));
extern const uint8_t _binary_test_wav_end[]          __attribute__((weak));

esp_err_t audio_play_embedded(void){
    const uint8_t *start = NULL, *end = NULL;
    if (_binary_assets_test_wav_start && _binary_assets_test_wav_end) {
        start = _binary_assets_test_wav_start; end = _binary_assets_test_wav_end;
    } else if (_binary_test_wav_start && _binary_test_wav_end) {
        start = _binary_test_wav_start; end = _binary_test_wav_end;
    } else {
        return ESP_ERR_NOT_FOUND;
    }
    size_t sz = (size_t)(end - start);
    return audio_play_wav_mem(start, sz);
}

static esp_err_t play_tone(int hz, int ms){
    ESP_RETURN_ON_ERROR(audio_restore_bus(), TAG, "restore I2S");
    if (hz <= 0) hz = 440;
    if (ms <= 0) ms = 250;

    const int Fs = s_curr_rate;
    const size_t CHUNK = 512;                  // frames per chunk
    int32_t *st = (int32_t*)malloc(CHUNK * 2 * sizeof(int32_t)); // stereo s32
    if (!st) return ESP_ERR_NO_MEM;

    uint32_t phase = 0;                        // Q16.16
    uint32_t step  = (uint32_t)((((uint64_t)hz) << 16) / Fs);
    int frames_left = (Fs * ms) / 1000;
    unsigned int yield_count = 0;

    esp_err_t en = audio_enable_tx();
    if (en != ESP_OK) {
        ESP_LOGW(TAG, "tone: audio_enable_tx failed: %s", esp_err_to_name(en));
        free(st);
        return en;
    }

    while (frames_left > 0) {
        int n = frames_left > (int)CHUNK ? (int)CHUNK : frames_left;
        for (int i=0;i<n;i++) {
            float t = (phase / 65536.0f) * 2.0f * 3.14159265f;
            int16_t v16 = (int16_t)(sinf(t) * 20000.0f);
            int32_t vs  = (int32_t)((v16 * (int64_t)s_vol_q15) >> 15);
            int32_t v32 = vs << 16;
            st[2*i+0] = v32; st[2*i+1] = v32;
            phase += step;
        }
        size_t bytes = (size_t)n * 2 * sizeof(int32_t);
        size_t wr = 0, just = 0;
        int idle_retries = 0;
        while (wr < bytes) {
            size_t req = bytes - wr;
            if (req > AUDIO_TX_WRITE_SLICE_BYTES) req = AUDIO_TX_WRITE_SLICE_BYTES;
            esp_err_t e = i2s_channel_write(s_tx, ((uint8_t*)st)+wr, req, &just, 60);
            if (e == ESP_ERR_TIMEOUT) {
                // Give scheduler/idle time to run and avoid WDT when I2S stalls.
                if (++idle_retries >= 200) {
                    ESP_LOGW(TAG, "tone: i2s write timeout stall (wr=%u/%u, fs=%d)",
                             (unsigned)wr, (unsigned)bytes, Fs);
                    free(st);
                    return ESP_ERR_TIMEOUT;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            if (e != ESP_OK) { free(st); return e; }
            if (just == 0) {
                if (++idle_retries >= 200) {
                    ESP_LOGW(TAG, "tone: i2s write made no progress (wr=%u/%u, fs=%d)",
                             (unsigned)wr, (unsigned)bytes, Fs);
                    free(st);
                    return ESP_ERR_TIMEOUT;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            idle_retries = 0;
            wr += just;
        }
        frames_left -= n;
        if (++yield_count >= AUDIO_PLAY_YIELD_EVERY) {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_PLAY_YIELD_DELAY_MS));
            yield_count = 0;
        }
    }
    free(st);
    (void)audio_quiet_tail(12, 8);
    return ESP_OK;
}

// Track every local playback path, including the startup tone (which does not
// go through audio_player). FFT must not steal its clock or mute its amplifier.
esp_err_t audio_play_wav_local_stream(FILE *f, const audio_wav_info_t *info)
{ audio_output_enter(); return audio_output_leave(play_wav_local_stream(f, info)); }
esp_err_t audio_play_wav_file(const char *path)
{ audio_output_enter(); return audio_output_leave(play_wav_file(path)); }
esp_err_t audio_play_wav_mem(const void *data, size_t size)
{ audio_output_enter(); return audio_output_leave(play_wav_mem(data, size)); }
esp_err_t audio_play_tone(int hz, int ms)
{ audio_output_enter(); return audio_output_leave(play_tone(hz, ms)); }

esp_err_t audio_play_generated(audio_generator_t generate, void *user, int rate)
{
    if (!generate || rate < 8000 || rate > 48000) return ESP_ERR_INVALID_ARG;
    // Do not overlap another local output (including the startup tone).
    portENTER_CRITICAL(&s_output_mux);
    bool busy = s_output_users != 0;
    if (!busy) ++s_output_users;
    portEXIT_CRITICAL(&s_output_mux);
    if (busy) return ESP_ERR_INVALID_STATE;
    int16_t mono[256];
    int32_t stereo[512];
    esp_err_t err = audio_set_rate(rate);
    bool enabled = false;
    if (err == ESP_OK) { err = audio_enable_tx(); enabled = err == ESP_OK; }
    while (err == ESP_OK) {
        size_t n = generate(mono, 256, user);
        if (!n) break;
        if (n > 256) { err = ESP_ERR_INVALID_SIZE; break; }
        for (size_t i = 0; i < n; ++i) {
            int32_t v = (int32_t)(((int64_t)mono[i] * s_vol_q15) >> 15);
            stereo[2*i] = stereo[2*i+1] = v * 65536;
        }
        size_t offset = 0, bytes = n * 2 * sizeof(int32_t);
        while (offset < bytes && err == ESP_OK) {
            size_t written = 0;
            err = i2s_channel_write(s_tx, (uint8_t *)stereo + offset,
                                    bytes - offset, &written, 60);
            offset += written;
            if (err == ESP_OK && !written) err = ESP_ERR_TIMEOUT;
        }
    }
    if (enabled) (void)audio_quiet_tail(12, 8);
    return audio_output_leave(err);
}
