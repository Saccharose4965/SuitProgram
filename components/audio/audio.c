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
#include "esp_check.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

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
static int                 s_curr_rate = 48000; // default boot rate
static int32_t             s_vol_q15   = 32767; // 1.0 in Q1.15
static bool                s_tx_enabled = false;
static bool                s_rx_enabled = false;

// Yield occasionally during long streaming loops to keep UI responsive.
#define AUDIO_PLAY_YIELD_EVERY    4
#define AUDIO_PLAY_YIELD_DELAY_MS 1
// Keep TX writes <= one DMA frame worth of bytes to avoid all-or-nothing stalls.
#define AUDIO_TX_WRITE_SLICE_BYTES 2048

// Small static zero blocks to avoid stack usage
static int32_t s_zero256_stereo[256 * 2] = {0};
static int32_t s_zero512_stereo[512 * 2] = {0};

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

esp_err_t audio_init(const audio_i2s_config_t* cfg){
    if (!cfg) return ESP_ERR_INVALID_ARG;
    g_last_cfg  = *cfg;
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

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &tx_cfg), TAG, "init tx");
    if (cfg->din_gpio >= 0) {
        ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &rx_cfg), TAG, "init rx");
    } else {
        s_rx = NULL;
    }

    ESP_LOGI(TAG, "I2S ready: fs=%d, BCLK=%d, LRCK=%d, DOUT=%d, DIN=%d",
             s_curr_rate, cfg->bclk_gpio, cfg->lrck_gpio, cfg->dout_gpio, cfg->din_gpio);
    return ESP_OK;
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
    s_curr_rate = hz;
    ESP_RETURN_ON_ERROR(audio_reconfig_clock(s_tx, hz), TAG, "tx re-clock");
    ESP_RETURN_ON_ERROR(audio_reconfig_clock(s_rx, hz), TAG, "rx re-clock");
    return ESP_OK;
}

esp_err_t audio_enable_tx(void){
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
// Supports mono or stereo PCM16. For the embedded speaker we always
// output mono: stereo is downmixed and duplicated to L/R I2S.
//
// ====================== Local playback using parsed WAV header ======================
//
// Uses audio_wav_info_t (parsed by audio_wav_parse_file) to play mono/stereo
// PCM16 data via the mono I2S speaker. Stereo is mixed down to mono.
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

// Local WAV playback using I2S speaker (mono or stereo PCM16)
esp_err_t audio_play_wav_local_stream(FILE *f, const audio_wav_info_t *info)
{
    if (!s_tx || !f || !info) {
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

    int16_t *in  = (int16_t *)malloc(IN_SAMPLES * sizeof(int16_t));           // input PCM16
    int32_t *out = (int32_t *)malloc(FRAMES_PER_CHUNK * 2 * sizeof(int32_t)); // stereo s32
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

        // Convert bytes → samples → frames
        size_t samples = rd / sizeof(int16_t);              // PCM16 samples read
        size_t frames  = samples / fmt->num_channels;       // audio frames

        if (frames == 0) {
            break;
        }

        // Build stereo s32 from mono/stereo PCM16
        for (size_t i = 0; i < frames; ++i) {
            int32_t s;

            if (fmt->num_channels == 1) {
                // Mono file → duplicate to L/R
                s = in[i];
            } else {
                // Stereo file: mix L+R to mono for the single physical speaker
                int32_t l = in[2 * i + 0];
                int32_t r = in[2 * i + 1];
                s = (l + r) / 2;
            }

            // Apply volume (Q15), then convert s16 → s32 MSB-justified
            int32_t vs  = (int32_t)((s * (int64_t)s_vol_q15) >> 15);
            int32_t v32 = vs << 16;

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

            if (e == ESP_ERR_TIMEOUT) {
                // Retry instead of dropping samples; avoid underruns that speed playback up.
                continue;
            }
            if (e != ESP_OK) {
                ESP_LOGE(TAG, "local_stream: i2s write %s", esp_err_to_name(e));
                free(in);
                free(out);
                return e;
            }
            if (just == 0) {
                // No progress – bail out of loop
                break;
            }
            written += just;
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


esp_err_t audio_play_wav_file(const char* path){
    if (!s_tx || !path) return ESP_ERR_INVALID_STATE;

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

    if (info.fmt.num_channels != 1 && info.fmt.num_channels != 2) {
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

    const int   channels  = info.fmt.num_channels;
    const size_t FRAMES   = 1024; // frames per chunk
    int16_t *in  = (int16_t*)malloc(FRAMES * channels * sizeof(int16_t));
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
        size_t bytes_to_read  = frames_to_read * channels * sizeof(int16_t);
        if (bytes_left < bytes_to_read) {
            frames_to_read = bytes_left / (channels * sizeof(int16_t));
            bytes_to_read  = frames_to_read * channels * sizeof(int16_t);
        }
        if (frames_to_read == 0) break;

        size_t rd = fread_retry(in, 1, bytes_to_read, f);
        if (rd == 0) break;

        size_t frames = rd / (channels * sizeof(int16_t));

        if (channels == 1) {
            // Mono in → mono out (duplicated into both I2S slots)
            for (size_t i = 0; i < frames; ++i) {
                int32_t v  = in[i];
                int32_t vs = (int32_t)((v * (int64_t)s_vol_q15) >> 15); // s16
                int32_t v32 = vs << 16;                                 // s16->s32 MSB
                out[2*i + 0] = v32;
                out[2*i + 1] = v32;
            }
        } else { // channels == 2
            // Stereo in → downmix to mono: (L+R)/2, then duplicate to L/R
            int16_t *st = in;
            for (size_t i = 0; i < frames; ++i) {
                int32_t L  = st[2*i + 0];
                int32_t R  = st[2*i + 1];
                int32_t m  = (L + R) / 2; // simple average
                int32_t vs = (int32_t)((m * (int64_t)s_vol_q15) >> 15);
                int32_t v32 = vs << 16;
                out[2*i + 0] = v32;
                out[2*i + 1] = v32;
            }
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
esp_err_t audio_play_wav_mem(const void* data, size_t size){
    if (!s_tx || !data || size < sizeof(riff_t)) return ESP_ERR_INVALID_ARG;

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

esp_err_t audio_play_tone(int hz, int ms){
    if (!s_tx) return ESP_ERR_INVALID_STATE;
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
