#include "bt_audio.h"
#include "bt_audio_internal.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"

static const char *TAG = "bt_audio";

// ============================ File-backed source ============================
static FILE    *s_bt_fp              = NULL;
static uint32_t s_bt_bytes_left      = 0;
static uint32_t s_bt_bytes_total     = 0;
static uint16_t s_bt_channels        = 2;
static SemaphoreHandle_t s_bt_fp_lock = NULL; // protects FILE access across callback/control paths

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

static void close_bt_file_locked(void)
{
    if (s_bt_fp) {
        fclose(s_bt_fp);
        s_bt_fp = NULL;
    }
    s_bt_bytes_left = 0;
    s_bt_bytes_total = 0;
}

static void close_bt_file(void)
{
    if (lock_fp(pdMS_TO_TICKS(100))) {
        close_bt_file_locked();
        unlock_fp();
    } else {
        s_bt_bytes_left = 0;
        ESP_LOGW(TAG, "Failed to lock BT file for close");
    }
}

void bt_audio_stream_close(void)
{
    close_bt_file();
}

size_t bt_audio_stream_queued_frames(void)
{
    size_t bytes_per_frame = (s_bt_channels == 2) ? 4 : 2;
    return bytes_per_frame ? (s_bt_bytes_left / bytes_per_frame) : 0;
}

int32_t bt_audio_a2dp_data_cb(uint8_t *data, int32_t len)
{
    if (!data || len <= 0) return 0;

    if (!lock_fp(pdMS_TO_TICKS(10))) {
        memset(data, 0, len);
        return len;
    }

    if (!s_bt_fp || s_bt_bytes_left == 0) {
        memset(data, 0, len);
        unlock_fp();
        return len;
    }

    // Stereo fast path (16-bit)
    if (s_bt_channels == 2) {
        size_t to_read = (size_t)len;
        if (to_read > s_bt_bytes_left) to_read = s_bt_bytes_left;
        size_t rd = fread(data, 1, to_read, s_bt_fp);
        if (rd == 0) {
            memset(data, 0, len);
        } else {
            if (rd < (size_t)len) {
                memset(data + rd, 0, (size_t)len - rd);
            }
            if (s_bt_bytes_left >= rd) s_bt_bytes_left -= (uint32_t)rd;
            else s_bt_bytes_left = 0;
        }
    } else { // mono -> duplicate to stereo
        size_t frames_req = (size_t)len / 4; // stereo frames requested
        uint8_t mono_buf[1024];
        size_t copied = 0;
        uint8_t *out_bytes = data;

        while (copied < (size_t)len) {
            size_t chunk_frames = frames_req;
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

            size_t rd = fread(mono_buf, 1, chunk_mono_bytes, s_bt_fp);
            if (rd == 0) {
                break;
            }
            s_bt_bytes_left -= (uint32_t)rd;
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

    if (s_bt_bytes_left == 0 && s_bt_fp) {
        close_bt_file_locked();
    }

    unlock_fp();
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
        return ESP_FAIL;
    }

    s_bt_fp             = fp;
    s_bt_bytes_left     = data_size;
    s_bt_bytes_total    = data_size;
    s_bt_channels       = num_channels;

    setvbuf(fp, NULL, _IOFBF, 32 * 1024);

    esp_err_t err = bt_audio_start(false);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        s_bt_fp = NULL;
        s_bt_bytes_left = 0;
        return err;
    }

    err = bt_audio_start_stream();
    if (err != ESP_OK) {
        s_bt_fp = NULL;
        s_bt_bytes_left = 0;
        return err;
    }

    ESP_LOGI(TAG, "BT WAV playback started: %u bytes @ %u Hz, ch=%u",
             data_size, sample_rate, num_channels);
    return ESP_OK;
}

bool bt_audio_get_playback_progress(uint32_t *out_total_bytes, uint32_t *out_bytes_left)
{
    if (!out_total_bytes || !out_bytes_left) return false;
    if (!s_bt_fp || s_bt_bytes_total == 0) {
        *out_total_bytes = 0;
        *out_bytes_left = 0;
        return false;
    }
    *out_total_bytes = s_bt_bytes_total;
    *out_bytes_left = s_bt_bytes_left;
    return true;
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
