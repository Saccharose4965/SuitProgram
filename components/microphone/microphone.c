#include "microphone.h"
#include "audio.h"
#include "storage_sd.h"
#include "hw.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#ifndef MIC_REC_RATE_HZ
#define MIC_REC_RATE_HZ 44100 // match playback path to avoid resample issues
#endif

static const char *TAG = "mic_rec";

#if configNUMBER_OF_CORES > 1
#define BG_TASK_CORE 0
#else
#define BG_TASK_CORE 0
#endif

/* State */
static i2s_chan_handle_t s_rx = NULL;
static FILE  *s_wav = NULL;
static volatile bool s_rec = false;
static size_t s_bytes_data = 0;
static char   s_last_path[128] = {0};
static int    s_rate = MIC_REC_RATE_HZ;

/* WAV helpers */
static void write_wav_header(FILE *f, int fs, int bits, int ch, uint32_t data_bytes){
    uint8_t hdr[44] = {0};
    uint32_t byte_rate = fs * ch * (bits/8);
    uint16_t block_align = ch * (bits/8);
    memcpy(hdr+0, "RIFF", 4);
    uint32_t riff_sz = 36 + data_bytes; memcpy(hdr+4,  &riff_sz, 4);
    memcpy(hdr+8, "WAVE", 4);
    memcpy(hdr+12,"fmt ", 4);
    uint32_t sc1sz = 16;   memcpy(hdr+16, &sc1sz, 4);
    uint16_t pcm=1;        memcpy(hdr+20, &pcm, 2);
    uint16_t nch=1;        memcpy(hdr+22, &nch, 2);
    memcpy(hdr+24, &fs, 4);
    memcpy(hdr+28, &byte_rate, 4);
    memcpy(hdr+32, &block_align, 2);
    uint16_t bps = bits;   memcpy(hdr+34, &bps, 2);
    memcpy(hdr+36,"data",4);
    memcpy(hdr+40,&data_bytes,4);
    fseek(f, 0, SEEK_SET);
    fwrite(hdr, 1, sizeof(hdr), f);
    fflush(f);
}

static void finalize_wav(FILE *f, int fs, size_t data_bytes){
    write_wav_header(f, fs, 16, 1, (uint32_t)data_bytes);
}

/* Counter-based auto path */
static esp_err_t make_autopath(char out[128]){
    uint32_t n = 0;
    esp_err_t se = storage_boot_counter_increment(&n);
    if (se != ESP_OK) {
        n = (uint32_t)(xTaskGetTickCount() & 0xFFFF);
    }
    snprintf(out, 128, HW_SD_MOUNT_POINT "/rec_%05" PRIu32 ".wav", n);
    return ESP_OK;
}

/* Task: pull 32-bit samples from RX, scale to s16, write to SD */
static void recorder_task(void *arg){
    const size_t frames    = 256;                               // per DMA frame
    const size_t in_bytes  = frames * sizeof(int32_t) * 2;      // 32-bit L+R
    const size_t out_bytes = frames * sizeof(int16_t);          // mono s16

    int32_t *rx32  = (int32_t*)heap_caps_malloc(in_bytes,  MALLOC_CAP_8BIT);
    int16_t *pcm16 = (int16_t*)heap_caps_malloc(out_bytes, MALLOC_CAP_8BIT);
    if (!rx32 || !pcm16) {
        ESP_LOGE(TAG, "OOM");
        s_rec = false;
    }

    int  slot_index = 0;     // 0=LEFT, 1=RIGHT
    bool decided    = false; // set after first non-empty buffer
    bool warned_z   = false; // printed “zeros” warning once

    while (s_rec) {
        size_t nread = 0;
        esp_err_t err = i2s_channel_read(s_rx, rx32, in_bytes, &nread, pdMS_TO_TICKS(1000));
        if (err != ESP_OK || nread == 0) {
            static int zc = 0;
            if (nread == 0 && zc++ < 3) ESP_LOGW(TAG, "i2s read returned 0 bytes (no clocks?)");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t words32   = nread / sizeof(int32_t);   // total 32-bit words
        size_t frames_rd = words32 / 2;               // (L,R) pairs

        if (!decided && frames_rd >= 8) {
            uint64_t sumL = 0, sumR = 0;
            for (size_t i = 0; i < frames_rd; i++) {
                sumL += llabs((long long)rx32[2*i + 0]);
                sumR += llabs((long long)rx32[2*i + 1]);
            }
            slot_index = (sumR > sumL) ? 1 : 0;
            decided = true;
            ESP_LOGI(TAG, "Mic slot picked: %s", slot_index ? "RIGHT" : "LEFT");
        }

        // Convert chosen slot’s MSB-justified 32-bit to s16
        for (size_t i = 0; i < frames_rd; i++) {
            int32_t v = rx32[2*i + slot_index];
            pcm16[i]  = (int16_t)(v >> 14);           // adjust gain via shift
        }

        // Simple energy check (helps catch wiring issues)
        int32_t energy = 0;
        for (size_t i = 0; i < frames_rd; i++) energy |= pcm16[i];
        if (!energy && !warned_z) {
            ESP_LOGW(TAG, "RX is all zeros (check BCLK/WS/SD wiring, slot=%s)",
                     slot_index ? "RIGHT" : "LEFT");
            warned_z = true;
        }

        size_t wrote = fwrite(pcm16, 1, frames_rd * sizeof(int16_t), s_wav);
        s_bytes_data += wrote;

        static int flush_ctr = 0;
        if ((++flush_ctr & 0x1F) == 0) fflush(s_wav); // periodic flush
    }

    if (s_wav) {
        finalize_wav(s_wav, s_rate, s_bytes_data);
        fclose(s_wav);
        s_wav = NULL;
        ESP_LOGI(TAG, "Saved %s (%u bytes payload)", s_last_path, (unsigned)s_bytes_data);
    }

    free(rx32);
    free(pcm16);
    vTaskDelete(NULL);
}

/* Public API */
esp_err_t mic_rec_init(const mic_rec_cfg_t *cfg){
    if (cfg && cfg->sample_rate_hz > 0) s_rate = cfg->sample_rate_hz;

    s_rx = audio_rx_handle();
    if (!s_rx) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Mic ready (fs=%d)", s_rate);
    return ESP_OK;
}

esp_err_t mic_rec_start(const char *path){
    if (!s_rx) return ESP_ERR_INVALID_STATE;
    if (s_rec) return ESP_OK;

    if (!path) {
        make_autopath(s_last_path);
        path = s_last_path;
    } else {
        strncpy(s_last_path, path, sizeof(s_last_path)-1);
        s_last_path[sizeof(s_last_path)-1] = 0;
    }

    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "sd mount");

    s_wav = fopen(path, "wb+");
    if (!s_wav) { ESP_LOGE(TAG, "open %s failed", path); return ESP_FAIL; }

    uint8_t z[44] = {0};
    fwrite(z, 1, sizeof(z), s_wav);
    s_bytes_data = 0;

    /* make sure I²S is at mic rate and RX is active (TX disabled) */
    ESP_RETURN_ON_ERROR(audio_set_rate(s_rate), TAG, "set fs");
    esp_err_t e = audio_enable_rx();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "enable rx failed: %s", esp_err_to_name(e));
        fclose(s_wav);
        s_wav = NULL;
        return e;
    }

    s_rec = true;
    xTaskCreatePinnedToCore(recorder_task, "mic_rec", 4096, NULL, 5, NULL, BG_TASK_CORE);
    ESP_LOGI(TAG, "Recording -> %s", path);
    return ESP_OK;
}

esp_err_t mic_rec_stop(void){
    if (!s_rec) return ESP_OK;
    s_rec = false;
    /* recorder task will finalize/close */
    if (s_rx) i2s_channel_disable(s_rx);
    return ESP_OK;
}

bool mic_rec_is_recording(void) { return s_rec; }
const char* mic_rec_last_path(void) { return s_last_path[0] ? s_last_path : NULL; }
