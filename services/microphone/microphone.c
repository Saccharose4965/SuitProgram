#include "microphone.h"
#include "audio_i2s.h"
#include "storage_sd.h"
#include "recording_counter.h"
#include "hw.h"
#include "recording_envelope.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

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
static char   s_partial_path[136] = {0};
static int    s_rate = MIC_REC_RATE_HZ;
static TaskHandle_t s_rec_task = NULL;
static portMUX_TYPE s_envelope_mux=portMUX_INITIALIZER_UNLOCKED;
static recording_envelope_t s_envelope;
size_t mic_rec_envelope(uint8_t *out,size_t capacity)
{
    if(!out)return 0;
    portENTER_CRITICAL(&s_envelope_mux);
    size_t n=recording_envelope_copy(&s_envelope,out,capacity);
    portEXIT_CRITICAL(&s_envelope_mux);
    return n;
}
static void publish_envelope(const recording_envelope_t *e)
{
    portENTER_CRITICAL(&s_envelope_mux);
    s_envelope=*e;
    portEXIT_CRITICAL(&s_envelope_mux);
}

/* WAV helpers */
static esp_err_t write_wav_header(FILE *f, int fs, int bits, int ch, uint32_t data_bytes){
    if (!f) return ESP_ERR_INVALID_ARG;
    uint8_t hdr[44] = {0};
    uint32_t byte_rate = fs * ch * (bits/8);
    uint16_t block_align = ch * (bits/8);
    memcpy(hdr+0, "RIFF", 4);
    uint32_t riff_sz = 36 + data_bytes; memcpy(hdr+4,  &riff_sz, 4);
    memcpy(hdr+8, "WAVE", 4);
    memcpy(hdr+12,"fmt ", 4);
    uint32_t sc1sz = 16;   memcpy(hdr+16, &sc1sz, 4);
    uint16_t pcm=1;        memcpy(hdr+20, &pcm, 2);
    uint16_t nch=(uint16_t)ch; memcpy(hdr+22, &nch, 2);
    memcpy(hdr+24, &fs, 4);
    memcpy(hdr+28, &byte_rate, 4);
    memcpy(hdr+32, &block_align, 2);
    uint16_t bps = bits;   memcpy(hdr+34, &bps, 2);
    memcpy(hdr+36,"data",4);
    memcpy(hdr+40,&data_bytes,4);
    if (fseek(f, 0, SEEK_SET) != 0) return ESP_FAIL;
    if (fwrite(hdr, 1, sizeof(hdr), f) != sizeof(hdr)) return ESP_FAIL;
    if (fflush(f) != 0) return ESP_FAIL;
    int fd = fileno(f);
    return fd >= 0 && fsync(fd) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t finalize_wav(FILE *f, int fs, size_t data_bytes){
    if (data_bytes > UINT32_MAX) return ESP_ERR_INVALID_SIZE;
    return write_wav_header(f, fs, 16, 1, (uint32_t)data_bytes);
}

static void discard_partial(void)
{
    if (s_wav) {
        (void)fclose(s_wav);
        s_wav = NULL;
    }
    if (s_partial_path[0]) (void)unlink(s_partial_path);
    s_partial_path[0] = '\0';
}

static esp_err_t open_exclusive_recording(const char *requested_path)
{
    for (unsigned attempt = 0; attempt < 1024; ++attempt) {
        char final_path[sizeof(s_last_path)];
        if (requested_path) {
            int n = snprintf(final_path, sizeof(final_path), "%s", requested_path);
            if (n <= 0 || (size_t)n >= sizeof(final_path)) return ESP_ERR_INVALID_SIZE;
        } else {
            uint32_t number = 0;
            ESP_RETURN_ON_ERROR(recording_counter_next(&number), TAG, "recording counter");
            int n = snprintf(final_path, sizeof(final_path),
                             HW_SD_MOUNT_POINT "/rec_%05" PRIu32 ".wav", number);
            if (n <= 0 || (size_t)n >= sizeof(final_path)) return ESP_ERR_INVALID_SIZE;
        }

        char partial_path[sizeof(s_partial_path)];
        int n = snprintf(partial_path, sizeof(partial_path), "%s.part", final_path);
        if (n <= 0 || (size_t)n >= sizeof(partial_path)) return ESP_ERR_INVALID_SIZE;
        if (access(final_path, F_OK) == 0) {
            if (requested_path) return ESP_ERR_INVALID_STATE;
            continue;
        }

        int fd = open(partial_path, O_CREAT | O_EXCL | O_RDWR, 0664);
        if (fd < 0) {
            if (errno == EEXIST && !requested_path) continue;
            return errno == EEXIST ? ESP_ERR_INVALID_STATE : ESP_FAIL;
        }
        s_wav = fdopen(fd, "wb+");
        if (s_wav) {
            memcpy(s_last_path, final_path, sizeof(s_last_path));
            memcpy(s_partial_path, partial_path, sizeof(s_partial_path));
            return ESP_OK;
        }
        close(fd);
        (void)unlink(partial_path);
        return ESP_FAIL;
    }
    return ESP_ERR_NOT_FOUND;
}

/* Task: pull 32-bit samples from RX, scale to s16, write to SD */
static void recorder_task(void *arg){
    (void)arg;
    recording_envelope_t envelope={.window=(unsigned)s_rate/20};
    publish_envelope(&envelope);
    const size_t frames    = 256;                               // per DMA frame
    const size_t in_bytes  = frames * sizeof(int32_t) * 2;      // 32-bit L+R
    const size_t out_bytes = frames * sizeof(int16_t);          // mono s16

    int32_t *rx32  = (int32_t*)heap_caps_malloc(in_bytes,  MALLOC_CAP_8BIT);
    int16_t *pcm16 = (int16_t*)heap_caps_malloc(out_bytes, MALLOC_CAP_8BIT);
    if (!rx32 || !pcm16) {
        ESP_LOGE(TAG, "OOM");
        s_rec = false;
        free(rx32);
        free(pcm16);
        discard_partial();
        audio_i2s_disable_all();
        s_rec_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    const int slot_index = 0; // 0=LEFT
    bool warned_z = false; // printed “zeros” warning once
    unsigned flush_ctr = 0;

    ESP_LOGI(TAG, "Mic slot fixed: LEFT");

    while (s_rec) {
        size_t nread = 0;
        esp_err_t err = i2s_channel_read(s_rx, rx32, in_bytes, &nread, 1000);
        if (err != ESP_OK || nread == 0) {
            static int zc = 0;
            if (nread == 0 && zc++ < 3) ESP_LOGW(TAG, "i2s read returned 0 bytes (no clocks?)");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t words32   = nread / sizeof(int32_t);   // total 32-bit words
        size_t frames_rd = words32 / 2;               // (L,R) pairs

        // Convert the left slot's MSB-justified 32-bit samples to s16.
        for (size_t i = 0; i < frames_rd; i++) {
            int32_t v = rx32[2*i + slot_index];
            int32_t scaled = v / 16384;
            pcm16[i] = scaled > 32767 ? 32767 : scaled < -32768 ? -32768 : scaled;
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
        recording_envelope_feed(&envelope,pcm16,wrote/sizeof(int16_t));
        publish_envelope(&envelope);
        if(wrote != frames_rd * sizeof(int16_t)) {
            ESP_LOGE(TAG, "SD write failed; stopping recording");
            s_rec=false;
        }

        if ((++flush_ctr & 0x1F) == 0) fflush(s_wav); // periodic flush
    }

    if(envelope.frames)recording_envelope_push(&envelope);
    publish_envelope(&envelope);
    if (s_wav) {
        bool saved = finalize_wav(s_wav, s_rate, s_bytes_data) == ESP_OK;
        if (fclose(s_wav) != 0) saved = false;
        s_wav = NULL;
        if (saved && rename(s_partial_path, s_last_path) == 0) {
            ESP_LOGI(TAG, "Saved %s (%u bytes payload)", s_last_path, (unsigned)s_bytes_data);
            s_partial_path[0] = '\0';
        } else {
            ESP_LOGE(TAG, "Recording finalization failed; partial kept at %s", s_partial_path);
        }
    }

    free(rx32);
    free(pcm16);
    audio_i2s_disable_all();
    s_rec_task = NULL;
    vTaskDelete(NULL);
}

/* Public API */
esp_err_t mic_rec_init(const mic_rec_cfg_t *cfg){
    if (cfg && cfg->sample_rate_hz > 0) s_rate = cfg->sample_rate_hz;

    s_rx = audio_i2s_rx_handle();
    if (!s_rx) {
        discard_partial();
        audio_i2s_disable_all();
        return ESP_ERR_INVALID_STATE;
    }
    // Idle Bluetooth may have released I2S; mic_rec_start restores it via
    // audio_i2s_set_rate and refreshes this handle before creating the worker.

    ESP_LOGI(TAG, "Mic ready (fs=%d)", s_rate);
    return ESP_OK;
}

esp_err_t mic_rec_start(const char *path){
    if (s_rec) return ESP_OK;
    if (s_rec_task) return ESP_ERR_INVALID_STATE; // previous WAV still finalizing

    ESP_RETURN_ON_ERROR(storage_mount_sd(), TAG, "sd mount");
    esp_err_t e = open_exclusive_recording(path);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "create recording failed: %s", esp_err_to_name(e));
        return e;
    }
    e = write_wav_header(s_wav, s_rate, 16, 1, 0);
    if (e != ESP_OK) {
        discard_partial();
        return e;
    }
    s_bytes_data = 0;

    /* make sure I²S is at mic rate and RX is active (TX disabled) */
    e = audio_i2s_set_rate(s_rate);
    if (e == ESP_OK) e = audio_i2s_enable_rx();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "enable rx failed: %s", esp_err_to_name(e));
        discard_partial();
        audio_i2s_disable_all();
        return e;
    }
    // Bluetooth may have released/recreated the I2S bus since mic_rec_init.
    s_rx = audio_i2s_rx_handle();

    recording_envelope_t empty={0};
    publish_envelope(&empty);
    s_rec = true;
    BaseType_t ok = xTaskCreatePinnedToCore(recorder_task, "mic_rec", 4096, NULL, 5, &s_rec_task, BG_TASK_CORE);
    if (ok != pdPASS) {
        s_rec = false;
        discard_partial();
        audio_i2s_disable_all();
        s_rec_task = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Recording -> %s", s_last_path);
    return ESP_OK;
}

esp_err_t mic_rec_stop(void){
    if (!s_rec) return ESP_OK;
    s_rec = false;
    audio_i2s_disable_all(); // keep the shared bus's enabled-state bookkeeping correct
    for (int i = 0; i < 100 && s_rec_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_OK;
}

bool mic_rec_is_recording(void) { return s_rec || s_rec_task != NULL; }
const char* mic_rec_last_path(void) { return s_last_path[0] ? s_last_path : NULL; }
