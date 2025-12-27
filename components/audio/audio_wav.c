// Shared WAV header parser (PCM16 mono/stereo)
#include "audio_wav.h"

#include <string.h>
#include "esp_log.h"

typedef struct __attribute__((packed)) {
    char     riff[4];
    uint32_t riff_size;
    char     wave[4];
} riff_t;

typedef struct __attribute__((packed)) {
    char     id[4];
    uint32_t size;
} chunk_hdr_t;

static inline uint32_t fcc4(const char s[4]){
    return ((uint32_t)(uint8_t)s[0]) |
           ((uint32_t)(uint8_t)s[1] << 8) |
           ((uint32_t)(uint8_t)s[2] << 16) |
           ((uint32_t)(uint8_t)s[3] << 24);
}

#define FCC(a,b,c,d) ( ((uint32_t)(a)) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24) )

static const char *TAG = "audio_wav";

esp_err_t audio_wav_parse_file(FILE *f, audio_wav_info_t *out_info){
    if (!f || !out_info) return ESP_ERR_INVALID_ARG;

    riff_t rh;
    if (fread(&rh, 1, sizeof(rh), f) != sizeof(rh) ||
        memcmp(rh.riff, "RIFF", 4) != 0 ||
        memcmp(rh.wave, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Not a RIFF/WAVE file");
        return ESP_ERR_INVALID_RESPONSE;
    }

    audio_wav_fmt_t fmt = (audio_wav_fmt_t){0};
    long data_pos = -1;
    uint32_t data_size = 0;

    for (;;) {
        chunk_hdr_t ch;
        if (fread(&ch, 1, sizeof(ch), f) != sizeof(ch)) {
            ESP_LOGE(TAG, "Failed to read chunk header");
            return ESP_ERR_INVALID_RESPONSE;
        }
        uint32_t id = fcc4(ch.id);

        if (id == FCC('f','m','t',' ')) {
            size_t n = ch.size < sizeof(fmt) ? ch.size : sizeof(fmt);
            if (fread(&fmt, 1, n, f) != n) {
                ESP_LOGE(TAG, "Failed to read fmt chunk");
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (ch.size > n) {
                fseek(f, (long)(ch.size - n), SEEK_CUR);
            }
        } else if (id == FCC('d','a','t','a')) {
            data_pos  = ftell(f);
            data_size = ch.size;
            break; // first data chunk
        } else {
            fseek(f, (long)ch.size, SEEK_CUR);
        }

        if (ch.size & 1) {
            fseek(f, 1, SEEK_CUR); // pad
        }
    }

    if (data_pos < 0 || data_size == 0) {
        ESP_LOGE(TAG, "No data chunk found");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (fmt.audio_format != 1 || fmt.bits_per_sample != 16 ||
        (fmt.num_channels != 1 && fmt.num_channels != 2)) {
        ESP_LOGW(TAG, "Unsupported WAV format: fmt=%u ch=%u bits=%u rate=%u",
                 fmt.audio_format, fmt.num_channels,
                 fmt.bits_per_sample, fmt.sample_rate);
        return ESP_ERR_NOT_SUPPORTED;
    }

    out_info->data_offset = (uint32_t)data_pos;
    out_info->data_size   = data_size;
    out_info->fmt         = fmt;
    return ESP_OK;
}
