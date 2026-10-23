#pragma once

#include <stdint.h>
#include <stdio.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t audio_format;    // 1 = PCM
    uint16_t num_channels;    // 1 or 2 supported
    uint32_t sample_rate;     // Hz
    uint32_t byte_rate;       // bytes/sec
    uint16_t block_align;
    uint16_t bits_per_sample; // 16 supported
} audio_wav_fmt_t;

typedef struct {
    uint32_t        data_offset; // byte offset to first data byte
    uint32_t        data_size;   // bytes of audio payload
    audio_wav_fmt_t fmt;         // parsed fmt chunk
} audio_wav_info_t;

// Parse a minimal WAV header (PCM16 mono/stereo). Returns:
//  - ESP_OK on success, filling out_info
//  - ESP_ERR_INVALID_RESPONSE if not a RIFF/WAVE or chunks missing
//  - ESP_ERR_NOT_SUPPORTED for formats other than PCM16 mono/stereo
//  - ESP_ERR_INVALID_ARG on bad parameters
esp_err_t audio_wav_parse_file(FILE *f, audio_wav_info_t *out_info);

#ifdef __cplusplus
}
#endif
