#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "fft.h"
#include "fft_detector.h"

#define FFT_HISTORY_COLUMNS 128
#define FFT_HISTORY_FRAMES_PER_COLUMN 2
#define FFT_HISTORY_ROWS 32

typedef struct {
    float logmag[FFT_CFG_SIZE / 2 + 1];
    float magnitudes[FFT_HISTORY_ROWS];
    fft_sync_state_t status;
    float decision;
    float clock_origin;
    double time;
} fft_history_packet_t;

typedef struct {
    uint8_t spectrogram[FFT_HISTORY_COLUMNS][FFT_HISTORY_ROWS];
    float confidence[FFT_HISTORY_COLUMNS];
    float evidence[FFT_HISTORY_COLUMNS];
    unsigned write;
    unsigned count;
    unsigned pending_frames;
    uint8_t pending[FFT_HISTORY_ROWS];
    fft_history_packet_t latest;
    detector_result_t detector;
} fft_history_snapshot_t;

void fft_history_model_reset(fft_history_snapshot_t *history);
void fft_history_model_push(fft_history_snapshot_t *history,
                            const fft_history_packet_t *packet);
esp_err_t fft_history_init(void);
void fft_history_reset(void);
void fft_history_update_detector(const detector_result_t *result);
void fft_history_submit(const fft_history_packet_t *packet);
uint32_t fft_history_snapshot(fft_history_snapshot_t *out);
