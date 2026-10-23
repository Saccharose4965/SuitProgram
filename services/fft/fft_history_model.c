#include "fft_history.h"

#include <math.h>
#include <string.h>

static float clamp(float value, float maximum)
{
    return isfinite(value) ? fminf(maximum, fmaxf(0, value)) : 0;
}

void fft_history_model_reset(fft_history_snapshot_t *history)
{
    if (history) memset(history, 0, sizeof(*history));
}

void fft_history_model_push(fft_history_snapshot_t *history,
                            const fft_history_packet_t *packet)
{
    if (!history || !packet) return;
    float peak = 0;
    for (int row = 0; row < FFT_HISTORY_ROWS; ++row)
        peak = fmaxf(peak, clamp(packet->magnitudes[row], 1e6f));
    float threshold = fmaxf(.01f, peak * .01f);
    for (int row = 0; row < FFT_HISTORY_ROWS; ++row) {
        if (isfinite(packet->magnitudes[row]) &&
            packet->magnitudes[row] >= threshold)
            history->pending[row] = 255;
    }
    history->latest = *packet;
    if (++history->pending_frames < FFT_HISTORY_FRAMES_PER_COLUMN) return;

    unsigned slot = history->write;
    memcpy(history->spectrogram[slot], history->pending,
           sizeof(history->pending));
    memset(history->pending, 0, sizeof(history->pending));
    history->pending_frames = 0;
    history->confidence[slot] = clamp(packet->decision, 1.1f);
    history->evidence[slot] = clamp(packet->status.evidence, 1.1f);
    history->write = (slot + 1) % FFT_HISTORY_COLUMNS;
    if (history->count < FFT_HISTORY_COLUMNS) ++history->count;
}
