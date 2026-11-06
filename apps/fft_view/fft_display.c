#include "fft_display.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define W FFT_DISPLAY_WIDTH
#define H FFT_DISPLAY_HEIGHT
static float clamp(float x, float max) { return isfinite(x) ? fminf(max, fmaxf(0, x)) : 0; }
static float wrap(float x) { return x - floorf(x); }

float fft_display_relative_phase(float absolute, float origin, unsigned folds)
{
    return wrap((absolute - origin) * (folds ? folds : 1));
}

static void pixel(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= W || (unsigned)y >= H) return;
    int bit = y * W + x;
    fb[bit / 8] |= 1u << (7 - bit % 8);
}

static void vertical(uint8_t *fb, int x, int top, int bottom, int stride)
{
    for (int y = top; y <= bottom; y += stride) pixel(fb, x, y);
}

static void line(uint8_t *fb, int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0, dy = y1 - y0;
    int steps = dx > abs(dy) ? dx : abs(dy);
    if (!steps) { pixel(fb, x0, y0); return; }
    for (int i = 0; i <= steps; ++i) pixel(fb, x0 + dx * i / steps, y0 + dy * i / steps);
}

static int history_slot(const fft_history_snapshot_t *d, int column)
{
    // One stored column per screen column; both histories share the same groups.
    if (column < FFT_HISTORY_COLUMNS - (int)d->count) return -1;
    return (d->write + column) % FFT_HISTORY_COLUMNS;
}

static void spectrogram(const fft_history_snapshot_t *d, uint8_t *fb, int x, int y, int w, int h)
{
    for (int col = 0; col < w; ++col) {
        int slot = history_slot(d, col * FFT_HISTORY_COLUMNS / w);
        if (slot < 0) continue;
        for (int row = 0; row < h; ++row) {
            if (d->spectrogram[slot][row * FFT_HISTORY_ROWS / h])
                pixel(fb, x + col, y + h - 1 - row);
        }
    }
}

static void confidence(const fft_history_snapshot_t *d, uint8_t *fb, int x, int y, int w, int h)
{
    int previous = -1, previous_evidence = -1;
    for (int col = 0; col < w; ++col) {
        int t = (col + 1) * FFT_HISTORY_COLUMNS / w - 1;
        int slot = history_slot(d, t);
        if (slot < 0) continue;
        int cy = y + h - 1 - (int)lroundf(d->confidence[slot] / 1.1f * (h - 1));
        int ey = y + h - 1 - (int)lroundf(d->evidence[slot] / 1.1f * (h - 1));
        if (previous >= 0) line(fb, x + col - 1, previous, x + col, cy);
        else pixel(fb, x + col, cy);
        if (previous_evidence >= 0) line(fb, x + col - 1, previous_evidence, x + col, ey);
        else pixel(fb, x + col, ey);
        previous = cy;
        previous_evidence = ey;
    }
}

static void tempo(const fft_history_snapshot_t *d, uint8_t *fb, int x, int y, int w, int h)
{
    float scale = 1e-9f;
    for (int i = 0; i < DETECTOR_BPM_COUNT; ++i) scale = fmaxf(scale, d->detector.scores[i]);
    int count = w < DETECTOR_BPM_COUNT ? w : DETECTOR_BPM_COUNT;
    int left = x + (w - count) / 2;
    for (int col = 0; col < count; ++col) {
        float value = d->detector.scores[col];
        int height = (int)lroundf(clamp(value / scale, 1) * h);
        if (height) vertical(fb, left + col, y + h - height, y + h - 1, 1);
    }
}

static void phase_band(const float *scores, int count, float origin, unsigned folds,
                       uint8_t *fb, int x, int y, int w, int h)
{
    float low = scores[0], high = low;
    for (int i = 1; i < count; ++i) {
        low = fminf(low, scores[i]);
        high = fmaxf(high, scores[i]);
    }
    for (int col = 0; col < w; ++col) {
        float relative = (float)col / w;
        float absolute = wrap(origin + relative / folds);
        int bin = (int)floorf(absolute * DETECTOR_PHASE_BINS + 1e-4f) % count;
        float level = high > low ? (scores[bin] - low) / (high - low) : 0;
        int height = (int)lroundf(clamp(level, 1) * h);
        if (height) vertical(fb, x + col, y + h - height, y + h - 1, 1);
    }
}

static void phase(const fft_history_snapshot_t *d, uint8_t *fb, int x, int y, int w, int h)
{
    if (!d->detector.phase_valid || d->latest.status.bpm <= 0 || h < 2) return;

    if (d->latest.status.clock_base_bpm > 0 && d->detector.held_bpm > 0 &&
        fabsf(d->latest.status.clock_base_bpm - d->detector.held_bpm) >= .5f) return;

    float folded[DETECTOR_PHASE_BINS / 2];
    for (int i = 0; i < DETECTOR_PHASE_BINS / 2; ++i)
        folded[i] = d->detector.phase[i] + d->detector.phase[i + DETECTOR_PHASE_BINS / 2];

    int top = h / 2;
    float origin = d->latest.clock_origin;
    phase_band(folded, DETECTOR_PHASE_BINS / 2, origin, 2, fb, x, y, w, top);
    phase_band(d->detector.phase, DETECTOR_PHASE_BINS, origin, 1,
               fb, x, y + top, w, h - top);
}

void fft_display_draw(const fft_history_snapshot_t *d, fft_view_t view,
                      uint8_t fb[FFT_DISPLAY_BYTES])
{
    memset(fb, 0, FFT_DISPLAY_BYTES);
    switch (view) {
    case FFT_VIEW_SPECTROGRAM: spectrogram(d, fb, 0, 0, W, H); break;
    case FFT_VIEW_CONFIDENCE: confidence(d, fb, 0, 0, W, H); break;
    case FFT_VIEW_TEMPO_RAW: tempo(d, fb, 0, 0, W, H); break;
    case FFT_VIEW_PHASE_COMB: phase(d, fb, 0, 0, W, H); break;
    case FFT_VIEW_SPECTRUM: {
        float peak = 1;
        for (int i = 0; i < FFT_CFG_SIZE / 2; ++i) peak = fmaxf(peak, d->latest.logmag[i]);
        for (int x = 0; x < W; ++x) {
            float sum = 0;
            for (int k = x * 4; k < x * 4 + 4; ++k) sum += d->latest.logmag[k];
            int height = (int)lroundf(clamp(sum / (4 * peak), 1) * H);
            if (height) vertical(fb, x, H - height, H - 1, 1);
        }
        break;
    }
    default: break;
    }
}
