#include "fft_display.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static fft_display_t display, before;
static fft_display_packet_t packet;
static int pixels(const uint8_t *fb, int x0, int y0, int x1, int y1)
{
    int n = 0;
    for (int y = y0; y < y1; ++y) for (int x = x0; x < x1; ++x) {
        int bit = y * FFT_DISPLAY_WIDTH + x;
        n += (fb[bit / 8] >> (7 - bit % 8)) & 1;
    }
    return n;
}
static void draw(fft_view_t view, uint8_t *fb)
{
    fft_display_draw(&display, view, fb);
}
int main(void)
{
    struct { uint8_t left[16], fb[FFT_DISPLAY_BYTES], right[16]; } guarded;
    memset(&guarded, 0xa5, sizeof(guarded));
    fft_display_reset(&display);
    draw(FFT_VIEW_SPECTRUM, guarded.fb);
    assert(!pixels(guarded.fb, 0, 0, 128, 64));
    packet.status = (fft_sync_state_t){.held_bpm=120, .detected_bpm=122, .bpm=240,
        .ratio=2, .running=true, .beat_enabled=true, .beat_phase=.25f,
        .trigger_phase=.5f, .evidence=1.05f, .lost_seconds=12.5f};
    packet.decision = 1.08f; packet.time = 10; packet.clock_origin = .17f;
    // Energy only above 11 kHz must reach the top of the graph. The old view
    // accidentally covered only the lower 256 FFT bins (half the frequency range).
    for (int i = 16; i < FFT_DISPLAY_ROWS; ++i) packet.magnitudes[i] = 50;
    for (int i = 0; i < 340; ++i) fft_display_push(&display, &packet);
    assert(display.count == 320 && display.write == 20);
    assert(display.confidence[19] == 1.08f && display.evidence[19] == 1.05f);
    assert(display.spectrogram[19][31] == 255 && display.spectrogram[19][0] == 0);
    draw(FFT_VIEW_SPECTROGRAM, guarded.fb);
    assert(pixels(guarded.fb, 0, 0, 128, 32) == 4096);
    assert(pixels(guarded.fb, 0, 32, 128, 64) == 0);
    // An impulse in the confidence timeline and spectrogram must occupy the
    // same rightmost history slot, with older data retained on rollover.
    packet.decision = .2f; packet.status.evidence = .3f;
    memset(packet.magnitudes, 0, sizeof(packet.magnitudes));
    fft_display_push(&display, &packet);
    assert(display.confidence[20] == .2f && display.evidence[20] == .3f);
    assert(display.spectrogram[20][31] == 0 && display.spectrogram[19][31] == 255);
    display.detector.phase_valid = true;
    display.detector.candidate_phase = .67f; display.detector.accepted_phase = .17f;
    display.detector.scores[40] = .4f;
    for (int i = 0; i < DETECTOR_PHASE_BINS; ++i) display.detector.phase[i] = 2 + sinf(i * .1f);
    before = display;
    for (int view = 0; view < FFT_VIEW_COUNT; ++view) {
        draw((fft_view_t)view, guarded.fb);
        assert(pixels(guarded.fb, 0, 0, 128, 64) > 0 || view == FFT_VIEW_SPECTRUM);
        assert(memcmp(&before, &display, sizeof(display)) == 0); // drawing never advances history
    }
    for (int i = 0; i < 16; ++i) assert(guarded.left[i] == 0xa5 && guarded.right[i] == 0xa5);
    assert(fabsf(fft_display_clock_phase(.67f, .17f, 1) - .5f) < 1e-6f);
    assert(fabsf(fft_display_clock_phase(.67f, .17f, 2)) < 1e-6f);
    assert(fabsf(fft_display_clock_phase(.67f, .17f, 3) - .5f) < 1e-6f);
    fft_display_reset(&display);
    assert(!display.count && !display.detector.phase_valid);
    draw(FFT_VIEW_PHASE_COMB, guarded.fb);
    assert(!pixels(guarded.fb, 0, 0, 128, 64)); // no stale clock after reset
    puts("Display regressions passed (full-band history, alignment, rollover, markers, bounds, reset).");
}
