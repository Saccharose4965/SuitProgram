#include "app_shell.h"
#include "ui_draw.h"
#include "audio_policy.h"

#include "fft.h"
#include "fft_display.h"
#include "fft_history.h"
#include "oled.h"
#include "esp_attr.h"

#include <string.h>

static fft_view_t g_fft_view = FFT_VIEW_SPECTRUM;
static EXT_RAM_BSS_ATTR fft_history_snapshot_t s_draw_snapshot;
static EXT_RAM_BSS_ATTR uint8_t s_cached_frame[FFT_DISPLAY_BYTES];
static uint32_t s_draw_generation;
static fft_view_t s_draw_view;
static bool s_have_frame;

void fft_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    switch (ev->button) {
        case INPUT_BTN_A:
            g_fft_view = (fft_view_t)((g_fft_view + FFT_VIEW_COUNT - 1) % FFT_VIEW_COUNT);
            break;
        case INPUT_BTN_B:
            g_fft_view = (fft_view_t)((g_fft_view + 1) % FFT_VIEW_COUNT);
            break;
        case INPUT_BTN_C:
        case INPUT_BTN_D:
            fft_detector_reset();
            return;
        default:
            return;
    }

}

void fft_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    g_fft_view = FFT_VIEW_SPECTRUM;
    audio_policy_fft_request(AUDIO_POLICY_FFT_REQ_VIEW, true);
    s_have_frame = false;
}

void fft_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    audio_policy_fft_request(AUDIO_POLICY_FFT_REQ_VIEW, false);
}

void fft_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    if (!fb) return;
    if (!fft_visualizer_running()) {
        if (audio_policy_fft_blocked_by_bluetooth()) {
            oled_draw_text3x5(fb, 0, 0, "FFT paused for Bluetooth");
            oled_draw_text3x5(fb, 0, 10, "Resumes when Bluetooth is idle");
        } else {
            oled_draw_text3x5(fb, 0, 0, "FFT waiting for audio");
        }
        return;
    }
    uint32_t generation = fft_history_snapshot(&s_draw_snapshot);
    if (!s_have_frame || generation != s_draw_generation ||
        g_fft_view != s_draw_view) {
        fft_display_draw(&s_draw_snapshot, g_fft_view, s_cached_frame);
        s_draw_generation = generation;
        s_draw_view = g_fft_view;
        s_have_frame = true;
    }
    memcpy(fb, s_cached_frame, FFT_DISPLAY_BYTES);
}
