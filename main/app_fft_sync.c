#include "app_shell.h"
#include "shell_audio.h"

#include <math.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "fft.h"
#include "oled.h"

static const char *TAG = "fft_sync";
static esp_err_t s_fft_sync_start_err = ESP_OK;

const shell_legend_t FFT_SYNC_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_RIGHT, SHELL_ICON_LINK, SHELL_ICON_OK },
};

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if (!fb) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_fill(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            fb_pset(fb, x, y);
        }
    }
}

static void fb_rect_outline(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int x = x0; x <= x1; ++x) {
        fb_pset(fb, x, y0);
        fb_pset(fb, x, y1);
    }
    for (int y = y0; y <= y1; ++y) {
        fb_pset(fb, x0, y);
        fb_pset(fb, x1, y);
    }
}

static int fft_sync_signed_offset_percent(float phase_offset)
{
    float signed_phase = phase_offset;
    while (signed_phase >= 0.5f) signed_phase -= 1.0f;
    while (signed_phase < -0.5f) signed_phase += 1.0f;
    return (int)lroundf(signed_phase * 100.0f);
}

void fft_sync_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_fft_sync_start_err = ESP_OK;
    if (!shell_audio_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT sync unavailable");
        s_fft_sync_start_err = ESP_ERR_INVALID_STATE;
        fft_set_display_enabled(false);
        return;
    }

    fft_set_display_enabled(false);
    s_fft_sync_start_err = fft_visualizer_start();
    if (s_fft_sync_start_err != ESP_OK) {
        ESP_LOGE(TAG, "fft_visualizer_start failed: %s", esp_err_to_name(s_fft_sync_start_err));
    }
}

void fft_sync_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
}

void fft_sync_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev) return;
    if (ev->type != INPUT_EVENT_PRESS && ev->type != INPUT_EVENT_LONG_PRESS) return;

    float phase_step = (ev->type == INPUT_EVENT_LONG_PRESS) ? 0.10f : 0.02f;
    switch (ev->button) {
        case INPUT_BTN_A:
            fft_beat_phase_offset_add(-phase_step);
            break;
        case INPUT_BTN_B:
            fft_beat_phase_offset_add(phase_step);
            break;
        case INPUT_BTN_C:
            fft_beat_lock_toggle();
            break;
        case INPUT_BTN_D:
            fft_beat_enable_toggle();
            break;
        default:
            break;
    }
}

void fft_sync_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)h;
    if (!fb) return;

    if (s_fft_sync_start_err != ESP_OK) {
        char line[24];
        oled_draw_text3x5(fb, x + 2, y + 2, "FFT SYNC FAIL");
        snprintf(line, sizeof(line), "err:%s", esp_err_to_name(s_fft_sync_start_err));
        oled_draw_text3x5(fb, x + 2, y + 12, line);
        return;
    }

    fft_sync_state_t st = {0};
    fft_get_sync_state(&st);

    char line[32];
    oled_draw_text3x5(fb, x + 2, y + 2, "FFT SYNC");

    snprintf(line, sizeof(line), "b:%3u d:%3u",
             (unsigned)lroundf(st.bpm),
             (unsigned)lroundf(st.detected_bpm));
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "L:%s E:%s",
             st.bpm_locked ? "on" : "off",
             st.beat_enabled ? "on" : "off");
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    snprintf(line, sizeof(line), "ofs:%+d%% c:%02u",
             fft_sync_signed_offset_percent(st.phase_offset),
             (unsigned)lroundf(st.confidence * 99.0f));
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    int bar_x = x + 2;
    int bar_w = w - 8;
    int phase_fill = (int)lroundf((float)(bar_w - 2) * st.beat_phase);
    if (phase_fill < 0) phase_fill = 0;
    if (phase_fill > bar_w - 2) phase_fill = bar_w - 2;
    int marker_x = bar_x + 1 + (int)lroundf((float)(bar_w - 3) * st.trigger_phase);
    if (marker_x < bar_x + 1) marker_x = bar_x + 1;
    if (marker_x > bar_x + bar_w - 2) marker_x = bar_x + bar_w - 2;
    fb_rect_outline(fb, bar_x, y + 35, bar_w, 6);
    if (phase_fill > 0) {
        fb_rect_fill(fb, bar_x + 1, y + 36, phase_fill, 4);
    }
    fb_rect_fill(fb, marker_x, y + 34, 1, 8);

    if (!st.running) {
        oled_draw_text3x5(fb, x + 2, y + 43, "fft:stopped");
    } else {
        oled_draw_text3x5(fb, x + 2, y + 43, "AB:ofs C:lock D:on");
    }
}
