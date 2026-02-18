#include "app_shell.h"
#include "shell_audio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "fft.h"
#include "oled.h"

#include <stdio.h>

static const char *TAG = "shell";
static esp_err_t s_fft_start_err = ESP_OK;

const shell_legend_t FFT_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_RIGHT, SHELL_ICON_NONE, SHELL_ICON_LINK },
};

static fft_view_t g_fft_view = FFT_VIEW_BPM_TEXT;

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
        default:
            return;
    }

    fft_visualizer_set_view(g_fft_view);
}

void fft_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_fft_start_err = ESP_OK;
    if (!shell_audio_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT won't run");
        s_fft_start_err = ESP_ERR_INVALID_STATE;
        fft_set_display_enabled(false);
        return;
    }
    g_fft_view = FFT_VIEW_BPM_TEXT;
    fft_visualizer_set_view(g_fft_view);
    s_fft_start_err = fft_visualizer_start();
    if (s_fft_start_err != ESP_OK) {
        ESP_LOGE(TAG, "fft_visualizer_start failed: %s", esp_err_to_name(s_fft_start_err));
        fft_set_display_enabled(false);
        return;
    }
    fft_set_display_enabled(true);
}

void fft_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    fft_set_display_enabled(false);
}

void fft_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    if (!fb) return;
    if (s_fft_start_err != ESP_OK) {
        char line[24];
        oled_draw_text3x5(fb, 0, 0, "FFT start failed");
        snprintf(line, sizeof(line), "err:%s", esp_err_to_name(s_fft_start_err));
        oled_draw_text3x5(fb, 0, 10, line);
        oled_draw_text3x5(fb, 0, 20, "Try closing BT scan");
        return;
    }
    fft_copy_frame(fb, PANEL_W * PANEL_H / 8);
}
