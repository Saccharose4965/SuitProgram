#include "app_shell.h"
#include "shell_audio.h"

#include "esp_log.h"

#include "fft.h"

static const char *TAG = "shell";

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
    if (!shell_audio_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT won't run");
        return;
    }
    g_fft_view = FFT_VIEW_BPM_TEXT;
    fft_visualizer_set_view(g_fft_view);
    (void)fft_visualizer_start();
    fft_set_display_enabled(true);
}

void fft_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    fft_set_display_enabled(false);
}
