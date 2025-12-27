#include "app_shell.h"
#include "shell_audio.h"

#include "esp_log.h"

#include "fft.h"

static const char *TAG = "shell";

const shell_legend_t FFT_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_LEFT, SHELL_ICON_RIGHT, SHELL_ICON_LINK },
};

static fft_view_t g_fft_view = FFT_VIEW_BPM_TEXT;
static input_button_t s_fft_view_latch = INPUT_BTN_NONE; // require release before next view change
static TickType_t s_fft_next_ok_tick = 0;                // debounce window for view changes

void fft_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        ctx->request_switch("menu", ctx->request_user_data);
        return;
    }

    // B/C cycle views; require release before another change
    if (ev->button == INPUT_BTN_B || ev->button == INPUT_BTN_C) {
        if (ev->type == INPUT_EVENT_RELEASE && ev->button == s_fft_view_latch) {
            s_fft_view_latch = INPUT_BTN_NONE;
        } else if (ev->type == INPUT_EVENT_PRESS) {
            if (s_fft_view_latch != INPUT_BTN_NONE) return; // wait for prior release
            if (ev->at_ticks < s_fft_next_ok_tick) return;  // debounce multiple triggers while held/bouncing

            if (ev->button == INPUT_BTN_B) {
                if (g_fft_view == 0) g_fft_view = FFT_VIEW_PHASE_COMB;
                else g_fft_view = (fft_view_t)((g_fft_view - 1) % (FFT_VIEW_PHASE_COMB + 1));
            } else {
                g_fft_view = (fft_view_t)((g_fft_view + 1) % (FFT_VIEW_PHASE_COMB + 1));
            }
            fft_visualizer_set_view(g_fft_view);
            s_fft_view_latch = ev->button;
            s_fft_next_ok_tick = ev->at_ticks + pdMS_TO_TICKS(200); // ~200 ms debounce
        }
        return;
    }
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
