#include "app_shell.h"
#include "ui_draw.h"
#include "audio_bus.h"
#include "audio_policy.h"

#include "esp_err.h"
#include "esp_log.h"

#include "fft.h"
#include "fft_display.h"
#include "fft_history.h"
#include "oled.h"
#include "esp_attr.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "shell";
static esp_err_t s_fft_start_err = ESP_OK;

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
    s_fft_start_err = ESP_OK;
    if (audio_policy_fft_blocked_by_bluetooth()) {
        s_fft_start_err = ESP_ERR_INVALID_STATE;
        return;
    }
    if (!audio_bus_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT won't run");
        s_fft_start_err = ESP_ERR_INVALID_STATE;
        return;
    }
    g_fft_view = FFT_VIEW_SPECTRUM;
    s_fft_start_err = fft_visualizer_start();
    if (s_fft_start_err != ESP_OK) {
        ESP_LOGE(TAG, "fft_visualizer_start failed: %s", esp_err_to_name(s_fft_start_err));
        return;
    }
    s_have_frame = false;
}

void fft_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
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
        if (audio_policy_fft_blocked_by_bluetooth()) {
            oled_draw_text3x5(fb, 0, 0, "FFT paused for Bluetooth");
            oled_draw_text3x5(fb, 0, 10, "Disconnect BT, then reopen FFT");
            return;
        }
        char line[24];
        oled_draw_text3x5(fb, 0, 0, "FFT start failed");
        snprintf(line, sizeof(line), "err:%s", esp_err_to_name(s_fft_start_err));
        oled_draw_text3x5(fb, 0, 10, line);
        oled_draw_text3x5(fb, 0, 20, "Try closing BT scan");
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
