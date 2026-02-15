#include "app_shell.h"

#include <stdio.h>

#include "esp_err.h"

#include "audio_rx.h"
#include "storage_sd.h"

const shell_legend_t FILE_RX_LEGEND = {
    .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_OK, SHELL_ICON_MENU },
};

typedef struct {
    esp_err_t last_err;
    bool sd_mounted;
} file_rx_state_t;

static file_rx_state_t s_file_rx = {
    .last_err = ESP_OK,
    .sd_mounted = false,
};

static esp_err_t file_rx_start_service(void)
{
    if (!storage_sd_is_mounted()) {
        esp_err_t mount_err = storage_mount_sd();
        if (mount_err != ESP_OK) {
            s_file_rx.sd_mounted = false;
            return mount_err;
        }
    }
    s_file_rx.sd_mounted = true;
    return audio_rx_start(AUDIO_RX_DEFAULT_PORT);
}

void file_rx_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_file_rx.last_err = ESP_OK;
    s_file_rx.sd_mounted = storage_sd_is_mounted();
}

void file_rx_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_C) {
        if (audio_rx_is_running()) {
            s_file_rx.last_err = audio_rx_stop();
        } else {
            s_file_rx.last_err = file_rx_start_service();
        }
    } else if (ev->button == INPUT_BTN_D) {
        if (ctx->request_switch) {
            ctx->request_switch("menu", ctx->request_user_data);
        }
    }
}

void file_rx_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    (void)h;
    if (!fb) return;

    bool running = audio_rx_is_running();
    if (running) {
        s_file_rx.sd_mounted = true;
    } else {
        s_file_rx.sd_mounted = storage_sd_is_mounted();
    }

    char line[48];
    oled_draw_text3x5(fb, x + 2, y + 2, "FILE RX");

    snprintf(line, sizeof(line), "State: %s", running ? "ON" : "OFF");
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "SD: %s", s_file_rx.sd_mounted ? "mounted" : "not mounted");
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    snprintf(line, sizeof(line), "Port: %d", AUDIO_RX_DEFAULT_PORT);
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    snprintf(line, sizeof(line), "Last: %s", esp_err_to_name(s_file_rx.last_err));
    oled_draw_text3x5(fb, x + 2, y + 34, line);

    oled_draw_text3x5(fb, x + 2, y + 42, "C: start/stop");
}
