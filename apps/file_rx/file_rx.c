#include "app_shell.h"
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "audio_rx.h"
#include "audio_player.h"
#include "bt_audio.h"
#include "fft.h"
#include "storage_sd.h"

static const char *TAG = "app_file_rx";

const shell_legend_t FILE_RX_LEGEND = {
    .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_SELECT, SHELL_ICON_NONE },
};

typedef struct {
    esp_err_t last_err;
    bool sd_mounted;
} file_rx_state_t;

static file_rx_state_t s_file_rx = {
    .last_err = ESP_OK,
    .sd_mounted = false,
};

static void file_rx_interrupt_services_if_idle(void)
{
    if (audio_player_is_active()) {
        ESP_LOGI(TAG, "music active; leaving BT/FFT running during file RX");
        return;
    }

    // File RX is memory-heavy on DMA/internal heap; stop competing services.
    fft_visualizer_stop();

    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    if (st.state != BT_AUDIO_STATE_DISABLED && st.state != BT_AUDIO_STATE_IDLE) {
        (void)bt_audio_stop_stream();
        (void)bt_audio_disconnect();
    }
}

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
    file_rx_interrupt_services_if_idle();
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
    }
}

void file_rx_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    bool running = audio_rx_is_running();
    if (running) {
        s_file_rx.sd_mounted = true;
    } else {
        s_file_rx.sd_mounted = storage_sd_is_mounted();
    }

    char state[24], sd[32], port[24], last[48];
    snprintf(state, sizeof(state), "State: %s", running ? "ON" : "OFF");
    snprintf(sd, sizeof(sd), "SD: %s", s_file_rx.sd_mounted ? "mounted" : "not mounted");
    snprintf(port, sizeof(port), "Port: %d", AUDIO_RX_DEFAULT_PORT);
    snprintf(last, sizeof(last), "Last: %s", esp_err_to_name(s_file_rx.last_err));
    const char *lines[] = { state, sd, port, last };
    const shell_text_page_t page = {
        .title = "FILE RX", .lines = lines, .line_count = 4,
    };
    shell_ui_draw_text_page(fb, x, y, w, h, &page);
}
