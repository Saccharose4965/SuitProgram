#include "app_shell.h"

#include <stdio.h>
#include <string.h>

#include "audio.h"
#include "audio_rx.h"
#include "bt_audio.h"
#include "esp_err.h"
#include "fft.h"
#include "oled.h"
#include "shell_audio.h"
#include "storage_sd.h"

typedef enum {
    RESTART_ITEM_BACK = 0,
    RESTART_ITEM_SD_REMOUNT,
    RESTART_ITEM_OLED_RESET,
    RESTART_ITEM_AUDIO_IDLE,
    RESTART_ITEM_BT_DISCONNECT,
    RESTART_ITEM_FFT_STOP,
    RESTART_ITEM_FILE_RX_STOP,
    RESTART_ITEM_TODO_LINK,
    RESTART_ITEM_TODO_GPS,
    RESTART_ITEM_TODO_ORI,
    RESTART_ITEM_TODO_POWER,
} restart_item_id_t;

typedef struct {
    const char *label;
    restart_item_id_t id;
} restart_entry_t;

typedef struct {
    size_t selected;
    esp_err_t last_err;
    char status[28];
} restart_state_t;

static const restart_entry_t s_entries[] = {
    { "Back",            RESTART_ITEM_BACK },
    { "SD remount",      RESTART_ITEM_SD_REMOUNT },
    { "OLED reset",      RESTART_ITEM_OLED_RESET },
    { "Audio idle",      RESTART_ITEM_AUDIO_IDLE },
    { "BT disconnect",   RESTART_ITEM_BT_DISCONNECT },
    { "FFT stop",        RESTART_ITEM_FFT_STOP },
    { "FileRX stop",     RESTART_ITEM_FILE_RX_STOP },
    { "Link restart*",   RESTART_ITEM_TODO_LINK },
    { "GPS restart*",    RESTART_ITEM_TODO_GPS },
    { "IMU restart*",    RESTART_ITEM_TODO_ORI },
    { "Power restart*",  RESTART_ITEM_TODO_POWER },
};

static restart_state_t s_restart = {
    .selected = 0,
    .last_err = ESP_OK,
    .status = "C: run, * needs API",
};

const shell_legend_t SERVICE_RESTART_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_OK, SHELL_ICON_MENU },
};

static void restart_set_status(const char *msg, esp_err_t err)
{
    if (!msg) msg = "";
    s_restart.last_err = err;
    strncpy(s_restart.status, msg, sizeof(s_restart.status) - 1);
    s_restart.status[sizeof(s_restart.status) - 1] = '\0';
}

static void restart_do_action(shell_app_context_t *ctx, restart_item_id_t id)
{
    esp_err_t err = ESP_OK;

    switch (id) {
        case RESTART_ITEM_BACK:
            if (ctx && ctx->request_switch) {
                ctx->request_switch("menu", ctx->request_user_data);
            }
            return;

        case RESTART_ITEM_SD_REMOUNT:
            if (storage_sd_is_mounted()) {
                storage_unmount_sd();
            }
            err = storage_mount_sd();
            restart_set_status((err == ESP_OK) ? "SD remounted" : "SD remount failed", err);
            return;

        case RESTART_ITEM_OLED_RESET:
            oled_init();
            oled_clear();
            restart_set_status("OLED reset done", ESP_OK);
            return;

        case RESTART_ITEM_AUDIO_IDLE:
            if (!shell_audio_init_if_needed()) {
                err = ESP_FAIL;
            } else {
                err = audio_disable_all();
            }
            restart_set_status((err == ESP_OK) ? "Audio set to idle" : "Audio idle failed", err);
            return;

        case RESTART_ITEM_BT_DISCONNECT: {
            esp_err_t stop_err = bt_audio_stop_stream();
            if (stop_err != ESP_OK && stop_err != ESP_ERR_INVALID_STATE) {
                restart_set_status("BT stop stream failed", stop_err);
                return;
            }
            err = bt_audio_disconnect();
            if (err == ESP_ERR_INVALID_STATE) err = ESP_OK;
            restart_set_status((err == ESP_OK) ? "BT disconnected" : "BT disconnect failed", err);
            return;
        }

        case RESTART_ITEM_FFT_STOP:
            fft_set_display_enabled(false);
            fft_visualizer_stop();
            restart_set_status("FFT stopped", ESP_OK);
            return;

        case RESTART_ITEM_FILE_RX_STOP:
            if (!audio_rx_is_running()) {
                restart_set_status("FileRX already stopped", ESP_OK);
                return;
            }
            err = audio_rx_stop();
            restart_set_status((err == ESP_OK) ? "FileRX stopped" : "FileRX stop failed", err);
            return;

        case RESTART_ITEM_TODO_LINK:
            restart_set_status("Link: need stop/deinit API", ESP_ERR_NOT_SUPPORTED);
            return;

        case RESTART_ITEM_TODO_GPS:
            restart_set_status("GPS: need stop/deinit API", ESP_ERR_NOT_SUPPORTED);
            return;

        case RESTART_ITEM_TODO_ORI:
            restart_set_status("IMU: need stop/deinit API", ESP_ERR_NOT_SUPPORTED);
            return;

        case RESTART_ITEM_TODO_POWER:
            restart_set_status("Power: need stop/deinit API", ESP_ERR_NOT_SUPPORTED);
            return;

        default:
            restart_set_status("Unknown restart item", ESP_ERR_INVALID_ARG);
            return;
    }
}

void service_restart_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_restart.selected = 0;
    s_restart.last_err = ESP_OK;
    restart_set_status("C: run, * needs API", ESP_OK);
}

void service_restart_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;

    const size_t count = sizeof(s_entries) / sizeof(s_entries[0]);
    if (count == 0) return;

    if (ev->button == INPUT_BTN_A) {
        if (s_restart.selected > 0) s_restart.selected--;
    } else if (ev->button == INPUT_BTN_B) {
        if (s_restart.selected + 1 < count) s_restart.selected++;
    } else if (ev->button == INPUT_BTN_C) {
        restart_do_action(ctx, s_entries[s_restart.selected].id);
    } else if (ev->button == INPUT_BTN_D) {
        if (ctx->request_switch) {
            ctx->request_switch("menu", ctx->request_user_data);
        }
    }
}

void service_restart_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    if (!fb || h <= 12) return;

    const int row_h = 8;
    const int title_y = y + 1;
    const int rows_top = y + 8;
    const int rows_fit = (h - 20) / row_h;
    const int visible_rows = (rows_fit < 1) ? 1 : rows_fit;
    const size_t count = sizeof(s_entries) / sizeof(s_entries[0]);

    int start = 0;
    if ((int)s_restart.selected >= visible_rows) {
        start = (int)s_restart.selected - visible_rows + 1;
    }
    if (start < 0) start = 0;
    if (start > (int)count - visible_rows) {
        start = (int)count - visible_rows;
        if (start < 0) start = 0;
    }

    oled_draw_text3x5(fb, x + 2, title_y, "COMP RESTART");
    for (int row = 0; row < visible_rows; ++row) {
        int idx = start + row;
        if (idx < 0 || idx >= (int)count) break;

        char line[24];
        snprintf(line, sizeof(line), "%c%s",
                 (size_t)idx == s_restart.selected ? '>' : ' ',
                 s_entries[idx].label);
        oled_draw_text3x5(fb, x + 2, rows_top + row * row_h, line);
    }

    char err_line[24];
    snprintf(err_line, sizeof(err_line), "err:%s", esp_err_to_name(s_restart.last_err));
    oled_draw_text3x5(fb, x + 2, y + h - 11, s_restart.status);
    oled_draw_text3x5(fb, x + 2, y + h - 5, err_line);
}
