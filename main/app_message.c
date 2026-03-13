#include "app_shell.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "audio_player.h"
#include "bt_audio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "fft.h"
#include "microphone.h"
#include "oled.h"
#include "storage_sd.h"
#include "telemetry.h"

static const char *TAG = "app_message";

enum {
    MESSAGE_SAMPLE_RATE_HZ = 16000,
    MESSAGE_PORT_OFFSET = 1,
};

typedef struct {
    esp_err_t last_err;
    esp_err_t send_err;
    bool sd_ready;
    bool mic_ready;
    bool rx_ready;
    bool send_active;
    size_t send_sent;
    size_t send_total;
    int64_t rec_started_us;
    char outbox_dir[128];
    char inbox_dir[128];
    char last_path[128];
} message_state_t;

typedef struct {
    char path[128];
} message_send_task_ctx_t;

static message_state_t s_message = {
    .last_err = ESP_OK,
    .send_err = ESP_OK,
};

shell_legend_t MESSAGE_LEGEND = {
    .slots = { SHELL_ICON_RECORD, SHELL_ICON_PLAY, SHELL_ICON_NONE, SHELL_ICON_SELECT },
};

static void message_update_legend(void)
{
    MESSAGE_LEGEND.slots[0] = mic_rec_is_recording() ? SHELL_ICON_STOP : SHELL_ICON_RECORD;
}

static uint16_t message_port(void)
{
#if CONFIG_LINK_WIFI_PC_PORT >= 65535
    return (uint16_t)CONFIG_LINK_WIFI_PC_PORT;
#else
    return (uint16_t)(CONFIG_LINK_WIFI_PC_PORT + MESSAGE_PORT_OFFSET);
#endif
}

static const char *basename_or_self(const char *path)
{
    if (!path || !path[0]) return "(none)";
    const char *base = strrchr(path, '/');
    return base ? (base + 1) : path;
}

static bool ensure_dir(const char *path)
{
    struct stat st = {0};
    if (stat(path, &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    if (mkdir(path, 0777) == 0) {
        return true;
    }
    return errno == EEXIST;
}

static bool message_prepare_dirs(void)
{
    if (storage_mount_sd() != ESP_OK) {
        s_message.sd_ready = false;
        return false;
    }

    char root_dir[128];
    if (storage_sd_make_path(root_dir, sizeof(root_dir), "messages") != ESP_OK) {
        s_message.sd_ready = false;
        return false;
    }
    if (storage_sd_make_path(s_message.outbox_dir, sizeof(s_message.outbox_dir), "messages/outbox") != ESP_OK ||
        storage_sd_make_path(s_message.inbox_dir, sizeof(s_message.inbox_dir), "messages/inbox") != ESP_OK) {
        s_message.sd_ready = false;
        return false;
    }

    if (!ensure_dir(root_dir) || !ensure_dir(s_message.outbox_dir) || !ensure_dir(s_message.inbox_dir)) {
        ESP_LOGW(TAG, "message dirs create failed");
        s_message.sd_ready = false;
        return false;
    }

    s_message.sd_ready = true;
    return true;
}

static void message_interrupt_services_for_recording(void)
{
    fft_set_display_enabled(false);
    fft_visualizer_stop();
    audio_player_stop();

    bt_audio_status_t st = {0};
    bt_audio_get_status(&st);
    if (st.state != BT_AUDIO_STATE_DISABLED && st.state != BT_AUDIO_STATE_IDLE) {
        (void)bt_audio_stop_stream();
        (void)bt_audio_disconnect();
    }
}

static esp_err_t message_ensure_mic_ready(void)
{
    if (s_message.mic_ready) return ESP_OK;
    mic_rec_cfg_t cfg = {
        .sample_rate_hz = MESSAGE_SAMPLE_RATE_HZ,
        .bclk_gpio = 4,
        .ws_gpio = 22,
        .din_gpio = 36,
        .i2s_id = 0,
    };
    esp_err_t err = mic_rec_init(&cfg);
    if (err == ESP_OK) {
        s_message.mic_ready = true;
    }
    return err;
}

static esp_err_t message_start_receiver_if_possible(void)
{
    if (s_message.rx_ready) return ESP_OK;
    if (!s_message.sd_ready && !message_prepare_dirs()) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = telemetry_start_file_receiver(message_port(), s_message.inbox_dir);
    if (err == ESP_OK) {
        s_message.rx_ready = true;
    }
    return err;
}

static esp_err_t message_build_record_path(char *out, size_t out_len)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;
    if (!s_message.sd_ready && !message_prepare_dirs()) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t stamp_ms = (uint64_t)(esp_timer_get_time() / 1000LL);
    int n = snprintf(out, out_len, "%s/msg_%" PRIu64 ".wav", s_message.outbox_dir, stamp_ms);
    return (n > 0 && (size_t)n < out_len) ? ESP_OK : ESP_ERR_NO_MEM;
}

static void message_capture_last_path(void)
{
    const char *path = mic_rec_last_path();
    if (!path || !path[0]) return;
    strncpy(s_message.last_path, path, sizeof(s_message.last_path) - 1);
    s_message.last_path[sizeof(s_message.last_path) - 1] = 0;
}

static void message_restore_latest_outbox_path(void)
{
    if (!s_message.outbox_dir[0]) return;

    DIR *dir = opendir(s_message.outbox_dir);
    if (!dir) return;

    char best_name[64] = {0};
    struct dirent *ent = NULL;
    while ((ent = readdir(dir)) != NULL) {
        const char *name = ent->d_name;
        size_t len = strlen(name);
        if (len < 5) continue;
        if (strncmp(name, "msg_", 4) != 0) continue;
        if (strcmp(name + len - 4, ".wav") != 0) continue;
        if (best_name[0] == 0 || strcmp(name, best_name) > 0) {
            strncpy(best_name, name, sizeof(best_name) - 1);
            best_name[sizeof(best_name) - 1] = 0;
        }
    }
    closedir(dir);

    if (!best_name[0]) return;
    int n = snprintf(s_message.last_path, sizeof(s_message.last_path), "%s/%s",
                     s_message.outbox_dir, best_name);
    if (n < 0 || (size_t)n >= sizeof(s_message.last_path)) {
        s_message.last_path[0] = 0;
    }
}

static void message_send_progress(size_t sent, size_t total, void *user_ctx)
{
    (void)user_ctx;
    s_message.send_sent = sent;
    s_message.send_total = total;
}

static void message_send_task(void *arg)
{
    message_send_task_ctx_t *ctx = (message_send_task_ctx_t *)arg;
    esp_err_t err = ESP_FAIL;

    telemetry_cfg_t cfg = {
        .ssid = CONFIG_LINK_WIFI_SSID,
        .pass = CONFIG_LINK_WIFI_PASS,
        .pc_ip = CONFIG_LINK_WIFI_PC_IP,
        .pc_port = (uint16_t)CONFIG_LINK_WIFI_PC_PORT,
    };

    s_message.send_sent = 0;
    s_message.send_total = 0;

    err = telemetry_start(&cfg);
    if (err == ESP_OK) {
        (void)message_start_receiver_if_possible();
        err = telemetry_send_file_udp_ex(ctx->path,
                                         CONFIG_LINK_WIFI_PC_IP,
                                         message_port(),
                                         message_send_progress,
                                         NULL);
    }

    s_message.send_err = err;
    s_message.last_err = err;
    s_message.send_active = false;
    free(ctx);
    vTaskDelete(NULL);
}

static esp_err_t message_start_recording(void)
{
    char path[128];
    if (s_message.send_active) return ESP_ERR_INVALID_STATE;
    esp_err_t err = message_prepare_dirs() ? ESP_OK : ESP_FAIL;
    if (err != ESP_OK) return err;

    err = message_ensure_mic_ready();
    if (err != ESP_OK) return err;

    err = message_build_record_path(path, sizeof(path));
    if (err != ESP_OK) return err;

    message_interrupt_services_for_recording();
    err = mic_rec_start(path);
    if (err == ESP_OK) {
        s_message.rec_started_us = esp_timer_get_time();
        strncpy(s_message.last_path, path, sizeof(s_message.last_path) - 1);
        s_message.last_path[sizeof(s_message.last_path) - 1] = 0;
    }
    return err;
}

static esp_err_t message_stop_recording(void)
{
    esp_err_t err = mic_rec_stop();
    message_capture_last_path();
    s_message.rec_started_us = 0;
    return err;
}

static esp_err_t message_toggle_playback(void)
{
    if (!s_message.last_path[0]) return ESP_ERR_NOT_FOUND;
    if (mic_rec_is_recording()) return ESP_ERR_INVALID_STATE;
    if (s_message.send_active) return ESP_ERR_INVALID_STATE;
    if (audio_player_is_active()) {
        audio_player_stop();
        return ESP_OK;
    }
    return audio_player_play(s_message.last_path);
}

static esp_err_t message_start_send(void)
{
    if (!s_message.last_path[0]) return ESP_ERR_NOT_FOUND;
    if (mic_rec_is_recording()) return ESP_ERR_INVALID_STATE;
    if (s_message.send_active) return ESP_ERR_INVALID_STATE;

    message_send_task_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;

    strncpy(ctx->path, s_message.last_path, sizeof(ctx->path) - 1);
    s_message.send_active = true;
    s_message.send_err = ESP_OK;
    s_message.send_sent = 0;
    s_message.send_total = 0;

    BaseType_t ok = xTaskCreatePinnedToCore(message_send_task,
                                            "msg_send",
                                            6144,
                                            ctx,
                                            4,
                                            NULL,
                                            0);
    if (ok != pdPASS) {
        s_message.send_active = false;
        free(ctx);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void message_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_message.last_err = ESP_OK;
    s_message.send_err = ESP_OK;
    s_message.sd_ready = storage_sd_is_mounted();
    s_message.rec_started_us = 0;
    if (s_message.sd_ready || message_prepare_dirs()) {
        message_restore_latest_outbox_path();
        (void)message_start_receiver_if_possible();
    }
    if (message_ensure_mic_ready() != ESP_OK) {
        s_message.mic_ready = false;
    }
    message_update_legend();
}

void message_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    if (mic_rec_is_recording()) {
        (void)message_stop_recording();
    }
    message_update_legend();
}

void message_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_B) {
        s_message.last_err = message_toggle_playback();
        return;
    }

    if (ev->button == INPUT_BTN_A) {
        s_message.last_err = mic_rec_is_recording()
            ? message_stop_recording()
            : message_start_recording();
        message_update_legend();
        return;
    }

    if (ev->button == INPUT_BTN_D) {
        s_message.last_err = message_start_send();
    }
}

void message_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    (void)h;
    if (!fb) return;

    char line[48];
    oled_draw_text3x5(fb, x + 2, y + 2, "MESSAGE");

    if (mic_rec_is_recording()) {
        int secs = (int)((esp_timer_get_time() - s_message.rec_started_us) / 1000000LL);
        snprintf(line, sizeof(line), "rec:%ds", secs);
    } else if (s_message.send_active) {
        if (s_message.send_total > 0) {
            unsigned pct = (unsigned)((100u * s_message.send_sent) / s_message.send_total);
            snprintf(line, sizeof(line), "send:%u%%", pct);
        } else {
            snprintf(line, sizeof(line), "send:starting");
        }
    } else if (s_message.last_path[0]) {
        snprintf(line, sizeof(line), "ready");
    } else {
        snprintf(line, sizeof(line), "idle");
    }
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "%s", basename_or_self(s_message.last_path));
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    if (s_message.last_path[0]) {
        struct stat st = {0};
        if (stat(s_message.last_path, &st) == 0 && st.st_size > 0) {
            snprintf(line, sizeof(line), "sz:%luk p:%u",
                     (unsigned long)((st.st_size + 1023) / 1024),
                     (unsigned)message_port());
        } else {
            snprintf(line, sizeof(line), "p:%u", (unsigned)message_port());
        }
    } else {
        snprintf(line, sizeof(line), "p:%u", (unsigned)message_port());
    }
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    snprintf(line, sizeof(line), "last:%s", esp_err_to_name(s_message.last_err));
    oled_draw_text3x5(fb, x + 2, y + 34, line);

}
