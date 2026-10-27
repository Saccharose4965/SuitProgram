#include "app_shell.h"
#include "ui_draw.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_timer.h"

#include "audio.h"
#include "audio_player.h"
#include "bad_apple.h"
#include "storage_sd.h"

typedef struct {
    const char *path;
    bad_apple_source_type_t type;
    const char *label;
} bad_apple_source_candidate_t;

static const bad_apple_source_candidate_t k_sources[] = {
    { "/sdcard/badapple/video.hs",  BAD_APPLE_SOURCE_HEATSHRINK_RLE, "hs+rle" },
    { "/sdcard/video.hs",           BAD_APPLE_SOURCE_HEATSHRINK_RLE, "hs+rle" },
    { "/sdcard/badapple/video.rle", BAD_APPLE_SOURCE_RLE,            "rle"    },
    { "/sdcard/video.rle",          BAD_APPLE_SOURCE_RLE,            "rle"    },
    { "/sdcard/badapple/video.raw", BAD_APPLE_SOURCE_RAW,            "raw"    },
    { "/sdcard/video.raw",          BAD_APPLE_SOURCE_RAW,            "raw"    },
    { "/sdcard/badapple/video.uc",  BAD_APPLE_SOURCE_RAW,            "raw"    },
    { "/sdcard/video.uc",           BAD_APPLE_SOURCE_RAW,            "raw"    },
    { "/sdcard/badapple/video.bin", BAD_APPLE_SOURCE_RAW,            "raw"    },
    { "/sdcard/video.bin",          BAD_APPLE_SOURCE_RAW,            "raw"    },
};

static const char *k_audio_paths[] = {
    "/sdcard/badapple/BadApple.wav",
    "/sdcard/BadApple.wav",
    "/sdcard/badapple/badapple.wav",
    "/sdcard/badapple.wav",
};

#define BAD_APPLE_AUDIO_START_TIMEOUT_US 5000000LL
#define BAD_APPLE_AUDIO_START_GRACE_US    250000LL

typedef struct {
    esp_err_t last_err;
    bool sd_mounted;
    int current_source;
    bool audio_pending;
    int pending_source;
    int64_t audio_request_us;
} bad_apple_app_state_t;

static bad_apple_app_state_t s_bad_apple_app = {
    .last_err = ESP_OK,
    .sd_mounted = false,
    .current_source = -1,
    .audio_pending = false,
    .pending_source = -1,
    .audio_request_us = 0,
};

const shell_legend_t BAD_APPLE_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_RIGHT, SHELL_ICON_PAUSE, SHELL_ICON_REFRESH },
};

static bool file_exists(const char *path)
{
    if (!path || !path[0]) return false;
    struct stat st;
    return (stat(path, &st) == 0) && S_ISREG(st.st_mode);
}

static int find_first_available_source(void)
{
    for (size_t i = 0; i < sizeof(k_sources) / sizeof(k_sources[0]); ++i) {
        if (file_exists(k_sources[i].path)) return (int)i;
    }
    return -1;
}

static const char *find_audio_path(void)
{
    for (size_t i = 0; i < sizeof(k_audio_paths) / sizeof(k_audio_paths[0]); ++i) {
        if (file_exists(k_audio_paths[i])) return k_audio_paths[i];
    }
    return NULL;
}

static esp_err_t start_media(int idx)
{
    if (idx < 0 || idx >= (int)(sizeof(k_sources) / sizeof(k_sources[0]))) {
        return ESP_ERR_INVALID_ARG;
    }
    const char *audio_path = find_audio_path();
    if (!audio_path) return ESP_ERR_NOT_FOUND;

    audio_player_stop();
    bad_apple_stop();
    esp_err_t err = audio_player_play(audio_path);
    if (err != ESP_OK) return err;

    s_bad_apple_app.audio_pending = true;
    s_bad_apple_app.pending_source = idx;
    s_bad_apple_app.current_source = idx;
    s_bad_apple_app.audio_request_us = esp_timer_get_time();
    s_bad_apple_app.last_err = ESP_OK;
    return ESP_OK;
}

static esp_err_t start_source_index(int idx)
{
    if (idx < 0 || idx >= (int)(sizeof(k_sources) / sizeof(k_sources[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    bad_apple_config_t cfg = {
        .path = k_sources[idx].path,
        .type = k_sources[idx].type,
        .target_fps = 30,
        .loop = false,
    };

    esp_err_t err = bad_apple_start(&cfg);
    s_bad_apple_app.last_err = err;
    if (err == ESP_OK) {
        s_bad_apple_app.current_source = idx;
    }
    return err;
}

static esp_err_t cycle_source(int dir)
{
    const int count = (int)(sizeof(k_sources) / sizeof(k_sources[0]));
    if (count == 0) return ESP_ERR_NOT_FOUND;

    if (s_bad_apple_app.current_source < 0 || s_bad_apple_app.current_source >= count) {
        int first = find_first_available_source();
        if (first < 0) {
            s_bad_apple_app.last_err = ESP_ERR_NOT_FOUND;
            return ESP_ERR_NOT_FOUND;
        }
        return start_media(first);
    }
    int start = s_bad_apple_app.current_source;

    for (int step = 1; step <= count; ++step) {
        int idx = (start + (dir * step)) % count;
        if (idx < 0) idx += count;
        if (!file_exists(k_sources[idx].path)) continue;

        if (idx == s_bad_apple_app.current_source) {
            return ESP_OK;
        }
        return start_media(idx);
    }

    s_bad_apple_app.last_err = ESP_ERR_NOT_FOUND;
    return ESP_ERR_NOT_FOUND;
}

void bad_apple_app_init(shell_app_context_t *ctx)
{
    (void)ctx;

    s_bad_apple_app.last_err = ESP_OK;
    s_bad_apple_app.current_source = -1;
    s_bad_apple_app.audio_pending = false;
    s_bad_apple_app.pending_source = -1;
    s_bad_apple_app.audio_request_us = 0;

    if (!storage_sd_is_mounted()) {
        s_bad_apple_app.last_err = storage_mount_sd();
        s_bad_apple_app.sd_mounted = (s_bad_apple_app.last_err == ESP_OK);
    } else {
        s_bad_apple_app.sd_mounted = true;
    }

    if (!s_bad_apple_app.sd_mounted) {
        bad_apple_stop();
        return;
    }

    int idx = find_first_available_source();
    if (idx < 0) {
        s_bad_apple_app.last_err = ESP_ERR_NOT_FOUND;
        bad_apple_stop();
        return;
    }

    s_bad_apple_app.last_err = start_media(idx);
}

void bad_apple_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    s_bad_apple_app.audio_pending = false;
    s_bad_apple_app.pending_source = -1;
    s_bad_apple_app.audio_request_us = 0;
    bad_apple_stop();
    audio_player_stop();
}

void bad_apple_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        (void)cycle_source(-1);
    } else if (ev->button == INPUT_BTN_B) {
        (void)cycle_source(+1);
    } else if (ev->button == INPUT_BTN_C) {
        if (audio_player_toggle_play_pause() == ESP_OK) {
            bad_apple_set_paused(audio_player_is_paused());
        }
    } else if (ev->button == INPUT_BTN_D) {
        int idx = s_bad_apple_app.current_source;
        if (idx >= 0) {
            s_bad_apple_app.last_err = start_media(idx);
        } else {
            s_bad_apple_app.last_err = ESP_ERR_INVALID_STATE;
        }
    }
}

void bad_apple_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    (void)dt_sec;
    int64_t now_us = esp_timer_get_time();
    audio_timeline_t timeline = audio_timeline_get(now_us);

    if (s_bad_apple_app.audio_pending) {
        if (timeline.valid) {
            s_bad_apple_app.audio_pending = false;
            int idx = s_bad_apple_app.pending_source;
            s_bad_apple_app.pending_source = -1;
            s_bad_apple_app.last_err = start_source_index(idx);
        } else if ((now_us - s_bad_apple_app.audio_request_us) >
                       BAD_APPLE_AUDIO_START_TIMEOUT_US ||
                   ((now_us - s_bad_apple_app.audio_request_us) >
                        BAD_APPLE_AUDIO_START_GRACE_US &&
                    !audio_player_is_active())) {
            s_bad_apple_app.audio_pending = false;
            s_bad_apple_app.pending_source = -1;
            s_bad_apple_app.last_err = ESP_ERR_TIMEOUT;
        }
    }

    if (!s_bad_apple_app.audio_pending && timeline.valid) {
        bool paused = audio_player_is_paused() || timeline.stalled;
        bad_apple_set_paused(paused);
        if (!paused) {
            esp_err_t err = bad_apple_sync_playhead(timeline.position_ms);
            if (err != ESP_OK) s_bad_apple_app.last_err = err;
        }
    }
}

static const char *source_label_for_index(int idx)
{
    if (idx < 0 || idx >= (int)(sizeof(k_sources) / sizeof(k_sources[0]))) {
        return "?";
    }
    return k_sources[idx].label;
}

void bad_apple_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)x;
    (void)y;
    (void)w;
    (void)h;

    if (!fb) return;

    bad_apple_status_t st;
    bad_apple_get_status(&st);

    if (bad_apple_copy_frame(fb, PANEL_W * PANEL_H / 8) == ESP_OK) {
        if (st.paused) {
            oled_draw_text3x5(fb, 2, 2, "PAUSE");
        } else if (!st.playing && st.eof) {
            oled_draw_text3x5(fb, 2, 2, "END");
        }
        return;
    }

    oled_draw_text3x5(fb, 2, 2, "BAD APPLE");

    if (s_bad_apple_app.audio_pending) {
        oled_draw_text3x5(fb, 2, 10, "Waiting for audio...");
        return;
    }

    if (!s_bad_apple_app.sd_mounted) {
        char line[48];
        snprintf(line, sizeof(line), "SD mount err: %s", esp_err_to_name(s_bad_apple_app.last_err));
        oled_draw_text3x5(fb, 2, 10, line);
        return;
    }

    if (s_bad_apple_app.last_err == ESP_ERR_NOT_FOUND) {
        if (!find_audio_path()) {
            oled_draw_text3x5(fb, 2, 10, "Missing audio WAV");
            oled_draw_text3x5(fb, 2, 18, "/sdcard/badapple/");
            oled_draw_text3x5(fb, 2, 26, "BadApple.wav");
        } else {
            oled_draw_text3x5(fb, 2, 10, "Missing video file");
            oled_draw_text3x5(fb, 2, 18, "/sdcard/badapple/");
            oled_draw_text3x5(fb, 2, 26, "video.hs or .raw");
        }
        return;
    }

    if (st.path[0]) {
        char line[48];
        snprintf(line, sizeof(line), "Src: %s", source_label_for_index(s_bad_apple_app.current_source));
        oled_draw_text3x5(fb, 2, 10, line);
        const int max_file_chars = (int)sizeof(line) - (int)sizeof("File: ");
        snprintf(line, sizeof(line), "File: %.*s", max_file_chars, st.path);
        oled_draw_text3x5(fb, 2, 18, line);
    }

    {
        char line[48];
        snprintf(line, sizeof(line), "Err: %s", esp_err_to_name(st.last_err));
        oled_draw_text3x5(fb, 2, 26, line);
    }
}
