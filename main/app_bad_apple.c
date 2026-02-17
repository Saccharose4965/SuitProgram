#include "app_shell.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_err.h"

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

typedef struct {
    esp_err_t last_err;
    bool sd_mounted;
    int current_source;
} bad_apple_app_state_t;

static bad_apple_app_state_t s_bad_apple_app = {
    .last_err = ESP_OK,
    .sd_mounted = false,
    .current_source = -1,
};

const shell_legend_t BAD_APPLE_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_OK, SHELL_ICON_BACK },
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

static esp_err_t start_source_index(int idx)
{
    if (idx < 0 || idx >= (int)(sizeof(k_sources) / sizeof(k_sources[0]))) {
        return ESP_ERR_INVALID_ARG;
    }

    bad_apple_config_t cfg = {
        .path = k_sources[idx].path,
        .type = k_sources[idx].type,
        .target_fps = 30,
        .loop = true,
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
        return start_source_index(first);
    }
    int start = s_bad_apple_app.current_source;

    for (int step = 1; step <= count; ++step) {
        int idx = (start + (dir * step)) % count;
        if (idx < 0) idx += count;
        if (!file_exists(k_sources[idx].path)) continue;

        if (idx == s_bad_apple_app.current_source) {
            return ESP_OK;
        }
        return start_source_index(idx);
    }

    s_bad_apple_app.last_err = ESP_ERR_NOT_FOUND;
    return ESP_ERR_NOT_FOUND;
}

void bad_apple_app_init(shell_app_context_t *ctx)
{
    (void)ctx;

    s_bad_apple_app.last_err = ESP_OK;
    s_bad_apple_app.current_source = -1;

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

    (void)start_source_index(idx);
}

void bad_apple_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    bad_apple_stop();
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
        bad_apple_toggle_pause();
    } else if (ev->button == INPUT_BTN_D) {
        s_bad_apple_app.last_err = bad_apple_restart();
    }
}

void bad_apple_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    bad_apple_tick(dt_sec);
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

    if (!s_bad_apple_app.sd_mounted) {
        char line[48];
        snprintf(line, sizeof(line), "SD mount err: %s", esp_err_to_name(s_bad_apple_app.last_err));
        oled_draw_text3x5(fb, 2, 10, line);
        return;
    }

    if (s_bad_apple_app.last_err == ESP_ERR_NOT_FOUND) {
        oled_draw_text3x5(fb, 2, 10, "Missing video file");
        oled_draw_text3x5(fb, 2, 18, "/sdcard/badapple/");
        oled_draw_text3x5(fb, 2, 26, "video.hs or .raw");
        return;
    }

    if (st.path[0]) {
        char line[48];
        snprintf(line, sizeof(line), "Src: %s", source_label_for_index(s_bad_apple_app.current_source));
        oled_draw_text3x5(fb, 2, 10, line);
        snprintf(line, sizeof(line), "File: %s", st.path);
        oled_draw_text3x5(fb, 2, 18, line);
    }

    {
        char line[48];
        snprintf(line, sizeof(line), "Err: %s", esp_err_to_name(st.last_err));
        oled_draw_text3x5(fb, 2, 26, line);
    }
}
