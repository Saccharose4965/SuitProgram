#include "app_settings.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>

#include "audio.h"
#include "bt_audio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "storage_sd.h"

#define SETTINGS_FILE_NAME "suit_settings.txt"

static const char *TAG = "settings";

static app_settings_t s_settings = {
    .volume = 1.0f,
    .muted = false,
};

static float clamp_volume(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;
    return v;
}

static int volume_to_bt_percent(float v)
{
    int pct = (int)lroundf(v * 50.0f); // 0.0->0, 2.0->100
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

static void apply_audio_volume(const app_settings_t *s)
{
    if (!s) return;
    float v = clamp_volume(s->volume);
    if (s->muted) {
        audio_set_volume(0.0f);
        bt_audio_volume_set_percent(0);
        return;
    }
    audio_set_volume(v);
    bt_audio_volume_set_percent(volume_to_bt_percent(v));
}

static char *trim(char *s)
{
    if (!s) return s;
    while (isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) end--;
    *end = '\0';
    return s;
}

static bool settings_load_from_sd(app_settings_t *out)
{
    if (!out) return false;
    char path[96];
    if (storage_sd_make_path(path, sizeof(path), SETTINGS_FILE_NAME) != ESP_OK) {
        return false;
    }

    FILE *f = fopen(path, "r");
    if (!f) return false;

    app_settings_t tmp = *out;
    char line[96];
    while (fgets(line, sizeof(line), f)) {
        char *p = trim(line);
        if (*p == '\0' || *p == '#' || *p == ';') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);
        if (strcmp(key, "volume") == 0) {
            char *end = NULL;
            float v = strtof(val, &end);
            if (end != val) {
                tmp.volume = clamp_volume(v);
            }
        } else if (strcmp(key, "muted") == 0) {
            char *end = NULL;
            long m = strtol(val, &end, 10);
            if (end != val) {
                tmp.muted = (m != 0);
            }
        }
    }

    fclose(f);
    *out = tmp;
    return true;
}

static void settings_save_to_sd(const app_settings_t *s)
{
    if (!s) return;
    if (storage_mount_sd() != ESP_OK) return;

    char path[96];
    if (storage_sd_make_path(path, sizeof(path), SETTINGS_FILE_NAME) != ESP_OK) {
        return;
    }

    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open %s for write", path);
        return;
    }

    fprintf(f, "volume=%.2f\n", s->volume);
    fprintf(f, "muted=%d\n", s->muted ? 1 : 0);
    fflush(f);
    int fd = fileno(f);
    if (fd >= 0) fsync(fd);
    fclose(f);
}

void app_settings_init(void)
{
    app_settings_t loaded = s_settings;
    esp_err_t sd = storage_mount_sd();
    if (sd == ESP_OK) {
        if (settings_load_from_sd(&loaded)) {
            s_settings = loaded;
        }
    } else {
        ESP_LOGW(TAG, "SD mount failed (%s); using defaults", esp_err_to_name(sd));
    }

    apply_audio_volume(&s_settings);
}

const app_settings_t *app_settings_get(void)
{
    return &s_settings;
}

void app_settings_set_volume(float volume, bool muted, bool persist)
{
    s_settings.volume = clamp_volume(volume);
    s_settings.muted = muted;
    apply_audio_volume(&s_settings);
    if (persist) {
        settings_save_to_sd(&s_settings);
    }
}
