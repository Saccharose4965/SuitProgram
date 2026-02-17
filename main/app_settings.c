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
    .speaker_volume = 1.0f,
    .bt_volume = 1.0f,
    .speaker_muted = false,
    .bt_muted = false,
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
    float spk = clamp_volume(s->speaker_volume);
    float bt = clamp_volume(s->bt_volume);

    audio_set_volume(s->speaker_muted ? 0.0f : spk);
    bt_audio_volume_set_percent(s->bt_muted ? 0 : volume_to_bt_percent(bt));
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
            // Backward-compat legacy key: apply to both sinks.
            char *end = NULL;
            float v = strtof(val, &end);
            if (end != val) {
                float clamped = clamp_volume(v);
                tmp.speaker_volume = clamped;
                tmp.bt_volume = clamped;
            }
        } else if (strcmp(key, "muted") == 0) {
            // Backward-compat legacy key: apply to both sinks.
            char *end = NULL;
            long m = strtol(val, &end, 10);
            if (end != val) {
                bool muted = (m != 0);
                tmp.speaker_muted = muted;
                tmp.bt_muted = muted;
            }
        } else if (strcmp(key, "speaker_volume") == 0) {
            char *end = NULL;
            float v = strtof(val, &end);
            if (end != val) {
                tmp.speaker_volume = clamp_volume(v);
            }
        } else if (strcmp(key, "bt_volume") == 0) {
            char *end = NULL;
            float v = strtof(val, &end);
            if (end != val) {
                tmp.bt_volume = clamp_volume(v);
            }
        } else if (strcmp(key, "speaker_muted") == 0) {
            char *end = NULL;
            long m = strtol(val, &end, 10);
            if (end != val) {
                tmp.speaker_muted = (m != 0);
            }
        } else if (strcmp(key, "bt_muted") == 0) {
            char *end = NULL;
            long m = strtol(val, &end, 10);
            if (end != val) {
                tmp.bt_muted = (m != 0);
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

    fprintf(f, "speaker_volume=%.2f\n", clamp_volume(s->speaker_volume));
    fprintf(f, "bt_volume=%.2f\n", clamp_volume(s->bt_volume));
    fprintf(f, "speaker_muted=%d\n", s->speaker_muted ? 1 : 0);
    fprintf(f, "bt_muted=%d\n", s->bt_muted ? 1 : 0);
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

void app_settings_set_audio(float speaker_volume,
                            bool speaker_muted,
                            float bt_volume,
                            bool bt_muted,
                            bool persist)
{
    s_settings.speaker_volume = clamp_volume(speaker_volume);
    s_settings.bt_volume = clamp_volume(bt_volume);
    s_settings.speaker_muted = speaker_muted;
    s_settings.bt_muted = bt_muted;
    apply_audio_volume(&s_settings);
    if (persist) {
        settings_save_to_sd(&s_settings);
    }
}
