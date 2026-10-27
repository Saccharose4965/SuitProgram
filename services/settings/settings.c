#include "settings.h"

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>

#include "audio.h"
#include "bt_audio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "led.h"
#include "led_modes.h"
#include "shell_ui.h"
#include "storage_sd.h"

#define SETTINGS_FILE_NAME "suit_settings.txt"

static const char *TAG = "settings";

static settings_t s_settings = {
    .speaker_volume = 1.0f,
    .bt_volume = 1.0f,
    .speaker_muted = false,
    .bt_muted = false,
    .use_2048_direction_icons = false,
    .led_animation = {
        .backend = SETTINGS_LED_BACKEND_BEAT,
        .mode = LED_BEAT_ANIM_PULSE,
        .ambient_speed_percent = 100,
    },
    .led_color = {
        .primary_r = 255, .primary_g = 96, .primary_b = 0,
        .secondary_r = 0, .secondary_g = 72, .secondary_b = 255,
        .color_cycle = LED_COLOR_CYCLE_STATIC,
        .color_style = LED_COLOR_STYLE_MONO,
        .highlight_mode = LED_HIGHLIGHT_OFF,
        .audio_energy_range = LED_AUDIO_ENERGY_RANGE_FULL,
        .brightness = 96,
    },
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

static void apply_audio_volume(const settings_t *s)
{
    if (!s) return;
    float spk = clamp_volume(s->speaker_volume);
    float bt = clamp_volume(s->bt_volume);

    audio_set_volume(s->speaker_muted ? 0.0f : spk);
    bt_audio_volume_set_percent(s->bt_muted ? 0 : volume_to_bt_percent(bt));
}

static void apply_shell_preferences(const settings_t *s)
{
    if (!s) return;
    shell_ui_set_direction_icon_style(
        s->use_2048_direction_icons ? SHELL_DIRECTION_ICON_STYLE_2048
                                    : SHELL_DIRECTION_ICON_STYLE_SHELL);
}

static bool parse_long(const char *text, long *out)
{
    if (!text || !out) return false;
    char *end = NULL;
    long value = strtol(text, &end, 10);
    if (end == text || !end || *end != '\0') return false;
    *out = value;
    return true;
}

static bool parse_rgb(const char *text, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (!text || !r || !g || !b) return false;
    unsigned red = 0, green = 0, blue = 0;
    char trailing = '\0';
    if (sscanf(text, "%u,%u,%u%c", &red, &green, &blue, &trailing) != 3 ||
        red > UINT8_MAX || green > UINT8_MAX || blue > UINT8_MAX) {
        return false;
    }
    *r = (uint8_t)red;
    *g = (uint8_t)green;
    *b = (uint8_t)blue;
    return true;
}

static settings_led_animation_t sanitize_led_animation(settings_led_animation_t value)
{
    if (value.backend != SETTINGS_LED_BACKEND_BEAT &&
        value.backend != SETTINGS_LED_BACKEND_AMBIENT) {
        value.backend = SETTINGS_LED_BACKEND_BEAT;
    }
    int count = value.backend == SETTINGS_LED_BACKEND_BEAT
        ? led_beat_anim_count() : led_modes_count();
    if (count <= 0) {
        value.mode = 0;
    } else if (value.mode < 0 || value.mode >= count) {
        value.mode = value.backend == SETTINGS_LED_BACKEND_BEAT
            ? LED_BEAT_ANIM_PULSE : 0;
    }
    if (value.ambient_speed_percent < 10) value.ambient_speed_percent = 10;
    if (value.ambient_speed_percent > 250) value.ambient_speed_percent = 250;
    return value;
}

static settings_led_color_t sanitize_led_color(settings_led_color_t value)
{
    if (value.color_cycle > LED_COLOR_CYCLE_BIOHAZARD) {
        value.color_cycle = LED_COLOR_CYCLE_STATIC;
    }
    if (value.color_style > LED_COLOR_STYLE_PALETTE) {
        value.color_style = LED_COLOR_STYLE_MONO;
    }
    if (value.highlight_mode > LED_HIGHLIGHT_PEAKS) {
        value.highlight_mode = LED_HIGHLIGHT_OFF;
    }
    if (value.audio_energy_range > LED_AUDIO_ENERGY_RANGE_HIGH) {
        value.audio_energy_range = LED_AUDIO_ENERGY_RANGE_FULL;
    }
    return value;
}

static void apply_led_color(const settings_led_color_t *color)
{
    if (!color) return;
    led_beat_settings_t beat;
    led_beat_settings_get(&beat);
    beat.primary_r = color->primary_r;
    beat.primary_g = color->primary_g;
    beat.primary_b = color->primary_b;
    beat.secondary_r = color->secondary_r;
    beat.secondary_g = color->secondary_g;
    beat.secondary_b = color->secondary_b;
    beat.color_cycle = (led_color_cycle_t)color->color_cycle;
    beat.color_style = (led_color_style_t)color->color_style;
    beat.highlight_mode = (led_highlight_mode_t)color->highlight_mode;
    beat.audio_brightness_enabled = color->audio_brightness_enabled;
    beat.audio_energy_range = (led_audio_energy_range_t)color->audio_energy_range;
    beat.brightness = color->brightness;
    led_beat_settings_apply(&beat);

    led_modes_settings_t ambient;
    led_modes_settings_get(&ambient);
    ambient.primary_r = color->primary_r;
    ambient.primary_g = color->primary_g;
    ambient.primary_b = color->primary_b;
    ambient.secondary_r = color->secondary_r;
    ambient.secondary_g = color->secondary_g;
    ambient.secondary_b = color->secondary_b;
    ambient.color_cycle = (led_color_cycle_t)color->color_cycle;
    ambient.color_style = (led_color_style_t)color->color_style;
    ambient.highlight_mode = (led_highlight_mode_t)color->highlight_mode;
    ambient.brightness = color->brightness;
    led_modes_settings_apply(&ambient);
}

static void apply_led_animation_parameters(const settings_led_animation_t *animation)
{
    if (!animation) return;
    led_beat_settings_t beat;
    led_beat_settings_get(&beat);
    beat.plane_background_enabled = animation->plane_background_enabled;
    beat.ring_background_enabled = animation->ring_background_enabled;
    led_beat_settings_apply(&beat);

    led_modes_settings_t ambient;
    led_modes_settings_get(&ambient);
    ambient.speed_percent = animation->ambient_speed_percent;
    ambient.plane_background_enabled = animation->plane_background_enabled;
    ambient.ring_background_enabled = animation->ring_background_enabled;
    led_modes_settings_apply(&ambient);
}

static esp_err_t select_led_animation(const settings_led_animation_t *animation)
{
    if (!animation) return ESP_ERR_INVALID_ARG;
    if (animation->backend == SETTINGS_LED_BACKEND_AMBIENT) {
        esp_err_t err = led_modes_start();
        if (err != ESP_OK) return err;
        led_beat_enable(false);
        led_modes_set_sync(true);
        led_modes_set(animation->mode);
        led_modes_enable(true);
    } else {
        led_modes_enable(false);
        led_modes_set_sync(false);
        led_beat_anim_set((led_beat_anim_t)animation->mode);
        led_beat_enable(true);
    }
    return ESP_OK;
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

static bool settings_load_from_sd(settings_t *out)
{
    if (!out) return false;
    char path[96];
    if (storage_sd_make_path(path, sizeof(path), SETTINGS_FILE_NAME) != ESP_OK) {
        return false;
    }

    FILE *f = fopen(path, "r");
    if (!f) return false;

    settings_t tmp = *out;
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
        } else if (strcmp(key, "direction_icons") == 0) {
            if (strcmp(val, "2048") == 0) {
                tmp.use_2048_direction_icons = true;
            } else if (strcmp(val, "shell") == 0) {
                tmp.use_2048_direction_icons = false;
            } else {
                char *end = NULL;
                long mode = strtol(val, &end, 10);
                if (end != val) {
                    tmp.use_2048_direction_icons = (mode != 0);
                }
            }
        } else if (strcmp(key, "led_backend") == 0) {
            if (strcmp(val, "ambient") == 0) {
                tmp.led_animation.backend = SETTINGS_LED_BACKEND_AMBIENT;
            } else if (strcmp(val, "beat") == 0) {
                tmp.led_animation.backend = SETTINGS_LED_BACKEND_BEAT;
            }
        } else if (strcmp(key, "led_mode") == 0) {
            long mode = 0;
            if (parse_long(val, &mode) && mode >= 0 && mode <= INT_MAX) {
                tmp.led_animation.mode = (int)mode;
            }
        } else if (strcmp(key, "led_speed") == 0) {
            long speed = 0;
            if (parse_long(val, &speed) && speed >= 0 && speed <= UINT8_MAX) {
                tmp.led_animation.ambient_speed_percent = (uint8_t)speed;
            }
        } else if (strcmp(key, "led_plane_bg") == 0) {
            long enabled = 0;
            if (parse_long(val, &enabled)) tmp.led_animation.plane_background_enabled = enabled != 0;
        } else if (strcmp(key, "led_ring_bg") == 0) {
            long enabled = 0;
            if (parse_long(val, &enabled)) tmp.led_animation.ring_background_enabled = enabled != 0;
        } else if (strcmp(key, "led_primary") == 0) {
            (void)parse_rgb(val, &tmp.led_color.primary_r,
                            &tmp.led_color.primary_g, &tmp.led_color.primary_b);
        } else if (strcmp(key, "led_secondary") == 0) {
            (void)parse_rgb(val, &tmp.led_color.secondary_r,
                            &tmp.led_color.secondary_g, &tmp.led_color.secondary_b);
        } else if (strcmp(key, "led_color_cycle") == 0 ||
                   strcmp(key, "led_color_style") == 0 ||
                   strcmp(key, "led_highlight") == 0 ||
                   strcmp(key, "led_audio_range") == 0 ||
                   strcmp(key, "led_brightness") == 0) {
            long value = 0;
            if (!parse_long(val, &value) || value < 0 || value > UINT8_MAX) continue;
            if (strcmp(key, "led_color_cycle") == 0) tmp.led_color.color_cycle = (uint8_t)value;
            else if (strcmp(key, "led_color_style") == 0) tmp.led_color.color_style = (uint8_t)value;
            else if (strcmp(key, "led_highlight") == 0) tmp.led_color.highlight_mode = (uint8_t)value;
            else if (strcmp(key, "led_audio_range") == 0) tmp.led_color.audio_energy_range = (uint8_t)value;
            else tmp.led_color.brightness = (uint8_t)value;
        } else if (strcmp(key, "led_audio_brightness") == 0) {
            long enabled = 0;
            if (parse_long(val, &enabled)) tmp.led_color.audio_brightness_enabled = enabled != 0;
        }
    }

    fclose(f);
    tmp.led_animation = sanitize_led_animation(tmp.led_animation);
    tmp.led_color = sanitize_led_color(tmp.led_color);
    *out = tmp;
    return true;
}

static void settings_save_to_sd(const settings_t *s)
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
    fprintf(f, "direction_icons=%s\n", s->use_2048_direction_icons ? "2048" : "shell");
    fprintf(f, "led_backend=%s\n",
            s->led_animation.backend == SETTINGS_LED_BACKEND_AMBIENT ? "ambient" : "beat");
    fprintf(f, "led_mode=%d\n", s->led_animation.mode);
    fprintf(f, "led_speed=%u\n", (unsigned)s->led_animation.ambient_speed_percent);
    fprintf(f, "led_plane_bg=%d\n", s->led_animation.plane_background_enabled ? 1 : 0);
    fprintf(f, "led_ring_bg=%d\n", s->led_animation.ring_background_enabled ? 1 : 0);
    fprintf(f, "led_primary=%u,%u,%u\n", (unsigned)s->led_color.primary_r,
            (unsigned)s->led_color.primary_g, (unsigned)s->led_color.primary_b);
    fprintf(f, "led_secondary=%u,%u,%u\n", (unsigned)s->led_color.secondary_r,
            (unsigned)s->led_color.secondary_g, (unsigned)s->led_color.secondary_b);
    fprintf(f, "led_color_cycle=%u\n", (unsigned)s->led_color.color_cycle);
    fprintf(f, "led_color_style=%u\n", (unsigned)s->led_color.color_style);
    fprintf(f, "led_highlight=%u\n", (unsigned)s->led_color.highlight_mode);
    fprintf(f, "led_audio_brightness=%d\n", s->led_color.audio_brightness_enabled ? 1 : 0);
    fprintf(f, "led_audio_range=%u\n", (unsigned)s->led_color.audio_energy_range);
    fprintf(f, "led_brightness=%u\n", (unsigned)s->led_color.brightness);
    fflush(f);
    int fd = fileno(f);
    if (fd >= 0) fsync(fd);
    fclose(f);
}

void settings_init(void)
{
    settings_t loaded = s_settings;
    esp_err_t sd = storage_mount_sd();
    if (sd == ESP_OK) {
        if (settings_load_from_sd(&loaded)) {
            s_settings = loaded;
        }
    } else {
        ESP_LOGW(TAG, "SD mount failed (%s); using defaults", esp_err_to_name(sd));
    }

    apply_audio_volume(&s_settings);
    apply_shell_preferences(&s_settings);
}

const settings_t *settings_get(void)
{
    return &s_settings;
}

void settings_set_audio(float speaker_volume,
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

void settings_set_direction_icons(bool use_2048_direction_icons, bool persist)
{
    s_settings.use_2048_direction_icons = use_2048_direction_icons;
    apply_shell_preferences(&s_settings);
    if (persist) {
        settings_save_to_sd(&s_settings);
    }
}

esp_err_t settings_apply_led_runtime(void)
{
    s_settings.led_animation = sanitize_led_animation(s_settings.led_animation);
    s_settings.led_color = sanitize_led_color(s_settings.led_color);
    apply_led_color(&s_settings.led_color);
    apply_led_animation_parameters(&s_settings.led_animation);
    return select_led_animation(&s_settings.led_animation);
}

void settings_set_led_animation(const settings_led_animation_t *animation,
                                bool persist)
{
    if (!animation) return;
    settings_led_animation_t previous = s_settings.led_animation;
    s_settings.led_animation = sanitize_led_animation(*animation);
    apply_led_animation_parameters(&s_settings.led_animation);
    if (previous.backend != s_settings.led_animation.backend ||
        previous.mode != s_settings.led_animation.mode) {
        esp_err_t err = select_led_animation(&s_settings.led_animation);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "LED animation apply failed: %s", esp_err_to_name(err));
        }
    }
    if (persist) settings_save_to_sd(&s_settings);
}

void settings_set_led_color(const settings_led_color_t *color, bool persist)
{
    if (!color) return;
    s_settings.led_color = sanitize_led_color(*color);
    apply_led_color(&s_settings.led_color);
    if (persist) settings_save_to_sd(&s_settings);
}
