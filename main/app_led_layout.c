#include "app_shell.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"

#include "led.h"
#include "led_layout.h"
#include "led_modes.h"

const shell_legend_t LED_LAYOUT_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
};

enum {
    LED_LAYOUT_FIELD_BACK = 0,
    LED_LAYOUT_FIELD_SIDE,
    LED_LAYOUT_FIELD_PART,
    LED_LAYOUT_FIELD_LENGTH,
    LED_LAYOUT_FIELD_REVERSED,
    LED_LAYOUT_FIELD_PREVIEW,
    LED_LAYOUT_FIELD_SAVE,
    LED_LAYOUT_FIELD_RELOAD,
    LED_LAYOUT_FIELD_RESET,
    LED_LAYOUT_FIELD_COUNT,
};

typedef enum {
    LED_LAYOUT_PREVIEW_OFF = 0,
    LED_LAYOUT_PREVIEW_SECTION,
    LED_LAYOUT_PREVIEW_RUNNER,
} led_layout_preview_mode_t;

typedef struct {
    size_t field;
    uint8_t strip;
    size_t strip_section;
    led_layout_preview_mode_t preview;
    size_t runner_offset;
    int64_t last_runner_step_us;
    int64_t status_until_us;
    bool dirty;
    char status[28];
} led_layout_app_state_t;

static const char *TAG = "app_led_layout";
static led_layout_app_state_t s_layout_app = {
    .field = LED_LAYOUT_FIELD_PART,
    .strip = 0,
    .strip_section = 0,
    .preview = LED_LAYOUT_PREVIEW_RUNNER,
    .runner_offset = 0,
    .last_runner_step_us = 0,
    .status_until_us = 0,
    .dirty = false,
    .status = "",
};
static EXT_RAM_BSS_ATTR uint8_t s_preview_frame[LED_LAYOUT_MAX_PIXELS * 3];

static inline const char *preview_name(led_layout_preview_mode_t mode)
{
    switch (mode) {
        case LED_LAYOUT_PREVIEW_SECTION: return "sec";
        case LED_LAYOUT_PREVIEW_RUNNER: return "run";
        case LED_LAYOUT_PREVIEW_OFF:
        default:
            return "off";
    }
}

static void layout_format_name(const char *src, char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return;
    if (!src) src = "";

    if (strcmp(src, "back_left_vertebra") == 0) {
        snprintf(out, out_sz, "back left vertebrae");
        return;
    }
    if (strcmp(src, "back_right_vertebra") == 0) {
        snprintf(out, out_sz, "back right vertebrae");
        return;
    }

    size_t w = 0;
    for (size_t r = 0; src[r] != '\0' && w + 1 < out_sz; ++r) {
        out[w++] = (src[r] == '_') ? ' ' : src[r];
    }
    out[w] = '\0';
}

static void layout_set_status(const char *prefix, esp_err_t err)
{
    if (!prefix) prefix = "status";
    if (err == ESP_OK) {
        snprintf(s_layout_app.status, sizeof(s_layout_app.status), "%s:ok", prefix);
    } else {
        snprintf(s_layout_app.status, sizeof(s_layout_app.status), "%s:%s", prefix, esp_err_to_name(err));
    }
    s_layout_app.status_until_us = esp_timer_get_time() + 2500000LL;
}

static size_t layout_strip_section_count(const led_layout_config_t *cfg, uint8_t strip)
{
    if (!cfg || strip >= cfg->strip_count) return 0;
    size_t count = 0;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        if (cfg->sections[i].strip == strip) {
            count++;
        }
    }
    return count;
}

static bool layout_global_section_index(const led_layout_config_t *cfg,
                                        uint8_t strip,
                                        size_t strip_section,
                                        size_t *out_global)
{
    if (!cfg || strip >= cfg->strip_count) return false;
    size_t local = 0;
    for (size_t i = 0; i < cfg->section_count; ++i) {
        if (cfg->sections[i].strip != strip) continue;
        if (local == strip_section) {
            if (out_global) *out_global = i;
            return true;
        }
        local++;
    }
    return false;
}

static void layout_clamp_selection(const led_layout_config_t *cfg)
{
    if (!cfg || cfg->section_count == 0 || cfg->strip_count == 0) {
        s_layout_app.strip = 0;
        s_layout_app.strip_section = 0;
        return;
    }

    if (s_layout_app.strip >= cfg->strip_count) {
        s_layout_app.strip = (uint8_t)(cfg->strip_count - 1);
    }

    size_t count = layout_strip_section_count(cfg, s_layout_app.strip);
    if (count == 0) {
        for (uint8_t strip = 0; strip < cfg->strip_count; ++strip) {
            count = layout_strip_section_count(cfg, strip);
            if (count > 0) {
                s_layout_app.strip = strip;
                break;
            }
        }
    }

    if (count == 0) {
        s_layout_app.strip_section = 0;
        return;
    }

    if (s_layout_app.strip_section >= count) {
        s_layout_app.strip_section = count - 1;
    }
}

static const led_layout_section_t *layout_selected_section(const led_layout_config_t *cfg,
                                                           size_t *out_global_idx)
{
    if (!cfg) return NULL;
    layout_clamp_selection(cfg);

    size_t global_idx = 0;
    if (!layout_global_section_index(cfg, s_layout_app.strip, s_layout_app.strip_section, &global_idx)) {
        return NULL;
    }
    if (out_global_idx) *out_global_idx = global_idx;
    return &cfg->sections[global_idx];
}

static void layout_render_preview(const led_layout_config_t *cfg)
{
    memset(s_preview_frame, 0, sizeof(s_preview_frame));
    if (!cfg || cfg->section_count == 0 || cfg->total_leds == 0) {
        (void)led_show_pixels(s_preview_frame, 1);
        return;
    }
    if (s_layout_app.preview == LED_LAYOUT_PREVIEW_OFF) {
        (void)led_show_pixels(s_preview_frame, cfg->total_leds);
        return;
    }

    size_t selected = 0;
    const led_layout_section_t *sec = layout_selected_section(cfg, &selected);
    if (!sec) {
        (void)led_show_pixels(s_preview_frame, cfg->total_leds);
        return;
    }

    for (size_t logical = 0; logical < cfg->total_leds; ++logical) {
        led_layout_mapping_t map;
        if (!led_layout_map_logical_from_config(cfg, logical, &map)) continue;
        if (map.section_index != selected) continue;

        uint8_t r = (sec->strip == 0) ? 0 : 110;
        uint8_t g = (sec->strip == 0) ? 120 : 40;
        uint8_t b = (sec->strip == 0) ? 120 : 0;
        if (s_layout_app.preview == LED_LAYOUT_PREVIEW_RUNNER) {
            r = 8;
            g = 20;
            b = 8;
            if (sec->length > 0 && map.section_offset == s_layout_app.runner_offset % sec->length) {
                r = 200;
                g = 200;
                b = 200;
            }
        }
        led_set_pixel_rgb(s_preview_frame, logical, r, g, b);
    }

    (void)led_show_pixels(s_preview_frame, cfg->total_leds);
}

static void layout_refresh_preview(void)
{
    led_layout_config_t cfg;
    led_layout_snapshot(&cfg);
    layout_render_preview(&cfg);
}

static void layout_cycle_field(bool next)
{
    if (next) {
        if (s_layout_app.field + 1 < LED_LAYOUT_FIELD_COUNT) {
            s_layout_app.field++;
        }
    } else if (s_layout_app.field > 0) {
        s_layout_app.field--;
    }
}

static void layout_cycle_section(bool next)
{
    led_layout_config_t cfg;
    led_layout_snapshot(&cfg);
    layout_clamp_selection(&cfg);

    size_t count = layout_strip_section_count(&cfg, s_layout_app.strip);
    if (count == 0) {
        s_layout_app.strip_section = 0;
        return;
    }

    if (next) {
        s_layout_app.strip_section = (s_layout_app.strip_section + 1) % count;
    } else if (s_layout_app.strip_section == 0) {
        s_layout_app.strip_section = count - 1;
    } else {
        s_layout_app.strip_section--;
    }
    s_layout_app.runner_offset = 0;
}

static void layout_cycle_strip(bool next)
{
    led_layout_config_t cfg;
    led_layout_snapshot(&cfg);
    if (cfg.strip_count == 0) {
        s_layout_app.strip = 0;
        s_layout_app.strip_section = 0;
        return;
    }

    if (next) {
        s_layout_app.strip = (uint8_t)((s_layout_app.strip + 1) % cfg.strip_count);
    } else if (s_layout_app.strip == 0) {
        s_layout_app.strip = (uint8_t)(cfg.strip_count - 1);
    } else {
        s_layout_app.strip--;
    }

    layout_clamp_selection(&cfg);
    s_layout_app.runner_offset = 0;
}

static void layout_apply_edit(bool increment, bool coarse)
{
    led_layout_config_t cfg;
    led_layout_snapshot(&cfg);
    size_t global_idx = 0;
    const led_layout_section_t *sec = layout_selected_section(&cfg, &global_idx);
    if (!sec) return;
    esp_err_t err = ESP_OK;

    switch (s_layout_app.field) {
        case LED_LAYOUT_FIELD_BACK:
            break;
        case LED_LAYOUT_FIELD_SIDE:
            layout_cycle_strip(increment);
            break;
        case LED_LAYOUT_FIELD_PART:
            layout_cycle_section(increment);
            break;
        case LED_LAYOUT_FIELD_LENGTH: {
            int delta = coarse ? 8 : 1;
            int next_len = (int)sec->length + (increment ? delta : -delta);
            if (next_len < 1) next_len = 1;
            err = led_layout_set_section_length(global_idx, (uint16_t)next_len);
            if (err == ESP_OK) s_layout_app.dirty = true;
            break;
        }
        case LED_LAYOUT_FIELD_REVERSED:
            err = led_layout_set_section_reversed(global_idx, !sec->reversed);
            if (err == ESP_OK) s_layout_app.dirty = true;
            break;
        case LED_LAYOUT_FIELD_PREVIEW:
            if (increment) {
                s_layout_app.preview = (led_layout_preview_mode_t)((s_layout_app.preview + 1) % 3);
            } else {
                s_layout_app.preview = (led_layout_preview_mode_t)((s_layout_app.preview + 2) % 3);
            }
            break;
        case LED_LAYOUT_FIELD_SAVE:
            if (increment) {
                err = led_layout_save();
                if (err == ESP_OK) s_layout_app.dirty = false;
                layout_set_status("save", err);
            }
            break;
        case LED_LAYOUT_FIELD_RELOAD:
            if (increment) {
                err = led_layout_reload();
                if (err == ESP_OK) s_layout_app.dirty = false;
                layout_set_status("reload", err);
                led_layout_snapshot(&cfg);
                layout_clamp_selection(&cfg);
            }
            break;
        case LED_LAYOUT_FIELD_RESET:
            if (increment) {
                err = led_layout_reset_default(false);
                if (err == ESP_OK) s_layout_app.dirty = true;
                layout_set_status("reset", err);
                led_layout_snapshot(&cfg);
                layout_clamp_selection(&cfg);
            }
            break;
        default:
            break;
    }

    if (err != ESP_OK &&
        s_layout_app.field != LED_LAYOUT_FIELD_SAVE &&
        s_layout_app.field != LED_LAYOUT_FIELD_RELOAD &&
        s_layout_app.field != LED_LAYOUT_FIELD_RESET) {
        layout_set_status("edit", err);
    }
}

void led_layout_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    memset(&s_layout_app, 0, sizeof(s_layout_app));
    s_layout_app.field = LED_LAYOUT_FIELD_PART;
    s_layout_app.preview = LED_LAYOUT_PREVIEW_RUNNER;

    esp_err_t err = led_layout_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_layout_init: %s", esp_err_to_name(err));
        layout_set_status("layout", err);
    }
    err = led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_init: %s", esp_err_to_name(err));
        layout_set_status("led", err);
    }

    led_modes_enable(false);
    led_beat_enable(false);
    system_state_set_led_mode(250, "layout");
    layout_refresh_preview();
}

void led_layout_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    memset(s_preview_frame, 0, sizeof(s_preview_frame));
    (void)led_show_pixels(s_preview_frame, 1);
}

void led_layout_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    bool coarse = (ev->type == INPUT_EVENT_LONG_PRESS);
    if (ev->type != INPUT_EVENT_PRESS && ev->type != INPUT_EVENT_LONG_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        layout_cycle_field(false);
        return;
    }
    if (ev->button == INPUT_BTN_B) {
        layout_cycle_field(true);
        return;
    }
    if (ev->button == INPUT_BTN_D) {
        if (s_layout_app.field == LED_LAYOUT_FIELD_BACK && ctx->request_switch) {
            ctx->request_switch("menu", ctx->request_user_data);
            return;
        }
        layout_apply_edit(true, coarse);
        layout_refresh_preview();
        return;
    }
    if (ev->button == INPUT_BTN_C) {
        if (s_layout_app.field != LED_LAYOUT_FIELD_BACK &&
            s_layout_app.field != LED_LAYOUT_FIELD_SAVE &&
            s_layout_app.field != LED_LAYOUT_FIELD_RELOAD &&
            s_layout_app.field != LED_LAYOUT_FIELD_RESET) {
            layout_apply_edit(false, coarse);
            layout_refresh_preview();
        }
    }
}

void led_layout_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    (void)dt_sec;

    if (s_layout_app.preview == LED_LAYOUT_PREVIEW_RUNNER) {
        led_layout_config_t cfg;
        led_layout_snapshot(&cfg);
        const led_layout_section_t *sec = layout_selected_section(&cfg, NULL);
        if (sec) {
            int64_t now = esp_timer_get_time();
            if (sec->length > 0 && (now - s_layout_app.last_runner_step_us) >= 70000LL) {
                s_layout_app.last_runner_step_us = now;
                s_layout_app.runner_offset = (s_layout_app.runner_offset + 1) % sec->length;
                layout_render_preview(&cfg);
            }
        }
    } else {
        layout_refresh_preview();
    }
}

void led_layout_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    (void)h;
    if (!fb) return;

    led_layout_config_t cfg;
    led_layout_snapshot(&cfg);
    layout_clamp_selection(&cfg);

    size_t strip_count = layout_strip_section_count(&cfg, s_layout_app.strip);
    const char *strip_name = (s_layout_app.strip < cfg.strip_count) ? cfg.strip_names[s_layout_app.strip] : "strip";

    char line[56];
    snprintf(line, sizeof(line), "%.11s %02u/%02u%s",
             strip_name,
             strip_count ? (unsigned)(s_layout_app.strip_section + 1) : 0u,
             (unsigned)strip_count,
             s_layout_app.dirty ? "*" : "");
    oled_draw_text3x5(fb, x + 2, y + 2, line);

    const led_layout_section_t *sec = NULL;
    if (cfg.section_count > 0) {
        sec = layout_selected_section(&cfg, NULL);
    }
    if (sec) {
        char pretty_name[28];
        layout_format_name(sec->name, pretty_name, sizeof(pretty_name));
        oled_draw_text3x5(fb, x + 2, y + 9, pretty_name);
    } else {
        oled_draw_text3x5(fb, x + 2, y + 9, "no sections");
    }

    if (esp_timer_get_time() < s_layout_app.status_until_us && s_layout_app.status[0]) {
        oled_draw_text3x5(fb, x + 2, y + 16, s_layout_app.status);
    } else if (sec) {
        uint16_t phys_end = (uint16_t)(sec->physical_start + sec->length - 1u);
        snprintf(line, sizeof(line), "%.11s %u-%u len%u",
                 strip_name,
                 (unsigned)sec->physical_start,
                 (unsigned)phys_end,
                 (unsigned)sec->length);
        oled_draw_text3x5(fb, x + 2, y + 16, line);
    }

    char fields[LED_LAYOUT_FIELD_COUNT][28];
    snprintf(fields[LED_LAYOUT_FIELD_BACK], sizeof(fields[LED_LAYOUT_FIELD_BACK]), "back");
    snprintf(fields[LED_LAYOUT_FIELD_SIDE], sizeof(fields[LED_LAYOUT_FIELD_SIDE]), "side:%s", strip_name);
    snprintf(fields[LED_LAYOUT_FIELD_PART], sizeof(fields[LED_LAYOUT_FIELD_PART]), "part:%02u/%02u",
             strip_count ? (unsigned)(s_layout_app.strip_section + 1) : 0u,
             (unsigned)strip_count);
    snprintf(fields[LED_LAYOUT_FIELD_LENGTH], sizeof(fields[LED_LAYOUT_FIELD_LENGTH]), "len:%u",
             sec ? (unsigned)sec->length : 0u);
    snprintf(fields[LED_LAYOUT_FIELD_REVERSED], sizeof(fields[LED_LAYOUT_FIELD_REVERSED]), "rev:%s",
             sec && sec->reversed ? "yes" : "no");
    snprintf(fields[LED_LAYOUT_FIELD_PREVIEW], sizeof(fields[LED_LAYOUT_FIELD_PREVIEW]), "preview:%s",
             preview_name(s_layout_app.preview));
    snprintf(fields[LED_LAYOUT_FIELD_SAVE], sizeof(fields[LED_LAYOUT_FIELD_SAVE]), "save%s",
             s_layout_app.dirty ? "*" : "");
    snprintf(fields[LED_LAYOUT_FIELD_RELOAD], sizeof(fields[LED_LAYOUT_FIELD_RELOAD]), "reload");
    snprintf(fields[LED_LAYOUT_FIELD_RESET], sizeof(fields[LED_LAYOUT_FIELD_RESET]), "reset def");

    size_t first = 0;
    const size_t visible = 4;
    if (s_layout_app.field > 1) {
        first = s_layout_app.field - 1;
    }
    if (first + visible > LED_LAYOUT_FIELD_COUNT) {
        first = LED_LAYOUT_FIELD_COUNT - visible;
    }

    for (size_t row = 0; row < visible; ++row) {
        size_t idx = first + row;
        if (idx >= LED_LAYOUT_FIELD_COUNT) break;
        snprintf(line, sizeof(line), "%c %s",
                 (idx == s_layout_app.field) ? '>' : ' ',
                 fields[idx]);
        oled_draw_text3x5(fb, x + 2, y + 23 + (int)row * 7, line);
    }
}
