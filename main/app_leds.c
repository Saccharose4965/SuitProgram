#include "app_shell.h"

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "led.h"
#include "led_modes.h"

const shell_legend_t LEDS_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_CUSTOM2 },
};

enum {
    LEDS_PAGE_AUDIO = 0,
    LEDS_PAGE_CUSTOM = 1,
};

enum {
    LEDS_CFG_PRESET = 0,
    LEDS_CFG_R,
    LEDS_CFG_G,
    LEDS_CFG_B,
    LEDS_CFG_BRIGHTNESS,
    LEDS_CFG_COUNT,
};

typedef struct {
    const char *name;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} leds_color_preset_t;

static const leds_color_preset_t s_color_presets[] = {
    { "tron",    0, 180, 255 },
    { "ice",   120, 220, 255 },
    { "magenta",255,  40, 180 },
    { "amber", 255, 120,   0 },
    { "white", 220, 220, 220 },
};
static const int kColorPresetCount = (int)(sizeof(s_color_presets) / sizeof(s_color_presets[0]));

static const char *TAG = "app_leds";

#define LEDS_MAX_AUDIO_ENTRIES 16
#define LEDS_MAX_CUSTOM_ENTRIES 28
static shell_menu_entry_t s_audio_entries[1 + LEDS_MAX_AUDIO_ENTRIES];
static shell_menu_entry_t s_custom_entries[1 + LEDS_MAX_CUSTOM_ENTRIES];

static char s_preset_label[24];
static char s_r_label[12];
static char s_g_label[12];
static char s_b_label[12];
static char s_bright_label[14];

typedef struct {
    int page;
    size_t audio_sel;  // includes "Back" at index 0
    size_t custom_sel; // includes "Back" at index 0
    int audio_mode;
    int custom_mode;
    int preset_idx; // -1 for manual
    uint8_t color_r;
    uint8_t color_g;
    uint8_t color_b;
    uint8_t brightness;
} leds_state_t;
static leds_state_t s_leds = {0};

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static size_t clamp_sel(size_t sel, size_t count)
{
    if (count == 0) return 0;
    if (sel >= count) return count - 1;
    return sel;
}

static int find_preset_exact(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < kColorPresetCount; ++i) {
        if (s_color_presets[i].r == r &&
            s_color_presets[i].g == g &&
            s_color_presets[i].b == b) {
            return i;
        }
    }
    return -1;
}

static uint8_t step_u8_clamp(uint8_t value, int step, uint8_t lo, uint8_t hi)
{
    int next = (int)value + step;
    if (next < (int)lo) next = lo;
    if (next > (int)hi) next = hi;
    return (uint8_t)next;
}

static void apply_preset_idx(int idx)
{
    idx = clamp_int(idx, 0, kColorPresetCount - 1);
    s_leds.preset_idx = idx;
    s_leds.color_r = s_color_presets[idx].r;
    s_leds.color_g = s_color_presets[idx].g;
    s_leds.color_b = s_color_presets[idx].b;
}

static void leds_apply_color(void)
{
    led_beat_color_set(s_leds.color_r, s_leds.color_g, s_leds.color_b);
    led_beat_brightness_set(s_leds.brightness);
    led_modes_set_primary_color(s_leds.color_r, s_leds.color_g, s_leds.color_b);
    led_modes_set_brightness(s_leds.brightness);
    s_leds.preset_idx = find_preset_exact(s_leds.color_r, s_leds.color_g, s_leds.color_b);
}

static void leds_load_color_state(void)
{
    led_beat_color_get(&s_leds.color_r, &s_leds.color_g, &s_leds.color_b);
    s_leds.preset_idx = find_preset_exact(s_leds.color_r, s_leds.color_g, s_leds.color_b);
    s_leds.brightness = led_beat_brightness_get();
    if (s_leds.brightness < 8) s_leds.brightness = 8;
}

static void refresh_color_labels(void)
{
    const char *preset_name = (s_leds.preset_idx >= 0 && s_leds.preset_idx < kColorPresetCount)
        ? s_color_presets[s_leds.preset_idx].name
        : "manual";
    snprintf(s_preset_label, sizeof(s_preset_label), "preset:%s", preset_name);
    snprintf(s_r_label, sizeof(s_r_label), "R:%u", (unsigned)s_leds.color_r);
    snprintf(s_g_label, sizeof(s_g_label), "G:%u", (unsigned)s_leds.color_g);
    snprintf(s_b_label, sizeof(s_b_label), "B:%u", (unsigned)s_leds.color_b);
    snprintf(s_bright_label, sizeof(s_bright_label), "bright:%u", (unsigned)s_leds.brightness);
}

static void append_color_entries(shell_menu_entry_t *entries, size_t *n, size_t cap)
{
    if (!entries || !n) return;
    refresh_color_labels();
    if (*n < cap) entries[(*n)++] = (shell_menu_entry_t){ .id = "cfg_preset", .label = s_preset_label };
    if (*n < cap) entries[(*n)++] = (shell_menu_entry_t){ .id = "cfg_r", .label = s_r_label };
    if (*n < cap) entries[(*n)++] = (shell_menu_entry_t){ .id = "cfg_g", .label = s_g_label };
    if (*n < cap) entries[(*n)++] = (shell_menu_entry_t){ .id = "cfg_b", .label = s_b_label };
    if (*n < cap) entries[(*n)++] = (shell_menu_entry_t){ .id = "cfg_brightness", .label = s_bright_label };
}

static size_t build_audio_entries(void)
{
    size_t n = 0;
    const size_t cap = sizeof(s_audio_entries) / sizeof(s_audio_entries[0]);
    s_audio_entries[n++] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    int count = led_beat_anim_count();
    for (int i = 0; i < count && n < cap; ++i) {
        const char *name = led_beat_anim_name(i);
        if (!name) name = "?";
        s_audio_entries[n++] = (shell_menu_entry_t){ .id = name, .label = name };
    }
    append_color_entries(s_audio_entries, &n, cap);
    return n;
}

static size_t build_custom_entries(void)
{
    size_t n = 0;
    const size_t cap = sizeof(s_custom_entries) / sizeof(s_custom_entries[0]);
    s_custom_entries[n++] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    int count = led_modes_count();
    for (int i = 0; i < count && n < cap; ++i) {
        const char *name = led_modes_name(i);
        if (!name) name = "?";
        s_custom_entries[n++] = (shell_menu_entry_t){ .id = name, .label = name };
    }
    append_color_entries(s_custom_entries, &n, cap);
    return n;
}

static int mode_count_for_page(int page)
{
    return (page == LEDS_PAGE_AUDIO) ? led_beat_anim_count() : led_modes_count();
}

static void leds_apply_audio(void)
{
    int count = led_beat_anim_count();
    if (count <= 0) return;
    s_leds.audio_mode = clamp_int(s_leds.audio_mode, 0, count - 1);
    led_beat_anim_set((led_beat_anim_t)s_leds.audio_mode);
    led_beat_enable(true);
    led_modes_enable(false);

    const char *name = led_beat_anim_name(s_leds.audio_mode);
    char label[24];
    snprintf(label, sizeof(label), "beat:%s", name ? name : "?");
    system_state_set_led_mode(100 + s_leds.audio_mode, label);
}

static void leds_apply_custom(void)
{
    int count = led_modes_count();
    if (count <= 0) return;
    s_leds.custom_mode = clamp_int(s_leds.custom_mode, 0, count - 1);
    led_modes_set(s_leds.custom_mode);
    led_modes_enable(true);
    led_beat_enable(false);

    const char *name = led_modes_name(s_leds.custom_mode);
    system_state_set_led_mode(s_leds.custom_mode, name ? name : "custom");
}

static void leds_apply_mode_for_page(void)
{
    if (s_leds.page == LEDS_PAGE_AUDIO) {
        leds_apply_audio();
    } else {
        leds_apply_custom();
    }
}

static void leds_cycle_cfg(int cfg_idx, bool inc)
{
    const int rgb_step = 16;
    const int bright_step = 16;
    switch (cfg_idx) {
        case LEDS_CFG_PRESET: {
            int idx = s_leds.preset_idx;
            if (idx < 0) idx = inc ? 0 : (kColorPresetCount - 1);
            else idx = (idx + (inc ? 1 : -1) + kColorPresetCount) % kColorPresetCount;
            apply_preset_idx(idx);
            break;
        }
        case LEDS_CFG_R:
            s_leds.color_r = step_u8_clamp(s_leds.color_r, inc ? rgb_step : -rgb_step, 0, 255);
            break;
        case LEDS_CFG_G:
            s_leds.color_g = step_u8_clamp(s_leds.color_g, inc ? rgb_step : -rgb_step, 0, 255);
            break;
        case LEDS_CFG_B:
            s_leds.color_b = step_u8_clamp(s_leds.color_b, inc ? rgb_step : -rgb_step, 0, 255);
            break;
        case LEDS_CFG_BRIGHTNESS:
            s_leds.brightness = step_u8_clamp(s_leds.brightness,
                                              inc ? bright_step : -bright_step,
                                              8, 255);
            break;
        default:
            return;
    }

    if (cfg_idx != LEDS_CFG_PRESET) {
        s_leds.preset_idx = find_preset_exact(s_leds.color_r, s_leds.color_g, s_leds.color_b);
    }
    leds_apply_color();
}

static void leds_init_page(shell_app_context_t *ctx, int page)
{
    (void)ctx;
    esp_err_t err = led_modes_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_modes_start failed: %s", esp_err_to_name(err));
    }

    s_leds.page = page;
    int audio_count = led_beat_anim_count();
    int custom_count = led_modes_count();
    s_leds.audio_mode = (audio_count > 0) ? clamp_int((int)led_beat_anim_get(), 0, audio_count - 1) : 0;
    s_leds.custom_mode = (custom_count > 0) ? clamp_int(led_modes_current(), 0, custom_count - 1) : 0;
    s_leds.audio_sel = (size_t)s_leds.audio_mode + 1;
    s_leds.custom_sel = (size_t)s_leds.custom_mode + 1;

    leds_load_color_state();
    leds_apply_color();

    if (s_leds.page == LEDS_PAGE_AUDIO) {
        shell_ui_menu_reset(s_leds.audio_sel);
    } else {
        shell_ui_menu_reset(s_leds.custom_sel);
    }
}

void leds_audio_app_init(shell_app_context_t *ctx)
{
    leds_init_page(ctx, LEDS_PAGE_AUDIO);
}

void leds_custom_app_init(shell_app_context_t *ctx)
{
    leds_init_page(ctx, LEDS_PAGE_CUSTOM);
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type != INPUT_EVENT_PRESS) return;

    shell_menu_entry_t *entries = NULL;
    size_t count = 0;
    size_t *sel = NULL;
    if (s_leds.page == LEDS_PAGE_AUDIO) {
        entries = s_audio_entries;
        count = build_audio_entries();
        sel = &s_leds.audio_sel;
    } else {
        entries = s_custom_entries;
        count = build_custom_entries();
        sel = &s_leds.custom_sel;
    }
    if (!entries || !sel || count == 0) return;
    *sel = clamp_sel(*sel, count);

    if (ev->button == INPUT_BTN_A && *sel > 0) {
        (*sel)--;
        shell_ui_menu_reset(*sel);
        return;
    }
    if (ev->button == INPUT_BTN_B && (*sel + 1) < count) {
        (*sel)++;
        shell_ui_menu_reset(*sel);
        return;
    }

    int mode_count = mode_count_for_page(s_leds.page);
    size_t first_cfg = 1u + (size_t)(mode_count > 0 ? mode_count : 0);

    if (ev->button == INPUT_BTN_D) {
        if (*sel == 0) {
            if (ctx->request_switch) {
                ctx->request_switch("menu", ctx->request_user_data);
            }
            return;
        }
        if (*sel < first_cfg) {
            if (s_leds.page == LEDS_PAGE_AUDIO) {
                s_leds.audio_mode = (int)(*sel - 1);
            } else {
                s_leds.custom_mode = (int)(*sel - 1);
            }
            leds_apply_mode_for_page();
            return;
        }
        int cfg_idx = (int)(*sel - first_cfg);
        leds_cycle_cfg(cfg_idx, true);
        return;
    }

    if (ev->button == INPUT_BTN_C) {
        if (*sel >= first_cfg) {
            int cfg_idx = (int)(*sel - first_cfg);
            leds_cycle_cfg(cfg_idx, false);
        }
    }
}

void leds_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    const shell_menu_entry_t *entries = NULL;
    size_t count = 0;
    size_t selected = 0;
    const char *title = NULL;

    if (s_leds.page == LEDS_PAGE_AUDIO) {
        count = build_audio_entries();
        entries = s_audio_entries;
        selected = clamp_sel(s_leds.audio_sel, count);
        s_leds.audio_sel = selected;
        title = "LED / AUDIO";
    } else {
        count = build_custom_entries();
        entries = s_custom_entries;
        selected = clamp_sel(s_leds.custom_sel, count);
        s_leds.custom_sel = selected;
        title = "LED / CUSTOM";
    }

    shell_menu_view_t view = {
        .entries = entries,
        .count = count,
        .selected = selected,
        .title = title,
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}
