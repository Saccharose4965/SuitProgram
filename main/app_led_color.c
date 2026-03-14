#include "app_shell.h"

#include <stdio.h>

#include "led.h"
#include "led_modes.h"

#include "app_led_support.h"

const shell_legend_t LED_COLOR_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
};

enum {
    LED_COLOR_CFG_PRESET = 0,
    LED_COLOR_CFG_STYLE,
    LED_COLOR_CFG_HIGHLIGHT,
    LED_COLOR_CFG_R,
    LED_COLOR_CFG_G,
    LED_COLOR_CFG_B,
    LED_COLOR_CFG_R2,
    LED_COLOR_CFG_G2,
    LED_COLOR_CFG_B2,
    LED_COLOR_CFG_AUDIO_INPUT,
    LED_COLOR_CFG_BRIGHTNESS,
};

typedef enum {
    LED_COLOR_ENTRY_BACK = 0,
    LED_COLOR_ENTRY_CFG,
} led_color_entry_kind_t;

typedef struct {
    led_color_entry_kind_t kind;
    int value;
} led_color_meta_t;

typedef struct {
    const char *name;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t r2;
    uint8_t g2;
    uint8_t b2;
    led_color_cycle_t cycle;
    led_color_style_t style;
    led_highlight_mode_t highlight;
} led_color_preset_t;

typedef struct {
    size_t selected;
    int preset_idx;
    uint8_t color_r;
    uint8_t color_g;
    uint8_t color_b;
    uint8_t color2_r;
    uint8_t color2_g;
    uint8_t color2_b;
    led_color_cycle_t color_cycle;
    led_color_style_t color_style;
    led_highlight_mode_t highlight_mode;
    bool audio_brightness_enabled;
    led_audio_energy_range_t audio_energy_range;
    uint8_t brightness;
} led_color_state_t;

static const led_color_preset_t s_color_presets[] = {
    { "signal", 255,   0,   0, 255, 160,   0, LED_COLOR_CYCLE_STATIC,    LED_COLOR_STYLE_MONO,    LED_HIGHLIGHT_OFF   },
    { "hazard", 255, 140,   0, 255, 255,   0, LED_COLOR_CYCLE_STATIC,    LED_COLOR_STYLE_MONO,    LED_HIGHLIGHT_OFF   },
    { "acid",   170, 255,   0,   0, 255,  96, LED_COLOR_CYCLE_STATIC,    LED_COLOR_STYLE_MONO,    LED_HIGHLIGHT_OFF   },
    { "cobalt",   0,  72, 255, 180,   0, 255, LED_COLOR_CYCLE_STATIC,    LED_COLOR_STYLE_MONO,    LED_HIGHLIGHT_OFF   },
    { "fuchsia",255,   0, 170,   0, 210, 255, LED_COLOR_CYCLE_STATIC,    LED_COLOR_STYLE_MONO,    LED_HIGHLIGHT_OFF   },
    { "bone",   255, 236, 208, 255, 196, 128, LED_COLOR_CYCLE_STATIC,    LED_COLOR_STYLE_MONO,    LED_HIGHLIGHT_OFF   },
    { "rainbow",255,   0,   0,   0, 255, 192, LED_COLOR_CYCLE_RAINBOW,   LED_COLOR_STYLE_PALETTE, LED_HIGHLIGHT_OFF   },
    { "siren",  255,   0,   0,   0,  48, 255, LED_COLOR_CYCLE_SIREN,     LED_COLOR_STYLE_PALETTE, LED_HIGHLIGHT_OFF   },
    { "arcade", 255,   0, 255,   0, 250, 255, LED_COLOR_CYCLE_ARCADE,    LED_COLOR_STYLE_PALETTE, LED_HIGHLIGHT_OFF   },
    { "inferno",255,  48,   0, 255, 210,  24, LED_COLOR_CYCLE_INFERNO,   LED_COLOR_STYLE_PALETTE, LED_HIGHLIGHT_OFF   },
    { "biohaz",  64, 255,   0, 190, 255,   0, LED_COLOR_CYCLE_BIOHAZARD, LED_COLOR_STYLE_PALETTE, LED_HIGHLIGHT_OFF   },
};
static const int kColorPresetCount = (int)(sizeof(s_color_presets) / sizeof(s_color_presets[0]));

#define LED_COLOR_ENTRY_COUNT 16
static shell_menu_entry_t s_entries[LED_COLOR_ENTRY_COUNT];
static led_color_meta_t s_meta[LED_COLOR_ENTRY_COUNT];
static char s_preset_label[24];
static char s_style_label[16];
static char s_hi_label[14];
static char s_r_label[12];
static char s_g_label[12];
static char s_b_label[12];
static char s_r2_label[12];
static char s_g2_label[12];
static char s_b2_label[12];
static char s_audio_label[18];
static char s_bright_label[14];
static led_color_state_t s_led_color = {0};

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

static const char *color_style_name(led_color_style_t style)
{
    switch (style) {
        case LED_COLOR_STYLE_DUO: return "duo";
        case LED_COLOR_STYLE_PALETTE: return "palette";
        case LED_COLOR_STYLE_MONO:
        default: return "mono";
    }
}

static const char *highlight_mode_name(led_highlight_mode_t mode)
{
    switch (mode) {
        case LED_HIGHLIGHT_PEAKS: return "peaks";
        case LED_HIGHLIGHT_OFF:
        default: return "off";
    }
}

static const char *audio_energy_range_name(led_audio_energy_range_t range)
{
    switch (range) {
        case LED_AUDIO_ENERGY_RANGE_MID: return "127-255";
        case LED_AUDIO_ENERGY_RANGE_HIGH: return "191-255";
        case LED_AUDIO_ENERGY_RANGE_FULL:
        default:
            return "0-255";
    }
}

static int audio_input_mode_get(void)
{
    if (!s_led_color.audio_brightness_enabled) {
        return 0;
    }

    switch (s_led_color.audio_energy_range) {
        case LED_AUDIO_ENERGY_RANGE_MID: return 2;
        case LED_AUDIO_ENERGY_RANGE_HIGH: return 3;
        case LED_AUDIO_ENERGY_RANGE_FULL:
        default:
            return 1;
    }
}

static void audio_input_mode_set(int mode)
{
    mode = clamp_int(mode, 0, 3);
    s_led_color.audio_brightness_enabled = (mode != 0);
    switch (mode) {
        case 2:
            s_led_color.audio_energy_range = LED_AUDIO_ENERGY_RANGE_MID;
            break;
        case 3:
            s_led_color.audio_energy_range = LED_AUDIO_ENERGY_RANGE_HIGH;
            break;
        case 1:
        case 0:
        default:
            s_led_color.audio_energy_range = LED_AUDIO_ENERGY_RANGE_FULL;
            break;
    }
}

static int find_preset_match(led_color_cycle_t cycle,
                             led_color_style_t style,
                             led_highlight_mode_t highlight,
                             uint8_t r, uint8_t g, uint8_t b,
                             uint8_t r2, uint8_t g2, uint8_t b2)
{
    for (int i = 0; i < kColorPresetCount; ++i) {
        if (s_color_presets[i].cycle != cycle) continue;
        if (s_color_presets[i].style != style) continue;
        if (s_color_presets[i].highlight != highlight) continue;
        if (s_color_presets[i].r != r || s_color_presets[i].g != g || s_color_presets[i].b != b) continue;
        if (s_color_presets[i].r2 != r2 || s_color_presets[i].g2 != g2 || s_color_presets[i].b2 != b2) continue;
        return i;
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

static int color_step_for_value(uint8_t value)
{
    if (value < 16u) return 1;
    if (value < 48u) return 2;
    if (value < 96u) return 4;
    if (value < 160u) return 8;
    return 16;
}

static uint8_t step_color_u8(uint8_t value, bool inc)
{
    int step = color_step_for_value(value);
    return step_u8_clamp(value, inc ? step : -step, 0, 255);
}

static void apply_preset_idx(int idx)
{
    idx = clamp_int(idx, 0, kColorPresetCount - 1);
    s_led_color.preset_idx = idx;
    s_led_color.color_r = s_color_presets[idx].r;
    s_led_color.color_g = s_color_presets[idx].g;
    s_led_color.color_b = s_color_presets[idx].b;
    s_led_color.color2_r = s_color_presets[idx].r2;
    s_led_color.color2_g = s_color_presets[idx].g2;
    s_led_color.color2_b = s_color_presets[idx].b2;
    s_led_color.color_cycle = s_color_presets[idx].cycle;
    s_led_color.color_style = s_color_presets[idx].style;
    s_led_color.highlight_mode = s_color_presets[idx].highlight;
}

static void led_color_apply(void)
{
    led_beat_color_set(s_led_color.color_r, s_led_color.color_g, s_led_color.color_b);
    led_beat_secondary_color_set(s_led_color.color2_r, s_led_color.color2_g, s_led_color.color2_b);
    led_beat_color_cycle_set(s_led_color.color_cycle);
    led_beat_color_style_set(s_led_color.color_style);
    led_beat_highlight_mode_set(s_led_color.highlight_mode);
    led_beat_brightness_set(s_led_color.brightness);
    led_audio_brightness_enable(s_led_color.audio_brightness_enabled);
    led_audio_energy_range_set(s_led_color.audio_energy_range);

    led_modes_set_primary_color(s_led_color.color_r, s_led_color.color_g, s_led_color.color_b);
    led_modes_set_secondary_color(s_led_color.color2_r, s_led_color.color2_g, s_led_color.color2_b);
    led_modes_color_cycle_set(s_led_color.color_cycle);
    led_modes_color_style_set(s_led_color.color_style);
    led_modes_highlight_mode_set(s_led_color.highlight_mode);
    led_modes_set_brightness(s_led_color.brightness);

    led_ui_fft_request(LED_UI_FFT_REQ_COLOR_APP, s_led_color.audio_brightness_enabled);

    s_led_color.preset_idx = find_preset_match(s_led_color.color_cycle,
                                               s_led_color.color_style,
                                               s_led_color.highlight_mode,
                                               s_led_color.color_r,
                                               s_led_color.color_g,
                                               s_led_color.color_b,
                                               s_led_color.color2_r,
                                               s_led_color.color2_g,
                                               s_led_color.color2_b);
}

static void led_color_load(void)
{
    led_beat_color_get(&s_led_color.color_r, &s_led_color.color_g, &s_led_color.color_b);
    led_beat_secondary_color_get(&s_led_color.color2_r, &s_led_color.color2_g, &s_led_color.color2_b);
    s_led_color.color_cycle = led_beat_color_cycle_get();
    s_led_color.color_style = led_beat_color_style_get();
    s_led_color.highlight_mode = led_beat_highlight_mode_get();
    s_led_color.audio_brightness_enabled = led_audio_brightness_enabled();
    s_led_color.audio_energy_range = led_audio_energy_range_get();
    s_led_color.brightness = led_beat_brightness_get();
    if (s_led_color.brightness < 8) s_led_color.brightness = 8;
    s_led_color.preset_idx = find_preset_match(s_led_color.color_cycle,
                                               s_led_color.color_style,
                                               s_led_color.highlight_mode,
                                               s_led_color.color_r,
                                               s_led_color.color_g,
                                               s_led_color.color_b,
                                               s_led_color.color2_r,
                                               s_led_color.color2_g,
                                               s_led_color.color2_b);
}

static void refresh_labels(void)
{
    const char *preset_name = (s_led_color.preset_idx >= 0 && s_led_color.preset_idx < kColorPresetCount)
        ? s_color_presets[s_led_color.preset_idx].name
        : "manual";
    snprintf(s_preset_label, sizeof(s_preset_label), "preset:%s", preset_name);
    snprintf(s_style_label, sizeof(s_style_label), "style:%s", color_style_name(s_led_color.color_style));
    snprintf(s_hi_label, sizeof(s_hi_label), "hi:%s", highlight_mode_name(s_led_color.highlight_mode));
    snprintf(s_r_label, sizeof(s_r_label), "R:%u", (unsigned)s_led_color.color_r);
    snprintf(s_g_label, sizeof(s_g_label), "G:%u", (unsigned)s_led_color.color_g);
    snprintf(s_b_label, sizeof(s_b_label), "B:%u", (unsigned)s_led_color.color_b);
    snprintf(s_r2_label, sizeof(s_r2_label), "R2:%u", (unsigned)s_led_color.color2_r);
    snprintf(s_g2_label, sizeof(s_g2_label), "G2:%u", (unsigned)s_led_color.color2_g);
    snprintf(s_b2_label, sizeof(s_b2_label), "B2:%u", (unsigned)s_led_color.color2_b);
    if (s_led_color.audio_brightness_enabled) {
        snprintf(s_audio_label, sizeof(s_audio_label), "audio:%s",
                 audio_energy_range_name(s_led_color.audio_energy_range));
    } else {
        snprintf(s_audio_label, sizeof(s_audio_label), "audio:off");
    }
    snprintf(s_bright_label, sizeof(s_bright_label), "bright:%u", (unsigned)s_led_color.brightness);
}

static void append_entry(size_t *n, int cfg_idx, const char *label)
{
    if (!n || !label || *n >= LED_COLOR_ENTRY_COUNT) return;
    s_entries[*n] = (shell_menu_entry_t){ .id = label, .label = label };
    s_meta[*n] = (led_color_meta_t){ .kind = LED_COLOR_ENTRY_CFG, .value = cfg_idx };
    (*n)++;
}

static size_t build_entries(void)
{
    refresh_labels();
    size_t n = 0;
    s_entries[n] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    s_meta[n] = (led_color_meta_t){ .kind = LED_COLOR_ENTRY_BACK, .value = 0 };
    n++;
    append_entry(&n, LED_COLOR_CFG_PRESET, s_preset_label);
    append_entry(&n, LED_COLOR_CFG_STYLE, s_style_label);
    append_entry(&n, LED_COLOR_CFG_HIGHLIGHT, s_hi_label);
    append_entry(&n, LED_COLOR_CFG_R, s_r_label);
    append_entry(&n, LED_COLOR_CFG_G, s_g_label);
    append_entry(&n, LED_COLOR_CFG_B, s_b_label);
    append_entry(&n, LED_COLOR_CFG_R2, s_r2_label);
    append_entry(&n, LED_COLOR_CFG_G2, s_g2_label);
    append_entry(&n, LED_COLOR_CFG_B2, s_b2_label);
    append_entry(&n, LED_COLOR_CFG_AUDIO_INPUT, s_audio_label);
    append_entry(&n, LED_COLOR_CFG_BRIGHTNESS, s_bright_label);
    return n;
}

static void led_color_cycle_cfg(int cfg_idx, bool inc)
{
    const int bright_step = 16;

    switch (cfg_idx) {
        case LED_COLOR_CFG_PRESET: {
            int idx = s_led_color.preset_idx;
            if (idx < 0) idx = inc ? 0 : (kColorPresetCount - 1);
            else idx = (idx + (inc ? 1 : -1) + kColorPresetCount) % kColorPresetCount;
            apply_preset_idx(idx);
            break;
        }
        case LED_COLOR_CFG_STYLE:
            s_led_color.color_style = (led_color_style_t)((((int)s_led_color.color_style) +
                                        (inc ? 1 : -1) + 3) % 3);
            if (s_led_color.color_style != LED_COLOR_STYLE_PALETTE &&
                s_led_color.color_cycle != LED_COLOR_CYCLE_STATIC) {
                s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            break;
        case LED_COLOR_CFG_HIGHLIGHT:
            s_led_color.highlight_mode = (s_led_color.highlight_mode == LED_HIGHLIGHT_OFF)
                ? LED_HIGHLIGHT_PEAKS
                : LED_HIGHLIGHT_OFF;
            break;
        case LED_COLOR_CFG_R:
            if (s_led_color.color_style == LED_COLOR_STYLE_PALETTE) {
                s_led_color.color_style = LED_COLOR_STYLE_MONO;
            }
            s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            s_led_color.color_r = step_color_u8(s_led_color.color_r, inc);
            break;
        case LED_COLOR_CFG_G:
            if (s_led_color.color_style == LED_COLOR_STYLE_PALETTE) {
                s_led_color.color_style = LED_COLOR_STYLE_MONO;
            }
            s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            s_led_color.color_g = step_color_u8(s_led_color.color_g, inc);
            break;
        case LED_COLOR_CFG_B:
            if (s_led_color.color_style == LED_COLOR_STYLE_PALETTE) {
                s_led_color.color_style = LED_COLOR_STYLE_MONO;
            }
            s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            s_led_color.color_b = step_color_u8(s_led_color.color_b, inc);
            break;
        case LED_COLOR_CFG_R2:
            if (s_led_color.color_style == LED_COLOR_STYLE_PALETTE) {
                s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            if (s_led_color.color_style == LED_COLOR_STYLE_MONO) {
                s_led_color.color_style = LED_COLOR_STYLE_DUO;
            }
            s_led_color.color2_r = step_color_u8(s_led_color.color2_r, inc);
            break;
        case LED_COLOR_CFG_G2:
            if (s_led_color.color_style == LED_COLOR_STYLE_PALETTE) {
                s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            if (s_led_color.color_style == LED_COLOR_STYLE_MONO) {
                s_led_color.color_style = LED_COLOR_STYLE_DUO;
            }
            s_led_color.color2_g = step_color_u8(s_led_color.color2_g, inc);
            break;
        case LED_COLOR_CFG_B2:
            if (s_led_color.color_style == LED_COLOR_STYLE_PALETTE) {
                s_led_color.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            if (s_led_color.color_style == LED_COLOR_STYLE_MONO) {
                s_led_color.color_style = LED_COLOR_STYLE_DUO;
            }
            s_led_color.color2_b = step_color_u8(s_led_color.color2_b, inc);
            break;
        case LED_COLOR_CFG_AUDIO_INPUT: {
            int mode = audio_input_mode_get();
            mode = (mode + (inc ? 1 : -1) + 4) % 4;
            audio_input_mode_set(mode);
            break;
        }
        case LED_COLOR_CFG_BRIGHTNESS:
            s_led_color.brightness = step_u8_clamp(s_led_color.brightness,
                                                   inc ? bright_step : -bright_step,
                                                   8, 255);
            break;
        default:
            return;
    }

    led_color_apply();
}

void led_color_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    led_color_load();
    led_ui_fft_request(LED_UI_FFT_REQ_COLOR_APP, s_led_color.audio_brightness_enabled);
    s_led_color.selected = clamp_sel(s_led_color.selected, build_entries());
    shell_ui_menu_reset(s_led_color.selected);
}

void led_color_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    led_ui_fft_request(LED_UI_FFT_REQ_COLOR_APP, false);
}

void led_color_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    shell_ui_menu_tick(dt_sec, s_led_color.selected);
}

void led_color_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;

    size_t count = build_entries();
    s_led_color.selected = clamp_sel(s_led_color.selected, count);

    if (ev->button == INPUT_BTN_A && s_led_color.selected > 0) {
        s_led_color.selected--;
        return;
    }
    if (ev->button == INPUT_BTN_B && (s_led_color.selected + 1) < count) {
        s_led_color.selected++;
        return;
    }
    if (ev->button != INPUT_BTN_C && ev->button != INPUT_BTN_D) {
        return;
    }

    led_color_meta_t selected = s_meta[s_led_color.selected];
    if (selected.kind == LED_COLOR_ENTRY_BACK) {
        if (ctx->request_switch) {
            ctx->request_switch("menu", ctx->request_user_data);
        }
        return;
    }

    led_color_cycle_cfg(selected.value, ev->button == INPUT_BTN_D);
}

void led_color_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    size_t count = build_entries();
    s_led_color.selected = clamp_sel(s_led_color.selected, count);

    shell_menu_view_t view = {
        .entries = s_entries,
        .count = count,
        .selected = s_led_color.selected,
        .title = "LED / COLOR",
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}
