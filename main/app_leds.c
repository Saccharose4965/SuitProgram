#include "app_shell.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "fft.h"
#include "led.h"
#include "led_modes.h"
#include "shell_audio.h"

const shell_legend_t LEDS_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_CUSTOM2 },
};

enum {
    LEDS_PAGE_AUDIO = 0,
    LEDS_PAGE_CUSTOM = 1,
};

typedef enum {
    LEDS_MODE_SOURCE_AUDIO = 0,
    LEDS_MODE_SOURCE_CUSTOM = 1,
} leds_mode_source_t;

enum {
    LEDS_CFG_PRESET = 0,
    LEDS_CFG_STYLE,
    LEDS_CFG_HIGHLIGHT,
    LEDS_CFG_R,
    LEDS_CFG_G,
    LEDS_CFG_B,
    LEDS_CFG_R2,
    LEDS_CFG_G2,
    LEDS_CFG_B2,
    LEDS_CFG_SPEED,
    LEDS_CFG_AUDIO_INPUT,
    LEDS_CFG_BRIGHTNESS,
    LEDS_CFG_COUNT,
};

typedef enum {
    LEDS_ENTRY_KIND_BACK = 0,
    LEDS_ENTRY_KIND_ANIM,
    LEDS_ENTRY_KIND_CFG,
} leds_entry_kind_t;

typedef struct {
    leds_entry_kind_t kind;
    int value;
} leds_menu_meta_t;

typedef struct {
    const char *label;
    const char *variant_labels[3];
    const char *audio_names[3];
    const char *custom_names[3];
    uint8_t variant_count;
} leds_animation_choice_t;

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
} leds_color_preset_t;

static const leds_color_preset_t s_color_presets[] = {
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

// Both LED pages expose the same labels. Each label maps to the nearest
// engine-native effect on the audio-reactive and continuous renderers.
static const leds_animation_choice_t s_animation_choices[] = {
    { "off",          { NULL,      NULL,      NULL      }, { NULL,          NULL,          NULL      }, { "off",     NULL,      NULL      }, 1 },
    { "fill",         { NULL,      NULL,      NULL      }, { "flash",       NULL,          NULL      }, { "fill",    NULL,      NULL      }, 1 },
    { "breathe",      { NULL,      NULL,      NULL      }, { "pulse",       NULL,          NULL      }, { "breathe", NULL,      NULL      }, 1 },
    { "scanner",      { NULL,      NULL,      NULL      }, { "comet",       NULL,          NULL      }, { "scanner", NULL,      NULL      }, 1 },
    { "energy",       { NULL,      NULL,      NULL      }, { "audio_energy",NULL,          NULL      }, { "energy",  NULL,      NULL      }, 1 },
    { "blink",        { NULL,      NULL,      NULL      }, { "flash",       NULL,          NULL      }, { "blink",   NULL,      NULL      }, 1 },
    { "chase",        { NULL,      NULL,      NULL      }, { "comet",       NULL,          NULL      }, { "chase",   NULL,      NULL      }, 1 },
    { "twinkle",      { NULL,      NULL,      NULL      }, { "spark",       NULL,          NULL      }, { "twinkle", NULL,      NULL      }, 1 },
    { "glitch",       { NULL,      NULL,      NULL      }, { "shock",       NULL,          NULL      }, { "glitch",  NULL,      NULL      }, 1 },
    { "plane",        { "sweep",   "pair",    "fan"     }, { "plane_sweep", "plane_pair",  "plane_fan" },
                                                                 { "plane",   "mirror",      "prism"   }, 3 },
    { "ring",         { "pulse",   "train",   NULL      }, { "ring_pulse",  "ring_train",  NULL      },
                                                                 { "ring",    "contour",     NULL      }, 2 },
    { "orbit",        { NULL,      NULL,      NULL      }, { "ring_train",  NULL,          NULL      }, { "orbit",   NULL,      NULL      }, 1 },
    { "aurora",       { NULL,      NULL,      NULL      }, { "plane_fan",   NULL,          NULL      }, { "aurora",  NULL,      NULL      }, 1 },
    { "vortex",       { NULL,      NULL,      NULL      }, { "crossfire",   NULL,          NULL      }, { "vortex",  NULL,      NULL      }, 1 },
    { "helix",        { NULL,      NULL,      NULL      }, { "crossfire",   NULL,          NULL      }, { "helix",   NULL,      NULL      }, 1 },
    { "random",       { NULL,      NULL,      NULL      }, { "random",      NULL,          NULL      }, { "random",  NULL,      NULL      }, 1 },
    { "flash",        { NULL,      NULL,      NULL      }, { "flash",       NULL,          NULL      }, { "blink",   NULL,      NULL      }, 1 },
    { "pulse",        { NULL,      NULL,      NULL      }, { "pulse",       NULL,          NULL      }, { "breathe", NULL,      NULL      }, 1 },
    { "spark",        { NULL,      NULL,      NULL      }, { "spark",       NULL,          NULL      }, { "twinkle", NULL,      NULL      }, 1 },
    { "comet",        { NULL,      NULL,      NULL      }, { "comet",       NULL,          NULL      }, { "chase",   NULL,      NULL      }, 1 },
    { "shock",        { NULL,      NULL,      NULL      }, { "shock",       NULL,          NULL      }, { "glitch",  NULL,      NULL      }, 1 },
    { "crossfire",    { NULL,      NULL,      NULL      }, { "crossfire",   NULL,          NULL      }, { "vortex",  NULL,      NULL      }, 1 },
    { "audio_energy", { NULL,      NULL,      NULL      }, { "audio_energy",NULL,          NULL      }, { "energy",  NULL,      NULL      }, 1 },
};
static const int kAnimationChoiceCount =
    (int)(sizeof(s_animation_choices) / sizeof(s_animation_choices[0]));

static const char *TAG = "app_leds";

#define LEDS_MAX_PAGE_ENTRIES 48
#define LEDS_ENTRY_LABEL_LEN 24
static shell_menu_entry_t s_audio_entries[LEDS_MAX_PAGE_ENTRIES];
static shell_menu_entry_t s_custom_entries[LEDS_MAX_PAGE_ENTRIES];
static leds_menu_meta_t s_audio_meta[LEDS_MAX_PAGE_ENTRIES];
static leds_menu_meta_t s_custom_meta[LEDS_MAX_PAGE_ENTRIES];
static char s_audio_entry_labels[LEDS_MAX_PAGE_ENTRIES][LEDS_ENTRY_LABEL_LEN];
static char s_custom_entry_labels[LEDS_MAX_PAGE_ENTRIES][LEDS_ENTRY_LABEL_LEN];

static char s_preset_label[24];
static char s_style_label[16];
static char s_hi_label[14];
static char s_r_label[12];
static char s_g_label[12];
static char s_b_label[12];
static char s_r2_label[12];
static char s_g2_label[12];
static char s_b2_label[12];
static char s_speed_label[14];
static char s_audio_label[18];
static char s_bright_label[14];

typedef struct {
    int page;
    size_t audio_sel;  // includes "Back" at index 0
    size_t custom_sel; // includes "Back" at index 0
    int audio_choice;
    int custom_choice;
    int preset_idx; // -1 for manual
    uint8_t color_r;
    uint8_t color_g;
    uint8_t color_b;
    uint8_t color2_r;
    uint8_t color2_g;
    uint8_t color2_b;
    led_color_cycle_t color_cycle;
    led_color_style_t color_style;
    led_highlight_mode_t highlight_mode;
    uint8_t custom_speed_percent;
    bool audio_brightness_enabled;
    led_audio_energy_range_t audio_energy_range;
    uint8_t brightness;
} leds_state_t;
static leds_state_t s_leds = {0};
static bool s_leds_fft_owned = false;
static uint8_t s_audio_choice_variants[LEDS_MAX_PAGE_ENTRIES];
static uint8_t s_custom_choice_variants[LEDS_MAX_PAGE_ENTRIES];

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

static int mode_count_for_source(leds_mode_source_t source)
{
    return (source == LEDS_MODE_SOURCE_AUDIO) ? led_beat_anim_count() : led_modes_count();
}

static const char *mode_name_for_source(leds_mode_source_t source, int idx)
{
    return (source == LEDS_MODE_SOURCE_AUDIO) ? led_beat_anim_name(idx) : led_modes_name(idx);
}

static uint8_t *variant_state_for_page(int page)
{
    return (page == LEDS_PAGE_AUDIO) ? s_audio_choice_variants : s_custom_choice_variants;
}

static int animation_variant_count(int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    int count = (int)s_animation_choices[choice_idx].variant_count;
    return (count > 0) ? count : 1;
}

static int animation_variant_index_for_page(int page, int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    uint8_t *variants = variant_state_for_page(page);
    int count = animation_variant_count(choice_idx);
    int idx = variants[choice_idx];
    if (idx >= count) idx = 0;
    return idx;
}

static const char *animation_choice_variant_label(int choice_idx, int variant_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    variant_idx = clamp_int(variant_idx, 0, animation_variant_count(choice_idx) - 1);
    return s_animation_choices[choice_idx].variant_labels[variant_idx];
}

static const char *animation_choice_mode_name(leds_mode_source_t source,
                                              int choice_idx,
                                              int variant_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    variant_idx = clamp_int(variant_idx, 0, animation_variant_count(choice_idx) - 1);
    return (source == LEDS_MODE_SOURCE_AUDIO)
        ? s_animation_choices[choice_idx].audio_names[variant_idx]
        : s_animation_choices[choice_idx].custom_names[variant_idx];
}

static int mode_index_for_source_name(leds_mode_source_t source, const char *name)
{
    if (!name) return -1;
    int count = mode_count_for_source(source);
    for (int i = 0; i < count; ++i) {
        const char *candidate = mode_name_for_source(source, i);
        if (candidate && strcmp(candidate, name) == 0) {
            return i;
        }
    }
    return -1;
}

static int animation_choice_mode_index(leds_mode_source_t source,
                                       int choice_idx,
                                       int variant_idx)
{
    return mode_index_for_source_name(source, animation_choice_mode_name(source, choice_idx, variant_idx));
}

static bool animation_choice_find_for_source_mode(leds_mode_source_t source,
                                                  const char *mode_name,
                                                  int *choice_idx,
                                                  int *variant_idx)
{
    if (!mode_name) return false;

    for (int i = 0; i < kAnimationChoiceCount; ++i) {
        int count = animation_variant_count(i);
        for (int v = 0; v < count; ++v) {
            const char *mapped = animation_choice_mode_name(source, i, v);
            if (mapped && strcmp(mapped, mode_name) == 0 &&
                strcmp(s_animation_choices[i].label, mode_name) == 0) {
                if (choice_idx) *choice_idx = i;
                if (variant_idx) *variant_idx = v;
                return true;
            }
        }
    }

    for (int i = 0; i < kAnimationChoiceCount; ++i) {
        int count = animation_variant_count(i);
        for (int v = 0; v < count; ++v) {
            const char *mapped = animation_choice_mode_name(source, i, v);
            if (mapped && strcmp(mapped, mode_name) == 0) {
                if (choice_idx) *choice_idx = i;
                if (variant_idx) *variant_idx = v;
                return true;
            }
        }
    }

    return false;
}

static void animation_choice_format_label(int page, int choice_idx,
                                          char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;

    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    int variant_idx = animation_variant_index_for_page(page, choice_idx);
    const char *variant = animation_choice_variant_label(choice_idx, variant_idx);
    if (animation_variant_count(choice_idx) > 1 && variant && variant[0]) {
        snprintf(buf, buf_len, "%s:%s", s_animation_choices[choice_idx].label, variant);
    } else {
        snprintf(buf, buf_len, "%s", s_animation_choices[choice_idx].label);
    }
}

static size_t animation_selection_index(int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    return (size_t)choice_idx + 1u;
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

static const char *audio_brightness_name(bool enabled)
{
    return enabled ? "on" : "off";
}

static const char *audio_energy_range_name(led_audio_energy_range_t range)
{
    switch (range) {
        case LED_AUDIO_ENERGY_RANGE_MID: return "127-255";
        case LED_AUDIO_ENERGY_RANGE_HIGH: return "191-255";
        case LED_AUDIO_ENERGY_RANGE_FULL:
        default: return "0-255";
    }
}

static int audio_input_mode_get(void)
{
    if (!s_leds.audio_brightness_enabled) {
        return 0;
    }

    switch (s_leds.audio_energy_range) {
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
    s_leds.audio_brightness_enabled = (mode != 0);
    switch (mode) {
        case 2:
            s_leds.audio_energy_range = LED_AUDIO_ENERGY_RANGE_MID;
            break;
        case 3:
            s_leds.audio_energy_range = LED_AUDIO_ENERGY_RANGE_HIGH;
            break;
        case 1:
        case 0:
        default:
            s_leds.audio_energy_range = LED_AUDIO_ENERGY_RANGE_FULL;
            break;
    }
}

static bool leds_fft_needed(void)
{
    return ((s_leds.page == LEDS_PAGE_AUDIO) &&
            (animation_choice_mode_index(LEDS_MODE_SOURCE_AUDIO,
                                         s_leds.audio_choice,
                                         animation_variant_index_for_page(LEDS_PAGE_AUDIO, s_leds.audio_choice)) >= 0)) ||
           s_leds.audio_brightness_enabled;
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
    s_leds.preset_idx = idx;
    s_leds.color_r = s_color_presets[idx].r;
    s_leds.color_g = s_color_presets[idx].g;
    s_leds.color_b = s_color_presets[idx].b;
    s_leds.color2_r = s_color_presets[idx].r2;
    s_leds.color2_g = s_color_presets[idx].g2;
    s_leds.color2_b = s_color_presets[idx].b2;
    s_leds.color_cycle = s_color_presets[idx].cycle;
    s_leds.color_style = s_color_presets[idx].style;
    s_leds.highlight_mode = s_color_presets[idx].highlight;
}

static void leds_apply_color(void)
{
    led_beat_color_set(s_leds.color_r, s_leds.color_g, s_leds.color_b);
    led_beat_secondary_color_set(s_leds.color2_r, s_leds.color2_g, s_leds.color2_b);
    led_beat_color_cycle_set(s_leds.color_cycle);
    led_beat_color_style_set(s_leds.color_style);
    led_beat_highlight_mode_set(s_leds.highlight_mode);
    led_beat_brightness_set(s_leds.brightness);
    led_audio_brightness_enable(s_leds.audio_brightness_enabled);
    led_audio_energy_range_set(s_leds.audio_energy_range);
    led_modes_set_primary_color(s_leds.color_r, s_leds.color_g, s_leds.color_b);
    led_modes_set_secondary_color(s_leds.color2_r, s_leds.color2_g, s_leds.color2_b);
    led_modes_color_cycle_set(s_leds.color_cycle);
    led_modes_color_style_set(s_leds.color_style);
    led_modes_highlight_mode_set(s_leds.highlight_mode);
    led_modes_set_speed_percent(s_leds.custom_speed_percent);
    led_modes_set_brightness(s_leds.brightness);
    s_leds.preset_idx = find_preset_match(s_leds.color_cycle,
                                          s_leds.color_style,
                                          s_leds.highlight_mode,
                                          s_leds.color_r,
                                          s_leds.color_g,
                                          s_leds.color_b,
                                          s_leds.color2_r,
                                          s_leds.color2_g,
                                          s_leds.color2_b);
}

static void leds_load_color_state(void)
{
    led_beat_color_get(&s_leds.color_r, &s_leds.color_g, &s_leds.color_b);
    led_beat_secondary_color_get(&s_leds.color2_r, &s_leds.color2_g, &s_leds.color2_b);
    s_leds.color_cycle = led_beat_color_cycle_get();
    s_leds.color_style = led_beat_color_style_get();
    s_leds.highlight_mode = led_beat_highlight_mode_get();
    s_leds.preset_idx = find_preset_match(s_leds.color_cycle,
                                          s_leds.color_style,
                                          s_leds.highlight_mode,
                                          s_leds.color_r,
                                          s_leds.color_g,
                                          s_leds.color_b,
                                          s_leds.color2_r,
                                          s_leds.color2_g,
                                          s_leds.color2_b);
    s_leds.custom_speed_percent = led_modes_get_speed_percent();
    s_leds.audio_brightness_enabled = led_audio_brightness_enabled();
    s_leds.audio_energy_range = led_audio_energy_range_get();
    s_leds.brightness = led_beat_brightness_get();
    if (s_leds.brightness < 8) s_leds.brightness = 8;
}

static void refresh_color_labels(void)
{
    const char *preset_name = (s_leds.preset_idx >= 0 && s_leds.preset_idx < kColorPresetCount)
        ? s_color_presets[s_leds.preset_idx].name
        : "manual";
    snprintf(s_preset_label, sizeof(s_preset_label), "preset:%s", preset_name);
    snprintf(s_style_label, sizeof(s_style_label), "style:%s", color_style_name(s_leds.color_style));
    snprintf(s_hi_label, sizeof(s_hi_label), "hi:%s", highlight_mode_name(s_leds.highlight_mode));
    snprintf(s_r_label, sizeof(s_r_label), "R:%u", (unsigned)s_leds.color_r);
    snprintf(s_g_label, sizeof(s_g_label), "G:%u", (unsigned)s_leds.color_g);
    snprintf(s_b_label, sizeof(s_b_label), "B:%u", (unsigned)s_leds.color_b);
    snprintf(s_r2_label, sizeof(s_r2_label), "R2:%u", (unsigned)s_leds.color2_r);
    snprintf(s_g2_label, sizeof(s_g2_label), "G2:%u", (unsigned)s_leds.color2_g);
    snprintf(s_b2_label, sizeof(s_b2_label), "B2:%u", (unsigned)s_leds.color2_b);
    snprintf(s_speed_label, sizeof(s_speed_label), "speed:%u%%", (unsigned)s_leds.custom_speed_percent);
    if (s_leds.audio_brightness_enabled) {
        snprintf(s_audio_label, sizeof(s_audio_label), "audio:%s",
                 audio_energy_range_name(s_leds.audio_energy_range));
    } else {
        snprintf(s_audio_label, sizeof(s_audio_label), "audio:%s",
                 audio_brightness_name(false));
    }
    snprintf(s_bright_label, sizeof(s_bright_label), "bright:%u", (unsigned)s_leds.brightness);
}

static void append_cfg_entry(shell_menu_entry_t *entries,
                             leds_menu_meta_t *meta,
                             size_t *n,
                             size_t cap,
                             int cfg_idx,
                             const char *label)
{
    if (!entries || !meta || !n || !label || *n >= cap) return;
    entries[*n] = (shell_menu_entry_t){ .id = label, .label = label };
    meta[*n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG, .value = cfg_idx };
    (*n)++;
}

static void append_color_entries(shell_menu_entry_t *entries,
                                 leds_menu_meta_t *meta,
                                 size_t *n,
                                 size_t cap)
{
    if (!entries || !meta || !n) return;
    refresh_color_labels();
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_PRESET, s_preset_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_STYLE, s_style_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_HIGHLIGHT, s_hi_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_R, s_r_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_G, s_g_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_B, s_b_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_R2, s_r2_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_G2, s_g2_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_B2, s_b2_label);
}

static void append_runtime_entries(shell_menu_entry_t *entries,
                                   leds_menu_meta_t *meta,
                                   size_t *n,
                                   size_t cap,
                                   int page)
{
    if (!entries || !meta || !n) return;
    refresh_color_labels();
    if (page == LEDS_PAGE_CUSTOM) {
        append_cfg_entry(entries, meta, n, cap, LEDS_CFG_SPEED, s_speed_label);
    }
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_AUDIO_INPUT, s_audio_label);
    append_cfg_entry(entries, meta, n, cap, LEDS_CFG_BRIGHTNESS, s_bright_label);
}

static void append_animation_entries(shell_menu_entry_t *entries,
                                     leds_menu_meta_t *meta,
                                     char labels[][LEDS_ENTRY_LABEL_LEN],
                                     size_t *n,
                                     size_t cap,
                                     int page)
{
    if (!entries || !meta || !labels || !n) return;

    for (int i = 0; i < kAnimationChoiceCount && *n < cap; ++i) {
        animation_choice_format_label(page, i, labels[*n], LEDS_ENTRY_LABEL_LEN);
        entries[*n] = (shell_menu_entry_t){ .id = labels[*n], .label = labels[*n] };
        meta[*n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_ANIM, .value = i };
        (*n)++;
    }
}

static size_t build_entries_for_page(int page,
                                     shell_menu_entry_t *entries,
                                     leds_menu_meta_t *meta,
                                     char labels[][LEDS_ENTRY_LABEL_LEN])
{
    if (!entries || !meta || !labels) return 0;

    size_t n = 0;
    const size_t cap = LEDS_MAX_PAGE_ENTRIES;
    entries[n] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_BACK, .value = 0 };
    n++;

    append_animation_entries(entries, meta, labels, &n, cap, page);
    append_color_entries(entries, meta, &n, cap);
    append_runtime_entries(entries, meta, &n, cap, page);
    return n;
}

static size_t build_audio_entries(void)
{
    return build_entries_for_page(LEDS_PAGE_AUDIO, s_audio_entries, s_audio_meta, s_audio_entry_labels);
}

static size_t build_custom_entries(void)
{
    return build_entries_for_page(LEDS_PAGE_CUSTOM, s_custom_entries, s_custom_meta, s_custom_entry_labels);
}

static void leds_audio_fft_ensure_started(void)
{
    if (!shell_audio_init_if_needed()) {
        ESP_LOGW(TAG, "audio init unavailable; beat modes will not react");
        return;
    }

    bool was_running = fft_visualizer_running();
    fft_set_display_enabled(false);
    esp_err_t err = fft_visualizer_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "fft_visualizer_start failed: %s", esp_err_to_name(err));
        return;
    }
    if (!was_running) {
        s_leds_fft_owned = true;
    }
}

static void leds_audio_fft_maybe_stop(void)
{
    if (fft_visualizer_running()) {
        fft_visualizer_stop();
    }
    s_leds_fft_owned = false;
}

static void leds_sync_fft_state(void)
{
    if (leds_fft_needed()) {
        leds_audio_fft_ensure_started();
    } else {
        leds_audio_fft_maybe_stop();
    }
}

static void leds_apply_audio(void)
{
    char label[LEDS_ENTRY_LABEL_LEN];
    led_modes_enable(false);
    s_leds.audio_choice = clamp_int(s_leds.audio_choice, 0, kAnimationChoiceCount - 1);
    int variant_idx = animation_variant_index_for_page(LEDS_PAGE_AUDIO, s_leds.audio_choice);
    animation_choice_format_label(LEDS_PAGE_AUDIO, s_leds.audio_choice, label, sizeof(label));
    int mode_idx = animation_choice_mode_index(LEDS_MODE_SOURCE_AUDIO, s_leds.audio_choice, variant_idx);
    if (mode_idx < 0) {
        led_beat_enable(false);
        leds_sync_fft_state();
        system_state_set_led_mode(-1, label);
        return;
    }

    led_beat_anim_set((led_beat_anim_t)mode_idx);
    led_beat_enable(true);
    leds_sync_fft_state();
    system_state_set_led_mode(100 + mode_idx, label);
}

static void leds_apply_custom(void)
{
    char label[LEDS_ENTRY_LABEL_LEN];
    led_beat_enable(false);
    s_leds.custom_choice = clamp_int(s_leds.custom_choice, 0, kAnimationChoiceCount - 1);
    int variant_idx = animation_variant_index_for_page(LEDS_PAGE_CUSTOM, s_leds.custom_choice);
    animation_choice_format_label(LEDS_PAGE_CUSTOM, s_leds.custom_choice, label, sizeof(label));
    int mode_idx = animation_choice_mode_index(LEDS_MODE_SOURCE_CUSTOM, s_leds.custom_choice, variant_idx);
    if (mode_idx < 0) {
        led_modes_enable(false);
        leds_sync_fft_state();
        system_state_set_led_mode(-1, label);
        return;
    }

    led_modes_set(mode_idx);
    led_modes_enable(true);
    leds_sync_fft_state();
    system_state_set_led_mode(mode_idx, label);
}

static void leds_select_animation_for_page(int page, int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    uint8_t *variants = variant_state_for_page(page);
    int *current_choice = (page == LEDS_PAGE_AUDIO) ? &s_leds.audio_choice : &s_leds.custom_choice;

    if (*current_choice == choice_idx) {
        int count = animation_variant_count(choice_idx);
        if (count > 1) {
            variants[choice_idx] = (uint8_t)((variants[choice_idx] + 1u) % (uint8_t)count);
        }
    } else {
        *current_choice = choice_idx;
    }

    if (page == LEDS_PAGE_AUDIO) {
        leds_apply_audio();
    } else {
        leds_apply_custom();
    }
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
    const int bright_step = 16;
    const int speed_step = 10;
    switch (cfg_idx) {
        case LEDS_CFG_PRESET: {
            int idx = s_leds.preset_idx;
            if (idx < 0) idx = inc ? 0 : (kColorPresetCount - 1);
            else idx = (idx + (inc ? 1 : -1) + kColorPresetCount) % kColorPresetCount;
            apply_preset_idx(idx);
            break;
        }
        case LEDS_CFG_STYLE:
            s_leds.color_style = (led_color_style_t)((((int)s_leds.color_style) +
                                  (inc ? 1 : -1) + 3) % 3);
            if (s_leds.color_style != LED_COLOR_STYLE_PALETTE &&
                s_leds.color_cycle != LED_COLOR_CYCLE_STATIC) {
                s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            break;
        case LEDS_CFG_HIGHLIGHT:
            s_leds.highlight_mode = (s_leds.highlight_mode == LED_HIGHLIGHT_OFF)
                ? LED_HIGHLIGHT_PEAKS
                : LED_HIGHLIGHT_OFF;
            break;
        case LEDS_CFG_R:
            if (s_leds.color_style == LED_COLOR_STYLE_PALETTE) {
                s_leds.color_style = LED_COLOR_STYLE_MONO;
            }
            s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            s_leds.color_r = step_color_u8(s_leds.color_r, inc);
            break;
        case LEDS_CFG_G:
            if (s_leds.color_style == LED_COLOR_STYLE_PALETTE) {
                s_leds.color_style = LED_COLOR_STYLE_MONO;
            }
            s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            s_leds.color_g = step_color_u8(s_leds.color_g, inc);
            break;
        case LEDS_CFG_B:
            if (s_leds.color_style == LED_COLOR_STYLE_PALETTE) {
                s_leds.color_style = LED_COLOR_STYLE_MONO;
            }
            s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            s_leds.color_b = step_color_u8(s_leds.color_b, inc);
            break;
        case LEDS_CFG_R2:
            if (s_leds.color_style == LED_COLOR_STYLE_PALETTE) {
                s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            if (s_leds.color_style == LED_COLOR_STYLE_MONO) {
                s_leds.color_style = LED_COLOR_STYLE_DUO;
            }
            s_leds.color2_r = step_color_u8(s_leds.color2_r, inc);
            break;
        case LEDS_CFG_G2:
            if (s_leds.color_style == LED_COLOR_STYLE_PALETTE) {
                s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            if (s_leds.color_style == LED_COLOR_STYLE_MONO) {
                s_leds.color_style = LED_COLOR_STYLE_DUO;
            }
            s_leds.color2_g = step_color_u8(s_leds.color2_g, inc);
            break;
        case LEDS_CFG_B2:
            if (s_leds.color_style == LED_COLOR_STYLE_PALETTE) {
                s_leds.color_cycle = LED_COLOR_CYCLE_STATIC;
            }
            if (s_leds.color_style == LED_COLOR_STYLE_MONO) {
                s_leds.color_style = LED_COLOR_STYLE_DUO;
            }
            s_leds.color2_b = step_color_u8(s_leds.color2_b, inc);
            break;
        case LEDS_CFG_SPEED:
            s_leds.custom_speed_percent = step_u8_clamp(s_leds.custom_speed_percent,
                                                        inc ? speed_step : -speed_step,
                                                        10, 250);
            break;
        case LEDS_CFG_AUDIO_INPUT: {
            int mode = audio_input_mode_get();
            mode = (mode + (inc ? 1 : -1) + 4) % 4;
            audio_input_mode_set(mode);
            break;
        }
        case LEDS_CFG_BRIGHTNESS:
            s_leds.brightness = step_u8_clamp(s_leds.brightness,
                                              inc ? bright_step : -bright_step,
                                              8, 255);
            break;
        default:
            return;
    }

    leds_apply_color();
    leds_sync_fft_state();
}

static void leds_init_page(shell_app_context_t *ctx, int page)
{
    (void)ctx;
    esp_err_t err = led_modes_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_modes_start failed: %s", esp_err_to_name(err));
    }

    s_leds.page = page;
    memset(s_audio_choice_variants, 0, sizeof(s_audio_choice_variants));
    memset(s_custom_choice_variants, 0, sizeof(s_custom_choice_variants));

    s_leds.audio_choice = 0;
    s_leds.custom_choice = 0;
    int choice_idx = 0;
    int variant_idx = 0;
    if (animation_choice_find_for_source_mode(LEDS_MODE_SOURCE_AUDIO,
                                              led_beat_anim_name((int)led_beat_anim_get()),
                                              &choice_idx,
                                              &variant_idx)) {
        s_leds.audio_choice = choice_idx;
        s_audio_choice_variants[choice_idx] = (uint8_t)variant_idx;
    }
    if (animation_choice_find_for_source_mode(LEDS_MODE_SOURCE_CUSTOM,
                                              led_modes_name(led_modes_current()),
                                              &choice_idx,
                                              &variant_idx)) {
        s_leds.custom_choice = choice_idx;
        s_custom_choice_variants[choice_idx] = (uint8_t)variant_idx;
    }
    s_leds.audio_sel = animation_selection_index(s_leds.audio_choice);
    s_leds.custom_sel = animation_selection_index(s_leds.custom_choice);

    leds_load_color_state();
    leds_apply_color();
    leds_apply_mode_for_page();

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

void leds_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    size_t selected = (s_leds.page == LEDS_PAGE_AUDIO) ? s_leds.audio_sel : s_leds.custom_sel;
    shell_ui_menu_tick(dt_sec, selected);
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type != INPUT_EVENT_PRESS) return;

    shell_menu_entry_t *entries = NULL;
    leds_menu_meta_t *meta = NULL;
    size_t count = 0;
    size_t *sel = NULL;
    if (s_leds.page == LEDS_PAGE_AUDIO) {
        entries = s_audio_entries;
        meta = s_audio_meta;
        count = build_audio_entries();
        sel = &s_leds.audio_sel;
    } else {
        entries = s_custom_entries;
        meta = s_custom_meta;
        count = build_custom_entries();
        sel = &s_leds.custom_sel;
    }
    if (!entries || !meta || !sel || count == 0) return;
    *sel = clamp_sel(*sel, count);

    if (ev->button == INPUT_BTN_A && *sel > 0) {
        (*sel)--;
        return;
    }
    if (ev->button == INPUT_BTN_B && (*sel + 1) < count) {
        (*sel)++;
        return;
    }
    leds_menu_meta_t selected = meta[*sel];

    if (ev->button == INPUT_BTN_D) {
        switch (selected.kind) {
            case LEDS_ENTRY_KIND_BACK:
                if (ctx->request_switch) {
                    ctx->request_switch("menu", ctx->request_user_data);
                }
                return;
            case LEDS_ENTRY_KIND_ANIM:
                leds_select_animation_for_page(s_leds.page, selected.value);
                return;
            case LEDS_ENTRY_KIND_CFG:
                leds_cycle_cfg(selected.value, true);
                return;
            default:
                return;
        }
    }

    if (ev->button == INPUT_BTN_C) {
        if (selected.kind == LEDS_ENTRY_KIND_CFG) {
            leds_cycle_cfg(selected.value, false);
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
