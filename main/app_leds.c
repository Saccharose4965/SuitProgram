#include "app_shell.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "led.h"
#include "led_modes.h"

#include "app_led_support.h"

const shell_legend_t LEDS_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
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
    LEDS_CFG_SPEED = 0,
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

#define LEDS_VARIANT_MAX 8

typedef struct {
    const char *label;
    const char *variant_labels[LEDS_VARIANT_MAX];
    const char *audio_names[LEDS_VARIANT_MAX];
    const char *custom_names[LEDS_VARIANT_MAX];
    uint8_t variant_count;
} leds_animation_choice_t;

static const leds_animation_choice_t s_animation_choices[] = {
    { "off",          { NULL,      NULL,      NULL      }, { NULL,          NULL,          NULL      }, { "off",     NULL,      NULL      }, 1 },
    { "fill",         { NULL,      NULL,      NULL      }, { "flash",       NULL,          NULL      }, { "fill",    NULL,      NULL      }, 1 },
    { "blink",        { "hard",    "smooth",  NULL      }, { "flash",       "soft_flash",  NULL      }, { "blink",   "blink_smooth", NULL }, 2 },
    { "breathe",      { NULL,      NULL,      NULL      }, { "breathe",     NULL,          NULL      }, { "breathe", NULL,      NULL      }, 1 },
    { "energy",       { NULL,      NULL,      NULL      }, { "energy",      NULL,          NULL      }, { "energy",  NULL,      NULL      }, 1 },
    { "twinkle",      { "soft",    "scatter", NULL      }, { "spark",       "shock",       NULL      }, { "twinkle", "glitch",  NULL      }, 2 },
    { "chase",        { "single",  "pair",    "swarm",     "single+bg", "pair+bg", "swarm+bg" },
                       { "comet",       "comet_pair",    "comet_swarm",
                         "comet_bg",    "comet_pair_bg", "comet_swarm_bg" },
                       { "chase",       "chase_pair",    "chase_swarm",
                         "chase_bg",    "chase_pair_bg", "chase_swarm_bg" }, 6 },
    { "plane span",   { NULL,      "bg",      NULL      }, { "plane_span",  "plane_span",  NULL      }, { "plane",   "plane",   NULL      }, 2 },
    { "plane",        { "sweep",   "pair",    "sweep+bg",  "pair+bg" },
                       { "plane_sweep", "plane_pair", "plane_sweep", "plane_pair" },
                       { "sweep",   "mirror",  "sweep",     "mirror" }, 4 },
    { "ring",         { "long",    "short",   "long+bg",  "short+bg" },
                       { "ring_pulse", "ring_train", "ring_pulse", "ring_train" },
                       { "ring",    "ring_short", "ring",   "ring_short" }, 4 },
    { "orbit",        { NULL,      NULL,      NULL      }, { "ring_train",  NULL,          NULL      }, { "orbit",   NULL,      NULL      }, 1 },
    { "aurora",       { NULL,      NULL,      NULL      }, { "plane_fan",   NULL,          NULL      }, { "aurora",  NULL,      NULL      }, 1 },
    { "vortex",       { NULL,      NULL,      NULL      }, { "crossfire",   NULL,          NULL      }, { "vortex",  NULL,      NULL      }, 1 },
    { "helix",        { NULL,      NULL,      NULL      }, { "crossfire",   NULL,          NULL      }, { "helix",   NULL,      NULL      }, 1 },
    { "random",       { NULL,      NULL,      NULL      }, { "random",      NULL,          NULL      }, { "random",  NULL,      NULL      }, 1 },
};
static const int kAnimationChoiceCount =
    (int)(sizeof(s_animation_choices) / sizeof(s_animation_choices[0]));

static const char *TAG = "app_leds";

#define LEDS_MAX_PAGE_ENTRIES 32
#define LEDS_ENTRY_LABEL_LEN 24
static shell_menu_entry_t s_audio_entries[LEDS_MAX_PAGE_ENTRIES];
static shell_menu_entry_t s_custom_entries[LEDS_MAX_PAGE_ENTRIES];
static leds_menu_meta_t s_audio_meta[LEDS_MAX_PAGE_ENTRIES];
static leds_menu_meta_t s_custom_meta[LEDS_MAX_PAGE_ENTRIES];
static char s_audio_entry_labels[LEDS_MAX_PAGE_ENTRIES][LEDS_ENTRY_LABEL_LEN];
static char s_custom_entry_labels[LEDS_MAX_PAGE_ENTRIES][LEDS_ENTRY_LABEL_LEN];
static char s_speed_label[14];

typedef struct {
    int page;
    size_t audio_sel;
    size_t custom_sel;
    int audio_choice;
    int custom_choice;
    uint8_t custom_speed_percent;
} leds_state_t;

static leds_state_t s_leds = {0};
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
    return (int)s_animation_choices[choice_idx].variant_count;
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

static bool animation_choice_is_plane(int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    return strncmp(s_animation_choices[choice_idx].label, "plane", 5) == 0;
}

static bool animation_choice_is_ring(int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    return strcmp(s_animation_choices[choice_idx].label, "ring") == 0;
}

static int animation_choice_background_variant_span(int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    const char *label = s_animation_choices[choice_idx].label;
    if (strcmp(label, "plane span") == 0) return 1;
    if (strcmp(label, "plane") == 0) return 2;
    if (strcmp(label, "ring") == 0) return 2;
    return 0;
}

static int animation_variant_base_index(int choice_idx, int variant_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    variant_idx = clamp_int(variant_idx, 0, animation_variant_count(choice_idx) - 1);
    int span = animation_choice_background_variant_span(choice_idx);
    if (span > 0 && variant_idx >= span) {
        return variant_idx - span;
    }
    return variant_idx;
}

static bool animation_variant_has_background(int choice_idx, int variant_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    variant_idx = clamp_int(variant_idx, 0, animation_variant_count(choice_idx) - 1);
    int span = animation_choice_background_variant_span(choice_idx);
    return span > 0 && variant_idx >= span;
}

static int animation_variant_with_background(int choice_idx, int variant_idx, bool enabled)
{
    int base_idx = animation_variant_base_index(choice_idx, variant_idx);
    int span = animation_choice_background_variant_span(choice_idx);
    if (span <= 0) return base_idx;
    return enabled ? (base_idx + span) : base_idx;
}

static bool plane_background_enabled_for_page(int page)
{
    int choice_idx = (page == LEDS_PAGE_AUDIO) ? s_leds.audio_choice : s_leds.custom_choice;
    int variant_idx = animation_variant_index_for_page(page, choice_idx);
    return animation_choice_is_plane(choice_idx) &&
           animation_variant_has_background(choice_idx, variant_idx);
}

static bool ring_background_enabled_for_page(int page)
{
    int choice_idx = (page == LEDS_PAGE_AUDIO) ? s_leds.audio_choice : s_leds.custom_choice;
    int variant_idx = animation_variant_index_for_page(page, choice_idx);
    return animation_choice_is_ring(choice_idx) &&
           animation_variant_has_background(choice_idx, variant_idx);
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
    if (strcmp(mode_name, "scanner") == 0) {
        mode_name = (source == LEDS_MODE_SOURCE_AUDIO) ? "comet" : "chase";
    } else if (strcmp(mode_name, "audio_energy") == 0) {
        mode_name = "energy";
    } else if (strcmp(mode_name, "pulse") == 0) {
        mode_name = "breathe";
    } else if (strcmp(mode_name, "contour") == 0) {
        mode_name = "ring_short";
    } else if (strcmp(mode_name, "prism") == 0) {
        mode_name = "plane";
    }

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

static void refresh_runtime_labels(void)
{
    snprintf(s_speed_label, sizeof(s_speed_label), "speed:%u%%",
             (unsigned)s_leds.custom_speed_percent);
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
    if (page == LEDS_PAGE_CUSTOM) {
        refresh_runtime_labels();
        append_cfg_entry(entries, meta, &n, cap, LEDS_CFG_SPEED, s_speed_label);
    }
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

static bool leds_fft_needed(void)
{
    return (s_leds.page == LEDS_PAGE_AUDIO) ||
           led_beat_enabled() ||
           led_audio_brightness_enabled();
}

static void leds_sync_fft_state(void)
{
    led_ui_fft_request(LED_UI_FFT_REQ_LED_APPS, leds_fft_needed());
}

static void leds_apply_audio(void)
{
    char label[LEDS_ENTRY_LABEL_LEN];

    led_modes_enable(false);
    s_leds.audio_choice = clamp_int(s_leds.audio_choice, 0, kAnimationChoiceCount - 1);
    int variant_idx = animation_variant_index_for_page(LEDS_PAGE_AUDIO, s_leds.audio_choice);
    led_beat_plane_background_enable(plane_background_enabled_for_page(LEDS_PAGE_AUDIO));
    led_beat_ring_background_enable(ring_background_enabled_for_page(LEDS_PAGE_AUDIO));
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
    led_modes_plane_background_enable(plane_background_enabled_for_page(LEDS_PAGE_CUSTOM));
    led_modes_ring_background_enable(ring_background_enabled_for_page(LEDS_PAGE_CUSTOM));
    led_modes_set_speed_percent(s_leds.custom_speed_percent);
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

static void leds_activate_animation_for_page(int page, int choice_idx)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    if (page == LEDS_PAGE_AUDIO) {
        s_leds.audio_choice = choice_idx;
        leds_apply_audio();
    } else {
        s_leds.custom_choice = choice_idx;
        leds_apply_custom();
    }
}

static void leds_cycle_animation_variant_for_page(int page, int choice_idx, bool inc)
{
    choice_idx = clamp_int(choice_idx, 0, kAnimationChoiceCount - 1);
    uint8_t *variants = variant_state_for_page(page);
    int count = animation_variant_count(choice_idx);
    if (count > 1) {
        int idx = animation_variant_index_for_page(page, choice_idx);
        idx = (idx + (inc ? 1 : -1) + count) % count;
        variants[choice_idx] = (uint8_t)idx;
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
    if (cfg_idx != LEDS_CFG_SPEED) return;
    s_leds.custom_speed_percent = (uint8_t)clamp_int((int)s_leds.custom_speed_percent +
                                                     (inc ? 10 : -10),
                                                     10, 250);
    led_modes_set_speed_percent(s_leds.custom_speed_percent);
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
    s_leds.custom_speed_percent = (uint8_t)clamp_int((int)led_modes_get_speed_percent(), 10, 250);

    int choice_idx = 0;
    int variant_idx = 0;
    if (animation_choice_find_for_source_mode(LEDS_MODE_SOURCE_AUDIO,
                                              led_beat_anim_name((int)led_beat_anim_get()),
                                              &choice_idx,
                                              &variant_idx)) {
        s_leds.audio_choice = choice_idx;
        variant_idx = animation_variant_with_background(choice_idx,
                                                        variant_idx,
                                                        animation_choice_is_plane(choice_idx)
                                                            ? led_beat_plane_background_enabled()
                                                            : led_beat_ring_background_enabled());
        s_audio_choice_variants[choice_idx] = (uint8_t)variant_idx;
    }
    if (animation_choice_find_for_source_mode(LEDS_MODE_SOURCE_CUSTOM,
                                              led_modes_name(led_modes_current()),
                                              &choice_idx,
                                              &variant_idx)) {
        s_leds.custom_choice = choice_idx;
        variant_idx = animation_variant_with_background(choice_idx,
                                                        variant_idx,
                                                        animation_choice_is_plane(choice_idx)
                                                            ? led_modes_plane_background_enabled()
                                                            : led_modes_ring_background_enabled());
        s_custom_choice_variants[choice_idx] = (uint8_t)variant_idx;
    }
    s_leds.audio_sel = animation_selection_index(s_leds.audio_choice);
    s_leds.custom_sel = animation_selection_index(s_leds.custom_choice);

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

void leds_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    led_ui_fft_request(LED_UI_FFT_REQ_LED_APPS, false);
}

void leds_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    size_t selected = (s_leds.page == LEDS_PAGE_AUDIO) ? s_leds.audio_sel : s_leds.custom_sel;
    shell_ui_menu_tick(dt_sec, selected);
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;

    leds_menu_meta_t *meta = NULL;
    size_t count = 0;
    size_t *sel = NULL;
    if (s_leds.page == LEDS_PAGE_AUDIO) {
        meta = s_audio_meta;
        count = build_audio_entries();
        sel = &s_leds.audio_sel;
    } else {
        meta = s_custom_meta;
        count = build_custom_entries();
        sel = &s_leds.custom_sel;
    }
    if (!meta || !sel || count == 0) return;
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
    if (ev->button != INPUT_BTN_C && ev->button != INPUT_BTN_D) {
        return;
    }

    bool inc = (ev->button == INPUT_BTN_D);
    if (selected.kind == LEDS_ENTRY_KIND_BACK) {
        if (ctx->request_switch) {
            ctx->request_switch("menu", ctx->request_user_data);
        }
        return;
    }
    if (selected.kind == LEDS_ENTRY_KIND_ANIM) {
        int current_choice = (s_leds.page == LEDS_PAGE_AUDIO)
            ? s_leds.audio_choice
            : s_leds.custom_choice;
        if (selected.value != current_choice) {
            leds_activate_animation_for_page(s_leds.page, selected.value);
            return;
        }
        leds_cycle_animation_variant_for_page(s_leds.page, selected.value, inc);
        return;
    }
    if (selected.kind == LEDS_ENTRY_KIND_CFG) {
        leds_cycle_cfg(selected.value, inc);
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
