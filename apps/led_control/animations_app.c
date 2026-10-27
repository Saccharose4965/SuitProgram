#include "app_shell.h"
#include "led_control_apps.h"
#include "ui_draw.h"

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "led.h"
#include "led_modes.h"
#include "oled.h"
#include "settings.h"

#include "audio_policy.h"

shell_legend_t LEDS_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_DOWN, SHELL_ICON_RIGHT, SHELL_ICON_SELECT },
};
static bool s_speed_editing;

typedef enum {
    LEDS_BACKEND_BEAT = 0,
    LEDS_BACKEND_AMBIENT,
} leds_backend_t;

typedef enum {
    LEDS_CATALOGUE_BEAT = 0,
    LEDS_CATALOGUE_AMBIENT,
} leds_catalogue_t;

typedef enum {
    LEDS_ENTRY_KIND_BACK = 0,
    LEDS_ENTRY_KIND_MODE,
    LEDS_ENTRY_KIND_CFG_SPEED,
    LEDS_ENTRY_KIND_CFG_PLANE_BG,
    LEDS_ENTRY_KIND_CFG_RING_BG,
} leds_entry_kind_t;

typedef struct {
    leds_entry_kind_t kind;
    leds_backend_t backend;
    int value;
} leds_menu_meta_t;

typedef struct {
    size_t selected;
    shell_grid_motion_t motion;
    leds_catalogue_t catalogue;
    leds_backend_t current_backend;
    int current_mode;
    uint8_t custom_speed_percent;
    bool plane_bg;
    bool ring_bg;
} leds_state_t;

static const char *TAG = "app_leds";

#define LEDS_MAX_ENTRIES 64
#define LEDS_LABEL_LEN 24

static shell_menu_entry_t s_entries[LEDS_MAX_ENTRIES];
static leds_menu_meta_t s_meta[LEDS_MAX_ENTRIES];
static char s_entry_labels[LEDS_MAX_ENTRIES][LEDS_LABEL_LEN];
static char s_speed_label[14];
static char s_plane_bg_label[14];
static char s_ring_bg_label[14];
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

static void refresh_runtime_labels(void)
{
    snprintf(s_speed_label, sizeof(s_speed_label), "speed:%u%%",
             (unsigned)s_leds.custom_speed_percent);
    snprintf(s_plane_bg_label, sizeof(s_plane_bg_label), "plane bg:%s",
             s_leds.plane_bg ? "on" : "off");
    snprintf(s_ring_bg_label, sizeof(s_ring_bg_label), "ring bg:%s",
             s_leds.ring_bg ? "on" : "off");
}

static void format_mode_label(leds_backend_t backend, int mode_idx,
                              char *buf, size_t buf_len)
{
    const char *name = (backend == LEDS_BACKEND_BEAT)
        ? led_beat_anim_name(mode_idx)
        : led_modes_name(mode_idx);
    if (!name) name = "?";
    if (backend == LEDS_BACKEND_BEAT && mode_idx == LED_BEAT_ANIM_AUDIO_ENERGY) {
        name = "audio energy";
    }
    snprintf(buf, buf_len, "%s", name);
    for (char *p = buf; *p; ++p) if (*p == '_') *p = ' ';
}

static size_t append_mode_entry(size_t n, size_t cap, leds_backend_t backend, int mode)
{
    if (n >= cap) return n;
    const char *name = (backend == LEDS_BACKEND_BEAT)
        ? led_beat_anim_name(mode) : led_modes_name(mode);
    if (!name) return n;
    format_mode_label(backend, mode, s_entry_labels[n], LEDS_LABEL_LEN);
    s_entries[n] = (shell_menu_entry_t){ .id = s_entry_labels[n], .label = s_entry_labels[n] };
    s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_MODE, .backend = backend, .value = mode };
    return n + 1;
}

static size_t build_entries(void)
{
    size_t n = 0;
    const size_t cap = LEDS_MAX_ENTRIES;

    s_entries[n] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_BACK, .backend = LEDS_BACKEND_AMBIENT, .value = 0 };
    n++;

    if (s_leds.catalogue == LEDS_CATALOGUE_BEAT) {
        for (int mode = 0; mode < led_beat_anim_count(); ++mode) {
            if (mode != LED_BEAT_ANIM_AUDIO_ENERGY) {
                n = append_mode_entry(n, cap, LEDS_BACKEND_BEAT, mode);
            }
        }
    } else {
        n = append_mode_entry(n, cap, LEDS_BACKEND_BEAT, LED_BEAT_ANIM_AUDIO_ENERGY);
        for (int mode = 0; mode < led_modes_count(); ++mode) {
            n = append_mode_entry(n, cap, LEDS_BACKEND_AMBIENT, mode);
        }
    }

    for (size_t i = 2; i < n; ++i) {
        for (size_t j = i; j > 1 && strcmp(s_entry_labels[j - 1], s_entry_labels[j]) > 0; --j) {
            char label[LEDS_LABEL_LEN];
            memcpy(label, s_entry_labels[j], sizeof(label));
            memcpy(s_entry_labels[j], s_entry_labels[j - 1], sizeof(label));
            memcpy(s_entry_labels[j - 1], label, sizeof(label));
            leds_menu_meta_t meta = s_meta[j];
            s_meta[j] = s_meta[j - 1];
            s_meta[j - 1] = meta;
        }
    }

    refresh_runtime_labels();
    if (s_leds.catalogue == LEDS_CATALOGUE_AMBIENT && n < cap) {
        s_entries[n] = (shell_menu_entry_t){ .id = s_speed_label, .label = s_speed_label };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG_SPEED, .backend = LEDS_BACKEND_AMBIENT, .value = 0 };
        n++;
    }
    if (n < cap) {
        s_entries[n] = (shell_menu_entry_t){ .id = s_plane_bg_label, .label = s_plane_bg_label };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG_PLANE_BG, .backend = LEDS_BACKEND_AMBIENT, .value = 0 };
        n++;
    }
    if (n < cap) {
        s_entries[n] = (shell_menu_entry_t){ .id = s_ring_bg_label, .label = s_ring_bg_label };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG_RING_BG, .backend = LEDS_BACKEND_AMBIENT, .value = 0 };
        n++;
    }
    return n;
}

static size_t entry_index_for_mode(leds_backend_t backend, int mode_idx)
{
    size_t count = build_entries();
    for (size_t i = 1; i < count; ++i) {
        if (s_meta[i].kind != LEDS_ENTRY_KIND_MODE) continue;
        if (s_meta[i].backend == backend && s_meta[i].value == mode_idx) {
            return i;
        }
    }
    return 0;
}

static settings_led_animation_t leds_current_preference(void)
{
    return (settings_led_animation_t){
        .backend = s_leds.current_backend == LEDS_BACKEND_AMBIENT
            ? SETTINGS_LED_BACKEND_AMBIENT : SETTINGS_LED_BACKEND_BEAT,
        .mode = s_leds.current_mode,
        .ambient_speed_percent = s_leds.custom_speed_percent,
        .plane_background_enabled = s_leds.plane_bg,
        .ring_background_enabled = s_leds.ring_bg,
    };
}

static void leds_store_current_preference(bool persist)
{
    settings_led_animation_t preference = leds_current_preference();
    settings_set_led_animation(&preference, persist);
}

static void leds_apply_current_mode(void)
{
    char label[LEDS_LABEL_LEN];
    audio_policy_fft_request(AUDIO_POLICY_FFT_REQ_LED_APPS, false);

    if (s_leds.current_backend == LEDS_BACKEND_BEAT) {
        int mode_idx = clamp_int(s_leds.current_mode, 0, led_beat_anim_count() - 1);
        s_leds.current_mode = mode_idx;
        format_mode_label(LEDS_BACKEND_BEAT, mode_idx, label, sizeof(label));
        leds_store_current_preference(true);
        system_state_set_led_mode(100 + mode_idx, label);
        return;
    }

    int mode_idx = clamp_int(s_leds.current_mode, 0, led_modes_count() - 1);
    s_leds.current_mode = mode_idx;
    format_mode_label(LEDS_BACKEND_AMBIENT, mode_idx, label, sizeof(label));
    leds_store_current_preference(true);
    system_state_set_led_mode(mode_idx, label);
}

static void leds_catalogue_init(shell_app_context_t *ctx, leds_catalogue_t catalogue)
{
    s_speed_editing = false;
    (void)ctx;
    esp_err_t err = led_modes_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_modes_start failed: %s", esp_err_to_name(err));
    }

    s_leds.custom_speed_percent = (uint8_t)clamp_int((int)led_modes_get_speed_percent(), 10, 250);
    s_leds.plane_bg = led_beat_plane_background_enabled() || led_modes_plane_background_enabled();
    s_leds.ring_bg = led_beat_ring_background_enabled() || led_modes_ring_background_enabled();

    s_leds.catalogue = catalogue;
    if (catalogue == LEDS_CATALOGUE_AMBIENT && led_beat_enabled() &&
        led_beat_anim_get() == LED_BEAT_ANIM_AUDIO_ENERGY) {
        s_leds.current_backend = LEDS_BACKEND_BEAT;
        s_leds.current_mode = LED_BEAT_ANIM_AUDIO_ENERGY;
    } else if (catalogue == LEDS_CATALOGUE_AMBIENT) {
        s_leds.current_backend = LEDS_BACKEND_AMBIENT;
        s_leds.current_mode = clamp_int(led_modes_current(), 0, led_modes_count() - 1);
    } else {
        s_leds.current_backend = LEDS_BACKEND_BEAT;
        s_leds.current_mode = clamp_int((int)led_beat_anim_get(), 0, led_beat_anim_count() - 1);
        if (s_leds.current_mode == LED_BEAT_ANIM_AUDIO_ENERGY) {
            s_leds.current_mode = LED_BEAT_ANIM_FLASH;
        }
    }

    s_leds.selected = entry_index_for_mode(s_leds.current_backend, s_leds.current_mode);
    shell_ui_grid_reset(&s_leds.motion, s_leds.selected);
    LEDS_LEGEND=(shell_legend_t){
        .slots={SHELL_ICON_LEFT,SHELL_ICON_DOWN,SHELL_ICON_RIGHT,SHELL_ICON_SELECT},
    };
}

void leds_animations_app_init(shell_app_context_t *ctx)
{
    leds_catalogue_init(ctx, LEDS_CATALOGUE_BEAT);
}

void leds_ambient_app_init(shell_app_context_t *ctx)
{
    leds_catalogue_init(ctx, LEDS_CATALOGUE_AMBIENT);
}

void leds_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    if (s_speed_editing) leds_store_current_preference(true);
    s_speed_editing = false;
    audio_policy_fft_request(AUDIO_POLICY_FFT_REQ_LED_APPS, false);
}

void leds_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    shell_ui_grid_tick(&s_leds.motion, dt_sec, s_leds.selected);
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;

    size_t count = build_entries();
    if (count == 0) return;
    s_leds.selected = clamp_sel(s_leds.selected, count);

    if (s_speed_editing) {
        if (ev->button == INPUT_BTN_A || ev->button == INPUT_BTN_C) {
            s_leds.custom_speed_percent = (uint8_t)clamp_int(
                (int)s_leds.custom_speed_percent + (ev->button == INPUT_BTN_C ? 10 : -10), 10, 250);
            leds_store_current_preference(false);
            return;
        }
        s_speed_editing = false;
        LEDS_LEGEND=(shell_legend_t){
            .slots={SHELL_ICON_LEFT,SHELL_ICON_DOWN,SHELL_ICON_RIGHT,SHELL_ICON_SELECT},
        };
        if (ev->button == INPUT_BTN_D) {
            leds_store_current_preference(true);
            return;
        }
    }
    if (ev->button == INPUT_BTN_A) {
        size_t row=s_leds.selected%3u;
        size_t col=s_leds.selected/3u;
        if(col>0) s_leds.selected=(col-1u)*3u+row;
        return;
    }
    if (ev->button == INPUT_BTN_B) {
        size_t col_start=(s_leds.selected/3u)*3u;
        size_t in_col=count-col_start;
        if(in_col>3u) in_col=3u;
        s_leds.selected=col_start+(s_leds.selected-col_start+1u)%in_col;
        return;
    }
    if (ev->button == INPUT_BTN_C) {
        size_t row=s_leds.selected%3u;
        size_t target=(s_leds.selected/3u+1u)*3u+row;
        if(target<count) s_leds.selected=target;
        else {
            size_t next_col=(s_leds.selected/3u+1u)*3u;
            if(next_col<count) s_leds.selected=count-1u;
        }
        return;
    }
    if (ev->button != INPUT_BTN_D) {
        return;
    }

    leds_menu_meta_t selected = s_meta[s_leds.selected];
    switch (selected.kind) {
        case LEDS_ENTRY_KIND_BACK:
            if (ctx->request_switch) {
                ctx->request_switch("menu", ctx->request_user_data);
            }
            break;
        case LEDS_ENTRY_KIND_MODE:
            s_leds.current_backend = selected.backend;
            s_leds.current_mode = selected.value;
            leds_apply_current_mode();
            break;
        case LEDS_ENTRY_KIND_CFG_SPEED:
            s_speed_editing = true;
            LEDS_LEGEND=(shell_legend_t){
                .slots={SHELL_ICON_LEFT,SHELL_ICON_NONE,SHELL_ICON_RIGHT,SHELL_ICON_SELECT},
            };
            break;
        case LEDS_ENTRY_KIND_CFG_PLANE_BG:
            s_leds.plane_bg = !s_leds.plane_bg;
            leds_apply_current_mode();
            break;
        case LEDS_ENTRY_KIND_CFG_RING_BG:
            s_leds.ring_bg = !s_leds.ring_bg;
            leds_apply_current_mode();
            break;
        default:
            break;
    }
}

void leds_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    size_t count = build_entries();
    s_leds.selected = clamp_sel(s_leds.selected, count);

    shell_grid_view_t view={
        .entries=s_entries,
        .count=count,
        .selected=s_leds.selected,
        .motion=&s_leds.motion,
    };
    shell_ui_draw_grid(fb,x,y,w,h,&view);
}
