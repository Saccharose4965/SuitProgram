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

typedef enum {
    LEDS_BACKEND_BEAT = 0,
    LEDS_BACKEND_CUSTOM,
} leds_backend_t;

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

    if (backend == LEDS_BACKEND_CUSTOM && strcmp(name, "off") == 0) {
        snprintf(buf, buf_len, "off");
        return;
    }

    snprintf(buf, buf_len, "%s:%s",
             (backend == LEDS_BACKEND_BEAT) ? "beat" : "sync",
             name);
}

static size_t append_mode_entries(size_t n, size_t cap, leds_backend_t backend)
{
    int count = (backend == LEDS_BACKEND_BEAT) ? led_beat_anim_count() : led_modes_count();
    for (int i = 0; i < count && n < cap; ++i) {
        const char *name = (backend == LEDS_BACKEND_BEAT) ? led_beat_anim_name(i) : led_modes_name(i);
        if (!name) continue;
        format_mode_label(backend, i, s_entry_labels[n], LEDS_LABEL_LEN);
        s_entries[n] = (shell_menu_entry_t){ .id = s_entry_labels[n], .label = s_entry_labels[n] };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_MODE, .backend = backend, .value = i };
        n++;
    }
    return n;
}

static size_t build_entries(void)
{
    size_t n = 0;
    const size_t cap = LEDS_MAX_ENTRIES;

    s_entries[n] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_BACK, .backend = LEDS_BACKEND_CUSTOM, .value = 0 };
    n++;

    n = append_mode_entries(n, cap, LEDS_BACKEND_BEAT);
    n = append_mode_entries(n, cap, LEDS_BACKEND_CUSTOM);

    refresh_runtime_labels();
    if (n < cap) {
        s_entries[n] = (shell_menu_entry_t){ .id = s_speed_label, .label = s_speed_label };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG_SPEED, .backend = LEDS_BACKEND_CUSTOM, .value = 0 };
        n++;
    }
    if (n < cap) {
        s_entries[n] = (shell_menu_entry_t){ .id = s_plane_bg_label, .label = s_plane_bg_label };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG_PLANE_BG, .backend = LEDS_BACKEND_CUSTOM, .value = 0 };
        n++;
    }
    if (n < cap) {
        s_entries[n] = (shell_menu_entry_t){ .id = s_ring_bg_label, .label = s_ring_bg_label };
        s_meta[n] = (leds_menu_meta_t){ .kind = LEDS_ENTRY_KIND_CFG_RING_BG, .backend = LEDS_BACKEND_CUSTOM, .value = 0 };
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

static void leds_apply_current_mode(void)
{
    char label[LEDS_LABEL_LEN];

    led_beat_plane_background_enable(s_leds.plane_bg);
    led_modes_plane_background_enable(s_leds.plane_bg);
    led_beat_ring_background_enable(s_leds.ring_bg);
    led_modes_ring_background_enable(s_leds.ring_bg);
    led_modes_set_speed_percent(s_leds.custom_speed_percent);
    led_ui_fft_request(LED_UI_FFT_REQ_LED_APPS, false);

    if (s_leds.current_backend == LEDS_BACKEND_BEAT) {
        int mode_idx = clamp_int(s_leds.current_mode, 0, led_beat_anim_count() - 1);
        s_leds.current_mode = mode_idx;
        format_mode_label(LEDS_BACKEND_BEAT, mode_idx, label, sizeof(label));
        led_modes_enable(false);
        led_modes_set_sync(false);
        led_beat_anim_set((led_beat_anim_t)mode_idx);
        led_beat_enable(true);
        system_state_set_led_mode(100 + mode_idx, label);
        return;
    }

    int mode_idx = clamp_int(s_leds.current_mode, 0, led_modes_count() - 1);
    s_leds.current_mode = mode_idx;
    format_mode_label(LEDS_BACKEND_CUSTOM, mode_idx, label, sizeof(label));
    led_beat_enable(false);
    led_modes_set_sync(true);
    led_modes_set(mode_idx);
    led_modes_enable(true);
    system_state_set_led_mode(mode_idx, label);
}

void leds_animations_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    esp_err_t err = led_modes_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_modes_start failed: %s", esp_err_to_name(err));
    }

    s_leds.custom_speed_percent = (uint8_t)clamp_int((int)led_modes_get_speed_percent(), 10, 250);
    s_leds.plane_bg = led_beat_plane_background_enabled() || led_modes_plane_background_enabled();
    s_leds.ring_bg = led_beat_ring_background_enabled() || led_modes_ring_background_enabled();

    if (led_modes_enabled()) {
        s_leds.current_backend = LEDS_BACKEND_CUSTOM;
        s_leds.current_mode = clamp_int(led_modes_current(), 0, led_modes_count() - 1);
    } else if (led_beat_enabled()) {
        s_leds.current_backend = LEDS_BACKEND_BEAT;
        s_leds.current_mode = clamp_int((int)led_beat_anim_get(), 0, led_beat_anim_count() - 1);
    } else {
        s_leds.current_backend = LEDS_BACKEND_CUSTOM;
        s_leds.current_mode = 0;
    }

    leds_apply_current_mode();
    s_leds.selected = entry_index_for_mode(s_leds.current_backend, s_leds.current_mode);
    shell_ui_menu_reset(s_leds.selected);
}

void leds_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    led_ui_fft_request(LED_UI_FFT_REQ_LED_APPS, false);
}

void leds_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    shell_ui_menu_tick(dt_sec, s_leds.selected);
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;

    size_t count = build_entries();
    if (count == 0) return;
    s_leds.selected = clamp_sel(s_leds.selected, count);

    if (ev->button == INPUT_BTN_A) {
        if (s_leds.selected > 0) s_leds.selected--;
        return;
    }
    if (ev->button == INPUT_BTN_B) {
        if (s_leds.selected + 1 < count) s_leds.selected++;
        return;
    }
    if (ev->button != INPUT_BTN_C && ev->button != INPUT_BTN_D) {
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
            s_leds.custom_speed_percent = (uint8_t)clamp_int((int)s_leds.custom_speed_percent +
                                                             ((ev->button == INPUT_BTN_D) ? 10 : -10),
                                                             10, 250);
            led_modes_set_speed_percent(s_leds.custom_speed_percent);
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

    shell_menu_view_t view = {
        .entries = s_entries,
        .count = count,
        .selected = s_leds.selected,
        .title = "LED / ANIMS",
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}
