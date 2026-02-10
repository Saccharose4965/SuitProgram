#include "app_shell.h"

#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "led.h"
#include "led_modes.h"

const shell_legend_t LEDS_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_CUSTOM2 },
};

enum {
    LEDS_PAGE_ROOT = 0,
    LEDS_PAGE_AUDIO = 1,
    LEDS_PAGE_CUSTOM = 2,
};

static const char *TAG = "app_leds";
static const shell_menu_entry_t s_root_entries[] = {
    { "audio", "Audio Reactive" },
    { "custom", "Custom" },
};

#define LEDS_MAX_AUDIO_ENTRIES 8
#define LEDS_MAX_CUSTOM_ENTRIES 20
static shell_menu_entry_t s_audio_entries[1 + LEDS_MAX_AUDIO_ENTRIES];
static shell_menu_entry_t s_custom_entries[1 + LEDS_MAX_CUSTOM_ENTRIES];

typedef struct {
    int page;
    size_t root_sel;
    size_t audio_sel;  // includes "Back" at index 0
    size_t custom_sel; // includes "Back" at index 0
    int audio_mode;
    int custom_mode;
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

static size_t build_audio_entries(void)
{
    size_t n = 0;
    s_audio_entries[n++] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    int count = led_beat_anim_count();
    for (int i = 0; i < count && n < (sizeof(s_audio_entries) / sizeof(s_audio_entries[0])); ++i) {
        const char *name = led_beat_anim_name(i);
        if (!name) name = "?";
        s_audio_entries[n++] = (shell_menu_entry_t){ .id = name, .label = name };
    }
    return n;
}

static size_t build_custom_entries(void)
{
    size_t n = 0;
    s_custom_entries[n++] = (shell_menu_entry_t){ .id = "back", .label = "Back" };
    int count = led_modes_count();
    for (int i = 0; i < count && n < (sizeof(s_custom_entries) / sizeof(s_custom_entries[0])); ++i) {
        const char *name = led_modes_name(i);
        if (!name) name = "?";
        s_custom_entries[n++] = (shell_menu_entry_t){ .id = name, .label = name };
    }
    return n;
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

void leds_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    esp_err_t err = led_modes_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "led_modes_start failed: %s", esp_err_to_name(err));
    }

    s_leds.page = LEDS_PAGE_ROOT;
    s_leds.root_sel = 0;
    int audio_count = led_beat_anim_count();
    int custom_count = led_modes_count();
    s_leds.audio_mode = (audio_count > 0) ? clamp_int((int)led_beat_anim_get(), 0, audio_count - 1) : 0;
    s_leds.custom_mode = (custom_count > 0) ? clamp_int(led_modes_current(), 0, custom_count - 1) : 0;
    s_leds.audio_sel = (size_t)s_leds.audio_mode + 1;
    s_leds.custom_sel = (size_t)s_leds.custom_mode + 1;

    // Default LED behavior on entry: audio-reactive flash.
    s_leds.audio_mode = LED_BEAT_ANIM_FLASH;
    s_leds.audio_sel = (size_t)s_leds.audio_mode + 1;
    leds_apply_audio();
    shell_ui_menu_reset(s_leds.root_sel);
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;

    if (ev->type == INPUT_EVENT_LONG_PRESS &&
        (ev->button == INPUT_BTN_A || ev->button == INPUT_BTN_TOP_COMBO)) {
        ctx->request_switch("menu", ctx->request_user_data);
        return;
    }
    if (ev->type != INPUT_EVENT_PRESS) return;

    if (s_leds.page == LEDS_PAGE_ROOT) {
        size_t count = sizeof(s_root_entries) / sizeof(s_root_entries[0]);
        if (ev->button == INPUT_BTN_A && s_leds.root_sel > 0) {
            s_leds.root_sel--;
            shell_ui_menu_reset(s_leds.root_sel);
        } else if (ev->button == INPUT_BTN_B && (s_leds.root_sel + 1) < count) {
            s_leds.root_sel++;
            shell_ui_menu_reset(s_leds.root_sel);
        } else if (ev->button == INPUT_BTN_D) {
            s_leds.page = (s_leds.root_sel == 0) ? LEDS_PAGE_AUDIO : LEDS_PAGE_CUSTOM;
            shell_ui_menu_reset((s_leds.page == LEDS_PAGE_AUDIO) ? s_leds.audio_sel : s_leds.custom_sel);
        }
        return;
    }

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
    } else if (ev->button == INPUT_BTN_B && (*sel + 1) < count) {
        (*sel)++;
        shell_ui_menu_reset(*sel);
    } else if (ev->button == INPUT_BTN_D) {
        if (*sel == 0) {
            s_leds.page = LEDS_PAGE_ROOT;
            shell_ui_menu_reset(s_leds.root_sel);
            return;
        }
        if (s_leds.page == LEDS_PAGE_AUDIO) {
            s_leds.audio_mode = (int)(*sel - 1);
            leds_apply_audio();
        } else {
            s_leds.custom_mode = (int)(*sel - 1);
            leds_apply_custom();
        }
    }
}

void leds_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    const shell_menu_entry_t *entries = s_root_entries;
    size_t count = sizeof(s_root_entries) / sizeof(s_root_entries[0]);
    size_t selected = s_leds.root_sel;
    const char *title = "LED";

    if (s_leds.page == LEDS_PAGE_AUDIO) {
        count = build_audio_entries();
        entries = s_audio_entries;
        selected = clamp_sel(s_leds.audio_sel, count);
        s_leds.audio_sel = selected;
        title = "LED / AUDIO";
    } else if (s_leds.page == LEDS_PAGE_CUSTOM) {
        count = build_custom_entries();
        entries = s_custom_entries;
        selected = clamp_sel(s_leds.custom_sel, count);
        s_leds.custom_sel = selected;
        title = "LED / CUSTOM";
    } else {
        s_leds.root_sel = clamp_sel(s_leds.root_sel, count);
        selected = s_leds.root_sel;
    }

    shell_menu_view_t view = {
        .entries = entries,
        .count = count,
        .selected = selected,
        .title = title,
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}
