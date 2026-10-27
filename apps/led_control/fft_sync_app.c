#include "led_control_apps.h"
#include "led_source.h"
#include "audio_bus.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "fft.h"

static const char *TAG = "led_controller";

enum {
    CONTROLLER_BACK = 0,
    CONTROLLER_RESET,
    CONTROLLER_FORCE,
    CONTROLLER_OFFSET,
    CONTROLLER_COUNT,
};

enum { OFFSET_MULTI_PRESS_US = 700000 };

static size_t s_selected;
static shell_menu_motion_t s_motion;
static int64_t s_last_offset_press_us;
static unsigned s_offset_press_count;
static char s_force_label[20];
static char s_offset_label[24];

shell_legend_t FFT_SYNC_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_SELECT },
};

static void refresh_labels(void)
{
    fft_sync_state_t state = {0};
    fft_get_sync_state(&state);
    snprintf(s_force_label, sizeof(s_force_label), "Force: %s",
             state.bpm_locked ? "on" : "off");
    snprintf(s_offset_label, sizeof(s_offset_label), "Offset: %s",
             led_source_timing_name(led_source_timing_get()));
}

static void offset_press(void)
{
    int64_t now = esp_timer_get_time();
    bool in_window = s_last_offset_press_us > 0 &&
                     now - s_last_offset_press_us <= OFFSET_MULTI_PRESS_US;
    led_timing_mode_t next;

    if (!in_window) {
        next = led_source_timing_get() == LED_TIMING_ON_BEAT
                 ? LED_TIMING_OFFBEAT : LED_TIMING_ON_BEAT;
        s_offset_press_count = 1;
    } else {
        s_offset_press_count++;
        if (s_offset_press_count == 2) {
            next = LED_TIMING_DOUBLE;
        } else {
            next = (led_timing_mode_t)((led_source_timing_get() + 1) % 3);
        }
    }

    s_last_offset_press_us = now;
    led_source_timing_set(next);
}

void fft_sync_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_selected = CONTROLLER_RESET;
    s_last_offset_press_us = 0;
    s_offset_press_count = 0;
    shell_ui_menu_reset(&s_motion, s_selected);

    if (led_source_mode_get() == LED_SOURCE_FFT && !audio_bus_init_if_needed()) {
        ESP_LOGE(TAG, "audio_init failed; FFT source unavailable");
    }
    led_source_service_tick(0.0f);
    refresh_labels();
}

void fft_sync_app_deinit(shell_app_context_t *ctx) { (void)ctx; }

void fft_sync_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    shell_ui_menu_tick(&s_motion, dt_sec, s_selected);
}

void fft_sync_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;
    if (ev->button == INPUT_BTN_A) {
        if (s_selected > 0) s_selected--;
        return;
    }
    if (ev->button == INPUT_BTN_B) {
        if (s_selected + 1 < CONTROLLER_COUNT) s_selected++;
        return;
    }
    if (ev->button != INPUT_BTN_D) return;

    switch (s_selected) {
        case CONTROLLER_BACK:
            if (ctx->request_switch) ctx->request_switch("menu", ctx->request_user_data);
            break;
        case CONTROLLER_RESET:
            fft_detector_reset();
            break;
        case CONTROLLER_FORCE: {
            fft_sync_state_t state = {0};
            fft_get_sync_state(&state);
            fft_beat_lock_set(!state.bpm_locked);
            break;
        }
        case CONTROLLER_OFFSET:
            offset_press();
            break;
        default:
            break;
    }
    refresh_labels();
}

void fft_sync_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;
    refresh_labels();
    const shell_menu_entry_t entries[CONTROLLER_COUNT] = {
        { .id = "back", .label = "Back" },
        { .id = "reset", .label = "Reset" },
        { .id = "force", .label = s_force_label },
        { .id = "offset", .label = s_offset_label },
    };
    shell_menu_view_t view = {
        .entries = entries,
        .count = CONTROLLER_COUNT,
        .selected = s_selected,
        .title = "LED / CONTROLLER",
        .motion = &s_motion,
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}
