#include "led_control_apps.h"
#include "led_source.h"

#include <stdio.h>
#include <math.h>

#include "fft.h"

enum {
    CONTROLLER_BACK = 0,
    CONTROLLER_RESET,
    CONTROLLER_FORCE,
    CONTROLLER_OFFSET,
    CONTROLLER_PHASE,
    CONTROLLER_COUNT,
};


static size_t s_selected;
static shell_menu_motion_t s_motion;
static char s_force_label[20];
static char s_offset_label[24];
static char s_phase_label[24];

shell_legend_t FFT_SYNC_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_SELECT },
};

static void refresh_labels(void)
{
    fft_sync_state_t state = {0};
    fft_get_sync_state(&state);
    snprintf(s_force_label, sizeof(s_force_label), "Force: %s",
             state.bpm_locked ? "on" : "off");
    led_timing_mode_t timing = led_source_timing_get();
    const char *name = led_source_mode_get() == LED_SOURCE_FFT && timing == LED_TIMING_ON_BEAT
        ? "auto" : led_source_timing_name(timing);
    snprintf(s_offset_label, sizeof(s_offset_label), "Set: %s", name);
    float offset = led_source_phase_offset_get();
    offset -= floorf(offset + .5f);
    if (led_source_is_following()) snprintf(s_phase_label, sizeof(s_phase_label), "Phase: follow");
    else snprintf(s_phase_label, sizeof(s_phase_label), "Phase: %+.2f", (double)offset);
    FFT_SYNC_LEGEND.slots[2] = s_selected == CONTROLLER_PHASE ? SHELL_ICON_LEFT : SHELL_ICON_NONE;
    FFT_SYNC_LEGEND.slots[3] = s_selected == CONTROLLER_PHASE ? SHELL_ICON_RIGHT : SHELL_ICON_SELECT;
}


void fft_sync_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_selected = CONTROLLER_RESET;
    shell_ui_menu_reset(&s_motion, s_selected);

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
    if (!ctx || !ev || (ev->type != INPUT_EVENT_PRESS && ev->type != INPUT_EVENT_LONG_PRESS)) return;
    if (s_selected == CONTROLLER_PHASE &&
        (ev->button == INPUT_BTN_C || ev->button == INPUT_BTN_D)) {
        float step = ev->type == INPUT_EVENT_LONG_PRESS ? .05f : .01f;
        led_source_phase_offset_adjust(ev->button == INPUT_BTN_C ? -step : step);
        refresh_labels();
        return;
    }
    if (ev->type != INPUT_EVENT_PRESS) return;
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
            led_source_timing_press();
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
        { .id = "phase", .label = s_phase_label },
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
