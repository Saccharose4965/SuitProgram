#include "shell_apps.h"

#include <math.h>
#include <stdio.h>

#include "app_led_support.h"
#include "fft.h"
#include "led_modes.h"

const shell_legend_t LED_SOURCE_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
};

enum {
    LED_SOURCE_ENTRY_BACK = 0,
    LED_SOURCE_ENTRY_MODE,
    LED_SOURCE_ENTRY_MANUAL,
    LED_SOURCE_ENTRY_FFT,
    LED_SOURCE_ENTRY_COUNT,
};

typedef struct {
    size_t selected;
    char source_label[18];
} led_source_app_state_t;

static led_source_app_state_t s_source_app = {
    .selected = LED_SOURCE_ENTRY_MODE,
};

static led_source_mode_t s_source_mode = LED_SOURCE_FFT;
static manual_bpm_sync_state_t s_manual = {
    .active = false,
    .bpm = 120.0f,
    .cycle_phase = 0.0f,
    .phase_offset = 0.0f,
};
static uint32_t s_manual_beat_count = 0;
static bool s_fft_phase_valid = false;
static float s_fft_prev_aligned_phase = 0.0f;
static uint32_t s_fft_beat_count = 0;

static float clamp_bpm(float bpm)
{
    if (bpm < 32.0f) bpm = 32.0f;
    if (bpm > 255.0f) bpm = 255.0f;
    return bpm;
}

static float wrap01(float v)
{
    while (v < 0.0f) v += 1.0f;
    while (v >= 1.0f) v -= 1.0f;
    return v;
}

static bool led_source_blocked_by_slave(void)
{
    return master_control_hud_state() == SYS_MASTER_CTRL_SLAVE;
}

static int beat_count_between(float start_phase, float end_phase, float trigger_phase)
{
    float start_rel = start_phase - trigger_phase;
    float end_rel = end_phase - trigger_phase;
    int start_wrap = (int)floorf(start_rel);
    int end_wrap = (int)floorf(end_rel);
    return end_wrap - start_wrap;
}

static void led_source_refresh_labels(void)
{
    snprintf(s_source_app.source_label, sizeof(s_source_app.source_label),
             "src:%s", led_source_mode_name(s_source_mode));
}

const char *led_source_mode_name(led_source_mode_t mode)
{
    return (mode == LED_SOURCE_MANUAL) ? "manual" : "fft";
}

void led_source_mode_set(led_source_mode_t mode)
{
    if (mode != LED_SOURCE_MANUAL && mode != LED_SOURCE_FFT) {
        mode = LED_SOURCE_FFT;
    }
    if (mode != s_source_mode) {
        s_source_mode = mode;
        s_fft_phase_valid = false;
    }
    led_source_refresh_labels();
    led_ui_fft_request(LED_UI_FFT_REQ_LED_SOURCE,
                       (s_source_mode == LED_SOURCE_FFT) && !led_source_blocked_by_slave());
    fft_led_export_enable((s_source_mode == LED_SOURCE_FFT) && !led_source_blocked_by_slave());
}

led_source_mode_t led_source_mode_get(void)
{
    return s_source_mode;
}

bool led_source_manual_state_get(manual_bpm_sync_state_t *out)
{
    if (!out) return false;
    *out = s_manual;
    out->active = (s_source_mode == LED_SOURCE_MANUAL) && !led_source_blocked_by_slave();
    out->bpm = clamp_bpm(out->bpm);
    out->cycle_phase = wrap01(out->cycle_phase);
    out->phase_offset = wrap01(out->phase_offset);
    return true;
}

void led_source_manual_bpm_adjust(float delta)
{
    s_manual.bpm = clamp_bpm(s_manual.bpm + delta);
}

void led_source_manual_phase_adjust(float delta)
{
    s_manual.phase_offset = wrap01(s_manual.phase_offset + delta);
}

void led_source_service_tick(float dt_sec)
{
    bool blocked = led_source_blocked_by_slave();
    bool want_fft = (s_source_mode == LED_SOURCE_FFT) && !blocked;
    led_ui_fft_request(LED_UI_FFT_REQ_LED_SOURCE, want_fft);
    fft_led_export_enable(want_fft);

    if (blocked) {
        led_modes_set_sync_clock(false, 0.0f);
        return;
    }

    if (s_source_mode == LED_SOURCE_MANUAL) {
        float bpm = clamp_bpm(s_manual.bpm);
        if (dt_sec > 0.0f) {
            float start_phase = s_manual.cycle_phase;
            float end_phase = start_phase + dt_sec * (bpm / 60.0f);
            int beats = beat_count_between(start_phase, end_phase, s_manual.phase_offset);
            while (beats-- > 0) {
                led_trigger_beat(255, 96, 0);
                s_manual_beat_count++;
            }
            s_manual.cycle_phase = wrap01(end_phase);
        }

        float aligned_phase = wrap01(s_manual.cycle_phase - s_manual.phase_offset);
        float hz = bpm / 60.0f;
        float timeline_sec = (hz > 0.05f)
            ? (((float)s_manual_beat_count + aligned_phase) / hz)
            : 0.0f;
        led_modes_set_sync_clock(true, timeline_sec);
        return;
    }

    fft_sync_state_t st = {0};
    fft_get_sync_state(&st);
    if (!(st.running && st.beat_enabled && st.bpm > 0.5f)) {
        s_fft_phase_valid = false;
        led_modes_set_sync_clock(false, 0.0f);
        return;
    }

    float aligned_phase = wrap01(st.beat_phase - st.trigger_phase);
    if (!s_fft_phase_valid) {
        s_fft_beat_count = 0;
        s_fft_phase_valid = true;
    } else if (aligned_phase + 0.05f < s_fft_prev_aligned_phase) {
        s_fft_beat_count++;
    }
    s_fft_prev_aligned_phase = aligned_phase;

    float hz = st.bpm / 60.0f;
    float timeline_sec = (hz > 0.05f)
        ? (((float)s_fft_beat_count + aligned_phase) / hz)
        : 0.0f;
    led_modes_set_sync_clock(true, timeline_sec);
}

void led_source_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    led_source_refresh_labels();
    s_source_app.selected = LED_SOURCE_ENTRY_MODE;
    shell_ui_menu_reset(s_source_app.selected);
}

void led_source_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        if (s_source_app.selected > 0) s_source_app.selected--;
        return;
    }
    if (ev->button == INPUT_BTN_B) {
        if (s_source_app.selected + 1 < LED_SOURCE_ENTRY_COUNT) s_source_app.selected++;
        return;
    }
    if (ev->button != INPUT_BTN_C && ev->button != INPUT_BTN_D) {
        return;
    }

    switch (s_source_app.selected) {
        case LED_SOURCE_ENTRY_BACK:
            if (ctx->request_switch) {
                ctx->request_switch("menu", ctx->request_user_data);
            }
            break;
        case LED_SOURCE_ENTRY_MODE:
            led_source_mode_set((s_source_mode == LED_SOURCE_MANUAL)
                                    ? LED_SOURCE_FFT
                                    : LED_SOURCE_MANUAL);
            break;
        case LED_SOURCE_ENTRY_MANUAL:
            led_source_mode_set(LED_SOURCE_MANUAL);
            if (ctx->request_switch) {
                ctx->request_switch("manual_bpm", ctx->request_user_data);
            }
            break;
        case LED_SOURCE_ENTRY_FFT:
            led_source_mode_set(LED_SOURCE_FFT);
            if (ctx->request_switch) {
                ctx->request_switch("fft_sync", ctx->request_user_data);
            }
            break;
        default:
            break;
    }
}

void led_source_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    led_source_refresh_labels();

    const shell_menu_entry_t entries[LED_SOURCE_ENTRY_COUNT] = {
        { .id = "back", .label = "Back" },
        { .id = "mode", .label = s_source_app.source_label },
        { .id = "manual_bpm", .label = "Manual BPM" },
        { .id = "fft_sync", .label = "FFT Sync" },
    };

    shell_menu_view_t view = {
        .entries = entries,
        .count = LED_SOURCE_ENTRY_COUNT,
        .selected = s_source_app.selected,
        .title = "LED / SOURCE",
    };
    shell_ui_draw_menu(fb, x, y, w, h, &view);
}
