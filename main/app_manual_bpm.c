#include "app_shell.h"
#include "shell_apps.h"

#include <math.h>
#include <stdio.h>

#include "led.h"
#include "led_modes.h"
#include "oled.h"

const shell_legend_t MANUAL_BPM_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
};

typedef struct {
    float prev_phase;
    bool have_prev_phase;
    int beat_flash_ticks;
} manual_bpm_ui_state_t;

static manual_bpm_ui_state_t s_manual_ui = {0};

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if (!fb) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_fill(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            fb_pset(fb, x, y);
        }
    }
}

static void fb_rect_outline(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int x = x0; x <= x1; ++x) {
        fb_pset(fb, x, y0);
        fb_pset(fb, x, y1);
    }
    for (int y = y0; y <= y1; ++y) {
        fb_pset(fb, x0, y);
        fb_pset(fb, x1, y);
    }
}

static float wrap01(float v)
{
    while (v < 0.0f) v += 1.0f;
    while (v >= 1.0f) v -= 1.0f;
    return v;
}

static int beat_count_between(float start_phase, float end_phase, float trigger_phase)
{
    float start_rel = start_phase - trigger_phase;
    float end_rel = end_phase - trigger_phase;
    int start_wrap = (int)floorf(start_rel);
    int end_wrap = (int)floorf(end_rel);
    return end_wrap - start_wrap;
}

void manual_bpm_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    led_source_mode_set(LED_SOURCE_MANUAL);

    manual_bpm_sync_state_t st = {0};
    (void)led_source_manual_state_get(&st);
    s_manual_ui.prev_phase = st.cycle_phase;
    s_manual_ui.have_prev_phase = true;
    s_manual_ui.beat_flash_ticks = 0;
}

void manual_bpm_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
}

void manual_bpm_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev) return;
    if (ev->type != INPUT_EVENT_PRESS && ev->type != INPUT_EVENT_LONG_PRESS) return;

    bool coarse = (ev->type == INPUT_EVENT_LONG_PRESS);
    float bpm_step = coarse ? 5.0f : 1.0f;
    float phase_step = coarse ? 0.10f : 0.02f;

    switch (ev->button) {
        case INPUT_BTN_A:
            led_source_manual_bpm_adjust(-bpm_step);
            break;
        case INPUT_BTN_B:
            led_source_manual_bpm_adjust(bpm_step);
            break;
        case INPUT_BTN_C:
            led_source_manual_phase_adjust(-phase_step);
            break;
        case INPUT_BTN_D:
            led_source_manual_phase_adjust(phase_step);
            break;
        default:
            break;
    }
}

void manual_bpm_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    (void)dt_sec;

    manual_bpm_sync_state_t st = {0};
    if (!led_source_manual_state_get(&st)) return;

    if (!s_manual_ui.have_prev_phase) {
        s_manual_ui.prev_phase = st.cycle_phase;
        s_manual_ui.have_prev_phase = true;
    } else {
        int beats = beat_count_between(s_manual_ui.prev_phase, st.cycle_phase, st.phase_offset);
        if (beats > 0) {
            s_manual_ui.beat_flash_ticks = 3;
        }
        s_manual_ui.prev_phase = st.cycle_phase;
    }

    if (s_manual_ui.beat_flash_ticks > 0) {
        s_manual_ui.beat_flash_ticks--;
    }
}

void manual_bpm_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)h;
    if (!fb) return;

    manual_bpm_sync_state_t st = {0};
    if (!led_source_manual_state_get(&st)) {
        oled_draw_text3x5(fb, x + 2, y + 2, "MANUAL BPM");
        oled_draw_text3x5(fb, x + 2, y + 12, "state unavailable");
        return;
    }

    char line[32];
    const char *mode = led_modes_enabled()
        ? led_modes_name(led_modes_current())
        : led_beat_anim_name((int)led_beat_anim_get());
    if (!mode) mode = "?";

    oled_draw_text3x5(fb, x + 2, y + 2, "MANUAL BPM");

    snprintf(line, sizeof(line), "%s:%s",
             led_modes_enabled() ? "sync" : "beat",
             mode);
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "bpm:%3u", (unsigned)lroundf(st.bpm));
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    snprintf(line, sizeof(line), "ofs:%3u%%", (unsigned)lroundf(wrap01(st.phase_offset) * 100.0f));
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    int bar_x = x + 2;
    int bar_w = w - 8;
    int phase_fill = (int)lroundf((float)(bar_w - 2) * wrap01(st.cycle_phase));
    if (phase_fill < 0) phase_fill = 0;
    if (phase_fill > bar_w - 2) phase_fill = bar_w - 2;
    int marker_x = bar_x + 1 + (int)lroundf((float)(bar_w - 3) * wrap01(st.phase_offset));
    if (marker_x < bar_x + 1) marker_x = bar_x + 1;
    if (marker_x > bar_x + bar_w - 2) marker_x = bar_x + bar_w - 2;
    fb_rect_outline(fb, bar_x, y + 35, bar_w, 6);
    if (phase_fill > 0) {
        fb_rect_fill(fb, bar_x + 1, y + 36, phase_fill, 4);
    }
    fb_rect_fill(fb, marker_x, y + 34, 1, 8);

    if (s_manual_ui.beat_flash_ticks > 0) {
        fb_rect_fill(fb, x + w - 8, y + 2, 5, 5);
    }
}

bool manual_bpm_get_sync_state(manual_bpm_sync_state_t *out)
{
    return led_source_manual_state_get(out);
}
