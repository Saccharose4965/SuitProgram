#include "app_shell.h"
#include "shell_apps.h"

#include <math.h>
#include <stdio.h>

#include "fft.h"
#include "led.h"
#include "led_modes.h"
#include "oled.h"

const shell_legend_t MANUAL_BPM_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_LEFT, SHELL_ICON_RIGHT },
};

typedef struct {
    bool active;
    float bpm;
    float cycle_phase;
    float phase_offset;
    int beat_flash_ticks;
} manual_bpm_state_t;

static manual_bpm_state_t s_manual_bpm = {
    .bpm = 120.0f,
    .cycle_phase = 0.0f,
    .phase_offset = 0.0f,
    .beat_flash_ticks = 0,
};

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

static void manual_bpm_trigger_beat(void)
{
    led_trigger_beat(255, 96, 0);
    s_manual_bpm.beat_flash_ticks = 3;
}

static void manual_bpm_shift_phase(float delta)
{
    s_manual_bpm.phase_offset = wrap01(s_manual_bpm.phase_offset + delta);
}

static int manual_bpm_beat_count_between(float start_phase, float end_phase, float trigger_phase)
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
    s_manual_bpm.active = true;
    s_manual_bpm.bpm = clamp_bpm(s_manual_bpm.bpm);
    s_manual_bpm.cycle_phase = wrap01(s_manual_bpm.cycle_phase);
    s_manual_bpm.phase_offset = wrap01(s_manual_bpm.phase_offset);
    s_manual_bpm.beat_flash_ticks = 0;

    fft_set_display_enabled(false);
    if (fft_visualizer_running()) {
        fft_visualizer_stop();
    }

    led_modes_enable(false);
    led_beat_enable(true);
}

void manual_bpm_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    s_manual_bpm.active = false;
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
            s_manual_bpm.bpm = clamp_bpm(s_manual_bpm.bpm - bpm_step);
            break;
        case INPUT_BTN_B:
            s_manual_bpm.bpm = clamp_bpm(s_manual_bpm.bpm + bpm_step);
            break;
        case INPUT_BTN_C:
            manual_bpm_shift_phase(-phase_step);
            break;
        case INPUT_BTN_D:
            manual_bpm_shift_phase(phase_step);
            break;
        default:
            break;
    }
}

void manual_bpm_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    if (dt_sec <= 0.0f) return;

    float start_phase = s_manual_bpm.cycle_phase;
    float end_phase = start_phase + dt_sec * (s_manual_bpm.bpm / 60.0f);
    int beats = manual_bpm_beat_count_between(start_phase, end_phase, s_manual_bpm.phase_offset);
    while (beats-- > 0) {
        manual_bpm_trigger_beat();
    }
    s_manual_bpm.cycle_phase = wrap01(end_phase);

    if (s_manual_bpm.beat_flash_ticks > 0) {
        s_manual_bpm.beat_flash_ticks--;
    }
}

void manual_bpm_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)h;
    if (!fb) return;

    char line[32];
    const char *mode = led_beat_anim_name((int)led_beat_anim_get());
    if (!mode) mode = "?";

    oled_draw_text3x5(fb, x + 2, y + 2, "MANUAL BPM");

    snprintf(line, sizeof(line), "mode:%s", mode);
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "bpm:%3u", (unsigned)lroundf(s_manual_bpm.bpm));
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    snprintf(line, sizeof(line), "ofs:%3u%%", (unsigned)lroundf(s_manual_bpm.phase_offset * 100.0f));
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    int bar_x = x + 2;
    int bar_w = w - 8;
    int phase_fill = (int)lroundf((float)(bar_w - 2) * s_manual_bpm.cycle_phase);
    if (phase_fill < 0) phase_fill = 0;
    if (phase_fill > bar_w - 2) phase_fill = bar_w - 2;
    int marker_x = bar_x + 1 + (int)lroundf((float)(bar_w - 3) * s_manual_bpm.phase_offset);
    if (marker_x < bar_x + 1) marker_x = bar_x + 1;
    if (marker_x > bar_x + bar_w - 2) marker_x = bar_x + bar_w - 2;
    fb_rect_outline(fb, bar_x, y + 35, bar_w, 6);
    if (phase_fill > 0) {
        fb_rect_fill(fb, bar_x + 1, y + 36, phase_fill, 4);
    }
    fb_rect_fill(fb, marker_x, y + 34, 1, 8);

    if (s_manual_bpm.beat_flash_ticks > 0) {
        fb_rect_fill(fb, x + w - 8, y + 2, 5, 5);
    }

}

bool manual_bpm_get_sync_state(manual_bpm_sync_state_t *out)
{
    if (!out) return false;
    out->active = s_manual_bpm.active;
    out->bpm = clamp_bpm(s_manual_bpm.bpm);
    out->cycle_phase = wrap01(s_manual_bpm.cycle_phase);
    out->phase_offset = wrap01(s_manual_bpm.phase_offset);
    return true;
}
