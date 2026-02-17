#include "app_shell.h"

#include <math.h>
#include <stdio.h>

#include "app_settings.h"
#include "audio.h"
#include "oled.h"

const shell_legend_t VOLUME_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_OK, SHELL_ICON_MUTE },
};

typedef struct {
    float speaker_vol;      // 0.0 .. 2.0
    float bt_vol;           // 0.0 .. 2.0
    float prev_speaker_vol; // restore point for mute toggle
    float prev_bt_vol;      // restore point for mute toggle
    bool  speaker_muted;
    bool  bt_muted;
    int   selected;         // 0: speaker, 1: bt
} volume_state_t;
static volume_state_t s_volume = {
    .speaker_vol = 1.0f,
    .bt_vol = 1.0f,
    .prev_speaker_vol = 1.0f,
    .prev_bt_vol = 1.0f,
    .speaker_muted = false,
    .bt_muted = false,
    .selected = 0,
};

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_fill(uint8_t *fb, int x0, int y0, int w, int h)
{
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
    if (w <= 0 || h <= 0) return;
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
static float clamp_vol(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;
    return v;
}

static int vol_to_pct(float v)
{
    int pct = (int)lroundf(clamp_vol(v) * 100.0f); // 0..200% (0.0..2.0)
    if (pct < 0) pct = 0;
    if (pct > 200) pct = 200;
    return pct;
}

static void volume_apply_all(bool persist)
{
    s_volume.speaker_vol = clamp_vol(s_volume.speaker_vol);
    s_volume.bt_vol = clamp_vol(s_volume.bt_vol);
    app_settings_set_audio(s_volume.speaker_vol, s_volume.speaker_muted,
                           s_volume.bt_vol, s_volume.bt_muted,
                           persist);
}

void volume_init(shell_app_context_t *ctx)
{
    (void)ctx;
    const app_settings_t *settings = app_settings_get();
    float spk = settings ? settings->speaker_volume : audio_get_volume();
    float bt = settings ? settings->bt_volume : spk;
    s_volume.speaker_vol = clamp_vol(spk);
    s_volume.bt_vol = clamp_vol(bt);
    s_volume.prev_speaker_vol = s_volume.speaker_vol > 0.0f ? s_volume.speaker_vol : 1.0f;
    s_volume.prev_bt_vol = s_volume.bt_vol > 0.0f ? s_volume.bt_vol : 1.0f;
    s_volume.speaker_muted = settings ? settings->speaker_muted : false;
    s_volume.bt_muted = settings ? settings->bt_muted : false;
    s_volume.selected = 0;
}

void volume_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type == INPUT_EVENT_PRESS){
        if (ev->button == INPUT_BTN_A){ // up -> louder
            if (s_volume.selected == 0) {
                s_volume.speaker_vol += 0.1f;
            } else {
                s_volume.bt_vol += 0.1f;
            }
            volume_apply_all(true);
        } else if (ev->button == INPUT_BTN_B){ // down -> quieter
            if (s_volume.selected == 0) {
                s_volume.speaker_vol -= 0.1f;
            } else {
                s_volume.bt_vol -= 0.1f;
            }
            volume_apply_all(true);
        } else if (ev->button == INPUT_BTN_C){ // switch target
            s_volume.selected = (s_volume.selected == 0) ? 1 : 0;
        } else if (ev->button == INPUT_BTN_D){ // mute toggle
            if (s_volume.selected == 0) {
                if (!s_volume.speaker_muted) {
                    s_volume.prev_speaker_vol = s_volume.speaker_vol > 0.0f ? s_volume.speaker_vol : 1.0f;
                    s_volume.speaker_muted = true;
                } else {
                    s_volume.speaker_muted = false;
                    s_volume.speaker_vol = s_volume.prev_speaker_vol;
                }
            } else {
                if (!s_volume.bt_muted) {
                    s_volume.prev_bt_vol = s_volume.bt_vol > 0.0f ? s_volume.bt_vol : 1.0f;
                    s_volume.bt_muted = true;
                } else {
                    s_volume.bt_muted = false;
                    s_volume.bt_vol = s_volume.prev_bt_vol;
                }
            }
            volume_apply_all(true);
        }
    }
}

void volume_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)w; (void)h;
    int spk_pct = vol_to_pct(s_volume.speaker_vol);
    int bt_pct = vol_to_pct(s_volume.bt_vol);

    oled_draw_text3x5(fb, x + 2, y + 2, "VOLUME");

    char line[24];
    snprintf(line, sizeof(line), "Sel:%s", (s_volume.selected == 0) ? "SPK" : "BT");
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    snprintf(line, sizeof(line), "%cSPK:%3d%%%s",
             (s_volume.selected == 0) ? '>' : ' ',
             spk_pct,
             s_volume.speaker_muted ? " M" : "");
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    snprintf(line, sizeof(line), "%cBT :%3d%%%s",
             (s_volume.selected == 1) ? '>' : ' ',
             bt_pct,
             s_volume.bt_muted ? " M" : "");
    oled_draw_text3x5(fb, x + 2, y + 26, line);

    int bar_w = w - 10;
    int bar_x = x + 2;

    int spk_filled = (spk_pct * (bar_w - 2)) / 200;
    int bt_filled = (bt_pct * (bar_w - 2)) / 200;

    fb_rect_outline(fb, bar_x, y + 36, bar_w, 5);
    if (!s_volume.speaker_muted && spk_filled > 0) {
        fb_rect_fill(fb, bar_x + 1, y + 37, spk_filled, 3);
    }

    fb_rect_outline(fb, bar_x, y + 44, bar_w, 5);
    if (!s_volume.bt_muted && bt_filled > 0) {
        fb_rect_fill(fb, bar_x + 1, y + 45, bt_filled, 3);
    }
}
