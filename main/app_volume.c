#include "app_shell.h"

#include <math.h>
#include <stdio.h>

#include "app_settings.h"
#include "audio.h"
#include "oled.h"

const shell_legend_t VOLUME_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_MUTE },
};

typedef struct {
    float vol;      // 0.0 .. 2.0
    float prev_vol; // for mute toggle
    bool  muted;
} volume_state_t;
static volume_state_t s_volume = { .vol = 1.0f, .prev_vol = 1.0f, .muted = false };

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
static void volume_apply(float v)
{
    // Clamp UI volume to 0.0 .. 2.0
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;

    s_volume.vol = v;
    app_settings_set_volume(s_volume.vol, s_volume.muted, true);
}

void volume_init(shell_app_context_t *ctx)
{
    (void)ctx;
    const app_settings_t *settings = app_settings_get();
    float v = settings ? settings->volume : audio_get_volume();
    if (v < 0.0f) v = 0.0f;
    if (v > 2.0f) v = 2.0f;
    s_volume.vol = v;
    s_volume.prev_vol = v > 0.0f ? v : 1.0f;
    s_volume.muted = settings ? settings->muted : false;
}

void volume_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type == INPUT_EVENT_PRESS){
        if (ev->button == INPUT_BTN_A){ // up -> louder
            volume_apply(s_volume.vol + 0.1f);
        } else if (ev->button == INPUT_BTN_B){ // down -> quieter
            volume_apply(s_volume.vol - 0.1f);
        } else if (ev->button == INPUT_BTN_D){ // mute toggle
            if (!s_volume.muted){
                s_volume.prev_vol = s_volume.vol > 0.0f ? s_volume.vol : 1.0f;
                s_volume.muted = true;
                app_settings_set_volume(s_volume.vol, s_volume.muted, true);
            } else {
                s_volume.muted = false;
                s_volume.vol = s_volume.prev_vol;
                app_settings_set_volume(s_volume.vol, s_volume.muted, true);
            }
        }
    }
}

void volume_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)w; (void)h;
    oled_draw_text3x5(fb, x + 2, y + 2, "VOLUME");
    int pct = (int)lroundf(s_volume.vol * 100.0f); // 0..200% (0.0..2.0)
    if (pct < 0) pct = 0;
    if (pct > 200) pct = 200;
    char line[24];
    snprintf(line, sizeof(line), "Level:%3d%%", pct);
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    int bar_w = w - 10;
    int filled = (pct * (bar_w - 2)) / 200;
    int bar_x = x + 2;
    int bar_y = y + 30;
    fb_rect_outline(fb, bar_x, bar_y, bar_w, 5);
    if (!s_volume.muted && filled > 0){
        fb_rect_fill(fb, bar_x + 1, bar_y + 1, filled, 3);
    }
}
