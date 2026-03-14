#include "shell_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// Local framebuffer helpers (1bpp linear buffer)
static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static inline void fb_pset_clip(uint8_t *fb, int x, int y, int x0, int y0, int x1, int y1)
{
    if ((unsigned)x < (unsigned)x0 || (unsigned)x > (unsigned)x1) return;
    if ((unsigned)y < (unsigned)y0 || (unsigned)y > (unsigned)y1) return;
    fb_pset(fb, x, y);
}

static void fb_rect_clear(uint8_t *fb, int x0, int y0, int w, int h)
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
            int idx = y * PANEL_W + x;
            fb[idx >> 3] &= (uint8_t)~(1u << (7 - (idx & 7)));
        }
    }
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

// ----------------------------------------------------------------------
// 5x5 legend icons + helpers
// ----------------------------------------------------------------------
static const uint8_t ICONS[][5] = {
    {0, 0, 0, 0, 0},                   // NONE
    {0b00100, 0b01110, 0b11111, 0b00100, 0b00100}, // UP
    {0b00100, 0b00100, 0b11111, 0b01110, 0b00100}, // DOWN
    {0b00110, 0b01100, 0b11111, 0b01100, 0b00110}, // LEFT
    {0b01100, 0b00110, 0b11111, 0b00110, 0b01100}, // RIGHT
    {0b00100, 0b01110, 0b11011, 0b01110, 0b00100}, // SELECT
    {0b00110, 0b01001, 0b00010, 0b10010, 0b01100}, // REFRESH
    {0b10001, 0b01010, 0b00100, 0b01010, 0b10001}, // MUTE (X)
    {0b01000, 0b01100, 0b01110, 0b01100, 0b01000}, // PLAY (triangle)
    {0b11011, 0b11011, 0b11011, 0b11011, 0b11011}, // PAUSE (bars)
    {0b00110, 0b00101, 0b00100, 0b01100, 0b01100}, // music note
    {0b01110, 0b01010, 0b11111, 0b11111, 0b11111}, // lock
    {0b01110, 0b11111, 0b11111, 0b11111, 0b01110}, // record
    {0b11111, 0b11111, 0b11111, 0b11111, 0b11111}, // stop
};

static const uint8_t DIR_ICONS_2048[][5] = {
    {0b00100, 0b01010, 0b10001, 0b10001, 0b11111}, // UP
    {0b11111, 0b10001, 0b10001, 0b01010, 0b00100}, // DOWN
    {0b00111, 0b01001, 0b10001, 0b01001, 0b00111}, // LEFT
    {0b11100, 0b10010, 0b10001, 0b10010, 0b11100}, // RIGHT
};

static shell_direction_icon_style_t s_direction_icon_style = SHELL_DIRECTION_ICON_STYLE_SHELL;

static const uint8_t *legend_icon_rows(shell_icon_id_t icon)
{
    if (icon >= SHELL_ICON_UP && icon <= SHELL_ICON_RIGHT &&
        s_direction_icon_style == SHELL_DIRECTION_ICON_STYLE_2048) {
        return DIR_ICONS_2048[icon - SHELL_ICON_UP];
    }
    if (icon <= SHELL_ICON_NONE || icon > SHELL_ICON_STOP) return NULL;
    return ICONS[icon];
}

static void draw_icon5(uint8_t *fb, int x, int y, shell_icon_id_t icon)
{
    const uint8_t *rows = legend_icon_rows(icon);
    if (!rows) return;
    for (int r = 0; r < 5; ++r) {
        uint8_t row = rows[r];
        for (int c = 0; c < 5; ++c) {
            if (row & (1u << (4 - c))) {
                fb_pset(fb, x + c, y + r);
            }
        }
    }
}

void shell_ui_set_direction_icon_style(shell_direction_icon_style_t style)
{
    if (style != SHELL_DIRECTION_ICON_STYLE_2048) {
        style = SHELL_DIRECTION_ICON_STYLE_SHELL;
    }
    s_direction_icon_style = style;
}

shell_direction_icon_style_t shell_ui_get_direction_icon_style(void)
{
    return s_direction_icon_style;
}

static const char *master_ctrl_hud_name(system_master_ctrl_t state)
{
    switch (state) {
        case SYS_MASTER_CTRL_MASTER: return "mst";
        case SYS_MASTER_CTRL_SLAVE:  return "slv";
        default:                     return "off";
    }
}

void shell_ui_draw_hud(uint8_t *fb, const system_state_t *state, const char *left_label)
{
    if (!fb || !state) return;

    fb_rect_clear(fb, 0, 0, PANEL_W, SHELL_UI_HUD_HEIGHT);

    const int margin = 2;
    char status_buf[32];
    if (state->fft_running) {
        unsigned bpm = (unsigned)((state->fft_bpm_centi + 50u) / 100u);
        if (bpm > 0u) {
            snprintf(status_buf, sizeof(status_buf), "fft:%u mc:%s%s",
                     bpm,
                     master_ctrl_hud_name(state->master_ctrl),
                     state->sync_link_active ? "*" : "");
        } else {
            snprintf(status_buf, sizeof(status_buf), "fft:on mc:%s%s",
                     master_ctrl_hud_name(state->master_ctrl),
                     state->sync_link_active ? "*" : "");
        }
    } else {
        snprintf(status_buf, sizeof(status_buf), "fft:off mc:%s%s",
                 master_ctrl_hud_name(state->master_ctrl),
                 state->sync_link_active ? "*" : "");
    }

    int status_w = (int)strlen(status_buf) * 4 - 1;
    if (status_w < 0) status_w = 0;
    int status_x = PANEL_W - margin - status_w;
    if (status_x < margin) status_x = margin;
    oled_draw_text3x5(fb, status_x, 1, status_buf);

    const char *label = left_label ? left_label : state->led_mode_name;
    if (label && *label) {
        int max_chars = (status_x - margin - 2) / 4;
        if (max_chars < 0) max_chars = 0;
        if ((int)strlen(label) > max_chars) {
            static char scratch[32];
            int n = max_chars;
            if (n > (int)sizeof(scratch) - 1) n = (int)sizeof(scratch) - 1;
            strncpy(scratch, label, n);
            scratch[n] = '\0';
            oled_draw_text3x5(fb, margin, 1, scratch);
        } else {
            oled_draw_text3x5(fb, margin, 1, label);
        }
    }

    for (int x = 0; x < PANEL_W; ++x) {
        fb_pset(fb, x, SHELL_UI_HUD_HEIGHT - 1);
    }
}

void shell_ui_draw_legend(uint8_t *fb, const shell_legend_t *legend)
{
    if (!fb || !legend) return;

    // Clear legend band before drawing icons
    fb_rect_clear(fb, 0, PANEL_H - SHELL_UI_LEGEND_HEIGHT, PANEL_W, SHELL_UI_LEGEND_HEIGHT);

    int y = PANEL_H - SHELL_UI_LEGEND_HEIGHT + 1; // leave 1px gap above, 1px gap below
    int spacing = PANEL_W / 4;
    for (int i = 0; i < 4; ++i) {
        if (legend->slots[i] == SHELL_ICON_NONE) continue;
        int cx = spacing * i + spacing / 2 - 2;
        draw_icon5(fb, cx, y, legend->slots[i]);
    }
}

// ----------------------------------------------------------------------
// Menu list rendering with smooth scrolling
// ----------------------------------------------------------------------
static float s_menu_scroll = 0.0f;

void shell_ui_menu_reset(size_t selected)
{
    s_menu_scroll = (float)selected;
}

void shell_ui_menu_tick(float dt_sec, size_t selected)
{
    float target = (float)selected;
    float diff = target - s_menu_scroll;
    if (fabsf(diff) < 0.01f) {
        s_menu_scroll = target;
        return;
    }

    float speed = 10.0f; // items per second toward target
    float step = diff * fminf(1.0f, dt_sec * speed);
    // Clamp step to avoid huge jumps on long frames
    if (step > 0.5f) step = 0.5f;
    if (step < -0.5f) step = -0.5f;
    s_menu_scroll += step;
}

static void draw_menu_icon(uint8_t *fb, int x, int y, int clip_x0, int clip_y0, int clip_x1, int clip_y1)
{
    // Simple 9x9 placeholder glyph
    static const uint16_t kIcon[9] = {
        0b001111100,
        0b011111110,
        0b111111111,
        0b111001111,
        0b111001111,
        0b111111111,
        0b111111111,
        0b011111110,
        0b001111100,
    };
    for (int r = 0; r < 9; ++r) {
        uint16_t row = kIcon[r];
        for (int c = 0; c < 9; ++c) {
            if (row & (1u << (8 - c))) {
                fb_pset_clip(fb, x + c, y + r, clip_x0, clip_y0, clip_x1, clip_y1);
            }
        }
    }
}

void shell_ui_draw_menu(uint8_t *fb, int x, int y, int w, int h, const shell_menu_view_t *view)
{
    if (!fb || !view || !view->entries || view->count == 0) return;

    size_t selected = view->selected;
    if (selected >= view->count) selected = view->count - 1;

    const int frame_h = 15;
    const int gap     = 1;
    const int visible = (view->count < 3) ? (int)view->count : 3;
    const int item_h  = frame_h + gap;
    const int stack_h = frame_h * visible + gap * (visible - 1);
    int y0 = y + (h - stack_h) / 2;
    if (y0 < y + 1) y0 = y + 1;

    // Window start (in items, fractional) centers selection when possible
    float window_start = s_menu_scroll - 1.0f;
    float min_start = 0.0f;
    float max_start = (float)view->count - (float)visible;
    if (max_start < 0.0f) max_start = 0.0f;
    if (window_start < min_start) window_start = min_start;
    if (window_start > max_start) window_start = max_start;

    int start_idx = (int)floorf(window_start);
    float frac = window_start - (float)start_idx;
    float base_y = (float)y0 - frac * item_h;

    const int icon_w = 9;
    const int icon_gap = 4;
    const int outer_pad = 3;
    int frame_x = x + outer_pad + icon_w + icon_gap;
    int frame_w = w - frame_x - outer_pad;

    int items_to_draw = visible + 1; // extra for fractional scroll
    int clip_x0 = x;
    int clip_x1 = x + w - 1;
    int clip_y0 = y;
    int clip_y1 = y + h - 1;
    for (int i = 0; i < items_to_draw; ++i) {
        int idx = start_idx + i;
        if (idx >= (int)view->count) break;
        float fy_f = base_y + i * item_h;
        int fy = (int)lroundf(fy_f);
        int fy0 = fy;
        int fy1 = fy + frame_h - 1;
        if (fy1 < clip_y0) continue;
        if (fy0 > clip_y1) break;

        int icon_x = x + outer_pad;
        int icon_y = fy + (frame_h - icon_w) / 2;
        draw_menu_icon(fb, icon_x, icon_y, clip_x0, clip_y0, clip_x1, clip_y1);

        // beveled frame
        int fx0 = frame_x;
        int fx1 = frame_x + frame_w - 1;
        for (int dx = fx0 + 1; dx < fx1; ++dx) {
            fb_pset_clip(fb, dx, fy0, clip_x0, clip_y0, clip_x1, clip_y1);
            fb_pset_clip(fb, dx, fy1, clip_x0, clip_y0, clip_x1, clip_y1);
        }
        for (int dy = fy0 + 1; dy < fy1; ++dy) {
            fb_pset_clip(fb, fx0, dy, clip_x0, clip_y0, clip_x1, clip_y1);
            fb_pset_clip(fb, fx1, dy, clip_x0, clip_y0, clip_x1, clip_y1);
        }
        fb_pset_clip(fb, fx0 + 1, fy0 + 1, clip_x0, clip_y0, clip_x1, clip_y1);
        fb_pset_clip(fb, fx1 - 1, fy0 + 1, clip_x0, clip_y0, clip_x1, clip_y1);
        fb_pset_clip(fb, fx0 + 1, fy1 - 1, clip_x0, clip_y0, clip_x1, clip_y1);
        fb_pset_clip(fb, fx1 - 1, fy1 - 1, clip_x0, clip_y0, clip_x1, clip_y1);

        // Highlight bar for the selected entry
        if ((size_t)idx == selected) {
            for (int dy = fy0 + 2; dy <= fy1 - 2; ++dy) {
                fb_pset_clip(fb, fx0 + 1, dy, clip_x0, clip_y0, clip_x1, clip_y1);
                fb_pset_clip(fb, fx0 + 2, dy, clip_x0, clip_y0, clip_x1, clip_y1);
            }
        }

        int text_x = fx0 + 4;
        int text_y = fy + (frame_h - 5) / 2;
        if (text_y <= clip_y1 && text_y + 4 >= clip_y0) {
            oled_draw_text3x5(fb, text_x, text_y, view->entries[idx].label);
        }
    }
}
