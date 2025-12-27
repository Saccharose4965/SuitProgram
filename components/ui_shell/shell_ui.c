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
    {0b10000, 0b11000, 0b11100, 0b11000, 0b10000}, // BACK
    {0b00100, 0b01110, 0b11111, 0b00100, 0b00100}, // UP
    {0b00100, 0b00100, 0b11111, 0b01110, 0b00100}, // DOWN
    {0b00110, 0b01100, 0b11111, 0b01100, 0b00110}, // LEFT
    {0b01100, 0b00110, 0b11111, 0b00110, 0b01100}, // RIGHT
    {0b00000, 0b00100, 0b01010, 0b00100, 0b01110}, // OK (diamond)
    {0b11111, 0b00000, 0b11111, 0b00000, 0b11111}, // MENU (stack)
    {0b00100, 0b01110, 0b11111, 0b00100, 0b00100}, // LINK / sync (reuse UP)
    {0b01101, 0b10011, 0b10111, 0b10000, 0b01111}, // CUSTOM1 scan
    {0b11110, 0b00001, 0b01101, 0b11111, 0b01100}, // CUSTOM2 select
    {0b10001, 0b01010, 0b00100, 0b01010, 0b10001}, // MUTE (X)
};

static void draw_icon5(uint8_t *fb, int x, int y, shell_icon_id_t icon)
{
    if (icon <= SHELL_ICON_NONE || icon > SHELL_ICON_MUTE) return;
    const uint8_t *rows = ICONS[icon];
    for (int r = 0; r < 5; ++r) {
        uint8_t row = rows[r];
        for (int c = 0; c < 5; ++c) {
            if (row & (1u << (4 - c))) {
                fb_pset(fb, x + c, y + r);
            }
        }
    }
}

static void draw_battery_icon(uint8_t *fb, int x, int y, uint8_t pct)
{
    // 8x5 body with 1x3 terminal; fits in HUD 5px height band
    const int w = 8;
    const int h = 5;
    fb_rect_fill(fb, x, y, w - 2, 1);          // top
    fb_rect_fill(fb, x, y + h - 1, w - 2, 1);  // bottom
    fb_rect_fill(fb, x, y, 1, h);              // left
    fb_rect_fill(fb, x + w - 2, y, 1, h);      // right
    fb_rect_fill(fb, x + w - 1, y + 1, 1, h - 2); // terminal

    int fill_w = ((w - 3) * pct) / 100;
    if (fill_w > 0) {
        fb_rect_fill(fb, x + 1, y + 1, fill_w, h - 2);
    }
}

static void draw_connection_icon(uint8_t *fb, int x, int y, system_connection_t conn)
{
    // simple wifi-ish bars (5x5) that fit in the HUD band
    const uint8_t rows_disconnected[5] = {
        0b00000,
        0b00100,
        0b00000,
        0b00100,
        0b00000,
    };
    const uint8_t rows_connecting[5] = {
        0b00100,
        0b01010,
        0b00000,
        0b00100,
        0b00000,
    };
    const uint8_t rows_connected[5] = {
        0b00100,
        0b01010,
        0b10001,
        0b00000,
        0b00100,
    };

    const uint8_t *rows = rows_disconnected;
    switch (conn) {
        case SYS_CONN_CONNECTING: rows = rows_connecting; break;
        case SYS_CONN_CONNECTED:  rows = rows_connected;  break;
        default: break;
    }

    for (int r = 0; r < 5; ++r) {
        uint8_t row = rows[r];
        for (int c = 0; c < 5; ++c) {
            if (row & (1u << (4 - c))) {
                fb_pset(fb, x + c, y + r);
            }
        }
    }
}

void shell_ui_draw_hud(uint8_t *fb, const system_state_t *state, const char *left_label)
{
    if (!fb || !state) return;

    // Clear the band to avoid clipping from content underneath
    fb_rect_clear(fb, 0, 0, PANEL_W, SHELL_UI_HUD_HEIGHT);

    char time_buf[8];
    if (state->time_valid) {
        unsigned h = state->hours % 100;
        unsigned m = state->minutes % 100;
        snprintf(time_buf, sizeof(time_buf), "%02u:%02u", h, m);
    } else {
        strncpy(time_buf, "--:--", sizeof(time_buf));
        time_buf[sizeof(time_buf) - 1] = '\0';
    }

    // Layout from the right edge leftwards: time, connection icon, batteries
    const int margin = 2;
    const int time_w = 19; // "HH:MM" in 3x5 font → 5*4 - 1
    int time_x = PANEL_W - margin - time_w;
    oled_draw_text3x5(fb, time_x, 1, time_buf);

    int conn_x = time_x - 7; // 5px icon + 2px gap
    draw_connection_icon(fb, conn_x, 1, state->connection);

    const int battery_w = 8;
    const int battery_gap = 1;
    const int battery_total = 3 * battery_w + 2 * battery_gap;
    int bx = conn_x - battery_total - 3;
    for (int i = 0; i < 3; ++i) {
        draw_battery_icon(fb, bx + i * (battery_w + battery_gap), 1, state->battery_pct[i]);
    }

    // Left label (truncated to avoid colliding with the batteries/time block)
    const char *label = left_label ? left_label : state->led_mode_name;
    if (label && *label) {
        int max_chars = (bx - margin) / 4; // 3px glyph + 1px spacing
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

    // Underline HUD band
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
