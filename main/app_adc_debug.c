#include "app_shell.h"

#include <stdio.h>
#include <string.h>

#include "input.h"

#define ADC_DEBUG_HISTORY_CAP PANEL_W
#define ADC_DEBUG_MV_MIN 0
#define ADC_DEBUG_MV_MAX 3500

typedef struct {
    int mv_hist[ADC_DEBUG_HISTORY_CAP];
    int head;
    int len;
    int last_mv;
    input_button_t last_btn;
    bool sample_ok;
} adc_debug_state_t;

static adc_debug_state_t s_adc_dbg = {
    .head = 0,
    .len = 0,
    .last_mv = 0,
    .last_btn = INPUT_BTN_NONE,
    .sample_ok = false,
};

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_line(uint8_t *fb, int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0;
    if (dx < 0) dx = -dx;
    int sx = (x0 < x1) ? 1 : -1;
    int dy = y1 - y0;
    if (dy < 0) dy = -dy;
    dy = -dy;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        fb_pset(fb, x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err << 1;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void fb_rect_outline(uint8_t *fb, int x, int y, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;
    int x1 = x + w - 1;
    int y1 = y + h - 1;
    for (int xx = x; xx <= x1; ++xx) {
        fb_pset(fb, xx, y);
        fb_pset(fb, xx, y1);
    }
    for (int yy = y; yy <= y1; ++yy) {
        fb_pset(fb, x, yy);
        fb_pset(fb, x1, yy);
    }
}

static const char *button_name(input_button_t btn)
{
    switch (btn) {
        case INPUT_BTN_A: return "A";
        case INPUT_BTN_B: return "B";
        case INPUT_BTN_C: return "C";
        case INPUT_BTN_D: return "D";
        case INPUT_BTN_AB_COMBO: return "AB";
        case INPUT_BTN_BC_COMBO: return "BC";
        case INPUT_BTN_NONE:
        default:
            return "-";
    }
}

static int mv_to_plot_y(int mv, int y_top, int plot_h)
{
    if (plot_h <= 1) return y_top;
    if (mv < ADC_DEBUG_MV_MIN) mv = ADC_DEBUG_MV_MIN;
    if (mv > ADC_DEBUG_MV_MAX) mv = ADC_DEBUG_MV_MAX;
    int max_y = y_top + plot_h - 1;
    int num = mv * (plot_h - 1) + (ADC_DEBUG_MV_MAX / 2);
    int y = max_y - (num / ADC_DEBUG_MV_MAX);
    if (y < y_top) y = y_top;
    if (y > max_y) y = max_y;
    return y;
}

static void adc_debug_clear_history(void)
{
    memset(s_adc_dbg.mv_hist, 0, sizeof(s_adc_dbg.mv_hist));
    s_adc_dbg.head = 0;
    s_adc_dbg.len = 0;
}

static void adc_debug_push_mv(int mv)
{
    s_adc_dbg.mv_hist[s_adc_dbg.head] = mv;
    s_adc_dbg.head = (s_adc_dbg.head + 1) % ADC_DEBUG_HISTORY_CAP;
    if (s_adc_dbg.len < ADC_DEBUG_HISTORY_CAP) {
        s_adc_dbg.len++;
    }
}

static void adc_debug_sample_once(void)
{
    input_button_t btn = INPUT_BTN_NONE;
    int mv = 0;
    if (!input_sample(&btn, &mv)) {
        s_adc_dbg.sample_ok = false;
        return;
    }
    s_adc_dbg.sample_ok = true;
    s_adc_dbg.last_mv = mv;
    s_adc_dbg.last_btn = btn;
    adc_debug_push_mv(mv);
}

void adc_debug_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    adc_debug_clear_history();
    s_adc_dbg.last_mv = 0;
    s_adc_dbg.last_btn = INPUT_BTN_NONE;
    s_adc_dbg.sample_ok = false;
    adc_debug_sample_once();
}

void adc_debug_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    (void)ev;
}

void adc_debug_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    (void)dt_sec;
    adc_debug_sample_once();
}

void adc_debug_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb || w <= 4 || h <= 10) return;

    char line[32];
    oled_draw_text3x5(fb, x + 2, y + 2, "ADC DEBUG");

    if (s_adc_dbg.sample_ok) {
        snprintf(line, sizeof(line), "mv:%4d btn:%s",
                 s_adc_dbg.last_mv,
                 button_name(s_adc_dbg.last_btn));
    } else {
        snprintf(line, sizeof(line), "mv:---- btn:?");
    }
    oled_draw_text3x5(fb, x + 2, y + 10, line);

    int hist_min = 0;
    int hist_max = 0;
    if (s_adc_dbg.len > 0) {
        int newest = (s_adc_dbg.head - 1 + ADC_DEBUG_HISTORY_CAP) % ADC_DEBUG_HISTORY_CAP;
        hist_min = s_adc_dbg.mv_hist[newest];
        hist_max = hist_min;
        for (int i = 0; i < s_adc_dbg.len; ++i) {
            int idx = (s_adc_dbg.head - 1 - i + ADC_DEBUG_HISTORY_CAP) % ADC_DEBUG_HISTORY_CAP;
            int v = s_adc_dbg.mv_hist[idx];
            if (v < hist_min) hist_min = v;
            if (v > hist_max) hist_max = v;
        }
    }
    snprintf(line, sizeof(line), "min:%4d max:%4d", hist_min, hist_max);
    oled_draw_text3x5(fb, x + 2, y + 18, line);

    int graph_x = x + 1;
    int graph_y = y + 26;
    int graph_w = w - 2;
    int graph_h = h - 27;
    if (graph_w < 8 || graph_h < 8) return;

    fb_rect_outline(fb, graph_x, graph_y, graph_w, graph_h);

    int plot_x = graph_x + 1;
    int plot_y = graph_y + 1;
    int plot_w = graph_w - 2;
    int plot_h = graph_h - 2;
    if (plot_w < 2 || plot_h < 2) return;

    int count = s_adc_dbg.len;
    if (count > plot_w) count = plot_w;
    if (count <= 0) return;

    int start = (s_adc_dbg.head - count + ADC_DEBUG_HISTORY_CAP) % ADC_DEBUG_HISTORY_CAP;
    int x_off = plot_w - count; // right-align newest samples

    int prev_x = -1;
    int prev_y = -1;
    for (int i = 0; i < count; ++i) {
        int idx = (start + i) % ADC_DEBUG_HISTORY_CAP;
        int mv = s_adc_dbg.mv_hist[idx];
        int xx = plot_x + x_off + i;
        int yy = mv_to_plot_y(mv, plot_y, plot_h);
        if (prev_x >= 0) {
            fb_line(fb, prev_x, prev_y, xx, yy);
        } else {
            fb_pset(fb, xx, yy);
        }
        prev_x = xx;
        prev_y = yy;
    }
}
