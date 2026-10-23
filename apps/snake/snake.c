#include "app_shell.h"
#include "snake_game.h"
#include "ui_draw.h"
#include "oled.h"
#include <stdio.h>

#define SNAKE_CELL_PX 4
#define SCORE_BAND_PX 8

static inline void fb_pset_local(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_fill_local(uint8_t *fb, int x0, int y0, int w, int h)
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
            fb_pset_local(fb, x, y);
        }
    }
}


const shell_legend_t SNAKE_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
};
void snake_app_init(shell_app_context_t *ctx)
{
    if (ctx) snake_game_init();
}
void snake_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;
    int direction = -1;
    switch (ev->button) {
    case INPUT_BTN_A: direction = 2; break;
    case INPUT_BTN_B: direction = 3; break;
    case INPUT_BTN_C: direction = 0; break;
    case INPUT_BTN_D: direction = 1; break;
    default: break;
    }
    snake_game_press(direction);
}
void snake_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    snake_game_tick(dt_sec);
}

void snake_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h){
    (void)ctx;
    const snake_state_t *state = snake_game_state();
    (void)x; (void)y; (void)w; (void)h;
    const int cell = SNAKE_CELL_PX;
    const int origin_x = 0;
    const int origin_y = SCORE_BAND_PX;
    fb_rect_fill_local(fb,
                 origin_x + state->food_x * cell,
                 origin_y + state->food_y * cell,
                 cell, cell);
    for (int i = 0; i < state->len; ++i){
        int cx = origin_x + state->body[i].x * cell;
        int cy = origin_y + state->body[i].y * cell;
        fb_rect_fill_local(fb, cx, cy, cell, cell);
    }
    char line[24];
    snprintf(line, sizeof(line), "L%d/B%d", state->len, snake_game_best());
    oled_draw_text3x5(fb, PANEL_W - 36, 0, line);
    if (!state->alive){
        oled_draw_text3x5(fb, PANEL_W/2 - 10, origin_y + 8, "DEAD");
        oled_draw_text3x5(fb, PANEL_W/2 - 20, origin_y + 16, "Press any");
    }
}
