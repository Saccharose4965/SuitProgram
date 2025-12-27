#include "snake.h"
#include "oled.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define SNAKE_CELL_PX 4
#define SNAKE_GRID_W  32
#define SNAKE_GRID_H  14
#define SNAKE_STEP_SEC (1.0f/6.0f)
#define SCORE_BAND_PX 8

typedef struct { int x, y; } snake_cell_t;
typedef struct {
    snake_cell_t body[120];
    int len;
    int dir; // 0=right,1=down,2=left,3=up
    int food_x;
    int food_y;
    bool alive;
    float acc;
} snake_state_t;

static snake_state_t s_snake = {0};
static snake_request_switch_fn s_switch_cb = NULL;
static void *s_switch_user = NULL;

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

void snake_set_request_switch(snake_request_switch_fn fn, void *user_data)
{
    s_switch_cb = fn;
    s_switch_user = user_data;
}

static void snake_spawn_food(void){
    int tries = 0;
    while (tries++ < 64){
        int fx = rand() % SNAKE_GRID_W;
        int fy = rand() % SNAKE_GRID_H;
        bool clash = false;
        for (int i = 0; i < s_snake.len; ++i){
            if (s_snake.body[i].x == fx && s_snake.body[i].y == fy){ clash = true; break; }
        }
        if (!clash){ s_snake.food_x = fx; s_snake.food_y = fy; return; }
    }
    s_snake.food_x = 0; s_snake.food_y = 0;
}

static void snake_reset(void){
    s_snake.len = 4;
    int cx = SNAKE_GRID_W / 2;
    int cy = SNAKE_GRID_H / 2;
    for (int i = 0; i < s_snake.len; ++i){
        s_snake.body[i].x = cx - i;
        s_snake.body[i].y = cy;
    }
    s_snake.dir = 0;
    s_snake.alive = true;
    s_snake.acc = 0.0f;
    snake_spawn_food();
}

void snake_app_init(void){
    snake_reset();
}

void snake_app_handle_input(const input_event_t *ev){
    if (!ev) return;
    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        if (s_switch_cb) {
            s_switch_cb("menu", s_switch_user);
        }
        return;
    }
    if (ev->type == INPUT_EVENT_PRESS){
        if (!s_snake.alive){
            snake_reset();
            return;
        }
        // Desired order: A=left, B=up, C=right, D=down
        if (ev->button == INPUT_BTN_A && s_snake.dir != 0) s_snake.dir = 2; // left
        else if (ev->button == INPUT_BTN_C && s_snake.dir != 2) s_snake.dir = 0; // right
        else if (ev->button == INPUT_BTN_B && s_snake.dir != 1) s_snake.dir = 3; // up
        else if (ev->button == INPUT_BTN_D && s_snake.dir != 3) s_snake.dir = 1; // down
    }
}

static void snake_step(void){
    if (!s_snake.alive) return;
    snake_cell_t head = s_snake.body[0];
    switch (s_snake.dir){
        case 0: head.x++; break;
        case 1: head.y++; break;
        case 2: head.x--; break;
        case 3: head.y--; break;
        default: break;
    }
    if (head.x < 0 || head.x >= SNAKE_GRID_W || head.y < 0 || head.y >= SNAKE_GRID_H){
        s_snake.alive = false;
        return;
    }
    for (int i = 0; i < s_snake.len; ++i){
        if (s_snake.body[i].x == head.x && s_snake.body[i].y == head.y){
            s_snake.alive = false;
            return;
        }
    }
    for (int i = s_snake.len - 1; i > 0; --i){
        s_snake.body[i] = s_snake.body[i-1];
    }
    s_snake.body[0] = head;
    if (head.x == s_snake.food_x && head.y == s_snake.food_y){
        if (s_snake.len < (int)(sizeof(s_snake.body)/sizeof(s_snake.body[0]))){
            s_snake.body[s_snake.len] = s_snake.body[s_snake.len-1];
            s_snake.len++;
        }
        snake_spawn_food();
    }
}

void snake_app_tick(float dt_sec){
    s_snake.acc += dt_sec;
    while (s_snake.acc >= SNAKE_STEP_SEC){
        snake_step();
        s_snake.acc -= SNAKE_STEP_SEC;
    }
}

void snake_app_draw(uint8_t *fb, int x, int y, int w, int h){
    (void)x; (void)y; (void)w; (void)h;
    const int cell = SNAKE_CELL_PX;
    const int origin_x = 0;
    const int origin_y = SCORE_BAND_PX;
    fb_rect_fill_local(fb,
                 origin_x + s_snake.food_x * cell,
                 origin_y + s_snake.food_y * cell,
                 cell, cell);
    for (int i = 0; i < s_snake.len; ++i){
        int cx = origin_x + s_snake.body[i].x * cell;
        int cy = origin_y + s_snake.body[i].y * cell;
        fb_rect_fill_local(fb, cx, cy, cell, cell);
    }
    char line[24];
    snprintf(line, sizeof(line), "LEN:%d", s_snake.len);
    oled_draw_text3x5(fb, PANEL_W - 36, 0, line);
    if (!s_snake.alive){
        oled_draw_text3x5(fb, PANEL_W/2 - 10, origin_y + 8, "DEAD");
        oled_draw_text3x5(fb, PANEL_W/2 - 20, origin_y + 16, "Press any");
    }
}
