#include "app_shell.h"
#include "snake_game.h"
#include "ui_draw.h"

#include <stdio.h>
#include <string.h>

enum {
    SNAKE_CELL_SIZE = 4,
    PLAYFIELD_X = 4,
    PLAYFIELD_Y = 8,
    PLAYFIELD_W = 120,
    PLAYFIELD_H = 48,
    GRID_X = PLAYFIELD_X + 2,
    GRID_Y = PLAYFIELD_Y + 2,
};

typedef enum {
    SNAKE_READY,
    SNAKE_PLAYING,
    SNAKE_RESULTS,
} snake_screen_t;

static const float kRetryDelaySeconds = 2.0f;
static snake_screen_t s_screen;
static float s_result_time;

shell_legend_t SNAKE_LEGEND;

static inline void pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int index = y * PANEL_W + x;
    fb[index >> 3] |= (uint8_t)(1u << (7 - (index & 7)));
}

static void rect_fill(uint8_t *fb, int x, int y, int w, int h)
{
    for (int py = y; py < y + h; ++py)
        for (int px = x; px < x + w; ++px) pset(fb, px, py);
}

static void draw_border(uint8_t *fb)
{
    rect_fill(fb, PLAYFIELD_X, PLAYFIELD_Y, PLAYFIELD_W, 2);
    rect_fill(fb, PLAYFIELD_X, PLAYFIELD_Y + PLAYFIELD_H - 2, PLAYFIELD_W, 2);
    rect_fill(fb, PLAYFIELD_X, PLAYFIELD_Y, 2, PLAYFIELD_H);
    rect_fill(fb, PLAYFIELD_X + PLAYFIELD_W - 2, PLAYFIELD_Y, 2, PLAYFIELD_H);
}

static void draw_apple(uint8_t *fb, int x, int y)
{
    static const uint8_t rows[4] = {
        0b0100,
        0b1110,
        0b1111,
        0b0110,
    };
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            if (rows[row] & (1u << (3 - col))) pset(fb, x + col, y + row);
}

static void show_direction_legend(void)
{
    SNAKE_LEGEND = (shell_legend_t){
        .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
    };
}

static void hide_legend(void)
{
    SNAKE_LEGEND = (shell_legend_t){ .slots = { SHELL_ICON_NONE } };
}

static void show_retry_legend(void)
{
    SNAKE_LEGEND = (shell_legend_t){
        .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_REFRESH },
    };
}

static bool input_direction(input_button_t button, int *direction)
{
    if (!direction) return false;
    switch (button) {
        case INPUT_BTN_A: *direction = 2; return true;
        case INPUT_BTN_B: *direction = 3; return true;
        case INPUT_BTN_C: *direction = 0; return true;
        case INPUT_BTN_D: *direction = 1; return true;
        default: return false;
    }
}

static void start_game(void)
{
    snake_game_reset();
    s_screen = SNAKE_PLAYING;
    s_result_time = 0.0f;
    show_direction_legend();
}

void snake_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    snake_game_init();
    s_screen = SNAKE_READY;
    s_result_time = 0.0f;
    show_direction_legend();
}

void snake_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;
    int direction;
    if (s_screen == SNAKE_READY) {
        if (input_direction(ev->button, &direction)) s_screen = SNAKE_PLAYING;
        return;
    }
    if (s_screen == SNAKE_RESULTS) {
        if (s_result_time >= kRetryDelaySeconds && ev->button == INPUT_BTN_D) start_game();
        return;
    }
    if (input_direction(ev->button, &direction)) snake_game_queue_direction(direction);
}

void snake_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    if (dt_sec < 0.0f) dt_sec = 0.0f;
    if (dt_sec > 0.25f) dt_sec = 0.25f;

    if (s_screen == SNAKE_RESULTS) {
        float previous = s_result_time;
        s_result_time += dt_sec;
        if (previous < kRetryDelaySeconds && s_result_time >= kRetryDelaySeconds) {
            show_retry_legend();
        }
        return;
    }
    if (s_screen != SNAKE_PLAYING) return;

    snake_game_tick(dt_sec);
    if (!snake_game_state()->alive) {
        s_screen = SNAKE_RESULTS;
        s_result_time = 0.0f;
        hide_legend();
    }
}

void snake_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)x; (void)y; (void)w; (void)h;
    if (!fb) return;
    const snake_state_t *state = snake_game_state();
    draw_border(fb);

    draw_apple(fb,
               GRID_X + state->food_x * SNAKE_CELL_SIZE,
               GRID_Y + state->food_y * SNAKE_CELL_SIZE);
    for (int i = 0; i < state->len; ++i) {
        rect_fill(fb,
                  GRID_X + state->body[i].x * SNAKE_CELL_SIZE,
                  GRID_Y + state->body[i].y * SNAKE_CELL_SIZE,
                  SNAKE_CELL_SIZE, SNAKE_CELL_SIZE);
    }

    char score[20], best[20];
    int apples = state->len >= 4 ? state->len - 4 : 0;
    int best_apples = snake_game_best() >= 4 ? snake_game_best() - 4 : 0;
    snprintf(score, sizeof(score), "SCORE %d", apples);
    snprintf(best, sizeof(best), "BEST %d", best_apples);
    oled_draw_text3x5(fb, 2, 1, score);
    oled_draw_text3x5(fb, PANEL_W - (int)strlen(best) * 4 - 2, 1, best);

    if (s_screen != SNAKE_PLAYING) {
        shell_overlay_view_t overlay = {
            .title = s_screen == SNAKE_READY ? "SNAKE" : "GAME OVER",
            .line1 = s_screen == SNAKE_READY ? "PRESS A B C D" : score,
            .line2 = s_screen == SNAKE_READY ? "" : best,
            .prompt = "",
        };
        shell_ui_draw_overlay(fb, 18, 12, 92, 36, &overlay);
    }
}
