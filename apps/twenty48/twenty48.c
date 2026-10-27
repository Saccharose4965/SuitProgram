#include "app_shell.h"
#include "t48_sprites.h"
#include "twenty48_game.h"
#include "ui_draw.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum {
    T48_STRIDE = 14,
    T48_TILE = T48_TILE_SIZE,
};

typedef enum {
    T48_READY,
    T48_PLAYING,
    T48_RESULTS,
} t48_screen_t;

static const int kBoardX0 = 35;
static const int kBoardY0 = 0;
static const float kAnimationRate = 0.2f / 0.012f;
static const float kRetryDelaySeconds = 2.0f;

static uint8_t s_background[PANEL_W * PANEL_H / 8];
static t48_screen_t s_screen;
static float s_animation_phase;
static float s_result_time;
static bool s_animating;

shell_legend_t T2048_LEGEND;

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int index = y * PANEL_W + x;
    fb[index >> 3] |= (uint8_t)(1u << (7 - (index & 7)));
}

static inline void fb_pclr(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int index = y * PANEL_W + x;
    fb[index >> 3] &= (uint8_t)~(1u << (7 - (index & 7)));
}

static void fb_clear_rect(uint8_t *fb, int x, int y, int w, int h)
{
    for (int py = y; py < y + h; ++py)
        for (int px = x; px < x + w; ++px) fb_pclr(fb, px, py);
}

static void draw_rect(uint8_t *fb, int x, int y, int w, int h)
{
    int x1 = x + w - 1, y1 = y + h - 1;
    for (int px = x; px <= x1; ++px) {
        fb_pset(fb, px, y);
        fb_pset(fb, px, y1);
    }
    for (int py = y; py <= y1; ++py) {
        fb_pset(fb, x, py);
        fb_pset(fb, x1, py);
    }
}

static void blit_tile(uint8_t *fb, const uint16_t *rows, int size, int x, int y)
{
    for (int dy = 0; dy < size; ++dy) {
        uint16_t bits = rows[dy];
        for (int dx = 0; dx < size; ++dx)
            if (bits & (1u << (size - 1 - dx))) fb_pset(fb, x + dx, y + dy);
    }
}

static void build_background(void)
{
    memset(s_background, 0, sizeof(s_background));
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            draw_rect(s_background,
                      kBoardX0 + col * T48_STRIDE,
                      kBoardY0 + row * T48_STRIDE,
                      T48_TILE, T48_TILE);
        }
    }
}

static void show_direction_legend(void)
{
    T2048_LEGEND = (shell_legend_t){
        .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
    };
}

static void hide_legend(void)
{
    T2048_LEGEND = (shell_legend_t){ .slots = { SHELL_ICON_NONE } };
}

static void show_retry_legend(void)
{
    T2048_LEGEND = (shell_legend_t){
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
    t48_game_init();
    s_screen = T48_PLAYING;
    s_animation_phase = 0.0f;
    s_result_time = 0.0f;
    s_animating = false;
    show_direction_legend();
}

void t2048_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    t48_game_init();
    build_background();
    s_screen = T48_READY;
    s_animation_phase = 0.0f;
    s_result_time = 0.0f;
    s_animating = false;
    show_direction_legend();
}

void t2048_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    t48_game_deinit();
}

void t2048_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    int direction;
    if (s_screen == T48_READY) {
        if (input_direction(ev->button, &direction)) s_screen = T48_PLAYING;
        return;
    }
    if (s_screen == T48_RESULTS) {
        if (s_result_time >= kRetryDelaySeconds && ev->button == INPUT_BTN_D) start_game();
        return;
    }
    if (s_animating || !input_direction(ev->button, &direction)) return;

    if (t48_game_move(direction)) {
        s_animation_phase = 0.0f;
        s_animating = true;
    }
}

void t2048_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    if (dt_sec < 0.0f) dt_sec = 0.0f;
    if (dt_sec > 0.25f) dt_sec = 0.25f;

    if (s_screen == T48_RESULTS) {
        float previous = s_result_time;
        s_result_time += dt_sec;
        if (previous < kRetryDelaySeconds && s_result_time >= kRetryDelaySeconds) {
            show_retry_legend();
        }
        return;
    }
    if (!s_animating) return;

    s_animation_phase += kAnimationRate * dt_sec;
    if (s_animation_phase >= (float)M_PI) {
        t48_game_commit();
        s_animation_phase = 0.0f;
        s_animating = false;
        if (t48_game_is_over()) {
            t48_game_deinit();
            s_screen = T48_RESULTS;
            s_result_time = 0.0f;
            hide_legend();
        }
    }
}

void t2048_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)x; (void)y; (void)w; (void)h;
    if (!fb) return;
    memcpy(fb, s_background, sizeof(s_background));
    const t48_state_t *state = t48_game_state();
    float offset_scale = (cosf(s_animation_phase) - 1.0f) * 0.5f;

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            int exponent = state->grid_old[row][col];
            int sprite = exponent - 1;
            if (sprite < 0 || sprite >= T48_TILE_COUNT) continue;
            float dx = 0.0f, dy = 0.0f;
            if ((state->direction & 1) == 0) {
                dx = offset_scale * (state->direction - 1) * state->motion[row][col] * T48_STRIDE;
            } else {
                dy = offset_scale * (state->direction - 2) * state->motion[row][col] * T48_STRIDE;
            }
            blit_tile(fb, t48_tiles_15[sprite], T48_TILE,
                      kBoardX0 + col * T48_STRIDE + (int)lroundf(dx),
                      kBoardY0 + row * T48_STRIDE + (int)lroundf(dy));
        }
    }

    fb_clear_rect(fb, 0, 0, kBoardX0 - 2, 26);
    char text[16];
    oled_draw_text3x5(fb, 2, 2, "SCORE:");
    snprintf(text, sizeof(text), " %u", (unsigned)state->score);
    oled_draw_text3x5(fb, 2, 8, text);
    oled_draw_text3x5(fb, 2, 14, "BEST:");
    snprintf(text, sizeof(text), " %u", (unsigned)state->best);
    oled_draw_text3x5(fb, 2, 20, text);

    if (s_screen != T48_PLAYING) {
        shell_overlay_view_t overlay = {
            .title = s_screen == T48_READY ? "2048" : "GAME OVER",
            .line1 = s_screen == T48_READY ? "PRESS A B C D" : text,
            .line2 = "",
            .prompt = "",
        };
        if (s_screen == T48_RESULTS) snprintf(text, sizeof(text), "SCORE %u", (unsigned)state->score);
        shell_ui_draw_overlay(fb, 18, 8, 92, 40, &overlay);
    }
}
