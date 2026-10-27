#include "app_shell.h"
#include "flappy_game.h"
#include "shell_ui.h"

#include <math.h>
#include <stdio.h>

typedef enum { FLAPPY_READY, FLAPPY_PLAYING, FLAPPY_RESULTS } flappy_screen_t;

static flappy_screen_t s_screen;
static float s_step_time;
static bool s_flap_pending;
static bool s_run_open;

static inline void pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int index = y * PANEL_W + x;
    fb[index >> 3] |= (uint8_t)(1u << (7 - (index & 7)));
}

static void vline(uint8_t *fb, int x, int y0, int y1)
{
    if (y0 > y1) { int swap = y0; y0 = y1; y1 = swap; }
    for (int y = y0; y <= y1; ++y) pset(fb, x, y);
}

static void rect_fill(uint8_t *fb, int x, int y, int w, int h)
{
    for (int py = y; py < y + h; ++py)
        for (int px = x; px < x + w; ++px) pset(fb, px, py);
}

static const uint8_t DIGIT_3X5[10][5] = {
    {0b111,0b101,0b101,0b101,0b111}, {0b010,0b110,0b010,0b010,0b111},
    {0b111,0b001,0b111,0b100,0b111}, {0b111,0b001,0b111,0b001,0b111},
    {0b101,0b101,0b111,0b001,0b001}, {0b111,0b100,0b111,0b001,0b111},
    {0b111,0b100,0b111,0b101,0b111}, {0b111,0b001,0b010,0b010,0b010},
    {0b111,0b101,0b111,0b101,0b111}, {0b111,0b101,0b111,0b001,0b111},
};

static void draw_digit(uint8_t *fb, int x, int y, int digit)
{
    if (digit < 0 || digit > 9) return;
    for (int row = 0; row < 5; ++row)
        for (int col = 0; col < 3; ++col)
            if (DIGIT_3X5[digit][row] & (1u << (2 - col))) pset(fb, x + col, y + row);
}

static void draw_score(uint8_t *fb, int x, int y, int score)
{
    int digits[5], count = 0;
    if (score < 0) score = 0;
    do { digits[count++] = score % 10; score /= 10; } while (score && count < 5);
    for (int i = 0; i < count; ++i) draw_digit(fb, x + (count - 1 - i) * 4, y, digits[i]);
}

static void draw_bird(uint8_t *fb, int x, int y, bool flip)
{
    int x0 = x - BIRD_W / 2, y0 = y - BIRD_H / 2;
    for (int row = 0; row < BIRD_H; ++row) {
        int shape_row = flip ? BIRD_H - 1 - row : row;
        for (int col = 0; col < BIRD_W; ++col)
            if (flappy_bird_rows[shape_row] & (1u << (BIRD_W - 1 - col)))
                pset(fb, x0 + col, y0 + row);
    }
}

static void draw_pipe(uint8_t *fb, int x, int gap_y)
{
    int gap_top = gap_y - PIPE_GAP / 2, gap_bottom = gap_y + PIPE_GAP / 2;
    for (int px = x; px < x + PIPE_W; ++px) {
        vline(fb, px, 0, gap_top - 1);
        vline(fb, px, gap_bottom + 1, PANEL_H - 1);
    }
    rect_fill(fb, x - 1, gap_top - 2, PIPE_W + 2, 2);
    rect_fill(fb, x - 1, gap_bottom + 1, PIPE_W + 2, 2);
}

static void start_run(void)
{
    flappy_game_init();
    s_screen = FLAPPY_PLAYING;
    s_step_time = 0.0f;
    s_flap_pending = false;
    s_run_open = true;
}

void flappy_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    flappy_game_init();
    s_screen = FLAPPY_READY;
    s_step_time = 0.0f;
    s_flap_pending = false;
    s_run_open = false;
}

void flappy_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    if (s_run_open) flappy_game_finish();
    s_run_open = false;
    s_flap_pending = false;
}

void flappy_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;
    if (s_screen == FLAPPY_PLAYING) s_flap_pending = true;
    else if (ev->button == INPUT_BTN_D) start_run();
}

void flappy_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    if (s_screen != FLAPPY_PLAYING) return;
    const float fixed_step = 1.0f / FPS;
    if (dt_sec < 0.0f) dt_sec = 0.0f;
    if (dt_sec > 0.25f) dt_sec = 0.25f;
    s_step_time += dt_sec;

    int steps = 0;
    while (s_step_time >= fixed_step && steps++ < 8) {
        bool flap = s_flap_pending;
        s_flap_pending = false;
        flappy_game_step(flap);
        s_step_time -= fixed_step;
        if (!flappy_game_state()->running) {
            flappy_game_finish();
            s_run_open = false;
            s_screen = FLAPPY_RESULTS;
            s_step_time = 0.0f;
            break;
        }
    }
}

void flappy_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)x; (void)y; (void)w; (void)h;
    if (!fb) return;
    const flappy_state_t *state = flappy_game_state();
    for (int i = 0; i < 3; ++i) draw_pipe(fb, state->pipe_x[i], state->pipe_gap_y[i]);
    for (int px = 0; px < PANEL_W; ++px) pset(fb, px, PANEL_H - 1);

    bool visible = state->state == STATE_PLAYING || (state->death_frames / 6) % 2 == 0;
    if (visible) draw_bird(fb, (int)roundf(state->bird_x), (int)roundf(state->bird_y), state->upside_down);
    draw_score(fb, 2, 2, state->score);
    draw_score(fb, PANEL_W - 14, 2, state->best);

    if (s_screen != FLAPPY_PLAYING) {
        char line1[24], line2[24];
        shell_overlay_view_t overlay = {
            .title = "FLAPPY", .line1 = line1, .line2 = line2,
            .prompt = s_screen == FLAPPY_READY ? "D START" : "D RETRY",
        };
        if (s_screen == FLAPPY_READY) {
            snprintf(line1, sizeof(line1), "BEST %d", state->best);
            line2[0] = '\0';
        } else {
            snprintf(line1, sizeof(line1), "SCORE %d", state->score);
            snprintf(line2, sizeof(line2), "BEST %d", state->best);
        }
        shell_ui_draw_overlay(fb, 24, 10, 80, 44, &overlay);
    }
}
