#include "app_shell.h"
#include "tetris_game.h"
#include "ui_draw.h"

#include <stdio.h>
#include <string.h>

#define CELL_SIZE 4

typedef enum {
    TETRIS_READY,
    TETRIS_PLAYING,
    TETRIS_RESULTS,
} tetris_screen_t;

static const float kDropSeconds = 0.450f;
static const float kActionCooldownSeconds = 0.140f;
static const float kRetryDelaySeconds = 2.0f;

static tetris_screen_t s_screen;
static float s_drop_time;
static float s_action_cooldown;
static float s_result_time;

shell_legend_t TETRIS_LEGEND;

static inline void pset(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int index = y * PANEL_W + x;
    fb[index >> 3] |= (uint8_t)(1u << (7 - (index & 7)));
}

static void show_play_legend(void)
{
    TETRIS_LEGEND = (shell_legend_t){
        .slots = { SHELL_ICON_LEFT, SHELL_ICON_REFRESH, SHELL_ICON_REFRESH, SHELL_ICON_RIGHT },
    };
}

static void hide_legend(void)
{
    TETRIS_LEGEND = (shell_legend_t){ .slots = { SHELL_ICON_NONE } };
}

static void show_retry_legend(void)
{
    TETRIS_LEGEND = (shell_legend_t){
        .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_REFRESH },
    };
}

static bool is_game_button(input_button_t button)
{
    return button == INPUT_BTN_A || button == INPUT_BTN_B ||
           button == INPUT_BTN_C || button == INPUT_BTN_D;
}

static void start_game(void)
{
    tetris_game_init();
    s_screen = TETRIS_PLAYING;
    s_drop_time = 0.0f;
    s_action_cooldown = 0.0f;
    s_result_time = 0.0f;
    show_play_legend();
}

static void enter_results(void)
{
    tetris_game_finish();
    s_screen = TETRIS_RESULTS;
    s_result_time = 0.0f;
    hide_legend();
}

void tetris_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    tetris_game_init();
    s_screen = TETRIS_READY;
    s_drop_time = 0.0f;
    s_action_cooldown = 0.0f;
    s_result_time = 0.0f;
    show_play_legend();
}

void tetris_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    tetris_game_finish();
}

void tetris_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (s_screen == TETRIS_READY) {
        if (is_game_button(ev->button)) s_screen = TETRIS_PLAYING;
        return;
    }
    if (s_screen == TETRIS_RESULTS) {
        if (s_result_time >= kRetryDelaySeconds && ev->button == INPUT_BTN_D) start_game();
        return;
    }
    if (s_action_cooldown > 0.0f) return;

    switch (ev->button) {
        case INPUT_BTN_A: tetris_game_move(-1); break;
        case INPUT_BTN_B: tetris_game_rotate(-1); break;
        case INPUT_BTN_C: tetris_game_rotate(1); break;
        case INPUT_BTN_D: tetris_game_move(1); break;
        default: return;
    }
    s_action_cooldown = kActionCooldownSeconds;
}

void tetris_app_tick(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    if (dt_sec < 0.0f) dt_sec = 0.0f;
    if (dt_sec > 0.25f) dt_sec = 0.25f;

    if (s_screen == TETRIS_RESULTS) {
        float previous = s_result_time;
        s_result_time += dt_sec;
        if (previous < kRetryDelaySeconds && s_result_time >= kRetryDelaySeconds) {
            show_retry_legend();
        }
        return;
    }
    if (s_screen != TETRIS_PLAYING) return;

    if (s_action_cooldown > 0.0f) {
        s_action_cooldown -= dt_sec;
        if (s_action_cooldown < 0.0f) s_action_cooldown = 0.0f;
    }

    s_drop_time += dt_sec;
    if (s_drop_time >= kDropSeconds) {
        s_drop_time -= kDropSeconds;
        tetris_game_drop();
        if (tetris_game_state()->game_over) enter_results();
    }
}

static void draw_piece(uint8_t *fb, piece_t piece, int origin_x, int origin_y, int fill_size)
{
    const cell_t *cells = tetris_piece_cells(piece);
    for (int i = 0; i < 4; ++i) {
        int col = piece.x + cells[i].x;
        int row = piece.y + cells[i].y;
        if (col < 0 || col >= BOARD_W || row < 0 || row >= BOARD_H) continue;
        for (int py = 0; py < fill_size; ++py)
            for (int px = 0; px < fill_size; ++px)
                pset(fb, origin_x + col * CELL_SIZE + 1 + px,
                     origin_y + row * CELL_SIZE + 1 + py);
    }
}

void tetris_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)x; (void)y; (void)w; (void)h;
    if (!fb) return;
    const tetris_state_t *state = tetris_game_state();
    int board_x = (PANEL_W - BOARD_W * CELL_SIZE) / 2;
    int board_y = 8;

    for (int row = 0; row <= BOARD_H; ++row)
        for (int col = 0; col <= BOARD_W; ++col)
            pset(fb, board_x + col * CELL_SIZE, board_y + row * CELL_SIZE);

    for (int row = 0; row < BOARD_H; ++row) {
        for (int col = 0; col < BOARD_W; ++col) {
            if (!state->board[row][col]) continue;
            for (int py = 0; py < CELL_SIZE - 1; ++py)
                for (int px = 0; px < CELL_SIZE - 1; ++px)
                    pset(fb, board_x + col * CELL_SIZE + 1 + px,
                         board_y + row * CELL_SIZE + 1 + py);
        }
    }
    if (!state->game_over) draw_piece(fb, state->current, board_x, board_y, CELL_SIZE - 2);

    piece_t preview = { .x = 0, .y = 0, .shape = state->next_shape, .rot = 0 };
    const cell_t *preview_cells = tetris_piece_cells(preview);
    int preview_x = PANEL_W - 4 * CELL_SIZE - 2;
    int preview_y = 10;
    for (int i = 0; i < 4; ++i) {
        for (int py = 0; py < CELL_SIZE - 2; ++py)
            for (int px = 0; px < CELL_SIZE - 2; ++px)
                pset(fb, preview_x + preview_cells[i].x * CELL_SIZE + px,
                     preview_y + preview_cells[i].y * CELL_SIZE + py);
    }

    char text[16];
    snprintf(text, sizeof(text), "%lu", (unsigned long)state->score);
    oled_draw_text3x5(fb, 2, 1, text);
    snprintf(text, sizeof(text), "%lu", (unsigned long)state->best);
    oled_draw_text3x5(fb, PANEL_W - (int)strlen(text) * 4 - 2, 1, text);

    if (s_screen != TETRIS_PLAYING) {
        char score[20];
        shell_overlay_view_t overlay = {
            .title = s_screen == TETRIS_READY ? "TETRIS" : "GAME OVER",
            .line1 = s_screen == TETRIS_READY ? "PRESS A B C D" : score,
            .line2 = "",
            .prompt = "",
        };
        if (s_screen == TETRIS_RESULTS) {
            snprintf(score, sizeof(score), "SCORE %lu", (unsigned long)state->score);
        }
        shell_ui_draw_overlay(fb, 18, 8, 92, 40, &overlay);
    }
}
