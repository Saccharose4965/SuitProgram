#include "app_shell.h"
#include "tetris_game.h"
#include "ui_draw.h"
#include "oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#define CELL_SZ 4
static uint8_t g_fb[PANEL_W * PANEL_H / 8];
static volatile bool s_tetris_stop;
static inline void fb_clear(void){ memset(g_fb, 0, sizeof(g_fb)); }
static inline void pset(int x,int y){
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y*PANEL_W + x;
    g_fb[idx>>3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void draw_board(void){
    const tetris_state_t *state=tetris_game_state();
    piece_t falling=state->current;
    fb_clear();
    int board_px_w = BOARD_W * CELL_SZ;
    int ox = (PANEL_W - board_px_w) / 2;
    if (ox < 0) ox = 0;
    int oy = 8; // leave room for score text at top

    // Board outline
    for (int r=0;r<=BOARD_H;++r){
        for (int c=0;c<=BOARD_W;++c){
            pset(ox + c*CELL_SZ, oy + r*CELL_SZ);
        }
    }
    // Static blocks
    for (int r=0;r<BOARD_H;++r){
        for (int c=0;c<BOARD_W;++c){
            if (state->board[r][c]){
                for (int yy=0; yy<CELL_SZ-1; ++yy)
                    for (int xx=0; xx<CELL_SZ-1; ++xx)
                        pset(ox + c*CELL_SZ + 1 + xx, oy + r*CELL_SZ + 1 + yy);
            }
        }
    }
    // Falling piece
    const cell_t *falling_cells = tetris_piece_cells(falling);
    for (int i = 0; i < 4; ++i){
        int bx = falling.x + falling_cells[i].x;
        int by = falling.y + falling_cells[i].y;
        if (bx>=0 && bx<BOARD_W && by>=0 && by<BOARD_H){
            for (int yy=0; yy<CELL_SZ-2; ++yy)
                for (int xx=0; xx<CELL_SZ-2; ++xx)
                    pset(ox + bx*CELL_SZ + 1 + xx, oy + by*CELL_SZ + 1 + yy);
        }
    }

    // Next piece preview (top-right corner)
    piece_t next = { 0, 0, state->next_shape, 0 };
    const cell_t *next_cells = tetris_piece_cells(next);
    int px = PANEL_W - 4*CELL_SZ - 2;
    int py = 0;
    for (int i = 0; i < 4; ++i){
        int c = next_cells[i].x;
        int r = next_cells[i].y;
        for (int yy=0; yy<CELL_SZ-2; ++yy)
            for (int xx=0; xx<CELL_SZ-2; ++xx)
                pset(px + c*CELL_SZ + xx, py + r*CELL_SZ + yy);
    }
}

static void tetris_run(void)
{
    s_tetris_stop = false;
    tetris_game_init();
    TickType_t last = xTaskGetTickCount();
    const TickType_t drop_ticks_normal = pdMS_TO_TICKS(450);
    TickType_t last_move = last;
    const TickType_t move_ticks = pdMS_TO_TICKS(140);
    input_button_t last_btn = INPUT_BTN_NONE;

    while (!s_tetris_stop) {
        // simple input: A=left, B=rot left, C=rot right, D=right
        input_button_t b = INPUT_BTN_NONE;
        int mv = 0;
        if (!input_sample(&b, &mv)) {
            b = INPUT_BTN_NONE;
        }
        TickType_t now = xTaskGetTickCount();
        bool move_ready = (now - last_move) >= move_ticks;

        if (b != last_btn && move_ready) {
            if (b == INPUT_BTN_A) {
                tetris_game_move(-1);
                last_move = now;
            } else if (b == INPUT_BTN_D) {
                tetris_game_move(1);
                last_move = now;
            } else if (b == INPUT_BTN_B) {
                tetris_game_rotate(-1);
                last_move = now;
            } else if (b == INPUT_BTN_C) {
                tetris_game_rotate(1);
                last_move = now;
            }
        }
        last_btn = b;

        TickType_t drop_int = drop_ticks_normal;
        if ((now - last) >= drop_int) {
            tetris_game_drop();
            last = now;
        }

        draw_board();
        const tetris_state_t *state=tetris_game_state();
        // Scores on top (simple left/right)
        char buf[16];
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)state->score);
        oled_draw_text3x5(g_fb, 2, 1, buf);
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)state->best);
        int len = (int)strlen(buf);
        oled_draw_text3x5(g_fb, PANEL_W - 4*len - 2, 1, buf);

        vTaskDelay(pdMS_TO_TICKS(30));
    }

    tetris_game_finish();
}



#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#else
#define APP_TASK_CORE 0
#endif

static TaskHandle_t s_tetris_task = NULL;

const shell_legend_t TETRIS_LEGEND = {
    .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE },
};

static void tetris_task_fn(void *arg)
{
    (void)arg;
    tetris_run();
    s_tetris_task = NULL;
    vTaskDelete(NULL);
}

void tetris_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    if (!s_tetris_task) {
        xTaskCreatePinnedToCore(tetris_task_fn, "tetris", 4096, NULL, 5, &s_tetris_task, APP_TASK_CORE);
    }
}

void tetris_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    s_tetris_stop = true;
    for (int i = 0; i < 50 && s_tetris_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_tetris_task = NULL;
}

void tetris_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    if (!fb) return;
    memcpy(fb,g_fb,sizeof(g_fb));
}
