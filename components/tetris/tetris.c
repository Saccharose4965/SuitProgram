#include "tetris.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_random.h"

#include "input.h"
#include "hw.h"
#include "oled.h"

// Simple monochrome Tetris-like demo (falls, move, rotate).
// Not feature-complete; serves as a playable placeholder.

#define BOARD_W 10
#define BOARD_H 12
#define CELL_SZ 4

static uint8_t g_board[BOARD_H][BOARD_W];
static uint8_t g_fb[PANEL_W * PANEL_H / 8];
static uint32_t g_score = 0;
static uint32_t g_high = 0;
static uint8_t g_next_shape = 0;
static volatile bool s_tetris_stop = false;

typedef struct { int x,y; uint8_t shape; uint8_t rot; } piece_t;

static inline uint32_t urand(void){ return esp_random(); }
static inline int rand_shape(void){ return (int)(urand() % 7); }

// Standard tetromino masks in 4x4 (row-major, top→bottom, MSB first)
static const uint16_t kShapes[7][4] = {
    {0x0F00, 0x2222, 0x00F0, 0x4444}, // I
    {0x0660, 0x0660, 0x0660, 0x0660}, // O
    {0x0E40, 0x4C40, 0x4E00, 0x8C80}, // T
    {0x06C0, 0x8C40, 0x06C0, 0x8C40}, // S
    {0x0C60, 0x4C80, 0x0C60, 0x4C80}, // Z
    {0x0E10, 0x44C0, 0x8E00, 0x6440}, // J
    {0x0E80, 0xC440, 0x2E00, 0x4460}, // L
};

static inline void fb_clear(void){ memset(g_fb, 0, sizeof(g_fb)); }
static inline void pset(int x,int y){
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y*PANEL_W + x;
    g_fb[idx>>3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static bool collision(piece_t p){
    uint16_t mask = kShapes[p.shape][p.rot % 4];
    for (int r=0; r<4; ++r){
        for (int c=0; c<4; ++c){
            if (mask & (1u << (15 - (r*4+c)))){
                int bx = p.x + c;
                int by = p.y + r;
                if (bx < 0 || bx >= BOARD_W || by >= BOARD_H) return true;
                if (by >=0 && g_board[by][bx]) return true;
            }
        }
    }
    return false;
}

static void place(piece_t p){
    uint16_t mask = kShapes[p.shape][p.rot % 4];
    for (int r=0; r<4; ++r){
        for (int c=0; c<4; ++c){
            if (mask & (1u << (15 - (r*4+c)))){
                int bx = p.x + c;
                int by = p.y + r;
                if (bx>=0 && bx<BOARD_W && by>=0 && by<BOARD_H){
                    g_board[by][bx] = 1;
                }
            }
        }
    }
}

static int clear_lines(void){
    int cleared = 0;
    for (int r=BOARD_H-1; r>=0; --r){
        bool full=true;
        for (int c=0; c<BOARD_W; ++c){ if (!g_board[r][c]) { full=false; break; } }
        if (full){
            for (int rr=r; rr>0; --rr) memcpy(g_board[rr], g_board[rr-1], BOARD_W);
            memset(g_board[0], 0, BOARD_W);
            r++; // recheck this row
            cleared++;
        }
    }
    return cleared;
}

static void draw_board(piece_t falling){
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
            if (g_board[r][c]){
                for (int yy=0; yy<CELL_SZ-1; ++yy)
                    for (int xx=0; xx<CELL_SZ-1; ++xx)
                        pset(ox + c*CELL_SZ + 1 + xx, oy + r*CELL_SZ + 1 + yy);
            }
        }
    }
    // Falling piece
    uint16_t mask = kShapes[falling.shape][falling.rot % 4];
    for (int r=0; r<4; ++r){
        for (int c=0; c<4; ++c){
            if (mask & (1u << (15 - (r*4+c)))){
                int bx = falling.x + c;
                int by = falling.y + r;
                if (bx>=0 && bx<BOARD_W && by>=0 && by<BOARD_H){
                    for (int yy=0; yy<CELL_SZ-2; ++yy)
                        for (int xx=0; xx<CELL_SZ-2; ++xx)
                            pset(ox + bx*CELL_SZ + 1 + xx, oy + by*CELL_SZ + 1 + yy);
                }
            }
        }
    }

    // Next piece preview (top-right corner)
    uint16_t next_mask = kShapes[g_next_shape][0];
    int px = PANEL_W - 4*CELL_SZ - 2;
    int py = 0;
    for (int r=0; r<4; ++r){
        for (int c=0; c<4; ++c){
            if (next_mask & (1u << (15 - (r*4+c)))){
                for (int yy=0; yy<CELL_SZ-2; ++yy)
                    for (int xx=0; xx<CELL_SZ-2; ++xx)
                        pset(px + c*CELL_SZ + xx, py + r*CELL_SZ + yy);
            }
        }
    }
}

void tetris_run(void)
{
    s_tetris_stop = false;
    memset(g_board, 0, sizeof(g_board));
    g_next_shape = (uint8_t)rand_shape();
    piece_t cur = { BOARD_W/2 - 2, -2, g_next_shape, 0 };
    g_next_shape = (uint8_t)rand_shape();
    g_score = 0; // reset per run
    TickType_t last = xTaskGetTickCount();
    const TickType_t drop_ticks_normal = pdMS_TO_TICKS(450);
    const TickType_t drop_ticks_fast   = pdMS_TO_TICKS(120);
    TickType_t last_move = last;
    const TickType_t move_ticks = pdMS_TO_TICKS(140);
    input_button_t last_btn = INPUT_BTN_NONE;

    while (!s_tetris_stop) {
        // simple input: A=left, B=rot left, C=rot right, D=right; B+C combo = fast drop
        input_button_t b = INPUT_BTN_NONE;
        int mv = 0;
        if (!input_sample(&b, &mv)) {
            b = INPUT_BTN_NONE;
        }
        TickType_t now = xTaskGetTickCount();
        bool move_ready = (now - last_move) >= move_ticks;
        bool fast_drop = (b == INPUT_BTN_BC_COMBO);

        if (b != last_btn && move_ready) {
            if (b == INPUT_BTN_A) {
                piece_t p = cur; p.x--;
                if (!collision(p)) cur = p;
                last_move = now;
            } else if (b == INPUT_BTN_D) {
                piece_t p = cur; p.x++;
                if (!collision(p)) cur = p;
                last_move = now;
            } else if (b == INPUT_BTN_B) {
                piece_t p = cur; p.rot = (p.rot + 3) & 3;
                if (!collision(p)) cur = p;
                last_move = now;
            } else if (b == INPUT_BTN_C) {
                piece_t p = cur; p.rot = (p.rot + 1) & 3;
                if (!collision(p)) cur = p;
                last_move = now;
            }
        }
        last_btn = b;

        TickType_t drop_int = fast_drop ? drop_ticks_fast : drop_ticks_normal;
        if ((now - last) >= drop_int) {
            piece_t p = cur; p.y++;
            if (collision(p)) {
                if (cur.y < 0) { // game over
                    memset(g_board, 0, sizeof(g_board));
                    g_score = 0;
                    g_next_shape = (uint8_t)rand_shape();
                } else {
                    place(cur);
                    int cleared = clear_lines();
                    if (cleared > 0) {
                        // 2048-style additive: more lines -> more points
                        g_score += (uint32_t)(cleared * 100);
                        if (g_score > g_high) g_high = g_score;
                    }
                    cur.x = BOARD_W/2 - 2;
                    cur.y = -2;
                    cur.shape = g_next_shape;
                    cur.rot = 0;
                    g_next_shape = (uint8_t)rand_shape();
                }
            } else {
                cur = p;
            }
            last = now;
        }

        draw_board(cur);
        // Scores on top (simple left/right)
        char buf[16];
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_score);
        oled_draw_text3x5(g_fb, 2, 1, buf);
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_high);
        int len = (int)strlen(buf);
        oled_draw_text3x5(g_fb, PANEL_W - 4*len - 2, 1, buf);

        oled_blit_full(g_fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void tetris_request_stop(void)
{
    s_tetris_stop = true;
}
