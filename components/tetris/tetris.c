#include "tetris.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_random.h"

#include "input.h"
#include "hw.h"
#include "oled.h"
#include "scores.h"

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
static const char *TAG = "tetris";
static const char *kScoreKey = "tetris";

typedef struct { int x,y; uint8_t shape; uint8_t rot; } piece_t;
typedef struct { int8_t x, y; } cell_t;

static inline uint32_t urand(void){ return esp_random(); }
static inline int rand_shape(void){ return (int)(urand() % 7); }

// Tetromino cells in a 4x4 local box. Explicit cells keep rotation states
// aligned and avoid malformed masks.
static const cell_t kShapeCells[7][4][4] = {
    { // I
        {{0,1}, {1,1}, {2,1}, {3,1}},
        {{1,0}, {1,1}, {1,2}, {1,3}},
        {{0,2}, {1,2}, {2,2}, {3,2}},
        {{2,0}, {2,1}, {2,2}, {2,3}},
    },
    { // O
        {{1,0}, {2,0}, {1,1}, {2,1}},
        {{1,0}, {2,0}, {1,1}, {2,1}},
        {{1,0}, {2,0}, {1,1}, {2,1}},
        {{1,0}, {2,0}, {1,1}, {2,1}},
    },
    { // T
        {{1,0}, {0,1}, {1,1}, {2,1}},
        {{1,0}, {1,1}, {2,1}, {1,2}},
        {{0,1}, {1,1}, {2,1}, {1,2}},
        {{1,0}, {0,1}, {1,1}, {1,2}},
    },
    { // S
        {{1,0}, {2,0}, {0,1}, {1,1}},
        {{1,0}, {1,1}, {2,1}, {2,2}},
        {{1,1}, {2,1}, {0,2}, {1,2}},
        {{0,0}, {0,1}, {1,1}, {1,2}},
    },
    { // Z
        {{0,0}, {1,0}, {1,1}, {2,1}},
        {{2,0}, {1,1}, {2,1}, {1,2}},
        {{0,1}, {1,1}, {1,2}, {2,2}},
        {{1,0}, {0,1}, {1,1}, {0,2}},
    },
    { // J
        {{0,0}, {0,1}, {1,1}, {2,1}},
        {{1,0}, {2,0}, {1,1}, {1,2}},
        {{0,1}, {1,1}, {2,1}, {2,2}},
        {{1,0}, {1,1}, {0,2}, {1,2}},
    },
    { // L
        {{2,0}, {0,1}, {1,1}, {2,1}},
        {{1,0}, {1,1}, {1,2}, {2,2}},
        {{0,1}, {1,1}, {2,1}, {0,2}},
        {{0,0}, {1,0}, {1,1}, {1,2}},
    },
};

static inline const cell_t *piece_cells(piece_t p)
{
    return kShapeCells[p.shape % 7][p.rot & 3];
}

static inline void fb_clear(void){ memset(g_fb, 0, sizeof(g_fb)); }
static inline void pset(int x,int y){
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y*PANEL_W + x;
    g_fb[idx>>3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static bool collision(piece_t p){
    const cell_t *cells = piece_cells(p);
    for (int i = 0; i < 4; ++i){
        int bx = p.x + cells[i].x;
        int by = p.y + cells[i].y;
        if (bx < 0 || bx >= BOARD_W || by >= BOARD_H) return true;
        if (by >= 0 && g_board[by][bx]) return true;
    }
    return false;
}

static void place(piece_t p){
    const cell_t *cells = piece_cells(p);
    for (int i = 0; i < 4; ++i){
        int bx = p.x + cells[i].x;
        int by = p.y + cells[i].y;
        if (bx>=0 && bx<BOARD_W && by>=0 && by<BOARD_H){
            g_board[by][bx] = 1;
        }
    }
}

static bool try_rotate(piece_t *cur, int delta)
{
    static const int8_t kKickTests[][2] = {
        { 0,  0},
        {-1,  0},
        { 1,  0},
        {-2,  0},
        { 2,  0},
        { 0, -1},
    };

    if (!cur) return false;
    piece_t rotated = *cur;
    rotated.rot = (uint8_t)((rotated.rot + delta) & 3);
    for (size_t i = 0; i < sizeof(kKickTests) / sizeof(kKickTests[0]); ++i) {
        piece_t cand = rotated;
        cand.x += kKickTests[i][0];
        cand.y += kKickTests[i][1];
        if (!collision(cand)) {
            *cur = cand;
            return true;
        }
    }
    return false;
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
    const cell_t *falling_cells = piece_cells(falling);
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
    piece_t next = { 0, 0, g_next_shape, 0 };
    const cell_t *next_cells = piece_cells(next);
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

void tetris_run(void)
{
    s_tetris_stop = false;
    memset(g_board, 0, sizeof(g_board));
    g_next_shape = (uint8_t)rand_shape();
    piece_t cur = { BOARD_W/2 - 2, -2, g_next_shape, 0 };
    g_next_shape = (uint8_t)rand_shape();
    g_score = 0; // reset per run
    bool high_dirty = false;
    uint32_t saved_high = 0;
    if (scores_load_high(kScoreKey, &saved_high) == ESP_OK) {
        g_high = saved_high;
    } else {
        g_high = 0;
        ESP_LOGW(TAG, "Failed to load high score");
    }
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
                piece_t p = cur; p.x--;
                if (!collision(p)) cur = p;
                last_move = now;
            } else if (b == INPUT_BTN_D) {
                piece_t p = cur; p.x++;
                if (!collision(p)) cur = p;
                last_move = now;
            } else if (b == INPUT_BTN_B) {
                (void)try_rotate(&cur, -1);
                last_move = now;
            } else if (b == INPUT_BTN_C) {
                (void)try_rotate(&cur, 1);
                last_move = now;
            }
        }
        last_btn = b;

        TickType_t drop_int = drop_ticks_normal;
        if ((now - last) >= drop_int) {
            piece_t p = cur; p.y++;
            if (collision(p)) {
                if (cur.y < 0) { // game over
                    memset(g_board, 0, sizeof(g_board));
                    g_score = 0;
                    cur.x = BOARD_W/2 - 2;
                    cur.y = -2;
                    cur.shape = g_next_shape;
                    cur.rot = 0;
                    g_next_shape = (uint8_t)rand_shape();
                } else {
                    place(cur);
                    int cleared = clear_lines();
                    if (cleared > 0) {
                        // 2048-style additive: more lines -> more points
                        g_score += (uint32_t)(cleared * 100);
                        if (g_score > g_high) {
                            g_high = g_score;
                            high_dirty = true;
                        }
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

        vTaskDelay(pdMS_TO_TICKS(30));
    }

    if (high_dirty) {
        uint32_t high_out = 0;
        if (scores_update_high(kScoreKey, g_high, &high_out) == ESP_OK) {
            g_high = high_out;
        } else {
            ESP_LOGW(TAG, "Failed to save high score");
        }
    }
}

void tetris_request_stop(void)
{
    s_tetris_stop = true;
}

void tetris_copy_frame(uint8_t *dst_fb, size_t dst_len){
    if (!dst_fb || dst_len < sizeof(g_fb)) return;
    memcpy(dst_fb, g_fb, sizeof(g_fb));
}
