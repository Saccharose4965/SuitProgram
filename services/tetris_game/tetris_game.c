#include "tetris_game.h"
#include "scores.h"
#include "esp_random.h"
#include "esp_log.h"
#include <string.h>
static tetris_state_t s;
static bool high_dirty;
static const char *TAG="tetris";
static const char *kScoreKey="tetris";
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

const cell_t *tetris_piece_cells(piece_t p)
{
    return kShapeCells[p.shape % 7][p.rot & 3];
}

static bool collision(piece_t p){
    const cell_t *cells = tetris_piece_cells(p);
    for (int i = 0; i < 4; ++i){
        int bx = p.x + cells[i].x;
        int by = p.y + cells[i].y;
        if (bx < 0 || bx >= BOARD_W || by >= BOARD_H) return true;
        if (by >= 0 && s.board[by][bx]) return true;
    }
    return false;
}

static void place(piece_t p){
    const cell_t *cells = tetris_piece_cells(p);
    for (int i = 0; i < 4; ++i){
        int bx = p.x + cells[i].x;
        int by = p.y + cells[i].y;
        if (bx>=0 && bx<BOARD_W && by>=0 && by<BOARD_H){
            s.board[by][bx] = 1;
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
        for (int c=0; c<BOARD_W; ++c){ if (!s.board[r][c]) { full=false; break; } }
        if (full){
            for (int rr=r; rr>0; --rr) memcpy(s.board[rr], s.board[rr-1], BOARD_W);
            memset(s.board[0], 0, BOARD_W);
            r++; // recheck this row
            cleared++;
        }
    }
    return cleared;
}


void tetris_game_init(void) {
    memset(s.board, 0, sizeof(s.board));
    s.next_shape = (uint8_t)rand_shape();
    s.current = (piece_t){ BOARD_W/2 - 2, -2, s.next_shape, 0 };
    s.next_shape = (uint8_t)rand_shape();
    s.score = 0; // reset per run
    high_dirty = false;
    uint32_t saved_high = 0;
    if (scores_load_high(kScoreKey, &saved_high) == ESP_OK) {
        s.best = saved_high;
    } else {
        s.best = 0;
        ESP_LOGW(TAG, "Failed to load high score");
    }
}
void tetris_game_drop(void) {
    piece_t p = s.current;
    p.y++;
    if (!collision(p)) {
        s.current = p;
        return;
    }
    if (s.current.y < 0) {
        memset(s.board, 0, sizeof(s.board));
        s.score = 0;
    } else {
        place(s.current);
        int cleared = clear_lines();
        if (cleared > 0) {
            s.score += (uint32_t)(cleared * 100);
            if (s.score > s.best) {
                s.best = s.score;
                high_dirty = true;
            }
        }
    }
    s.current = (piece_t){BOARD_W/2 - 2, -2, s.next_shape, 0};
    s.next_shape = (uint8_t)rand_shape();
}
void tetris_game_finish(void) {
    if (high_dirty) {
        uint32_t high_out = 0;
        if (scores_update_high(kScoreKey, s.best, &high_out) == ESP_OK) {
            s.best = high_out;
        } else {
            ESP_LOGW(TAG, "Failed to save high score");
        }
    }
}

void tetris_game_move(int dx) {
    piece_t p=s.current; p.x+=dx;
    if (!collision(p)) s.current=p;
}
void tetris_game_rotate(int delta) { (void)try_rotate(&s.current,delta); }
const tetris_state_t *tetris_game_state(void) { return &s; }
