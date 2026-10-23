#pragma once
#include <stdint.h>
#include <stdbool.h>
#define BOARD_W 10
#define BOARD_H 12
typedef struct { int x,y; uint8_t shape; uint8_t rot; } piece_t;
typedef struct { int8_t x, y; } cell_t;


typedef struct {
    uint8_t board[BOARD_H][BOARD_W];
    piece_t current;
    uint8_t next_shape;
    uint32_t score, best;
} tetris_state_t;
void tetris_game_init(void);
void tetris_game_finish(void);
void tetris_game_move(int dx);
void tetris_game_rotate(int delta);
void tetris_game_drop(void);
const tetris_state_t *tetris_game_state(void);
const cell_t *tetris_piece_cells(piece_t piece);
