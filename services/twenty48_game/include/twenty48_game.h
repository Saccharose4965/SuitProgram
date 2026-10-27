#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int grid_old[4][4], grid_new[4][4], motion[4][4];
    int direction; // 0 right, 1 down, 2 left, 3 up
    uint32_t score, best;
} t48_state_t;
void t48_game_init(void);
void t48_game_deinit(void);
bool t48_game_move(int direction);
void t48_game_commit(void);
bool t48_game_is_over(void);
const t48_state_t *t48_game_state(void);
