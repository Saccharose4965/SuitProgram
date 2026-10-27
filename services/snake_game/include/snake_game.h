#pragma once
#include <stdbool.h>

#define SNAKE_GRID_W 29
#define SNAKE_GRID_H 11

typedef struct { int x, y; } snake_cell_t;
typedef struct {
    snake_cell_t body[120];
    int len;
    int dir; // 0=right,1=down,2=left,3=up
    int food_x;
    int food_y;
    int queued_dir[2];
    int queued_count;
    bool alive;
    float acc;
} snake_state_t;


void snake_game_init(void);
void snake_game_reset(void);
void snake_game_queue_direction(int direction);
void snake_game_tick(float dt_sec);
const snake_state_t *snake_game_state(void);
int snake_game_best(void);
