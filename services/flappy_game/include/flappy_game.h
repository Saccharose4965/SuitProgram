#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "flappy_shape.h"
#define FLAPPY_WIDTH 128
#define FLAPPY_HEIGHT 64
enum { FPS=30, PIPE_W=14, PIPE_GAP=34, BIRD_START_X=30,
       BIRD_W=FLAPPY_BIRD_W, BIRD_H=FLAPPY_BIRD_H };
typedef enum { STATE_PLAYING, STATE_DEAD_FALL, STATE_DEAD_SLIDE, STATE_DEAD_SETTLED } flappy_phase_t;
typedef struct {
    float bird_x, bird_y, bird_vy, acc_px;
    int pipe_x[3], pipe_gap_y[3], score, best;
    bool pipe_scored[3];
    int settle_frames, death_frames;
    flappy_phase_t state;
    bool upside_down, running, best_dirty;
} flappy_state_t;
void flappy_game_init(void);
void flappy_game_step(bool flap);
void flappy_game_finish(void);
const flappy_state_t *flappy_game_state(void);
