#pragma once
#include <stdbool.h>
typedef struct { float x, y, vx, vy; } pong_ball_t;
void pong_advance(pong_ball_t *ball, float seconds, float top, float bottom);
bool pong_paddle_hit(const pong_ball_t *ball, float paddle_y, float height, float radius);
void pong_bounce(pong_ball_t *ball, unsigned side, float paddle_y, float height);
float pong_correction_weight(float elapsed, float duration);
