#include "snake_game.h"
#include "scores.h"
#include "esp_log.h"
#include <stdlib.h>

#define SNAKE_STEP_SEC (1.0f/6.0f)

static snake_state_t s_snake = {0};
static int s_best_len = 0;
static const char *TAG = "snake";
static const char *kScoreKey = "snake";

static void snake_spawn_food(void){
    int tries = 0;
    while (tries++ < 64){
        int fx = rand() % SNAKE_GRID_W;
        int fy = rand() % SNAKE_GRID_H;
        bool clash = false;
        for (int i = 0; i < s_snake.len; ++i){
            if (s_snake.body[i].x == fx && s_snake.body[i].y == fy){ clash = true; break; }
        }
        if (!clash){ s_snake.food_x = fx; s_snake.food_y = fy; return; }
    }
    s_snake.food_x = 0; s_snake.food_y = 0;
}

static void snake_reset(void){
    s_snake.len = 4;
    int cx = SNAKE_GRID_W / 2;
    int cy = SNAKE_GRID_H / 2;
    for (int i = 0; i < s_snake.len; ++i){
        s_snake.body[i].x = cx - i;
        s_snake.body[i].y = cy;
    }
    s_snake.dir = 0;
    s_snake.alive = true;
    s_snake.acc = 0.0f;
    snake_spawn_food();
}

void snake_game_init(void){
    uint32_t saved_best = 0;
    if (scores_load_high(kScoreKey, &saved_best) == ESP_OK) {
        s_best_len = (int)saved_best;
    } else {
        s_best_len = 0;
        ESP_LOGW(TAG, "Failed to load high score");
    }
    if (s_best_len < 4) s_best_len = 4;
    snake_reset();
}


void snake_game_press(int direction)
{
    if (!s_snake.alive) { snake_reset(); return; }
    if (direction >= 0 && direction < 4 && direction != (s_snake.dir + 2) % 4)
        s_snake.dir = direction;
}

static void snake_step(void){
    if (!s_snake.alive) return;
    snake_cell_t head = s_snake.body[0];
    switch (s_snake.dir){
        case 0: head.x++; break;
        case 1: head.y++; break;
        case 2: head.x--; break;
        case 3: head.y--; break;
        default: break;
    }
    if (head.x < 0 || head.x >= SNAKE_GRID_W || head.y < 0 || head.y >= SNAKE_GRID_H){
        s_snake.alive = false;
        return;
    }
    for (int i = 0; i < s_snake.len; ++i){
        if (s_snake.body[i].x == head.x && s_snake.body[i].y == head.y){
            s_snake.alive = false;
            return;
        }
    }
    for (int i = s_snake.len - 1; i > 0; --i){
        s_snake.body[i] = s_snake.body[i-1];
    }
    s_snake.body[0] = head;
    if (head.x == s_snake.food_x && head.y == s_snake.food_y){
        if (s_snake.len < (int)(sizeof(s_snake.body)/sizeof(s_snake.body[0]))){
            s_snake.body[s_snake.len] = s_snake.body[s_snake.len-1];
            s_snake.len++;
            if (s_snake.len > s_best_len){
                uint32_t best_out = 0;
                if (scores_update_high(kScoreKey, (uint32_t)s_snake.len, &best_out) == ESP_OK){
                    s_best_len = (int)best_out;
                } else {
                    s_best_len = s_snake.len;
                    ESP_LOGW(TAG, "Failed to save high score");
                }
            }
        }
        snake_spawn_food();
    }
}

void snake_game_tick(float dt_sec){
    s_snake.acc += dt_sec;
    while (s_snake.acc >= SNAKE_STEP_SEC){
        snake_step();
        s_snake.acc -= SNAKE_STEP_SEC;
    }
}


const snake_state_t *snake_game_state(void) { return &s_snake; }
int snake_game_best(void) { return s_best_len; }
