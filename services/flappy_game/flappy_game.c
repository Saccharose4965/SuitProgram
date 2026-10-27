#include "flappy_game.h"
#include "scores.h"
#include "esp_random.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#define DT (1.0f / FPS)
#define GRAVITY 75.0f
#define FLAP_VY (-55.0f)
#define MAX_VY 140.0f
#define PIPE_SPACING 70
#define PIPE_SPEED 26.0f
#define BIRD_MARGIN 2
#define BIRD_SLIDE_SPEED 32.0f
static flappy_state_t s;
static const char *TAG="flappy";
static const char *kScoreKey="flappy";
// PRNG
static inline uint32_t urand(void){
    return esp_random();
}
static int rand_range(int a, int b){ // inclusive [a..b]
    uint32_t r = urand();
    return a + (int)(r % (uint32_t)(b - a + 1));
}

static bool bird_hits_pipe(int bird_x, int bird_y, int pipe_x, int gap_y, bool flip_vert, int margin){
    int gap_top = gap_y - PIPE_GAP/2 - margin;
    int gap_bot = gap_y + PIPE_GAP/2 + margin;
    int body_x0 = pipe_x - margin;
    int body_x1 = pipe_x + PIPE_W - 1 + margin;
    int cap_x0  = pipe_x - 1 - margin;
    int cap_x1  = pipe_x + PIPE_W + margin;
    int cap_top0 = gap_top - 2 - margin;
    int cap_top1 = gap_top - 1 + margin;
    int cap_bot0 = gap_bot + 1 - margin;
    int cap_bot1 = gap_bot + 2 + margin;

    int y0 = bird_y - BIRD_H/2;
    int x0 = bird_x - BIRD_W/2;

    for (int r = 0; r < BIRD_H; ++r) {
        int rr = flip_vert ? (BIRD_H - 1 - r) : r;
        uint16_t row = flappy_bird_rows[rr];
        int yy = y0 + r;
        for (int c = 0; c < BIRD_W; ++c) {
            if (!(row & (1u << (BIRD_W - 1 - c)))) continue;
            int xx = x0 + c;
            // Off-screen -> collision
            if (yy < 0 || yy >= FLAPPY_HEIGHT) return true;
            if (xx < 0 || xx >= FLAPPY_WIDTH) continue;

            bool in_pipe_body = (xx >= body_x0 && xx <= body_x1) &&
                                (yy < gap_top || yy > gap_bot);
            bool in_caps = (xx >= cap_x0 && xx <= cap_x1) &&
                           ((yy >= cap_top0 && yy <= cap_top1) ||
                            (yy >= cap_bot0 && yy <= cap_bot1));
            if (in_pipe_body || in_caps) return true;
        }
    }
    return false;
}


void flappy_game_init(void) {
    memset(&s,0,sizeof(s));
    s.bird_x=BIRD_START_X;
    s.bird_y=FLAPPY_HEIGHT/2.0f;
    s.state=STATE_PLAYING;
    s.running=true;
    uint32_t saved_best = 0;
    if (scores_load_high(kScoreKey, &saved_best) == ESP_OK) {
        s.best = (int)saved_best;
    } else {
        ESP_LOGW(TAG, "Failed to load high s.score");
    }


    int start_x = FLAPPY_WIDTH + 20;
    for (int i = 0; i < 3; i++){
        s.pipe_x[i]    = start_x + i * PIPE_SPACING;
        s.pipe_gap_y[i] = rand_range(20, FLAPPY_HEIGHT - 20);
    }

 }
void flappy_game_step(bool flap) {
    if(s.state==STATE_PLAYING && flap) s.bird_vy=FLAP_VY;
        // --- physics ---
        // Pipes keep moving in all states
        s.acc_px += PIPE_SPEED * DT;
        while (s.acc_px >= 1.0f) {
            for (int i = 0; i < 3; i++) {
                s.pipe_x[i] -= 1;
            }
            s.acc_px -= 1.0f;
        }

        // recycle pipes
        for (int i = 0; i < 3; i++) {
            if (s.pipe_x[i] + PIPE_W < 0) {
                int furthest = s.pipe_x[0];
                for (int k = 1; k < 3; k++) {
                    if (s.pipe_x[k] > furthest) {
                        furthest = s.pipe_x[k];
                    }
                }
                s.pipe_x[i]     = furthest + PIPE_SPACING;
                s.pipe_gap_y[i] = rand_range(18, FLAPPY_HEIGHT - 18);
                s.pipe_scored[i] = false;
            }
        }

        if (s.state == STATE_PLAYING) {
            s.bird_vy += GRAVITY * DT;
            if (s.bird_vy > MAX_VY) s.bird_vy = MAX_VY;
            s.bird_y += s.bird_vy * DT;

            // Each physical pipe scores once when its trailing edge reaches the bird.
            for (int i = 0; i < 3; i++) {
                if (!s.pipe_scored[i] &&
                    s.pipe_x[i] + PIPE_W <= BIRD_START_X - BIRD_W/2) {
                    s.pipe_scored[i] = true;
                    s.score++;
                    if (s.score > s.best) {
                        s.best = s.score;
                        s.best_dirty = true;
                    }
                }
            }
        } else if (s.state == STATE_DEAD_FALL) {
            s.bird_vy += GRAVITY * DT;
            if (s.bird_vy > MAX_VY) s.bird_vy = MAX_VY;
            s.bird_y += s.bird_vy * DT;
            if (s.bird_y > (FLAPPY_HEIGHT - 1 - BIRD_H/2)) {
                s.bird_y = (float)(FLAPPY_HEIGHT - 1 - BIRD_H/2);
                s.bird_vy = 0.0f;
                s.state = STATE_DEAD_SLIDE;
            }
        } else if (s.state == STATE_DEAD_SLIDE) {
            s.bird_y = (float)(FLAPPY_HEIGHT - 1 - BIRD_H/2);
            s.bird_x += BIRD_SLIDE_SPEED * DT;

            bool overlap = false;
            for (int i = 0; i < 3; i++) {
                if (bird_hits_pipe((int)roundf(s.bird_x), (int)roundf(s.bird_y), s.pipe_x[i], s.pipe_gap_y[i], s.upside_down, BIRD_MARGIN)) {
                    overlap = true;
                    break;
                }
            }
            if (!overlap) {
                s.state = STATE_DEAD_SETTLED;
                s.settle_frames = FPS / 2;
                s.upside_down = true;
            }
        } else if (s.state == STATE_DEAD_SETTLED) {
            if (s.settle_frames > 0) s.settle_frames--;
            else {
                // placeholder menu

                s.running = false;
            }
        }
        if (s.state != STATE_PLAYING) {
            s.death_frames++;
        } else {
            s.death_frames = 0;
        }

        // --- collisions ---
        if (s.state == STATE_PLAYING) {
            bool dead = false;
            // Any lit pixel leaving the screen counts as collision (handled in bird_hits_pipe)
            if (s.bird_y < (BIRD_H/2) || s.bird_y > (FLAPPY_HEIGHT - 1 - BIRD_H/2)) {
                dead = true;
            }
            for (int i = 0; i < 3 && !dead; i++) {
                if (bird_hits_pipe((int)roundf(s.bird_x), (int)roundf(s.bird_y), s.pipe_x[i], s.pipe_gap_y[i], false, 0)) {
                    dead = true;
                }
            }
            if (dead) {
                s.state = STATE_DEAD_FALL;
                s.upside_down = true;
                s.death_frames = 0;
            }
        }

 }
void flappy_game_finish(void) {
    if (s.best_dirty) {
        uint32_t best_out = 0;
        if (scores_update_high(kScoreKey, (uint32_t)s.best, &best_out) == ESP_OK) {
            s.best = (int)best_out;
            s.best_dirty = false;
        } else {
            ESP_LOGW(TAG, "Failed to save high s.score");
        }
    }
    ESP_LOGI(TAG, "Flappy: exited");
}

const flappy_state_t *flappy_game_state(void) { return &s; }
