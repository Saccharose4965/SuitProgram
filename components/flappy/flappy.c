#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_random.h"

#include "hw.h"
#include "oled.h"
#include "flappy.h"
#include "flappy_sprite.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "flappy";

// ----------------- Framebuffer helpers -----------------
static uint8_t gfb[PANEL_W * PANEL_H / 8];

static inline void fb_clear(void) { memset(gfb, 0, sizeof(gfb)); }

static inline void pset(int x, int y) {
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    gfb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void vline(int x, int y0, int y1) {
    if (x < 0 || x >= PANEL_W) return;
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    if (y1 < 0 || y0 >= PANEL_H) return;
    if (y0 < 0) y0 = 0;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        pset(x, y);
    }
}

static void rect_fill(int x0, int y0, int w, int h) {
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;

    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;

    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            pset(x, y);
        }
    }
}

// tiny 3×5 digit font for score (top-left)
static const uint8_t DIGIT_3x5[10][5] = {
    {0b111,0b101,0b101,0b101,0b111}, //0
    {0b010,0b110,0b010,0b010,0b111}, //1
    {0b111,0b001,0b111,0b100,0b111}, //2
    {0b111,0b001,0b111,0b001,0b111}, //3
    {0b101,0b101,0b111,0b001,0b001}, //4
    {0b111,0b100,0b111,0b001,0b111}, //5
    {0b111,0b100,0b111,0b101,0b111}, //6
    {0b111,0b001,0b010,0b010,0b010}, //7
    {0b111,0b101,0b111,0b101,0b111}, //8
    {0b111,0b101,0b111,0b001,0b111}, //9
};

static void draw_digit3x5(int x, int y, int d){
    if (d < 0 || d > 9) return;
    for (int r = 0; r < 5; r++){
        uint8_t row = DIGIT_3x5[d][r];
        for (int c = 0; c < 3; c++){
            if (row & (1u << (2 - c))) {
                pset(x + c, y + r);
            }
        }
    }
}

static void draw_score(int x, int y, int score){
    int s = (score < 0) ? 0 : score;
    int digs[5];
    int n = 0;

    do {
        digs[n++] = s % 10;
        s /= 10;
    } while (s && n < 5);

    for (int i = 0; i < n; i++) {
        draw_digit3x5(x + (n - 1 - i) * 4, y, digs[i]);
    }
}

// ----------------- Game params -----------------
enum { FPS = 30 };
#define DT          (1.0f / (float)FPS)

// Tuned for 64px height
#define GRAVITY     (75.0f)        // px/s^2 (down)
#define FLAP_VY     (-55.0f)       // px/s (up)
#define MAX_VY      (140.0f)       // clamp fall speed

// Pipes: slightly slower & further apart for comfort
#define PIPE_W      14
#define PIPE_GAP    34             // vertical opening
#define PIPE_SPACING  (70)         // distance between pipes
#define PIPE_SPEED  (26.0f)        // px/s

#define BIRD_START_X (30)
#define BIRD_W      FLAPPY_BIRD_W
#define BIRD_H      FLAPPY_BIRD_H
#define BIRD_MARGIN (2)            // margin used when checking overlap after death
#define BIRD_SLIDE_SPEED (32.0f)   // px/s when sliding on ground after death

// PRNG
static inline uint32_t urand(void){
    return esp_random();
}
static int rand_range(int a, int b){ // inclusive [a..b]
    uint32_t r = urand();
    return a + (int)(r % (uint32_t)(b - a + 1));
}

// Button helper
static bool button_is_pressed(void){
    hw_button_id_t b = HW_BTN_NONE;
    if (hw_buttons_read(&b) != ESP_OK) {
        return false;
    }
    return (b != HW_BTN_NONE);
}

static volatile bool g_stop = false;
void flappy_request_stop(void){ g_stop = true; }

// ----------------- Rendering -----------------
static void draw_bird(int x, int y, bool flip_vert){
    int x0 = x - BIRD_W/2;
    int y0 = y - BIRD_H/2;
    for (int r = 0; r < BIRD_H; ++r) {
        int rr = flip_vert ? (BIRD_H - 1 - r) : r;
        int yy = y0 + r;
        if ((unsigned)yy >= PANEL_H) continue;
        uint16_t row = flappy_bird_rows[rr];
        for (int c = 0; c < BIRD_W; ++c) {
            int xx = x0 + c;
            if ((unsigned)xx >= PANEL_W) continue;
            if (row & (1u << (BIRD_W - 1 - c))) {
                pset(xx, yy);
            }
        }
    }
}

static void draw_pipe(int x, int gap_y){
    int gap_top = gap_y - PIPE_GAP/2;
    int gap_bot = gap_y + PIPE_GAP/2;

    for (int cx = x; cx < x + PIPE_W; ++cx) {
        if (cx < 0 || cx >= PANEL_W) continue;
        vline(cx, 0, gap_top - 1);
        vline(cx, gap_bot + 1, PANEL_H - 1);
    }
    // caps
    rect_fill(x - 1, gap_top - 2, PIPE_W + 2, 2);
    rect_fill(x - 1, gap_bot + 1, PIPE_W + 2, 2);
}

static void draw_ground(void){
    for (int x = 0; x < PANEL_W; x++) {
        pset(x, PANEL_H - 1);
    }
}

// ----------------- Collision -----------------
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
            if (yy < 0 || yy >= PANEL_H) return true;
            if (xx < 0 || xx >= PANEL_W) continue;

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

// ----------------- Main loop -----------------
void flappy_run(void)
{
    g_stop = false;
    // Shell owns OLED initialization; this app only renders.
    oled_clear();

    // --- splash screen ---
    {
        fb_clear();
        // Simple decorative lines
        rect_fill(16, 10, 96, 1);
        rect_fill(16, 28, 96, 1);

        // Bird preview in the middle
        draw_bird(BIRD_START_X, PANEL_H / 2, false);

        // “Press to start” as two bars
        rect_fill(8, PANEL_H - 18, 3, 3);
        rect_fill(14, PANEL_H - 20, 60, 1);
        rect_fill(8, PANEL_H - 10, 3, 3);
        rect_fill(14, PANEL_H - 12, 42, 1);

        oled_blit_full(gfb);

        // wait for button press
        while (!button_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // debounce / wait for release
        while (button_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Game state
    float bird_x = BIRD_START_X;
    float bird_y = PANEL_H / 2.0f;
    float bird_vy = 0.0f;

    int pipe_x[3];
    int pipe_gap_y[3];
    int score = 0;
    int best  = 0;
    float acc_px = 0.0f;

    int start_x = PANEL_W + 20;
    for (int i = 0; i < 3; i++){
        pipe_x[i]    = start_x + i * PIPE_SPACING;
        pipe_gap_y[i] = rand_range(20, PANEL_H - 20);
    }

    const TickType_t frame_delay = pdMS_TO_TICKS(1000 / FPS);
    enum { STATE_PLAYING = 0, STATE_DEAD_FALL, STATE_DEAD_SLIDE, STATE_DEAD_SETTLED } state = STATE_PLAYING;
    int settle_frames = 0;
    bool upside_down = false;
    int death_frames = 0;
    bool running = true;
    bool pressed_prev = false;

    while (running) {
        if (g_stop) {
            break;
        }
        // --- input ---
        bool pressed = button_is_pressed();
        if (state == STATE_PLAYING && pressed && !pressed_prev) {
            bird_vy = FLAP_VY;
        }
        pressed_prev = pressed;

        // --- physics ---
        // Pipes keep moving in all states
        acc_px += PIPE_SPEED * DT;
        while (acc_px >= 1.0f) {
            for (int i = 0; i < 3; i++) {
                pipe_x[i] -= 1;
            }
            acc_px -= 1.0f;
        }

        // recycle pipes
        for (int i = 0; i < 3; i++) {
            if (pipe_x[i] + PIPE_W < 0) {
                int furthest = pipe_x[0];
                for (int k = 1; k < 3; k++) {
                    if (pipe_x[k] > furthest) {
                        furthest = pipe_x[k];
                    }
                }
                pipe_x[i]     = furthest + PIPE_SPACING;
                pipe_gap_y[i] = rand_range(18, PANEL_H - 18);
            }
        }

        if (state == STATE_PLAYING) {
            bird_vy += GRAVITY * DT;
            if (bird_vy > MAX_VY) bird_vy = MAX_VY;
            bird_y += bird_vy * DT;

            // scoring edge (bird passes trailing edge)
            for (int i = 0; i < 3; i++) {
                if (pipe_x[i] + PIPE_W == BIRD_START_X - BIRD_W/2) {
                    score++;
                    if (score > best) best = score;
                }
            }
        } else if (state == STATE_DEAD_FALL) {
            bird_vy += GRAVITY * DT;
            if (bird_vy > MAX_VY) bird_vy = MAX_VY;
            bird_y += bird_vy * DT;
            if (bird_y > (PANEL_H - 1 - BIRD_H/2)) {
                bird_y = (float)(PANEL_H - 1 - BIRD_H/2);
                bird_vy = 0.0f;
                state = STATE_DEAD_SLIDE;
            }
        } else if (state == STATE_DEAD_SLIDE) {
            bird_y = (float)(PANEL_H - 1 - BIRD_H/2);
            bird_x += BIRD_SLIDE_SPEED * DT;

            bool overlap = false;
            for (int i = 0; i < 3; i++) {
                if (bird_hits_pipe((int)roundf(bird_x), (int)roundf(bird_y), pipe_x[i], pipe_gap_y[i], upside_down, BIRD_MARGIN)) {
                    overlap = true;
                    break;
                }
            }
            if (!overlap) {
                state = STATE_DEAD_SETTLED;
                settle_frames = FPS / 2;
                upside_down = true;
            }
        } else if (state == STATE_DEAD_SETTLED) {
            if (settle_frames > 0) settle_frames--;
            else {
                // placeholder menu
                oled_clear();
                oled_render_three_lines("MENU", "(placeholder)", "");
                running = false;
            }
        }
        if (state != STATE_PLAYING) {
            death_frames++;
        } else {
            death_frames = 0;
        }

        // --- collisions ---
        if (state == STATE_PLAYING) {
            bool dead = false;
            // Any lit pixel leaving the screen counts as collision (handled in bird_hits_pipe)
            if (bird_y < (BIRD_H/2) || bird_y > (PANEL_H - 1 - BIRD_H/2)) {
                dead = true;
            }
            for (int i = 0; i < 3 && !dead; i++) {
                if (bird_hits_pipe((int)roundf(bird_x), (int)roundf(bird_y), pipe_x[i], pipe_gap_y[i], false, 0)) {
                    dead = true;
                }
            }
            if (dead) {
                state = STATE_DEAD_FALL;
                upside_down = true;
                death_frames = 0;
            }
        }

        // --- render ---
        fb_clear();
        for (int i = 0; i < 3; i++) {
        draw_pipe(pipe_x[i], pipe_gap_y[i]);
    }
    draw_ground();
    bool bird_visible = (state == STATE_PLAYING) || ((death_frames / 6) % 2 == 0);
    if (bird_visible) {
        draw_bird((int)roundf(bird_x), (int)roundf(bird_y), upside_down);
    }
    draw_score(2, 2, score);                   // current
    draw_score(PANEL_W - 4*3 - 2, 2, best);    // best

        oled_blit_full(gfb);

        vTaskDelay(frame_delay);
    }

    ESP_LOGI(TAG, "Flappy: exited");
}
