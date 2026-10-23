#include "app_shell.h"
#include "flappy_game.h"
#include "ui_draw.h"
#include "oled.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
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


static volatile bool g_stop, s_press_pending;
static bool button_take_press(void) {
    if (!s_press_pending) return false;
    s_press_pending=false;
    return true;
}
static void flappy_run(void) {
    g_stop=false;
    s_press_pending=false;
    fb_clear();
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

        // wait for button press trigger
        while (!button_take_press()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }


    flappy_game_init();
    const TickType_t frame_delay=pdMS_TO_TICKS(1000/FPS);
    const flappy_state_t *state=flappy_game_state();
    while(state->running && !g_stop) {
        bool flap=state->state==STATE_PLAYING && button_take_press();
        flappy_game_step(flap);
        // --- render ---
        fb_clear();
        for (int i = 0; i < 3; i++) {
            draw_pipe(state->pipe_x[i], state->pipe_gap_y[i]);
        }
        draw_ground();
        bool bird_visible = (state->state == STATE_PLAYING) || ((state->death_frames / 6) % 2 == 0);
        if (bird_visible) {
            draw_bird((int)roundf(state->bird_x), (int)roundf(state->bird_y), state->upside_down);
        }
        draw_score(2, 2, state->score);
        draw_score(FLAPPY_WIDTH - 4*3 - 2, 2, state->best);
        vTaskDelay(frame_delay);
    }
    flappy_game_finish();
}



#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#else
#define APP_TASK_CORE 0
#endif

static TaskHandle_t s_flappy_task = NULL;

const shell_legend_t FLAPPY_LEGEND = {
    .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE },
};

static void flappy_task_fn(void *arg)
{
    (void)arg;
    flappy_run(); // blocking
    s_flappy_task = NULL;
    vTaskDelete(NULL);
}

void flappy_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev) return;
    if (ev->type == INPUT_EVENT_PRESS) {
        s_press_pending=true;
    }
}

void flappy_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    if (!s_flappy_task) {
        xTaskCreatePinnedToCore(flappy_task_fn, "flappy", 4096, NULL, 5, &s_flappy_task, APP_TASK_CORE);
    }
}

void flappy_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    g_stop=true;
    // wait briefly for task to exit
    for (int i = 0; i < 50 && s_flappy_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_flappy_task = NULL;
}

void flappy_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    if (!fb) return;
    memcpy(fb,gfb,sizeof(gfb));
}
