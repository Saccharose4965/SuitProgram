#include "app_shell.h"
#include "twenty48_game.h"
#include "ui_draw.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "esp_log.h"

#include "hw.h"
#include "oled.h"
#include "t48_sprites.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Board layout: tiles 15x15, stride 14, origins at x={35,49,63,77}, y={0,14,28,42}
enum { T48_STRIDE = 14, T48_TILE = T48_TILE_SIZE };
static const int kBoardX0 = 35;
static const int kBoardY0 = 0;

// Framebuffer helpers
static uint8_t g_fb[PANEL_W * PANEL_H / 8];
static uint8_t g_bg[PANEL_W * PANEL_H / 8]; // static background (grid only)
static inline void fb_clear(void){ memset(g_fb, 0, sizeof(g_fb)); }
static inline void fb_pset(int x, int y){
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    g_fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}
static inline void fb_pclr(int x, int y){
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    g_fb[idx >> 3] &= (uint8_t)~(1u << (7 - (idx & 7)));
}
static void fb_clear_rect(int x0, int y0, int w, int h){
    for (int y = y0; y < y0 + h; ++y)
        for (int x = x0; x < x0 + w; ++x)
            fb_pclr(x, y);
}
static void draw_rect(int x0, int y0, int w, int h){
    int x1 = x0 + w - 1, y1 = y0 + h - 1;
    for (int x = x0; x <= x1; ++x){ fb_pset(x, y0); fb_pset(x, y1); }
    for (int y = y0; y <= y1; ++y){ fb_pset(x0, y); fb_pset(x1, y); }
}

static void blit_tile(const uint16_t *rows, int size, int x, int y){
    for (int dy = 0; dy < size; ++dy){
        int yy = y + dy;
        if ((unsigned)yy >= PANEL_H) continue;
        uint16_t bits = rows[dy];
        for (int dx = 0; dx < size; ++dx){
            int xx = x + dx;
            if ((unsigned)xx >= PANEL_W) continue;
            if (bits & (1u << (size - 1 - dx))) fb_pset(xx, yy);
        }
    }
}

static float a, da;
static void build_background(void){
    fb_clear();
    // Board grid
    for (int r = 0; r < 4; ++r){
        for (int c = 0; c < 4; ++c){
            int x = kBoardX0 + c * T48_STRIDE;
            int y = kBoardY0 + r * T48_STRIDE;
            draw_rect(x, y, T48_TILE, T48_TILE);
        }
    }
    memcpy(g_bg, g_fb, sizeof(g_fb));
}

static void render(void){
    const t48_state_t *state = t48_game_state();
    memcpy(g_fb, g_bg, sizeof(g_fb));

    // Draw tiles (during animation: from old grid with state->motion; after: state->grid_old already updated)
    float ca = cosf(a);
    float offset_scale = (ca - 1.0f) * 0.5f; // matches (cos(a)-1)/2

    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            int exp = state->grid_old[i][j];
            if (exp <= 0) continue;
            int sprite = exp - 1;
            if (sprite < 0 || sprite >= T48_TILE_COUNT) continue;

            int base_x = kBoardX0 + j * T48_STRIDE;
            int base_y = kBoardY0 + i * T48_STRIDE;
            // state->motion is positive along dir; apply direction factors like the sketch
            float dx = 0.0f, dy = 0.0f;
            if ((state->direction & 1) == 0){ // horizontal
                dx = offset_scale * (float)(state->direction - 1) * (float)state->motion[i][j] * (float)T48_STRIDE;
            } else {           // vertical
                dy = offset_scale * (float)(state->direction - 2) * (float)state->motion[i][j] * (float)T48_STRIDE;
            }

            int x = base_x + (int)lroundf(dx);
            int y = base_y + (int)lroundf(dy);
            blit_tile(t48_tiles_15[sprite], T48_TILE, x, y);
        }
    }

    // Draw score + high score stacked in the left margin
    fb_clear_rect(0, 0, kBoardX0 - 2, 24);
    oled_draw_text3x5(g_fb, 2, 2, "SCORE:");
    char buf[16];
    snprintf(buf, sizeof(buf), " %u", (unsigned)state->score);
    oled_draw_text3x5(g_fb, 2, 8, buf);
    oled_draw_text3x5(g_fb, 2, 14, "BEST:");
    snprintf(buf, sizeof(buf), " %u", (unsigned)state->best);
    oled_draw_text3x5(g_fb, 2, 20, buf);

}

static void t48_screen_tick(void){
    // Button handling: edges only
    static hw_button_id_t last = HW_BTN_NONE;
    hw_button_id_t b = HW_BTN_NONE;
    if (hw_buttons_read(&b) != ESP_OK) b = HW_BTN_NONE;

    if (last == HW_BTN_NONE && b != HW_BTN_NONE && a == 0.0f){
        int d;
        // Map buttons: A=left, B=up, C=right, D=down
        if      (b == HW_BTN_A) d = 2;
        else if (b == HW_BTN_B) d = 3;
        else if (b == HW_BTN_C) d = 0;
        else if (b == HW_BTN_D) d = 1;
        else d = 0;

        da = t48_game_move(d) ? 0.2f : 0.0f;
    }
    last = b;

    // Animate
    if (da != 0.0f){
        a += da;
        render();
        if (a >= (float)M_PI){
            t48_game_commit();
            a = 0.0f;
            da = 0.0f;
            render();
        }
    }
}




#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#else
#define APP_TASK_CORE 0
#endif

static TaskHandle_t s_t2048_task = NULL;
static volatile bool s_t2048_stop = false;

const shell_legend_t T2048_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
};

static void t2048_task_fn(void *arg)
{
    (void)arg;
    t48_game_init();
    a = da = 0.0f;
    build_background();
    render();
    while (!s_t2048_stop) {
        t48_screen_tick();
        vTaskDelay(pdMS_TO_TICKS(12));
    }
    t48_game_deinit();
    s_t2048_task = NULL;
    vTaskDelete(NULL);
}

void t2048_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    (void)ev;
}

void t2048_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_t2048_stop = false;
    if (!s_t2048_task) {
        xTaskCreatePinnedToCore(t2048_task_fn, "t2048", 4096, NULL, 5, &s_t2048_task, APP_TASK_CORE);
    }
}

void t2048_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    s_t2048_stop = true;
    for (int i = 0; i < 50 && s_t2048_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_t2048_task = NULL;
}

void t2048_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)x;
    (void)y;
    (void)w;
    (void)h;
    if (!fb) return;
    memcpy(fb, g_fb, sizeof(g_fb));
}
