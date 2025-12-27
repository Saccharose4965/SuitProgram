#include "twenty48.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"

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
static uint8_t g_bg[PANEL_W * PANEL_H / 8]; // static background (grid + arrows)
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

static void blit_arrow(const uint8_t *rows, int size, int x, int y){
    for (int dy = 0; dy < size; ++dy){
        int yy = y + dy;
        if ((unsigned)yy >= PANEL_H) continue;
        uint8_t bits = rows[dy];
        for (int dx = 0; dx < size; ++dx){
            int xx = x + dx;
            if ((unsigned)xx >= PANEL_W) continue;
            if (bits & (1u << (size - 1 - dx))) fb_pset(xx, yy);
        }
    }
}

// -------------------------------------------------------------------
// Game state (mirrors 2048.txt flow, but stores actual tile values)
// -------------------------------------------------------------------
static int grid_old[4][4]; // stores exponent (1=2, 2=4, etc.)
static int grid_new[4][4]; // stores exponent (same as above)
static int motion[4][4];
static int d = 0;          // 0=right,1=down,2=left,3=up
static float a = 0.0f;     // animation phase
static float da = 0.0f;    // animation increment
static uint32_t g_score = 0;
static uint32_t g_high_score = 0;

// mapcoord from the Processing sketch
static inline int mapcoord(int dir, int i, int j){
    int m = dir & 3;
    if (m == 1 || m == 3) return i;
    return (m == 0) ? (3 - j) : j;
}

static void copy_grid(int dst[4][4], int src[4][4]){
    memcpy(dst, src, sizeof(int) * 16);
}

static bool equals_grid(int aG[4][4], int bG[4][4]){
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (aG[i][j] != bG[i][j]) return false;
    return true;
}

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
    // Arrows along the bottom with small margins (A=left, B=up, C=right, D=down)
    const int margin = 6;
    const int spacing = (PANEL_W - 2 * margin - (T48_ARROW_SIZE * T48_ARROW_COUNT)) / (T48_ARROW_COUNT - 1);
    int arrow_y = PANEL_H - T48_ARROW_SIZE - 1;
    int x = margin;
    const int order[4] = {3, 0, 1, 2}; // indices into t48_arrows_5 (up,right,down,left)
    for (int i = 0; i < T48_ARROW_COUNT; ++i){
        blit_arrow(t48_arrows_5[order[i]], T48_ARROW_SIZE, x, arrow_y);
        x += T48_ARROW_SIZE + spacing;
    }
    memcpy(g_bg, g_fb, sizeof(g_fb));
}

static void slide(void){
    memset(motion, 0, sizeof(motion));
    memset(grid_new, 0, sizeof(grid_new));
    for (int line = 0; line < 4; ++line){
        int shift = 0, lastValue = 0;
        for (int pos = 0; pos < 4; ++pos){
            int col = mapcoord(d, line, pos);
            int row = mapcoord(d + 3, line, pos);
            int value = grid_old[row][col]; // exponent form
            if (value == 0){
                shift++;
                continue;
            }
            int targetPos = pos - shift;                    // target index after sliding
            bool merge = (lastValue == value);              // can we merge?
            motion[row][col] += shift + (merge ? 1 : 0);    // distance moved (+1 if merge)
            int mergeOffset = merge ? 1 : 0;                // merge lands one earlier
            int tcol = mapcoord(d, line, targetPos - mergeOffset);
            int trow = mapcoord(d + 3, line, targetPos - mergeOffset);
            grid_new[trow][tcol] += merge ? 1 : value;       // merged cell (increment exponent) or placed exponent
            if (merge && value < 31){
                uint32_t inc = (1u << (value + 1));
                g_score += inc;
                if (g_score > g_high_score) g_high_score = g_score;
            }
            lastValue = merge ? 0 : value;                   // reset after merge
            shift += merge ? 1 : 0;                          // merged cell opens a gap
        }
    }
}

static void addtile(void){
    int tile = (esp_random() % 10 == 0) ? 2 : 1; // exponent: 1=2, 2=4
    int c = 0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (grid_new[i][j] == 0) c++;
    if (c == 0) return;
    int r = (int)(esp_random() % (uint32_t)c);
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            if (grid_new[i][j] != 0) continue;
            if (r == 0){
                grid_new[i][j] = tile;
                return;
            }
            r--;
        }
    }
}

// -------------------------------------------------------------------
// Rendering (uses oldgrid + motion during animation)
// -------------------------------------------------------------------
static void render(void){
    memcpy(g_fb, g_bg, sizeof(g_fb));

    // Draw tiles (during animation: from old grid with motion; after: grid_old already updated)
    float ca = cosf(a);
    float offset_scale = (ca - 1.0f) * 0.5f; // matches (cos(a)-1)/2

    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            int exp = grid_old[i][j];
            if (exp <= 0) continue;
            int sprite = exp - 1;
            if (sprite < 0 || sprite >= T48_TILE_COUNT) continue;
            
            int base_x = kBoardX0 + j * T48_STRIDE;
            int base_y = kBoardY0 + i * T48_STRIDE;
            // motion is positive along dir; apply direction factors like the sketch
            float dx = 0.0f, dy = 0.0f;
            if ((d & 1) == 0){ // horizontal
                dx = offset_scale * (float)(d - 1) * (float)motion[i][j] * (float)T48_STRIDE;
            } else {           // vertical
                dy = offset_scale * (float)(d - 2) * (float)motion[i][j] * (float)T48_STRIDE;
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
    snprintf(buf, sizeof(buf), " %u", (unsigned)g_score);
    oled_draw_text3x5(g_fb, 2, 8, buf);
    oled_draw_text3x5(g_fb, 2, 14, "BEST:");
    snprintf(buf, sizeof(buf), " %u", (unsigned)g_high_score);
    oled_draw_text3x5(g_fb, 2, 20, buf);

    oled_blit_full(g_fb);
}

// -------------------------------------------------------------------
// Public API (init + per-tick)
// -------------------------------------------------------------------
void t48_game_init(void){
    memset(grid_old, 0, sizeof(grid_old));
    memset(grid_new, 0, sizeof(grid_new));
    memset(motion, 0, sizeof(motion));
    g_score = 0;
    a = 0.0f;
    da = 0.0f;
    addtile();
    addtile();
    copy_grid(grid_old, grid_new);
    build_background();
    render();
}

void t48_game_tick(void){
    // Button handling: edges only
    static hw_button_id_t last = HW_BTN_NONE;
    hw_button_id_t b = HW_BTN_NONE;
    if (hw_buttons_read(&b) != ESP_OK) b = HW_BTN_NONE;

    if (last == HW_BTN_NONE && b != HW_BTN_NONE && a == 0.0f){
        // Map buttons: A=left, B=up, C=right, D=down
        if      (b == HW_BTN_A) d = 2;
        else if (b == HW_BTN_B) d = 3;
        else if (b == HW_BTN_C) d = 0;
        else if (b == HW_BTN_D) d = 1;
        else d = 0;

        slide();
        if (!equals_grid(grid_old, grid_new)){
            addtile();
            da = 0.2f;
        } else {
            da = 0.0f;
        }
    }
    last = b;

    // Animate
    if (da != 0.0f){
        a += da;
        render();
        if (a >= (float)M_PI){
            copy_grid(grid_old, grid_new);
            a = 0.0f;
            da = 0.0f;
            render();
        }
    }
}
