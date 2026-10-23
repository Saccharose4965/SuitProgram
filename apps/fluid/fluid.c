#include "app_shell.h"
#include "fluid_service.h"
#include "oled.h"

#include <stdint.h>
#include <string.h>

static uint8_t s_mask[FLUID_SIM_PIXELS];

const shell_legend_t FLUID_LEGEND = {
    .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE },
};

static inline void pixel_set(uint8_t *fb, int x, int y)
{
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int index = y * PANEL_W + x;
    fb[index >> 3] |= (uint8_t)(1u << (7 - (index & 7)));
}

static bool fit_simulation(int x, int y, int w, int h,
                           int *out_x, int *out_y, int *out_scale)
{
    int scale_x = w / FLUID_SIM_WIDTH;
    int scale_y = h / FLUID_SIM_HEIGHT;
    int scale = (scale_x < scale_y) ? scale_x : scale_y;
    if (scale <= 0) return false;

    *out_x = x + (w - FLUID_SIM_WIDTH * scale) / 2;
    *out_y = y + (h - FLUID_SIM_HEIGHT * scale) / 2;
    *out_scale = scale;
    return true;
}

static void draw_mask(uint8_t *fb, int origin_x, int origin_y, int scale)
{
    for (int y = 0; y < FLUID_SIM_HEIGHT; ++y) {
        for (int x = 0; x < FLUID_SIM_WIDTH; ++x) {
            if (!s_mask[x + y * FLUID_SIM_WIDTH]) continue;
            int pixel_x = origin_x + x * scale;
            int pixel_y = origin_y + y * scale;
            for (int dy = 0; dy < scale; ++dy) {
                for (int dx = 0; dx < scale; ++dx) {
                    pixel_set(fb, pixel_x + dx, pixel_y + dy);
                }
            }
        }
    }
}

void fluid_app_init_wrapper(shell_app_context_t *ctx)
{
    (void)ctx;
    fluid_service_start();
}

void fluid_app_deinit_wrapper(shell_app_context_t *ctx)
{
    (void)ctx;
    fluid_service_stop();
}

void fluid_app_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    (void)ev;
}

void fluid_app_tick_wrapper(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    (void)dt_sec;
    fluid_service_tick_fallback();
}

void fluid_app_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb,
                            int x, int y, int w, int h)
{
    (void)ctx;
    if (!fb) return;

    memset(fb, 0, PANEL_W * PANEL_H / 8);
    fluid_service_copy_mask(s_mask);

    int origin_x;
    int origin_y;
    int scale;
    if (fit_simulation(x, y, w, h, &origin_x, &origin_y, &scale)) {
        draw_mask(fb, origin_x, origin_y, scale);
    }
}
