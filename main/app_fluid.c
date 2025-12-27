#include "app_shell.h"

#include "fluid.h"

const shell_legend_t FLUID_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
};

void fluid_app_init_wrapper(shell_app_context_t *ctx)
{
    (void)ctx;
    fluid_app_init();
}

void fluid_app_deinit_wrapper(shell_app_context_t *ctx)
{
    (void)ctx;
    fluid_app_deinit();
}

void fluid_app_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (ev && ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        if (ctx && ctx->request_switch) {
            ctx->request_switch("menu", ctx->request_user_data);
        }
        return;
    }
    (void)ctx;
    fluid_app_handle_input(ev);
}

void fluid_app_tick_wrapper(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    fluid_app_tick(dt_sec);
}

void fluid_app_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    fluid_app_draw(fb, x, y, w, h);
}
