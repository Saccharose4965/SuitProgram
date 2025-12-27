#include "app_shell.h"

#include "pong.h"

const shell_legend_t PONG_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_OK },
};

void pong_app_init_wrapper(shell_app_context_t *ctx)
{
    if (!ctx) return;
    pong_set_request_switch(ctx->request_switch, ctx->request_user_data);
    pong_app_init();
}

void pong_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    pong_app_handle_input(ev);
}

void pong_tick_wrapper(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    pong_app_tick(dt_sec);
}

void pong_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    pong_app_draw(fb, x, y, w, h);
}
