#include "app_shell.h"

#include "snake.h"

const shell_legend_t SNAKE_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
};

void snake_app_init_wrapper(shell_app_context_t *ctx)
{
    if (!ctx) return;
    snake_set_request_switch(ctx->request_switch, ctx->request_user_data);
    snake_app_init();
}

void snake_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    snake_app_handle_input(ev);
}

void snake_tick_wrapper(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    snake_app_tick(dt_sec);
}

void snake_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    snake_app_draw(fb, x, y, w, h);
}
