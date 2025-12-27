#include "app_shell.h"
#include "shell_orientation.h"

#include "threedee.h"

const shell_legend_t THREEDEE_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_OK },
};

void threedee_app_init_wrapper(shell_app_context_t *ctx)
{
    if (!ctx) return;
    threedee_set_request_switch(ctx->request_switch, ctx->request_user_data);
    imu_orientation_t *ori = shell_orientation_ctx();
    if (ori) {
        threedee_set_orientation_ctx(ori);
    }
    threedee_app_init();
}

void threedee_app_deinit_wrapper(shell_app_context_t *ctx)
{
    (void)ctx;
    threedee_app_deinit();
}

void threedee_handle_input_wrapper(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    threedee_app_handle_input(ev);
}

void threedee_tick_wrapper(shell_app_context_t *ctx, float dt_sec)
{
    (void)ctx;
    threedee_app_tick(dt_sec);
}

void threedee_draw_wrapper(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    threedee_app_draw(fb, x, y, w, h);
}
