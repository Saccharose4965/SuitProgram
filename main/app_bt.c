#include "app_shell.h"
#include "bt_audio_shell.h"

const shell_legend_t BT_AUDIO_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_CUSTOM1, SHELL_ICON_CUSTOM2 },
};

void bt_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    bt_audio_shell_init();
}

void bt_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    bt_audio_shell_handle_input(ev);
}

void bt_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    bt_audio_shell_draw(fb, x, y, w, h);
}
