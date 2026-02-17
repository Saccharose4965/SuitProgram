#include "app_shell.h"

#include "calculator.h"

const shell_legend_t CALCULATOR_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_DOWN, SHELL_ICON_RIGHT, SHELL_ICON_OK },
};

void calculator_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    calculator_init();
}

void calculator_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    calculator_handle_input(ev);
}

void calculator_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    calculator_draw(fb, x, y, w, h);
}
