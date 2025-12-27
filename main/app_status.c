#include "app_shell.h"

#include <stdio.h>

#include "link.h"

void status_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        ctx->request_switch("menu", ctx->request_user_data);
    }
}

void status_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)w; (void)h;
    if (!ctx || !ctx->state) return;

    char line[32];
    snprintf(line, sizeof(line), "LED: %s", ctx->state->led_mode_name);
    oled_draw_text3x5(fb, x + 2, y + 2, line);

    snprintf(line, sizeof(line), "BAT: %u|%u|%u",
             (unsigned)ctx->state->battery_pct[0],
             (unsigned)ctx->state->battery_pct[1],
             (unsigned)ctx->state->battery_pct[2]);
    oled_draw_text3x5(fb, x + 2, y + 9, line);

    const char *path = link_path_name(link_active_path());
    snprintf(line, sizeof(line), "NET: %s (%s)",
             system_state_connection_str(ctx->state->connection),
             path ? path : "?");
    oled_draw_text3x5(fb, x + 2, y + 16, line);

    snprintf(line, sizeof(line), "TIME: %02u:%02u%s",
             (unsigned)ctx->state->hours, (unsigned)ctx->state->minutes,
             ctx->state->time_valid ? "" : " (?)");
    oled_draw_text3x5(fb, x + 2, y + 23, line);
}

const shell_legend_t STATUS_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_MENU },
};
