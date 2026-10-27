#include "app_shell.h"
#include <stdio.h>

#include "link.h"

void status_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    (void)ev;
}

void status_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    if (!ctx || !ctx->state) return;

    char led[32], battery[32], network[32], time[32];
    snprintf(led, sizeof(led), "LED: %s", ctx->state->led_mode_name);

    snprintf(battery, sizeof(battery), "BAT: %u|%u|%u",
             (unsigned)ctx->state->battery_pct[0],
             (unsigned)ctx->state->battery_pct[1],
             (unsigned)ctx->state->battery_pct[2]);

    const char *path = link_path_name(link_active_path());
    snprintf(network, sizeof(network), "NET: %s (%s)",
             system_state_connection_str(ctx->state->connection),
             path ? path : "?");

    snprintf(time, sizeof(time), "TIME: %02u:%02u%s",
             (unsigned)ctx->state->hours, (unsigned)ctx->state->minutes,
             ctx->state->time_valid ? "" : " (?)");
    const char *lines[] = { led, battery, network, time };
    const shell_text_page_t page = {
        .title = "STATUS", .lines = lines, .line_count = 4,
    };
    shell_ui_draw_text_page(fb, x, y, w, h, &page);
}

const shell_legend_t STATUS_LEGEND = {
    .slots = { SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_NONE },
};
