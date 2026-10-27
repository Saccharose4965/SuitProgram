#include "call_app.h"

#include <stdio.h>

#include "call_service.h"
void call_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    (void)call_service_start();
}

void call_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    call_service_stop();
}

void call_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (ev && ev->type == INPUT_EVENT_PRESS && ev->button == INPUT_BTN_A)
        call_service_toggle_talk();
}

void call_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                   int x, int y, int w, int h)
{
    (void)ctx;
    call_service_status_t status = call_service_get_status();
    char traffic[48];
    snprintf(traffic, sizeof(traffic), "tx:%lu rx:%lu",
             (unsigned long)status.sent, (unsigned long)status.received);
    const char *lines[] = {
        status.active ? (status.talking ? "A: stop talking" : "A: talk / listening")
                      : "Call stopped",
        traffic,
        esp_err_to_name(status.error),
    };
    const shell_text_page_t page = {
        .title = "CALL - DIRECT SUIT", .lines = lines, .line_count = 3,
    };
    shell_ui_draw_text_page(fb, x, y, w, h, &page);
}
