#include "call_app.h"

#include <stdio.h>

#include "call_service.h"
#include "oled.h"
#include "ui_draw.h"

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
    (void)w;
    (void)h;
    call_service_status_t status = call_service_get_status();
    char line[48];
    oled_draw_text3x5(fb, x + 2, y + 2, "CALL - DIRECT SUIT");
    oled_draw_text3x5(fb, x + 2, y + 12,
        status.active ? (status.talking ? "A: stop talking" : "A: talk / listening")
                      : "Call stopped");
    snprintf(line, sizeof(line), "tx:%lu rx:%lu",
             (unsigned long)status.sent, (unsigned long)status.received);
    oled_draw_text3x5(fb, x + 2, y + 22, line);
    oled_draw_text3x5(fb, x + 2, y + 32, esp_err_to_name(status.error));
}
