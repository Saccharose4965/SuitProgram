#include "synth_app.h"
#include "synth_service.h"
#include "ui_draw.h"
#include "oled.h"
#include <stdio.h>

void synth_init(shell_app_context_t *ctx)
{
    (void)ctx;
    synth_service_open();
}

void synth_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    synth_service_close();
}

void synth_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;
    if (ev->button == INPUT_BTN_A && ctx && ctx->request_switch) {
        ctx->request_switch("menu_sim", ctx->request_user_data);
    } else if (ev->button == INPUT_BTN_B) {
        synth_service_calibrate();
    } else if (ev->button == INPUT_BTN_D) {
        synth_service_toggle_mute();
    }
}

void synth_tick(shell_app_context_t *ctx, float dt)
{
    (void)ctx;
    (void)dt;
    synth_service_tick();
}

void synth_draw(shell_app_context_t *ctx, uint8_t *fb,
                int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    (void)h;
    synth_status_t status = synth_service_status();
    char line[40];
    oled_draw_text3x5(fb, x + 2, y + 2,
                      status.bluetooth ? "Arm Synth / Bluetooth" :
                                         "Arm Synth / speaker");
    if (status.error != ESP_OK) {
        oled_draw_text3x5(fb, x + 2, y + 14, esp_err_to_name(status.error));
        oled_draw_text3x5(fb, x + 2, y + 24, "Audio busy? Exit/reopen");
    } else {
        snprintf(line, sizeof(line), "%.0f Hz / %.0f%%",
                 status.hz, status.gain * 100);
        oled_draw_text3x5(fb, x + 2, y + 14,
                          status.calibrated ? line : "Fold arm; B calibrates");
        oled_draw_text3x5(fb, x + 2, y + 24,
                          status.muted ? "D: unmute" : "D: mute");
        oled_draw_text3x5(fb, x + 2, y + 34,
                          "Up: pitch / side: volume");
    }
    oled_draw_text3x5(fb, x + 2, y + 44, "A back / B calibrate");
}
