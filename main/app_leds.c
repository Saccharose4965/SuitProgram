#include "app_shell.h"

#include <stdio.h>

#include "led_modes.h"

const shell_legend_t LEDS_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_OK },
};

typedef struct {
    int sel;
    bool sync;
    int brightness; // 0-255
} leds_state_t;
static leds_state_t s_leds = {0};

static void leds_apply_selection(void)
{
    led_modes_set(s_leds.sel);
    const char *name = led_modes_name(s_leds.sel);
    system_state_set_led_mode(s_leds.sel, name ? name : "led");
}

void leds_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_leds.sel = led_modes_current();
    s_leds.sync = led_modes_sync_enabled();
    s_leds.brightness = led_modes_get_brightness();
    led_modes_set_sync(s_leds.sync);
    led_modes_set_brightness((uint8_t)s_leds.brightness);
    leds_apply_selection();
}

void leds_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        ctx->request_switch("menu", ctx->request_user_data);
        return;
    }
    if (ev->type == INPUT_EVENT_PRESS) {
        if (ev->button == INPUT_BTN_B && s_leds.sel > 0) {
            s_leds.sel--;
        } else if (ev->button == INPUT_BTN_C && s_leds.sel + 1 < led_modes_count()) {
            s_leds.sel++;
        } else if (ev->button == INPUT_BTN_D) {
            leds_apply_selection();
        }
    } else if (ev->type == INPUT_EVENT_LONG_PRESS) {
        if (ev->button == INPUT_BTN_D) {
            s_leds.sync = !s_leds.sync;
            led_modes_set_sync(s_leds.sync);
        } else if (ev->button == INPUT_BTN_B) {
            if (s_leds.brightness < 255) s_leds.brightness += 15;
            if (s_leds.brightness > 255) s_leds.brightness = 255;
            led_modes_set_brightness((uint8_t)s_leds.brightness);
        } else if (ev->button == INPUT_BTN_C) {
            if (s_leds.brightness > 0) s_leds.brightness -= 15;
            if (s_leds.brightness < 0) s_leds.brightness = 0;
            led_modes_set_brightness((uint8_t)s_leds.brightness);
        }
    }
}

void leds_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)w; (void)h;
    oled_draw_text3x5(fb, x + 2, y + 2, "LED MODES");
    const int max_vis = 3;
    int count = led_modes_count();
    if (count <= 0) return;
    if (s_leds.sel < 0) s_leds.sel = 0;
    if (s_leds.sel >= count) s_leds.sel = count - 1;

    int start = 0;
    if (s_leds.sel >= max_vis) start = s_leds.sel - (max_vis - 1);
    int visible = count - start;
    if (visible > max_vis) visible = max_vis;
    for (int i = 0; i < visible; ++i){
        int idx = start + i;
        int yy = y + 10 + i * 8;
        char line[32];
        const char *name = led_modes_name(idx);
        snprintf(line, sizeof(line), "%c %s", (idx == s_leds.sel) ? '>' : ' ', name ? name : "?");
        oled_draw_text3x5(fb, x + 2, yy, line);
    }
    char status[32];
    snprintf(status, sizeof(status), "sync:%s bright:%d", s_leds.sync ? "on" : "off", s_leds.brightness);
    oled_draw_text3x5(fb, x + 2, y + 34, status);
    oled_draw_text3x5(fb, x + 2, y + 42, "B+/C- long=brightness");
}
