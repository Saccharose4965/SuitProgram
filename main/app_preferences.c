#include "app_shell.h"

#include "app_settings.h"
#include "oled.h"

typedef struct {
    bool use_2048_direction_icons;
} preferences_state_t;

static preferences_state_t s_preferences = {
    .use_2048_direction_icons = false,
};

const shell_legend_t PREFERENCES_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_SELECT, SHELL_ICON_NONE },
};

static void preferences_apply(bool use_2048_direction_icons)
{
    s_preferences.use_2048_direction_icons = use_2048_direction_icons;
    app_settings_set_direction_icons(use_2048_direction_icons, true);
}

void preferences_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    const app_settings_t *settings = app_settings_get();
    s_preferences.use_2048_direction_icons =
        settings ? settings->use_2048_direction_icons : false;
}

void preferences_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        preferences_apply(false);
    } else if (ev->button == INPUT_BTN_B) {
        preferences_apply(true);
    } else if (ev->button == INPUT_BTN_C) {
        preferences_apply(!s_preferences.use_2048_direction_icons);
    }
}

void preferences_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    (void)h;
    if (!fb) return;

    oled_draw_text3x5(fb, x + 2, y + 2, "PREFERENCES");
    oled_draw_text3x5(fb, x + 2, y + 12, "DIR ICONS");
    oled_draw_text3x5(fb, x + 2, y + 24,
                      s_preferences.use_2048_direction_icons ? " SHELL" : ">SHELL");
    oled_draw_text3x5(fb, x + 2, y + 32,
                      s_preferences.use_2048_direction_icons ? ">2048" : " 2048");
    oled_draw_text3x5(fb, x + 2, y + 44, "GLOBAL LEGEND");
}
