#include "settings_apps.h"
#include "ui_draw.h"

#include "settings.h"
#include "oled.h"

typedef struct {
    bool use_2048_direction_icons;
    size_t selected;
    shell_menu_motion_t motion;
} preferences_state_t;

static preferences_state_t s_preferences = {
    .use_2048_direction_icons = false,
};

const shell_legend_t PREFERENCES_LEGEND = {
    .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_NONE, SHELL_ICON_SELECT },
};

static void preferences_apply(bool use_2048_direction_icons)
{
    s_preferences.use_2048_direction_icons = use_2048_direction_icons;
    settings_set_direction_icons(use_2048_direction_icons, true);
}

void preferences_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    const settings_t *settings = settings_get();
    s_preferences.use_2048_direction_icons =
        settings ? settings->use_2048_direction_icons : false;
    s_preferences.selected=s_preferences.use_2048_direction_icons?1u:0u;
    shell_ui_menu_reset(&s_preferences.motion,s_preferences.selected);
}

void preferences_app_tick(shell_app_context_t *ctx,float dt_sec)
{
    (void)ctx;
    shell_ui_menu_tick(&s_preferences.motion,dt_sec,s_preferences.selected);
}

void preferences_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if(ev->button==INPUT_BTN_A && s_preferences.selected>0) --s_preferences.selected;
    else if(ev->button==INPUT_BTN_B && s_preferences.selected<1) ++s_preferences.selected;
    else if(ev->button==INPUT_BTN_D) preferences_apply(s_preferences.selected==1);
}

void preferences_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx;
    (void)w;
    (void)h;
    if (!fb) return;

    const shell_menu_entry_t entries[]={
        {.id="shell",.label=s_preferences.use_2048_direction_icons?"Shell":"Shell *"},
        {.id="2048",.label=s_preferences.use_2048_direction_icons?"2048 *":"2048"},
    };
    shell_menu_view_t view={
        .entries=entries,.count=2,.selected=s_preferences.selected,
        .title="DIRECTION ICONS",.motion=&s_preferences.motion,
    };
    shell_ui_draw_menu(fb,x,y,w,h,&view);
}
