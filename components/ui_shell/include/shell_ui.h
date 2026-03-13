#pragma once

#include <stddef.h>
#include <stdint.h>

#include "oled.h"
#include "system_state.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SHELL_UI_HUD_HEIGHT    = 8,
    SHELL_UI_LEGEND_HEIGHT = 7,
};

typedef enum {
    SHELL_ICON_NONE = 0,
    SHELL_ICON_UP,
    SHELL_ICON_DOWN,
    SHELL_ICON_LEFT,
    SHELL_ICON_RIGHT,
    SHELL_ICON_SELECT,
    SHELL_ICON_REFRESH,
    SHELL_ICON_MUTE,
    SHELL_ICON_PLAY,
    SHELL_ICON_PAUSE,
    SHELL_ICON_NOTE,
    SHELL_ICON_LOCK,
    SHELL_ICON_RECORD,
    SHELL_ICON_STOP,
} shell_icon_id_t;

typedef enum {
    SHELL_DIRECTION_ICON_STYLE_SHELL = 0,
    SHELL_DIRECTION_ICON_STYLE_2048,
} shell_direction_icon_style_t;

typedef struct {
    shell_icon_id_t slots[4]; // A,B,C,D
} shell_legend_t;

typedef struct {
    const char *id;
    const char *label;
} shell_menu_entry_t;

typedef struct {
    const shell_menu_entry_t *entries;
    size_t count;
    size_t selected;
    const char *title;
} shell_menu_view_t;

// Draw the HUD band (time, batteries, connection) with a left-aligned label.
void shell_ui_draw_hud(uint8_t *fb, const system_state_t *state, const char *left_label);

// Draw the bottom legend row of 5x5 icons.
void shell_ui_draw_legend(uint8_t *fb, const shell_legend_t *legend);

// Configure which 5x5 direction icon family is used for UP/DOWN/LEFT/RIGHT.
void shell_ui_set_direction_icon_style(shell_direction_icon_style_t style);
shell_direction_icon_style_t shell_ui_get_direction_icon_style(void);

// Smooth scrolling support for menu lists.
void shell_ui_menu_reset(size_t selected);
void shell_ui_menu_tick(float dt_sec, size_t selected);
void shell_ui_draw_menu(uint8_t *fb, int x, int y, int w, int h, const shell_menu_view_t *view);

#ifdef __cplusplus
}
#endif
