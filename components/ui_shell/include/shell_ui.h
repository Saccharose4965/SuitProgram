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
    SHELL_ICON_BACK,
    SHELL_ICON_UP,
    SHELL_ICON_DOWN,
    SHELL_ICON_LEFT,
    SHELL_ICON_RIGHT,
    SHELL_ICON_OK,
    SHELL_ICON_MENU,
    SHELL_ICON_LINK,
    SHELL_ICON_CUSTOM1,
    SHELL_ICON_CUSTOM2,
    SHELL_ICON_MUTE,
} shell_icon_id_t;

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

// Smooth scrolling support for menu lists.
void shell_ui_menu_reset(size_t selected);
void shell_ui_menu_tick(float dt_sec, size_t selected);
void shell_ui_draw_menu(uint8_t *fb, int x, int y, int w, int h, const shell_menu_view_t *view);

#ifdef __cplusplus
}
#endif
