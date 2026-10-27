#pragma once

#include <stdbool.h>
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
    float scroll;
} shell_menu_motion_t;

typedef struct {
    float column;
} shell_grid_motion_t;

typedef struct {
    const shell_menu_entry_t *entries;
    // Optional array of count pointers to 9-row, 9-bit monochrome sprites.
    // A NULL array or entry uses the shared placeholder until artwork exists.
    const uint16_t *const *sprites;
    size_t count;
    size_t selected;
    const char *title;
    const shell_menu_motion_t *motion;
} shell_menu_view_t;

typedef struct {
    const shell_menu_entry_t *entries;
    const uint16_t *const *sprites;
    size_t count;
    size_t selected;
    const shell_grid_motion_t *motion;
} shell_grid_view_t;

typedef struct {
    const char *title;
    const char *line1;
    const char *line2;
    const char *prompt;
} shell_overlay_view_t;

typedef struct {
    const char *title;
    const char *const *lines;
    size_t line_count;
} shell_text_page_t;

enum {
    SHELL_UI_CAPTURE_MAX_ENTRIES = 9,
    SHELL_UI_CAPTURE_LABEL_LEN = 16,
    SHELL_UI_CAPTURE_TITLE_LEN = 16,
};

typedef enum {
    SHELL_UI_CAPTURE_NONE = 0,
    SHELL_UI_CAPTURE_MENU,
    SHELL_UI_CAPTURE_GRID,
} shell_ui_capture_kind_t;

typedef struct {
    shell_ui_capture_kind_t kind;
    uint8_t total_count;
    uint8_t selected;
    uint8_t first;
    uint8_t entry_count;
    char title[SHELL_UI_CAPTURE_TITLE_LEN];
    char labels[SHELL_UI_CAPTURE_MAX_ENTRIES][SHELL_UI_CAPTURE_LABEL_LEN];
    shell_legend_t legend;
} shell_ui_capture_t;

// Draw the HUD band with a left-aligned label and right-aligned runtime status.
void shell_ui_draw_hud(uint8_t *fb, const system_state_t *state, const char *left_label);

// Draw the bottom legend row of 5x5 icons.
void shell_ui_draw_legend(uint8_t *fb, const shell_legend_t *legend);

// Configure which 5x5 direction icon family is used for UP/DOWN/LEFT/RIGHT.
void shell_ui_set_direction_icon_style(shell_direction_icon_style_t style);
shell_direction_icon_style_t shell_ui_get_direction_icon_style(void);

// Per-screen smooth scrolling support for three-item choice lists.
void shell_ui_menu_reset(shell_menu_motion_t *motion, size_t selected);
void shell_ui_menu_tick(shell_menu_motion_t *motion, float dt_sec, size_t selected);
void shell_ui_draw_menu(uint8_t *fb, int x, int y, int w, int h, const shell_menu_view_t *view);

// Three-row carousel grid. The active column is centred and neighbouring
// columns remain partially visible; selection within a column is vertical.
void shell_ui_grid_reset(shell_grid_motion_t *motion, size_t selected);
void shell_ui_grid_tick(shell_grid_motion_t *motion, float dt_sec, size_t selected);
void shell_ui_draw_grid(uint8_t *fb, int x, int y, int w, int h, const shell_grid_view_t *view);

// Draw a self-contained modal card over existing app content.
void shell_ui_draw_overlay(uint8_t *fb, int x, int y, int w, int h,
                           const shell_overlay_view_t *view);

// Consistent clipped title and 3x5 status lines for simple information screens.
void shell_ui_draw_text_page(uint8_t *fb, int x, int y, int w, int h,
                             const shell_text_page_t *page);

// One semantic snapshot of the standard UI drawn during the current frame.
// Remote Control uses this to reproduce menus without running their app locally.
void shell_ui_capture_begin(void);
void shell_ui_capture_get(shell_ui_capture_t *out);

// Small mode cue that preserves the content inside the display borders.
void shell_ui_draw_remote_corners(uint8_t *fb, bool controller);

#ifdef __cplusplus
}
#endif
