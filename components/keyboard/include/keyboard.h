#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    KEYBOARD_COLS = 10,
    KEYBOARD_ROWS = 5,
    KEYBOARD_KEY_COUNT = KEYBOARD_COLS * KEYBOARD_ROWS,
};

typedef struct {
    float   key_px;           // size of one key in pixels (defaults to 12)
    float   long_press_sec;   // hold time before accents/backspace repeat (defaults to 0.4 s)
    float   repeat_rate_hz;   // repeat rate for held backspace (defaults to 10 Hz)
    int     origin_x;         // top-left of keyboard; defaults to 0
    int     origin_y;         // defaults to bottom-aligned with key_px rows
    char   *text_buffer;      // optional external buffer
    size_t  text_capacity;    // capacity of external buffer (excludes terminator)
} keyboard_config_t;

typedef struct {
    float x;
    float y;
    bool  pressed;
    float dt_sec;
} keyboard_pointer_event_t;

#define KEYBOARD_INLINE_TEXT_CAP 96

typedef struct {
    // public-ish state
    bool   visible;
    bool   shift;
    bool   shift_lock;
    bool   symbol_page;

    // input tracking
    bool   pointer_down;
    int    active_idx;
    int    hot_idx;
    float  hold_time;
    float  repeat_accum;

    // popup accents
    bool   popup_active;
    int    popup_idx;
    int    popup_selection;

    // config-derived
    float  key_px;
    float  long_press_sec;
    float  repeat_interval_sec;
    int    origin_x;
    int    origin_y;

    // text buffer
    char  *text;
    size_t text_len;
    size_t text_cap;
    char   text_inline[KEYBOARD_INLINE_TEXT_CAP + 1];
} keyboard_state_t;

void keyboard_init(keyboard_state_t *kb, const keyboard_config_t *cfg);
void keyboard_set_origin(keyboard_state_t *kb, int x, int y);
void keyboard_show(keyboard_state_t *kb, bool show);
bool keyboard_is_visible(const keyboard_state_t *kb);

void keyboard_clear_text(keyboard_state_t *kb);
void keyboard_set_text(keyboard_state_t *kb, const char *text);
const char *keyboard_get_text(const keyboard_state_t *kb);
size_t keyboard_get_length(const keyboard_state_t *kb);

// Returns true if the text buffer changed.
bool keyboard_handle_pointer(keyboard_state_t *kb, const keyboard_pointer_event_t *ev);

// Deletes the last character (if any). Returns true on change.
bool keyboard_backspace(keyboard_state_t *kb);

// Draws the keyboard into a 1bpp framebuffer (same layout as oled_blit_full()).
void keyboard_draw(const keyboard_state_t *kb, uint8_t *fb);

#ifdef __cplusplus
}
#endif
