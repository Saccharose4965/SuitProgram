#include "app_keyboard.h"

#include <string.h>

#include "keyboard.h"
#include "oled.h"
#include "shell_ui.h"

const shell_legend_t KEYBOARD_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_DOWN, SHELL_ICON_RIGHT, SHELL_ICON_OK },
};

typedef struct {
    keyboard_state_t kb;
    int row;
    int col;
    bool pointer_down;
} keyboard_app_state_t;

static keyboard_app_state_t s_keyboard_app;

static inline void keyboard_clamp_focus(void)
{
    if (s_keyboard_app.row < 0) s_keyboard_app.row = KEYBOARD_ROWS - 1;
    if (s_keyboard_app.row >= KEYBOARD_ROWS) s_keyboard_app.row = 0;
    if (s_keyboard_app.col < 0) s_keyboard_app.col = KEYBOARD_COLS - 1;
    if (s_keyboard_app.col >= KEYBOARD_COLS) s_keyboard_app.col = 0;
}

static void keyboard_send_pointer(bool pressed, float dt_sec)
{
    float key_px = s_keyboard_app.kb.key_px;
    float x = s_keyboard_app.kb.origin_x + (s_keyboard_app.col + 0.5f) * key_px;
    float y = s_keyboard_app.kb.origin_y + (s_keyboard_app.row + 0.5f) * key_px;
    keyboard_pointer_event_t ev = {
        .x = x,
        .y = y,
        .pressed = pressed,
        .dt_sec = dt_sec,
    };
    (void)keyboard_handle_pointer(&s_keyboard_app.kb, &ev);
}

static void keyboard_move_focus(int drow, int dcol)
{
    // If we are on the spacebar and moving right, jump to the key after it.
    if (dcol > 0 && s_keyboard_app.row == (KEYBOARD_ROWS - 1)
        && s_keyboard_app.col >= 3 && s_keyboard_app.col <= 6) {
        s_keyboard_app.col = 7;
    } else {
        s_keyboard_app.row += drow;
        s_keyboard_app.col += dcol;
    }
    keyboard_clamp_focus();
    keyboard_send_pointer(s_keyboard_app.pointer_down, 0.0f);
}

void keyboard_app_init(struct shell_app_context *ctx)
{
    (void)ctx;
    memset(&s_keyboard_app, 0, sizeof(s_keyboard_app));
    const float key_px = 10.0f; // leave a few pixels at the top for text
    const int kb_w = (int)(key_px * KEYBOARD_COLS);
    const int kb_h = (int)(key_px * KEYBOARD_ROWS);
    int origin_x = (PANEL_W - kb_w) / 2;
    if (origin_x < 0) origin_x = 0;
    int origin_y = PANEL_H - SHELL_UI_LEGEND_HEIGHT - kb_h;
    if (origin_y < SHELL_UI_HUD_HEIGHT) origin_y = SHELL_UI_HUD_HEIGHT;
    keyboard_config_t cfg = {
        .key_px = key_px,
        .origin_x = origin_x,
        .origin_y = origin_y,
    };
    keyboard_init(&s_keyboard_app.kb, &cfg);
    s_keyboard_app.row = 0;
    s_keyboard_app.col = 0;
    keyboard_send_pointer(false, 0.0f);
}

void keyboard_app_handle_input(struct shell_app_context *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev) return;

    if (ev->type == INPUT_EVENT_PRESS) {
        switch (ev->button) {
            case INPUT_BTN_A: keyboard_move_focus(0, -1); break;   // left
            case INPUT_BTN_B: keyboard_move_focus(1, 0); break;    // down
            case INPUT_BTN_C: keyboard_move_focus(0, 1); break;    // right
            case INPUT_BTN_BC_COMBO: keyboard_move_focus(1, 0); break; // middle-two: down
            case INPUT_BTN_D:
                s_keyboard_app.pointer_down = true;
                keyboard_send_pointer(true, 0.0f);
                break;
            default: break;
        }
    } else if (ev->type == INPUT_EVENT_LONG_PRESS) {
        if (ev->button == INPUT_BTN_BC_COMBO) {
            (void)keyboard_backspace(&s_keyboard_app.kb);
        }
    } else if (ev->type == INPUT_EVENT_RELEASE) {
        if (ev->button == INPUT_BTN_D && s_keyboard_app.pointer_down) {
            keyboard_send_pointer(false, 0.0f);
            s_keyboard_app.pointer_down = false;
        }
    }
}

void keyboard_app_tick(struct shell_app_context *ctx, float dt_sec)
{
    (void)ctx;
    // Keep hot/click state updated for held presses (backspace repeat, accents).
    keyboard_send_pointer(s_keyboard_app.pointer_down, dt_sec);
    if (!keyboard_is_visible(&s_keyboard_app.kb)) {
        keyboard_show(&s_keyboard_app.kb, true);
    }
}

void keyboard_app_draw(struct shell_app_context *ctx, uint8_t *fb, int x, int y, int w, int h)
{
    (void)ctx; (void)x; (void)y; (void)w; (void)h;
    const char *text = keyboard_get_text(&s_keyboard_app.kb);
    if (text) {
        oled_draw_text3x5(fb, 0, 0, text);
    }
    oled_draw_text3x5(fb, 0, 8, "A< Bv C> D=sel");
    oled_draw_text3x5(fb, 0, 14, "B+C long=del");
    keyboard_draw(&s_keyboard_app.kb, fb);
}
