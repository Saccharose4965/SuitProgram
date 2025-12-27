#include "keyboard.h"

#include <math.h>
#include <string.h>

#include "oled.h"

#define KB_SHIFT_IDX       30
#define KB_BACKSPACE_IDX   39
#define KB_PAGE_IDX        40
#define KB_SPACE_START     43
#define KB_SPACE_END       46
#define KB_ENTER_IDX       49

#define KB_MAX_SPECIALS    6

typedef struct {
    const char *options[KB_MAX_SPECIALS];
    uint8_t count;
} key_specials_t;

static const char *kLayout[4][KEYBOARD_KEY_COUNT] = {
    { // lowercase letters / symbols
        "1","2","3","4","5","6","7","8","9","0",
        "a","z","e","r","t","y","u","i","o","p",
        "q","s","d","f","g","h","j","k","l","m",
        "","","w","x","c","v","b","n","\"","",
        "",",",""," "," "," "," ","",".",""
    },
    { // uppercase letters
        "1","2","3","4","5","6","7","8","9","0",
        "A","Z","E","R","T","Y","U","I","O","P",
        "Q","S","D","F","G","H","J","K","L","M",
        "","","W","X","C","V","B","N","\"","",
        "",",",""," "," "," "," ","",".",""
    },
    { // symbols page 1
        "1","2","3","4","5","6","7","8","9","0",
        "+","*","/","=","-","_","<",">","[","]",
        "!","@","#","$","%","^","&","*","(",")",
        "1/2","","-","'","\"",":",";",",","?","",
        "ABC",",",""," "," "," "," ","",".",""
    },
    { // symbols page 2
        "1","2","3","4","5","6","7","8","9","0",
        "~","`","\\","|","{","}","$","_",":",";",
        "!","?","^","%","+","-","#","@","<",">",
        "2/2","","","","=",".",",","!","?","",
        "ABC",",",""," "," "," "," ","",".",""
    },
};

static const key_specials_t kSpecials[4][KEYBOARD_KEY_COUNT] = {
    { // lowercase
        [10] = { .options = {"a","à","â","ä"}, .count = 4 },
        [12] = { .options = {"e","é","è","ê","ë"}, .count = 5 },
        [16] = { .options = {"u","ù","û","ü"}, .count = 4 },
        [17] = { .options = {"i","î","ï"}, .count = 3 },
        [18] = { .options = {"o","ô","ö"}, .count = 3 },
        [21] = { .options = {"s","ç"}, .count = 2 },
    },
    { // uppercase
        [10] = { .options = {"A","À","Â","Ä"}, .count = 4 },
        [12] = { .options = {"E","É","È","Ê","Ë"}, .count = 5 },
        [16] = { .options = {"U","Ù","Û","Ü"}, .count = 4 },
        [17] = { .options = {"I","Î","Ï"}, .count = 3 },
        [18] = { .options = {"O","Ô","Ö"}, .count = 3 },
        [21] = { .options = {"S","Ç"}, .count = 2 },
    },
    { // symbols page 1
    },
    { // symbols page 2
    },
};

// ---------------------------------------------------------------------------
// Framebuffer helpers
// ---------------------------------------------------------------------------

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if (!fb) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static inline void fb_ptoggle(uint8_t *fb, int x, int y)
{
    if (!fb) return;
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] ^= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_fill(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            fb_pset(fb, x, y);
        }
    }
}

static void fb_rect_invert(uint8_t *fb, int x0, int y0, int w, int h)
{
    if (w <= 0 || h <= 0) return;
    int x1 = x0 + w - 1;
    int y1 = y0 + h - 1;
    if (x1 < 0 || y1 < 0 || x0 >= PANEL_W || y0 >= PANEL_H) return;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            fb_ptoggle(fb, x, y);
        }
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int layout_mode(const keyboard_state_t *kb)
{
    int page_offset = kb->symbol_page ? 2 : 0;
    bool shift_active = kb->shift || (kb->shift_lock && !kb->symbol_page);
    return page_offset + (shift_active ? 1 : 0);
}

static const key_specials_t *get_specials(int mode, int idx)
{
    if (mode < 0 || mode > 3 || idx < 0 || idx >= KEYBOARD_KEY_COUNT) return NULL;
    return &kSpecials[mode][idx];
}

static const char *key_label(const keyboard_state_t *kb, int mode, int idx)
{
    (void)mode;
    if (idx == KB_SHIFT_IDX) {
        if (kb->symbol_page) {
            return kb->shift ? "2/2" : "1/2";
        }
        return kb->shift_lock ? "CAP" : "^";
    }
    if (idx == KB_BACKSPACE_IDX) return "<";
    if (idx == KB_PAGE_IDX) return kb->symbol_page ? "ABC" : "!#1";
    if (idx >= KB_SPACE_START && idx <= KB_SPACE_END) return "SP";
    if (idx == KB_ENTER_IDX) return "ENT";

    const char *l = kLayout[mode][idx];
    if (!l) return "";
    if (strcmp(l, "enter") == 0) return "ENT";
    return l;
}

static void set_text_ptr(keyboard_state_t *kb, const keyboard_config_t *cfg)
{
    kb->text = kb->text_inline;
    kb->text_cap = KEYBOARD_INLINE_TEXT_CAP;
    if (cfg && cfg->text_buffer && cfg->text_capacity > 0) {
        kb->text = cfg->text_buffer;
        kb->text_cap = cfg->text_capacity;
    }
    kb->text_len = 0;
    if (kb->text && kb->text_cap > 0) {
        kb->text[0] = '\0';
    }
}

static bool append_text(keyboard_state_t *kb, const char *glyph)
{
    if (!kb || !glyph || !glyph[0]) return false;
    size_t glen = strlen(glyph);
    if (glen == 0 || kb->text_len >= kb->text_cap) return false;
    if (kb->text_len + glen > kb->text_cap) {
        glen = kb->text_cap - kb->text_len;
    }
    memcpy(kb->text + kb->text_len, glyph, glen);
    kb->text_len += glen;
    kb->text[kb->text_len] = '\0';
    return true;
}

bool keyboard_backspace(keyboard_state_t *kb)
{
    if (!kb || !kb->text || kb->text_len == 0) return false;
    kb->text_len--;
    kb->text[kb->text_len] = '\0';
    kb->shift = false;
    return true;
}

static bool process_key(keyboard_state_t *kb, int mode, int idx, const char *override_glyph)
{
    if (!kb || idx < 0 || idx >= KEYBOARD_KEY_COUNT) return false;

    if (idx == KB_SHIFT_IDX) {
        if (kb->symbol_page) {
            kb->shift = !kb->shift;
            kb->shift_lock = false;
        } else if (kb->shift_lock) {
            kb->shift = false;
            kb->shift_lock = false;
        } else if (kb->shift) {
            kb->shift_lock = true;
        } else {
            kb->shift = true;
        }
        return false;
    }
    if (idx == KB_PAGE_IDX) {
        kb->symbol_page = !kb->symbol_page;
        kb->shift = false;
        kb->shift_lock = false;
        return false;
    }
    if (idx == KB_BACKSPACE_IDX) {
        return keyboard_backspace(kb);
    }
    if (idx == KB_ENTER_IDX) {
        if (kb->text_len > 0) {
            kb->text_len = 0;
            kb->text[0] = '\0';
        }
        kb->visible = false;
        kb->shift = false;
        kb->shift_lock = false;
        return true;
    }
    if (idx >= KB_SPACE_START && idx <= KB_SPACE_END) {
        return append_text(kb, " ");
    }

    const char *glyph = override_glyph ? override_glyph : kLayout[mode][idx];
    if (!glyph || !glyph[0]) return false;
    bool changed = append_text(kb, glyph);
    if (!kb->symbol_page && !kb->shift_lock) {
        kb->shift = false;
    }
    return changed;
}

static int index_from_coords(const keyboard_state_t *kb, float x, float y)
{
    float lx = x - (float)kb->origin_x;
    float ly = y - (float)kb->origin_y;
    if (lx < 0.0f || ly < 0.0f) return -1;
    int col = (int)floorf(lx / kb->key_px);
    int row = (int)floorf(ly / kb->key_px);
    if (col < 0 || col >= KEYBOARD_COLS || row < 0 || row >= KEYBOARD_ROWS) return -1;
    return row * KEYBOARD_COLS + col;
}

static int popup_pick(const keyboard_state_t *kb, float x, int options)
{
    if (!kb || options <= 0) return 0;
    int col = kb->popup_idx % KEYBOARD_COLS;
    float left = (float)kb->origin_x + (float)col * kb->key_px - 0.25f * kb->key_px;
    float slot_w = kb->key_px * 1.5f;
    int pick = (int)floorf((x - left) / slot_w);
    return clamp_int(pick, 0, options - 1);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void keyboard_init(keyboard_state_t *kb, const keyboard_config_t *cfg)
{
    if (!kb) return;
    memset(kb, 0, sizeof(*kb));

    kb->visible = true;
    kb->key_px = (cfg && cfg->key_px > 0.0f) ? cfg->key_px : 12.0f;
    kb->long_press_sec = (cfg && cfg->long_press_sec > 0.0f) ? cfg->long_press_sec : 0.4f;
    float repeat_hz = (cfg && cfg->repeat_rate_hz > 0.0f) ? cfg->repeat_rate_hz : 10.0f;
    kb->repeat_interval_sec = 1.0f / repeat_hz;
    int cell_px = (int)(kb->key_px < 4.0f ? 4.0f : kb->key_px);

    int default_x = (int)((PANEL_W - (cell_px * KEYBOARD_COLS)) / 2.0f);
    kb->origin_x = cfg ? cfg->origin_x : default_x;
    int default_y = PANEL_H - (int)(cell_px * KEYBOARD_ROWS) - cell_px; // lifted a row above bottom
    if (default_y < 0) default_y = 0;
    kb->origin_y = cfg ? cfg->origin_y : default_y;

    kb->active_idx = -1;
    kb->hot_idx = -1;
    kb->popup_idx = -1;
    set_text_ptr(kb, cfg);
}

void keyboard_set_origin(keyboard_state_t *kb, int x, int y)
{
    if (!kb) return;
    kb->origin_x = x;
    kb->origin_y = y;
}

void keyboard_show(keyboard_state_t *kb, bool show)
{
    if (!kb) return;
    kb->visible = show;
}

bool keyboard_is_visible(const keyboard_state_t *kb)
{
    return kb ? kb->visible : false;
}

void keyboard_clear_text(keyboard_state_t *kb)
{
    if (!kb || !kb->text) return;
    kb->text_len = 0;
    kb->text[0] = '\0';
}

void keyboard_set_text(keyboard_state_t *kb, const char *text)
{
    if (!kb || !kb->text) return;
    keyboard_clear_text(kb);
    if (!text) return;
    size_t len = strlen(text);
    if (len > kb->text_cap) len = kb->text_cap;
    memcpy(kb->text, text, len);
    kb->text_len = len;
    kb->text[kb->text_len] = '\0';
}

const char *keyboard_get_text(const keyboard_state_t *kb)
{
    if (!kb) return NULL;
    return kb->text;
}

size_t keyboard_get_length(const keyboard_state_t *kb)
{
    return kb ? kb->text_len : 0;
}

bool keyboard_handle_pointer(keyboard_state_t *kb, const keyboard_pointer_event_t *ev)
{
    if (!kb || !ev || !kb->visible) return false;

    int idx = index_from_coords(kb, ev->x, ev->y);
    kb->hot_idx = idx;
    int mode = layout_mode(kb);
    bool changed = false;

    if (ev->pressed) {
        if (!kb->pointer_down) {
            if (idx < 0) return false; // ignore presses outside
            kb->pointer_down = true;
            kb->active_idx = idx;
            kb->hold_time = 0.0f;
            kb->repeat_accum = 0.0f;
            kb->popup_active = false;
            kb->popup_idx = -1;
            kb->popup_selection = 0;
        } else {
            if (idx != kb->active_idx) {
                kb->active_idx = idx;
                kb->hold_time = 0.0f;
                kb->repeat_accum = 0.0f;
                kb->popup_active = false;
                kb->popup_idx = -1;
                kb->popup_selection = 0;
            } else {
                kb->hold_time += ev->dt_sec;
            }
        }

        if (kb->active_idx == KB_BACKSPACE_IDX && kb->hold_time >= kb->long_press_sec) {
            kb->repeat_accum += ev->dt_sec;
            while (kb->repeat_accum >= kb->repeat_interval_sec) {
                changed |= process_key(kb, mode, KB_BACKSPACE_IDX, NULL);
                kb->repeat_accum -= kb->repeat_interval_sec;
            }
        } else if (!kb->popup_active && kb->hold_time >= kb->long_press_sec) {
            const key_specials_t *spec = get_specials(mode, kb->active_idx);
            if (spec && spec->count > 1) {
                kb->popup_active = true;
                kb->popup_idx = kb->active_idx;
                kb->popup_selection = popup_pick(kb, ev->x, spec->count);
            }
        } else if (kb->popup_active) {
            const key_specials_t *spec = get_specials(mode, kb->popup_idx);
            if (spec && spec->count > 0) {
                kb->popup_selection = popup_pick(kb, ev->x, spec->count);
            }
        }
    } else {
        if (kb->pointer_down && kb->active_idx >= 0) {
            if (kb->popup_active) {
                const key_specials_t *spec = get_specials(mode, kb->popup_idx);
                if (spec && spec->count > 0) {
                    int pick = clamp_int(kb->popup_selection, 0, (int)spec->count - 1);
                    changed |= process_key(kb, mode, kb->popup_idx, spec->options[pick]);
                }
            } else if (idx == kb->active_idx) {
                changed |= process_key(kb, mode, kb->active_idx, NULL);
            }
        }
        kb->pointer_down = false;
        kb->active_idx = -1;
        kb->hold_time = 0.0f;
        kb->repeat_accum = 0.0f;
        kb->popup_active = false;
        kb->popup_idx = -1;
        kb->popup_selection = 0;
    }

    return changed;
}

static void draw_label_centered(uint8_t *fb, int x0, int y0, int w, int h, const char *label)
{
    if (!fb || !label) return;
    int len = (int)strlen(label);
    if (len == 0) return;
    int text_w = len * 4 - 1; // 3 px glyph + 1 px gap per char
    int text_h = 5;
    int tx = x0 + (w - text_w) / 2;
    int ty = y0 + (h - text_h) / 2;
    if (tx < x0) tx = x0;
    if (ty < y0) ty = y0;
    oled_draw_text3x5(fb, tx, ty, label);
}

static void draw_key(uint8_t *fb, int x0, int y0, int size, const char *label, bool highlight, bool merge_left, bool draw_bottom, bool draw_right)
{
    int w = size;
    int h = size;

    // Frame with shared borders: top/left always, right/bottom only on the outer edge.
    fb_rect_fill(fb, x0, y0, w, 1);           // top
    if (!merge_left) {
        fb_rect_fill(fb, x0, y0, 1, h);       // left
    }
    if (draw_right) {
        fb_rect_fill(fb, x0 + w - 1, y0, 1, h);
    }
    if (draw_bottom) {
        fb_rect_fill(fb, x0, y0 + h - 1, w, 1);
    }

    draw_label_centered(fb, x0, y0, w, h, label);
    if (highlight && w > 2 && h > 2) {
        // invert only the interior to keep the border stable
        fb_rect_invert(fb, x0 + 1, y0 + 1, w - 2, h - 2);
    }
}

static void draw_spacebar(uint8_t *fb, int x0, int y0, int w, int h, bool highlight)
{
    if (!fb || w <= 0 || h <= 0) return;
    // Outer frame
    fb_rect_fill(fb, x0, y0, w, 1);
    fb_rect_fill(fb, x0, y0, 1, h);
    fb_rect_fill(fb, x0 + w - 1, y0, 1, h);
    fb_rect_fill(fb, x0, y0 + h - 1, w, 1);

    // Simple bar shape inside
    int bar_h = h / 3;
    if (bar_h < 2) bar_h = 2;
    int bar_y = y0 + (h - bar_h) / 2;
    int bar_w = w - 4;
    if (bar_w > 0) {
        fb_rect_fill(fb, x0 + 2, bar_y, bar_w, bar_h);
    }

    draw_label_centered(fb, x0, y0, w, h, "SPACE");
    if (highlight && w > 2 && h > 2) {
        fb_rect_invert(fb, x0 + 1, y0 + 1, w - 2, h - 2);
    }
}

static void draw_popup(const keyboard_state_t *kb, uint8_t *fb, int mode)
{
    if (!kb || !kb->popup_active || kb->popup_idx < 0 || !fb) return;
    const key_specials_t *spec = get_specials(mode, kb->popup_idx);
    if (!spec || spec->count == 0) return;

    int col = kb->popup_idx % KEYBOARD_COLS;
    int row = kb->popup_idx / KEYBOARD_COLS;
    int slot_w = (int)(kb->key_px * 1.5f);
    int x0 = (int)((float)kb->origin_x + (float)col * kb->key_px - 0.25f * kb->key_px);
    int y0 = (int)((float)kb->origin_y + (float)row * kb->key_px - kb->key_px);
    if (x0 < 0) x0 = 0;
    int total_w = slot_w * spec->count;
    if (x0 + total_w > PANEL_W) x0 = PANEL_W - total_w;
    if (y0 < 0) y0 = 0;

    for (int i = 0; i < spec->count; ++i) {
        int px = x0 + i * slot_w;
        bool highlight = (i == kb->popup_selection);
        draw_key(fb, px, y0, slot_w, spec->options[i], highlight, false, true, true);
    }
}

void keyboard_draw(const keyboard_state_t *kb, uint8_t *fb)
{
    if (!kb || !fb || !kb->visible) return;
    int mode = layout_mode(kb);
    int cell_px = (int)(kb->key_px < 4.0f ? 4.0f : kb->key_px);
    int key_px = cell_px; // fill the cell to avoid extra gaps between keys

    const int space_row = KB_SPACE_START / KEYBOARD_COLS;
    const int space_start_col = KB_SPACE_START % KEYBOARD_COLS;
    const int space_end_col = KB_SPACE_END % KEYBOARD_COLS;
    bool space_hot = false;
    bool space_active = false;

    for (int row = 0; row < KEYBOARD_ROWS; ++row) {
        for (int col = 0; col < KEYBOARD_COLS; ++col) {
            int idx = row * KEYBOARD_COLS + col;
            const char *label = key_label(kb, mode, idx);
            int cell_x0 = kb->origin_x + col * cell_px;
            int cell_y0 = kb->origin_y + row * cell_px;
            int x0 = cell_x0 + (cell_px - key_px) / 2;
            int y0 = cell_y0 + (cell_px - key_px) / 2;
            bool highlight = (kb->pointer_down && kb->active_idx == idx) || (kb->hot_idx == idx);
            bool is_space = (idx >= KB_SPACE_START && idx <= KB_SPACE_END);
            if (is_space) {
                if (highlight) {
                    space_hot = space_hot || (kb->hot_idx == idx);
                    space_active = space_active || (kb->pointer_down && kb->active_idx == idx);
                }
                continue;
            }

            bool merge_left = (row == space_row) && (col > space_start_col) && (col <= space_end_col);
            bool draw_bottom = (row == KEYBOARD_ROWS - 1);
            bool draw_right = (col == KEYBOARD_COLS - 1);
            draw_key(fb, x0, y0, key_px, label, highlight, merge_left, draw_bottom, draw_right);
        }
    }

    bool space_highlight = space_hot || space_active;
    int space_cell_x0 = kb->origin_x + space_start_col * cell_px;
    int space_cell_y0 = kb->origin_y + space_row * cell_px;
    int space_w = cell_px * (KB_SPACE_END - KB_SPACE_START + 1);
    int space_x0 = space_cell_x0 + (cell_px - key_px) / 2;
    int space_y0 = space_cell_y0 + (cell_px - key_px) / 2;
    draw_spacebar(fb, space_x0, space_y0, space_w, key_px, space_highlight);
    draw_popup(kb, fb, mode);
}
