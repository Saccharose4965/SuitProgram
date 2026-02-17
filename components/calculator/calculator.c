#include "calculator.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "oled.h"

#define CALC_EXPR_MAX 64
#define CALC_ROWS 5
#define CALC_COLS 4

typedef enum {
    KEY_ACT_CHAR = 0,
    KEY_ACT_EVAL,
    KEY_ACT_CLEAR,
    KEY_ACT_BACKSPACE,
} key_action_t;

typedef struct {
    const char *label;
    key_action_t action;
    char value;
} calc_key_t;

static const calc_key_t k_keys[CALC_ROWS][CALC_COLS] = {
    { {"7", KEY_ACT_CHAR, '7'}, {"8", KEY_ACT_CHAR, '8'}, {"9", KEY_ACT_CHAR, '9'}, {"/", KEY_ACT_CHAR, '/'} },
    { {"4", KEY_ACT_CHAR, '4'}, {"5", KEY_ACT_CHAR, '5'}, {"6", KEY_ACT_CHAR, '6'}, {"*", KEY_ACT_CHAR, '*'} },
    { {"1", KEY_ACT_CHAR, '1'}, {"2", KEY_ACT_CHAR, '2'}, {"3", KEY_ACT_CHAR, '3'}, {"-", KEY_ACT_CHAR, '-'} },
    { {"0", KEY_ACT_CHAR, '0'}, {".", KEY_ACT_CHAR, '.'}, {"(", KEY_ACT_CHAR, '('}, {")", KEY_ACT_CHAR, ')'} },
    { {"C", KEY_ACT_CLEAR, 0},   {"<", KEY_ACT_BACKSPACE, 0}, {"=", KEY_ACT_EVAL, 0},   {"+", KEY_ACT_CHAR, '+'} },
};

typedef struct {
    char expr[CALC_EXPR_MAX];
    size_t len;
    int row;
    int col;
    char status[24];
    bool status_is_error;
} calc_state_t;

static calc_state_t s_calc = {0};

typedef struct {
    const char *s;
    size_t pos;
    bool ok;
    bool div_zero;
} parser_t;

static inline bool is_op(char c)
{
    return c == '+' || c == '-' || c == '*' || c == '/';
}

static inline bool fb_in_bounds(int x, int y)
{
    return ((unsigned)x < PANEL_W) && ((unsigned)y < PANEL_H);
}

static inline void fb_pset(uint8_t *fb, int x, int y)
{
    if (!fb || !fb_in_bounds(x, y)) return;
    int idx = y * PANEL_W + x;
    fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void fb_rect_outline(uint8_t *fb, int x, int y, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;

    int x1 = x + w - 1;
    int y1 = y + h - 1;

    if (x1 < 0 || y1 < 0 || x >= PANEL_W || y >= PANEL_H) return;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x1 >= PANEL_W) x1 = PANEL_W - 1;
    if (y1 >= PANEL_H) y1 = PANEL_H - 1;

    for (int ix = x; ix <= x1; ++ix) {
        fb_pset(fb, ix, y);
        fb_pset(fb, ix, y1);
    }
    for (int iy = y; iy <= y1; ++iy) {
        fb_pset(fb, x, iy);
        fb_pset(fb, x1, iy);
    }
}

static void parser_skip_ws(parser_t *p)
{
    while (p && p->s[p->pos] && isspace((unsigned char)p->s[p->pos])) {
        p->pos++;
    }
}

static double parser_parse_expr(parser_t *p);

static double parser_parse_number(parser_t *p)
{
    if (!p) return 0.0;
    parser_skip_ws(p);

    const char *start = p->s + p->pos;
    char *end = NULL;
    double v = strtod(start, &end);
    if (!end || end == start) {
        p->ok = false;
        return 0.0;
    }

    p->pos += (size_t)(end - start);
    return v;
}

static double parser_parse_factor(parser_t *p)
{
    if (!p) return 0.0;
    parser_skip_ws(p);

    char c = p->s[p->pos];
    if (c == '+') {
        p->pos++;
        return parser_parse_factor(p);
    }
    if (c == '-') {
        p->pos++;
        return -parser_parse_factor(p);
    }
    if (c == '(') {
        p->pos++;
        double v = parser_parse_expr(p);
        parser_skip_ws(p);
        if (!p->ok || p->s[p->pos] != ')') {
            p->ok = false;
            return 0.0;
        }
        p->pos++;
        return v;
    }

    return parser_parse_number(p);
}

static double parser_parse_term(parser_t *p)
{
    if (!p) return 0.0;
    double v = parser_parse_factor(p);

    while (p->ok) {
        parser_skip_ws(p);
        char op = p->s[p->pos];
        if (op != '*' && op != '/') break;

        p->pos++;
        double rhs = parser_parse_factor(p);
        if (!p->ok) return 0.0;

        if (op == '*') {
            v *= rhs;
        } else {
            if (fabs(rhs) <= 1e-12) {
                p->ok = false;
                p->div_zero = true;
                return 0.0;
            }
            v /= rhs;
        }
    }

    return v;
}

static double parser_parse_expr(parser_t *p)
{
    if (!p) return 0.0;
    double v = parser_parse_term(p);

    while (p->ok) {
        parser_skip_ws(p);
        char op = p->s[p->pos];
        if (op != '+' && op != '-') break;

        p->pos++;
        double rhs = parser_parse_term(p);
        if (!p->ok) return 0.0;

        if (op == '+') v += rhs;
        else v -= rhs;
    }

    return v;
}

static bool eval_expression(const char *expr, double *out_result, bool *out_div_zero)
{
    if (!expr || !expr[0] || !out_result) return false;

    parser_t p = {
        .s = expr,
        .pos = 0,
        .ok = true,
        .div_zero = false,
    };

    double v = parser_parse_expr(&p);
    parser_skip_ws(&p);

    if (!p.ok || p.s[p.pos] != '\0' || !isfinite(v)) {
        if (out_div_zero) *out_div_zero = p.div_zero;
        return false;
    }

    *out_result = v;
    if (out_div_zero) *out_div_zero = false;
    return true;
}

static int expr_balance_parens(void)
{
    int bal = 0;
    for (size_t i = 0; i < s_calc.len; ++i) {
        if (s_calc.expr[i] == '(') bal++;
        if (s_calc.expr[i] == ')') bal--;
    }
    return bal;
}

static bool can_append_dot(void)
{
    if (s_calc.len == 0) return true;

    for (int i = (int)s_calc.len - 1; i >= 0; --i) {
        char c = s_calc.expr[i];
        if (c == '.') return false;
        if (is_op(c) || c == '(' || c == ')') return true;
    }
    return true;
}

static void set_status(const char *msg, bool is_error)
{
    if (!msg) msg = "";
    strncpy(s_calc.status, msg, sizeof(s_calc.status) - 1);
    s_calc.status[sizeof(s_calc.status) - 1] = '\0';
    s_calc.status_is_error = is_error;
}

static void append_char(char c)
{
    if (s_calc.len + 1 >= sizeof(s_calc.expr)) {
        set_status("FULL", true);
        return;
    }

    char prev = (s_calc.len > 0) ? s_calc.expr[s_calc.len - 1] : '\0';

    if (isdigit((unsigned char)c)) {
        if (prev == ')') {
            set_status("need op", true);
            return;
        }
        s_calc.expr[s_calc.len++] = c;
        s_calc.expr[s_calc.len] = '\0';
        return;
    }

    if (c == '.') {
        if (prev == ')' || !can_append_dot()) {
            set_status("dot", true);
            return;
        }
        if (s_calc.len == 0 || is_op(prev) || prev == '(') {
            s_calc.expr[s_calc.len++] = '0';
        }
        s_calc.expr[s_calc.len++] = '.';
        s_calc.expr[s_calc.len] = '\0';
        return;
    }

    if (c == '(') {
        if (s_calc.len > 0 && (isdigit((unsigned char)prev) || prev == ')' || prev == '.')) {
            set_status("need *", true);
            return;
        }
        s_calc.expr[s_calc.len++] = '(';
        s_calc.expr[s_calc.len] = '\0';
        return;
    }

    if (c == ')') {
        if (expr_balance_parens() <= 0) {
            set_status(") err", true);
            return;
        }
        if (s_calc.len == 0 || is_op(prev) || prev == '(') {
            set_status(") err", true);
            return;
        }
        s_calc.expr[s_calc.len++] = ')';
        s_calc.expr[s_calc.len] = '\0';
        return;
    }

    if (is_op(c)) {
        if (s_calc.len == 0) {
            if (c == '-') {
                s_calc.expr[s_calc.len++] = '-';
                s_calc.expr[s_calc.len] = '\0';
            } else {
                set_status("op err", true);
            }
            return;
        }

        if (prev == '(') {
            if (c == '-') {
                s_calc.expr[s_calc.len++] = '-';
                s_calc.expr[s_calc.len] = '\0';
            } else {
                set_status("op err", true);
            }
            return;
        }

        if (is_op(prev)) {
            s_calc.expr[s_calc.len - 1] = c;
            return;
        }

        if (prev == '.') {
            set_status("op err", true);
            return;
        }

        s_calc.expr[s_calc.len++] = c;
        s_calc.expr[s_calc.len] = '\0';
    }
}

static void apply_key(const calc_key_t *key)
{
    if (!key) return;

    if (key->action == KEY_ACT_CLEAR) {
        s_calc.len = 0;
        s_calc.expr[0] = '\0';
        set_status("", false);
        return;
    }

    if (key->action == KEY_ACT_BACKSPACE) {
        if (s_calc.len > 0) {
            s_calc.len--;
            s_calc.expr[s_calc.len] = '\0';
        }
        set_status("", false);
        return;
    }

    if (key->action == KEY_ACT_EVAL) {
        if (s_calc.len == 0) {
            set_status("empty", true);
            return;
        }

        double out = 0.0;
        bool div_zero = false;
        if (!eval_expression(s_calc.expr, &out, &div_zero)) {
            set_status(div_zero ? "DIV0" : "ERR", true);
            return;
        }

        char tmp[CALC_EXPR_MAX];
        snprintf(tmp, sizeof(tmp), "%.8g", out);
        strncpy(s_calc.expr, tmp, sizeof(s_calc.expr) - 1);
        s_calc.expr[sizeof(s_calc.expr) - 1] = '\0';
        s_calc.len = strlen(s_calc.expr);
        set_status("OK", false);
        return;
    }

    if (key->action == KEY_ACT_CHAR) {
        append_char(key->value);
    }
}

void calculator_init(void)
{
    memset(&s_calc, 0, sizeof(s_calc));
    s_calc.row = 0;
    s_calc.col = 0;
    s_calc.expr[0] = '\0';
    set_status("A/B row C col D key", false);
}

void calculator_handle_input(const input_event_t *ev)
{
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        s_calc.row--;
        if (s_calc.row < 0) s_calc.row = CALC_ROWS - 1;
    } else if (ev->button == INPUT_BTN_B) {
        s_calc.row = (s_calc.row + 1) % CALC_ROWS;
    } else if (ev->button == INPUT_BTN_C) {
        s_calc.col = (s_calc.col + 1) % CALC_COLS;
    } else if (ev->button == INPUT_BTN_D) {
        apply_key(&k_keys[s_calc.row][s_calc.col]);
    }
}

static void draw_expr_line(uint8_t *fb, int x, int y, int w)
{
    char line[40];
    const int max_chars = (w > 8) ? ((w - 8) / 4) : 1;
    if (max_chars <= 0) return;
    if (max_chars <= 2) {
        oled_draw_text3x5(fb, x + 2, y, "E");
        return;
    }

    const char *expr = s_calc.expr;
    size_t len = s_calc.len;
    if (len == 0) {
        snprintf(line, sizeof(line), "E:0");
    } else {
        size_t keep = len;
        if ((int)keep > max_chars - 2) {
            keep = (size_t)(max_chars - 2);
            expr = s_calc.expr + (len - keep);
            snprintf(line, sizeof(line), "E:%s", expr);
            if ((int)strlen(line) > max_chars) {
                line[max_chars] = '\0';
            }
        } else {
            snprintf(line, sizeof(line), "E:%s", expr);
        }
    }

    oled_draw_text3x5(fb, x + 2, y, line);
}

void calculator_draw(uint8_t *fb, int x, int y, int w, int h)
{
    if (!fb) return;

    oled_draw_text3x5(fb, x + 2, y + 1, "CALC");
    draw_expr_line(fb, x, y + 8, w);

    if (s_calc.status[0]) {
        char line[24];
        snprintf(line, sizeof(line), "%s%s", s_calc.status_is_error ? "!" : "", s_calc.status);
        oled_draw_text3x5(fb, x + 2, y + 14, line);
    }

    int top = y + 22;
    int avail_h = h - (top - y);
    if (avail_h < CALC_ROWS * 6) return;

    int cell_w = w / CALC_COLS;
    if (cell_w < 10) cell_w = 10;

    int cell_h = avail_h / CALC_ROWS;
    if (cell_h > 8) cell_h = 8;
    if (cell_h < 6) cell_h = 6;

    for (int r = 0; r < CALC_ROWS; ++r) {
        for (int c = 0; c < CALC_COLS; ++c) {
            int cx = x + c * cell_w;
            int cy = top + r * cell_h;
            int bw = cell_w - 1;
            int bh = cell_h - 1;
            if (bw < 6 || bh < 5) continue;

            fb_rect_outline(fb, cx, cy, bw, bh);
            if (r == s_calc.row && c == s_calc.col) {
                fb_rect_outline(fb, cx + 1, cy + 1, bw - 2, bh - 2);
            }

            const char *label = k_keys[r][c].label;
            int len = (int)strlen(label);
            int tx = cx + (bw - (len * 4)) / 2;
            if (tx < cx + 1) tx = cx + 1;
            int ty = cy + ((bh - 5) / 2);
            if (ty < cy + 1) ty = cy + 1;
            oled_draw_text3x5(fb, tx, ty, label);
        }
    }
}
