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
#define CALC_COLS 5
#define CALC_PI 3.14159265358979323846

typedef enum {
    KEY_ACT_INSERT = 0,
    KEY_ACT_EVAL,
    KEY_ACT_CLEAR,
    KEY_ACT_BACKSPACE,
} key_action_t;

typedef struct {
    const char *label;
    key_action_t action;
    const char *text;
} calc_key_t;

static const calc_key_t k_keys[CALC_ROWS][CALC_COLS] = {
    { {"DEL",  KEY_ACT_BACKSPACE, NULL   }, {"(",   KEY_ACT_INSERT, "("    }, {")",   KEY_ACT_INSERT, ")"    }, {"mod",  KEY_ACT_INSERT, "%"    }, {"sqrt", KEY_ACT_INSERT, "sqrt("} },
    { {"7",    KEY_ACT_INSERT,    "7"    }, {"8",   KEY_ACT_INSERT, "8"    }, {"9",   KEY_ACT_INSERT, "9"    }, {"/",    KEY_ACT_INSERT, "/"    }, {"x^2",  KEY_ACT_INSERT, "^2"   } },
    { {"4",    KEY_ACT_INSERT,    "4"    }, {"5",   KEY_ACT_INSERT, "5"    }, {"6",   KEY_ACT_INSERT, "6"    }, {"*",    KEY_ACT_INSERT, "*"    }, {"ln",   KEY_ACT_INSERT, "ln("  } },
    { {"1",    KEY_ACT_INSERT,    "1"    }, {"2",   KEY_ACT_INSERT, "2"    }, {"3",   KEY_ACT_INSERT, "3"    }, {"-",    KEY_ACT_INSERT, "-"    }, {"e^x",  KEY_ACT_INSERT, "exp(" } },
    { {"0",    KEY_ACT_INSERT,    "0"    }, {".",   KEY_ACT_INSERT, "."    }, {"pi",  KEY_ACT_INSERT, "pi"   }, {"+",    KEY_ACT_INSERT, "+"    }, {"=",    KEY_ACT_EVAL,   NULL   } },
};

typedef struct {
    char expr[CALC_EXPR_MAX];
    size_t len;
    int row;
    int col;
    char status[24];
    bool status_is_error;
    bool just_evaluated;
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
    return c == '+' || c == '-' || c == '*' || c == '/' || c == '%' || c == '^';
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
static double parser_parse_term(parser_t *p);
static double parser_parse_power(parser_t *p);
static double parser_parse_unary(parser_t *p);
static double parser_parse_primary(parser_t *p);

static bool parser_match_kw(parser_t *p, const char *kw)
{
    if (!p || !kw) return false;
    parser_skip_ws(p);

    size_t start = p->pos;
    size_t n = strlen(kw);
    size_t remaining = strlen(p->s + start);
    if (remaining < n) {
        return false;
    }
    if (strncmp(p->s + start, kw, n) != 0) {
        return false;
    }
    char tail = p->s[start + n];
    if (isalnum((unsigned char)tail) || tail == '_') {
        return false;
    }
    p->pos = start + n;
    return true;
}

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

static double parser_parse_primary(parser_t *p)
{
    if (!p) return 0.0;
    parser_skip_ws(p);

    char c = p->s[p->pos];
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

    if (isalpha((unsigned char)c)) {
        if (parser_match_kw(p, "pi")) {
            return CALC_PI;
        }
        if (parser_match_kw(p, "sqrt")) {
            parser_skip_ws(p);
            if (p->s[p->pos] != '(') {
                p->ok = false;
                return 0.0;
            }
            p->pos++;
            double arg = parser_parse_expr(p);
            parser_skip_ws(p);
            if (!p->ok || p->s[p->pos] != ')' || arg < 0.0) {
                p->ok = false;
                return 0.0;
            }
            p->pos++;
            return sqrt(arg);
        }
        if (parser_match_kw(p, "ln")) {
            parser_skip_ws(p);
            if (p->s[p->pos] != '(') {
                p->ok = false;
                return 0.0;
            }
            p->pos++;
            double arg = parser_parse_expr(p);
            parser_skip_ws(p);
            if (!p->ok || p->s[p->pos] != ')' || arg <= 0.0) {
                p->ok = false;
                return 0.0;
            }
            p->pos++;
            return log(arg);
        }
        if (parser_match_kw(p, "exp")) {
            parser_skip_ws(p);
            if (p->s[p->pos] != '(') {
                p->ok = false;
                return 0.0;
            }
            p->pos++;
            double arg = parser_parse_expr(p);
            parser_skip_ws(p);
            if (!p->ok || p->s[p->pos] != ')') {
                p->ok = false;
                return 0.0;
            }
            p->pos++;
            double v = exp(arg);
            if (!isfinite(v)) {
                p->ok = false;
                return 0.0;
            }
            return v;
        }
        p->ok = false;
        return 0.0;
    }

    return parser_parse_number(p);
}

static double parser_parse_unary(parser_t *p)
{
    if (!p) return 0.0;
    parser_skip_ws(p);
    char c = p->s[p->pos];

    if (c == '+') {
        p->pos++;
        return parser_parse_unary(p);
    }
    if (c == '-') {
        p->pos++;
        return -parser_parse_unary(p);
    }

    return parser_parse_primary(p);
}

static double parser_parse_power(parser_t *p)
{
    if (!p) return 0.0;
    double base = parser_parse_unary(p);
    if (!p->ok) return 0.0;

    parser_skip_ws(p);
    if (p->s[p->pos] == '^') {
        p->pos++;
        double rhs = parser_parse_power(p); // right-associative exponentiation
        if (!p->ok) return 0.0;

        double v = pow(base, rhs);
        if (!isfinite(v)) {
            p->ok = false;
            return 0.0;
        }
        return v;
    }

    return base;
}

static double parser_parse_term(parser_t *p)
{
    if (!p) return 0.0;
    double v = parser_parse_power(p);

    while (p->ok) {
        parser_skip_ws(p);
        char op = '\0';
        if (p->s[p->pos] == '*' || p->s[p->pos] == '/' || p->s[p->pos] == '%') {
            op = p->s[p->pos];
            p->pos++;
        } else if (parser_match_kw(p, "mod")) {
            op = '%';
        } else {
            break;
        }

        double rhs = parser_parse_power(p);
        if (!p->ok) return 0.0;

        if (op == '*') {
            v *= rhs;
        } else if (op == '/') {
            if (fabs(rhs) <= 1e-12) {
                p->ok = false;
                p->div_zero = true;
                return 0.0;
            }
            v /= rhs;
        } else { // '%'
            if (fabs(rhs) <= 1e-12) {
                p->ok = false;
                p->div_zero = true;
                return 0.0;
            }
            v = fmod(v, rhs);
        }

        if (!isfinite(v)) {
            p->ok = false;
            return 0.0;
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
        if (is_op(c) || c == '(' || c == ')' || isalpha((unsigned char)c)) return true;
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

static inline char expr_prev_char(void)
{
    return (s_calc.len > 0) ? s_calc.expr[s_calc.len - 1] : '\0';
}

static inline bool expr_ends_with_value(void)
{
    char prev = expr_prev_char();
    return isdigit((unsigned char)prev) || prev == ')' || prev == '.' || prev == 'i';
}

static bool is_operator_insert_token(const char *text)
{
    if (!text || !text[0]) return false;
    if (strcmp(text, "^2") == 0) return true;
    return (strlen(text) == 1) && is_op(text[0]);
}

static bool append_raw(const char *text)
{
    if (!text || !text[0]) return false;
    size_t n = strlen(text);
    if (s_calc.len + n >= sizeof(s_calc.expr)) {
        set_status("FULL", true);
        return false;
    }
    memcpy(s_calc.expr + s_calc.len, text, n);
    s_calc.len += n;
    s_calc.expr[s_calc.len] = '\0';
    return true;
}

static void append_insert(const char *text)
{
    if (!text || !text[0]) return;

    if (s_calc.just_evaluated) {
        // After "=", numeric/function input starts a fresh expression.
        // Operator-like input continues from the computed result.
        if (!is_operator_insert_token(text)) {
            s_calc.len = 0;
            s_calc.expr[0] = '\0';
        }
        s_calc.just_evaluated = false;
    }

    char prev = expr_prev_char();
    size_t tok_len = strlen(text);

    if (tok_len == 1 && isdigit((unsigned char)text[0])) {
        if (prev == ')' || prev == 'i') {
            set_status("need op", true);
            return;
        }
        if (append_raw(text)) set_status("", false);
        return;
    }

    if (strcmp(text, ".") == 0) {
        if (prev == ')' || isalpha((unsigned char)prev) || !can_append_dot()) {
            set_status("dot", true);
            return;
        }
        if (s_calc.len == 0 || is_op(prev) || prev == '(') {
            if (!append_raw("0")) return;
        }
        if (append_raw(".")) set_status("", false);
        return;
    }

    if (strcmp(text, "(") == 0) {
        if (expr_ends_with_value()) {
            set_status("need *", true);
            return;
        }
        if (append_raw("(")) set_status("", false);
        return;
    }

    if (strcmp(text, ")") == 0) {
        if (expr_balance_parens() <= 0) {
            set_status(") err", true);
            return;
        }
        if (s_calc.len == 0 || is_op(prev) || prev == '(') {
            set_status(") err", true);
            return;
        }
        if (append_raw(")")) set_status("", false);
        return;
    }

    if (strcmp(text, "pi") == 0 ||
        strcmp(text, "sqrt(") == 0 ||
        strcmp(text, "ln(") == 0 ||
        strcmp(text, "exp(") == 0) {
        if (expr_ends_with_value()) {
            set_status("need *", true);
            return;
        }
        if (append_raw(text)) set_status("", false);
        return;
    }

    if (strcmp(text, "^2") == 0) {
        if (!expr_ends_with_value()) {
            set_status("pow err", true);
            return;
        }
        if (append_raw("^2")) set_status("", false);
        return;
    }

    if (tok_len == 1 && is_op(text[0])) {
        char c = text[0];
        if (s_calc.len == 0) {
            if (c == '-') {
                if (append_raw("-")) set_status("", false);
            } else {
                set_status("op err", true);
            }
            return;
        }

        if (prev == '(') {
            if (c == '-') {
                if (append_raw("-")) set_status("", false);
            } else {
                set_status("op err", true);
            }
            return;
        }

        if (is_op(prev)) {
            s_calc.expr[s_calc.len - 1] = c;
            set_status("", false);
            return;
        }

        if (!expr_ends_with_value()) {
            set_status("op err", true);
            return;
        }

        if (append_raw(text)) set_status("", false);
        return;
    }

    if (append_raw(text)) set_status("", false);
}

static void apply_key(const calc_key_t *key)
{
    if (!key) return;

    if (key->action == KEY_ACT_CLEAR) {
        s_calc.len = 0;
        s_calc.expr[0] = '\0';
        s_calc.just_evaluated = false;
        set_status("", false);
        return;
    }

    if (key->action == KEY_ACT_BACKSPACE) {
        if (s_calc.len > 0) {
            s_calc.len--;
            s_calc.expr[s_calc.len] = '\0';
        }
        s_calc.just_evaluated = false;
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
        s_calc.just_evaluated = true;
        set_status("OK", false);
        return;
    }

    if (key->action == KEY_ACT_INSERT) {
        append_insert(key->text);
    }
}

void calculator_init(void)
{
    memset(&s_calc, 0, sizeof(s_calc));
    s_calc.row = 0;
    s_calc.col = 0;
    s_calc.expr[0] = '\0';
    s_calc.just_evaluated = false;
    set_status("A< Bv C> D=key", false);
}

void calculator_handle_input(const input_event_t *ev)
{
    if (!ev || ev->type != INPUT_EVENT_PRESS) return;

    if (ev->button == INPUT_BTN_A) {
        s_calc.col--;
        if (s_calc.col < 0) s_calc.col = CALC_COLS - 1;
    } else if (ev->button == INPUT_BTN_B || ev->button == INPUT_BTN_BC_COMBO) {
        s_calc.row = (s_calc.row + 1) % CALC_ROWS;
    } else if (ev->button == INPUT_BTN_C) {
        s_calc.col = (s_calc.col + 1) % CALC_COLS;
    } else if (ev->button == INPUT_BTN_D) {
        apply_key(&k_keys[s_calc.row][s_calc.col]);
    }
}

static void draw_window_line(uint8_t *fb, int x, int y, int w, const char *prefix, const char *text)
{
    if (!fb || w <= 4) return;

    int max_chars = (w - 4) / 4;
    if (max_chars <= 0) return;

    char line[48];
    int out = 0;

    if (prefix) {
        for (const char *p = prefix; *p && out < max_chars; ++p) {
            line[out++] = *p;
        }
    }

    if (text && out < max_chars) {
        int text_len = (int)strlen(text);
        int keep = max_chars - out;
        int start = (text_len > keep) ? (text_len - keep) : 0;
        for (int i = start; i < text_len && out < max_chars; ++i) {
            line[out++] = text[i];
        }
    }

    line[out] = '\0';
    oled_draw_text3x5(fb, x + 2, y, line);
}

void calculator_draw(uint8_t *fb, int x, int y, int w, int h)
{
    if (!fb || w <= 0 || h <= 0) return;

    int pad_gap = 2;
    int pad_w = (w * 55) / 100; // narrower keypad to leave more room for the window
    int min_pad_w = CALC_COLS * 8 + (CALC_COLS + 1); // room for keys + 1px inter-key gaps
    if (pad_w < min_pad_w) pad_w = min_pad_w;
    if (pad_w > w - 20) pad_w = w - 20;

    int window_w = w - pad_w - pad_gap;
    if (window_w < 18) {
        window_w = 18;
        pad_w = w - window_w - pad_gap;
    }
    if (pad_w < CALC_COLS * 6 || window_w <= 0) return;

    int window_x = x;
    int pad_x = window_x + window_w + pad_gap;

    fb_rect_outline(fb, window_x, y, window_w, h);
    fb_rect_outline(fb, pad_x, y, pad_w, h);

    draw_window_line(fb, window_x, y + 2, window_w, "CALC", NULL);
    draw_window_line(fb, window_x, y + 10, window_w, "E:", s_calc.len ? s_calc.expr : "0");

    if (s_calc.status[0]) {
        draw_window_line(fb, window_x, y + 18, window_w, s_calc.status_is_error ? "!" : "", s_calc.status);
    }

    const calc_key_t *sel = &k_keys[s_calc.row][s_calc.col];
    int key_line_y = y + h - 7;
    if (key_line_y >= y + 26) {
        draw_window_line(fb, window_x, key_line_y, window_w, "SEL:", sel->label);
    }

    const int key_gap = 1;
    int grid_x = pad_x + 1;
    int grid_y = y + 1;
    int grid_w = pad_w - 2;
    int grid_h = h - 2;
    int usable_w = grid_w - key_gap * (CALC_COLS + 1);
    int usable_h = grid_h - key_gap * (CALC_ROWS + 1);
    if (usable_w <= 0 || usable_h <= 0) return;
    int base_w = usable_w / CALC_COLS;
    int rem_w = usable_w % CALC_COLS;
    int base_h = usable_h / CALC_ROWS;
    int rem_h = usable_h % CALC_ROWS;

    int cy = grid_y + key_gap;
    for (int r = 0; r < CALC_ROWS; ++r) {
        int bh = base_h + (r < rem_h ? 1 : 0);

        int cx = grid_x + key_gap;
        for (int c = 0; c < CALC_COLS; ++c) {
            int bw = base_w + (c < rem_w ? 1 : 0);
            int cx0 = cx;
            int cy0 = cy;
            if (bw >= 6 && bh >= 5) {
                fb_rect_outline(fb, cx0, cy0, bw, bh);
                if (r == s_calc.row && c == s_calc.col && bw > 4 && bh > 4) {
                    fb_rect_outline(fb, cx0 + 1, cy0 + 1, bw - 2, bh - 2);
                }

                const char *label = k_keys[r][c].label;
                int label_len = (int)strlen(label);
                int tx = cx0 + (bw - (label_len * 4)) / 2;
                if (tx < cx0 + 1) tx = cx0 + 1;
                int ty = cy0 + ((bh - 5) / 2);
                if (ty < cy0 + 1) ty = cy0 + 1;
                oled_draw_text3x5(fb, tx, ty, label);
            }

            cx += bw + key_gap;
        }
        cy += bh + key_gap;
    }
}
