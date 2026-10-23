#include "calculator_math.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define CALC_PI 3.14159265358979323846

typedef struct {
    const char *s;
    size_t pos;
    bool ok;
    bool div_zero;
} parser_t;

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

bool calculator_evaluate(const char *expr, double *out_result, bool *out_div_zero)
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
