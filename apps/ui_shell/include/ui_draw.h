#pragma once
#include "oled.h"
#ifdef __cplusplus
extern "C" {
#endif
int oled_calc_scroll_px_slick(int step, int total_steps);
void oled_render_three_lines(const char *a, const char *b, const char *c);
void oled_draw_text3x5(uint8_t *fb, int x, int y, const char *text);
#ifdef __cplusplus
}
#endif
