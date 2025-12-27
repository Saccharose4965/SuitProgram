#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Panel (physical) size
#define PANEL_W 128
#define PANEL_H 64

void oled_init(void);
void oled_clear(void);

// Full-screen blit for 128x64 UIs
void oled_blit_full(const uint8_t *buf);

// 64x64 helpers for your boot animation (buffer = 64x64)
void oled_blit64_center(const uint8_t *buf);
void oled_blit64_offset(const uint8_t *buf, int xoff_px);

// The scroll curve reused by the animation “slide out”.
int oled_calc_scroll_px_slick(int step, int total_steps);

// 3-line text renderer (centered) that pushes via oled_blit_full()
void oled_render_three_lines(const char* a, const char* b, const char* c);

// Tiny 3x5 text helper: draws into a 1bpp framebuffer (same layout as oled_blit_full)
void oled_draw_text3x5(uint8_t *fb, int x, int y, const char *text);

#ifdef __cplusplus
}
#endif
