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

#ifdef __cplusplus
}
#endif
