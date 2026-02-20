// 1bpp 2048 renderer for the 64x128 OLED.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Direction enum matches the PNG layout tooling.
typedef enum {
    T48_DIR_LEFT  = 0,
    T48_DIR_RIGHT = 1,
    T48_DIR_UP    = 2,
    T48_DIR_DOWN  = 3,
} t48_dir_t;

// Renders a board state with optional motion overlay.
// - grid_old/new hold tile values (2^n, 0 == empty).
// - motion is a shift count in “cells” for each origin cell in grid_old
//   (positive toward the slide direction).
// - dir selects draw order; moving tiles are drawn after stationary ones by
//   iterating opposite the slide direction.
// - lerp is 0..1 progress for the animation (0 = start, 1 = landed).
void t48_render(const uint16_t grid_old[4][4],
                const uint16_t grid_new[4][4],
                const int8_t  motion[4][4],
                t48_dir_t dir,
                float lerp);

// Self-contained 2048 demo (handles input, animation, rendering).
// Call t48_game_init() once after OLED/inputs are ready, then call t48_game_tick()
// regularly (e.g., every ~12–16 ms) from your main loop.
void t48_game_init(void);
void t48_game_deinit(void);
void t48_game_tick(void);
void t48_game_copy_frame(uint8_t *dst_fb, size_t dst_len);

#ifdef __cplusplus
}
#endif
