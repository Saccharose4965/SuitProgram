#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Blocking Tetris demo; renders to OLED and reads hardware buttons.
void tetris_run(void);
void tetris_request_stop(void);
void tetris_copy_frame(uint8_t *dst_fb, size_t dst_len);

#ifdef __cplusplus
}
#endif
