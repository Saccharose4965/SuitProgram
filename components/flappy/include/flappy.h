#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// Runs the game in a blocking loop until the user long-presses to exit
// or the task is deleted. Safe to call from app_main().
void flappy_run(void);

// Signal a running flappy_run() loop to stop gracefully.
void flappy_request_stop(void);

// Feed one-shot press triggers from the shell input layer.
void flappy_trigger_press(void);

// Copy latest rendered frame (PANEL_W*PANEL_H/8 bytes) into dst_fb.
void flappy_copy_frame(uint8_t *dst_fb, size_t dst_len);

#ifdef __cplusplus
}
#endif
