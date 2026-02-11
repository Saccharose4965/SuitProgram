#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

// Runs the game in a blocking loop until the user long-presses to exit
// or the task is deleted. Safe to call from app_main().
void flappy_run(void);

// Signal a running flappy_run() loop to stop gracefully.
void flappy_request_stop(void);

// Feed debounced button state from the shell input layer.
void flappy_set_button_pressed(bool pressed);

#ifdef __cplusplus
}
#endif
