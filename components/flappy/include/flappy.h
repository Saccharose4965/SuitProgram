#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// Runs the game in a blocking loop until the user long-presses to exit
// or the task is deleted. Safe to call from app_main().
void flappy_run(void);

// Signal a running flappy_run() loop to stop gracefully.
void flappy_request_stop(void);

#ifdef __cplusplus
}
#endif
