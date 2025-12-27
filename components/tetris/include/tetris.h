#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Blocking Tetris demo; renders to OLED and reads hardware buttons.
void tetris_run(void);
void tetris_request_stop(void);

#ifdef __cplusplus
}
#endif
