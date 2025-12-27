#pragma once

#include <stdint.h>
#include "input.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*snake_request_switch_fn)(const char *id, void *user_data);

void snake_set_request_switch(snake_request_switch_fn fn, void *user_data);

void snake_app_init(void);
void snake_app_handle_input(const input_event_t *ev);
void snake_app_tick(float dt_sec);
void snake_app_draw(uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
