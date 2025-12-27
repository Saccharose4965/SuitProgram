#pragma once

#include <stdint.h>
#include "input.h"
#include "link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*pong_request_switch_fn)(const char *id, void *user_data);

void pong_set_request_switch(pong_request_switch_fn fn, void *user_data);

void pong_app_init(void);
void pong_app_handle_input(const input_event_t *ev);
void pong_app_tick(float dt_sec);
void pong_app_draw(uint8_t *fb, int x, int y, int w, int h);

void pong_handle_link_frame(link_msg_type_t type, const uint8_t *payload, size_t len);

#ifdef __cplusplus
}
#endif
