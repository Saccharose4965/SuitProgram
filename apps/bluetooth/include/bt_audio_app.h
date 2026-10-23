#pragma once

#include <stdint.h>
#include "input.h"

#ifdef __cplusplus
extern "C" {
#endif

void bt_audio_app_init(void);
void bt_audio_app_handle_input(const input_event_t *ev);
void bt_audio_app_draw(uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
