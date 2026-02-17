#pragma once

#include <stdint.h>

#include "input.h"

#ifdef __cplusplus
extern "C" {
#endif

void calculator_init(void);
void calculator_handle_input(const input_event_t *ev);
void calculator_draw(uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
