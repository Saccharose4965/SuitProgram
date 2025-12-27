#pragma once

#include <stdint.h>

#include "input.h"
#include "shell_ui.h"

#ifdef __cplusplus
extern "C" {
#endif

struct shell_app_context;

extern const shell_legend_t KEYBOARD_LEGEND;

void keyboard_app_init(struct shell_app_context *ctx);
void keyboard_app_handle_input(struct shell_app_context *ctx, const input_event_t *ev);
void keyboard_app_tick(struct shell_app_context *ctx, float dt_sec);
void keyboard_app_draw(struct shell_app_context *ctx, uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
