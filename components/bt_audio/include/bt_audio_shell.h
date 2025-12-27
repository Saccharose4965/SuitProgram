#pragma once

#include "app_shell.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BT_AUDIO_SHELL_LEGEND_INIT { .slots = { SHELL_ICON_UP, SHELL_ICON_DOWN, SHELL_ICON_CUSTOM1, SHELL_ICON_CUSTOM2 } }

void bt_app_init(shell_app_context_t *ctx);
void bt_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void bt_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);

#ifdef __cplusplus
}
#endif
