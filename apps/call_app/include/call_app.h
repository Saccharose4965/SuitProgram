#pragma once

#include "app_shell.h"

void call_app_init(shell_app_context_t *ctx);
void call_app_deinit(shell_app_context_t *ctx);
void call_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                   int x, int y, int w, int h);
void call_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
