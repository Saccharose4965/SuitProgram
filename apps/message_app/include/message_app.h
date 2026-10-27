#pragma once

#include "app_shell.h"

void message_app_init(shell_app_context_t *ctx);
void message_app_deinit(shell_app_context_t *ctx);
void message_app_handle_input(shell_app_context_t *ctx,
                              const input_event_t *ev);
void message_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                      int x, int y, int w, int h);
extern shell_legend_t MESSAGE_LEGEND;
