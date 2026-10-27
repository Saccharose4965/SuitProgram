#pragma once

#include "app_shell.h"

extern shell_legend_t REMOTE_CONTROL_LEGEND;

void remote_control_app_init(shell_app_context_t *ctx);
void remote_control_app_deinit(shell_app_context_t *ctx);
void remote_control_app_tick(shell_app_context_t *ctx, float dt_sec);
void remote_control_app_handle_input(shell_app_context_t *ctx,
                                     const input_event_t *event);
void remote_control_app_draw(shell_app_context_t *ctx, uint8_t *fb,
                             int x, int y, int w, int h);
