#pragma once

#include "app_shell.h"

void synth_init(shell_app_context_t *ctx);
void synth_deinit(shell_app_context_t *ctx);
void synth_input(shell_app_context_t *ctx, const input_event_t *ev);
void synth_tick(shell_app_context_t *ctx, float dt);
void synth_draw(shell_app_context_t *ctx, uint8_t *fb,
                int x, int y, int w, int h);
