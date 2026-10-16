#pragma once
#include "app_shell.h"
#include "link.h"
void call_app_init(shell_app_context_t *ctx);
void call_app_deinit(shell_app_context_t *ctx);
void call_app_draw(shell_app_context_t *ctx, uint8_t *fb, int x, int y, int w, int h);
void call_app_handle_input(shell_app_context_t *ctx, const input_event_t *ev);
void call_app_receive(link_msg_type_t type, const uint8_t *src, const uint8_t *data, size_t len);
