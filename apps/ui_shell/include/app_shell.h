#pragma once

#include <stddef.h>
#include <stdint.h>

#include "input.h"
#include "shell_ui.h"
#include "system_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SHELL_APP_FLAG_SHOW_HUD    = 1u << 0,
    SHELL_APP_FLAG_SHOW_LEGEND = 1u << 1,
    SHELL_APP_FLAG_EXTERNAL    = 1u << 2, // app draws directly; shell skips blit
} shell_app_flags_t;

struct shell_app_context;
typedef struct shell_app_context shell_app_context_t;

typedef void (*shell_app_init_fn)(shell_app_context_t *ctx);
typedef void (*shell_app_deinit_fn)(shell_app_context_t *ctx);
typedef void (*shell_app_tick_fn)(shell_app_context_t *ctx, float dt_sec);
typedef void (*shell_app_input_fn)(shell_app_context_t *ctx, const input_event_t *ev);
typedef void (*shell_app_draw_fn)(shell_app_context_t *ctx,
                                  uint8_t *fb, int x, int y, int w, int h);

typedef struct {
    const char         *id;
    const char         *name;
    uint32_t            flags;
    const shell_legend_t *legend;
    shell_app_init_fn   init;
    shell_app_deinit_fn deinit;
    shell_app_tick_fn   tick;
    shell_app_input_fn  handle_input;
    shell_app_draw_fn   draw;
} shell_app_desc_t;

struct shell_app_context {
    const system_state_t   *state;          // snapshot for the current frame
    const shell_app_desc_t *registry;       // app registry
    size_t                  registry_count;
    size_t                  current_index;  // index within registry
    void (*request_switch)(const char *id, void *user_data);
    void *request_user_data;
};

// Launch the UI shell with the built-in registry (menu + status stubs).
void app_shell_start(void);

#ifdef __cplusplus
}
#endif
