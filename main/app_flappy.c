#include "app_shell.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flappy.h"

static TaskHandle_t s_flappy_task = NULL;
static void (*s_flappy_request_switch)(const char *id, void *user_data) = NULL;
static void *s_flappy_request_user_data = NULL;

const shell_legend_t FLAPPY_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_MENU },
};

static void flappy_task_fn(void *arg)
{
    (void)arg;
    flappy_run(); // blocking; handles its own exit via long-press
    if (s_flappy_request_switch) {
        s_flappy_request_switch("menu", s_flappy_request_user_data);
    }
    s_flappy_task = NULL;
    vTaskDelete(NULL);
}

void flappy_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    if (!ctx || !ev) return;
    if (ev->type == INPUT_EVENT_LONG_PRESS && ev->button == INPUT_BTN_A) {
        ctx->request_switch("menu", ctx->request_user_data);
    }
}

void flappy_app_init(shell_app_context_t *ctx)
{
    if (ctx) {
        s_flappy_request_switch = ctx->request_switch;
        s_flappy_request_user_data = ctx->request_user_data;
    }
    if (!s_flappy_task) {
        xTaskCreate(flappy_task_fn, "flappy", 4096, NULL, 5, &s_flappy_task);
    }
}

void flappy_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    flappy_request_stop();
    // wait briefly for task to exit
    for (int i = 0; i < 50 && s_flappy_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_flappy_task = NULL;
}
