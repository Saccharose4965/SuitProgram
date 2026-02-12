#include "app_shell.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flappy.h"

static TaskHandle_t s_flappy_task = NULL;

const shell_legend_t FLAPPY_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_MENU },
};

static void flappy_task_fn(void *arg)
{
    (void)arg;
    flappy_run(); // blocking
    s_flappy_task = NULL;
    vTaskDelete(NULL);
}

void flappy_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    if (!ev) return;
    if (ev->type == INPUT_EVENT_PRESS) {
        flappy_trigger_press();
    }
}

void flappy_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
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
