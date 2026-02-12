#include "app_shell.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "twenty48.h"

#if configNUMBER_OF_CORES > 1
#define APP_TASK_CORE 1
#else
#define APP_TASK_CORE 0
#endif

static TaskHandle_t s_t2048_task = NULL;
static volatile bool s_t2048_stop = false;

const shell_legend_t T2048_LEGEND = {
    .slots = { SHELL_ICON_LEFT, SHELL_ICON_UP, SHELL_ICON_RIGHT, SHELL_ICON_DOWN },
};

static void t2048_task_fn(void *arg)
{
    (void)arg;
    t48_game_init();
    while (!s_t2048_stop) {
        t48_game_tick();
        vTaskDelay(pdMS_TO_TICKS(12));
    }
    s_t2048_task = NULL;
    vTaskDelete(NULL);
}

void t2048_stub_handle_input(shell_app_context_t *ctx, const input_event_t *ev)
{
    (void)ctx;
    (void)ev;
}

void t2048_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    s_t2048_stop = false;
    if (!s_t2048_task) {
        xTaskCreatePinnedToCore(t2048_task_fn, "t2048", 4096, NULL, 5, &s_t2048_task, APP_TASK_CORE);
    }
}

void t2048_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    s_t2048_stop = true;
    for (int i = 0; i < 50 && s_t2048_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_t2048_task = NULL;
}
