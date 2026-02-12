#include "app_shell.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tetris.h"

static TaskHandle_t s_tetris_task = NULL;

const shell_legend_t TETRIS_LEGEND = {
    .slots = { SHELL_ICON_BACK, SHELL_ICON_NONE, SHELL_ICON_NONE, SHELL_ICON_MENU },
};

static void tetris_task_fn(void *arg)
{
    (void)arg;
    tetris_run();
    s_tetris_task = NULL;
    vTaskDelete(NULL);
}

void tetris_app_init(shell_app_context_t *ctx)
{
    (void)ctx;
    if (!s_tetris_task) {
        xTaskCreate(tetris_task_fn, "tetris", 4096, NULL, 5, &s_tetris_task);
    }
}

void tetris_app_deinit(shell_app_context_t *ctx)
{
    (void)ctx;
    tetris_request_stop();
    for (int i = 0; i < 50 && s_tetris_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_tetris_task = NULL;
}
