#include "app_shell.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "app_main";

static void shell_task(void *arg)
{
    (void)arg;
    app_shell_start();
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Run UI shell on CPU1 to keep CPU0 available for BT/Wi-Fi controller work.
    BaseType_t ok = xTaskCreatePinnedToCore(
        shell_task,
        "shell_main",
        12288,
        NULL,
        5,
        NULL,
        1
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to start shell task on CPU1");
    }
}
