#include "fft_history.h"

#include <string.h>

#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static EXT_RAM_BSS_ATTR fft_history_snapshot_t s_history;
static StaticSemaphore_t s_mutex_storage;
static SemaphoreHandle_t s_mutex;
static uint32_t s_generation;

esp_err_t fft_history_init(void)
{
    if (!s_mutex) s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_storage);
    return s_mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

void fft_history_reset(void)
{
    if (!s_mutex) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fft_history_model_reset(&s_history);
    ++s_generation;
    xSemaphoreGive(s_mutex);
}

void fft_history_update_detector(const detector_result_t *result)
{
    if (!s_mutex || !result) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_history.detector = *result;
    ++s_generation;
    xSemaphoreGive(s_mutex);
}

void fft_history_submit(const fft_history_packet_t *packet)
{
    if (!s_mutex || !packet) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    fft_history_model_push(&s_history, packet);
    ++s_generation;
    xSemaphoreGive(s_mutex);
}

uint32_t fft_history_snapshot(fft_history_snapshot_t *out)
{
    if (!out || !s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_history, sizeof(*out));
    uint32_t generation = s_generation;
    xSemaphoreGive(s_mutex);
    return generation;
}
