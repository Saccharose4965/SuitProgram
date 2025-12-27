#include "audio_capture.h"
#include "hw.h"

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"

static const char *TAG = "audio_capture";

// ------------------------ Config ---------------------------------------

#define AUDIO_CAPTURE_DEFAULT_RING_SAMPLES   (48000 / 2)  // 0.5 s @ 48 kHz
#define AUDIO_CAPTURE_DMA_SAMPLES            256          // per I2S read, mono

// ------------------------ State ----------------------------------------

static int16_t           *s_ring          = NULL;  // PSRAM ring buffer (int16)
static size_t             s_ring_capacity = 0;     // in samples
static size_t             s_write_index   = 0;     // next write position
static size_t             s_samples_total = 0;     // debug/stat only

static TaskHandle_t       s_task_handle   = NULL;
static volatile bool      s_running       = false;

static const size_t       s_sample_rate   = 48000;

// ------------------------ Helpers --------------------------------------

static inline bool capture_is_ready(void)
{
    return (s_ring != NULL) && (s_ring_capacity > 0);
}

// 24-bit -> 16-bit down-conversion helper (I2S std API gives 32-bit aligned)
static inline int16_t convert_24_to_16(int32_t sample24)
{
    // Samples are sign-extended 24-bit left-justified in 32 bits.
    // Shifting right by 8 keeps the sign and maps to 16-bit range.
    return (int16_t)(sample24 >> 8);
}

// ------------------------ Capture task ---------------------------------

static void audio_capture_task(void *arg)
{
    (void)arg;

    // Ensure I2S RX is initialized
    if (hw_i2s_init_rx_24bit_mono_48k() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S RX, capture task exiting");
        vTaskDelete(NULL);
        return;
    }

    i2s_chan_handle_t rx = hw_i2s_get_rx();
    if (!rx) {
        ESP_LOGE(TAG, "I2S RX handle is NULL");
        vTaskDelete(NULL);
        return;
    }

    // DMA read buffer in internal RAM: 32-bit words for 24-bit samples
    int32_t dma_buf[AUDIO_CAPTURE_DMA_SAMPLES];

    ESP_LOGI(TAG, "Audio capture task started (ring=%u samples)",
             (unsigned)s_ring_capacity);

    s_running = true;

    while (s_running) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_channel_read(
            rx,
            dma_buf,
            sizeof(dma_buf),
            &bytes_read,
            portMAX_DELAY
        );

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_read failed: %s", esp_err_to_name(err));
            continue;
        }

        size_t samples = bytes_read / sizeof(int32_t);
        if (samples == 0) {
            continue;
        }

        // Convert and push into ring
        for (size_t i = 0; i < samples; ++i) {
            int16_t s16 = convert_24_to_16(dma_buf[i]);
            s_ring[s_write_index] = s16;

            s_write_index++;
            if (s_write_index >= s_ring_capacity) {
                s_write_index = 0;
            }
        }

        s_samples_total += samples;
    }

    ESP_LOGI(TAG, "Audio capture task stopping");
    vTaskDelete(NULL);
}

// ------------------------ Public API -----------------------------------

esp_err_t audio_capture_init(size_t ring_capacity_samples)
{
    if (capture_is_ready()) {
        // Already initialized
        return ESP_OK;
    }

    if (ring_capacity_samples == 0) {
        ring_capacity_samples = AUDIO_CAPTURE_DEFAULT_RING_SAMPLES;
    }

    // Allocate ring in PSRAM (int16 samples)
    size_t bytes = ring_capacity_samples * sizeof(int16_t);
    int16_t *buf = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to alloc %u bytes for capture ring in PSRAM",
                 (unsigned)bytes);
        return ESP_ERR_NO_MEM;
    }

    memset(buf, 0, bytes);
    s_ring          = buf;
    s_ring_capacity = ring_capacity_samples;
    s_write_index   = 0;
    s_samples_total = 0;

    ESP_LOGI(TAG, "Audio capture init: ring=%u samples (%.2f s @ %u Hz)",
             (unsigned)s_ring_capacity,
             (double)s_ring_capacity / (double)s_sample_rate,
             (unsigned)s_sample_rate);

    return ESP_OK;
}

esp_err_t audio_capture_start(void)
{
    if (!capture_is_ready()) {
        ESP_LOGE(TAG, "capture_start: init not done");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_task_handle) {
        // Already running
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        audio_capture_task,
        "audio_capture",
        4096,          // stack size; adjust if needed
        NULL,
        configMAX_PRIORITIES - 2,   // high priority, but not max
        &s_task_handle,
        1              // pin to core 1 (leave core 0 more for Wi-Fi)
    );

    if (ok != pdPASS) {
        s_task_handle = NULL;
        ESP_LOGE(TAG, "Failed to create audio_capture task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t audio_capture_stop(void)
{
    if (!s_task_handle) {
        return ESP_OK;
    }
    s_running = false;

    // Let task exit; we don't join in FreeRTOS, but give it time.
    vTaskDelay(pdMS_TO_TICKS(20));
    s_task_handle = NULL;
    return ESP_OK;
}

esp_err_t audio_capture_read_latest(int16_t *dst,
                                    size_t   max_samples,
                                    size_t  *out_samples)
{
    if (!dst || max_samples == 0) return ESP_ERR_INVALID_ARG;
    if (!capture_is_ready() || !s_task_handle) return ESP_ERR_INVALID_STATE;

    size_t cap = s_ring_capacity;
    if (max_samples > cap) {
        max_samples = cap;
    }

    // Snapshot the write index to avoid races
    size_t wr = s_write_index;

    // Start position of the window
    size_t start;
    if (wr >= max_samples) {
        start = wr - max_samples;
    } else {
        start = cap + wr - max_samples;
    }

    // Two-part copy if wrap
    size_t first_part = cap - start;
    if (first_part >= max_samples) {
        // One contiguous chunk
        memcpy(dst, &s_ring[start], max_samples * sizeof(int16_t));
    } else {
        // Wrap-around
        size_t second_part = max_samples - first_part;
        memcpy(dst, &s_ring[start], first_part * sizeof(int16_t));
        memcpy(dst + first_part, &s_ring[0], second_part * sizeof(int16_t));
    }

    if (out_samples) {
        *out_samples = max_samples;
    }
    return ESP_OK;
}

esp_err_t audio_capture_get_stats(audio_capture_stats_t *out_stats)
{
    if (!out_stats) return ESP_ERR_INVALID_ARG;
    if (!capture_is_ready()) return ESP_ERR_INVALID_STATE;

    out_stats->sample_rate_hz  = s_sample_rate;
    out_stats->ring_capacity   = s_ring_capacity;
    out_stats->samples_written = s_samples_total;
    return ESP_OK;
}

