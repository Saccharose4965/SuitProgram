#include "led_strip.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define LED_RMT_RES_HZ 10000000
#define LED_RMT_MEM_SYMBOLS 256
#define LED_RMT_TX_TIMEOUT_MS 1000

#ifndef __containerof
#define __containerof(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

typedef struct {
    uint32_t resolution;
} strip_encoder_config_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} strip_encoder_t;

typedef struct {
    gpio_num_t gpio;
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
} strip_output_t;

static const char *TAG = "led_strip";
static strip_output_t s_outputs[LED_STRIP_OUTPUT_COUNT];
static StaticSemaphore_t s_lock_storage;
static SemaphoreHandle_t s_lock;
static bool s_initialized;

RMT_ENCODER_FUNC_ATTR
static size_t encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                     const void *data, size_t size,
                     rmt_encode_state_t *result)
{
    strip_encoder_t *strip = __containerof(encoder, strip_encoder_t, base);
    rmt_encode_state_t session = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t symbols = 0;

    switch (strip->state) {
        case 0:
            symbols += strip->bytes_encoder->encode(
                strip->bytes_encoder, channel, data, size, &session);
            if (session & RMT_ENCODING_COMPLETE) strip->state = 1;
            if (session & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
                break;
            }
            /* fall through */
        case 1:
            symbols += strip->copy_encoder->encode(
                strip->copy_encoder, channel, &strip->reset_code,
                sizeof(strip->reset_code), &session);
            if (session & RMT_ENCODING_COMPLETE) {
                strip->state = 0;
                state |= RMT_ENCODING_COMPLETE;
            }
            if (session & RMT_ENCODING_MEM_FULL)
                state |= RMT_ENCODING_MEM_FULL;
            break;
    }
    *result = state;
    return symbols;
}

static esp_err_t delete_encoder(rmt_encoder_t *encoder)
{
    strip_encoder_t *strip = __containerof(encoder, strip_encoder_t, base);
    rmt_del_encoder(strip->bytes_encoder);
    rmt_del_encoder(strip->copy_encoder);
    free(strip);
    return ESP_OK;
}

RMT_ENCODER_FUNC_ATTR
static esp_err_t reset_encoder(rmt_encoder_t *encoder)
{
    strip_encoder_t *strip = __containerof(encoder, strip_encoder_t, base);
    rmt_encoder_reset(strip->bytes_encoder);
    rmt_encoder_reset(strip->copy_encoder);
    strip->state = 0;
    return ESP_OK;
}

static esp_err_t create_encoder(const strip_encoder_config_t *config,
                                rmt_encoder_handle_t *result)
{
    if (!config || !result) return ESP_ERR_INVALID_ARG;
    strip_encoder_t *strip = rmt_alloc_encoder_mem(sizeof(*strip));
    if (!strip) return ESP_ERR_NO_MEM;

    strip->base.encode = encode;
    strip->base.del = delete_encoder;
    strip->base.reset = reset_encoder;

    rmt_bytes_encoder_config_t bytes = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 0.3 * config->resolution / 1000000,
            .level1 = 0,
            .duration1 = 1.2 * config->resolution / 1000000,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 1.2 * config->resolution / 1000000,
            .level1 = 0,
            .duration1 = 0.3 * config->resolution / 1000000,
        },
        .flags.msb_first = 1,
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes, &strip->bytes_encoder);
    if (err != ESP_OK) goto fail;
    rmt_copy_encoder_config_t copy = {};
    err = rmt_new_copy_encoder(&copy, &strip->copy_encoder);
    if (err != ESP_OK) goto fail;

    uint32_t reset_ticks = config->resolution / 1000000 * 300;
    strip->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    *result = &strip->base;
    return ESP_OK;

fail:
    if (strip->bytes_encoder) rmt_del_encoder(strip->bytes_encoder);
    if (strip->copy_encoder) rmt_del_encoder(strip->copy_encoder);
    free(strip);
    return err;
}

static esp_err_t init_output(strip_output_t *output)
{
    if (output->channel && output->encoder) return ESP_OK;
    if (output->channel || output->encoder) return ESP_ERR_INVALID_STATE;
    rmt_tx_channel_config_t channel = {
        .gpio_num = output->gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = LED_RMT_MEM_SYMBOLS,
        .resolution_hz = LED_RMT_RES_HZ,
        .trans_queue_depth = 1,
        .flags = {.with_dma = false},
    };
    esp_err_t err = rmt_new_tx_channel(&channel, &output->channel);
    if (err != ESP_OK) return err;

    strip_encoder_config_t encoder = {.resolution = LED_RMT_RES_HZ};
    err = create_encoder(&encoder, &output->encoder);
    if (err != ESP_OK) goto fail_channel;
    err = rmt_enable(output->channel);
    if (err != ESP_OK) goto fail_encoder;
    (void)gpio_set_drive_capability(output->gpio, GPIO_DRIVE_CAP_3);
    ESP_LOGI(TAG, "GPIO%d core=%d symbols=%u", (int)output->gpio,
             xPortGetCoreID(), (unsigned)LED_RMT_MEM_SYMBOLS);
    return ESP_OK;

fail_encoder:
    rmt_del_encoder(output->encoder);
    output->encoder = NULL;
fail_channel:
    rmt_del_channel(output->channel);
    output->channel = NULL;
    return err;
}

esp_err_t led_strip_init(int gpio_a, int gpio_b)
{
    if (s_initialized) return ESP_OK;
    if (!s_lock) s_lock = xSemaphoreCreateMutexStatic(&s_lock_storage);
    if (!s_lock) return ESP_ERR_NO_MEM;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    if (s_initialized) {
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    s_outputs[0].gpio = (gpio_num_t)gpio_a;
    s_outputs[1].gpio = (gpio_num_t)gpio_b;
    esp_err_t err = init_output(&s_outputs[0]);
    if (err == ESP_OK) err = init_output(&s_outputs[1]);
    if (err == ESP_OK) s_initialized = true;
    xSemaphoreGive(s_lock);
    return err;
}

static esp_err_t drain(void)
{
    esp_err_t result = ESP_OK;
    for (size_t i = 0; i < LED_STRIP_OUTPUT_COUNT; ++i) {
        esp_err_t err = rmt_tx_wait_all_done(
            s_outputs[i].channel, LED_RMT_TX_TIMEOUT_MS);
        if (result == ESP_OK && err != ESP_OK) result = err;
    }
    return result;
}

static esp_err_t transmit(const uint8_t *frames[LED_STRIP_OUTPUT_COUNT],
                          const size_t lengths[LED_STRIP_OUTPUT_COUNT])
{
    rmt_transmit_config_t config = {.loop_count = 0};
    esp_err_t err = ESP_OK;
    for (size_t i = 0; i < LED_STRIP_OUTPUT_COUNT; ++i) {
        if (!lengths[i]) continue;
        err = rmt_transmit(s_outputs[i].channel, s_outputs[i].encoder,
                           frames[i], lengths[i] * 3u, &config);
        if (err != ESP_OK) break;
    }
    esp_err_t drained = drain();
    return err != ESP_OK ? err : drained;
}

esp_err_t led_strip_write(const uint8_t *frames[LED_STRIP_OUTPUT_COUNT],
                          const size_t lengths[LED_STRIP_OUTPUT_COUNT])
{
    if (!frames || !lengths) return ESP_ERR_INVALID_ARG;
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    for (size_t i = 0; i < LED_STRIP_OUTPUT_COUNT; ++i) {
        if (lengths[i] && !frames[i]) return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = drain();
    if (err == ESP_OK) err = transmit(frames, lengths);
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "transmission timeout; resetting channels once");
        err = ESP_OK;
        for (size_t i = 0; i < LED_STRIP_OUTPUT_COUNT; ++i) {
            err = rmt_disable(s_outputs[i].channel);
            if (err != ESP_OK) break;
            err = rmt_enable(s_outputs[i].channel);
            if (err != ESP_OK) break;
        }
        if (err == ESP_OK) err = transmit(frames, lengths);
    }
    xSemaphoreGive(s_lock);
    return err;
}
