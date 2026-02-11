// dual_ws2815_rmt.c
// ESP-IDF (v5+/v6.x) – classic ESP32 (WROVER-E) – two WS2815/WS2812-style strips
// GPIO32 + GPIO33, driven concurrently using two RMT TX channels.
// Drop this into main/ and set it as your app_main compilation unit.

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

// --------- User config ----------
#define STRIP_A_GPIO        GPIO_NUM_32
#define STRIP_B_GPIO        GPIO_NUM_33

#define LEDS_PER_STRIP      500
#define PIXEL_BYTES         3              // GRB
#define FRAME_BYTES         (LEDS_PER_STRIP * PIXEL_BYTES)

// RMT resolution: 10 MHz => 0.1 us / tick
#define RMT_RES_HZ          10000000

// WS281x/WS2815 timing @ ~800kHz (approx):
// T0H≈0.3us T0L≈1.2us, T1H≈1.2us T1L≈0.3us. Reset >= 280us (use 300us).
#define T0H_US              0.3f
#define T0L_US              1.2f
#define T1H_US              1.2f
#define T1L_US              0.3f
#define RESET_US            300

// Max time to wait for a strip transmit to finish
#define TX_WAIT_MS          200

static const char *TAG = "dual_ws2815";

// ---------- Minimal stateful LED strip encoder (based on ESP-IDF example pattern) ----------
typedef struct {
    uint32_t resolution_hz;
} led_strip_encoder_config_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_strip_encoder_t;

#ifndef __containerof
#define __containerof(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

static size_t led_strip_encode(rmt_encoder_t *encoder,
                              rmt_channel_handle_t channel,
                              const void *primary_data,
                              size_t data_size,
                              rmt_encode_state_t *ret_state)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t out_state = RMT_ENCODING_RESET;
    size_t encoded = 0;

    switch (led_encoder->state) {
    case 0: // RGB bytes
        encoded += led_encoder->bytes_encoder->encode(
            led_encoder->bytes_encoder, channel, primary_data, data_size, &session_state);

        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            out_state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // fallthrough
    case 1: // reset code
        session_state = RMT_ENCODING_RESET;
        encoded += led_encoder->copy_encoder->encode(
            led_encoder->copy_encoder, channel, &led_encoder->reset_code, sizeof(led_encoder->reset_code), &session_state);

        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 0;
            out_state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            out_state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        break;
    default:
        led_encoder->state = 0;
        out_state |= RMT_ENCODING_COMPLETE;
        break;
    }

out:
    *ret_state = out_state;
    return encoded;
}

static esp_err_t led_strip_encoder_del(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    if (led_encoder->bytes_encoder) rmt_del_encoder(led_encoder->bytes_encoder);
    if (led_encoder->copy_encoder)  rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = 0;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *cfg,
                                          rmt_encoder_handle_t *ret_encoder)
{
    ESP_RETURN_ON_FALSE(cfg && ret_encoder, ESP_ERR_INVALID_ARG, TAG, "bad args");

    led_strip_encoder_t *enc = (led_strip_encoder_t *)calloc(1, sizeof(led_strip_encoder_t));
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_NO_MEM, TAG, "no mem");

    enc->base.encode = led_strip_encode;
    enc->base.del    = led_strip_encoder_del;
    enc->base.reset  = led_strip_encoder_reset;
    enc->state = 0;

    // Convert microseconds to ticks at cfg->resolution_hz
    const float ticks_per_us = (float)cfg->resolution_hz / 1000000.0f;

    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = {
            .level0 = 1,
            .duration0 = (uint16_t)(T0H_US * ticks_per_us),
            .level1 = 0,
            .duration1 = (uint16_t)(T0L_US * ticks_per_us),
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = (uint16_t)(T1H_US * ticks_per_us),
            .level1 = 0,
            .duration1 = (uint16_t)(T1L_US * ticks_per_us),
        },
        .flags.msb_first = 1,
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &enc->bytes_encoder);
    if (err != ESP_OK) { free(enc); return err; }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &enc->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(enc->bytes_encoder);
        free(enc);
        return err;
    }

    uint32_t reset_ticks = (uint32_t)(RESET_US * ticks_per_us);
    enc->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };

    *ret_encoder = &enc->base;
    return ESP_OK;
}

// ---------- Per-strip driver ----------
typedef struct {
    gpio_num_t gpio;
    rmt_channel_handle_t chan;
    rmt_encoder_handle_t enc;
    uint8_t *frame;               // GRB bytes
    SemaphoreHandle_t lock;       // guards frame + transmit start
} ws_strip_t;

static esp_err_t ws_strip_init(ws_strip_t *s, gpio_num_t gpio)
{
    ESP_RETURN_ON_FALSE(s, ESP_ERR_INVALID_ARG, TAG, "strip null");
    memset(s, 0, sizeof(*s));
    s->gpio = gpio;

    // buffer
    s->frame = (uint8_t *)heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(s->frame, ESP_ERR_NO_MEM, TAG, "frame malloc failed");
    memset(s->frame, 0, FRAME_BYTES);

    // lock
    s->lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s->lock, ESP_ERR_NO_MEM, TAG, "mutex create failed");

    // RMT channel
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .flags = { .with_dma = false },
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &s->chan), TAG, "new tx channel failed");

    // Encoder (MUST be per-strip; it is stateful)
    led_strip_encoder_config_t enc_cfg = { .resolution_hz = RMT_RES_HZ };
    ESP_RETURN_ON_ERROR(rmt_new_led_strip_encoder(&enc_cfg, &s->enc), TAG, "new encoder failed");

    ESP_RETURN_ON_ERROR(rmt_enable(s->chan), TAG, "enable channel failed");

    return ESP_OK;
}

static void ws_strip_deinit(ws_strip_t *s)
{
    if (!s) return;
    if (s->chan) {
        rmt_disable(s->chan);
        rmt_del_channel(s->chan);
    }
    if (s->enc)  rmt_del_encoder(s->enc);
    if (s->frame) free(s->frame);
    if (s->lock) vSemaphoreDelete(s->lock);
    memset(s, 0, sizeof(*s));
}

static inline void ws_strip_set_all(ws_strip_t *s, uint8_t r, uint8_t g, uint8_t b)
{
    // frame is GRB order
    for (int i = 0; i < LEDS_PER_STRIP; i++) {
        int idx = i * 3;
        s->frame[idx + 0] = g;
        s->frame[idx + 1] = r;
        s->frame[idx + 2] = b;
    }
}

static esp_err_t ws_strip_start_tx(ws_strip_t *s)
{
    ESP_RETURN_ON_FALSE(s && s->chan && s->enc && s->frame, ESP_ERR_INVALID_STATE, TAG, "strip not ready");

    // NOTE: rmt_transmit() is asynchronous; we do not wait here.
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    return rmt_transmit(s->chan, s->enc, s->frame, FRAME_BYTES, &tx_cfg);
}

static esp_err_t ws_strip_wait_done(ws_strip_t *s, TickType_t ticks)
{
    ESP_RETURN_ON_FALSE(s && s->chan, ESP_ERR_INVALID_STATE, TAG, "strip not ready");
    return rmt_tx_wait_all_done(s->chan, ticks);
}

// ---------- Demo: concurrent refresh loop ----------
void app_main(void)
{
    ws_strip_t A, B;
    ESP_ERROR_CHECK(ws_strip_init(&A, STRIP_A_GPIO));
    ESP_ERROR_CHECK(ws_strip_init(&B, STRIP_B_GPIO));

    // Clear both once (concurrently)
    xSemaphoreTake(A.lock, portMAX_DELAY);
    xSemaphoreTake(B.lock, portMAX_DELAY);
    ws_strip_set_all(&A, 0, 0, 0);
    ws_strip_set_all(&B, 0, 0, 0);
    ESP_ERROR_CHECK(ws_strip_start_tx(&A));
    ESP_ERROR_CHECK(ws_strip_start_tx(&B));
    xSemaphoreGive(B.lock);
    xSemaphoreGive(A.lock);
    ESP_ERROR_CHECK(ws_strip_wait_done(&A, pdMS_TO_TICKS(TX_WAIT_MS)));
    ESP_ERROR_CHECK(ws_strip_wait_done(&B, pdMS_TO_TICKS(TX_WAIT_MS)));

    // Simple test animation: alternate colors, updating both strips concurrently
    while (1) {
        // Frame 1
        xSemaphoreTake(A.lock, portMAX_DELAY);
        xSemaphoreTake(B.lock, portMAX_DELAY);

        ws_strip_set_all(&A, 32, 0, 0);  // red-ish
        ws_strip_set_all(&B, 0, 0, 32);  // blue-ish

        ESP_ERROR_CHECK(ws_strip_start_tx(&A));
        ESP_ERROR_CHECK(ws_strip_start_tx(&B));

        xSemaphoreGive(B.lock);
        xSemaphoreGive(A.lock);

        // Wait for both (this wait is where you can choose to block only your LED task)
        ESP_ERROR_CHECK(ws_strip_wait_done(&A, pdMS_TO_TICKS(TX_WAIT_MS)));
        ESP_ERROR_CHECK(ws_strip_wait_done(&B, pdMS_TO_TICKS(TX_WAIT_MS)));

        vTaskDelay(pdMS_TO_TICKS(250));

        // Frame 2
        xSemaphoreTake(A.lock, portMAX_DELAY);
        xSemaphoreTake(B.lock, portMAX_DELAY);

        ws_strip_set_all(&A, 0, 32, 0);  // green-ish
        ws_strip_set_all(&B, 32, 32, 0); // yellow-ish

        ESP_ERROR_CHECK(ws_strip_start_tx(&A));
        ESP_ERROR_CHECK(ws_strip_start_tx(&B));

        xSemaphoreGive(B.lock);
        xSemaphoreGive(A.lock);

        ESP_ERROR_CHECK(ws_strip_wait_done(&A, pdMS_TO_TICKS(TX_WAIT_MS)));
        ESP_ERROR_CHECK(ws_strip_wait_done(&B, pdMS_TO_TICKS(TX_WAIT_MS)));

        vTaskDelay(pdMS_TO_TICKS(250));
    }

    // not reached
    // ws_strip_deinit(&A);
    // ws_strip_deinit(&B);
}
