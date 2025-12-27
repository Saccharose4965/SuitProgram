#include "led.h"
#include "hw.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

// RMT resolution: 10 MHz -> 0.1 us per tick
#define LED_RMT_RES_HZ       10000000
#define LED_RMT_TX_TIMEOUT_MS 200

#ifndef LED_STRIP_LENGTH
#define LED_STRIP_LENGTH 140
#endif

#define LED_COUNT        LED_STRIP_LENGTH
#define LED_ON_INTENSITY 32

#ifndef __containerof
#define __containerof(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

// Minimal LED strip encoder pulled from ESP-IDF example until a public helper lands
typedef struct {
    uint32_t resolution; // Encoder resolution in Hz
} led_strip_encoder_config_t;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_strip_encoder_t;

RMT_ENCODER_FUNC_ATTR
static size_t led_strip_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data,
                               size_t data_size, rmt_encode_state_t *ret_state)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t led_strip_encoder_del(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

RMT_ENCODER_FUNC_ATTR
static esp_err_t led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    led_strip_encoder_t *led_encoder = __containerof(encoder, led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    led_strip_encoder_t *led_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, "led_enc", "invalid arg");
    led_encoder = rmt_alloc_encoder_mem(sizeof(led_strip_encoder_t));
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, "led_enc", "no mem");
    led_encoder->base.encode = led_strip_encode;
    led_encoder->base.del = led_strip_encoder_del;
    led_encoder->base.reset = led_strip_encoder_reset;

    // WS2815 timing (≈800 kHz): T0H≈0.3us / T0L≈1.2us, T1H≈1.2us / T1L≈0.3us
    rmt_bytes_encoder_config_t bytes_cfg = {
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
        .flags.msb_first = 1
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_cfg, &led_encoder->bytes_encoder), err, "led_enc", "bytes");

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_cfg, &led_encoder->copy_encoder), err, "led_enc", "copy");

    // WS2815 needs a longer reset (>280 us). Use 300 us.
    uint32_t reset_ticks = config->resolution / 1000000 * 300;
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    led_encoder->state = RMT_ENCODING_RESET;

    *ret_encoder = &led_encoder->base;
    return ESP_OK;
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}

static bool s_inited = false;
static bool s_state = false;
static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static StaticSemaphore_t s_lock_buf;
static SemaphoreHandle_t s_lock = NULL;

// Travelling pulse visualization state
typedef struct {
    float pos;
    float amp;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool active;
} led_pulse_t;

static led_pulse_t s_pulses[4];
static uint8_t *s_frame = NULL;        // GRB data for entire strip
static StaticSemaphore_t s_pulse_lock_buf;
static SemaphoreHandle_t s_pulse_lock = NULL;
static TaskHandle_t s_pulse_task = NULL;
static bool s_pulse_was_on = false;
static uint8_t *s_pulse_frame = NULL;

// Pulse tuning: faster, no damping
static const float kLedPulseSpeed = 220.0f;
static const float kLedPulseWidth = 10.0f;
static const float kLedPulseDecay = 1.0f;   // no decay
static const float kLedPulseFloor = 0.001f; // effectively off threshold
static const uint8_t kLedPulseIntensity = 96;
static const float kLedPulseDt = 1.0f / 62.5f; // match audio hop cadence

static inline esp_err_t led_lock(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
        if (!s_lock) return ESP_ERR_NO_MEM;
    }
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS)) == pdTRUE
           ? ESP_OK
           : ESP_ERR_TIMEOUT;
}

static inline void led_unlock(void)
{
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

static esp_err_t led_flush_locked(const uint8_t *grb, size_t len_bytes, bool lock_held){
    if (!s_chan || !s_encoder) return ESP_ERR_INVALID_STATE;
    bool locked_here = false;
    if (!lock_held) {
        ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
        locked_here = true;
    }
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    esp_err_t err = rmt_transmit(s_chan, s_encoder, grb, len_bytes, &tx_cfg);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS));
    }
    if (locked_here) {
        led_unlock();
    }
    return err;
}

static inline esp_err_t led_flush(const uint8_t *grb, size_t len_bytes){
    return led_flush_locked(grb, len_bytes, false);
}

esp_err_t led_init(void){
    if (s_inited) return ESP_OK;
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    if (s_inited) { // double-checked after taking lock
        led_unlock();
        return ESP_OK;
    }
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = (gpio_num_t)PIN_LED_STRIP_A,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = LED_RMT_RES_HZ,
        .trans_queue_depth = 1, // serialized with a mutex; keep queue shallow
        .flags = { .with_dma = false } // DMA not supported on all targets; stick to PIO
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_chan);
    if (err != ESP_OK) { led_unlock(); return err; }
    led_strip_encoder_config_t enc_cfg = { .resolution = LED_RMT_RES_HZ };
    err = rmt_new_led_strip_encoder(&enc_cfg, &s_encoder);
    if (err != ESP_OK) {
        rmt_del_channel(s_chan);
        s_chan = NULL;
        led_unlock();
        return err;
    }
    err = rmt_enable(s_chan);
    if (err != ESP_OK) {
        rmt_del_encoder(s_encoder);
        s_encoder = NULL;
        rmt_del_channel(s_chan);
        s_chan = NULL;
        led_unlock();
        return err;
    }

    if (!s_frame) {
        s_frame = heap_caps_malloc(LED_COUNT * 3, MALLOC_CAP_DEFAULT);
    }
    if (!s_frame) {
        led_unlock();
        return ESP_ERR_NO_MEM;
    }
    // Clear strip
    memset(s_frame, 0, LED_COUNT * 3);
    err = led_flush_locked(s_frame, LED_COUNT * 3, true);
    if (err != ESP_OK) {
        rmt_disable(s_chan);
        rmt_del_encoder(s_encoder);
        rmt_del_channel(s_chan);
        s_chan = NULL;
        s_encoder = NULL;
        led_unlock();
        return err;
    }
    s_state = false;
    s_inited = true;
    if (!s_pulse_lock){
        s_pulse_lock = xSemaphoreCreateMutexStatic(&s_pulse_lock_buf);
        if (!s_pulse_lock){
            led_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_pulse_frame) {
        s_pulse_frame = heap_caps_malloc(LED_COUNT * 3, MALLOC_CAP_DEFAULT);
        if (!s_pulse_frame) {
            led_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    led_unlock();
    return ESP_OK;
}

static esp_err_t led_send_all(uint8_t r, uint8_t g, uint8_t b){
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    for (int i=0;i<LED_COUNT;++i){
        s_frame[i*3 + 0] = g;
        s_frame[i*3 + 1] = r;
        s_frame[i*3 + 2] = b;
    }
    esp_err_t err = led_flush_locked(s_frame, LED_COUNT * 3, true);
    led_unlock();
    return err;
}

esp_err_t led_set(bool on){
    if (!s_inited){
        esp_err_t err = led_init();
        if (err != ESP_OK) return err;
    }
    s_state = on;
    uint8_t val = on ? LED_ON_INTENSITY : 0;
    return led_send_all(val, val, val);
}

esp_err_t led_toggle(void){
    return led_set(!s_state);
}

esp_err_t led_show_pixels(const uint8_t *grb, size_t count){
    if (!grb) return ESP_ERR_INVALID_ARG;
    if (count > LED_COUNT) count = LED_COUNT;
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    memcpy(s_frame, grb, count * 3);
    esp_err_t err = led_flush_locked(s_frame, LED_COUNT * 3, true);
    led_unlock();
    return err;
}
 
static bool led_pulse_lock(void){
    if (!s_pulse_lock) return false;
    return xSemaphoreTake(s_pulse_lock, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS)) == pdTRUE;
}

static void led_pulse_unlock(void){
    if (s_pulse_lock){
        xSemaphoreGive(s_pulse_lock);
    }
}

static void led_pulse_spawn(uint8_t r, uint8_t g, uint8_t b){
    if (!led_pulse_lock()) return;
    led_pulse_t *slot = NULL;
    float weakest = FLT_MAX;
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        led_pulse_t *p = &s_pulses[i];
        if (!p->active){ slot = p; break; }
        float score = p->amp + 0.001f * p->pos;
        if (score < weakest){ weakest = score; slot = p; }
    }
    if (slot){
        slot->pos = -kLedPulseWidth;
        slot->amp = 1.0f;
        slot->active = true;
        slot->r = r;
        slot->g = g;
        slot->b = b;
    }
    led_pulse_unlock();
}

static bool led_pulse_step_locked(void){
    bool any_active = false;
    memset(s_pulse_frame, 0, LED_COUNT * 3);
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        led_pulse_t *p = &s_pulses[i];
        if (!p->active) continue;
        p->pos += kLedPulseSpeed * kLedPulseDt;
        p->amp *= kLedPulseDecay;
        if (p->pos > (float)LED_COUNT + kLedPulseWidth || p->amp < kLedPulseFloor){
            p->active = false;
            continue;
        }
        any_active = true;
        for (int led = 0; led < LED_COUNT; ++led){
            float d = fabsf((float)led - p->pos);
            float w = expf(-d / kLedPulseWidth);
            float v = p->amp * w;
            if (v <= 0.0f) continue;
            float add = v * (float)kLedPulseIntensity;
            int idx = led * 3;
            float next_g = (float)s_pulse_frame[idx + 0] + add * ((float)p->g / 255.0f);
            float next_r = (float)s_pulse_frame[idx + 1] + add * ((float)p->r / 255.0f);
            float next_b = (float)s_pulse_frame[idx + 2] + add * ((float)p->b / 255.0f);
            if (next_g > 255.0f) next_g = 255.0f;
            if (next_r > 255.0f) next_r = 255.0f;
            if (next_b > 255.0f) next_b = 255.0f;
            s_pulse_frame[idx + 0] = (uint8_t)next_g;
            s_pulse_frame[idx + 1] = (uint8_t)next_r;
            s_pulse_frame[idx + 2] = (uint8_t)next_b;
        }
    }
    return any_active;
}

static void led_pulse_task(void *arg){
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(16);
    while (1){
        bool active = false;
        if (led_pulse_lock()){
            active = led_pulse_step_locked();
            led_pulse_unlock();
        }
        if (active){
            if (led_show_pixels(s_pulse_frame, LED_COUNT) == ESP_OK){
                s_pulse_was_on = true;
            }
        } else if (s_pulse_was_on){
            memset(s_pulse_frame, 0, LED_COUNT * 3);
            if (led_show_pixels(s_pulse_frame, LED_COUNT) == ESP_OK){
                s_pulse_was_on = false;
            }
        }
        vTaskDelay(delay_ticks);
    }
}

static void led_pulse_start_task(void){
    if (s_pulse_task) return;
    xTaskCreate(led_pulse_task, "ledpulse", 2048, NULL, 4, &s_pulse_task);
}

void sendpulse(uint8_t r, uint8_t g, uint8_t b){
    if (!s_inited){
        if (led_init() != ESP_OK){
            return;
        }
    }
    led_pulse_start_task();
    led_pulse_spawn(r, g, b);
}
