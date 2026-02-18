#include "led.h"
#include "hw.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#define LED_RMT_TX_TIMEOUT_MS 1000
#define LED_RMT_MEM_SYMBOLS   128

#ifndef LED_MAX_FPS
#define LED_MAX_FPS 30
#endif

#define LED_COUNT LED_STRIP_LENGTH

#define LED_ON_INTENSITY 32

#ifndef LED_TEST_BRIGHTNESS
#define LED_TEST_BRIGHTNESS 96
#endif

#ifndef LED_TEST_PIXELS
#define LED_TEST_PIXELS LED_COUNT
#endif

#ifndef LED_TEST_STEP_MS
#define LED_TEST_STEP_MS 180
#endif

#ifndef LED_TEST_CHASE_STEP_MS
#define LED_TEST_CHASE_STEP_MS 10
#endif

static const char *TAG = "led";

#ifndef LED_PULSE_SINGLE_MODE
#define LED_PULSE_SINGLE_MODE 0
#endif

#if CONFIG_FREERTOS_UNICORE
#define LED_TASK_CORE 0
#else
// Keep LED workers off FFT's default core (core 1).
#define LED_TASK_CORE 0
#endif

// LED TX must not be lower priority than FFT producer, otherwise frame
// submissions can stall under sustained DSP load.
#define LED_TX_TASK_PRIO    5
#define LED_PULSE_TASK_PRIO 4

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
static StaticSemaphore_t s_tx_lock_buf;
static SemaphoreHandle_t s_tx_lock = NULL;
static TaskHandle_t s_tx_task = NULL;
static uint8_t *s_tx_shadow = NULL;
static size_t s_tx_len_bytes = 0;

// Travelling pulse visualization state
typedef struct {
    float pos;
    float amp;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool active;
} led_pulse_t;

static led_pulse_t s_pulses[10];
static uint8_t *s_frame = NULL;        // Packed data for entire strip (RGB)
static StaticSemaphore_t s_pulse_lock_buf;
static SemaphoreHandle_t s_pulse_lock = NULL;
static TaskHandle_t s_pulse_task = NULL;
static bool s_pulse_was_on = false;
static uint8_t *s_pulse_frame = NULL;
static volatile led_beat_anim_t s_beat_anim = LED_BEAT_ANIM_PULSE;
static volatile bool s_beat_enabled = true;
static uint8_t s_flash_r = 0;
static uint8_t s_flash_g = 0;
static uint8_t s_flash_b = 0;
static int s_flash_ticks = 0;
static uint8_t s_wave_r = 0;
static uint8_t s_wave_g = 0;
static uint8_t s_wave_b = 0;
static float s_wave_phase = 0.0f;
static float s_wave_amp = 0.0f;
static float s_pulse_period_sec = 60.0f / 120.0f; // default 120 BPM
static int64_t s_last_pulse_spawn_us = 0;

// Pulse tuning: tempo-synced travel, no damping
static const float kLedPulseWidth = 10.0f;
static const float kLedPulseDecay = 1.0f;   // no decay
static const float kLedPulseFloor = 0.001f; // effectively off threshold
static const uint8_t kLedPulseIntensity = 96;
static const float kLedPulseDt = 1.0f / 62.5f; // match audio hop cadence
static const int kLedFlashTicks = 3; // ~48 ms at 16 ms pulse task cadence
static const float kLedPulseMinPeriodSec = 60.0f / 240.0f; // 240 BPM cap
static const float kLedPulseMaxPeriodSec = 60.0f / 40.0f;  // 40 BPM floor
static const float kLedPulseTempoSmooth = 0.25f;
static const float kLedWaveHz = 5.0f;
static const float kLedWaveDecay = 0.90f;
static const float kLedWaveFloor = 0.01f;
static const float kLedWavePi = 3.1415927f;
static const float kLedWaveTwoPi = 6.2831853f;

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

static inline esp_err_t led_tx_lock_take(void)
{
    if (!s_tx_lock) {
        s_tx_lock = xSemaphoreCreateMutexStatic(&s_tx_lock_buf);
        if (!s_tx_lock) return ESP_ERR_NO_MEM;
    }
    return xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS)) == pdTRUE
           ? ESP_OK
           : ESP_ERR_TIMEOUT;
}

static inline void led_tx_lock_give(void)
{
    if (s_tx_lock) {
        xSemaphoreGive(s_tx_lock);
    }
}

static uint8_t *led_alloc_frame(size_t bytes, const char *label)
{
    uint8_t *buf = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGW(TAG, "%s internal RAM alloc failed, falling back", label ? label : "frame");
        buf = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    }
    return buf;
}

static esp_err_t led_flush_locked(const uint8_t *grb, size_t len_bytes, bool lock_held){
    if (!s_chan || !s_encoder) return ESP_ERR_INVALID_STATE;
    bool locked_here = false;
    if (!lock_held) {
        ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
        locked_here = true;
    }
    esp_err_t lock_err = led_tx_lock_take();
    if (lock_err != ESP_OK) {
        if (locked_here) {
            led_unlock();
        }
        return lock_err;
    }
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    esp_err_t err = rmt_transmit(s_chan, s_encoder, grb, len_bytes, &tx_cfg);
    if (err == ESP_OK) {
        err = rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS));
        if (err == ESP_ERR_TIMEOUT) {
            // Recover once: reset channel state and retry the same frame.
            ESP_LOGW(TAG, "RMT TX timeout (%u bytes), retrying once", (unsigned)len_bytes);
            (void)rmt_disable(s_chan);
            (void)rmt_enable(s_chan);
            err = rmt_transmit(s_chan, s_encoder, grb, len_bytes, &tx_cfg);
            if (err == ESP_OK) {
                err = rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS));
            }
        }
    }
    led_tx_lock_give();
    if (locked_here) {
        led_unlock();
    }
    return err;
}

static esp_err_t led_submit_async_locked(const uint8_t *grb, size_t len_bytes){
    if (!grb) return ESP_ERR_INVALID_ARG;
    if (!s_inited || !s_frame) return ESP_ERR_INVALID_STATE;
    size_t max_bytes = LED_COUNT * 3;
    if (len_bytes < 3) len_bytes = 3;
    if (len_bytes > max_bytes) len_bytes = max_bytes;

    if (grb != s_frame){
        memset(s_frame, 0, max_bytes);
        memcpy(s_frame, grb, len_bytes);
    }
    // Always transmit a full frame for deterministic strip state.
    s_tx_len_bytes = max_bytes;
    if (s_tx_task){
        xTaskNotifyGive(s_tx_task);
    }
    return ESP_OK;
}

static esp_err_t led_submit_async(const uint8_t *grb, size_t len_bytes){
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    esp_err_t err = led_submit_async_locked(grb, len_bytes);
    led_unlock();
    return err;
}

static void led_tx_task(void *arg){
    (void)arg;
    const int64_t wire_frame_us = ((int64_t)LED_COUNT * 30LL) + 300LL; // 24 bits @1.25us + reset
    const int64_t fps_frame_us = 1000000LL / (int64_t)LED_MAX_FPS;
    const int64_t min_frame_us = (wire_frame_us > fps_frame_us) ? wire_frame_us : fps_frame_us;
    int64_t next_tx_us = 0;
    while (1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Coalesce bursts: we'll send the most recent frame only.
        while (ulTaskNotifyTake(pdTRUE, 0) > 0) {}

        int64_t now_us = esp_timer_get_time();
        if (next_tx_us > now_us){
            int64_t wait_us = next_tx_us - now_us;
            TickType_t wait_ticks = pdMS_TO_TICKS((wait_us + 999) / 1000);
            if (wait_ticks > 0){
                vTaskDelay(wait_ticks);
            }
        }

        // If new frames arrived while waiting, collapse them again.
        while (ulTaskNotifyTake(pdTRUE, 0) > 0) {}

        size_t len_bytes = 0;
        if (led_lock() == ESP_OK){
            len_bytes = s_tx_len_bytes;
            if (len_bytes < 3) len_bytes = 3;
            if (len_bytes > LED_COUNT * 3) len_bytes = LED_COUNT * 3;
            memcpy(s_tx_shadow, s_frame, len_bytes);
            led_unlock();
        } else {
            continue;
        }

        // Single TX task owns the channel; don't hold the shared state lock while waiting.
        (void)led_flush_locked(s_tx_shadow, len_bytes, true);
        next_tx_us = esp_timer_get_time() + min_frame_us;
    }
}

static void led_test_delay_ms(uint32_t ms){
    if (ms == 0) return;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

static esp_err_t led_test_show_prefix(uint8_t r, uint8_t g, uint8_t b, size_t lit_count){
    if (lit_count > LED_COUNT) lit_count = LED_COUNT;
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    for (size_t i = 0; i < lit_count; ++i){
        led_set_pixel_rgb(s_frame, i, r, g, b);
    }
    esp_err_t err = led_flush_locked(s_frame, LED_COUNT * 3, true);
    led_unlock();
    return err;
}

static esp_err_t led_test_show_chase(uint8_t r, uint8_t g, uint8_t b, size_t span, size_t pos){
    if (span > LED_COUNT) span = LED_COUNT;
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    if (span > 0){
        size_t p = pos % span;
        led_set_pixel_rgb(s_frame, p, r, g, b);
    }
    esp_err_t err = led_flush_locked(s_frame, LED_COUNT * 3, true);
    led_unlock();
    return err;
}

esp_err_t led_run_test_pattern(void){
    if (!s_inited){
        ESP_RETURN_ON_ERROR(led_init(), "led", "init");
    }
    if (!s_frame) return ESP_ERR_INVALID_STATE;

    size_t test_pixels = LED_TEST_PIXELS;
    if (test_pixels < 1) test_pixels = 1;
    if (test_pixels > LED_COUNT) test_pixels = LED_COUNT;
    uint8_t v = LED_TEST_BRIGHTNESS;

    ESP_RETURN_ON_ERROR(led_test_show_prefix(v, 0, 0, test_pixels), "led", "test red");
    led_test_delay_ms(LED_TEST_STEP_MS);
    ESP_RETURN_ON_ERROR(led_test_show_prefix(0, v, 0, test_pixels), "led", "test green");
    led_test_delay_ms(LED_TEST_STEP_MS);
    ESP_RETURN_ON_ERROR(led_test_show_prefix(0, 0, v, test_pixels), "led", "test blue");
    led_test_delay_ms(LED_TEST_STEP_MS);
    ESP_RETURN_ON_ERROR(led_test_show_prefix(v, v, v, test_pixels), "led", "test white");
    led_test_delay_ms(LED_TEST_STEP_MS);

    for (size_t i = 0; i < test_pixels; ++i){
        ESP_RETURN_ON_ERROR(led_test_show_chase(v, v, v, test_pixels, i), "led", "test chase");
        led_test_delay_ms(LED_TEST_CHASE_STEP_MS);
    }

    ESP_RETURN_ON_ERROR(led_test_show_prefix(0, 0, 0, LED_COUNT), "led", "test off");
    return ESP_OK;
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
        .mem_block_symbols = LED_RMT_MEM_SYMBOLS,
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
    // Max drive strength helps keep logic-high margin on longer/noisy traces.
    (void)gpio_set_drive_capability((gpio_num_t)PIN_LED_STRIP_A, GPIO_DRIVE_CAP_3);

    if (!s_frame) {
        s_frame = led_alloc_frame(LED_COUNT * 3, "s_frame");
    }
    if (!s_frame) {
        led_unlock();
        return ESP_ERR_NO_MEM;
    }
    if (!s_tx_shadow){
        s_tx_shadow = led_alloc_frame(LED_COUNT * 3, "s_tx_shadow");
        if (!s_tx_shadow){
            led_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    // Clear strip
    memset(s_frame, 0, LED_COUNT * 3);
    err = led_flush_locked(s_frame, LED_COUNT * 3, true);
    if (err != ESP_OK) {
        // Don't hard-fail init on clear timeout: allow later pulses to run with
        // shorter frames for bring-up diagnostics.
        ESP_LOGW(TAG, "Initial strip clear failed: %s", esp_err_to_name(err));
        if (err != ESP_ERR_TIMEOUT) {
            rmt_disable(s_chan);
            rmt_del_encoder(s_encoder);
            rmt_del_channel(s_chan);
            s_chan = NULL;
            s_encoder = NULL;
            led_unlock();
            return err;
        }
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
        s_pulse_frame = led_alloc_frame(LED_COUNT * 3, "s_pulse_frame");
        if (!s_pulse_frame) {
            led_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    if (!s_tx_task){
        BaseType_t ok = xTaskCreatePinnedToCore(led_tx_task, "ledtx", 3072, NULL,
                                                LED_TX_TASK_PRIO,
                                                &s_tx_task, LED_TASK_CORE);
        if (ok != pdPASS){
            led_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "LED count: %u", (unsigned)LED_COUNT);
    ESP_LOGI(TAG, "LED color order: RGB");
    ESP_LOGI(TAG, "LED workers: core=%d tx_prio=%d pulse_prio=%d",
             LED_TASK_CORE, LED_TX_TASK_PRIO, LED_PULSE_TASK_PRIO);
    led_unlock();
    return ESP_OK;
}

static esp_err_t led_send_all(uint8_t r, uint8_t g, uint8_t b){
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    for (int i = 0; i < LED_COUNT; ++i){
        led_set_pixel_rgb(s_frame, (size_t)i, r, g, b);
    }
    esp_err_t err = led_submit_async_locked(s_frame, LED_COUNT * 3);
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

esp_err_t led_show_pixels(const uint8_t *frame, size_t count){
    if (!frame) return ESP_ERR_INVALID_ARG;
    if (count > LED_COUNT) count = LED_COUNT;
    return led_submit_async(frame, count * 3);
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

static void led_pulse_update_period_locked(void){
    const int64_t now_us = esp_timer_get_time();
    if (s_last_pulse_spawn_us > 0){
        const int64_t delta_us = now_us - s_last_pulse_spawn_us;
        if (delta_us > 0){
            const float raw_period_sec = (float)delta_us / 1000000.0f;
            if (raw_period_sec >= kLedPulseMinPeriodSec && raw_period_sec <= kLedPulseMaxPeriodSec){
                s_pulse_period_sec = (1.0f - kLedPulseTempoSmooth) * s_pulse_period_sec
                                     + kLedPulseTempoSmooth * raw_period_sec;
            }
        }
    }
    s_last_pulse_spawn_us = now_us;
}

static void led_pulse_spawn_locked(uint8_t r, uint8_t g, uint8_t b, float start_pos){
#if LED_PULSE_SINGLE_MODE
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        s_pulses[i].active = false;
    }
    led_pulse_t *slot = &s_pulses[0];
#else
    led_pulse_t *slot = NULL;
    float weakest = FLT_MAX;
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        led_pulse_t *p = &s_pulses[i];
        if (!p->active){ slot = p; break; }
        float score = p->amp + 0.001f * p->pos;
        if (score < weakest){ weakest = score; slot = p; }
    }
#endif
    if (slot){
        slot->pos = start_pos;
        slot->amp = 1.0f;
        slot->active = true;
        slot->r = r;
        slot->g = g;
        slot->b = b;
    }
}

static void led_pulse_spawn(uint8_t r, uint8_t g, uint8_t b){
    if (!led_pulse_lock()) return;
    led_pulse_update_period_locked();
    led_pulse_spawn_locked(r, g, b, -kLedPulseWidth);
    led_pulse_unlock();
}

static bool led_pulse_step_locked(void){
    bool any_active = false;
    memset(s_pulse_frame, 0, LED_COUNT * 3);
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        led_pulse_t *p = &s_pulses[i];
        if (!p->active) continue;
        float period_sec = s_pulse_period_sec;
        if (period_sec < kLedPulseMinPeriodSec) period_sec = kLedPulseMinPeriodSec;
        if (period_sec > kLedPulseMaxPeriodSec) period_sec = kLedPulseMaxPeriodSec;
        const float travel_leds = (float)LED_COUNT + kLedPulseWidth;
        const float speed = travel_leds / period_sec;
        p->pos += kLedPulseDt * speed;
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
            uint8_t cur_r = 0, cur_g = 0, cur_b = 0;
            led_get_pixel_rgb(s_pulse_frame, (size_t)led, &cur_r, &cur_g, &cur_b);

            float next_r = (float)cur_r + add * ((float)p->r / 255.0f);
            float next_g = (float)cur_g + add * ((float)p->g / 255.0f);
            float next_b = (float)cur_b + add * ((float)p->b / 255.0f);
            if (next_g > 255.0f) next_g = 255.0f;
            if (next_r > 255.0f) next_r = 255.0f;
            if (next_b > 255.0f) next_b = 255.0f;
            led_set_pixel_rgb(
                s_pulse_frame,
                (size_t)led,
                (uint8_t)next_r,
                (uint8_t)next_g,
                (uint8_t)next_b
            );
        }
    }
    if (s_flash_ticks > 0){
        for (int led = 0; led < LED_COUNT; ++led){
            led_set_pixel_rgb(s_pulse_frame, (size_t)led, s_flash_r, s_flash_g, s_flash_b);
        }
        s_flash_ticks--;
        any_active = true;
    }
    if (s_wave_amp > kLedWaveFloor){
        float level = s_wave_amp * (0.5f + 0.5f * sinf(s_wave_phase));

        if (level > 0.0f){
            float wr = level * (float)s_wave_r;
            float wg = level * (float)s_wave_g;
            float wb = level * (float)s_wave_b;
            if (wr > 255.0f) wr = 255.0f;
            if (wg > 255.0f) wg = 255.0f;
            if (wb > 255.0f) wb = 255.0f;

            for (int led = 0; led < LED_COUNT; ++led){
                led_set_pixel_rgb(
                    s_pulse_frame,
                    (size_t)led,
                    (uint8_t)wr,
                    (uint8_t)wg,
                    (uint8_t)wb
                );
            }
        }

        s_wave_phase += kLedWaveTwoPi * kLedWaveHz * kLedPulseDt;
        if (s_wave_phase >= kLedWaveTwoPi){
            s_wave_phase -= kLedWaveTwoPi;
        }
        s_wave_amp *= kLedWaveDecay;
        if (s_wave_amp < kLedWaveFloor){
            s_wave_amp = 0.0f;
        }
        any_active = true;
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
            // Push the pulse frame directly (no intermediate memcpy path).
            if (led_submit_async(s_pulse_frame, LED_COUNT * 3) == ESP_OK){
                s_pulse_was_on = true;
            }
        } else if (s_pulse_was_on){
            memset(s_pulse_frame, 0, LED_COUNT * 3);
            if (led_submit_async(s_pulse_frame, LED_COUNT * 3) == ESP_OK){
                s_pulse_was_on = false;
            }
        }
        vTaskDelay(delay_ticks);
    }
}

static void led_pulse_start_task(void){
    if (s_pulse_task) return;
    xTaskCreatePinnedToCore(led_pulse_task, "ledpulse", 2048, NULL,
                            LED_PULSE_TASK_PRIO,
                            &s_pulse_task, LED_TASK_CORE);
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

int led_beat_anim_count(void){
    return 3;
}

const char *led_beat_anim_name(int idx){
    switch (idx){
        case LED_BEAT_ANIM_FLASH: return "flash";
        case LED_BEAT_ANIM_PULSE: return "pulse";
        case LED_BEAT_ANIM_WAVE: return "wave";
        default: return NULL;
    }
}

void led_beat_anim_set(led_beat_anim_t mode){
    if (mode != LED_BEAT_ANIM_FLASH && mode != LED_BEAT_ANIM_PULSE && mode != LED_BEAT_ANIM_WAVE){
        mode = LED_BEAT_ANIM_FLASH;
    }
    s_beat_anim = mode;
}

led_beat_anim_t led_beat_anim_get(void){
    return s_beat_anim;
}

void led_beat_enable(bool enabled){
    s_beat_enabled = enabled;
}

bool led_beat_enabled(void){
    return s_beat_enabled;
}

void led_trigger_beat(uint8_t r, uint8_t g, uint8_t b){
    if (!s_beat_enabled) return;
    if (!s_inited){
        if (led_init() != ESP_OK){
            return;
        }
    }
    led_pulse_start_task();

    if (s_beat_anim == LED_BEAT_ANIM_PULSE){
        if (!led_pulse_lock()) return;
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        led_pulse_unlock();
        led_pulse_spawn(r, g, b);
        return;
    }

    if (!led_pulse_lock()) return;
    for (size_t i = 0; i < sizeof(s_pulses) / sizeof(s_pulses[0]); ++i){
        s_pulses[i].active = false;
    }
    if (s_beat_anim == LED_BEAT_ANIM_WAVE){
        s_flash_ticks = 0;
        s_wave_r = r;
        s_wave_g = g;
        s_wave_b = b;
        s_wave_phase = 0.5f * kLedWavePi; // start at crest, then ring down
        s_wave_amp = 1.0f;
        led_pulse_unlock();
        return;
    }

    s_wave_amp = 0.0f;
    s_flash_r = r;
    s_flash_g = g;
    s_flash_b = b;
    s_flash_ticks = kLedFlashTicks;
    led_pulse_unlock();
}
