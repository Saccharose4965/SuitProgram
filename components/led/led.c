#include "led.h"
#include "hw.h"
#include "led_layout.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
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
#define LED_MAX_FPS 60
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
#define LED_OUTPUT_COUNT    2

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

typedef struct {
    gpio_num_t gpio;
    rmt_channel_handle_t chan;
    rmt_encoder_handle_t encoder;
} led_output_t;

static bool s_inited = false;
static bool s_state = false;
static led_output_t s_outputs[LED_OUTPUT_COUNT] = {
    { .gpio = (gpio_num_t)PIN_LED_STRIP_A, .chan = NULL, .encoder = NULL },
    { .gpio = (gpio_num_t)PIN_LED_STRIP_B, .chan = NULL, .encoder = NULL },
};
static StaticSemaphore_t s_lock_buf;
static SemaphoreHandle_t s_lock = NULL;
static StaticSemaphore_t s_tx_lock_buf;
static SemaphoreHandle_t s_tx_lock = NULL;
static TaskHandle_t s_tx_task = NULL;
static uint8_t *s_tx_shadow = NULL;
static size_t s_tx_len_bytes = 0;
static uint8_t *s_output_frames[LED_OUTPUT_COUNT] = { NULL, NULL };

// Travelling pulse visualization state
typedef struct {
    float pos;
    float amp;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool active;
} led_pulse_t;

typedef struct {
    float head;
    float speed;
    float tail_leds;
    float amp;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    int8_t dir;
    bool active;
} led_comet_t;

typedef struct {
    float front;
    float speed;
    float amp;
    float width_scale;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool active;
} led_ring_pulse_t;

typedef struct {
    float offset;
    float speed;
    float width;
    float amp;
    float nx;
    float ny;
    float nz;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool span_mode;
    bool active;
} led_plane_sweep_t;

typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    led_point_t ring_center;
    float ring_radius;
    float radial_max;
    float ring_wave_max;
} led_geo_layout_stats_t;

static led_pulse_t s_pulses[10];
#define LED_COMET_SLOTS 6
static led_comet_t s_comets[LED_COMET_SLOTS];
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
static uint8_t s_beat_color_r = 255;
static uint8_t s_beat_color_g = 96;
static uint8_t s_beat_color_b = 0;
static uint8_t s_beat_secondary_r = 0;
static uint8_t s_beat_secondary_g = 72;
static uint8_t s_beat_secondary_b = 255;
static volatile led_color_cycle_t s_beat_color_cycle = LED_COLOR_CYCLE_STATIC;
static volatile led_color_style_t s_beat_color_style = LED_COLOR_STYLE_MONO;
static volatile led_highlight_mode_t s_beat_highlight_mode = LED_HIGHLIGHT_OFF;
static volatile bool s_beat_plane_background_enabled = false;
static volatile bool s_beat_ring_background_enabled = false;
static volatile bool s_audio_brightness_enabled = false;
static volatile led_audio_energy_range_t s_audio_energy_range = LED_AUDIO_ENERGY_RANGE_FULL;
static uint8_t s_beat_brightness = 96;
static uint32_t s_beat_duo_flip = 0;
static float s_audio_energy_phase = 0.0f;
static volatile float s_audio_raw_volume = 0.0f;
static led_beat_anim_t s_beat_random_anim = LED_BEAT_ANIM_FLASH;
static int64_t s_beat_random_next_switch_us = 0;
static int64_t s_breathe_anchor_us = 0;
static uint8_t s_spark_r = 255;
static uint8_t s_spark_g = 96;
static uint8_t s_spark_b = 0;
static uint8_t s_spark_energy[LED_COUNT];
static float s_pulse_period_sec = 60.0f / 120.0f; // default 120 BPM
static int64_t s_last_pulse_spawn_us = 0;
static EXT_RAM_BSS_ATTR led_layout_config_t s_output_layout = {0};
#define LED_GEO_RING_SLOTS 32
#define LED_GEO_PLANE_SLOTS 8
static EXT_RAM_BSS_ATTR float s_geo_ring_distance_cache[LED_COUNT];
static led_ring_pulse_t s_ring_pulses[LED_GEO_RING_SLOTS];
static led_plane_sweep_t s_plane_sweeps[LED_GEO_PLANE_SLOTS];
static EXT_RAM_BSS_ATTR led_layout_config_t s_effect_layout = {0};

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
static const float kLedPulseSnapBpmDelta = 6.0f;
static const float kLedAudioEnergyBaseSpeed = 8.5f;
#ifndef LED_BEAT_RANDOM_INTERVAL_US
#define LED_BEAT_RANDOM_INTERVAL_US (10LL * 1000000LL)
#endif
static const float kLedAudioBrightnessThreshold = 0.07f;
static const float kLedWaveHz = 5.0f;
static const float kLedWaveDecay = 0.90f;
static const float kLedWaveFloor = 0.01f;
static const float kLedWavePi = 3.1415927f;
static const float kLedWaveTwoPi = 6.2831853f;
static const int kLedSparkSeedsPerBeat = 28;
static const uint8_t kLedSparkSeedEnergy = 255;
static const uint8_t kLedSparkDecay = 208;
static const uint8_t kLedSparkFloor = 6;
static const float kLedCometTailMin = 10.0f;
static const float kLedCometTailMax = 52.0f;
static const float kLedCometDecay = 0.993f;
static const float kLedCometHeadLead = 1.5f;
static const float kLedCometBackgroundScale = 0.10f;
static const float kLedGeoRingDecay = 1.0f;
static const float kLedGeoPlaneDecay = 0.973f;
static const float kLedGeoRingSpeedUnitsPerSec = 30.0f; // layout units are treated as cm
static const float kLedGeoRingBaseWidth = 1.2f;
static const float kLedGeoRingWidthScale = 0.015f;
static const float kLedGeoPlaneSpeedFactor = 2.0f;
static const size_t kLedGeoRingTrainWagons = 4u;
static const float kLedGeoRingTrainWidthScale = 0.42f;
static const float kLedGeoPlaneBackgroundScale = 0.18f;
static const float kLedGeoRingBackgroundScale = 0.18f;
static int8_t s_comet_spawn_dir = 1;

static void led_pulse_start_task(void);
static bool led_beat_mode_uses_ring(led_beat_anim_t mode);
static bool led_beat_mode_uses_plane(led_beat_anim_t mode);
static bool led_beat_mode_uses_chase_background(led_beat_anim_t mode);

static uint8_t lerp_u8(uint8_t a, uint8_t b, float t)
{
    float v = (1.0f - t) * (float)a + t * (float)b;
    if (v < 0.0f) v = 0.0f;
    if (v > 255.0f) v = 255.0f;
    return (uint8_t)(v + 0.5f);
}

static float ease_cosine01(float t)
{
    const float kPi = 3.14159265f;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return 0.5f - 0.5f * cosf(kPi * t);
}

static void sample_palette3(uint8_t c0r, uint8_t c0g, uint8_t c0b,
                            uint8_t c1r, uint8_t c1g, uint8_t c1b,
                            uint8_t c2r, uint8_t c2g, uint8_t c2b,
                            float phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    while (phase < 0.0f) phase += 1.0f;
    while (phase >= 1.0f) phase -= 1.0f;
    float seg = phase * 3.0f;
    int idx = (int)seg;
    float t = seg - (float)idx;
    if (idx <= 0) {
        if (r) *r = lerp_u8(c0r, c1r, t);
        if (g) *g = lerp_u8(c0g, c1g, t);
        if (b) *b = lerp_u8(c0b, c1b, t);
    } else if (idx == 1) {
        if (r) *r = lerp_u8(c1r, c2r, t);
        if (g) *g = lerp_u8(c1g, c2g, t);
        if (b) *b = lerp_u8(c1b, c2b, t);
    } else {
        if (r) *r = lerp_u8(c2r, c0r, t);
        if (g) *g = lerp_u8(c2g, c0g, t);
        if (b) *b = lerp_u8(c2b, c0b, t);
    }
}

static void sample_rainbow_phase(float phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    static const uint8_t kAnchors[6][3] = {
        {255,   0,   0},
        {255, 220,   0},
        {  0, 255,   0},
        {  0, 255, 255},
        {  0,   0, 255},
        {255,   0, 255},
    };
    const float hold = 0.28f;
    const float move = 1.0f - hold;

    while (phase < 0.0f) phase += 1.0f;
    while (phase >= 1.0f) phase -= 1.0f;

    float seg = phase * 6.0f;
    int idx = (int)seg;
    float local = seg - (float)idx;
    idx %= 6;
    if (idx < 0) idx += 6;
    int next = (idx + 1) % 6;

    if (local <= hold || move <= 0.0001f) {
        if (r) *r = kAnchors[idx][0];
        if (g) *g = kAnchors[idx][1];
        if (b) *b = kAnchors[idx][2];
        return;
    }

    float t = ease_cosine01((local - hold) / move);
    if (r) *r = lerp_u8(kAnchors[idx][0], kAnchors[next][0], t);
    if (g) *g = lerp_u8(kAnchors[idx][1], kAnchors[next][1], t);
    if (b) *b = lerp_u8(kAnchors[idx][2], kAnchors[next][2], t);
}

static void sample_siren_phase(float phase, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (phase < 0.0f) phase += 1.0f;
    while (phase >= 1.0f) phase -= 1.0f;

    if (phase < 0.08f || (phase >= 0.50f && phase < 0.58f)) {
        if (r) *r = 255;
        if (g) *g = 255;
        if (b) *b = 255;
    } else if (phase < 0.50f) {
        if (r) *r = 255;
        if (g) *g = 0;
        if (b) *b = 0;
    } else {
        if (r) *r = 0;
        if (g) *g = 48;
        if (b) *b = 255;
    }
}

static void led_beat_sample_cycle_color(float t_sec, float phase_offset,
                                        uint8_t *r, uint8_t *g, uint8_t *b)
{
    switch (s_beat_color_cycle) {
        case LED_COLOR_CYCLE_RAINBOW:
            sample_rainbow_phase(0.10f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_SIREN:
            sample_siren_phase(1.35f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_ARCADE:
            sample_palette3(255, 0, 255,
                            0, 250, 255,
                            255, 220, 0,
                            0.11f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_INFERNO:
            sample_palette3(255, 20, 0,
                            255, 110, 0,
                            255, 210, 24,
                            0.085f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_BIOHAZARD:
            sample_palette3(0, 255, 64,
                            190, 255, 0,
                            18, 110, 0,
                            0.09f * t_sec + phase_offset, r, g, b);
            break;
        case LED_COLOR_CYCLE_STATIC:
        default:
            if (r) *r = s_beat_color_r;
            if (g) *g = s_beat_color_g;
            if (b) *b = s_beat_color_b;
            break;
    }
}

static void led_beat_sample_primary_color(float t_sec, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s_beat_color_style == LED_COLOR_STYLE_PALETTE &&
        s_beat_color_cycle != LED_COLOR_CYCLE_STATIC) {
        led_beat_sample_cycle_color(t_sec, 0.0f, r, g, b);
        return;
    }
    if (r) *r = s_beat_color_r;
    if (g) *g = s_beat_color_g;
    if (b) *b = s_beat_color_b;
}

static void led_beat_sample_secondary_color(float t_sec, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s_beat_color_style == LED_COLOR_STYLE_PALETTE &&
        s_beat_color_cycle != LED_COLOR_CYCLE_STATIC) {
        led_beat_sample_cycle_color(t_sec, 0.33f, r, g, b);
        return;
    }
    if (s_beat_color_style == LED_COLOR_STYLE_DUO) {
        if (r) *r = s_beat_secondary_r;
        if (g) *g = s_beat_secondary_g;
        if (b) *b = s_beat_secondary_b;
        return;
    }
    if (r) *r = s_beat_color_r;
    if (g) *g = s_beat_color_g;
    if (b) *b = s_beat_color_b;
}

static void led_beat_select_trigger_color(float t_sec, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s_beat_color_style == LED_COLOR_STYLE_DUO) {
        bool use_secondary = (s_beat_duo_flip++ & 1u) != 0u;
        if (use_secondary) {
            led_beat_sample_secondary_color(t_sec, r, g, b);
        } else {
            led_beat_sample_primary_color(t_sec, r, g, b);
        }
        return;
    }
    led_beat_sample_primary_color(t_sec, r, g, b);
}

static float led_beat_highlight_scale(void)
{
    return (s_beat_highlight_mode == LED_HIGHLIGHT_PEAKS) ? 1.0f : 0.0f;
}

static float led_audio_volume_range_scale(void)
{
    float floor_scale = 0.0f;
    if (s_audio_energy_range == LED_AUDIO_ENERGY_RANGE_MID) {
        floor_scale = 127.0f / 255.0f;
    } else if (s_audio_energy_range == LED_AUDIO_ENERGY_RANGE_HIGH) {
        floor_scale = 191.0f / 255.0f;
    }

    float raw = s_audio_raw_volume;
    if (raw <= kLedAudioBrightnessThreshold) {
        return floor_scale;
    }
    float norm = (raw - kLedAudioBrightnessThreshold) / (1.0f - kLedAudioBrightnessThreshold);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return floor_scale + (1.0f - floor_scale) * norm;
}

static float led_audio_brightness_live_scale(void)
{
    if (!s_audio_brightness_enabled) {
        return 1.0f;
    }
    return led_audio_volume_range_scale();
}

static void led_apply_audio_brightness_inplace(uint8_t *buf, size_t len_bytes)
{
    if (!buf || len_bytes == 0) return;
    float scale = led_audio_brightness_live_scale();
    if (scale >= 0.999f) return;
    if (scale < 0.0f) scale = 0.0f;
    for (size_t i = 0; i < len_bytes; ++i) {
        buf[i] = (uint8_t)((float)buf[i] * scale);
    }
}

void led_audio_levels_set(float overall, float low, float mid, float high)
{
    (void)overall;
    (void)low;
    (void)mid;
    (void)high;
}

void led_audio_raw_volume_set(float volume)
{
    if (!isfinite(volume)) volume = 0.0f;
    if (volume < 0.0f) volume = 0.0f;
    if (volume > 1.0f) volume = 1.0f;
    s_audio_raw_volume = volume;
}

void led_audio_brightness_enable(bool enabled)
{
    s_audio_brightness_enabled = enabled;
}

bool led_audio_brightness_enabled(void)
{
    return s_audio_brightness_enabled;
}

float led_audio_brightness_scale_get(void)
{
    return led_audio_brightness_live_scale();
}

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

static uint8_t *led_alloc_frame(size_t bytes, const char *label, bool prefer_internal)
{
    uint32_t primary_caps = prefer_internal
        ? (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
        : (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    uint8_t *buf = (uint8_t *)heap_caps_malloc(bytes, primary_caps);
    if (!buf) {
        ESP_LOGW(TAG, "%s %s alloc failed, falling back",
                 label ? label : "frame",
                 prefer_internal ? "internal RAM" : "SPIRAM");
        buf = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    }
    return buf;
}

static size_t led_active_count(void)
{
    size_t count = led_layout_count();
    if (count == 0 || count > LED_COUNT) count = LED_COUNT;
    return count;
}

static void led_split_frame(const uint8_t *rgb, size_t logical_pixels)
{
    led_layout_snapshot(&s_output_layout);
    if (s_output_layout.total_leds == 0 || s_output_layout.total_leds > LED_COUNT) {
        s_output_layout.total_leds = (uint16_t)led_active_count();
    }

    for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
        if (s_output_frames[strip]) {
            memset(s_output_frames[strip], 0, LED_COUNT * 3u);
        }
    }

    if (!rgb) return;
    if (logical_pixels > s_output_layout.total_leds) {
        logical_pixels = s_output_layout.total_leds;
    }

    for (size_t logical = 0; logical < logical_pixels; ++logical) {
        led_layout_mapping_t map;
        if (!led_layout_map_logical_from_config(&s_output_layout, logical, &map)) continue;
        if (map.strip >= LED_OUTPUT_COUNT) continue;
        if (map.physical_index >= LED_COUNT || !s_output_frames[map.strip]) continue;

        size_t src = led_pixel_offset(logical);
        size_t dst = led_pixel_offset(map.physical_index);
        s_output_frames[map.strip][dst + 0] = rgb[src + 0];
        s_output_frames[map.strip][dst + 1] = rgb[src + 1];
        s_output_frames[map.strip][dst + 2] = rgb[src + 2];
    }
}

static esp_err_t led_retry_outputs_locked(bool lock_held)
{
    for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
        if (!s_outputs[strip].chan || !s_outputs[strip].encoder || !s_output_frames[strip]) {
            return ESP_ERR_INVALID_STATE;
        }
    }

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
    esp_err_t err = ESP_OK;
    for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
        err = rmt_transmit(s_outputs[strip].chan,
                           s_outputs[strip].encoder,
                           s_output_frames[strip],
                           LED_COUNT * 3u,
                           &tx_cfg);
        if (err != ESP_OK) {
            break;
        }
    }
    if (err == ESP_OK) {
        for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
            err = rmt_tx_wait_all_done(s_outputs[strip].chan, pdMS_TO_TICKS(LED_RMT_TX_TIMEOUT_MS));
            if (err != ESP_OK) {
                break;
            }
        }
    }
    led_tx_lock_give();
    if (locked_here) {
        led_unlock();
    }
    return err;
}

static esp_err_t led_flush_locked(const uint8_t *rgb, size_t len_bytes, bool lock_held){
    for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
        if (!s_outputs[strip].chan || !s_outputs[strip].encoder || !s_output_frames[strip]) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    bool locked_here = false;
    if (!lock_held) {
        ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
        locked_here = true;
        lock_held = true;
    }

    size_t logical_pixels = len_bytes / 3u;
    if (logical_pixels > LED_COUNT) logical_pixels = LED_COUNT;
    led_split_frame(rgb, logical_pixels);

    esp_err_t err = led_retry_outputs_locked(true);
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "RMT TX timeout (%u logical px), retrying once", (unsigned)logical_pixels);
        for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
            (void)rmt_disable(s_outputs[strip].chan);
            (void)rmt_enable(s_outputs[strip].chan);
        }
        err = led_retry_outputs_locked(true);
    }
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
    } else if (len_bytes < max_bytes) {
        memset(s_frame + len_bytes, 0, max_bytes - len_bytes);
    }
    led_apply_audio_brightness_inplace(s_frame, len_bytes);
    s_tx_len_bytes = len_bytes;
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
        (void)led_flush_locked(s_tx_shadow, len_bytes, false);
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
    size_t active = led_active_count();
    if (lit_count > active) lit_count = active;
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    for (size_t i = 0; i < lit_count; ++i){
        led_set_pixel_rgb(s_frame, i, r, g, b);
    }
    esp_err_t err = led_flush_locked(s_frame, active * 3u, true);
    led_unlock();
    return err;
}

static esp_err_t led_test_show_chase(uint8_t r, uint8_t g, uint8_t b, size_t span, size_t pos){
    size_t active = led_active_count();
    if (span > active) span = active;
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    if (span > 0){
        size_t p = pos % span;
        led_set_pixel_rgb(s_frame, p, r, g, b);
    }
    esp_err_t err = led_flush_locked(s_frame, active * 3u, true);
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
    size_t active = led_active_count();
    if (test_pixels > active) test_pixels = active;
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

    ESP_RETURN_ON_ERROR(led_test_show_prefix(0, 0, 0, active), "led", "test off");
    return ESP_OK;
}

static esp_err_t led_init_output(size_t idx)
{
    if (idx >= LED_OUTPUT_COUNT) return ESP_ERR_INVALID_ARG;
    led_output_t *out = &s_outputs[idx];
    if (out->chan && out->encoder) return ESP_OK;

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = out->gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = LED_RMT_MEM_SYMBOLS,
        .resolution_hz = LED_RMT_RES_HZ,
        .trans_queue_depth = 1,
        .flags = { .with_dma = false }
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &out->chan), TAG, "new tx channel");

    led_strip_encoder_config_t enc_cfg = { .resolution = LED_RMT_RES_HZ };
    esp_err_t err = rmt_new_led_strip_encoder(&enc_cfg, &out->encoder);
    if (err != ESP_OK) {
        rmt_del_channel(out->chan);
        out->chan = NULL;
        return err;
    }

    err = rmt_enable(out->chan);
    if (err != ESP_OK) {
        rmt_del_encoder(out->encoder);
        out->encoder = NULL;
        rmt_del_channel(out->chan);
        out->chan = NULL;
        return err;
    }

    (void)gpio_set_drive_capability(out->gpio, GPIO_DRIVE_CAP_3);
    return ESP_OK;
}

esp_err_t led_init(void){
    if (s_inited) return ESP_OK;
    (void)led_layout_init();
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    if (s_inited) { // double-checked after taking lock
        led_unlock();
        return ESP_OK;
    }
    for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
        esp_err_t err = led_init_output(strip);
        if (err != ESP_OK) {
            led_unlock();
            return err;
        }
    }

    if (!s_frame) {
        s_frame = led_alloc_frame(LED_COUNT * 3, "s_frame", false);
    }
    if (!s_frame) {
        led_unlock();
        return ESP_ERR_NO_MEM;
    }
    if (!s_tx_shadow){
        s_tx_shadow = led_alloc_frame(LED_COUNT * 3, "s_tx_shadow", false);
        if (!s_tx_shadow){
            led_unlock();
            return ESP_ERR_NO_MEM;
        }
    }
    for (size_t strip = 0; strip < LED_OUTPUT_COUNT; ++strip) {
        if (!s_output_frames[strip]) {
            char label[20];
            snprintf(label, sizeof(label), "strip%u", (unsigned)strip);
            s_output_frames[strip] = led_alloc_frame(LED_COUNT * 3, label, true);
            if (!s_output_frames[strip]) {
                led_unlock();
                return ESP_ERR_NO_MEM;
            }
        }
    }
    // Clear strip
    memset(s_frame, 0, LED_COUNT * 3);
    esp_err_t err = led_flush_locked(s_frame, 3, true);
    if (err != ESP_OK) {
        // Don't hard-fail init on clear timeout: allow later pulses to run with
        // shorter frames for bring-up diagnostics.
        ESP_LOGW(TAG, "Initial strip clear failed: %s", esp_err_to_name(err));
        if (err != ESP_ERR_TIMEOUT) {
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
        s_pulse_frame = led_alloc_frame(LED_COUNT * 3, "s_pulse_frame", false);
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
    ESP_LOGI(TAG, "LED count max: %u active: %u", (unsigned)LED_COUNT, (unsigned)led_active_count());
    ESP_LOGI(TAG, "LED outputs: GPIO%d GPIO%d", (int)PIN_LED_STRIP_A, (int)PIN_LED_STRIP_B);
    ESP_LOGI(TAG, "LED workers: core=%d tx_prio=%d pulse_prio=%d",
             LED_TASK_CORE, LED_TX_TASK_PRIO, LED_PULSE_TASK_PRIO);
    led_unlock();
    return ESP_OK;
}

static esp_err_t led_send_all(uint8_t r, uint8_t g, uint8_t b){
    size_t active = led_active_count();
    ESP_RETURN_ON_ERROR(led_lock(), "led", "lock");
    memset(s_frame, 0, LED_COUNT * 3);
    for (size_t i = 0; i < active; ++i){
        led_set_pixel_rgb(s_frame, i, r, g, b);
    }
    esp_err_t err = led_submit_async_locked(s_frame, active * 3u);
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
    size_t active = led_active_count();
    if (count > active) count = active;
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
                const float target_bpm = 60.0f / raw_period_sec;
                const float current_bpm = (s_pulse_period_sec > 0.0f) ? (60.0f / s_pulse_period_sec) : target_bpm;
                if (fabsf(target_bpm - current_bpm) > kLedPulseSnapBpmDelta){
                    s_pulse_period_sec = raw_period_sec;
                } else {
                    s_pulse_period_sec = (1.0f - kLedPulseTempoSmooth) * s_pulse_period_sec
                                         + kLedPulseTempoSmooth * raw_period_sec;
                }
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

static void led_spark_seed_locked(uint8_t r, uint8_t g, uint8_t b)
{
    size_t active = led_active_count();
    if (active == 0) return;
    s_spark_r = r;
    s_spark_g = g;
    s_spark_b = b;
    for (int i = 0; i < kLedSparkSeedsPerBeat; ++i){
        size_t idx = (size_t)(esp_random() % active);
        uint16_t next = (uint16_t)s_spark_energy[idx] + kLedSparkSeedEnergy;
        s_spark_energy[idx] = (next > 255u) ? 255u : (uint8_t)next;
    }
}

static void led_comet_clear_locked(void)
{
    memset(s_comets, 0, sizeof(s_comets));
}

static void led_geo_clear_locked(void)
{
    memset(s_ring_pulses, 0, sizeof(s_ring_pulses));
    memset(s_plane_sweeps, 0, sizeof(s_plane_sweeps));
}

static void led_geo_clear_ring_locked(void)
{
    memset(s_ring_pulses, 0, sizeof(s_ring_pulses));
}

static void led_geo_clear_plane_locked(void)
{
    memset(s_plane_sweeps, 0, sizeof(s_plane_sweeps));
}

static bool led_geo_any_active_locked(void)
{
    for (size_t i = 0; i < LED_GEO_RING_SLOTS; ++i) {
        if (s_ring_pulses[i].active) return true;
    }
    for (size_t i = 0; i < LED_GEO_PLANE_SLOTS; ++i) {
        if (s_plane_sweeps[i].active) return true;
    }
    return false;
}

static void led_beat_clear_effect_state_locked(void)
{
    memset(s_pulses, 0, sizeof(s_pulses));
    memset(s_spark_energy, 0, sizeof(s_spark_energy));
    led_geo_clear_locked();
    led_comet_clear_locked();
    s_flash_ticks = 0;
    s_wave_amp = 0.0f;
    s_breathe_anchor_us = 0;
}

static led_beat_anim_t led_beat_pick_random_anim_locked(led_beat_anim_t prev_mode)
{
    const int first = LED_BEAT_ANIM_FLASH;
    const int last = LED_BEAT_ANIM_SOFT_FLASH;
    const int count = last - first + 1;
    int picked = first + (int)(esp_random() % (uint32_t)count);
    if (count > 1 && picked == (int)prev_mode) {
        picked = first + ((picked - first + 1 + (int)(esp_random() % (uint32_t)(count - 1))) % count);
    }
    return (led_beat_anim_t)picked;
}

static led_beat_anim_t led_beat_effective_mode_locked(int64_t now_us)
{
    if (s_beat_anim != LED_BEAT_ANIM_RANDOM) {
        return s_beat_anim;
    }

    if (s_beat_random_anim < LED_BEAT_ANIM_FLASH ||
        s_beat_random_anim > LED_BEAT_ANIM_SOFT_FLASH ||
        now_us >= s_beat_random_next_switch_us) {
        led_beat_anim_t next = led_beat_pick_random_anim_locked(s_beat_random_anim);
        if (next != s_beat_random_anim) {
            led_beat_clear_effect_state_locked();
            if (next == LED_BEAT_ANIM_AUDIO_ENERGY) {
                s_audio_energy_phase = 0.0f;
            }
            s_beat_random_anim = next;
        }
        s_beat_random_next_switch_us = now_us + LED_BEAT_RANDOM_INTERVAL_US;
    }

    return s_beat_random_anim;
}

static void led_geo_add_rgb_locked(size_t led, float add_r, float add_g, float add_b)
{
    uint8_t cur_r = 0, cur_g = 0, cur_b = 0;
    led_get_pixel_rgb(s_pulse_frame, led, &cur_r, &cur_g, &cur_b);
    float next_r = (float)cur_r + add_r;
    float next_g = (float)cur_g + add_g;
    float next_b = (float)cur_b + add_b;
    if (next_r > 255.0f) next_r = 255.0f;
    if (next_g > 255.0f) next_g = 255.0f;
    if (next_b > 255.0f) next_b = 255.0f;
    if (next_r < 0.0f) next_r = 0.0f;
    if (next_g < 0.0f) next_g = 0.0f;
    if (next_b < 0.0f) next_b = 0.0f;
    led_set_pixel_rgb(s_pulse_frame, led, (uint8_t)next_r, (uint8_t)next_g, (uint8_t)next_b);
}

static bool led_geo_fill_plane_background_locked(size_t count, float brightness_scale)
{
    if (!s_beat_plane_background_enabled || count == 0 || brightness_scale <= 0.0f) {
        return false;
    }

    float scale = brightness_scale * kLedGeoPlaneBackgroundScale;
    uint8_t pr = 0, pg = 0, pb = 0;
    float t_sec = (float)esp_timer_get_time() / 1000000.0f;
    led_beat_sample_primary_color(t_sec, &pr, &pg, &pb);
    uint8_t bg_r = (uint8_t)(scale * (float)pr);
    uint8_t bg_g = (uint8_t)(scale * (float)pg);
    uint8_t bg_b = (uint8_t)(scale * (float)pb);
    if (bg_r == 0 && bg_g == 0 && bg_b == 0) {
        return false;
    }

    for (size_t led = 0; led < count; ++led) {
        led_set_pixel_rgb(s_pulse_frame, led, bg_r, bg_g, bg_b);
    }
    return true;
}

static bool led_geo_fill_ring_background_locked(size_t count, float brightness_scale)
{
    if (!s_beat_ring_background_enabled || count == 0 || brightness_scale <= 0.0f) {
        return false;
    }

    float scale = brightness_scale * kLedGeoRingBackgroundScale;
    uint8_t pr = 0, pg = 0, pb = 0;
    float t_sec = (float)esp_timer_get_time() / 1000000.0f;
    led_beat_sample_primary_color(t_sec, &pr, &pg, &pb);
    uint8_t bg_r = (uint8_t)(scale * (float)pr);
    uint8_t bg_g = (uint8_t)(scale * (float)pg);
    uint8_t bg_b = (uint8_t)(scale * (float)pb);
    if (bg_r == 0 && bg_g == 0 && bg_b == 0) {
        return false;
    }

    for (size_t led = 0; led < count; ++led) {
        led_set_pixel_rgb(s_pulse_frame, led, bg_r, bg_g, bg_b);
    }
    return true;
}

static bool led_comet_fill_background_locked(size_t count, float brightness_scale)
{
    if (count == 0 || brightness_scale <= 0.0f) {
        return false;
    }

    float scale = brightness_scale * kLedCometBackgroundScale;
    uint8_t pr = 0, pg = 0, pb = 0;
    float t_sec = (float)esp_timer_get_time() / 1000000.0f;
    led_beat_sample_primary_color(t_sec, &pr, &pg, &pb);
    uint8_t bg_r = (uint8_t)(scale * (float)pr);
    uint8_t bg_g = (uint8_t)(scale * (float)pg);
    uint8_t bg_b = (uint8_t)(scale * (float)pb);
    if (bg_r == 0 && bg_g == 0 && bg_b == 0) {
        return false;
    }

    for (size_t led = 0; led < count; ++led) {
        led_set_pixel_rgb(s_pulse_frame, led, bg_r, bg_g, bg_b);
    }
    return true;
}

static void led_geo_layout_stats(const led_layout_config_t *cfg, size_t count,
                                 led_geo_layout_stats_t *stats)
{
    if (!cfg || !stats) return;
    memset(stats, 0, sizeof(*stats));
    stats->min_x = stats->min_y = stats->min_z = FLT_MAX;
    stats->max_x = stats->max_y = stats->max_z = -FLT_MAX;

    for (size_t i = 0; i < cfg->section_count; ++i) {
        const led_layout_section_t *sec = &cfg->sections[i];
        if (sec->geom_kind == LED_LAYOUT_GEOM_ARC && strstr(sec->name, "ring") != NULL) {
            stats->ring_center = sec->center;
            stats->ring_radius = fabsf(sec->radius);
            break;
        }
    }

    bool have_point = false;
    for (size_t i = 0; i < count; ++i) {
        led_point_t p;
        s_geo_ring_distance_cache[i] = FLT_MAX;
        if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
        have_point = true;
        if (p.x < stats->min_x) stats->min_x = p.x;
        if (p.x > stats->max_x) stats->max_x = p.x;
        if (p.y < stats->min_y) stats->min_y = p.y;
        if (p.y > stats->max_y) stats->max_y = p.y;
        if (p.z < stats->min_z) stats->min_z = p.z;
        if (p.z > stats->max_z) stats->max_z = p.z;
        float dx = p.x - stats->ring_center.x;
        float dy = p.y - stats->ring_center.y;
        float dz = p.z - stats->ring_center.z;
        float radial = sqrtf(dx * dx + dy * dy);
        if (radial > stats->radial_max) stats->radial_max = radial;
        float ring_delta = radial - stats->ring_radius;
        float ring_wave = sqrtf(ring_delta * ring_delta + dz * dz);
        s_geo_ring_distance_cache[i] = ring_wave;
        if (ring_wave > stats->ring_wave_max) stats->ring_wave_max = ring_wave;
    }

    if (!have_point) {
        stats->min_x = stats->min_y = stats->min_z = -1.0f;
        stats->max_x = stats->max_y = stats->max_z = 1.0f;
    }
    if (stats->radial_max < 1.0f) {
        float hx = 0.5f * (stats->max_x - stats->min_x);
        float hy = 0.5f * (stats->max_y - stats->min_y);
        stats->radial_max = sqrtf(hx * hx + hy * hy);
    }
    if (stats->ring_wave_max < 1.0f) {
        stats->ring_wave_max = stats->radial_max;
    }
}

static float led_geo_compact_wave(float distance, float front, float width)
{
    if (width <= 0.0001f) return 0.0f;
    float x = fabsf(distance - front) / width;
    if (x >= 1.0f) return 0.0f;
    float y = 1.0f - x * x;
    return y * y;
}

static float led_geo_projection_extent(const led_geo_layout_stats_t *stats,
                                       float nx, float ny, float nz)
{
    if (!stats) return 1.0f;
    float hx = 0.5f * (stats->max_x - stats->min_x);
    float hy = 0.5f * (stats->max_y - stats->min_y);
    float hz = 0.5f * (stats->max_z - stats->min_z);
    float extent = fabsf(nx) * hx + fabsf(ny) * hy + fabsf(nz) * hz;
    return (extent < 1.0f) ? 1.0f : extent;
}

static void led_geo_ring_spawn_scaled_locked(uint8_t r, uint8_t g, uint8_t b,
                                             float start_front, float width_scale)
{
    led_ring_pulse_t *slot = NULL;
    float furthest_front = -FLT_MAX;
    for (size_t i = 0; i < LED_GEO_RING_SLOTS; ++i) {
        led_ring_pulse_t *pulse = &s_ring_pulses[i];
        if (!pulse->active) {
            slot = pulse;
            break;
        }
        if (pulse->front > furthest_front) {
            furthest_front = pulse->front;
            slot = pulse;
        }
    }
    if (!slot) return;

    slot->r = r;
    slot->g = g;
    slot->b = b;
    slot->front = start_front;
    slot->speed = kLedGeoRingSpeedUnitsPerSec;
    slot->amp = 1.0f;
    slot->width_scale = width_scale;
    slot->active = true;
}

static void led_geo_ring_spawn_locked(uint8_t r, uint8_t g, uint8_t b, float start_front)
{
    led_geo_ring_spawn_scaled_locked(r, g, b, start_front, 1.0f);
}

static void led_geo_ring_trigger_locked(uint8_t r, uint8_t g, uint8_t b)
{
    if (led_active_count() == 0) return;
    led_geo_ring_spawn_locked(r, g, b, 0.0f);
}

static void led_geo_ring_train_trigger_locked(uint8_t r, uint8_t g, uint8_t b,
                                              size_t wagons, float spacing_units)
{
    if (led_active_count() == 0) return;
    if (wagons < 1u) wagons = 1u;
    if (spacing_units < 0.2f) spacing_units = 0.2f;
    for (size_t i = 0; i < wagons; ++i) {
        led_geo_ring_spawn_scaled_locked(r, g, b,
                                         -spacing_units * (float)i,
                                         kLedGeoRingTrainWidthScale);
    }
}

static void led_geo_plane_spawn_locked(uint8_t r, uint8_t g, uint8_t b,
                                       float nx, float ny, float nz)
{
    size_t active = led_active_count();
    if (active == 0) return;
    led_layout_snapshot(&s_effect_layout);
    if (s_effect_layout.total_leds > 0 && active > s_effect_layout.total_leds) {
        active = s_effect_layout.total_leds;
    }

    led_geo_layout_stats_t stats;
    led_geo_layout_stats(&s_effect_layout, active, &stats);

    float inv_len = 1.0f / sqrtf(nx * nx + ny * ny + nz * nz);
    nx *= inv_len;
    ny *= inv_len;
    nz *= inv_len;

    float extent = led_geo_projection_extent(&stats, nx, ny, nz);
    float width = 1.1f + 0.12f * stats.radial_max;
    float period_sec = s_pulse_period_sec;
    if (period_sec < kLedPulseMinPeriodSec) period_sec = kLedPulseMinPeriodSec;
    if (period_sec > kLedPulseMaxPeriodSec) period_sec = kLedPulseMaxPeriodSec;

    for (size_t i = 0; i < LED_GEO_PLANE_SLOTS; ++i) {
        led_plane_sweep_t *sweep = &s_plane_sweeps[i];
        if (!sweep->active || sweep->span_mode) continue;
        float sweep_extent = led_geo_projection_extent(&stats, sweep->nx, sweep->ny, sweep->nz);
        sweep->speed = kLedGeoPlaneSpeedFactor *
                       (2.0f * (sweep_extent + sweep->width)) / period_sec;
    }

    led_plane_sweep_t *slot = NULL;
    float weakest = FLT_MAX;
    for (size_t i = 0; i < LED_GEO_PLANE_SLOTS; ++i) {
        led_plane_sweep_t *sweep = &s_plane_sweeps[i];
        if (!sweep->active) {
            slot = sweep;
            break;
        }
        if (sweep->amp < weakest) {
            weakest = sweep->amp;
            slot = sweep;
        }
    }
    if (!slot) return;

    slot->r = r;
    slot->g = g;
    slot->b = b;
    slot->nx = nx;
    slot->ny = ny;
    slot->nz = nz;
    slot->width = width;
    slot->offset = -extent - width;
    slot->speed = kLedGeoPlaneSpeedFactor * (2.0f * (extent + width)) / period_sec;
    slot->amp = 1.0f;
    slot->span_mode = false;
    slot->active = true;
}

static void led_geo_plane_span_trigger_locked(uint8_t r, uint8_t g, uint8_t b)
{
    size_t active = led_active_count();
    if (active == 0) return;
    led_layout_snapshot(&s_effect_layout);
    if (s_effect_layout.total_leds > 0 && active > s_effect_layout.total_leds) {
        active = s_effect_layout.total_leds;
    }

    led_geo_layout_stats_t stats;
    led_geo_layout_stats(&s_effect_layout, active, &stats);

    float angle = ((float)(esp_random() & 1023u) / 1023.0f) * kLedWaveTwoPi;
    float tilt = ((float)((int)(esp_random() & 255u) - 128) / 128.0f) * 0.20f;
    float nx = cosf(angle);
    float ny = sinf(angle);
    float nz = tilt;
    float inv_len = 1.0f / sqrtf(nx * nx + ny * ny + nz * nz);
    nx *= inv_len;
    ny *= inv_len;
    nz *= inv_len;

    float width = 1.1f + 0.12f * stats.radial_max;

    led_plane_sweep_t *slot = NULL;
    float weakest = FLT_MAX;
    for (size_t i = 0; i < LED_GEO_PLANE_SLOTS; ++i) {
        led_plane_sweep_t *sweep = &s_plane_sweeps[i];
        if (!sweep->active) {
            slot = sweep;
            break;
        }
        if (sweep->amp < weakest) {
            weakest = sweep->amp;
            slot = sweep;
        }
    }
    if (!slot) return;

    slot->r = r;
    slot->g = g;
    slot->b = b;
    slot->nx = nx;
    slot->ny = ny;
    slot->nz = nz;
    slot->width = width;
    slot->offset = 0.0f;
    slot->speed = 0.0f;
    slot->amp = 1.0f;
    slot->span_mode = true;
    slot->active = true;
}

static void led_geo_plane_trigger_locked(uint8_t r, uint8_t g, uint8_t b)
{
    float angle = ((float)(esp_random() & 1023u) / 1023.0f) * kLedWaveTwoPi;
    float tilt = ((float)((int)(esp_random() & 255u) - 128) / 128.0f) * 0.32f;
    led_geo_plane_spawn_locked(r, g, b, cosf(angle), sinf(angle), tilt);
}

static void led_geo_plane_pair_trigger_locked(uint8_t r, uint8_t g, uint8_t b)
{
    float angle = ((float)(esp_random() & 1023u) / 1023.0f) * kLedWaveTwoPi;
    float tilt = ((float)((int)(esp_random() & 255u) - 128) / 128.0f) * 0.22f;
    float nx = cosf(angle);
    float ny = sinf(angle);

    led_geo_plane_spawn_locked(r, g, b, nx, ny, tilt);
    led_geo_plane_spawn_locked(r, g, b, -nx, -ny, -tilt);
}

static void led_geo_plane_fan_trigger_locked(uint8_t r, uint8_t g, uint8_t b)
{
    float base = ((float)(esp_random() & 1023u) / 1023.0f) * kLedWaveTwoPi;
    static const float kTilts[3] = { -0.16f, 0.0f, 0.16f };
    for (size_t i = 0; i < 3u; ++i) {
        float angle = base + ((float)i * (2.0f * kLedWavePi / 3.0f));
        led_geo_plane_spawn_locked(r, g, b, cosf(angle), sinf(angle), kTilts[i]);
    }
}

static bool led_geo_ring_render_locked(const led_layout_config_t *cfg, size_t count,
                                       const led_geo_layout_stats_t *stats,
                                       float brightness_scale)
{
    if (!cfg || !stats || count == 0) return false;
    bool rendered = false;
    float white_scale = led_beat_highlight_scale();

    float base_width = kLedGeoRingBaseWidth + kLedGeoRingWidthScale * stats->ring_wave_max;
    if (base_width < kLedGeoRingBaseWidth) base_width = kLedGeoRingBaseWidth;

    for (size_t pulse_idx = 0; pulse_idx < LED_GEO_RING_SLOTS; ++pulse_idx) {
        led_ring_pulse_t *pulse = &s_ring_pulses[pulse_idx];
        if (!pulse->active) continue;
        float width = base_width * pulse->width_scale;
        if (width < 0.45f) width = 0.45f;

        for (size_t i = 0; i < count; ++i) {
            float ring_d = s_geo_ring_distance_cache[i];
            if (ring_d >= FLT_MAX * 0.5f) continue;
            float crest = led_geo_compact_wave(ring_d, pulse->front, width);
            if (crest <= 0.0f) continue;
            float primary = pulse->amp * brightness_scale * crest;
            float white = pulse->amp * brightness_scale * white_scale * (0.06f * crest * crest);
            led_geo_add_rgb_locked(
                i,
                primary * (float)pulse->r + white * 255.0f,
                primary * (float)pulse->g + white * 255.0f,
                primary * (float)pulse->b + white * 255.0f
            );
        }

        rendered = true;
        pulse->front += pulse->speed * kLedPulseDt;
        pulse->amp *= kLedGeoRingDecay;
        if (pulse->front > stats->ring_wave_max + width || pulse->amp < 0.04f) {
            pulse->active = false;
        }
    }

    return rendered;
}

static bool led_geo_plane_render_locked(const led_layout_config_t *cfg, size_t count,
                                        const led_geo_layout_stats_t *stats,
                                        float brightness_scale)
{
    if (!cfg || !stats || count == 0) return false;
    bool rendered = false;
    float white_scale = led_beat_highlight_scale();
    bool background_enabled = s_beat_plane_background_enabled;

    float cx = 0.5f * (stats->min_x + stats->max_x);
    float cy = 0.5f * (stats->min_y + stats->max_y);
    float cz = 0.5f * (stats->min_z + stats->max_z);

    for (size_t sweep_idx = 0; sweep_idx < LED_GEO_PLANE_SLOTS; ++sweep_idx) {
        led_plane_sweep_t *sweep = &s_plane_sweeps[sweep_idx];
        if (!sweep->active) continue;
        float extent = led_geo_projection_extent(stats, sweep->nx, sweep->ny, sweep->nz);

        for (size_t i = 0; i < count; ++i) {
            led_point_t p;
            if (!led_layout_get_from_config(cfg, (int)i, &p)) continue;
            float proj = (p.x - cx) * sweep->nx +
                         (p.y - cy) * sweep->ny +
                         (p.z - cz) * sweep->nz;
            float crest = sweep->span_mode
                ? expf(-fabsf(proj - sweep->offset) / sweep->width)
                : led_geo_compact_wave(proj, sweep->offset, sweep->width);
            if (crest <= 0.0f) continue;
            float trail = 0.0f;
            if (!sweep->span_mode) {
                float behind = sweep->offset - proj;
                if (behind > 0.0f) {
                    trail = expf(-behind / (2.2f * sweep->width));
                }
            }
            float primary = sweep->amp * brightness_scale *
                            (sweep->span_mode
                                ? (background_enabled ? (1.08f * crest)
                                                      : (0.88f * crest))
                                : (background_enabled
                                ? (1.18f * crest + 0.30f * trail)
                                : (0.92f * crest + 0.18f * trail)));
            float white = sweep->amp * brightness_scale * white_scale *
                          ((sweep->span_mode
                              ? (background_enabled ? 0.12f : 0.07f)
                              : (background_enabled ? 0.14f : 0.08f))) * crest * crest;
            led_geo_add_rgb_locked(
                i,
                primary * (float)sweep->r + white * 255.0f,
                primary * (float)sweep->g + white * 255.0f,
                primary * (float)sweep->b + white * 255.0f
            );
        }

        rendered = true;
        if (!sweep->span_mode) {
            sweep->offset += sweep->speed * kLedPulseDt;
        }
        sweep->amp *= kLedGeoPlaneDecay;
        if ((sweep->span_mode && sweep->amp < 0.04f) ||
            (!sweep->span_mode &&
             (sweep->offset > extent + sweep->width || sweep->amp < 0.04f))) {
            sweep->active = false;
        }
    }

    return rendered;
}

static bool led_audio_energy_render_locked(const led_layout_config_t *cfg, size_t count,
                                           float brightness_scale)
{
    (void)cfg;
    if (count == 0) return false;

    float period_sec = s_pulse_period_sec;
    if (period_sec < kLedPulseMinPeriodSec) period_sec = kLedPulseMinPeriodSec;
    if (period_sec > kLedPulseMaxPeriodSec) period_sec = kLedPulseMaxPeriodSec;

    float bpm_scale = (60.0f / period_sec) / 120.0f;
    float speed = kLedAudioEnergyBaseSpeed * bpm_scale;
    if (speed < 2.5f) speed = 2.5f;
    s_audio_energy_phase += speed * kLedPulseDt;
    if (s_audio_energy_phase > 4096.0f) {
        s_audio_energy_phase -= 4096.0f;
    }

    uint8_t pr = 0, pg = 0, pb = 0;
    float t_sec = (float)esp_timer_get_time() / 1000000.0f;
    led_beat_sample_primary_color(t_sec, &pr, &pg, &pb);
    if (brightness_scale < 0.004f) return false;

    for (size_t led = 0; led < count; ++led) {
        float x = (float)led * 0.16f - s_audio_energy_phase;
        float carrier = 0.5f + 0.5f * sinf(x);
        float ripple = 0.5f + 0.5f * sinf(0.37f * x + 1.7f);
        float level = carrier * carrier * (0.65f + 0.35f * ripple);
        float add_r = brightness_scale * ((float)pr * level);
        float add_g = brightness_scale * ((float)pg * level);
        float add_b = brightness_scale * ((float)pb * level);
        if (add_r < 1.0f && add_g < 1.0f && add_b < 1.0f) continue;
        led_geo_add_rgb_locked(led, add_r, add_g, add_b);
    }

    return true;
}

static led_comet_t *led_comet_pick_slot_locked(void)
{
    led_comet_t *slot = NULL;
    float weakest = FLT_MAX;
    for (size_t i = 0; i < LED_COMET_SLOTS; ++i) {
        led_comet_t *comet = &s_comets[i];
        if (!comet->active) {
            return comet;
        }
        if (comet->amp < weakest) {
            weakest = comet->amp;
            slot = comet;
        }
    }
    return slot;
}

static void led_comet_spawn_locked(uint8_t r, uint8_t g, uint8_t b,
                                   int8_t dir, float speed_scale,
                                   float tail_scale, float phase_offset,
                                   float amp)
{
    size_t active = led_active_count();
    if (active == 0) return;
    float period_sec = s_pulse_period_sec;
    if (period_sec < kLedPulseMinPeriodSec) period_sec = kLedPulseMinPeriodSec;
    if (period_sec > kLedPulseMaxPeriodSec) period_sec = kLedPulseMaxPeriodSec;

    float tail_leds = 0.08f * (float)active * tail_scale;
    if (tail_leds < kLedCometTailMin) tail_leds = kLedCometTailMin;
    if (tail_leds > kLedCometTailMax) tail_leds = kLedCometTailMax;
    if (phase_offset < 0.0f) phase_offset = 0.0f;
    if (amp < 0.12f) amp = 0.12f;

    led_comet_t *slot = led_comet_pick_slot_locked();
    if (!slot) return;

    float travel_span = (float)active + tail_leds + kLedCometHeadLead;
    slot->r = r;
    slot->g = g;
    slot->b = b;
    slot->amp = amp;
    slot->tail_leds = tail_leds;
    slot->speed = speed_scale * (travel_span / period_sec);
    slot->dir = (dir >= 0) ? 1 : -1;
    slot->head = (slot->dir > 0)
        ? (-kLedCometHeadLead - phase_offset * travel_span)
        : ((float)active + kLedCometHeadLead + phase_offset * travel_span);
    slot->active = true;
}

static void led_comet_trigger_locked(uint8_t r, uint8_t g, uint8_t b, int profile)
{
    int8_t dir = s_comet_spawn_dir;

    if (profile <= 0) {
        led_comet_spawn_locked(r, g, b, dir, 0.82f, 1.70f, 0.00f, 1.00f);
    } else if (profile == 1) {
        static const float kPairSpeeds[] = { 0.84f, 1.10f };
        static const float kPairTails[] = { 1.65f, 1.35f };
        static const float kPairOffsets[] = { 0.00f, 0.30f };
        static const float kPairAmps[] = { 1.00f, 0.86f };
        for (size_t i = 0; i < sizeof(kPairSpeeds) / sizeof(kPairSpeeds[0]); ++i) {
            led_comet_spawn_locked(r, g, b, dir,
                                   kPairSpeeds[i],
                                   kPairTails[i],
                                   kPairOffsets[i],
                                   kPairAmps[i]);
        }
    } else {
        static const float kSwarmSpeeds[] = { 0.56f, 0.74f, 0.90f, 1.08f, 1.30f };
        static const float kSwarmTails[] = { 1.70f, 1.45f, 1.22f, 1.00f, 0.82f };
        static const float kSwarmOffsets[] = { 0.00f, 0.12f, 0.28f, 0.50f, 0.72f };
        static const float kSwarmAmps[] = { 1.00f, 0.92f, 0.84f, 0.74f, 0.66f };
        for (size_t i = 0; i < sizeof(kSwarmSpeeds) / sizeof(kSwarmSpeeds[0]); ++i) {
            led_comet_spawn_locked(r, g, b, dir,
                                   kSwarmSpeeds[i],
                                   kSwarmTails[i],
                                   kSwarmOffsets[i],
                                   kSwarmAmps[i]);
        }
    }

    s_spark_r = r;
    s_spark_g = g;
    s_spark_b = b;
    s_comet_spawn_dir = (int8_t)-s_comet_spawn_dir;
}

static bool led_comet_step_locked(float brightness_scale)
{
    size_t active = led_active_count();
    if (active == 0) {
        led_comet_clear_locked();
        return false;
    }

    bool rendered = false;
    for (size_t comet_idx = 0; comet_idx < LED_COMET_SLOTS; ++comet_idx) {
        led_comet_t *comet = &s_comets[comet_idx];
        if (!comet->active) continue;

        comet->head += (float)comet->dir * comet->speed * kLedPulseDt;
        comet->amp *= kLedCometDecay;

        const float max_pos = (float)active + comet->tail_leds;
        const float min_pos = -comet->tail_leds;
        if (comet->head > max_pos || comet->head < min_pos || comet->amp < 0.08f) {
            comet->active = false;
            continue;
        }

        rendered = true;

        if ((esp_random() & 0x7u) == 0u) {
            uint32_t trail_span = (uint32_t)(0.6f * comet->tail_leds);
            if (trail_span < 2u) trail_span = 2u;
            int trail_leds = (int)(esp_random() % trail_span);
            int spark_led = (int)lroundf(comet->head - (float)comet->dir * (float)trail_leds);
            if (spark_led >= 0 && spark_led < (int)active) {
                uint16_t next = (uint16_t)s_spark_energy[spark_led] + 72u;
                s_spark_energy[spark_led] = (next > 255u) ? 255u : (uint8_t)next;
            }
        }

        for (size_t led = 0; led < active; ++led) {
            float delta = (comet->dir > 0)
                ? (comet->head - (float)led)
                : ((float)led - comet->head);
            if (delta < 0.0f || delta > comet->tail_leds) continue;

            float tail_w = 1.0f - (delta / comet->tail_leds);
            tail_w = tail_w * (0.55f + 0.45f * tail_w);
            float head_w = expf(-1.6f * fabsf((float)led - comet->head));
            float w = comet->amp * brightness_scale * (0.98f * tail_w + 0.52f * head_w);
            if (w <= 0.0f) continue;

            led_geo_add_rgb_locked(led,
                                   w * (float)comet->r,
                                   w * (float)comet->g,
                                   w * (float)comet->b);
        }
    }
    return rendered;
}

static void led_pulse_spawn(uint8_t r, uint8_t g, uint8_t b){
    if (!led_pulse_lock()) return;
    led_pulse_update_period_locked();
    led_pulse_spawn_locked(r, g, b, -kLedPulseWidth);
    led_pulse_unlock();
}

static bool led_pulse_step_locked(void){
    bool any_active = false;
    size_t active = led_active_count();
    float brightness_scale = (float)s_beat_brightness / 255.0f;
    int64_t now_us = esp_timer_get_time();
    led_beat_anim_t effective_mode = led_beat_effective_mode_locked(now_us);
    memset(s_pulse_frame, 0, LED_COUNT * 3);
    led_layout_snapshot(&s_effect_layout);
    if (s_effect_layout.total_leds > 0 && active > s_effect_layout.total_leds) {
        active = s_effect_layout.total_leds;
    }
    if (s_beat_enabled &&
        led_beat_mode_uses_plane(effective_mode) &&
        led_geo_fill_plane_background_locked(active, brightness_scale)) {
        any_active = true;
    }
    if (s_beat_enabled &&
        led_beat_mode_uses_ring(effective_mode) &&
        led_geo_fill_ring_background_locked(active, brightness_scale)) {
        any_active = true;
    }
    if (s_beat_enabled &&
        led_beat_mode_uses_chase_background(effective_mode) &&
        led_comet_fill_background_locked(active, brightness_scale)) {
        any_active = true;
    }
    if (s_beat_enabled && effective_mode == LED_BEAT_ANIM_BREATHE && active > 0) {
        float period_sec = s_pulse_period_sec;
        if (period_sec < kLedPulseMinPeriodSec) period_sec = kLedPulseMinPeriodSec;
        if (period_sec > kLedPulseMaxPeriodSec) period_sec = kLedPulseMaxPeriodSec;
        if (s_breathe_anchor_us <= 0) {
            s_breathe_anchor_us = now_us;
        }
        float elapsed_sec = (float)(now_us - s_breathe_anchor_us) / 1000000.0f;
        float phase = elapsed_sec / period_sec;
        phase -= floorf(phase);
        float wave = 0.5f * (cosf(kLedWaveTwoPi * phase) + 1.0f);
        wave = 0.15f + 0.85f * (wave * wave);
        wave *= brightness_scale;
        if (wave > 0.0f) {
            float t_sec = (float)now_us / 1000000.0f;
            uint8_t br = 0, bg = 0, bb = 0;
            led_beat_sample_primary_color(t_sec, &br, &bg, &bb);
            uint8_t fr = (uint8_t)(wave * (float)br);
            uint8_t fg = (uint8_t)(wave * (float)bg);
            uint8_t fb = (uint8_t)(wave * (float)bb);
            for (size_t led = 0; led < active; ++led) {
                led_set_pixel_rgb(s_pulse_frame, led, fr, fg, fb);
            }
            any_active = true;
        }
    }
    for (size_t i = 0; i < sizeof(s_pulses)/sizeof(s_pulses[0]); ++i){
        led_pulse_t *p = &s_pulses[i];
        if (!p->active) continue;
        if (active == 0) {
            p->active = false;
            continue;
        }
        float period_sec = s_pulse_period_sec;
        if (period_sec < kLedPulseMinPeriodSec) period_sec = kLedPulseMinPeriodSec;
        if (period_sec > kLedPulseMaxPeriodSec) period_sec = kLedPulseMaxPeriodSec;
        const float speed = 0.25f * ((float)active / period_sec);
        p->pos += kLedPulseDt * speed;
        p->amp *= kLedPulseDecay;
        if (p->pos > (float)active + kLedPulseWidth || p->amp < kLedPulseFloor){
            p->active = false;
            continue;
        }
        any_active = true;
        for (size_t led = 0; led < active; ++led){
            float d = fabsf((float)led - p->pos);
            float w = expf(-d / kLedPulseWidth);
            float v = p->amp * w;
            if (v <= 0.0f) continue;
            float add = v * (float)kLedPulseIntensity * brightness_scale;
            uint8_t cur_r = 0, cur_g = 0, cur_b = 0;
            led_get_pixel_rgb(s_pulse_frame, led, &cur_r, &cur_g, &cur_b);

            float next_r = (float)cur_r + add * ((float)p->r / 255.0f);
            float next_g = (float)cur_g + add * ((float)p->g / 255.0f);
            float next_b = (float)cur_b + add * ((float)p->b / 255.0f);
            if (next_g > 255.0f) next_g = 255.0f;
            if (next_r > 255.0f) next_r = 255.0f;
            if (next_b > 255.0f) next_b = 255.0f;
            led_set_pixel_rgb(
                s_pulse_frame,
                led,
                (uint8_t)next_r,
                (uint8_t)next_g,
                (uint8_t)next_b
            );
        }
    }
    if (led_comet_step_locked(brightness_scale)){
        any_active = true;
    }
    if (s_flash_ticks > 0){
        uint8_t fr = (uint8_t)((float)s_flash_r * brightness_scale);
        uint8_t fg = (uint8_t)((float)s_flash_g * brightness_scale);
        uint8_t fb = (uint8_t)((float)s_flash_b * brightness_scale);
        for (size_t led = 0; led < active; ++led){
            led_set_pixel_rgb(s_pulse_frame, led, fr, fg, fb);
        }
        s_flash_ticks--;
        any_active = true;
    }
    if (s_wave_amp > kLedWaveFloor){
        float level = s_wave_amp * (0.75f + 0.25f * sinf(s_wave_phase)); //better (keep this)
        level *= brightness_scale;

        if (level > 0.0f){
            float wr = level * (float)s_wave_r;
            float wg = level * (float)s_wave_g;
            float wb = level * (float)s_wave_b;
            if (wr > 255.0f) wr = 255.0f;
            if (wg > 255.0f) wg = 255.0f;
            if (wb > 255.0f) wb = 255.0f;

            for (size_t led = 0; led < active; ++led){
                led_set_pixel_rgb(
                    s_pulse_frame,
                    led,
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
    if (s_beat_enabled && effective_mode == LED_BEAT_ANIM_AUDIO_ENERGY) {
        if (led_audio_energy_render_locked(&s_effect_layout, active, brightness_scale)) {
            any_active = true;
        }
    }
    if (led_geo_any_active_locked() && active > 0){
        size_t geo_count = active;
        if (s_effect_layout.total_leds > 0 && geo_count > s_effect_layout.total_leds) {
            geo_count = s_effect_layout.total_leds;
        }
        if (geo_count > 0) {
            led_geo_layout_stats_t geo_stats;
            led_geo_layout_stats(&s_effect_layout, geo_count, &geo_stats);
            if (led_geo_ring_render_locked(&s_effect_layout, geo_count, &geo_stats, brightness_scale)) {
                any_active = true;
            }
            if (led_geo_plane_render_locked(&s_effect_layout, geo_count, &geo_stats, brightness_scale)) {
                any_active = true;
            }
        }
    }
    bool spark_active = false;
    for (size_t led = 0; led < active; ++led){
        uint8_t e = s_spark_energy[led];
        if (e < kLedSparkFloor){
            s_spark_energy[led] = 0;
            continue;
        }
        spark_active = true;
        float w = ((float)e / 255.0f) * brightness_scale;
        uint8_t cur_r = 0, cur_g = 0, cur_b = 0;
        led_get_pixel_rgb(s_pulse_frame, led, &cur_r, &cur_g, &cur_b);
        float next_r = (float)cur_r + w * (float)s_spark_r;
        float next_g = (float)cur_g + w * (float)s_spark_g;
        float next_b = (float)cur_b + w * (float)s_spark_b;
        if (next_r > 255.0f) next_r = 255.0f;
        if (next_g > 255.0f) next_g = 255.0f;
        if (next_b > 255.0f) next_b = 255.0f;
        led_set_pixel_rgb(s_pulse_frame, led,
                          (uint8_t)next_r, (uint8_t)next_g, (uint8_t)next_b);
        s_spark_energy[led] = (uint8_t)(((uint16_t)e * (uint16_t)kLedSparkDecay) / 255u);
    }
    if (spark_active){
        any_active = true;
    }
    return any_active;
}

static void led_pulse_task(void *arg){
    (void)arg;
    const TickType_t delay_ticks = pdMS_TO_TICKS(16);
    while (1){
        bool active = false;
        size_t count = led_active_count();
        if (led_pulse_lock()){
            active = led_pulse_step_locked();
            led_pulse_unlock();
        }
        if (active){
            // Push the pulse frame directly (no intermediate memcpy path).
            if (led_submit_async(s_pulse_frame, count * 3u) == ESP_OK){
                s_pulse_was_on = true;
            }
        } else if (s_pulse_was_on){
            memset(s_pulse_frame, 0, LED_COUNT * 3);
            if (led_submit_async(s_pulse_frame, count * 3u) == ESP_OK){
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
    return LED_BEAT_ANIM_RANDOM + 1;
}

const char *led_beat_anim_name(int idx){
    switch (idx){
        case LED_BEAT_ANIM_FLASH: return "flash";
        case LED_BEAT_ANIM_PULSE: return "pulse";
        case LED_BEAT_ANIM_RING_TRAIN: return "ring_train";
        case LED_BEAT_ANIM_PLANE_PAIR: return "plane_pair";
        case LED_BEAT_ANIM_SPARK: return "spark";
        case LED_BEAT_ANIM_COMET: return "comet";
        case LED_BEAT_ANIM_SHOCK: return "shock";
        case LED_BEAT_ANIM_RING_PULSE: return "ring_pulse";
        case LED_BEAT_ANIM_PLANE_SWEEP: return "plane_sweep";
        case LED_BEAT_ANIM_CROSSFIRE: return "crossfire";
        case LED_BEAT_ANIM_PLANE_FAN: return "plane_fan";
        case LED_BEAT_ANIM_AUDIO_ENERGY: return "energy";
        case LED_BEAT_ANIM_BREATHE: return "breathe";
        case LED_BEAT_ANIM_PLANE_SPAN: return "plane_span";
        case LED_BEAT_ANIM_COMET_PAIR: return "comet_pair";
        case LED_BEAT_ANIM_COMET_SWARM: return "comet_swarm";
        case LED_BEAT_ANIM_COMET_BG: return "comet_bg";
        case LED_BEAT_ANIM_COMET_PAIR_BG: return "comet_pair_bg";
        case LED_BEAT_ANIM_COMET_SWARM_BG: return "comet_swarm_bg";
        case LED_BEAT_ANIM_SOFT_FLASH: return "soft_flash";
        case LED_BEAT_ANIM_RANDOM: return "random";
        default: return NULL;
    }
}

void led_beat_anim_set(led_beat_anim_t mode){
    if (mode != LED_BEAT_ANIM_FLASH &&
        mode != LED_BEAT_ANIM_PULSE &&
        mode != LED_BEAT_ANIM_RING_TRAIN &&
        mode != LED_BEAT_ANIM_PLANE_PAIR &&
        mode != LED_BEAT_ANIM_SPARK &&
        mode != LED_BEAT_ANIM_COMET &&
        mode != LED_BEAT_ANIM_SHOCK &&
        mode != LED_BEAT_ANIM_RING_PULSE &&
        mode != LED_BEAT_ANIM_PLANE_SWEEP &&
        mode != LED_BEAT_ANIM_CROSSFIRE &&
        mode != LED_BEAT_ANIM_PLANE_FAN &&
        mode != LED_BEAT_ANIM_AUDIO_ENERGY &&
        mode != LED_BEAT_ANIM_BREATHE &&
        mode != LED_BEAT_ANIM_PLANE_SPAN &&
        mode != LED_BEAT_ANIM_COMET_PAIR &&
        mode != LED_BEAT_ANIM_COMET_SWARM &&
        mode != LED_BEAT_ANIM_COMET_BG &&
        mode != LED_BEAT_ANIM_COMET_PAIR_BG &&
        mode != LED_BEAT_ANIM_COMET_SWARM_BG &&
        mode != LED_BEAT_ANIM_SOFT_FLASH &&
        mode != LED_BEAT_ANIM_RANDOM){
        mode = LED_BEAT_ANIM_FLASH;
    }
    s_beat_anim = mode;
    s_beat_random_anim = LED_BEAT_ANIM_FLASH;
    s_beat_random_next_switch_us = 0;
    if ((mode == LED_BEAT_ANIM_AUDIO_ENERGY ||
         mode == LED_BEAT_ANIM_BREATHE ||
         mode == LED_BEAT_ANIM_PLANE_SPAN ||
         mode == LED_BEAT_ANIM_SOFT_FLASH ||
         mode == LED_BEAT_ANIM_RANDOM) && led_pulse_lock()) {
        led_beat_clear_effect_state_locked();
        s_audio_energy_phase = 0.0f;
        if (mode == LED_BEAT_ANIM_AUDIO_ENERGY) {
            s_audio_raw_volume = 0.0f;
        }
        led_pulse_unlock();
    }
}

led_beat_anim_t led_beat_anim_get(void){
    return s_beat_anim;
}

void led_beat_enable(bool enabled){
    s_beat_enabled = enabled;
    if (enabled) {
        led_pulse_start_task();
    }
}

bool led_beat_enabled(void){
    return s_beat_enabled;
}

static bool led_beat_mode_uses_ring(led_beat_anim_t mode)
{
    return mode == LED_BEAT_ANIM_RING_PULSE ||
           mode == LED_BEAT_ANIM_RING_TRAIN ||
           mode == LED_BEAT_ANIM_CROSSFIRE;
}

static bool led_beat_mode_uses_plane(led_beat_anim_t mode)
{
    return mode == LED_BEAT_ANIM_PLANE_SWEEP ||
           mode == LED_BEAT_ANIM_PLANE_PAIR ||
           mode == LED_BEAT_ANIM_CROSSFIRE ||
           mode == LED_BEAT_ANIM_PLANE_FAN ||
           mode == LED_BEAT_ANIM_PLANE_SPAN;
}

static bool led_beat_mode_uses_chase_background(led_beat_anim_t mode)
{
    return mode == LED_BEAT_ANIM_COMET_BG ||
           mode == LED_BEAT_ANIM_COMET_PAIR_BG ||
           mode == LED_BEAT_ANIM_COMET_SWARM_BG;
}

static int led_beat_chase_profile(led_beat_anim_t mode)
{
    switch (mode) {
        case LED_BEAT_ANIM_COMET:
        case LED_BEAT_ANIM_COMET_BG:
            return 0;
        case LED_BEAT_ANIM_COMET_PAIR:
        case LED_BEAT_ANIM_COMET_PAIR_BG:
            return 1;
        case LED_BEAT_ANIM_COMET_SWARM:
        case LED_BEAT_ANIM_COMET_SWARM_BG:
            return 2;
        default:
            return -1;
    }
}

void led_beat_color_set(uint8_t r, uint8_t g, uint8_t b){
    if (led_pulse_lock()){
        s_beat_color_r = r;
        s_beat_color_g = g;
        s_beat_color_b = b;
        led_pulse_unlock();
    } else {
        s_beat_color_r = r;
        s_beat_color_g = g;
        s_beat_color_b = b;
    }
}

void led_beat_color_get(uint8_t *r, uint8_t *g, uint8_t *b){
    if (led_pulse_lock()){
        if (r) *r = s_beat_color_r;
        if (g) *g = s_beat_color_g;
        if (b) *b = s_beat_color_b;
        led_pulse_unlock();
        return;
    }
    if (r) *r = s_beat_color_r;
    if (g) *g = s_beat_color_g;
    if (b) *b = s_beat_color_b;
}

void led_beat_secondary_color_set(uint8_t r, uint8_t g, uint8_t b){
    if (led_pulse_lock()){
        s_beat_secondary_r = r;
        s_beat_secondary_g = g;
        s_beat_secondary_b = b;
        led_pulse_unlock();
    } else {
        s_beat_secondary_r = r;
        s_beat_secondary_g = g;
        s_beat_secondary_b = b;
    }
}

void led_beat_secondary_color_get(uint8_t *r, uint8_t *g, uint8_t *b){
    if (led_pulse_lock()){
        if (r) *r = s_beat_secondary_r;
        if (g) *g = s_beat_secondary_g;
        if (b) *b = s_beat_secondary_b;
        led_pulse_unlock();
        return;
    }
    if (r) *r = s_beat_secondary_r;
    if (g) *g = s_beat_secondary_g;
    if (b) *b = s_beat_secondary_b;
}

void led_beat_plane_background_enable(bool enabled){
    s_beat_plane_background_enabled = enabled;
}

bool led_beat_plane_background_enabled(void){
    return s_beat_plane_background_enabled;
}

void led_beat_ring_background_enable(bool enabled){
    s_beat_ring_background_enabled = enabled;
}

bool led_beat_ring_background_enabled(void){
    return s_beat_ring_background_enabled;
}

void led_beat_color_cycle_set(led_color_cycle_t mode){
    if (mode < LED_COLOR_CYCLE_STATIC || mode > LED_COLOR_CYCLE_BIOHAZARD) {
        mode = LED_COLOR_CYCLE_STATIC;
    }
    s_beat_color_cycle = mode;
}

led_color_cycle_t led_beat_color_cycle_get(void){
    return s_beat_color_cycle;
}

void led_beat_color_style_set(led_color_style_t style){
    if (style < LED_COLOR_STYLE_MONO || style > LED_COLOR_STYLE_PALETTE) {
        style = LED_COLOR_STYLE_MONO;
    }
    s_beat_color_style = style;
    s_beat_duo_flip = 0;
}

led_color_style_t led_beat_color_style_get(void){
    return s_beat_color_style;
}

void led_beat_highlight_mode_set(led_highlight_mode_t mode){
    if (mode < LED_HIGHLIGHT_OFF || mode > LED_HIGHLIGHT_PEAKS) {
        mode = LED_HIGHLIGHT_OFF;
    }
    s_beat_highlight_mode = mode;
}

led_highlight_mode_t led_beat_highlight_mode_get(void){
    return s_beat_highlight_mode;
}

void led_beat_brightness_set(uint8_t level){
    if (led_pulse_lock()){
        s_beat_brightness = level;
        led_pulse_unlock();
    } else {
        s_beat_brightness = level;
    }
}

uint8_t led_beat_brightness_get(void){
    if (led_pulse_lock()){
        uint8_t level = s_beat_brightness;
        led_pulse_unlock();
        return level;
    }
    return s_beat_brightness;
}

void led_audio_energy_range_set(led_audio_energy_range_t range){
    if (range < LED_AUDIO_ENERGY_RANGE_FULL || range > LED_AUDIO_ENERGY_RANGE_HIGH) {
        range = LED_AUDIO_ENERGY_RANGE_FULL;
    }
    s_audio_energy_range = range;
}

led_audio_energy_range_t led_audio_energy_range_get(void){
    return s_audio_energy_range;
}

void led_trigger_beat(uint8_t r, uint8_t g, uint8_t b){
    if (!s_beat_enabled) return;
    if (!s_inited){
        if (led_init() != ESP_OK){
            return;
        }
    }
    int64_t now_us = esp_timer_get_time();
    float t_sec = (float)now_us / 1000000.0f;
    led_beat_select_trigger_color(t_sec, &r, &g, &b);
    led_pulse_start_task();

    if (!led_pulse_lock()) return;
    led_beat_anim_t effective_mode = led_beat_effective_mode_locked(now_us);
    if (effective_mode == LED_BEAT_ANIM_PULSE){
        led_comet_clear_locked();
        led_geo_clear_locked();
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_pulse_update_period_locked();
        led_pulse_spawn_locked(r, g, b, -kLedPulseWidth);
        led_pulse_unlock();
        return;
    }

    led_pulse_update_period_locked();
    for (size_t i = 0; i < sizeof(s_pulses) / sizeof(s_pulses[0]); ++i){
        s_pulses[i].active = false;
    }
    led_comet_clear_locked();
    if (led_beat_mode_uses_ring(effective_mode) && !led_beat_mode_uses_plane(effective_mode)) {
        led_geo_clear_plane_locked();
    } else if (led_beat_mode_uses_plane(effective_mode) && !led_beat_mode_uses_ring(effective_mode)) {
        led_geo_clear_ring_locked();
    } else if (!led_beat_mode_uses_ring(effective_mode) && !led_beat_mode_uses_plane(effective_mode)) {
        led_geo_clear_locked();
    }
    if (effective_mode == LED_BEAT_ANIM_PLANE_PAIR){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_geo_plane_pair_trigger_locked(r, g, b);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_PLANE_FAN){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_geo_plane_fan_trigger_locked(r, g, b);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_PLANE_SPAN){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_geo_plane_span_trigger_locked(r, g, b);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_AUDIO_ENERGY){
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_BREATHE){
        led_beat_clear_effect_state_locked();
        s_breathe_anchor_us = now_us;
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_SPARK){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        led_spark_seed_locked(r, g, b);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_SOFT_FLASH){
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        s_flash_ticks = 0;
        s_wave_r = r;
        s_wave_g = g;
        s_wave_b = b;
        s_wave_phase = 0.5f * kLedWavePi;
        s_wave_amp = 1.0f;
        led_pulse_unlock();
        return;
    }
    int chase_profile = led_beat_chase_profile(effective_mode);
    if (chase_profile >= 0){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        led_pulse_update_period_locked();
        led_comet_trigger_locked(r, g, b, chase_profile);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_SHOCK){
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        s_flash_r = r;
        s_flash_g = g;
        s_flash_b = b;
        s_flash_ticks = kLedFlashTicks + 3;
        s_wave_r = r;
        s_wave_g = g;
        s_wave_b = b;
        s_wave_phase = 0.5f * kLedWavePi;
        s_wave_amp = 1.0f;
        led_pulse_spawn_locked(r, g, b, -kLedPulseWidth);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_RING_PULSE){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_geo_ring_trigger_locked(r, g, b);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_RING_TRAIN){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_geo_ring_spawn_scaled_locked(r, g, b,
                                         0.0f,
                                         kLedGeoRingTrainWidthScale);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_PLANE_SWEEP){
        s_flash_ticks = 0;
        s_wave_amp = 0.0f;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        led_geo_plane_trigger_locked(r, g, b);
        led_pulse_unlock();
        return;
    }
    if (effective_mode == LED_BEAT_ANIM_CROSSFIRE){
        s_flash_ticks = 0;
        memset(s_spark_energy, 0, sizeof(s_spark_energy));
        s_wave_amp = 0.0f;
        led_geo_ring_train_trigger_locked(r, g, b,
                                          kLedGeoRingTrainWagons + 1u,
                                          1.35f);
        led_geo_plane_pair_trigger_locked(r, g, b);
        led_pulse_unlock();
        return;
    }

    s_wave_amp = 0.0f;
    memset(s_spark_energy, 0, sizeof(s_spark_energy));
    s_flash_r = r;
    s_flash_g = g;
    s_flash_b = b;
    s_flash_ticks = kLedFlashTicks;
    led_pulse_unlock();
}
