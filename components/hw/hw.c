#include "hw.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/i2s_std.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Some 6.x dev snapshots don’t expose 11 dB; map to 12 dB if needed.
#ifndef ADC_ATTEN_DB_11
#define ADC_ATTEN_DB_11 ADC_ATTEN_DB_12
#endif

#if __has_include("driver/adc.h")
  #include "driver/adc.h"      // for adc_gpio_init (classic ESP32)
#endif
#if __has_include("driver/rtc_io.h")
  #include "driver/rtc_io.h"   // for rtc_gpio_hold_dis
#endif

// ======================================================================
// Module state
// ======================================================================

static const char* TAG = "hw";

// SPI
static bool s_spi2_inited = false;

#define SPI2_TRACKED_DEVICES 8
typedef struct {
    bool used;
    gpio_num_t cs_pin;
    uint32_t clock_hz;
    uint8_t spi_mode;
    spi_device_handle_t handle;
} spi2_device_slot_t;
static spi2_device_slot_t s_spi2_devices[SPI2_TRACKED_DEVICES];

// I2S
static i2s_chan_handle_t s_i2s_tx = NULL;
static i2s_chan_handle_t s_i2s_rx = NULL;

// ADC (oneshot)
static adc_oneshot_unit_handle_t s_adc_unit = NULL;
static adc_cali_handle_t         s_adc_cali = NULL;
static bool                      s_adc_cali_enabled = false;
static SemaphoreHandle_t         s_adc_mutex = NULL;
static portMUX_TYPE              s_adc_mutex_create_lock = portMUX_INITIALIZER_UNLOCKED;

// ======================================================================
// GPIO helpers
// ======================================================================

static inline bool gpio_is_valid_num(gpio_num_t pin) {
    return (pin >= 0) && (pin < GPIO_NUM_MAX);
}
static inline uint64_t gpio_pin_mask(gpio_num_t pin) {
    // Only compute mask for valid pins to avoid UB on shifts
    return gpio_is_valid_num(pin) ? (1ULL << (unsigned)pin) : 0ULL;
}

// Drive known-safe defaults for SPI-attached devices
void hw_gpio_init(void)
{
    // --- CS lines default HIGH; OLED DC default LOW ---
    const gpio_num_t cs_pins[] = {
        (gpio_num_t)PIN_CS_OLED,
        (gpio_num_t)PIN_CS_SD,
        (gpio_num_t)PIN_CS_IMU1,
        (gpio_num_t)PIN_CS_IMU2,
        (gpio_num_t)PIN_CS_IMU3,
    };

    uint64_t cs_mask = 0;
    for (size_t i = 0; i < sizeof(cs_pins)/sizeof(cs_pins[0]); ++i) {
        cs_mask |= gpio_pin_mask(cs_pins[i]);
    }
    if (cs_mask) {
        gpio_config_t io_cs = {
            .pin_bit_mask = cs_mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,   // weak but helps keep bare sockets idle
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_cs);
        for (size_t i = 0; i < sizeof(cs_pins)/sizeof(cs_pins[0]); ++i) {
            if (gpio_is_valid_num(cs_pins[i])) {
                gpio_set_level(cs_pins[i], 1);   // deselect
            }
        }
    }

    // OLED DC (no pull-up), default LOW
    if (gpio_is_valid_num((gpio_num_t)PIN_DC_OLED)) {
        uint64_t dc_mask = gpio_pin_mask((gpio_num_t)PIN_DC_OLED);
        gpio_config_t io_dc = {
            .pin_bit_mask = dc_mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_dc);
        gpio_set_level((gpio_num_t)PIN_DC_OLED, 0);
    }

    // Optional OLED reset pin (may be NC)
    if (gpio_is_valid_num((gpio_num_t)PIN_RST_OLED)) {
        uint64_t mask = gpio_pin_mask((gpio_num_t)PIN_RST_OLED);
        gpio_config_t io = {
            .pin_bit_mask = mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        gpio_set_level((gpio_num_t)PIN_RST_OLED, 1);
    }
}

// Public helper: ensure *all* SPI CS are high (use before SD mount)
void hw_spi2_idle_all_cs_high(void)
{
    const gpio_num_t cs_pins[] = {
        (gpio_num_t)PIN_CS_OLED,
        (gpio_num_t)PIN_CS_SD,
        (gpio_num_t)PIN_CS_IMU1,
        (gpio_num_t)PIN_CS_IMU2,
        (gpio_num_t)PIN_CS_IMU3,
    };
    for (size_t i = 0; i < sizeof(cs_pins)/sizeof(cs_pins[0]); ++i) {
        if (gpio_is_valid_num(cs_pins[i])) {
            gpio_set_level(cs_pins[i], 1);
        }
    }
    ESP_LOGI(TAG, "SPI CS lines driven HIGH");
}


// ======================================================================
// SPI2 bus
// ======================================================================

esp_err_t hw_spi2_init_once(void)
{
    if (s_spi2_inited) return ESP_OK;

    hw_gpio_init();

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 4096,
        .intr_flags = 0
    };

    esp_err_t err = spi_bus_initialize(HW_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err == ESP_ERR_INVALID_STATE) {
        // already initialized elsewhere
        s_spi2_inited = true;
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "spi_bus_initialize");
    s_spi2_inited = true;

    ESP_LOGI(TAG, "SPI2 initialized: HOST=%d MOSI=%d MISO=%d CLK=%d",
             HW_SPI_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK);
    return ESP_OK;
}

esp_err_t hw_spi2_find_device(gpio_num_t cs_pin, spi_device_handle_t *out_handle)
{
    if (!out_handle) return ESP_ERR_INVALID_ARG;
    if (!gpio_is_valid_num(cs_pin)) return ESP_ERR_INVALID_ARG;

    for (size_t i = 0; i < SPI2_TRACKED_DEVICES; ++i) {
        if (!s_spi2_devices[i].used) continue;
        if (s_spi2_devices[i].cs_pin != cs_pin) continue;
        *out_handle = s_spi2_devices[i].handle;
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t hw_spi2_add_device(gpio_num_t cs_pin, uint32_t clock_hz,
                             uint8_t spi_mode, int queue_size,
                             spi_device_handle_t* out_handle)
{
    if (!out_handle) return ESP_ERR_INVALID_ARG;

    if (!s_spi2_inited) {
        ESP_RETURN_ON_ERROR(hw_spi2_init_once(), TAG, "init spi2");
    }

    spi_device_handle_t existing = NULL;
    if (hw_spi2_find_device(cs_pin, &existing) == ESP_OK && existing) {
        *out_handle = existing;
        return ESP_OK;
    }

    // Make sure the CS pin is configured as output high before attaching.
    if (gpio_is_valid_num(cs_pin)) {
        gpio_config_t io = {
            .pin_bit_mask = gpio_pin_mask(cs_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        gpio_set_level(cs_pin, 1);
    }

    spi_device_interface_config_t dev = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = spi_mode,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 2,      // small lead/lag is enough; keep bus snappy
        .cs_ena_posttrans = 2,
        .clock_speed_hz = (int)clock_hz,
        .spics_io_num = cs_pin,
        .queue_size = queue_size,
        .flags = SPI_DEVICE_NO_DUMMY,
        .input_delay_ns = 0
    };
    esp_err_t err = spi_bus_add_device(HW_SPI_HOST, &dev, out_handle);
    if (err != ESP_OK) return err;

    for (size_t i = 0; i < SPI2_TRACKED_DEVICES; ++i) {
        if (s_spi2_devices[i].used) continue;
        s_spi2_devices[i].used = true;
        s_spi2_devices[i].cs_pin = cs_pin;
        s_spi2_devices[i].clock_hz = clock_hz;
        s_spi2_devices[i].spi_mode = spi_mode;
        s_spi2_devices[i].handle = *out_handle;
        return ESP_OK;
    }

    ESP_LOGW(TAG, "SPI2 device tracking full; CS%d reuse lookup disabled for new entry", (int)cs_pin);
    return ESP_OK;
}


// ======================================================================
// I2S (std API) — MAX98357A TX (16-bit mono @ 48 kHz), INMP441 RX (24-bit)
// ======================================================================

esp_err_t hw_i2s_init_tx_16bit_mono_48k(void)
{
    if (s_i2s_tx) return ESP_OK;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL), TAG, "i2s_new_channel TX");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_LRCK,
            .dout = PIN_I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    // Mono Left output for MAX98357A
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg), TAG, "i2s_channel_init_std_mode TX");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s_tx), TAG, "i2s_channel_enable TX");
    ESP_LOGI(TAG, "I2S TX ready: 16-bit mono-left @48k (BCLK=%d, LRCK=%d, DOUT=%d)",
             PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DOUT);
    return ESP_OK;
}

esp_err_t hw_i2s_init_rx_24bit_mono_48k(void)
{
    if (s_i2s_rx) return ESP_OK;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &s_i2s_rx), TAG, "i2s_new_channel RX");

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_LRCK,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_I2S_DIN,
        },
    };
    // INMP441 outputs on LEFT channel by default
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_rx, &std_cfg), TAG, "i2s_channel_init_std_mode RX");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s_rx), TAG, "i2s_channel_enable RX");
    ESP_LOGI(TAG, "I2S RX ready: 24-bit mono-left @48k (BCLK=%d, LRCK=%d, DIN=%d)",
             PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DIN);
    return ESP_OK;
}


// ======================================================================
// UART0 — GPS
// ======================================================================

esp_err_t hw_uart_gps_init(int baud)
{
    // NOTE: Using UART0 requires disabling the console on UART0 in menuconfig:
    //   Component config → ESP System Settings → Channel for console output → None
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(HW_GPS_UART_NUM, 2048, 0, 0, NULL, 0), TAG, "uart_driver_install");
    ESP_RETURN_ON_ERROR(uart_param_config(HW_GPS_UART_NUM, &cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(HW_GPS_UART_NUM, HW_GPS_TXD, HW_GPS_RXD,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin");
    ESP_LOGI(TAG, "GPS UART0 ready on TX=%d RX=%d @%d", HW_GPS_TXD, HW_GPS_RXD, baud);
    return ESP_OK;
}
// ======================================================================
// ADC1 — bring-up + raw / mV reads (Hall A/B + buttons ladder)
// ======================================================================

// If your toolchain doesn't expose 11 dB, the alias above maps it to 12 dB
#ifndef ADC_ATTEN_DB_11
#define ADC_ATTEN_DB_11 ADC_ATTEN_DB_12
#endif

// Optionally override at build time: -DHW_ADC_FALLBACK_FS_MV=3300
#ifndef HW_ADC_FALLBACK_FS_MV
#define HW_ADC_FALLBACK_FS_MV 3300
#endif

static inline void hw_adc_release_rtc_and_route_pads(void)
{
    // Make sure pads aren't RTC-held (common cause of "stuck" readings)
#if __has_include("driver/rtc_io.h")
    rtc_gpio_hold_dis((gpio_num_t)PIN_BUTTONS_LADDER);  // GPIO39 / SENSOR_VN
    rtc_gpio_hold_dis((gpio_num_t)PIN_HALL_A);          // GPIO34
    rtc_gpio_hold_dis((gpio_num_t)PIN_HALL_B);          // GPIO35
#endif

    // Explicitly attach pads to ADC1 (classic ESP32 path)
#if __has_include("driver/adc.h")
    adc_gpio_init(ADC_UNIT_1, HW_BUTTONS_ADC_CH);       // ADC1_CH3 ← GPIO39
    adc_gpio_init(ADC_UNIT_1, HW_HALL_A_ADC_CH);        // ADC1_CH6 ← GPIO34
    adc_gpio_init(ADC_UNIT_1, HW_HALL_B_ADC_CH);        // ADC1_CH7 ← GPIO35
#endif
}

static esp_err_t hw_adc_mutex_ensure(void)
{
    if (s_adc_mutex) return ESP_OK;
    taskENTER_CRITICAL(&s_adc_mutex_create_lock);
    if (!s_adc_mutex) {
        s_adc_mutex = xSemaphoreCreateMutex();
    }
    taskEXIT_CRITICAL(&s_adc_mutex_create_lock);
    return s_adc_mutex ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t hw_adc_lock(TickType_t timeout_ticks)
{
    esp_err_t err = hw_adc_mutex_ensure();
    if (err != ESP_OK) return err;
    if (xSemaphoreTake(s_adc_mutex, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void hw_adc_unlock(void)
{
    if (s_adc_mutex) {
        (void)xSemaphoreGive(s_adc_mutex);
    }
}

static esp_err_t hw_adc1_init_default_locked(void)
{
    if (s_adc_unit) return ESP_OK;

    hw_adc_release_rtc_and_route_pads();

    // Create oneshot unit (ADC1)
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_unit);
    ESP_RETURN_ON_ERROR(err, TAG, "adc_oneshot_new_unit");

    // Optional: sanity check the IO↔channel mapping (logs once)
#if __has_include("esp_adc/adc_oneshot.h")
    adc_channel_t ch_tmp;  // not strictly needed, but helps catch miswires
    if (adc_oneshot_io_to_channel((gpio_num_t)PIN_BUTTONS_LADDER, ADC_UNIT_1, &ch_tmp) == ESP_OK) {
        ESP_LOGI(TAG, "ADC1 map: GPIO%d -> CH%d (expect CH3 for GPIO39)", PIN_BUTTONS_LADDER, ch_tmp);
    } else {
        ESP_LOGW(TAG, "ADC1 map: unable to map GPIO%d; continuing with configured channel", PIN_BUTTONS_LADDER);
    }
#endif

    // Configure used channels at ~0..3.3 V
    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_config_channel(s_adc_unit, HW_HALL_A_ADC_CH,  &ch_cfg));
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_config_channel(s_adc_unit, HW_HALL_B_ADC_CH,  &ch_cfg));
    ESP_ERROR_CHECK_WITHOUT_ABORT(adc_oneshot_config_channel(s_adc_unit, HW_BUTTONS_ADC_CH, &ch_cfg));

    // Try enable calibration (line fitting)
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ch_cfg.atten,
        .bitwidth = ch_cfg.bitwidth,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        s_adc_cali_enabled = true;
        ESP_LOGI(TAG, "ADC1 calibration (line fitting) enabled");
    } else {
        s_adc_cali_enabled = false;
        ESP_LOGW(TAG, "ADC1 calibration not available; using uncalibrated fallback");
    }

    ESP_LOGI(TAG, "ADC1 oneshot ready (atten=%d, 12-bit)", ch_cfg.atten);
    return ESP_OK;
}

esp_err_t hw_adc1_init_default(void)
{
    esp_err_t err = hw_adc_lock(pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;
    err = hw_adc1_init_default_locked();
    hw_adc_unlock();
    return err;
}

esp_err_t hw_adc1_read_raw(adc_channel_t ch, int *out_value)
{
    if (!out_value) return ESP_ERR_INVALID_ARG;
    esp_err_t err = hw_adc_lock(pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;
    err = hw_adc1_init_default_locked();
    if (err == ESP_OK) {
        err = adc_oneshot_read(s_adc_unit, ch, out_value);
    }
    hw_adc_unlock();
    return err;
}

esp_err_t hw_adc1_read_mv(adc_channel_t ch, int *out_mv)
{
    if (!out_mv) return ESP_ERR_INVALID_ARG;
    esp_err_t err = hw_adc_lock(pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;
    err = hw_adc1_init_default_locked();
    if (err != ESP_OK) {
        hw_adc_unlock();
        return err;
    }

    int raw = 0;
    err = adc_oneshot_read(s_adc_unit, ch, &raw);
    if (err != ESP_OK) {
        hw_adc_unlock();
        return err;
    }

    if (s_adc_cali_enabled && s_adc_cali) {
        int mv = 0;
        err = adc_cali_raw_to_voltage(s_adc_cali, raw, &mv);
        if (err == ESP_OK) {
            *out_mv = mv;
            hw_adc_unlock();
            return ESP_OK;
        }
        ESP_LOGW(TAG, "ADC cali failed (e=%d), using linear fallback", err);
    }

    *out_mv = (raw * HW_ADC_FALLBACK_FS_MV) / 4095;
    hw_adc_unlock();
    return ESP_OK;
}

esp_err_t hw_buttons_read(hw_button_id_t *out_button)
{
    if (!out_button) return ESP_ERR_INVALID_ARG;

    int mv = 0;
    ESP_RETURN_ON_ERROR(hw_adc1_read_mv(HW_BUTTONS_ADC_CH, &mv), TAG, "read mv");

    // Target levels (mV): A≈825, B≈1650, C≈2475, D≈3300
    // Include diagnostic taps around 1.1 V and 2.2 V for divider calibration.
    const struct { int mv; hw_button_id_t id; } targets[] = {
        {3300, HW_BTN_D},
        {2475, HW_BTN_C},
        {2200, HW_BTN_NONE}, // divider sanity check
        {1650, HW_BTN_B},
        {1100, HW_BTN_NONE}, // divider sanity check
        {825,  HW_BTN_A},
    };
    const int tol = 200; // +/-200 mV window per step

    int best_diff = tol + 1;
    hw_button_id_t best = HW_BTN_NONE;
    for (size_t i = 0; i < sizeof(targets)/sizeof(targets[0]); ++i){
        int diff = targets[i].mv - mv;
        if (diff < 0) diff = -diff;
        if (diff <= tol && diff < best_diff){
            best_diff = diff;
            best = targets[i].id;
        }
    }

    *out_button = best;
    return ESP_OK;
}
