#pragma once
// ===== Tron Suit: Hardware mapping for ESP32-WROVER-E (16 MB) =====
//
// Board overview
// - MCU: ESP32-WROVER-E (16 MB flash), bare module
// - Flash/programming via borrowed USB-UART (Arduino) @155200 baud
// - Power: 3S Li-ion +BMS → 12 V (LED), 5 V (logic/audio), 3.3 V (logic/ESP)
// - ESP runs from 1 S rail (3.3 V). All logic at 3.3 V unless noted.
//
// Peripheral summary
// - SPI2 (HSPI): OLED (SSD1309), microSD, up to 3× MPU6500
// - UART0: GPS NEO-6M → GPIO3 (RX0) / GPIO1 (TX0)  → console must be disabled
// - I2S TX: MAX98357A (left mono) | I2S RX: INMP441 microphone
// - ADC1: Hall A/B (GPIO34/35) + buttons ladder (GPIO39 VN)
// - WS2815 LED strips on GPIO32/33 (12 V power via level shifter)
// - Three buttons via resistor ladder to GPIO39
//
// This header centralizes all pin definitions and small bring-up helpers.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Use the classic header paths; the includes are provided by split components.
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/i2s_std.h"

// New ADC lives in esp_adc component on your snapshot
#if __has_include("esp_adc/adc_oneshot.h")
  #include "esp_adc/adc_oneshot.h"
  #include "esp_adc/adc_cali.h"
  #include "esp_adc/adc_cali_scheme.h"
  // Compatibility aliases
  #define ADC1_CHANNEL_3  ADC_CHANNEL_3
  #define ADC1_CHANNEL_6  ADC_CHANNEL_6
  #define ADC1_CHANNEL_7  ADC_CHANNEL_7
#endif


#ifdef __cplusplus
extern "C" {
#endif

// SD mount point used across the project
#ifndef HW_SD_MOUNT_POINT
#define HW_SD_MOUNT_POINT   "/sdcard"
#endif

// Persistent counter file (hidden)
#ifndef HW_SD_COUNTER_PATH
#define HW_SD_COUNTER_PATH  HW_SD_MOUNT_POINT "/.rec_counter"
#endif

// ======================================================================
// SPI2 (HSPI)  — OLED, SD, up to 3× MPU6500
// ======================================================================
#define HW_SPI_HOST        SPI2_HOST
#define PIN_SPI_MOSI       23
#define PIN_SPI_MISO       19
#define PIN_SPI_CLK        18

// Individual chip-selects
#define PIN_CS_OLED        5      // OLED SSD1309 — with external pull-up
#define PIN_CS_SD          21     // microSD
#define PIN_CS_IMU1        14     // MPU6500 #1
#define PIN_CS_IMU2        26     // MPU6500 #2
#define PIN_CS_IMU3        27     // MPU6500 #3

// OLED sideband pins
#define PIN_DC_OLED        13
#define PIN_RST_OLED       GPIO_NUM_NC   // not wired

// ======================================================================
// I2S  — MAX98357A (TX) and INMP441 (RX)
// ======================================================================
#define PIN_I2S_BCLK       4      // BCLK / SCK
#define PIN_I2S_LRCK       22     // WS / LRC
#define PIN_I2S_DOUT       25     // SD TX → MAX98357A
#define PIN_I2S_DIN        36     // SD RX ← INMP441 (input-only)

// ======================================================================
// LED strips (WS2815 via level shifter)
// ======================================================================
#define PIN_LED_STRIP_A    32
#define PIN_LED_STRIP_B    33

// ======================================================================
// Analog sensors / buttons
// ======================================================================
#define PIN_HALL_A         34    // ADC1_CH6 (input-only)
#define PIN_HALL_B         35    // ADC1_CH7 (input-only)
#define PIN_BUTTONS_LADDER 39    // SENSOR_VN → ADC1_CH3

// ======================================================================
// UART — GPS NEO-6M on UART0
// ======================================================================
#define HW_GPS_UART_NUM    UART_NUM_0
#define HW_GPS_TXD         1     // → GPS RX
#define HW_GPS_RXD         3     // ← GPS TX
#define HW_GPS_BAUD_DEFAULT 155200

// ======================================================================
// Convenience aliases / ADC channels
// ======================================================================
#define HW_BUTTONS_ADC_CH  ADC1_CHANNEL_3  // GPIO39
#define HW_HALL_A_ADC_CH   ADC1_CHANNEL_6  // GPIO34
#define HW_HALL_B_ADC_CH   ADC1_CHANNEL_7  // GPIO35

// ======================================================================
// Bring-up helpers
// ======================================================================

// Configure static GPIOs (CS default high, DC low etc.)
void hw_gpio_init_fixed(void);

// Initialize shared SPI2 bus (idempotent)
esp_err_t hw_spi2_init_once(void);
void hw_spi2_idle_all_cs_high(void);

// Register a new device on SPI2
esp_err_t hw_spi2_add_device(gpio_num_t cs_pin,
                             uint32_t   clock_hz,
                             uint8_t    spi_mode,
                             int        queue_size,
                             spi_device_handle_t* out_handle);

// Convenience wrappers
static inline esp_err_t hw_spi2_add_oled(spi_device_handle_t* h) {
    // OLED runs comfortably at 10 MHz on solid wiring; faster can risk artifacts.
    return hw_spi2_add_device((gpio_num_t)PIN_CS_OLED, 10000000, 0, 4, h);
}
static inline esp_err_t hw_spi2_add_sd(spi_device_handle_t* h) {
    return hw_spi2_add_device((gpio_num_t)PIN_CS_SD, 20000000, 0, 8, h);
}
static inline esp_err_t hw_spi2_add_mpu(gpio_num_t cs_pin, spi_device_handle_t* h) {
    return hw_spi2_add_device(cs_pin, 8000000, 0, 4, h);
}

// ======================================================================
// I2S helpers — (MAX98357A TX / INMP441 RX)
// ======================================================================
esp_err_t hw_i2s_init_tx_16bit_mono_48k(void);
esp_err_t hw_i2s_init_rx_24bit_mono_48k(void);

// ======================================================================
// UART — GPS
// ======================================================================
esp_err_t hw_uart_gps_init(int baud);

// ======================================================================
// ADC — Hall sensors and buttons
// ======================================================================

esp_err_t hw_adc1_init_default(void);

// Low-level reads
esp_err_t hw_adc1_read_raw(adc_channel_t ch, int *out_value);
esp_err_t hw_adc1_read_mv(adc_channel_t ch, int *out_mv);

// ----------------------------------------------------------------------
// Buttons ladder (GPIO39 / ADC1_CH3)
// ----------------------------------------------------------------------

typedef enum {
    HW_BTN_NONE = 0,
    HW_BTN_A,
    HW_BTN_B,
    HW_BTN_C,
    HW_BTN_D,
} hw_button_id_t;

// Reads the ladder and returns which button is pressed (or NONE)
esp_err_t hw_buttons_read(hw_button_id_t *out_button);
