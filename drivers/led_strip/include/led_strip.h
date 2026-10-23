#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_STRIP_OUTPUT_COUNT 2

esp_err_t led_strip_init(int gpio_a, int gpio_b);
esp_err_t led_strip_write(const uint8_t *frames[LED_STRIP_OUTPUT_COUNT],
                          const size_t lengths[LED_STRIP_OUTPUT_COUNT]);

#ifdef __cplusplus
}
#endif
