#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start background power monitor (reads pack voltage via ADC divider).
esp_err_t power_monitor_start(void);

#ifdef __cplusplus
}
#endif
