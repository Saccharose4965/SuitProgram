#pragma once
#include <stdint.h>
#include "esp_err.h"

// Returns and durably records the next automatic recording number.
esp_err_t recording_counter_next(uint32_t *out_count);
