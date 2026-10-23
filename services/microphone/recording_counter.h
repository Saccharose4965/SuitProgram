#pragma once
#include <stdint.h>
#include "esp_err.h"

// Private recording-number persistence; called before the recording task starts.
esp_err_t recording_counter_next(uint32_t *out_count);
