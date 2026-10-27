#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Loads the highest valid primary/recovery copy. Missing files return 0 + ESP_OK;
// malformed files without a valid recovery copy return ESP_ERR_INVALID_RESPONSE.
esp_err_t scores_load_high(const char *game_key, uint32_t *out_high);

// Persists candidate if higher than current score, returns resulting high score.
esp_err_t scores_update_high(const char *game_key, uint32_t candidate, uint32_t *out_high);

#ifdef __cplusplus
}
#endif
