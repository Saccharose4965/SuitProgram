#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mic capture is always: mono, 48 kHz, 16-bit PCM in the ring buffer.
// The INMP441 actually delivers 24-bit; we downshift to 16-bit internally.

typedef struct {
    size_t sample_rate_hz;    // 48000
    size_t ring_capacity;     // in samples (int16_t)
    size_t samples_written;   // running total, for debugging/stats
} audio_capture_stats_t;

/**
 * @brief Initialize audio capture ring buffer and I2S RX side.
 *
 * Must be called once before start. Allocates PSRAM for the ring buffer.
 *
 * @param ring_capacity_samples  Size of ring in samples (int16). If 0, a sensible default is used.
 */
esp_err_t audio_capture_init(size_t ring_capacity_samples);

/**
 * @brief Start the capture task.
 *
 * Spawns a high-priority task that continuously reads from I2S RX and fills
 * the ring buffer.
 */
esp_err_t audio_capture_start(void);

/**
 * @brief Stop the capture task (does NOT free the ring buffer).
 */
esp_err_t audio_capture_stop(void);

/**
 * @brief Read the latest N samples from the ring buffer.
 *
 * If requested samples > ring capacity, it is clamped to the ring size.
 * Returns ESP_ERR_INVALID_STATE if capture is not initialized or not running.
 *
 * @param dst          Destination buffer (int16_t)
 * @param max_samples  Number of samples requested
 * @param out_samples  Actual number of samples copied (may be < max_samples)
 */
esp_err_t audio_capture_read_latest(int16_t *dst,
                                    size_t   max_samples,
                                    size_t  *out_samples);

/**
 * @brief Get capture statistics (non-blocking, approximate).
 */
esp_err_t audio_capture_get_stats(audio_capture_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

