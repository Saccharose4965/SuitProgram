#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "mpu6500.h"
#include "orientation.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start IMU sampler + orientation filter service. Safe to call multiple times.
esp_err_t orientation_service_start(void);

// Returns true once orientation has received enough samples to be usable.
bool orientation_service_ready(void);
// Coherent absolute sensor orientation. Feature references are caller-owned.
bool orientation_service_absolute_snapshot(imu_quat_t *q, uint64_t *sample_us);
bool orientation_service_fresh_snapshot(imu_quat_t *q, uint64_t max_age_us);

// Read latest MPU sample from the service-owned sampler. Returns false if unavailable.
bool orientation_service_read_sample(mpu6500_sample_t *out_sample);

// Bias copy/publication is synchronized; applied by the filter's own task.
void orientation_service_get_bias(float out[3]);
bool orientation_service_set_bias(const float bias[3]);

#ifdef __cplusplus
}
#endif
