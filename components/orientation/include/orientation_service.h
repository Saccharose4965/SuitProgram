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

// Access shared orientation state owned by the service.
imu_orientation_t *orientation_service_ctx(void);

// Read latest MPU sample from the service-owned sampler. Returns false if unavailable.
bool orientation_service_read_sample(mpu6500_sample_t *out_sample);

// Recenter current orientation as reference "forward".
void orientation_service_recenter(void);

#ifdef __cplusplus
}
#endif
