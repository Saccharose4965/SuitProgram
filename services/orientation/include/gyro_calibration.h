#pragma once
#include "esp_err.h"
#include <stdbool.h>

typedef enum { GYRO_IDLE, GYRO_SETTLING, GYRO_SAMPLING, GYRO_READY,
               GYRO_MOVED, GYRO_NO_DATA, GYRO_SAVED, GYRO_SAVE_FAILED } gyro_calibration_state_t;
typedef struct {
    gyro_calibration_state_t state;
    float progress, bias[3];
} gyro_calibration_status_t;
// Session operations run on the UI task. Leaving cancels unsaved measurements.
void gyro_calibration_begin(void);
void gyro_calibration_cancel(void);
void gyro_calibration_tick(void);
gyro_calibration_status_t gyro_calibration_status(void);
esp_err_t gyro_calibration_save(void);
// Call after SD initialization; invalid/missing files preserve current defaults.
esp_err_t gyro_calibration_load(void);
