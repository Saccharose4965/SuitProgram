#pragma once
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float temp_c;
    int64_t t_us;
} mpu6500_sample_t;

typedef struct {
    spi_device_handle_t spi;
    const char *tag; // optional per-instance log tag
} mpu6500_config_t;

typedef struct mpu6500_t {
    spi_device_handle_t dev;
    const char *tag;
    bool logged_unexpected_id;
    int hz;
    TaskHandle_t task;
    portMUX_TYPE mux;
    mpu6500_sample_t latest;
} mpu6500_t;

esp_err_t mpu6500_init(mpu6500_t *imu, const mpu6500_config_t *cfg);
esp_err_t mpu6500_read_once(mpu6500_t *imu, mpu6500_sample_t *out);
esp_err_t mpu6500_start_sampler(mpu6500_t *imu, int hz, UBaseType_t prio, int core_id, uint32_t stack_words);
mpu6500_sample_t mpu6500_latest(mpu6500_t *imu);

#ifdef __cplusplus
}
#endif
