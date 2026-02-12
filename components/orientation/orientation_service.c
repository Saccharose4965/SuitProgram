#include "orientation_service.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"

#include "hw.h"
#include "mpu6500.h"

static const char *TAG = "ori_svc";

static bool s_started = false;
static TaskHandle_t s_orient_task = NULL;
static mpu6500_t s_imu = { .mux = portMUX_INITIALIZER_UNLOCKED };
static imu_orientation_t s_ori = {0};

static void orientation_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(5); // ~200 Hz
    if (period < 1) period = 1;
    int64_t last_t_us = 0;

    for (;;) {
        vTaskDelayUntil(&last, period);
        mpu6500_sample_t raw = mpu6500_latest(&s_imu);
        if (raw.t_us == 0 || raw.t_us == last_t_us) continue;
        last_t_us = raw.t_us;
        imu_orientation_update(
            &s_ori,
            raw.ax_g, raw.ay_g, raw.az_g,
            raw.gx_dps, raw.gy_dps, raw.gz_dps,
            raw.t_us
        );
    }
}

esp_err_t orientation_service_start(void)
{
    if (s_started) return ESP_OK;

    ESP_RETURN_ON_ERROR(hw_spi2_init_once(), TAG, "spi2 init");
    hw_spi2_idle_all_cs_high();

    const gpio_num_t cs = (gpio_num_t)PIN_CS_IMU1;
    const uint32_t freqs[] = { 8000000, 1000000 };

    for (size_t f = 0; f < sizeof(freqs) / sizeof(freqs[0]); ++f) {
        spi_device_handle_t dev = NULL;
        if (hw_spi2_add_device(cs, freqs[f], 0, 4, &dev) != ESP_OK || !dev) continue;

        mpu6500_config_t cfg = {
            .spi = dev,
            .tag = "mpu_orient",
        };

        if (mpu6500_init(&s_imu, &cfg) != ESP_OK) {
            (void)spi_bus_remove_device(dev);
            continue;
        }

        imu_orientation_init(&s_ori, 0.05f);

        if (!s_orient_task) {
            BaseType_t ok = xTaskCreatePinnedToCore(
                orientation_task, "imu_orient", 3072, NULL, 2, &s_orient_task, 1
            );
            if (ok != pdPASS) {
                s_orient_task = NULL;
                (void)spi_bus_remove_device(dev);
                return ESP_ERR_NO_MEM;
            }
        }

        if (mpu6500_start_sampler(&s_imu, 200, 2, 1, 0) == ESP_OK) {
            s_started = true;
            ESP_LOGI(TAG, "orientation service started @CS%u (%u Hz)", (unsigned)cs, (unsigned)freqs[f]);
            return ESP_OK;
        }

        if (s_orient_task) {
            vTaskDelete(s_orient_task);
            s_orient_task = NULL;
        }
        (void)spi_bus_remove_device(dev);
    }

    return ESP_ERR_NOT_FOUND;
}

bool orientation_service_ready(void)
{
    return s_started && imu_orientation_ready(&s_ori);
}

imu_orientation_t *orientation_service_ctx(void)
{
    return &s_ori;
}

bool orientation_service_read_sample(mpu6500_sample_t *out_sample)
{
    if (!out_sample || !s_started) return false;
    mpu6500_sample_t sample = mpu6500_latest(&s_imu);
    if (sample.t_us == 0) return false;
    *out_sample = sample;
    return true;
}

void orientation_service_recenter(void)
{
    imu_orientation_recenter(&s_ori);
}
