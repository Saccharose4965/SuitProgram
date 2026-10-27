#include "orientation_service.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/spi_master.h"

#include "hw.h"
#include "mpu6500.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ori_svc";

static bool s_started = false;
static StaticSemaphore_t s_start_mutex_buffer;
static SemaphoreHandle_t s_start_mutex = NULL;
static portMUX_TYPE s_start_mutex_init_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_orient_task = NULL;
static mpu6500_t s_imu = { .mux = portMUX_INITIALIZER_UNLOCKED };
static imu_orientation_t s_ori = {0};
static portMUX_TYPE s_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;
static imu_quat_t s_snapshot;
static uint64_t s_snapshot_time;
static float s_bias[3]={-1.9f,.5f,0};

static SemaphoreHandle_t orientation_start_mutex(void)
{
    if (s_start_mutex) return s_start_mutex;
    taskENTER_CRITICAL(&s_start_mutex_init_lock);
    if (!s_start_mutex) {
        s_start_mutex = xSemaphoreCreateMutexStatic(&s_start_mutex_buffer);
    }
    taskEXIT_CRITICAL(&s_start_mutex_init_lock);
    return s_start_mutex;
}

static bool orientation_started(void)
{
    bool started;
    portENTER_CRITICAL(&s_snapshot_mux);
    started = s_started;
    portEXIT_CRITICAL(&s_snapshot_mux);
    return started;
}

static void orientation_publish_started(void)
{
    portENTER_CRITICAL(&s_snapshot_mux);
    s_started = true;
    portEXIT_CRITICAL(&s_snapshot_mux);
}

void orientation_service_get_bias(float out[3])
{
    if(!out) return;
    portENTER_CRITICAL(&s_snapshot_mux);
    memcpy(out,s_bias,sizeof(s_bias));
    portEXIT_CRITICAL(&s_snapshot_mux);
}
bool orientation_service_set_bias(const float bias[3])
{
    if(!bias) return false;
    for(int i=0;i<3;i++) if(!isfinite(bias[i]) || fabsf(bias[i])>10) return false;
    portENTER_CRITICAL(&s_snapshot_mux);
    memcpy(s_bias,bias,sizeof(s_bias));
    portEXIT_CRITICAL(&s_snapshot_mux);
    return true;
}

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
        orientation_service_get_bias(s_ori.gyro_bias_dps);
        imu_orientation_update(
            &s_ori,
            raw.ax_g, raw.ay_g, raw.az_g,
            raw.gx_dps, raw.gy_dps, raw.gz_dps,
            raw.t_us
        );
        portENTER_CRITICAL(&s_snapshot_mux);
        s_snapshot = s_ori.q_BW;
        s_snapshot_time = s_ori.ready ? raw.t_us : 0;
        portEXIT_CRITICAL(&s_snapshot_mux);
    }
}

static esp_err_t orientation_start_locked(void)
{
    if (orientation_started()) return ESP_OK;

    esp_err_t err = hw_spi2_init_once();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi2 init: %s", esp_err_to_name(err));
        return err;
    }
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
            s_imu.dev = NULL;
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
                s_imu.dev = NULL;
                return ESP_ERR_NO_MEM;
            }
        }

        if (mpu6500_start_sampler(&s_imu, 200, 2, 1, 0) == ESP_OK) {
            orientation_publish_started();
            ESP_LOGI(TAG, "orientation service started @CS%u (%u Hz)", (unsigned)cs, (unsigned)freqs[f]);
            return ESP_OK;
        }

        if (s_orient_task) {
            vTaskDelete(s_orient_task);
            s_orient_task = NULL;
        }
        (void)spi_bus_remove_device(dev);
        s_imu.dev = NULL;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t orientation_service_start(void)
{
    SemaphoreHandle_t mutex = orientation_start_mutex();
    if (!mutex) return ESP_ERR_NO_MEM;
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    esp_err_t err = orientation_start_locked();
    xSemaphoreGive(mutex);
    return err;
}

bool orientation_service_ready(void)
{
    bool ready;
    portENTER_CRITICAL(&s_snapshot_mux);
    ready = s_started && s_snapshot_time != 0;
    portEXIT_CRITICAL(&s_snapshot_mux);
    return ready;
}

bool orientation_service_absolute_snapshot(imu_quat_t *q, uint64_t *sample_us)
{
    if (!q || !sample_us) return false;
    bool available;
    portENTER_CRITICAL(&s_snapshot_mux);
    *q = s_snapshot;
    *sample_us = s_snapshot_time;
    available = s_started && s_snapshot_time != 0;
    portEXIT_CRITICAL(&s_snapshot_mux);
    return available;
}

bool orientation_service_fresh_snapshot(imu_quat_t *q, uint64_t max_age_us)
{
    uint64_t sample_us;
    if (!orientation_service_absolute_snapshot(q, &sample_us)) return false;
    int64_t now_us = esp_timer_get_time();
    return now_us >= 0 && sample_us <= (uint64_t)now_us &&
           (uint64_t)now_us - sample_us <= max_age_us;
}

bool orientation_service_read_sample(mpu6500_sample_t *out_sample)
{
    if (!out_sample || !orientation_started()) return false;
    mpu6500_sample_t sample = mpu6500_latest(&s_imu);
    if (sample.t_us == 0) return false;
    *out_sample = sample;
    return true;
}
