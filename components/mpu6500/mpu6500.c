#include "driver/spi_master.h"
#include "mpu6500.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

#define REG_USER_CTRL   0x6A
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACC_CFG     0x1C
#define REG_ACC_XOUT_H  0x3B

#define WHO_AM_I_VAL_1  0x70
#define WHO_AM_I_VAL_2  0x68
#define WHO_AM_I_VAL_3  0x71 // some compatible parts


/* ---- low-level SPI helpers ---- */

static inline esp_err_t reg_write(mpu6500_t *imu, uint8_t reg, uint8_t val){
    // 2 bytes TX: [reg(0x7F)] [val]
    spi_transaction_t t = {0};
    t.length   = 16; // bits
    t.flags    = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (uint8_t)(reg & 0x7F);
    t.tx_data[1] = val;
    return spi_device_transmit(imu->dev, &t);
}

static inline esp_err_t reg_read(mpu6500_t *imu, uint8_t reg, uint8_t *out){
    // full-duplex: send [reg|0x80, dummy], receive [garbage, value]
    spi_transaction_t t = {0};
    t.length   = 16; // bits
    t.rxlength = t.length;
    t.flags    = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = (uint8_t)(reg | 0x80);
    t.tx_data[1] = 0x00;
    esp_err_t err = spi_device_transmit(imu->dev, &t);
    if (err != ESP_OK) return err;
    *out = t.rx_data[1];
    return ESP_OK;
}

static inline esp_err_t burst_read(mpu6500_t *imu, uint8_t start_reg, uint8_t *out, size_t n){
    // TX must clock n+1 bytes: [reg|0x80, dummy...]; RX will contain [garbage, data...]
    if (n == 0) return ESP_OK;
    if (n + 1 > 64) return ESP_ERR_INVALID_SIZE;

    uint8_t tx[64] = {0};
    uint8_t rx[64] = {0};

    tx[0] = (uint8_t)(start_reg | 0x80);

    spi_transaction_t t = {0};
    t.length   = (n + 1) * 8;
    t.rxlength = t.length;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(imu->dev, &t);
    if (err == ESP_OK) {
        memcpy(out, &rx[1], n); // skip first garbage byte
    }
    return err;
}

/* ---- public API ---- */

esp_err_t mpu6500_init(mpu6500_t *imu, const mpu6500_config_t *cfg){
    if (!imu || !cfg || !cfg->spi) return ESP_ERR_INVALID_ARG;

    imu->dev = cfg->spi;
    imu->tag = cfg->tag ? cfg->tag : "mpu6500";
    imu->logged_unexpected_id = false;
    imu->task = NULL;
    imu->hz = 0;
    memset(&imu->latest, 0, sizeof(imu->latest));

    // Wake + basic config
    esp_err_t err;
    if ((err = reg_write(imu, REG_PWR_MGMT_1, 0x80)) != ESP_OK) return err; // device reset
    vTaskDelay(pdMS_TO_TICKS(100));
    if ((err = reg_write(imu, REG_PWR_MGMT_1, 0x01)) != ESP_OK) return err; // PLL, wake
    vTaskDelay(pdMS_TO_TICKS(50));
    if ((err = reg_write(imu, REG_USER_CTRL, 0x10)) != ESP_OK) return err;   // disable I2C
    vTaskDelay(pdMS_TO_TICKS(10));
    if ((err = reg_write(imu, REG_SMPLRT_DIV, 0x04)) != ESP_OK) return err; // 1k/(1+4)=200 Hz
    if ((err = reg_write(imu, REG_CONFIG,     0x03)) != ESP_OK) return err; // DLPF ~44/42 Hz
    if ((err = reg_write(imu, REG_GYRO_CFG,   0x08)) != ESP_OK) return err; // ±500 dps
    if ((err = reg_write(imu, REG_ACC_CFG,    0x08)) != ESP_OK) return err; // ±4 g

    uint8_t who = 0;
    for (int i = 0; i < 4; ++i){
        if ((err = reg_read(imu, REG_WHO_AM_I, &who)) != ESP_OK) return err;
        if (who == WHO_AM_I_VAL_1 || who == WHO_AM_I_VAL_2 || who == WHO_AM_I_VAL_3) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!imu->logged_unexpected_id) {
        uint8_t pwr = 0;
        esp_err_t pwr_err = reg_read(imu, REG_PWR_MGMT_1, &pwr);
        if (pwr_err == ESP_OK) {
            ESP_LOGW(imu->tag, "WHO_AM_I unexpected: 0x%02X (PWR_MGMT_1=0x%02X)", who, pwr);
        } else {
            ESP_LOGW(imu->tag, "WHO_AM_I unexpected: 0x%02X (PWR_MGMT_1 read failed: %s)", who, esp_err_to_name(pwr_err));
        }
        imu->logged_unexpected_id = true;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

static inline int16_t be16(const uint8_t *p){ return (int16_t)((p[0] << 8) | p[1]); }

esp_err_t mpu6500_read_once(mpu6500_t *imu, mpu6500_sample_t *o){
    if (!imu || !imu->dev || !o) return ESP_ERR_INVALID_STATE;

    uint8_t raw[14]; // accel(6) + temp(2) + gyro(6)
    esp_err_t err = burst_read(imu, REG_ACC_XOUT_H, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    int16_t ax=be16(raw+0),  ay=be16(raw+2),  az=be16(raw+4);
    int16_t tp=be16(raw+6);
    int16_t gx=be16(raw+8),  gy=be16(raw+10), gz=be16(raw+12);

    // scales: ±4g -> 8192 LSB/g ; ±500 dps -> 65.5 LSB/(°/s)
    o->ax_g   = ax / 8192.0f;
    o->ay_g   = ay / 8192.0f;
    o->az_g   = az / 8192.0f;
    o->gx_dps = gx / 65.5f;
    o->gy_dps = gy / 65.5f;
    o->gz_dps = gz / 65.5f;
    o->temp_c = (tp / 333.87f) + 21.0f;
    o->t_us   = esp_timer_get_time();
    return ESP_OK;
}

/* ---- lightweight sampler ---- */
static void sampler_task(void *arg){
    mpu6500_t *imu = (mpu6500_t*)arg;

    float ms_f = (imu->hz > 0) ? (1000.0f / (float)imu->hz) : 10.0f;
    TickType_t dt = pdMS_TO_TICKS(ms_f);
    if (dt == 0) dt = 1;
    TickType_t last = xTaskGetTickCount();
    for(;;){
        mpu6500_sample_t s;
        if (mpu6500_read_once(imu, &s) == ESP_OK) {
            taskENTER_CRITICAL(&imu->mux);
            imu->latest = s;
            taskEXIT_CRITICAL(&imu->mux);
        }
        vTaskDelayUntil(&last, dt);
    }
}

esp_err_t mpu6500_start_sampler(mpu6500_t *imu, int hz, UBaseType_t prio, int core_id, uint32_t stack_words){
    if (!imu || !imu->dev) return ESP_ERR_INVALID_STATE;
    if (imu->task) return ESP_OK; // already running
    if (hz <= 0) hz = 100;
    imu->hz = hz;
    if (prio == 0) prio = 2;
    if (stack_words == 0) stack_words = 4096 / sizeof(StackType_t);
    if (core_id < 0) core_id = 1;

    BaseType_t ok = xTaskCreatePinnedToCore(
        sampler_task, imu->tag, stack_words,
        imu, prio, &imu->task, core_id
    );
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

mpu6500_sample_t mpu6500_latest(mpu6500_t *imu){
    mpu6500_sample_t c = {0};
    if (!imu) return c;
    taskENTER_CRITICAL(&imu->mux);
    c = imu->latest;
    taskEXIT_CRITICAL(&imu->mux);
    return c;
}
