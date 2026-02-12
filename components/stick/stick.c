#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/spi_master.h"

#include "hw.h"
#include "oled.h"
#include "stick.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "stick";

// ==== minimal MPU6500 helpers (local, multi-device safe) ====
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACC_CFG     0x1C
#define REG_ACC_XOUT_H  0x3B
#define WHO_AM_I_VAL_1  0x70
#define WHO_AM_I_VAL_2  0x68

static inline esp_err_t reg_write(spi_device_handle_t dev, uint8_t reg, uint8_t val){
    spi_transaction_t t = {0};
    t.length = 16;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = (uint8_t)(reg & 0x7F);
    t.tx_data[1] = val;
    return spi_device_transmit(dev, &t);
}

static inline esp_err_t reg_read(spi_device_handle_t dev, uint8_t reg, uint8_t *out){
    spi_transaction_t t = {0};
    t.length = 16;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = (uint8_t)(reg | 0x80);
    t.tx_data[1] = 0;
    esp_err_t e = spi_device_transmit(dev, &t);
    if (e != ESP_OK) return e;
    *out = t.rx_data[1];
    return ESP_OK;
}

static inline int16_t be16(const uint8_t *p){ return (int16_t)((p[0] << 8) | p[1]); }

static esp_err_t burst_read(spi_device_handle_t dev, uint8_t start_reg, uint8_t *buf, size_t n){
    if (n == 0) return ESP_OK;
    uint8_t tx_small[20];
    uint8_t *tx = tx_small;
    bool heap = false;
    size_t len = n + 1;
    if (len > sizeof(tx_small)){
        tx = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_DEFAULT);
        if (!tx) return ESP_ERR_NO_MEM;
        heap = true;
    }
    tx[0] = (uint8_t)(start_reg | 0x80);
    memset(&tx[1], 0, n);
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = buf;
    esp_err_t e = spi_device_transmit(dev, &t);
    if (heap) free(tx);
    if (e != ESP_OK) return e;
    memmove(buf, buf + 1, n);
    return ESP_OK;
}

static esp_err_t imu_init(spi_device_handle_t dev){
    esp_err_t e;
    if ((e = reg_write(dev, REG_PWR_MGMT_1, 0x01)) != ESP_OK) return e;
    vTaskDelay(pdMS_TO_TICKS(50));
    if ((e = reg_write(dev, REG_SMPLRT_DIV, 0x04)) != ESP_OK) return e; // 200 Hz
    if ((e = reg_write(dev, REG_CONFIG,     0x03)) != ESP_OK) return e; // DLPF ~44 Hz
    if ((e = reg_write(dev, REG_GYRO_CFG,   0x08)) != ESP_OK) return e; // ±500 dps
    if ((e = reg_write(dev, REG_ACC_CFG,    0x08)) != ESP_OK) return e; // ±4 g
    uint8_t who=0;
    if ((e = reg_read(dev, REG_WHO_AM_I, &who)) != ESP_OK) return e;
    if (who != WHO_AM_I_VAL_1 && who != WHO_AM_I_VAL_2) return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    uint64_t t_us;
} sample_t;

static esp_err_t imu_read(spi_device_handle_t dev, sample_t *o){
    uint8_t raw[14];
    esp_err_t e = burst_read(dev, REG_ACC_XOUT_H, raw, sizeof(raw));
    if (e != ESP_OK) return e;
    int16_t ax=be16(raw+0),  ay=be16(raw+2),  az=be16(raw+4);
    int16_t gx=be16(raw+8),  gy=be16(raw+10), gz=be16(raw+12);
    o->ax = ax / 8192.0f;
    o->ay = ay / 8192.0f;
    o->az = az / 8192.0f;
    o->gx = gx / 65.5f * (float)M_PI / 180.0f; // rad/s
    o->gy = gy / 65.5f * (float)M_PI / 180.0f;
    o->gz = gz / 65.5f * (float)M_PI / 180.0f;
    o->t_us = esp_timer_get_time();
    return ESP_OK;
}

// ==== Stick figure math ====
typedef struct {
    float yaw, pitch, roll;
    uint64_t t_prev;
    bool init;
} orient_t;

typedef struct { float x,y,z; } vec3;

static vec3 vec3_add(vec3 a, vec3 b){ return (vec3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static vec3 vec3_scale(vec3 a, float s){ return (vec3){a.x*s, a.y*s, a.z*s}; }

static void basis_from_euler(float yaw, float pitch, float roll, vec3 *right, vec3 *up, vec3 *fwd){
    float cy = cosf(yaw),  sy = sinf(yaw);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cr = cosf(roll), sr = sinf(roll);
    // Rz * Ry * Rx
    if (right)  *right  = (vec3){ cp*cy,  cp*sy,  -sp };
    if (up)     *up     = (vec3){ cy*sr*sp - cr*sy,  cr*cy + sr*sp*sy,  cp*sr };
    if (fwd)    *fwd    = (vec3){ cr*cy*sp + sr*sy, -cy*sr + cr*sp*sy,  cr*cp };
}

static void fuse(sample_t *s, orient_t *o){
    float pitch_acc = atan2f(-s->ax, sqrtf(s->ay*s->ay + s->az*s->az));
    float roll_acc  = atan2f(s->ay, s->az);

    if (!o->init){
        o->pitch = pitch_acc;
        o->roll  = roll_acc;
        o->yaw   = 0.0f;
        o->t_prev = s->t_us;
        o->init = true;
        return;
    }
    float dt = (float)(s->t_us - o->t_prev) / 1e6f;
    if (dt < 0) dt = 0;
    if (dt > 0.05f) dt = 0.05f;
    o->t_prev = s->t_us;

    o->yaw   += s->gz * dt;
    o->pitch += s->gy * dt;
    o->roll  += s->gx * dt;

    const float alpha = 0.02f; // accel blend
    o->pitch = (1.0f - alpha) * o->pitch + alpha * pitch_acc;
    o->roll  = (1.0f - alpha) * o->roll  + alpha * roll_acc;
}

// ==== Drawing helpers ====
static uint8_t g_fb[PANEL_W * PANEL_H / 8];
static inline void fb_clear(void){ memset(g_fb, 0, sizeof(g_fb)); }
static inline void pset(int x,int y){
    if ((unsigned)x >= PANEL_W || (unsigned)y >= PANEL_H) return;
    int idx = y * PANEL_W + x;
    g_fb[idx >> 3] |= (uint8_t)(1u << (7 - (idx & 7)));
}

static void line(int x0,int y0,int x1,int y1){
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (1){
        pset(x0,y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy){ err += dy; x0 += sx; }
        if (e2 <= dx){ err += dx; y0 += sy; }
    }
}

static void project(vec3 p, int cx, int cy, float scale, int *ox, int *oy){
    *ox = cx + (int)(p.x * scale);
    *oy = cy - (int)(p.y * scale); // y up
}

void stick_run(void)
{
    ESP_LOGI(TAG, "starting stick figure");
    ESP_ERROR_CHECK(hw_spi2_init_once());
    hw_gpio_init();
    oled_init();
    oled_clear();

    spi_device_handle_t imu[3] = {0};
    gpio_num_t cs_pins[3] = { PIN_CS_IMU1, PIN_CS_IMU2, PIN_CS_IMU3 };
    for (int i = 0; i < 3; ++i){
        ESP_ERROR_CHECK(hw_spi2_add_mpu(cs_pins[i], &imu[i]));
        ESP_ERROR_CHECK(imu_init(imu[i]));
    }

    orient_t o[3] = {0};

    const float torso_len = 22.0f;
    const float arm_len   = 20.0f;
    const float shoulder_half = 10.0f;
    const int cx = PANEL_W / 2;
    const int cy = PANEL_H / 2 + 8;
    const float scale = 18.0f; // world units to px

    while (1){
        hw_button_id_t b = HW_BTN_NONE;
        if (hw_buttons_read(&b) == ESP_OK && b == HW_BTN_A) break;

        sample_t s[3];
        for (int i = 0; i < 3; ++i){
            if (imu_read(imu[i], &s[i]) == ESP_OK){
                fuse(&s[i], &o[i]);
            }
        }

        vec3 r_c, u_c, f_c;
        basis_from_euler(o[0].yaw, o[0].pitch, o[0].roll, &r_c, &u_c, &f_c);
        vec3 r_l, u_l, f_l;
        basis_from_euler(o[1].yaw, o[1].pitch, o[1].roll, &r_l, &u_l, &f_l);
        vec3 r_r, u_r, f_r;
        basis_from_euler(o[2].yaw, o[2].pitch, o[2].roll, &r_r, &u_r, &f_r);

        // skeleton in world space (meters -> arbitrary units)
        vec3 hip = {0, 0, 0};
        vec3 chest = vec3_add(hip, vec3_scale(u_c, torso_len / scale));
        vec3 shoulder = chest;
        vec3 shoulder_l = vec3_add(shoulder, vec3_scale(r_c, -shoulder_half / scale));
        vec3 shoulder_r = vec3_add(shoulder, vec3_scale(r_c,  shoulder_half / scale));

        vec3 wrist_l = vec3_add(shoulder_l, vec3_scale(f_l, arm_len / scale));
        vec3 wrist_r = vec3_add(shoulder_r, vec3_scale(f_r, arm_len / scale));

        int hx, hy, cxp, cyp, slx, sly, srx, sry, wlx, wly, wrx, wry;
        fb_clear();
        project(hip, cx, cy, scale, &hx, &hy);
        project(chest, cx, cy, scale, &cxp, &cyp);
        project(shoulder_l, cx, cy, scale, &slx, &sly);
        project(shoulder_r, cx, cy, scale, &srx, &sry);
        project(wrist_l, cx, cy, scale, &wlx, &wly);
        project(wrist_r, cx, cy, scale, &wrx, &wry);

        line(hx, hy, cxp, cyp);               // torso
        line(slx, sly, wlx, wly);             // left arm
        line(srx, sry, wrx, wry);             // right arm
        // small head
        int headx, heady;
        vec3 head = vec3_add(chest, vec3_scale(u_c, 8.0f / scale));
        project(head, cx, cy, scale, &headx, &heady);
        pset(headx, heady);

        oled_blit_full(g_fb);
        vTaskDelay(pdMS_TO_TICKS(16));
    }

    ESP_LOGI(TAG, "stick figure exit");
}
