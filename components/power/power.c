#include "power.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hw.h"
#include "system_state.h"

#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

// ===== Configurable constants (tune to your wiring) =====
// Divider ratio: Vpack / Vadcin. Replace with your measured ratio (menuconfig or fallback).
static float k_div_ratio = 6.0f; // placeholder: e.g., 220k:47k ≈ 5.7

// Expected per-cell voltage span (3S pack). Adjust for your chemistry/cutoffs (menuconfig or fallback).
static float k_cell_v_empty = 3.30f;
static float k_cell_v_full  = 4.15f;

// Low-pass smoothing for percent
#ifndef POWER_PCT_ALPHA
#define POWER_PCT_ALPHA 0.2f
#endif

static const char *TAG = "power";
static TaskHandle_t s_task = NULL;

static void power_init_constants(void){
    static bool inited = false;
    if (inited) return;
    inited = true;
#ifdef CONFIG_POWER_PACK_DIVIDER_RATIO
    const char *start_pack = CONFIG_POWER_PACK_DIVIDER_RATIO;
    char *end_pack = NULL;
    float v_pack = strtof(start_pack, &end_pack);
    if (end_pack && end_pack != start_pack && v_pack > 0.1f) {
        k_div_ratio = v_pack;
    }
#endif
#ifdef CONFIG_POWER_CELL_V_EMPTY
    const char *start_empty = CONFIG_POWER_CELL_V_EMPTY;
    char *end_empty = NULL;
    float v_empty = strtof(start_empty, &end_empty);
    if (end_empty && end_empty != start_empty && v_empty > 0.1f) {
        k_cell_v_empty = v_empty;
    }
#endif
#ifdef CONFIG_POWER_CELL_V_FULL
    const char *start_full = CONFIG_POWER_CELL_V_FULL;
    char *end_full = NULL;
    float v_full = strtof(start_full, &end_full);
    if (end_full && end_full != start_full && v_full > k_cell_v_empty) {
        k_cell_v_full = v_full;
    }
#endif
}

static float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static uint8_t voltage_to_pct(float vpack){
    // Convert total pack voltage to percent using per-cell span.
    float cell_v = vpack / 3.0f;
    float span = k_cell_v_full - k_cell_v_empty;
    if (span <= 0.05f) span = 0.05f;
    float pct = (cell_v - k_cell_v_empty) / span;
    pct = clampf(pct, 0.0f, 1.0f);
    return (uint8_t)(pct * 100.0f + 0.5f);
}

static void power_task(void *arg){
    (void)arg;
    float filtered_pct = 0.0f;
    power_init_constants();
    hw_adc1_init_default();

    while (1){
        int mv = 0;
        esp_err_t err = hw_adc1_read_mv(HW_HALL_B_ADC_CH, &mv); // Hall B repurposed as pack divider
        if (err == ESP_OK){
            float vpack = ((float)mv / 1000.0f) * k_div_ratio;
            uint8_t pct = voltage_to_pct(vpack);
            filtered_pct = POWER_PCT_ALPHA * (float)pct + (1.0f - POWER_PCT_ALPHA) * filtered_pct;
            uint8_t smoothed = (uint8_t)clampf(filtered_pct, 0.0f, 100.0f);
            // Mirror into all three slots until per-pack sensing is separated.
            system_state_set_battery(0, smoothed);
            system_state_set_battery(1, smoothed);
            system_state_set_battery(2, smoothed);
        } else {
            ESP_LOGW(TAG, "adc read failed: %d", err);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t power_monitor_start(void){
    if (s_task) return ESP_OK;
    BaseType_t ok = xTaskCreatePinnedToCore(power_task, "power", 2048, NULL, 4, &s_task, tskNO_AFFINITY);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
