#include "system_state.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static system_state_t g_state = {
    .current_led_mode = 0,
    .led_mode_name = "idle",
    .fft_running = false,
    .fft_bpm_centi = 0,
    .master_ctrl = SYS_MASTER_CTRL_OFF,
    .sync_link_active = false,
    .battery_pct = {0, 0, 0},
    .connection = SYS_CONN_DISCONNECTED,
    .time_valid = false,
    .hours = 0,
    .minutes = 0,
};

static StaticSemaphore_t s_lock_buf;
static SemaphoreHandle_t s_lock = NULL;

static void ensure_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }
}

void system_state_init(void)
{
    ensure_init();
}

static inline void lock(void)
{
    ensure_init();
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
}

static inline void unlock(void)
{
    if (s_lock) {
        xSemaphoreGive(s_lock);
    }
}

system_state_t system_state_get(void)
{
    system_state_t snap;
    lock();
    snap = g_state;
    unlock();
    return snap;
}

void system_state_set_led_mode(int mode_id, const char *label)
{
    lock();
    g_state.current_led_mode = mode_id;
    if (label && label[0]) {
        strncpy(g_state.led_mode_name, label, sizeof(g_state.led_mode_name) - 1);
        g_state.led_mode_name[sizeof(g_state.led_mode_name) - 1] = '\0';
    }
    unlock();
}

void system_state_set_battery(size_t idx, uint8_t pct)
{
    if (idx >= sizeof(g_state.battery_pct)) return;
    if (pct > 100) pct = 100;
    lock();
    g_state.battery_pct[idx] = pct;
    unlock();
}

void system_state_set_connection(system_connection_t state)
{
    lock();
    g_state.connection = state;
    unlock();
}

void system_state_set_fft_status(bool running, uint16_t bpm_centi)
{
    lock();
    g_state.fft_running = running;
    g_state.fft_bpm_centi = bpm_centi;
    unlock();
}

void system_state_set_master_control(system_master_ctrl_t state, bool link_active)
{
    lock();
    g_state.master_ctrl = state;
    g_state.sync_link_active = link_active;
    unlock();
}

void system_state_set_time(bool valid, uint8_t hours, uint8_t minutes)
{
    lock();
    g_state.time_valid = valid;
    if (valid) {
        g_state.hours = hours % 24;
        g_state.minutes = minutes % 60;
    }
    unlock();
}

const char *system_state_connection_str(system_connection_t state)
{
    switch (state) {
        case SYS_CONN_DISCONNECTED: return "disc";
        case SYS_CONN_CONNECTING:   return "link";
        case SYS_CONN_CONNECTED:    return "on";
        default:                    return "?";
    }
}
