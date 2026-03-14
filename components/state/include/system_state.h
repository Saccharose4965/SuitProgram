#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_CONN_DISCONNECTED = 0,
    SYS_CONN_CONNECTING,
    SYS_CONN_CONNECTED,
} system_connection_t;

typedef enum {
    SYS_MASTER_CTRL_OFF = 0,
    SYS_MASTER_CTRL_SLAVE,
    SYS_MASTER_CTRL_MASTER,
} system_master_ctrl_t;

typedef struct {
    int  current_led_mode;      // opaque mode id, set by LED service
    char led_mode_name[16];     // short human-friendly label for HUD
    bool fft_running;           // FFT service active
    uint16_t fft_bpm_centi;     // effective FFT BPM x100 when available
    system_master_ctrl_t master_ctrl;
    bool sync_link_active;      // master/slave sync link currently alive
    uint8_t battery_pct[3];     // 0–100 for costume batteries
    system_connection_t connection;
    bool time_valid;
    uint8_t hours;
    uint8_t minutes;
} system_state_t;

// Initialize SystemState storage (idempotent).
void system_state_init(void);

// Thread-safe snapshot of the current state.
system_state_t system_state_get(void);

// LED mode is owned by the LED service.
void system_state_set_led_mode(int mode_id, const char *label);

// Battery slots are owned by power/telemetry sensing.
void system_state_set_battery(size_t idx, uint8_t pct);

// Connection status is owned by telemetry/ESP-NOW.
void system_state_set_connection(system_connection_t state);

// FFT HUD status is owned by the shell/service layer.
void system_state_set_fft_status(bool running, uint16_t bpm_centi);

// Master-control HUD status is owned by the shell/service layer.
void system_state_set_master_control(system_master_ctrl_t state, bool link_active);

// Time validity is owned by GPS.
void system_state_set_time(bool valid, uint8_t hours, uint8_t minutes);

// Helper for HUD rendering.
const char *system_state_connection_str(system_connection_t state);

#ifdef __cplusplus
}
#endif
