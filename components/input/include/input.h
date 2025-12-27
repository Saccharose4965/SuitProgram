#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    INPUT_BTN_NONE = 0,
    INPUT_BTN_A,
    INPUT_BTN_B,
    INPUT_BTN_C,
    INPUT_BTN_D,
    INPUT_BTN_TOP_COMBO, // long-press escape to menu
    INPUT_BTN_FAST_FALL, // B+C combo (≈2.2 V) for drop
} input_button_t;

typedef enum {
    INPUT_EVENT_PRESS = 0,
    INPUT_EVENT_RELEASE,
    INPUT_EVENT_LONG_PRESS,
} input_event_type_t;

typedef struct {
    input_button_t     button;
    input_event_type_t type;
    TickType_t         at_ticks;
} input_event_t;

typedef struct {
    uint32_t long_press_ms; // default ~800 ms
    int combo_mv;           // optional ADC target for top-button combo (0 disables)
    int combo_tol_mv;       // tolerance around combo_mv
    uint32_t combo_verify_ms; // how long the combo voltage must stay before emitting
} input_config_t;

// Configure thresholds/long-press timings (safe to call with NULL for defaults).
void input_init(const input_config_t *cfg);

// Poll the ladder and return one event at a time (PRESS/RELEASE/LONG_PRESS).
// Returns true when an event is available and stored into out_event.
bool input_poll(input_event_t *out_event, TickType_t now_ticks);

// Sample the ladder and expose the decoded logical button + raw mV (for HUD/debug).
bool input_sample(input_button_t *out_btn, int *out_mv);

#ifdef __cplusplus
}
#endif
