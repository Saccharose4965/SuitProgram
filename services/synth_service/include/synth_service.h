#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool calibrated;
    bool muted;
    bool bluetooth;
    float hz;
    float gain;
    esp_err_t error;
} synth_status_t;

void synth_service_open(void);
void synth_service_close(void);
void synth_service_calibrate(void);
void synth_service_toggle_mute(void);
void synth_service_tick(void);
synth_status_t synth_service_status(void);
