#pragma once

#include <stdbool.h>

typedef struct {
    float volume; // 0.0 .. 2.0
    bool  muted;
} app_settings_t;

void app_settings_init(void);
const app_settings_t *app_settings_get(void);
void app_settings_set_volume(float volume, bool muted, bool persist);
