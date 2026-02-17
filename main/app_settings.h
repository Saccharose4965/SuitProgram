#pragma once

#include <stdbool.h>

typedef struct {
    float speaker_volume; // 0.0 .. 2.0
    float bt_volume;      // 0.0 .. 2.0
    bool  speaker_muted;
    bool  bt_muted;
} app_settings_t;

void app_settings_init(void);
const app_settings_t *app_settings_get(void);
void app_settings_set_audio(float speaker_volume,
                            bool speaker_muted,
                            float bt_volume,
                            bool bt_muted,
                            bool persist);
