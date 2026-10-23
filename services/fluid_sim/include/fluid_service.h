#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLUID_SIM_WIDTH 64
#define FLUID_SIM_HEIGHT 32
#define FLUID_SIM_PIXELS (FLUID_SIM_WIDTH * FLUID_SIM_HEIGHT)

void fluid_service_start(void);
void fluid_service_stop(void);
void fluid_service_tick_fallback(void);
void fluid_service_copy_mask(uint8_t out[FLUID_SIM_PIXELS]);

#ifdef __cplusplus
}
#endif
