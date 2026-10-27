#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void performance_monitor_init(void);
void performance_monitor_on_frame(const char *app_id,
                             bool app_external,
                             uint32_t frame_budget_us,
                             uint32_t tick_us,
                             uint32_t draw_us,
                             uint32_t work_us);

#ifdef __cplusplus
}
#endif
