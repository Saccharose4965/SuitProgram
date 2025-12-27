// 3-IMU stick figure visualizer (torso + two arms) on the OLED.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Blocks and renders at ~60 fps until the task is deleted or button A is pressed.
void stick_run(void);

#ifdef __cplusplus
}
#endif
