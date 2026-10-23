#pragma once

#include <stdbool.h>

// Single owner: audio playback worker. Diagnostics never issue BT commands.
void playback_diagnostics_begin(void);
void playback_diagnostics_sample(bool stalled);
void playback_diagnostics_end(void);
