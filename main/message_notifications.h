#pragma once
#include <stdint.h>

// Arrival badge for this boot; files remain on SD across reboot.
uint32_t message_arrival_snapshot(void);
unsigned message_unseen_count(void);
void message_mark_seen(uint32_t snapshot);
