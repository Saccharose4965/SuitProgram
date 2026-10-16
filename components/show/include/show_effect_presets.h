#pragma once
#include "show_package.h"

/* Named recipes use exactly the same compiled programs as timeline shows.
 * Programs are immutable, borrowed data; do not pass them to program_free(). */
typedef struct {
    const char *name;
    const show_package_program_t *program;
} show_effect_preset_t;

/* Intentionally empty until the standalone costume presets are designed. */
const show_effect_preset_t *show_effect_presets(size_t *count);
