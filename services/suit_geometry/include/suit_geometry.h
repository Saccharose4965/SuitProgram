#pragma once
#include "show_geometry.h"

// Canonical posed suit coordinates in cm: +X viewer-right (wearer's left),
// +Y down, +Z forward. Waist centreline is (0,33.5,0).
// Layout storage remains an unposed authoring/wiring description.
bool suit_geometry_get(const led_layout_config_t *cfg, int index, led_point_t *out);
// Derive ring centre/radius from canonical LED positions, not flat arc metadata.
bool suit_geometry_ring(const led_layout_config_t *cfg, led_point_t *center, float *radius);
