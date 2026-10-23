#pragma once

#include "led_layout.h"
#include "show_layout_ids.h"

/* Shared canonical pose from flat SD authoring geometry. Retained names/IDs
 * avoid changing show package encoding. Used when building geometry caches.
 * Matches ShowEditor/show_editor/layout/spatial_display.py:display_point_3d.
 * Noise coordinates are an explicit flattened texture mapping, not LED space. */
led_point_t show_geometry_pose(show_section_id_t section, led_point_t point, float section_u);
led_point_t show_geometry_noise_plane(show_section_id_t section, led_point_t point, float section_u);
