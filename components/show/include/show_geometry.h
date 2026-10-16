#pragma once

#include "led_layout.h"
#include "show_layout_ids.h"

/* Pose the flat SD layout for v1.2 shows; called only when loading a show.
 * Matches ShowEditor/show_editor/layout/spatial_display.py:display_point_3d. */
led_point_t show_geometry_pose(show_section_id_t section, led_point_t point, float section_u);
led_point_t show_geometry_noise_plane(show_section_id_t section, led_point_t point, float section_u);
