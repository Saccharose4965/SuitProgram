#pragma once

#include "led_layout.h"
#include "show_format.h"

/* Same stage convention as ShowEditor/show_editor/formation.py: +Z is front,
 * Y is vertical; spacing is the distance between neighbouring suit origins.
 * Fixed coefficients avoid trigonometry in the render loop. */
static inline led_point_t show_formation_point(led_point_t p, uint16_t formation,
                                               uint8_t role, float spacing)
{
    if (formation == SHOW_FORMATION_CIRCLE) {
        const float cos_yaw[3] = {0.5f, -1.0f, 0.5f};
        const float sin_yaw[3] = {0.8660254038f, 0.0f, -0.8660254038f};
        float radius = spacing * 0.5773502692f;
        float ox = ((float)role - 1.0f) * spacing * 0.5f;
        float oz = role == SHOW_ROLE_B ? radius : -radius * 0.5f;
        return (led_point_t){cos_yaw[role] * p.x + sin_yaw[role] * p.z + ox,
                             p.y, -sin_yaw[role] * p.x + cos_yaw[role] * p.z + oz};
    }
    p.x += ((float)role - 1.0f) * spacing;
    return p;
}

/* Move an effect into costume coordinates once, instead of transforming every
 * LED for each orb. Yaw is orthonormal, so distances are preserved. */
static inline led_point_t show_formation_local(led_point_t p, uint16_t formation,
                                              uint8_t role, float spacing)
{
    led_point_t origin = show_formation_point((led_point_t){0}, formation, role, spacing);
    float x = p.x - origin.x, z = p.z - origin.z;
    if (formation == SHOW_FORMATION_CIRCLE) {
        const float c[3] = {0.5f, -1.0f, 0.5f};
        const float s[3] = {0.8660254038f, 0.0f, -0.8660254038f};
        return (led_point_t){c[role]*x-s[role]*z, p.y, s[role]*x+c[role]*z};
    }
    return (led_point_t){x, p.y, z};
}
