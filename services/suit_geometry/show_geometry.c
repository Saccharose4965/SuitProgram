#include "show_geometry.h"

#include <math.h>

static void upper_arm_profile(float u, float *z, float *y)
{
    const float depth = 4.3f, top = -2.5f, bottom = 12.2f;
    float step = (2.0f * depth) / (1.0f + sqrtf(2.0f)) / sqrtf(2.0f);
    float drop = fmaxf(0.0f, bottom - top - step);
    const float profile[6][2] = {
        {-depth, top}, {-depth, top + drop}, {-depth + step, bottom},
        {depth - step, bottom}, {depth, top + drop}, {depth, top},
    };
    float lengths[5], total = 0;
    for (int i = 0; i < 5; ++i) {
        lengths[i] = hypotf(profile[i+1][0] - profile[i][0], profile[i+1][1] - profile[i][1]);
        total += lengths[i];
    }
    float target = fmaxf(0, fminf(1, u)) * total;
    for (int i = 0; i < 5; ++i) {
        if (target <= lengths[i] || i == 4) {
            float t = lengths[i] > 1e-6f ? target / lengths[i] : 0;
            *z = profile[i][0] + (profile[i+1][0] - profile[i][0]) * t;
            *y = profile[i][1] + (profile[i+1][1] - profile[i][1]) * t;
            return;
        }
        target -= lengths[i];
    }
}

led_point_t show_geometry_noise_plane(show_section_id_t section, led_point_t raw, float u)
{
    led_point_t p=show_geometry_pose(section,raw,u);
    bool torso=(section>=SHOW_SECTION_FRONT_LEFT_TOP && section<=SHOW_SECTION_FRONT_LEFT_BELT) ||
               (section>=SHOW_SECTION_FRONT_RIGHT_TOP && section<=SHOW_SECTION_FRONT_RIGHT_BELT) ||
               (section>=SHOW_SECTION_BACK_LEFT_BELT && section<=SHOW_SECTION_BACK_LEFT_TOP) ||
               (section>=SHOW_SECTION_BACK_RIGHT_BELT && section<=SHOW_SECTION_BACK_RIGHT_TOP);
    if (torso) p.x=raw.x;
    else if (section==SHOW_SECTION_LEFT_UPPER_ARM || section==SHOW_SECTION_RIGHT_UPPER_ARM)
        p.x=(section==SHOW_SECTION_LEFT_UPPER_ARM ? 1 : -1)*(38+fabsf(p.z)*.45f);
    else if (section==SHOW_SECTION_LEFT_FOREARM || section==SHOW_SECTION_RIGHT_FOREARM)
        p.x=(section==SHOW_SECTION_LEFT_FOREARM ? 42 : -42)+fabsf(p.z)*.25f;
    p.z=0;
    return p;
}

led_point_t show_geometry_pose(show_section_id_t section, led_point_t p, float u)
{
    bool front = (section >= SHOW_SECTION_FRONT_LEFT_TOP && section <= SHOW_SECTION_FRONT_LEFT_BELT) ||
                 (section >= SHOW_SECTION_FRONT_RIGHT_TOP && section <= SHOW_SECTION_FRONT_RIGHT_BELT);
    bool back = (section >= SHOW_SECTION_BACK_LEFT_BELT && section <= SHOW_SECTION_BACK_LEFT_TOP) ||
                (section >= SHOW_SECTION_BACK_RIGHT_BELT && section <= SHOW_SECTION_BACK_RIGHT_TOP);
    if (front || back) {
        if (back) p.y = 33.5f - p.y;
        else if (section == SHOW_SECTION_FRONT_LEFT_BELT || section == SHOW_SECTION_FRONT_RIGHT_BELT) p.y = 33.5f;
        float sign = p.x >= 0 ? 1.0f : -1.0f;
        float angle = sign * fminf(1, fabsf(p.x) / 26.0f) * 1.5707963268f;
        if (back) angle = 3.1415926536f - angle;
        p.x = 18.0f * sinf(angle);
        p.z = 12.0f * cosf(angle);
        if (section == SHOW_SECTION_FRONT_LEFT_TOP || section == SHOW_SECTION_FRONT_RIGHT_TOP ||
            section == SHOW_SECTION_BACK_LEFT_TOP || section == SHOW_SECTION_BACK_RIGHT_TOP) {
            float amount = fmaxf(0, fminf(1, (1.0f - p.y) / 16.0f));
            amount *= amount;
            p.x *= 1.0f - (back ? 0.06f : 0.16f) * amount;
            p.z *= 1.0f - (back ? 0.30f : 0.68f) * amount;
            p.y -= (back ? 1.2f : 1.8f) * amount;
        }
        // Preserve the front-view silhouette, but use two flat torso panels.
        // Their separation and formation yaw remain part of the stage geometry.
        p.z = back ? -12.0f : 12.0f;
    }
    if (section == SHOW_SECTION_LEFT_UPPER_ARM || section == SHOW_SECTION_RIGHT_UPPER_ARM) {
        upper_arm_profile(u, &p.z, &p.y);
        float sign = section == SHOW_SECTION_LEFT_UPPER_ARM ? 1 : -1;
        p.x = sign * (25.5f + 1.0f - fabsf(u * 2.0f - 1.0f));
    } else if (section == SHOW_SECTION_LEFT_FOREARM || section == SHOW_SECTION_RIGHT_FOREARM) {
        p.x = section == SHOW_SECTION_LEFT_FOREARM ? 28 : -28;
        p.y += 10.5f;
        p.z += 3.0f * u;
    }
    return p;
}
