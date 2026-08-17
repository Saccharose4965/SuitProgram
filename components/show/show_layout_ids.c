#include "show_layout_ids.h"

#include <string.h>

typedef struct {
    uint32_t id;
    const char *name;
} show_name_map_t;

static const show_name_map_t kSectionNames[] = {
    { SHOW_SECTION_FRONT_LEFT_TOP, "front_left_top" },
    { SHOW_SECTION_FRONT_LEFT_RING, "front_left_ring" },
    { SHOW_SECTION_FRONT_LEFT_RIB, "front_left_rib" },
    { SHOW_SECTION_FRONT_LEFT_ABS, "front_left_abs" },
    { SHOW_SECTION_FRONT_LEFT_BELT, "front_left_belt" },
    { SHOW_SECTION_BACK_LEFT_BELT, "back_left_belt" },
    { SHOW_SECTION_BACK_LEFT_VERTEBRA, "back_left_vertebra" },
    { SHOW_SECTION_BACK_LEFT_RIB, "back_left_rib" },
    { SHOW_SECTION_BACK_LEFT_TOP, "back_left_top" },
    { SHOW_SECTION_LEFT_UPPER_ARM, "left_upper_arm" },
    { SHOW_SECTION_LEFT_FOREARM, "left_forearm" },
    { SHOW_SECTION_FRONT_RIGHT_TOP, "front_right_top" },
    { SHOW_SECTION_FRONT_RIGHT_RIB, "front_right_rib" },
    { SHOW_SECTION_FRONT_RIGHT_ABS, "front_right_abs" },
    { SHOW_SECTION_FRONT_RIGHT_BELT, "front_right_belt" },
    { SHOW_SECTION_BACK_RIGHT_BELT, "back_right_belt" },
    { SHOW_SECTION_BACK_RIGHT_VERTEBRA, "back_right_vertebra" },
    { SHOW_SECTION_BACK_RIGHT_RIB, "back_right_rib" },
    { SHOW_SECTION_BACK_RIGHT_TOP, "back_right_top" },
    { SHOW_SECTION_RIGHT_UPPER_ARM, "right_upper_arm" },
    { SHOW_SECTION_RIGHT_FOREARM, "right_forearm" },
    { SHOW_SECTION_LEFT_THIGH_FRONT, "left_thigh_front" },
    { SHOW_SECTION_LEFT_SHIN_F_IN, "left_shin_f_in" },
    { SHOW_SECTION_LEFT_SHIN_F_OUT, "left_shin_f_out" },
    { SHOW_SECTION_LEFT_THIGH_BACK, "left_thigh_back" },
    { SHOW_SECTION_LEFT_SHIN_B_IN, "left_shin_b_in" },
    { SHOW_SECTION_LEFT_SHIN_B_OUT, "left_shin_b_out" },
    { SHOW_SECTION_RIGHT_THIGH_FRONT, "right_thigh_front" },
    { SHOW_SECTION_RIGHT_SHIN_F_IN, "right_shin_f_in" },
    { SHOW_SECTION_RIGHT_SHIN_F_OUT, "right_shin_f_out" },
    { SHOW_SECTION_RIGHT_THIGH_BACK, "right_thigh_back" },
    { SHOW_SECTION_RIGHT_SHIN_B_IN, "right_shin_b_in" },
    { SHOW_SECTION_RIGHT_SHIN_B_OUT, "right_shin_b_out" },
};

static const show_name_map_t kGroupNames[] = {
    { SHOW_GROUP_ALL, "all" },
    { SHOW_GROUP_FRONT_ALL, "front_all" },
    { SHOW_GROUP_BACK_ALL, "back_all" },
    { SHOW_GROUP_LEFT_ALL, "left_all" },
    { SHOW_GROUP_RIGHT_ALL, "right_all" },
    { SHOW_GROUP_TORSO_ALL, "torso_all" },
    { SHOW_GROUP_ARMS_ALL, "arms_all" },
    { SHOW_GROUP_LEFT_ARM_ALL, "left_arm_all" },
    { SHOW_GROUP_RIGHT_ARM_ALL, "right_arm_all" },
    { SHOW_GROUP_RING, "ring" },
    { SHOW_GROUP_BELT_ALL, "belt_all" },
    { SHOW_GROUP_SPINE, "spine" },
    { SHOW_GROUP_LEGS_ALL, "legs_all" },
    { SHOW_GROUP_LEFT_LEG_ALL, "left_leg_all" },
    { SHOW_GROUP_RIGHT_LEG_ALL, "right_leg_all" },
    { SHOW_GROUP_LEFT_THIGH_ALL, "left_thigh_all" },
    { SHOW_GROUP_LEFT_SHIN_ALL, "left_shin_all" },
    { SHOW_GROUP_RIGHT_THIGH_ALL, "right_thigh_all" },
    { SHOW_GROUP_RIGHT_SHIN_ALL, "right_shin_all" },
};

static const char *lookup_name(const show_name_map_t *map, size_t count, uint32_t id)
{
    for (size_t i = 0; i < count; ++i) {
        if (map[i].id == id) {
            return map[i].name;
        }
    }
    return NULL;
}

static bool lookup_id(const show_name_map_t *map, size_t count, const char *name, uint32_t *out)
{
    if (!name || !out) return false;
    for (size_t i = 0; i < count; ++i) {
        if (strcmp(map[i].name, name) == 0) {
            *out = map[i].id;
            return true;
        }
    }
    return false;
}

const char *show_section_name(show_section_id_t id)
{
    return lookup_name(kSectionNames, sizeof(kSectionNames) / sizeof(kSectionNames[0]), id);
}

const char *show_group_name(show_group_id_t id)
{
    return lookup_name(kGroupNames, sizeof(kGroupNames) / sizeof(kGroupNames[0]), id);
}

bool show_section_id_from_name(const char *name, show_section_id_t *out)
{
    uint32_t id = 0;
    if (!lookup_id(kSectionNames, sizeof(kSectionNames) / sizeof(kSectionNames[0]), name, &id)) {
        return false;
    }
    if (out) *out = (show_section_id_t)id;
    return true;
}

bool show_group_id_from_name(const char *name, show_group_id_t *out)
{
    uint32_t id = 0;
    if (!lookup_id(kGroupNames, sizeof(kGroupNames) / sizeof(kGroupNames[0]), name, &id)) {
        return false;
    }
    if (out) *out = (show_group_id_t)id;
    return true;
}
