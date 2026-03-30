from __future__ import annotations

SECTION_IDS = {
    "front_left_top": 1,
    "front_left_ring": 2,
    "front_left_rib": 3,
    "front_left_abs": 4,
    "front_left_belt": 5,
    "back_left_belt": 6,
    "back_left_vertebra": 7,
    "back_left_rib": 8,
    "back_left_top": 9,
    "left_upper_arm": 10,
    "left_forearm": 11,
    "front_right_top": 12,
    "front_right_rib": 13,
    "front_right_abs": 14,
    "front_right_belt": 15,
    "back_right_belt": 16,
    "back_right_vertebra": 17,
    "back_right_rib": 18,
    "back_right_top": 19,
    "right_upper_arm": 20,
    "right_forearm": 21,
    "left_upper_leg": 22,
    "left_lower_leg": 23,
    "right_upper_leg": 24,
    "right_lower_leg": 25,
}

GROUP_IDS = {
    "all": 1,
    "front_all": 2,
    "back_all": 3,
    "left_all": 4,
    "right_all": 5,
    "torso_all": 6,
    "arms_all": 7,
    "left_arm_all": 8,
    "right_arm_all": 9,
    "ring": 10,
    "belt_all": 11,
    "spine": 12,
    "legs_all": 13,
    "left_leg_all": 14,
    "right_leg_all": 15,
}

ROLE_NAMES = ("A", "B", "C")

GROUP_SECTIONS = {
    "all": tuple(SECTION_IDS.keys()),
    "front_all": (
        "front_left_top",
        "front_left_ring",
        "front_left_rib",
        "front_left_abs",
        "front_left_belt",
        "front_right_top",
        "front_right_rib",
        "front_right_abs",
        "front_right_belt",
    ),
    "back_all": (
        "back_left_belt",
        "back_left_vertebra",
        "back_left_rib",
        "back_left_top",
        "back_right_belt",
        "back_right_vertebra",
        "back_right_rib",
        "back_right_top",
    ),
    "left_all": (
        "front_left_top",
        "front_left_ring",
        "front_left_rib",
        "front_left_abs",
        "front_left_belt",
        "back_left_belt",
        "back_left_vertebra",
        "back_left_rib",
        "back_left_top",
        "left_upper_arm",
        "left_forearm",
        "left_upper_leg",
        "left_lower_leg",
    ),
    "right_all": (
        "front_right_top",
        "front_right_rib",
        "front_right_abs",
        "front_right_belt",
        "back_right_belt",
        "back_right_vertebra",
        "back_right_rib",
        "back_right_top",
        "right_upper_arm",
        "right_forearm",
        "right_upper_leg",
        "right_lower_leg",
    ),
    "torso_all": (
        "front_left_top",
        "front_left_ring",
        "front_left_rib",
        "front_left_abs",
        "front_left_belt",
        "back_left_belt",
        "back_left_vertebra",
        "back_left_rib",
        "back_left_top",
        "front_right_top",
        "front_right_rib",
        "front_right_abs",
        "front_right_belt",
        "back_right_belt",
        "back_right_vertebra",
        "back_right_rib",
        "back_right_top",
    ),
    "arms_all": (
        "left_upper_arm",
        "left_forearm",
        "right_upper_arm",
        "right_forearm",
    ),
    "left_arm_all": (
        "left_upper_arm",
        "left_forearm",
    ),
    "right_arm_all": (
        "right_upper_arm",
        "right_forearm",
    ),
    "ring": ("front_left_ring",),
    "belt_all": (
        "front_left_belt",
        "back_left_belt",
        "front_right_belt",
        "back_right_belt",
    ),
    "spine": (
        "back_left_vertebra",
        "back_right_vertebra",
    ),
    "legs_all": (
        "left_upper_leg",
        "left_lower_leg",
        "right_upper_leg",
        "right_lower_leg",
    ),
    "left_leg_all": (
        "left_upper_leg",
        "left_lower_leg",
    ),
    "right_leg_all": (
        "right_upper_leg",
        "right_lower_leg",
    ),
}


def section_id(name: str) -> int:
    return SECTION_IDS[name]


def group_id(name: str) -> int:
    return GROUP_IDS[name]


def group_sections(name: str) -> tuple[str, ...]:
    return GROUP_SECTIONS[name]
