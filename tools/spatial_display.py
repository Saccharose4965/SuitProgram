from __future__ import annotations

import math

from tools.layout_model import LayoutSection


BACK_Y_FLIP_PIVOT = 33.5
FRONT_BELT_Y = 33.5
FLAT_FRONT_X_OFFSET = 0.0
FLAT_BACK_X_OFFSET = 0.0
FLAT_FRONT_Y_OFFSET = 0.0
FLAT_BACK_Y_OFFSET = 60.0
FLAT_ARM_X_OFFSET = 38.0
FLAT_FOREARM_X_OFFSET = 4.0
FLAT_ARM_Z_SCALE = 0.45
FLAT_FOREARM_Z_SCALE = 0.25
FLAT_PATH_BREAK_DISTANCE = 24.0
TORSO_WRAP_HALF_WIDTH = 26.0
TORSO_RADIUS_X = 18.0
TORSO_RADIUS_Z = 12.0
UPPER_ARM_TOP_Y = -2.5
UPPER_ARM_BOTTOM_Y = 12.2
UPPER_ARM_DEPTH = 4.3
UPPER_ARM_OUTWARD_X = 25.5
UPPER_ARM_MID_BULGE = 1.0
SHOULDER_TOP_BLEND_START_Y = 1.0
SHOULDER_TOP_BLEND_END_Y = -15.0
FRONT_SHOULDER_TOP_Z_PULL = 0.68
FRONT_SHOULDER_TOP_X_PULL = 0.16
FRONT_SHOULDER_TOP_Y_LIFT = 1.8
BACK_SHOULDER_TOP_Z_PULL = 0.30
BACK_SHOULDER_TOP_X_PULL = 0.06
BACK_SHOULDER_TOP_Y_LIFT = 1.2
FOREARM_OUTWARD_X = 28.0
FOREARM_Y_OFFSET = 10.5
FOREARM_FRONT_OFFSET = 3.0
ARC_PREVIEW_STEPS = 96


def display_point(mode: str,
                  section_name: str,
                  point: tuple[float, float, float],
                  section_u: float = 0.5) -> tuple[float, float, float]:
    if mode == "3d":
        return display_point_3d(section_name, point, section_u)
    return display_point_2d(section_name, point, section_u)


def display_point_2d(section_name: str,
                     point: tuple[float, float, float],
                     section_u: float = 0.5) -> tuple[float, float, float]:
    x, _, _ = point
    posed_x, posed_y, posed_z = display_point_3d(section_name, point, section_u)
    if section_name.startswith("front_") or section_name.endswith("ring"):
        return (x + FLAT_FRONT_X_OFFSET, posed_y + FLAT_FRONT_Y_OFFSET, 0.0)
    if section_name.startswith("back_"):
        return (x + FLAT_BACK_X_OFFSET, posed_y + FLAT_BACK_Y_OFFSET, 0.0)
    if section_name.endswith("upper_arm"):
        side_sign = 1.0 if section_name.startswith("left_") else -1.0
        is_front_half = posed_z >= 0.0
        panel_y = FLAT_FRONT_Y_OFFSET if is_front_half else FLAT_BACK_Y_OFFSET
        flat_x = side_sign * (FLAT_ARM_X_OFFSET + abs(posed_z) * FLAT_ARM_Z_SCALE)
        return (flat_x, posed_y + panel_y, 0.0)
    if section_name.endswith("forearm"):
        side_sign = 1.0 if section_name.startswith("left_") else -1.0
        flat_x = (
            side_sign * (FLAT_ARM_X_OFFSET + FLAT_FOREARM_X_OFFSET)
            + abs(posed_z) * FLAT_FOREARM_Z_SCALE
        )
        return (flat_x, posed_y + FLAT_BACK_Y_OFFSET, 0.0)
    return (posed_x, posed_y, 0.0)


def display_point_3d(section_name: str,
                     point: tuple[float, float, float],
                     section_u: float) -> tuple[float, float, float]:
    x, y, z = point

    is_front = section_name.startswith("front_") or section_name.endswith("ring")
    is_back = section_name.startswith("back_")

    if is_front or is_back:
        if is_back:
            y = BACK_Y_FLIP_PIVOT - y
        elif section_name.endswith("belt"):
            y = FRONT_BELT_Y

        side_frac = min(1.0, abs(x) / TORSO_WRAP_HALF_WIDTH)
        side_sign = 1.0 if x >= 0.0 else -1.0
        if is_front:
            angle = side_sign * (side_frac * math.pi * 0.5)
        else:
            angle = math.pi - side_sign * (side_frac * math.pi * 0.5)

        x = TORSO_RADIUS_X * math.sin(angle)
        z = TORSO_RADIUS_Z * math.cos(angle)
        if section_name.endswith("_top"):
            top_amount = _shoulder_top_amount(y)
            if is_back:
                x_pull = BACK_SHOULDER_TOP_X_PULL
                z_pull = BACK_SHOULDER_TOP_Z_PULL
                y_lift = BACK_SHOULDER_TOP_Y_LIFT
            else:
                x_pull = FRONT_SHOULDER_TOP_X_PULL
                z_pull = FRONT_SHOULDER_TOP_Z_PULL
                y_lift = FRONT_SHOULDER_TOP_Y_LIFT
            x = x * (1.0 - x_pull * top_amount)
            z = z * (1.0 - z_pull * top_amount)
            y = y - y_lift * top_amount

    if section_name.endswith("upper_arm"):
        side_sign = 1.0 if section_name.startswith("left_") else -1.0
        z, y = _upper_arm_profile(section_u)
        x = side_sign * (
            UPPER_ARM_OUTWARD_X +
            UPPER_ARM_MID_BULGE * (1.0 - abs(section_u * 2.0 - 1.0))
        )
    elif section_name.endswith("forearm"):
        side_sign = 1.0 if section_name.startswith("left_") else -1.0
        x = side_sign * FOREARM_OUTWARD_X
        y = y + FOREARM_Y_OFFSET
        z = z + FOREARM_FRONT_OFFSET * section_u

    return (x, y, z)


def section_path_points(section: LayoutSection,
                        mode: str,
                        arc_steps: int = ARC_PREVIEW_STEPS) -> tuple[tuple[float, float, float], ...]:
    if section.geom != "arc":
        points = list(section.points)
        total = max(1, len(points) - 1)
        return tuple(
            display_point(mode, section.name, point, index / total if total > 0 else 0.5)
            for index, point in enumerate(points)
        )

    points: list[tuple[float, float, float]] = []
    for index in range(arc_steps + 1):
        t = index / arc_steps
        angle = section.start_deg + section.sweep_deg * t
        angle_rad = math.radians(angle)
        raw_point = (
            section.center[0] + section.radius * math.cos(angle_rad),
            section.center[1] + section.radius * math.sin(angle_rad),
            section.center[2],
        )
        points.append(display_point(mode, section.name, raw_point, t))
    return tuple(points)


def section_path_segments(section: LayoutSection,
                          mode: str,
                          arc_steps: int = ARC_PREVIEW_STEPS,
                          break_distance: float = FLAT_PATH_BREAK_DISTANCE) -> tuple[tuple[tuple[float, float, float], ...], ...]:
    points = section_path_points(section, mode, arc_steps=arc_steps)
    if len(points) < 2 or mode != "2d":
        return (points,) if points else ()

    segments: list[list[tuple[float, float, float]]] = [[points[0]]]
    for previous, current in zip(points, points[1:]):
        distance = math.dist(previous, current)
        if distance > break_distance:
            if len(segments[-1]) >= 2:
                segments.append([current])
            else:
                segments[-1] = [current]
            continue
        segments[-1].append(current)

    return tuple(tuple(segment) for segment in segments if len(segment) >= 2)


def _upper_arm_profile(section_u: float) -> tuple[float, float]:
    t = max(0.0, min(1.0, section_u))
    top_y = UPPER_ARM_TOP_Y
    bottom_y = UPPER_ARM_BOTTOM_Y
    depth = UPPER_ARM_DEPTH
    center_span = (2.0 * depth) / (1.0 + math.sqrt(2.0))
    diagonal_step = center_span / math.sqrt(2.0)
    vertical_drop = max(0.0, bottom_y - top_y - diagonal_step)
    profile = (
        (-depth, top_y),
        (-depth, top_y + vertical_drop),
        (-depth + diagonal_step, bottom_y),
        (depth - diagonal_step, bottom_y),
        (depth, top_y + vertical_drop),
        (depth, top_y),
    )
    return _sample_profile(profile, t)


def _shoulder_top_amount(y: float) -> float:
    if y >= SHOULDER_TOP_BLEND_START_Y:
        return 0.0
    if y <= SHOULDER_TOP_BLEND_END_Y:
        return 1.0
    ratio = (SHOULDER_TOP_BLEND_START_Y - y) / (
        SHOULDER_TOP_BLEND_START_Y - SHOULDER_TOP_BLEND_END_Y
    )
    return ratio * ratio


def _sample_profile(profile: tuple[tuple[float, float], ...],
                    t: float) -> tuple[float, float]:
    if not profile:
        return (0.0, 0.0)
    if len(profile) == 1:
        return profile[0]

    segment_lengths = [
        math.dist(start, end)
        for start, end in zip(profile, profile[1:])
    ]
    total_length = sum(segment_lengths)
    if total_length <= 1e-6:
        return profile[0]

    target = max(0.0, min(1.0, t)) * total_length
    walked = 0.0
    for index, segment_length in enumerate(segment_lengths):
        next_walked = walked + segment_length
        if target <= next_walked or index == len(segment_lengths) - 1:
            if segment_length <= 1e-6:
                return profile[index]
            local_t = (target - walked) / segment_length
            start = profile[index]
            end = profile[index + 1]
            return (
                start[0] + (end[0] - start[0]) * local_t,
                start[1] + (end[1] - start[1]) * local_t,
            )
        walked = next_walked
    return profile[-1]
