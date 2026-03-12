from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence


Point3 = tuple[float, float, float]

EPSILON = 1e-6
MAX_LAYOUT_NAME_CHARS = 19
DEFAULT_LED_DENSITY = 2.3

FRONT_Z = 0.0
BACK_Z = 0.0

RING_CENTER: Point3 = (0.0, 0.0, 0.0)
RING_DIAMETER = 8.5
RING_RADIUS = RING_DIAMETER * 0.5


@dataclass(frozen=True)
class GeometrySection:
    name: str
    strip: int
    reversed: bool
    connected_to_prev: bool
    geom: str
    points: tuple[Point3, ...] = ()
    center: Point3 = (0.0, 0.0, 0.0)
    radius: float = 0.0
    start_deg: float = 0.0
    sweep_deg: float = 0.0

    def curve_length(self) -> float:
        return curve_length(self.geom, self.points, self.radius, self.sweep_deg)

    def resolve(self, led_count: int) -> "LayoutSection":
        return LayoutSection(
            name=self.name,
            strip=self.strip,
            reversed=self.reversed,
            connected_to_prev=self.connected_to_prev,
            geom=self.geom,
            led_count=max(1, led_count),
            points=self.points,
            center=self.center,
            radius=self.radius,
            start_deg=self.start_deg,
            sweep_deg=self.sweep_deg,
        )


@dataclass(frozen=True)
class LayoutSection:
    name: str
    strip: int
    reversed: bool
    connected_to_prev: bool
    geom: str
    led_count: int
    points: tuple[Point3, ...] = ()
    center: Point3 = (0.0, 0.0, 0.0)
    radius: float = 0.0
    start_deg: float = 0.0
    sweep_deg: float = 0.0

    def curve_length(self) -> float:
        return curve_length(self.geom, self.points, self.radius, self.sweep_deg)


@dataclass(frozen=True)
class SuitProfile:
    name: str
    default_density: float = DEFAULT_LED_DENSITY
    strip_names: tuple[str, str] = ("left", "right")
    section_led_counts: tuple[tuple[str, int], ...] = ()

    def count_map(self) -> dict[str, int]:
        return dict(self.section_led_counts)


# Exterior front view coordinates: +x is viewer-right, which is left when worn.
RIGHT_FRONT_GROUPS_2D: dict[str, tuple[tuple[float, float], ...]] = {
    "upper": ((11.5, -13.0), (10.5, -6.0), (6.5, -3.5)),
    "outer": ((6.0, 9.0), (13.0, 15.0), (24.0, 17.5)),
    "inner": ((8.5, 16.5), (6.5, 18.0), (5.0, 29.0)),
    "belt": ((8.5, 33.5), (25.0, 30.0)),
}

# User supplied back y values cumulatively from the belt upward:
# 0,0 / 5.5,+3.5,+6 / +3,+5,+0 / +11,+9,+5.5
RIGHT_BACK_GROUPS_2D: dict[str, tuple[tuple[float, float], ...]] = {
    "back_1": ((21.0, 0.0), (7.5, 0.0)),
    "back_2": ((12.0, 5.5), (7.0, 9.0), (7.0, 15.0)),
    "back_3": ((27.0, 18.0), (16.0, 23.0), (12.0, 23.0)),
    "back_4": ((7.0, 34.0), (7.0, 43.0), (10.0, 48.5)),
}

# Worn-left arm coordinates. Positive x is left when worn.
LEFT_ARM_GROUPS_3D: dict[str, tuple[Point3, ...]] = {
    "upper_arm": (
        (16.0, -6.0, -2.0),
        (17.5, -1.5, -2.0),
        (20.0, 6.0, 0.0),
        (18.5, 4.0, 1.5),
        (17.0, 0.5, 2.0),
    ),
    "forearm": (
        (20.0, 6.0, 0.0),
        (23.0, 15.5, 0.0),
    ),
}

DEFAULT_FRONT_PROFILE = SuitProfile(
    name="chest_front_v1",
    section_led_counts=(("front_left_ring", 35),),
)

DEFAULT_BACK_PROFILE = SuitProfile(
    name="chest_back_v1",
)

DEFAULT_CHESTPLATE_PROFILE = SuitProfile(
    name="chestplate_v1",
    section_led_counts=DEFAULT_FRONT_PROFILE.section_led_counts,
)


def curve_length(geom: str,
                 points: Sequence[Point3],
                 radius: float,
                 sweep_deg: float) -> float:
    if geom == "arc":
        return abs(sweep_deg) * math.pi / 180.0 * radius

    if len(points) < 2:
        return 0.0

    total = 0.0
    for start, end in zip(points, points[1:]):
        total += math.dist(start, end)
    return total


def ensure_name_fits(name: str, kind: str) -> None:
    if len(name) > MAX_LAYOUT_NAME_CHARS:
        raise ValueError(
            f"{kind} '{name}' exceeds {MAX_LAYOUT_NAME_CHARS} characters"
        )


def mirror_point(point: Point3) -> Point3:
    return (-point[0], point[1], point[2])


def with_z(points_2d: Sequence[tuple[float, float]], z: float = 0.0) -> tuple[Point3, ...]:
    return tuple((x, y, z) for x, y in points_2d)


def estimate_led_count(length: float, density: float) -> int:
    return max(2, int(round(length * density)))


def make_profile(base_profile: SuitProfile = DEFAULT_CHESTPLATE_PROFILE,
                 name: str | None = None,
                 density: float | None = None,
                 overrides: Mapping[str, int] | None = None) -> SuitProfile:
    if density is None:
        density = base_profile.default_density
    if density <= 0.0:
        raise ValueError("density must be positive")
    counts = base_profile.count_map()
    if overrides:
        for section_name, led_count in overrides.items():
            if led_count < 1:
                raise ValueError(f"section '{section_name}' needs at least 1 LED")
            counts[section_name] = led_count
    return SuitProfile(
        name=name or base_profile.name,
        default_density=density,
        strip_names=base_profile.strip_names,
        section_led_counts=tuple(counts.items()),
    )


def parse_count_overrides(items: Sequence[str]) -> dict[str, int]:
    overrides: dict[str, int] = {}
    for item in items:
        name, sep, raw_count = item.partition("=")
        if not sep:
            raise ValueError(f"override '{item}' must look like section=count")
        section_name = name.strip()
        if not section_name:
            raise ValueError(f"override '{item}' is missing a section name")
        led_count = int(raw_count.strip())
        if led_count < 1:
            raise ValueError(f"section '{section_name}' needs at least 1 LED")
        overrides[section_name] = led_count
    return overrides


def polyline_section(name: str,
                     strip: int,
                     points: Sequence[Point3],
                     reversed_flag: bool = False,
                     connected_to_prev: bool = False) -> GeometrySection:
    return GeometrySection(
        name=name,
        strip=strip,
        reversed=reversed_flag,
        connected_to_prev=connected_to_prev,
        geom="polyline",
        points=tuple(points),
    )


def mirrored_strip_sections(groups_2d: Mapping[str, Sequence[tuple[float, float]]],
                            left_prefix: str,
                            right_prefix: str,
                            z: float) -> list[GeometrySection]:
    right = {name: with_z(points, z=z) for name, points in groups_2d.items()}
    left = {
        name: tuple(mirror_point(point) for point in points)
        for name, points in right.items()
    }

    sections: list[GeometrySection] = []
    for name in groups_2d:
        sections.append(polyline_section(f"{left_prefix}{name}", 0, left[name]))
        sections.append(polyline_section(f"{right_prefix}{name}", 1, right[name]))
    return sections


def left_front_geometry() -> list[GeometrySection]:
    sections = [
        polyline_section(
            "front_left_top",
            0,
            with_z(RIGHT_FRONT_GROUPS_2D["upper"], z=FRONT_Z),
        ),
        GeometrySection(
            name="front_left_ring",
            strip=0,
            reversed=False,
            connected_to_prev=False,
            geom="arc",
            center=RING_CENTER,
            radius=RING_RADIUS,
            # 0 deg is viewer-right, which is left when worn. Use a negative
            # sweep because torso coordinates use +y downward.
            start_deg=0.0,
            sweep_deg=-360.0,
        ),
        polyline_section(
            "front_left_rib",
            0,
            with_z(RIGHT_FRONT_GROUPS_2D["outer"], z=FRONT_Z),
        ),
        polyline_section(
            "front_left_abs",
            0,
            with_z(RIGHT_FRONT_GROUPS_2D["inner"], z=FRONT_Z),
        ),
        polyline_section(
            "front_left_belt",
            0,
            with_z(RIGHT_FRONT_GROUPS_2D["belt"], z=FRONT_Z),
        ),
    ]

    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def right_front_geometry() -> list[GeometrySection]:
    sections = [
        polyline_section(
            "front_right_top",
            1,
            tuple(mirror_point(point) for point in with_z(RIGHT_FRONT_GROUPS_2D["upper"], z=FRONT_Z)),
        ),
        polyline_section(
            "front_right_rib",
            1,
            tuple(mirror_point(point) for point in with_z(RIGHT_FRONT_GROUPS_2D["outer"], z=FRONT_Z)),
        ),
        polyline_section(
            "front_right_abs",
            1,
            tuple(mirror_point(point) for point in with_z(RIGHT_FRONT_GROUPS_2D["inner"], z=FRONT_Z)),
        ),
        polyline_section(
            "front_right_belt",
            1,
            tuple(mirror_point(point) for point in with_z(RIGHT_FRONT_GROUPS_2D["belt"], z=FRONT_Z)),
        ),
    ]

    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def left_back_geometry() -> list[GeometrySection]:
    left = {name: with_z(points, z=BACK_Z) for name, points in RIGHT_BACK_GROUPS_2D.items()}
    sections = [
        polyline_section("back_left_belt", 0, left["back_1"]),
        polyline_section("back_left_vertebra", 0, left["back_2"]),
        polyline_section("back_left_rib", 0, left["back_3"]),
        polyline_section("back_left_top", 0, left["back_4"]),
    ]

    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def right_back_geometry() -> list[GeometrySection]:
    right = {
        name: tuple(mirror_point(point) for point in with_z(points, z=BACK_Z))
        for name, points in RIGHT_BACK_GROUPS_2D.items()
    }
    sections = [
        polyline_section("back_right_belt", 1, right["back_1"]),
        polyline_section("back_right_vertebra", 1, right["back_2"]),
        polyline_section("back_right_rib", 1, right["back_3"]),
        polyline_section("back_right_top", 1, right["back_4"]),
    ]

    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def left_arm_geometry() -> list[GeometrySection]:
    sections = [
        polyline_section("left_upper_arm", 0, LEFT_ARM_GROUPS_3D["upper_arm"]),
        polyline_section("left_forearm", 0, LEFT_ARM_GROUPS_3D["forearm"], connected_to_prev=True),
    ]

    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def right_arm_geometry() -> list[GeometrySection]:
    right = {
        name: tuple(mirror_point(point) for point in points)
        for name, points in LEFT_ARM_GROUPS_3D.items()
    }
    sections = [
        polyline_section("right_upper_arm", 1, right["upper_arm"]),
        polyline_section("right_forearm", 1, right["forearm"], connected_to_prev=True),
    ]

    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def front_geometry() -> list[GeometrySection]:
    return left_front_geometry() + left_arm_geometry() + right_front_geometry() + right_arm_geometry()


def back_geometry() -> list[GeometrySection]:
    return left_back_geometry() + left_arm_geometry() + right_back_geometry() + right_arm_geometry()


def chestplate_geometry() -> list[GeometrySection]:
    sections = (
        left_front_geometry() +
        left_back_geometry() +
        left_arm_geometry() +
        right_front_geometry() +
        right_back_geometry() +
        right_arm_geometry()
    )
    for section in sections:
        ensure_name_fits(section.name, "section name")
    return sections


def build_layout_sections(profile: SuitProfile,
                          geometry: Sequence[GeometrySection] | None = None) -> list[LayoutSection]:
    ensure_name_fits(profile.name, "profile name")
    source = geometry if geometry is not None else chestplate_geometry()
    counts = profile.count_map()
    known_names = {section.name for section in source}
    unknown_names = sorted(name for name in counts if name not in known_names)
    if unknown_names:
        raise ValueError(
            "unknown section override(s): " + ", ".join(unknown_names)
        )

    sections: list[LayoutSection] = []
    for section in source:
        led_count = counts.get(section.name)
        if led_count is None:
            led_count = estimate_led_count(section.curve_length(), profile.default_density)
        sections.append(section.resolve(led_count))
    return sections


def build_front_layout(profile: SuitProfile = DEFAULT_FRONT_PROFILE) -> list[LayoutSection]:
    return build_layout_sections(profile, front_geometry())


def build_back_layout(profile: SuitProfile = DEFAULT_BACK_PROFILE) -> list[LayoutSection]:
    return build_layout_sections(profile, back_geometry())


def build_chestplate_layout(
    profile: SuitProfile = DEFAULT_CHESTPLATE_PROFILE,
) -> list[LayoutSection]:
    return build_layout_sections(profile, chestplate_geometry())


def is_closed_arc(section: LayoutSection) -> bool:
    return section.geom == "arc" and abs(abs(section.sweep_deg) - 360.0) < 0.001


def sample_polyline(points: Sequence[Point3], t: float) -> Point3:
    if not points:
        return (0.0, 0.0, 0.0)
    if len(points) == 1:
        return points[0]

    segment_lengths = [math.dist(start, end) for start, end in zip(points, points[1:])]
    total = sum(segment_lengths)
    if total < EPSILON:
        return points[0]

    target = max(0.0, min(1.0, t)) * total
    walked = 0.0
    for index, segment_length in enumerate(segment_lengths):
        next_walked = walked + segment_length
        if target <= next_walked or index == len(segment_lengths) - 1:
            if segment_length < EPSILON:
                return points[index]
            local_t = (target - walked) / segment_length
            start = points[index]
            end = points[index + 1]
            return (
                start[0] + (end[0] - start[0]) * local_t,
                start[1] + (end[1] - start[1]) * local_t,
                start[2] + (end[2] - start[2]) * local_t,
            )
        walked = next_walked

    return points[-1]


def sample_arc(section: LayoutSection, t: float) -> Point3:
    angle_deg = section.start_deg + section.sweep_deg * max(0.0, min(1.0, t))
    angle_rad = math.radians(angle_deg)
    return (
        section.center[0] + section.radius * math.cos(angle_rad),
        section.center[1] + section.radius * math.sin(angle_rad),
        section.center[2],
    )


def sample_section_point(section: LayoutSection, index: int) -> Point3:
    if section.led_count <= 1:
        t = 0.5
    elif is_closed_arc(section):
        t = index / section.led_count
    else:
        t = index / (section.led_count - 1)

    if section.geom == "arc":
        return sample_arc(section, t)
    return sample_polyline(section.points, t)


def sample_section_points(section: LayoutSection) -> tuple[Point3, ...]:
    return tuple(
        sample_section_point(section, index)
        for index in range(section.led_count)
    )


def section_endpoint(section: LayoutSection, start: bool) -> Point3:
    if section.geom == "arc":
        angle_deg = section.start_deg if start else section.start_deg + section.sweep_deg
        angle_rad = math.radians(angle_deg)
        return (
            section.center[0] + section.radius * math.cos(angle_rad),
            section.center[1] + section.radius * math.sin(angle_rad),
            section.center[2],
        )
    if not section.points:
        return (0.0, 0.0, 0.0)
    return section.points[0] if start else section.points[-1]


def write_layout_file(sections: Sequence[LayoutSection], profile: SuitProfile) -> str:
    ensure_name_fits(profile.name, "profile name")
    for strip_name in profile.strip_names:
        ensure_name_fits(strip_name, "strip name")

    lines = [
        "version=2",
        f"profile={profile.name}",
        f"strip_count={len(profile.strip_names)}",
    ]
    for strip_index, strip_name in enumerate(profile.strip_names):
        lines.append(f"strip{strip_index}_name={strip_name}")
    lines.append(f"section_count={len(sections)}")

    for index, section in enumerate(sections):
        ensure_name_fits(section.name, "section name")
        lines.append(f"section{index}_name={section.name}")
        lines.append(f"section{index}_strip={section.strip}")
        lines.append(f"section{index}_length={section.led_count}")
        lines.append(f"section{index}_reversed={1 if section.reversed else 0}")
        lines.append(f"section{index}_connected={1 if section.connected_to_prev else 0}")
        lines.append(f"section{index}_geom={section.geom}")
        if section.geom == "polyline":
            lines.append(f"section{index}_point_count={len(section.points)}")
            for point_index, point in enumerate(section.points):
                lines.append(
                    f"section{index}_p{point_index}="
                    f"{point[0]:.4f},{point[1]:.4f},{point[2]:.4f}"
                )
        elif section.geom == "arc":
            lines.append(
                f"section{index}_center="
                f"{section.center[0]:.4f},{section.center[1]:.4f},{section.center[2]:.4f}"
            )
            lines.append(f"section{index}_radius={section.radius:.4f}")
            lines.append(f"section{index}_start_deg={section.start_deg:.4f}")
            lines.append(f"section{index}_sweep_deg={section.sweep_deg:.4f}")

    return "\n".join(lines) + "\n"


def save_layout_file(path: str | Path,
                     sections: Sequence[LayoutSection],
                     profile: SuitProfile) -> Path:
    output_path = Path(path)
    output_path.write_text(write_layout_file(sections, profile), encoding="ascii")
    return output_path
