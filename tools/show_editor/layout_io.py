from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from tools.layout_model import (
    DEFAULT_CHESTPLATE_PROFILE,
    LayoutSection,
    SuitProfile,
    build_chestplate_layout,
)


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class LoadedLayout:
    profile: SuitProfile
    sections: tuple[LayoutSection, ...]
    source_path: Path | None = None


def _parse_bool(raw: str) -> bool:
    return int(raw.strip()) != 0


def _parse_point(raw: str) -> Point3:
    parts = [part.strip() for part in raw.split(",")]
    if len(parts) != 3:
        raise ValueError(f"invalid point value: {raw}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def generated_layout() -> LoadedLayout:
    sections = tuple(build_chestplate_layout(DEFAULT_CHESTPLATE_PROFILE))
    return LoadedLayout(profile=DEFAULT_CHESTPLATE_PROFILE, sections=sections, source_path=None)


def load_layout_file(path: str | Path) -> LoadedLayout:
    layout_path = Path(path)
    raw_map: dict[str, str] = {}
    for raw_line in layout_path.read_text(encoding="ascii").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, sep, value = line.partition("=")
        if not sep:
            continue
        raw_map[key.strip()] = value.strip()

    version = int(raw_map.get("version", "0"))
    if version != 2:
        raise ValueError(f"unsupported layout version: {version}")

    strip_count = int(raw_map["strip_count"])
    strip_names = tuple(raw_map.get(f"strip{index}_name", f"strip{index}") for index in range(strip_count))
    profile = SuitProfile(
        name=raw_map.get("profile", layout_path.stem),
        strip_names=strip_names,
    )

    sections: list[LayoutSection] = []
    section_count = int(raw_map["section_count"])
    for index in range(section_count):
        prefix = f"section{index}_"
        geom = raw_map[prefix + "geom"]
        common = {
            "name": raw_map[prefix + "name"],
            "strip": int(raw_map[prefix + "strip"]),
            "reversed": _parse_bool(raw_map.get(prefix + "reversed", "0")),
            "connected_to_prev": _parse_bool(raw_map.get(prefix + "connected", "0")),
            "geom": geom,
            "led_count": int(raw_map[prefix + "length"]),
        }
        if geom == "polyline":
            point_count = int(raw_map[prefix + "point_count"])
            points = tuple(_parse_point(raw_map[f"{prefix}p{point_index}"]) for point_index in range(point_count))
            sections.append(LayoutSection(points=points, **common))
            continue
        if geom == "arc":
            sections.append(
                LayoutSection(
                    center=_parse_point(raw_map[prefix + "center"]),
                    radius=float(raw_map[prefix + "radius"]),
                    start_deg=float(raw_map[prefix + "start_deg"]),
                    sweep_deg=float(raw_map[prefix + "sweep_deg"]),
                    **common,
                )
            )
            continue
        raise ValueError(f"unsupported geometry type: {geom}")

    return LoadedLayout(profile=profile, sections=tuple(sections), source_path=layout_path)
