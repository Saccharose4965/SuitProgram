from __future__ import annotations

import argparse
import math
from pathlib import Path
from xml.sax.saxutils import escape

from layout_model import (
    DEFAULT_BACK_PROFILE,
    DEFAULT_CHESTPLATE_PROFILE,
    DEFAULT_FRONT_PROFILE,
    build_back_layout,
    build_front_layout,
    make_profile,
    parse_count_overrides,
    sample_section_points,
)


LEFT_COLOR = "#1267a8"
RIGHT_COLOR = "#c7641e"
RING_COLOR = "#bd3c2a"
GRID_COLOR = "#c9c0b4"
BG_COLOR = "#f5efe4"
TEXT_COLOR = "#342a1f"
LED_FILL = "#fffaf2"
LED_STROKE = "#201912"
JUMP_COLOR = "#8d8170"

ARC_PREVIEW_SEGMENTS = 96
SVG_SCALE = 14.0
SVG_PADDING_X = 40.0
SVG_PADDING_Y = 40.0
PANE_GAP = 48.0

VIEW_CHOICES = ("front", "back", "chestplate")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render the LED layout preview as SVG."
    )
    parser.add_argument(
        "--view",
        choices=VIEW_CHOICES,
        default="front",
        help="which geometry to render",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="SVG output path",
    )
    parser.add_argument(
        "--profile-name",
        default=None,
        help="profile name written into the layout metadata",
    )
    parser.add_argument(
        "--density",
        type=float,
        default=None,
        help="fallback LEDs-per-unit for sections without explicit overrides",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="SECTION=COUNT",
        help="override a section LED count",
    )
    return parser.parse_args()


def default_output_path(view: str) -> Path:
    script_dir = Path(__file__).resolve().parent
    if view == "chestplate":
        return script_dir / "chestplate_layout_preview.svg"
    return script_dir / f"{view}_layout_preview.svg"


def default_profile_name(view: str) -> str:
    if view == "front":
        return DEFAULT_FRONT_PROFILE.name
    if view == "back":
        return DEFAULT_BACK_PROFILE.name
    return DEFAULT_CHESTPLATE_PROFILE.name


def default_density(view: str) -> float:
    if view == "front":
        return DEFAULT_FRONT_PROFILE.default_density
    if view == "back":
        return DEFAULT_BACK_PROFILE.default_density
    return DEFAULT_CHESTPLATE_PROFILE.default_density


def default_base_profile(view: str):
    if view == "front":
        return DEFAULT_FRONT_PROFILE
    if view == "back":
        return DEFAULT_BACK_PROFILE
    return DEFAULT_CHESTPLATE_PROFILE


def build_panes(view: str, profile):
    if view == "front":
        return [("front", build_front_layout(profile))]
    if view == "back":
        return [("back", build_back_layout(profile))]
    return [
        ("front", build_front_layout(profile)),
        ("back", build_back_layout(profile)),
    ]


def preview_points(section) -> tuple[tuple[float, float, float], ...]:
    if section.geom == "arc":
        points = []
        for index in range(ARC_PREVIEW_SEGMENTS + 1):
            t = index / ARC_PREVIEW_SEGMENTS
            angle_deg = section.start_deg + section.sweep_deg * t
            angle_rad = math.radians(angle_deg)
            points.append(
                (
                    section.center[0] + section.radius * math.cos(angle_rad),
                    section.center[1] + section.radius * math.sin(angle_rad),
                    section.center[2],
                )
            )
        return tuple(points)
    return section.points


def section_color(section) -> str:
    if section.geom == "arc":
        return RING_COLOR
    return LEFT_COLOR if section.strip == 0 else RIGHT_COLOR


def bounds_for_panes(panes) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for _, sections in panes:
        for section in sections:
            for x, y, _ in preview_points(section):
                xs.append(x)
                ys.append(y)
            for x, y, _ in sample_section_points(section):
                xs.append(x)
                ys.append(y)

    if not xs or not ys:
        return (-1.0, -1.0, 1.0, 1.0)
    return (min(xs), min(ys), max(xs), max(ys))


def svg_xy(point, bounds, pane_index: int, pane_width: float) -> tuple[float, float]:
    min_x, min_y, _, _ = bounds
    pane_origin_x = pane_index * (pane_width + PANE_GAP)
    return (
        pane_origin_x + SVG_PADDING_X + (point[0] - min_x) * SVG_SCALE,
        SVG_PADDING_Y + (point[1] - min_y) * SVG_SCALE,
    )


def path_data(points, bounds, pane_index: int, pane_width: float) -> str:
    commands: list[str] = []
    for index, point in enumerate(points):
        x, y = svg_xy(point, bounds, pane_index, pane_width)
        prefix = "M" if index == 0 else "L"
        commands.append(f"{prefix}{x:.2f},{y:.2f}")
    return " ".join(commands)


def label_point(section):
    leds = sample_section_points(section)
    if leds:
        return leds[len(leds) // 2]
    points = preview_points(section)
    if points:
        return points[len(points) // 2]
    return (0.0, 0.0, 0.0)


def jump_segments(sections) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    jumps: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    previous_by_strip = {}
    for section in sections:
        previous = previous_by_strip.get(section.strip)
        if (
            previous is not None
            and not section.connected_to_prev
            and previous.geom == "polyline"
            and section.geom == "polyline"
            and previous.points
            and section.points
        ):
            jumps.append((previous.points[-1], section.points[0]))
        previous_by_strip[section.strip] = section
    return jumps


def render_svg(panes, output_path: Path) -> Path:
    bounds = bounds_for_panes(panes)
    min_x, min_y, max_x, max_y = bounds
    pane_width = 2.0 * SVG_PADDING_X + (max_x - min_x) * SVG_SCALE
    pane_height = 2.0 * SVG_PADDING_Y + (max_y - min_y) * SVG_SCALE
    width = len(panes) * pane_width + max(0, len(panes) - 1) * PANE_GAP
    height = pane_height

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" '
            f'height="{height:.0f}" viewBox="0 0 {width:.2f} {height:.2f}">'
        ),
        f'<rect x="0" y="0" width="{width:.2f}" height="{height:.2f}" fill="{BG_COLOR}"/>',
        (
            f'<text x="18" y="24" fill="{TEXT_COLOR}" font-size="16" '
            'font-family="monospace">layout preview</text>'
        ),
        (
            f'<text x="18" y="44" fill="{TEXT_COLOR}" font-size="11" '
            'font-family="monospace">solid path = section geometry, dots = sampled LEDs, dashed = jump</text>'
        ),
    ]

    for pane_index, (pane_label, sections) in enumerate(panes):
        pane_origin_x = pane_index * (pane_width + PANE_GAP)
        center_x, _ = svg_xy((0.0, 0.0, 0.0), bounds, pane_index, pane_width)
        lines.append(
            f'<text x="{pane_origin_x + 18.0:.2f}" y="64" fill="{TEXT_COLOR}" '
            f'font-size="13" font-family="monospace">{escape(pane_label)}</text>'
        )
        lines.append(
            f'<line x1="{center_x:.2f}" y1="76.00" x2="{center_x:.2f}" '
            f'y2="{height - 12.0:.2f}" stroke="{GRID_COLOR}" stroke-width="1.2" '
            'stroke-dasharray="7 6"/>'
        )

        for start, end in jump_segments(sections):
            x1, y1 = svg_xy(start, bounds, pane_index, pane_width)
            x2, y2 = svg_xy(end, bounds, pane_index, pane_width)
            lines.append(
                f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
                f'stroke="{JUMP_COLOR}" stroke-width="1.4" stroke-dasharray="5 5"/>'
            )

        for section in sections:
            color = section_color(section)
            lines.append(
                f'<path d="{path_data(preview_points(section), bounds, pane_index, pane_width)}" '
                f'fill="none" stroke="{color}" stroke-width="4.5" stroke-linecap="round" '
                'stroke-linejoin="round"/>'
            )

            for x, y, _ in sample_section_points(section):
                sx, sy = svg_xy((x, y, 0.0), bounds, pane_index, pane_width)
                lines.append(
                    f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="2.2" '
                    f'fill="{LED_FILL}" stroke="{LED_STROKE}" stroke-width="0.8"/>'
                )

            lx, ly = svg_xy(label_point(section), bounds, pane_index, pane_width)
            label = escape(f"{section.name} [{section.led_count}]")
            lines.append(
                f'<text x="{lx + 8.0:.2f}" y="{ly - 8.0:.2f}" fill="{TEXT_COLOR}" '
                'font-size="10" font-family="monospace">'
                f"{label}</text>"
            )

    lines.append("</svg>")
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")
    return output_path


def main() -> None:
    args = parse_args()
    overrides = parse_count_overrides(args.overrides)
    profile = make_profile(
        base_profile=default_base_profile(args.view),
        name=args.profile_name or default_profile_name(args.view),
        density=args.density if args.density is not None else default_density(args.view),
        overrides=overrides,
    )
    panes = build_panes(args.view, profile)
    output_path = render_svg(panes, args.output or default_output_path(args.view))
    print(output_path)


if __name__ == "__main__":
    main()
