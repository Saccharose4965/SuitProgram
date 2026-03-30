from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import escape

from tools.spatial_display import display_point, section_path_points

from .layout_io import LoadedLayout
from .preview_model import PreviewFrame


BG_COLOR = "#11151c"
GRID_COLOR = "#2a3444"
TEXT_COLOR = "#d7dfeb"
UNLIT_LED_FILL = "#000000"

SVG_SCALE = 12.0
SVG_PADDING_X = 40.0
SVG_PADDING_Y = 50.0


def _project_point(point: tuple[float, float, float], projection: str) -> tuple[float, float]:
    x, y, z = point
    if projection == "iso":
        return (x + z * 0.85, y - z * 0.65)
    return (x, y)


def _display_mode(projection: str) -> str:
    return "3d" if projection == "iso" else "2d"


def _bounds(layout: LoadedLayout, frame: PreviewFrame, projection: str) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    mode = _display_mode(projection)
    for section in layout.sections:
        for point in section_path_points(section, mode):
            px, py = _project_point(point, projection)
            xs.append(px)
            ys.append(py)
    for section_name, point, section_u in zip(frame.section_names, frame.points, frame.section_us):
        point = display_point(mode, section_name, point, section_u)
        px, py = _project_point(point, projection)
        xs.append(px)
        ys.append(py)
    if not xs or not ys:
        return (-1.0, -1.0, 1.0, 1.0)
    return (min(xs), min(ys), max(xs), max(ys))


def _svg_point(point: tuple[float, float, float], bounds: tuple[float, float, float, float], projection: str) -> tuple[float, float]:
    min_x, min_y, _, _ = bounds
    px, py = _project_point(point, projection)
    return (
        SVG_PADDING_X + (px - min_x) * SVG_SCALE,
        SVG_PADDING_Y + (py - min_y) * SVG_SCALE,
    )


def _rgba_hex(color_rgba8: tuple[int, int, int, int]) -> tuple[str, float]:
    r, g, b, a = color_rgba8
    return f"#{r:02x}{g:02x}{b:02x}", a / 255.0


def render_preview_svg(
    layout: LoadedLayout,
    frame: PreviewFrame,
    output_path: str | Path,
    projection: str = "flat",
) -> Path:
    bounds = _bounds(layout, frame, projection)
    min_x, min_y, max_x, max_y = bounds
    width = 2.0 * SVG_PADDING_X + (max_x - min_x) * SVG_SCALE
    height = 2.0 * SVG_PADDING_Y + (max_y - min_y) * SVG_SCALE

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" '
            f'height="{height:.0f}" viewBox="0 0 {width:.2f} {height:.2f}">'
        ),
        f'<rect x="0" y="0" width="{width:.2f}" height="{height:.2f}" fill="{BG_COLOR}"/>',
        (
            f'<text x="16" y="22" fill="{TEXT_COLOR}" font-size="15" '
            f'font-family="monospace">{escape(layout.profile.name)} role={frame.role} '
            f't={frame.time_ms}ms projection={projection}</text>'
        ),
    ]

    for section_name, point, section_u, color_rgba8 in zip(
        frame.section_names,
        frame.points,
        frame.section_us,
        frame.colors_rgba8,
    ):
        fill, opacity = _rgba_hex(color_rgba8)
        point = display_point(_display_mode(projection), section_name, point, section_u)
        cx, cy = _svg_point(point, bounds, projection)
        if color_rgba8[3] == 0 or color_rgba8[:3] == (0, 0, 0):
            fill = UNLIT_LED_FILL
            opacity = 1.0
        lines.append(
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="3.0" fill="{fill}" '
            f'fill-opacity="{opacity:.3f}" stroke="none"/>'
        )

    lines.append("</svg>")
    output = Path(output_path)
    output.write_text("\n".join(lines) + "\n", encoding="ascii")
    return output
