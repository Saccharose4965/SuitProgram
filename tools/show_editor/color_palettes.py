from __future__ import annotations

from typing import Any


RAINBOW_PALETTE_TEXT = (
    "0:#ff0000; "
    "0.1667:#ffff00; "
    "0.3333:#00ff00; "
    "0.5:#00ffff; "
    "0.6667:#0000ff; "
    "0.8333:#ff00ff; "
    "1:#ff0000"
)

BUILTIN_COLOR_PALETTES: dict[str, str] = {
    "Rainbow": RAINBOW_PALETTE_TEXT,
}


def _fmt_float(value: float) -> str:
    text = f"{float(value):.4f}"
    text = text.rstrip("0").rstrip(".")
    return text or "0"


def _parse_hex_rgba(token: str) -> tuple[int, int, int, int] | None:
    color_token = str(token).strip().lstrip("#")
    if len(color_token) == 6:
        color_token += "ff"
    if len(color_token) != 8:
        return None
    try:
        return tuple(int(color_token[index:index + 2], 16) for index in range(0, 8, 2))  # type: ignore[return-value]
    except ValueError:
        return None


def normalize_palette_stops_even(stops: list[tuple[float, tuple[int, int, int, int]]]) -> list[tuple[float, tuple[int, int, int, int]]]:
    colors = [tuple(int(channel) for channel in rgba) for _, rgba in sorted(stops, key=lambda item: item[0])]
    if not colors:
        return []
    if len(colors) == 1:
        return [(0.0, colors[0])]
    return [
        (index / (len(colors) - 1), rgba)
        for index, rgba in enumerate(colors)
    ]


def parse_palette_text_even(text: str) -> list[tuple[float, tuple[int, int, int, int]]]:
    ordered: list[tuple[float, tuple[int, int, int, int]]] = []
    for index, segment in enumerate(str(text).split(";")):
        entry = segment.strip()
        if not entry:
            continue
        sort_key = float(index)
        color_token = entry
        if ":" in entry:
            left, right = entry.split(":", 1)
            rgba = _parse_hex_rgba(right.strip())
            if rgba is None:
                rgba = _parse_hex_rgba(entry.strip())
            else:
                try:
                    sort_key = float(left.strip())
                except ValueError:
                    sort_key = float(index)
                ordered.append((sort_key, rgba))
                continue
        rgba = _parse_hex_rgba(color_token.strip())
        if rgba is None:
            continue
        ordered.append((sort_key, rgba))
    return normalize_palette_stops_even(ordered)


def encode_palette_text_even(stops: list[tuple[float, tuple[int, int, int, int]]]) -> str:
    normalized = normalize_palette_stops_even(stops)
    parts: list[str] = []
    for point_t, rgba in normalized:
        r, g, b, a = rgba
        if a == 255:
            color_text = f"#{r:02x}{g:02x}{b:02x}"
        else:
            color_text = f"#{r:02x}{g:02x}{b:02x}{a:02x}"
        parts.append(f"{_fmt_float(point_t)}:{color_text}")
    return "; ".join(parts)


def normalize_palette_store_text(value: Any) -> str:
    if isinstance(value, str):
        return encode_palette_text_even(parse_palette_text_even(value))
    if not isinstance(value, list):
        return ""
    stops: list[tuple[float, tuple[int, int, int, int]]] = []
    for index, entry in enumerate(value):
        if not isinstance(entry, (list, tuple)):
            continue
        rgba: tuple[int, int, int, int] | None = None
        if len(entry) >= 5:
            try:
                rgba = tuple(max(0, min(255, int(channel))) for channel in entry[1:5])  # type: ignore[assignment]
            except (TypeError, ValueError):
                rgba = None
        elif len(entry) >= 4:
            try:
                rgba = tuple(max(0, min(255, int(channel))) for channel in entry[:4])  # type: ignore[assignment]
            except (TypeError, ValueError):
                rgba = None
        if rgba is None:
            continue
        stops.append((float(index), rgba))
    return encode_palette_text_even(stops)


def resolve_palette_text(project_palettes: dict[str, Any] | None, preset_name: str, custom_text: str = "") -> str:
    normalized_name = str(preset_name or "").strip()
    if normalized_name in BUILTIN_COLOR_PALETTES:
        return BUILTIN_COLOR_PALETTES[normalized_name]
    if normalized_name and project_palettes is not None:
        project_value = project_palettes.get(normalized_name)
        project_text = normalize_palette_store_text(project_value)
        if project_text:
            return project_text
    return str(custom_text or "").strip()
