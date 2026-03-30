from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import hashlib
import math

from tools.layout_model import LayoutSection, sample_section_points
from tools.spatial_display import display_point_3d

from .color_palettes import BUILTIN_COLOR_PALETTES, parse_palette_text_even, resolve_palette_text
from .canonical_layout import ROLE_NAMES, group_sections
from .layout_io import LoadedLayout
from .project_model import ShowClip, ShowProject


Color = tuple[float, float, float, float]


@dataclass(frozen=True)
class PreviewFrame:
    role: str
    time_ms: int
    section_names: tuple[str, ...]
    points: tuple[tuple[float, float, float], ...]
    section_us: tuple[float, ...]
    colors_rgba8: tuple[tuple[int, int, int, int], ...]


@dataclass(frozen=True)
class _TargetLed:
    section_name: str
    point: tuple[float, float, float]
    effect_point: tuple[float, float, float]
    section_u: float


@dataclass(frozen=True)
class _LayoutRuntime:
    section_names: tuple[str, ...]
    points: tuple[tuple[float, float, float], ...]
    section_us: tuple[float, ...]
    axis_x: tuple[float, ...]
    axis_y: tuple[float, ...]
    axis_z: tuple[float, ...]
    axis_radial: tuple[float, ...]
    section_indices: dict[str, tuple[int, ...]]
    all_indices: tuple[int, ...]


def _clamp01(value: float) -> float:
    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0
    return value


def _rgba_from_params(
    params: dict,
    key: str = "color",
    default: tuple[int, int, int, int] = (255, 255, 255, 255),
) -> Color:
    color = params.get(key, list(default))
    if not isinstance(color, (list, tuple)) or len(color) != 4:
        color = default
    return tuple(_clamp01(float(channel) / 255.0) for channel in color)  # type: ignore[return-value]


def _color_to_rgba8(color: Color) -> tuple[int, int, int, int]:
    return tuple(max(0, min(255, int(round(channel * 255.0)))) for channel in color)  # type: ignore[return-value]


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _parse_hex_rgba(text: str) -> tuple[int, int, int, int] | None:
    token = text.strip().lstrip("#")
    if len(token) == 6:
        token = token + "ff"
    if len(token) != 8:
        return None
    try:
        return tuple(int(token[index:index + 2], 16) for index in range(0, 8, 2))  # type: ignore[return-value]
    except ValueError:
        return None


@lru_cache(maxsize=256)
def _parse_curve_text(text: str) -> dict[str, tuple[tuple[float, float], ...]]:
    curves: dict[str, tuple[tuple[float, float], ...]] = {}
    raw = text.strip()
    if not raw:
        return curves
    for segment in raw.split(";"):
        entry = segment.strip()
        if not entry or "=" not in entry:
            continue
        key, points_text = entry.split("=", 1)
        key = key.strip()
        points: list[tuple[float, float]] = []
        for point_text in points_text.split(","):
            item = point_text.strip()
            if not item or ":" not in item:
                continue
            time_text, value_text = item.split(":", 1)
            try:
                point_t = _clamp01(float(time_text.strip()))
                point_value = float(value_text.strip())
            except ValueError:
                continue
            points.append((point_t, point_value))
        if points:
            points.sort(key=lambda item: item[0])
            curves[key] = tuple(points)
    return curves


def _sample_curve(points: tuple[tuple[float, float], ...], t: float) -> float:
    if not points:
        return 0.0
    t = _clamp01(t)
    if t <= points[0][0]:
        return points[0][1]
    if t >= points[-1][0]:
        return points[-1][1]
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if t0 <= t <= t1:
            if abs(t1 - t0) <= 1e-6:
                return v1
            local_t = (t - t0) / (t1 - t0)
            return _lerp(v0, v1, local_t)
    return points[-1][1]


@lru_cache(maxsize=256)
def _parse_palette_text(text: str) -> tuple[tuple[float, tuple[int, int, int, int]], ...]:
    return tuple(parse_palette_text_even(text))


def _sample_palette(stops: tuple[tuple[float, tuple[int, int, int, int]], ...], t: float) -> Color | None:
    if not stops:
        return None
    t = _clamp01(t)
    if t <= stops[0][0]:
        return tuple(channel / 255.0 for channel in stops[0][1])  # type: ignore[return-value]
    if t >= stops[-1][0]:
        return tuple(channel / 255.0 for channel in stops[-1][1])  # type: ignore[return-value]
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t0 <= t <= t1:
            if abs(t1 - t0) <= 1e-6:
                return tuple(channel / 255.0 for channel in c1)  # type: ignore[return-value]
            local_t = (t - t0) / (t1 - t0)
            return tuple(
                _clamp01(_lerp(c0[index] / 255.0, c1[index] / 255.0, local_t))
                for index in range(4)
            )  # type: ignore[return-value]
    return tuple(channel / 255.0 for channel in stops[-1][1])  # type: ignore[return-value]


def _led_points_in_order(section: LayoutSection) -> tuple[tuple[float, float, float], ...]:
    points = sample_section_points(section)
    if section.reversed:
        return tuple(reversed(points))
    return points


@lru_cache(maxsize=256)
def _section_leds(section: LayoutSection) -> tuple[_TargetLed, ...]:
    points = _led_points_in_order(section)
    total = len(points)
    if total <= 1:
        if not points:
            return ()
        return (
            _TargetLed(
                section_name=section.name,
                point=points[0],
                effect_point=display_point_3d(section.name, points[0], 0.5),
                section_u=0.5,
            ),
        )
    return tuple(
        _TargetLed(
            section_name=section.name,
            point=point,
            effect_point=display_point_3d(section.name, point, index / (total - 1)),
            section_u=index / (total - 1),
        )
        for index, point in enumerate(points)
    )


@lru_cache(maxsize=64)
def _layout_runtime(sections: tuple[LayoutSection, ...]) -> _LayoutRuntime:
    section_names: list[str] = []
    points: list[tuple[float, float, float]] = []
    section_us: list[float] = []
    axis_x: list[float] = []
    axis_y: list[float] = []
    axis_z: list[float] = []
    axis_radial: list[float] = []
    section_indices: dict[str, tuple[int, ...]] = {}

    next_index = 0
    for section in sections:
        leds = _section_leds(section)
        indices = tuple(range(next_index, next_index + len(leds)))
        section_indices[section.name] = indices
        next_index += len(leds)
        for led in leds:
            section_names.append(led.section_name)
            points.append(led.point)
            section_us.append(led.section_u)
            axis_x.append(led.effect_point[0])
            axis_y.append(led.effect_point[1])
            axis_z.append(led.effect_point[2])
            axis_radial.append(
                math.sqrt(
                    led.effect_point[0] ** 2 +
                    led.effect_point[1] ** 2 +
                    led.effect_point[2] ** 2
                )
            )

    return _LayoutRuntime(
        section_names=tuple(section_names),
        points=tuple(points),
        section_us=tuple(section_us),
        axis_x=tuple(axis_x),
        axis_y=tuple(axis_y),
        axis_z=tuple(axis_z),
        axis_radial=tuple(axis_radial),
        section_indices=section_indices,
        all_indices=tuple(range(len(points))),
    )


def _clip_active(clip: ShowClip, time_ms: int) -> bool:
    return clip.start_ms <= time_ms < clip.end_ms


def _clip_t(clip: ShowClip, time_ms: int) -> float:
    duration = max(1, clip.end_ms - clip.start_ms)
    return _clamp01((time_ms - clip.start_ms) / duration)


def _tempo_sync_enabled(clip: ShowClip) -> bool:
    return bool(clip.params.get("tempo_sync", False))


def _cycles_per_beat(clip: ShowClip) -> float:
    explicit_cycles = clip.params.get("cycles_per_beat")
    try:
        if explicit_cycles is not None and float(explicit_cycles) > 0.0:
            return max(0.0625, float(explicit_cycles))
    except (TypeError, ValueError):
        pass
    frequency = float(clip.params.get("frequency_hz", 1.0))
    if frequency > 0.0:
        return max(0.0625, frequency)
    beats_per_cycle = float(clip.params.get("beats_per_cycle", 1.0))
    if beats_per_cycle <= 0.0:
        return 1.0
    return max(0.0625, 1.0 / beats_per_cycle)


def _beat_phase(
    project: ShowProject,
    clip: ShowClip,
    time_ms: int,
    cycles_per_beat: float | None = None,
    phase: float = 0.0,
) -> float:
    return _beat_cycle_position(project, clip, time_ms, cycles_per_beat, phase) % 1.0


def _beat_cycle_position(
    project: ShowProject,
    clip: ShowClip,
    time_ms: int,
    cycles_per_beat: float | None = None,
    phase: float = 0.0,
) -> float:
    if project.tempo_bpm <= 0.0:
        return phase
    beat_ms = 60000.0 / project.tempo_bpm
    rate = _cycles_per_beat(clip) if cycles_per_beat is None else max(0.0625, cycles_per_beat)
    beat_position = (time_ms - project.beat_offset_ms) / beat_ms
    return beat_position * rate + phase


def _time_phase(clip: ShowClip, time_ms: int, frequency_hz: float, phase: float = 0.0) -> float:
    return _time_cycle_position(clip, time_ms, frequency_hz, phase) % 1.0


def _time_cycle_position(clip: ShowClip, time_ms: int, frequency_hz: float, phase: float = 0.0) -> float:
    if frequency_hz <= 0.0:
        return phase
    elapsed_sec = max(0.0, (time_ms - clip.start_ms) / 1000.0)
    return elapsed_sec * frequency_hz + phase


def _effect_phase(project: ShowProject, clip: ShowClip, time_ms: int, frequency_hz: float, phase: float = 0.0) -> float:
    if _tempo_sync_enabled(clip):
        return _beat_phase(project, clip, time_ms, _cycles_per_beat(clip), phase)
    return _time_phase(clip, time_ms, frequency_hz, phase)


def _axis_random_angle(clip: ShowClip | None, cycle_index: int = 0) -> float:
    if clip is None:
        return 0.0
    payload = (
        f"{clip.effect}|{clip.target_kind}|{clip.target}|"
        f"{clip.layer}|{clip.start_ms}|{clip.end_ms}|{max(0, int(cycle_index))}"
    ).encode("utf-8")
    digest = hashlib.blake2b(payload, digest_size=2).digest()
    return (int.from_bytes(digest, "little") / 65535.0) * (math.pi * 2.0)


def _hash_unit(*parts: object) -> float:
    payload = "|".join(str(part) for part in parts).encode("utf-8")
    digest = hashlib.blake2b(payload, digest_size=2).digest()
    return int.from_bytes(digest, "little") / 65535.0


def _axis_values(
    runtime: _LayoutRuntime,
    axis: str,
    clip: ShowClip | None = None,
    time_ms: int = 0,
    fps_hint: int = 30,
    cycle_index: int = 0,
) -> tuple[float, ...]:
    del time_ms
    del fps_hint
    if axis == "x":
        return runtime.axis_x
    if axis == "z":
        return runtime.axis_z
    if axis == "radial":
        return runtime.axis_radial
    if axis == "section_u":
        return runtime.section_us
    if axis == "random_xy":
        angle = _axis_random_angle(clip, cycle_index)
        axis_cos = math.cos(angle)
        axis_sin = math.sin(angle)
        return tuple(
            axis_x * axis_cos + axis_y * axis_sin
            for axis_x, axis_y in zip(runtime.axis_x, runtime.axis_y)
        )
    return runtime.axis_y


def _axis_bounds(
    runtime: _LayoutRuntime,
    indices: tuple[int, ...],
    axis: str,
    clip: ShowClip | None = None,
    time_ms: int = 0,
    fps_hint: int = 30,
    cycle_index: int = 0,
) -> tuple[float, float]:
    if not indices:
        return (0.0, 1.0)
    values = _axis_values(runtime, axis, clip, time_ms, fps_hint, cycle_index)
    lo = values[indices[0]]
    hi = lo
    for index in indices[1:]:
        value = values[index]
        if value < lo:
            lo = value
        elif value > hi:
            hi = value
    return (lo, hi)


def _role_world_offset_x(runtime: _LayoutRuntime, role: str) -> float:
    if role not in ROLE_NAMES or not runtime.axis_x:
        return 0.0
    span_x = max(runtime.axis_x) - min(runtime.axis_x)
    spacing_x = max(1.0, span_x * 1.5)
    center_index = (len(ROLE_NAMES) - 1) * 0.5
    return (ROLE_NAMES.index(role) - center_index) * spacing_x


def _world_xyz(runtime: _LayoutRuntime, role: str) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    offset_x = _role_world_offset_x(runtime, role)
    world_x = tuple(value + offset_x for value in runtime.axis_x)
    return (world_x, runtime.axis_y, runtime.axis_z)


def _world_axis_values(
    runtime: _LayoutRuntime,
    role: str,
    axis: str,
    clip: ShowClip | None = None,
    cycle_index: int = 0,
) -> tuple[float, ...]:
    world_x, world_y, world_z = _world_xyz(runtime, role)
    if axis == "x":
        return world_x
    if axis == "z":
        return world_z
    if axis == "radial":
        return tuple(
            math.sqrt(px * px + py * py + pz * pz)
            for px, py, pz in zip(world_x, world_y, world_z)
        )
    if axis == "section_u":
        return runtime.section_us
    if axis == "random_xy":
        angle = _axis_random_angle(clip, cycle_index)
        axis_cos = math.cos(angle)
        axis_sin = math.sin(angle)
        return tuple(
            px * axis_cos + py * axis_sin
            for px, py in zip(world_x, world_y)
        )
    return world_y


def _global_axis_bounds(
    runtime: _LayoutRuntime,
    axis: str,
    clip: ShowClip | None = None,
    cycle_index: int = 0,
) -> tuple[float, float]:
    first = True
    lo = 0.0
    hi = 0.0
    for role in ROLE_NAMES:
        values = _world_axis_values(runtime, role, axis, clip, cycle_index)
        if not values:
            continue
        role_lo = min(values)
        role_hi = max(values)
        if first:
            lo = role_lo
            hi = role_hi
            first = False
        else:
            lo = min(lo, role_lo)
            hi = max(hi, role_hi)
    if first:
        return (0.0, 1.0)
    return (lo, hi)


def _global_center(runtime: _LayoutRuntime) -> tuple[float, float, float]:
    if not runtime.axis_x:
        return (0.0, 0.0, 0.0)
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    for role in ROLE_NAMES:
        world_x, world_y, world_z = _world_xyz(runtime, role)
        xs.extend(world_x)
        ys.extend(world_y)
        zs.extend(world_z)
    return (
        (min(xs) + max(xs)) * 0.5,
        (min(ys) + max(ys)) * 0.5,
        (min(zs) + max(zs)) * 0.5,
    )


def _global_radius_extent(runtime: _LayoutRuntime, center: tuple[float, float, float]) -> float:
    max_distance = 1.0
    cx, cy, cz = center
    for role in ROLE_NAMES:
        world_x, world_y, world_z = _world_xyz(runtime, role)
        for px, py, pz in zip(world_x, world_y, world_z):
            max_distance = max(
                max_distance,
                math.sqrt((px - cx) ** 2 + (py - cy) ** 2 + (pz - cz) ** 2),
            )
    return max_distance


def _center_suit_ring_center(runtime: _LayoutRuntime) -> tuple[float, float, float]:
    ring_section_names = group_sections("ring")
    world_x, world_y, world_z = _world_xyz(runtime, "B")
    ring_indices: list[int] = []
    for section_name in ring_section_names:
        ring_indices.extend(runtime.section_indices.get(section_name, ()))
    if not ring_indices:
        return _global_center(runtime)
    count = float(len(ring_indices))
    return (
        sum(world_x[index] for index in ring_indices) / count,
        sum(world_y[index] for index in ring_indices) / count,
        sum(world_z[index] for index in ring_indices) / count,
    )


def _curve_value(clip: ShowClip, key: str, clip_t: float) -> float | None:
    raw_text = str(clip.params.get("curve_text", "")).strip()
    if not raw_text:
        return None
    points = _parse_curve_text(raw_text).get(key)
    if not points:
        return None
    return _sample_curve(points, clip_t)


def _param_float(clip: ShowClip, key: str, default: float, clip_t: float) -> float:
    curve_override = _curve_value(clip, key, clip_t)
    if curve_override is not None:
        return float(curve_override)
    try:
        return float(clip.params.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _param_int(clip: ShowClip, key: str, default: int, clip_t: float) -> int:
    return int(round(_param_float(clip, key, float(default), clip_t)))


def _animated_base_color(clip: ShowClip, clip_t: float) -> Color:
    del clip_t
    return _rgba_from_params(clip.params)


def _color_mode(clip: ShowClip) -> str:
    value = str(clip.params.get("color_mode", "hold")).strip().lower()
    return value if value in {"hold", "linear", "smooth", "cycle"} else "hold"


def _color_from(clip: ShowClip) -> Color:
    return _rgba_from_params(clip.params, key="color_from", default=_color_to_rgba8_default("color"))


def _color_to(clip: ShowClip) -> Color:
    default = tuple(
        int(channel * 255.0)
        for channel in _rgba_from_params(clip.params, key="color_from", default=_color_to_rgba8_default("color"))
    )
    return _rgba_from_params(clip.params, key="color_to", default=default)


def _color_to_rgba8_default(key: str) -> tuple[int, int, int, int]:
    del key
    return (255, 255, 255, 255)


def _color_rate(clip: ShowClip) -> float:
    try:
        return max(0.0, float(clip.params.get("color_rate", 1.0)))
    except (TypeError, ValueError):
        return 1.0


def _color_tempo_sync_enabled(clip: ShowClip) -> bool:
    if "color_tempo_sync" in clip.params:
        return bool(clip.params.get("color_tempo_sync", True))
    return True


def _resolved_color_palette_text(project: ShowProject, clip: ShowClip) -> str:
    preset_name = str(clip.params.get("color_palette_preset", "")).strip()
    custom_text = str(clip.params.get("palette_text", "")).strip()
    return resolve_palette_text(project.palettes, preset_name, custom_text)


def _smoothstep01(value: float) -> float:
    value = _clamp01(value)
    return value * value * (3.0 - 2.0 * value)


def _color_cycle_t(project: ShowProject, clip: ShowClip, time_ms: int, clip_t: float) -> float:
    rate = _color_rate(clip)
    if _color_tempo_sync_enabled(clip):
        if project.tempo_bpm <= 0.0:
            return clip_t
        beat_ms = 60000.0 / project.tempo_bpm
        beat_position = (time_ms - project.beat_offset_ms) / beat_ms
        return (beat_position * max(0.0625, rate)) % 1.0
    elapsed_sec = max(0.0, (time_ms - clip.start_ms) / 1000.0)
    return (elapsed_sec * rate) % 1.0 if rate > 0.0 else clip_t


def clip_color_at(project: ShowProject, clip: ShowClip, time_ms: int, clip_t: float | None = None) -> Color:
    if clip_t is None:
        clip_t = _clip_t(clip, time_ms)
    mode = _color_mode(clip)
    color_from = _rgba_from_params(
        clip.params,
        key="color_from",
        default=tuple(int(channel * 255.0) for channel in _rgba_from_params(clip.params, key="color")),
    )
    color_to = _rgba_from_params(
        clip.params,
        key="color_to",
        default=tuple(int(channel * 255.0) for channel in color_from),
    )
    if mode == "linear":
        t = _clamp01(clip_t)
        return tuple(_clamp01(_lerp(color_from[index], color_to[index], t)) for index in range(4))  # type: ignore[return-value]
    if mode == "smooth":
        t = _smoothstep01(clip_t)
        return tuple(_clamp01(_lerp(color_from[index], color_to[index], t)) for index in range(4))  # type: ignore[return-value]
    if mode == "cycle":
        palette_text = _resolved_color_palette_text(project, clip)
        palette_stops = _parse_palette_text(palette_text)
        cycle_t = _color_cycle_t(project, clip, time_ms, clip_t)
        palette_color = _sample_palette(palette_stops, cycle_t) if palette_stops else None
        if palette_color is not None:
            return palette_color
        return tuple(_clamp01(_lerp(color_from[index], color_to[index], cycle_t)) for index in range(4))  # type: ignore[return-value]
    return color_from


def clip_color_preview_rgba8(project: ShowProject, clip: ShowClip, sample_count: int = 7) -> tuple[tuple[int, int, int, int], ...]:
    sample_count = max(1, int(sample_count))
    if clip.end_ms <= clip.start_ms:
        return (_color_to_rgba8(clip_color_at(project, clip, clip.start_ms, 0.0)),)
    samples: list[tuple[int, int, int, int]] = []
    duration_ms = clip.end_ms - clip.start_ms
    for index in range(sample_count):
        if sample_count == 1:
            sample_t = 0.5
        else:
            sample_t = index / (sample_count - 1)
        time_ms = clip.start_ms + int(round(duration_ms * sample_t))
        samples.append(_color_to_rgba8(clip_color_at(project, clip, time_ms, sample_t)))
    return tuple(samples)


def _travel_margin(core_half_width: float, softness: float) -> float:
    return max(0.0, core_half_width + softness)


def _target_indices(runtime: _LayoutRuntime, clip: ShowClip) -> tuple[int, ...]:
    if clip.target_kind == "all":
        return runtime.all_indices
    if clip.target_kind == "section":
        indices: list[int] = []
        for section_name in (
            name.strip()
            for name in str(clip.target).split(",")
            if name.strip()
        ):
            indices.extend(runtime.section_indices.get(section_name, ()))
        return tuple(indices)
    if clip.target_kind == "group":
        indices: list[int] = []
        for group_name in (
            name.strip()
            for name in str(clip.target).split(",")
            if name.strip()
        ):
            try:
                sections = group_sections(group_name)
            except KeyError:
                continue
            for section_name in sections:
                indices.extend(runtime.section_indices.get(section_name, ()))
        return tuple(indices)
    return ()


def _blend_into(
    mode: str,
    index: int,
    src_r: float,
    src_g: float,
    src_b: float,
    src_a: float,
    dst_r: list[float],
    dst_g: list[float],
    dst_b: list[float],
    dst_a: list[float],
) -> None:
    if mode == "replace":
        dst_r[index] = src_r
        dst_g[index] = src_g
        dst_b[index] = src_b
        dst_a[index] = src_a
        return
    if mode == "add":
        alpha = _clamp01(src_a)
        dst_r[index] = _clamp01(dst_r[index] + src_r * alpha)
        dst_g[index] = _clamp01(dst_g[index] + src_g * alpha)
        dst_b[index] = _clamp01(dst_b[index] + src_b * alpha)
        dst_a[index] = _clamp01(max(dst_a[index], alpha))
        return
    if mode == "max":
        alpha = _clamp01(src_a)
        dst_r[index] = max(dst_r[index], src_r * alpha)
        dst_g[index] = max(dst_g[index], src_g * alpha)
        dst_b[index] = max(dst_b[index], src_b * alpha)
        dst_a[index] = max(dst_a[index], alpha)
        return
    alpha = _clamp01(src_a)
    inv = 1.0 - alpha
    dst_r[index] = _clamp01(src_r * alpha + dst_r[index] * inv)
    dst_g[index] = _clamp01(src_g * alpha + dst_g[index] * inv)
    dst_b[index] = _clamp01(src_b * alpha + dst_b[index] * inv)
    dst_a[index] = _clamp01(alpha + dst_a[index] * inv)


def _role_clips(project: ShowProject, role: str) -> list[ShowClip]:
    if role not in ROLE_NAMES:
        raise ValueError(f"unknown role: {role}")
    clips = list(project.global_clips)
    clips.extend(project.role_clips.get(role, []))
    clips.sort(key=lambda clip: (clip.layer, clip.start_ms, clip.end_ms))
    return clips


def _build_preview_frame(
    project: ShowProject,
    role: str,
    time_ms: int,
    runtime: _LayoutRuntime,
) -> PreviewFrame:
    clips = [clip for clip in _role_clips(project, role) if _clip_active(clip, time_ms)]
    led_count = len(runtime.points)
    color_r = [0.0] * led_count
    color_g = [0.0] * led_count
    color_b = [0.0] * led_count
    color_a = [0.0] * led_count

    for clip in clips:
        target_indices = _target_indices(runtime, clip)
        if not target_indices:
            continue

        blend_mode = clip.blend
        clip_t = _clip_t(clip, time_ms)
        base_r, base_g, base_b, base_a = clip_color_at(project, clip, time_ms, clip_t)
        intensity = _param_float(clip, "intensity", 1.0, clip_t)
        effect = clip.effect

        if effect in {"blink", "strobe"}:
            frequency_hz = _param_float(clip, "frequency_hz", 2.0 if effect == "blink" else 8.0, clip_t)
            duty_cycle = _clamp01(_param_float(clip, "duty_cycle", 0.5, clip_t))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            on = 1.0 if _effect_phase(project, clip, time_ms, frequency_hz, phase) < duty_cycle else 0.0
            scale = intensity * on
            src_r = _clamp01(base_r * scale)
            src_g = _clamp01(base_g * scale)
            src_b = _clamp01(base_b * scale)
            src_a = _clamp01(base_a * scale)
            for index in target_indices:
                _blend_into(blend_mode, index, src_r, src_g, src_b, src_a, color_r, color_g, color_b, color_a)
            continue

        if effect == "pulse":
            frequency_hz = _param_float(clip, "frequency_hz", 1.0, clip_t)
            phase = _param_float(clip, "phase", 0.0, clip_t)
            min_intensity = _param_float(clip, "min_intensity", 0.15, clip_t)
            max_intensity = _param_float(clip, "max_intensity", 1.0, clip_t)
            wave = 0.5 - 0.5 * math.cos(2.0 * math.pi * _effect_phase(project, clip, time_ms, frequency_hz, phase))
            amount = _lerp(min_intensity, max_intensity, wave)
            scale = intensity * amount
            src_r = _clamp01(base_r * scale)
            src_g = _clamp01(base_g * scale)
            src_b = _clamp01(base_b * scale)
            src_a = _clamp01(base_a * scale)
            for index in target_indices:
                _blend_into(blend_mode, index, src_r, src_g, src_b, src_a, color_r, color_g, color_b, color_a)
            continue

        if effect == "fade":
            target_r, target_g, target_b, target_a = _rgba_from_params(
                clip.params,
                key="to_color",
                default=(0, 0, 0, 255),
            )
            src_r = _clamp01(_lerp(base_r, target_r, clip_t) * intensity)
            src_g = _clamp01(_lerp(base_g, target_g, clip_t) * intensity)
            src_b = _clamp01(_lerp(base_b, target_b, clip_t) * intensity)
            src_a = _clamp01(_lerp(base_a, target_a, clip_t) * intensity)
            for index in target_indices:
                _blend_into(blend_mode, index, src_r, src_g, src_b, src_a, color_r, color_g, color_b, color_a)
            continue

        if effect == "gradient":
            target_r, target_g, target_b, target_a = _rgba_from_params(
                clip.params,
                key="to_color",
                default=(255, 255, 255, 255),
            )
            axis = str(clip.params.get("axis", "y"))
            axis_values = _axis_values(runtime, axis, clip, cycle_index=0)
            lo, hi = _axis_bounds(runtime, target_indices, axis, clip, cycle_index=0)
            inv_range = 0.0 if axis == "section_u" or abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            for index in target_indices:
                if axis == "section_u":
                    u = runtime.section_us[index]
                elif inv_range == 0.0:
                    u = 0.5
                else:
                    u = (axis_values[index] - lo) * inv_range
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(_lerp(base_r, target_r, u) * intensity),
                    _clamp01(_lerp(base_g, target_g, u) * intensity),
                    _clamp01(_lerp(base_b, target_b, u) * intensity),
                    _clamp01(_lerp(base_a, target_a, u) * intensity),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "palette_cycle":
            frequency_hz = _param_float(clip, "frequency_hz", 1.0, clip_t)
            phase = _param_float(clip, "phase", 0.0, clip_t)
            cycle_t = _effect_phase(project, clip, time_ms, frequency_hz, phase)
            target_r, target_g, target_b, target_a = _rgba_from_params(
                clip.params,
                key="to_color",
                default=(255, 255, 255, 255),
            )
            palette_text = str(clip.params.get("palette_text", "")).strip()
            palette_color = _sample_palette(_parse_palette_text(palette_text), cycle_t) if palette_text else None
            if palette_color is not None:
                src_r, src_g, src_b, src_a = palette_color
            else:
                src_r = _lerp(base_r, target_r, cycle_t)
                src_g = _lerp(base_g, target_g, cycle_t)
                src_b = _lerp(base_b, target_b, cycle_t)
                src_a = _lerp(base_a, target_a, cycle_t)
            src_r = _clamp01(src_r * intensity)
            src_g = _clamp01(src_g * intensity)
            src_b = _clamp01(src_b * intensity)
            src_a = _clamp01(src_a * intensity)
            for index in target_indices:
                _blend_into(blend_mode, index, src_r, src_g, src_b, src_a, color_r, color_g, color_b, color_a)
            continue

        if effect in {"sweep", "mirror_sweep"}:
            axis = str(clip.params.get("axis", "y"))
            width = max(0.02, _param_float(clip, "width", 0.22, clip_t))
            softness = max(0.001, _param_float(clip, "softness", width * 0.75, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            core_half_width = width * 0.5
            travel_margin = _travel_margin(core_half_width, softness)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
                cycle_index = int(math.floor(cycle_position))
            else:
                travel_t = clip_t
                cycle_index = 0
            if reverse:
                travel_t = 1.0 - travel_t
            axis_values = _axis_values(runtime, axis, clip, cycle_index=cycle_index)
            lo, hi = _axis_bounds(runtime, target_indices, axis, clip, cycle_index=cycle_index)
            inv_range = 0.0 if axis == "section_u" or abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            head = _lerp(-travel_margin, 1.0 + travel_margin, travel_t)
            for index in target_indices:
                if axis == "section_u":
                    pos = runtime.section_us[index]
                elif inv_range == 0.0:
                    pos = 0.5
                else:
                    pos = (axis_values[index] - lo) * inv_range
                if effect == "mirror_sweep":
                    distance = min(abs(pos - head), abs((1.0 - pos) - head))
                else:
                    distance = abs(pos - head)
                if distance <= core_half_width:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "fanout":
            width = max(0.02, _param_float(clip, "width", 0.24, clip_t))
            softness = max(0.001, _param_float(clip, "softness", width * 0.75, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
            else:
                frequency_hz = _param_float(clip, "frequency_hz", 1.0, clip_t)
                travel_t = _time_phase(clip, time_ms, frequency_hz, phase)
            if reverse:
                travel_t = 1.0 - travel_t
            core_half_width = width * 0.5
            travel_margin = _travel_margin(core_half_width, softness)
            head = _lerp(-travel_margin, 1.0 + travel_margin, travel_t)
            for index in target_indices:
                pos = abs(runtime.section_us[index] - 0.5) * 2.0
                distance = abs(pos - head)
                if distance <= core_half_width:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "chase":
            axis = str(clip.params.get("axis", "y"))
            repeats = max(1, _param_int(clip, "repeats", 4, clip_t))
            width = max(0.02, _param_float(clip, "width", 0.18, clip_t))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
                cycle_index = int(math.floor(cycle_position))
            else:
                frequency_hz = _param_float(clip, "frequency_hz", 1.0, clip_t)
                cycle_position = _time_cycle_position(clip, time_ms, frequency_hz, phase)
                travel_t = cycle_position % 1.0 if frequency_hz > 0.0 else clip_t
                cycle_index = int(math.floor(cycle_position)) if frequency_hz > 0.0 else 0
            axis_values = _axis_values(runtime, axis, clip, cycle_index=cycle_index)
            lo, hi = _axis_bounds(runtime, target_indices, axis, clip, cycle_index=cycle_index)
            inv_range = 0.0 if axis == "section_u" or abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            for index in target_indices:
                if axis == "section_u":
                    pos = runtime.section_us[index]
                elif inv_range == 0.0:
                    pos = 0.5
                else:
                    pos = (axis_values[index] - lo) * inv_range
                local = (pos * repeats + travel_t * repeats) % 1.0
                distance = min(local, 1.0 - local)
                falloff = _clamp01(1.0 - distance / width)
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "sparkle":
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                sparkle_t = cycle_position % 1.0
                cycle_index = int(math.floor(cycle_position))
            else:
                frequency_hz = _param_float(clip, "frequency_hz", 6.0, clip_t)
                cycle_position = _time_cycle_position(clip, time_ms, frequency_hz, phase)
                sparkle_t = cycle_position % 1.0 if frequency_hz > 0.0 else clip_t
                cycle_index = int(math.floor(cycle_position)) if frequency_hz > 0.0 else 0
            for index in target_indices:
                phase_offset = _hash_unit("sparkle_phase", clip.effect, role, index, cycle_index)
                amplitude = 0.35 + 0.65 * _hash_unit("sparkle_amp", clip.effect, role, index, cycle_index)
                local_t = (sparkle_t + phase_offset) % 1.0
                sparkle = max(0.0, 1.0 - abs(local_t * 2.0 - 1.0))
                sparkle = sparkle * sparkle * amplitude
                scale = intensity * sparkle
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "global_plane_sweep":
            axis = str(clip.params.get("axis", "x"))
            width = max(0.02, _param_float(clip, "width", 0.22, clip_t))
            softness = max(0.001, _param_float(clip, "softness", width * 0.75, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
                cycle_index = int(math.floor(cycle_position))
            else:
                travel_t = clip_t
                cycle_index = 0
            if reverse:
                travel_t = 1.0 - travel_t
            axis_values = _world_axis_values(runtime, role, axis, clip, cycle_index)
            lo, hi = _global_axis_bounds(runtime, axis, clip, cycle_index)
            inv_range = 0.0 if abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            core_half_width = width * 0.5
            travel_margin = _travel_margin(core_half_width, softness)
            head = _lerp(-travel_margin, 1.0 + travel_margin, travel_t)
            for index in target_indices:
                pos = 0.5 if inv_range == 0.0 else (axis_values[index] - lo) * inv_range
                distance = abs(pos - head)
                if distance <= core_half_width:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "radialray":
            width = max(0.02, _param_float(clip, "width", 0.10, clip_t))
            softness = max(0.001, _param_float(clip, "softness", width * 0.75, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
            else:
                frequency_hz = _param_float(clip, "frequency_hz", 1.0, clip_t)
                travel_t = _time_phase(clip, time_ms, frequency_hz, phase)
            if reverse:
                travel_t = 1.0 - travel_t
            center = _center_suit_ring_center(runtime)
            core_half_width = width * 0.5
            world_x, world_y, _ = _world_xyz(runtime, role)
            for index in target_indices:
                dx = world_x[index] - center[0]
                dy = world_y[index] - center[1]
                angle = (math.atan2(dx, -dy) / (math.pi * 2.0)) % 1.0
                distance = abs(((angle - travel_t + 0.5) % 1.0) - 0.5)
                if distance <= core_half_width:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "traveling_orb":
            axis = str(clip.params.get("axis", "x"))
            width = max(0.02, _param_float(clip, "width", 0.20, clip_t))
            softness = max(0.001, _param_float(clip, "softness", width * 0.75, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
                cycle_index = int(math.floor(cycle_position))
            else:
                travel_t = clip_t
                cycle_index = 0
            if reverse:
                travel_t = 1.0 - travel_t
            lo, hi = _global_axis_bounds(runtime, axis, clip, cycle_index)
            span = max(1.0, hi - lo)
            radius = span * width * 0.5
            softness_world = span * softness * 0.5
            travel_margin = radius + softness_world * 0.5
            head = _lerp(lo - travel_margin, hi + travel_margin, travel_t)
            if axis == "random_xy":
                angle = _axis_random_angle(clip, cycle_index)
                direction = (math.cos(angle), math.sin(angle), 0.0)
            elif axis == "y":
                direction = (0.0, 1.0, 0.0)
            elif axis == "z":
                direction = (0.0, 0.0, 1.0)
            else:
                direction = (1.0, 0.0, 0.0)
            center = _global_center(runtime)
            center_proj = center[0] * direction[0] + center[1] * direction[1] + center[2] * direction[2]
            orb_center = (
                center[0] + direction[0] * (head - center_proj),
                center[1] + direction[1] * (head - center_proj),
                center[2] + direction[2] * (head - center_proj),
            )
            world_x, world_y, world_z = _world_xyz(runtime, role)
            for index in target_indices:
                distance = math.sqrt(
                    (world_x[index] - orb_center[0]) ** 2 +
                    (world_y[index] - orb_center[1]) ** 2 +
                    (world_z[index] - orb_center[2]) ** 2
                )
                if distance <= radius:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (distance - radius) / max(1e-6, softness_world))
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "ring_burst":
            width = max(0.02, _param_float(clip, "width", 0.16, clip_t))
            softness = max(0.001, _param_float(clip, "softness", width * 0.75, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
            else:
                travel_t = clip_t
            if reverse:
                travel_t = 1.0 - travel_t
            center = _global_center(runtime)
            max_radius = _global_radius_extent(runtime, center)
            shell_radius = _lerp(0.0, max_radius + width * max_radius, travel_t)
            shell_half_width = max_radius * width * 0.5
            shell_softness = max_radius * softness * 0.5
            world_x, world_y, world_z = _world_xyz(runtime, role)
            for index in target_indices:
                distance = math.sqrt(
                    (world_x[index] - center[0]) ** 2 +
                    (world_y[index] - center[1]) ** 2 +
                    (world_z[index] - center[2]) ** 2
                )
                delta = abs(distance - shell_radius)
                if delta <= shell_half_width:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (delta - shell_half_width) / max(1e-6, shell_softness))
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        if effect == "ground_energy":
            softness = max(0.001, _param_float(clip, "softness", 0.14, clip_t))
            reverse = bool(clip.params.get("reverse", False))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            if _tempo_sync_enabled(clip):
                cycle_position = _beat_cycle_position(project, clip, time_ms, _cycles_per_beat(clip), phase)
                travel_t = cycle_position % 1.0
            else:
                travel_t = clip_t
            if reverse:
                travel_t = 1.0 - travel_t
            global_y_lo, global_y_hi = _global_axis_bounds(runtime, "y")
            edge_softness = max(1e-6, (global_y_hi - global_y_lo) * softness)
            level = _lerp(global_y_hi + edge_softness, global_y_lo, travel_t)
            _, world_y, _ = _world_xyz(runtime, role)
            for index in target_indices:
                if world_y[index] >= level:
                    falloff = 1.0
                else:
                    falloff = _clamp01(1.0 - (level - world_y[index]) / edge_softness)
                scale = intensity * falloff
                _blend_into(
                    blend_mode,
                    index,
                    _clamp01(base_r * scale),
                    _clamp01(base_g * scale),
                    _clamp01(base_b * scale),
                    _clamp01(base_a * scale),
                    color_r,
                    color_g,
                    color_b,
                    color_a,
                )
            continue

        src_r = _clamp01(base_r * intensity)
        src_g = _clamp01(base_g * intensity)
        src_b = _clamp01(base_b * intensity)
        src_a = _clamp01(base_a * intensity)
        for index in target_indices:
            _blend_into(blend_mode, index, src_r, src_g, src_b, src_a, color_r, color_g, color_b, color_a)

    return PreviewFrame(
        role=role,
        time_ms=time_ms,
        section_names=runtime.section_names,
        points=runtime.points,
        section_us=runtime.section_us,
        colors_rgba8=tuple(
            _color_to_rgba8((color_r[index], color_g[index], color_b[index], color_a[index]))
            for index in range(led_count)
        ),
    )


def build_preview_frame(project: ShowProject, layout: LoadedLayout, role: str, time_ms: int) -> PreviewFrame:
    runtime = _layout_runtime(tuple(layout.sections))
    return _build_preview_frame(project, role, time_ms, runtime)


def build_preview_frames(
    project: ShowProject,
    layout: LoadedLayout,
    roles: tuple[str, ...],
    time_ms: int,
) -> dict[str, PreviewFrame]:
    runtime = _layout_runtime(tuple(layout.sections))
    return {
        role: _build_preview_frame(project, role, time_ms, runtime)
        for role in roles
    }
