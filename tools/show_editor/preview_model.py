from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
import zlib

from tools.layout_model import LayoutSection, sample_section_points
from tools.spatial_display import display_point_3d

from .color_palettes import BUILTIN_COLOR_PALETTES, parse_palette_text_even, resolve_palette_text
from .canonical_layout import GROUP_SECTIONS, ROLE_NAMES, group_sections
from .layout_io import LoadedLayout
from .project_model import ShowClip, ShowProject


Color = tuple[float, float, float, float]
ROLE_INDEX = {role: index for index, role in enumerate(ROLE_NAMES)}


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
    group_indices: dict[str, tuple[int, ...]]
    all_indices: tuple[int, ...]
    world_x_by_role: tuple[tuple[float, ...], ...]
    world_y: tuple[float, ...]
    world_z: tuple[float, ...]
    global_x_bounds: tuple[float, float]
    global_y_bounds: tuple[float, float]
    global_z_bounds: tuple[float, float]
    global_center: tuple[float, float, float]
    global_radius_extent: float
    center_suit_ring_center: tuple[float, float, float]


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

    axis_x_tuple = tuple(axis_x)
    axis_y_tuple = tuple(axis_y)
    axis_z_tuple = tuple(axis_z)
    section_us_tuple = tuple(section_us)
    if axis_x_tuple:
        span_x = max(axis_x_tuple) - min(axis_x_tuple)
        spacing_x = max(1.0, span_x * 1.5)
        center_index = (len(ROLE_NAMES) - 1) * 0.5
        world_x_by_role = tuple(
            tuple(value + (role_index - center_index) * spacing_x for value in axis_x_tuple)
            for role_index, _role in enumerate(ROLE_NAMES)
        )
        global_x_bounds = (
            min(min(values) for values in world_x_by_role),
            max(max(values) for values in world_x_by_role),
        )
        global_y_bounds = (min(axis_y_tuple), max(axis_y_tuple))
        global_z_bounds = (min(axis_z_tuple), max(axis_z_tuple))
        global_center = (
            (global_x_bounds[0] + global_x_bounds[1]) * 0.5,
            (global_y_bounds[0] + global_y_bounds[1]) * 0.5,
            (global_z_bounds[0] + global_z_bounds[1]) * 0.5,
        )
        global_radius_extent = 1.0
        for role_index, _role in enumerate(ROLE_NAMES):
            world_x = world_x_by_role[role_index]
            for px, py, pz in zip(world_x, axis_y_tuple, axis_z_tuple):
                global_radius_extent = max(
                    global_radius_extent,
                    math.sqrt(
                        (px - global_center[0]) ** 2 +
                        (py - global_center[1]) ** 2 +
                        (pz - global_center[2]) ** 2
                    ),
                )
    else:
        world_x_by_role = tuple(() for _ in ROLE_NAMES)
        global_x_bounds = (0.0, 1.0)
        global_y_bounds = (0.0, 1.0)
        global_z_bounds = (0.0, 1.0)
        global_center = (0.0, 0.0, 0.0)
        global_radius_extent = 1.0

    group_indices: dict[str, tuple[int, ...]] = {}
    for group_name, section_names_in_group in GROUP_SECTIONS.items():
        indices: list[int] = []
        for section_name in section_names_in_group:
            indices.extend(section_indices.get(section_name, ()))
        group_indices[group_name] = tuple(indices)

    ring_indices = group_indices.get("ring", ())
    if ring_indices and world_x_by_role and ROLE_INDEX.get("B", 0) < len(world_x_by_role):
        world_x = world_x_by_role[ROLE_INDEX["B"]]
        ring_count = float(len(ring_indices))
        center_suit_ring_center = (
            sum(world_x[index] for index in ring_indices) / ring_count,
            sum(axis_y_tuple[index] for index in ring_indices) / ring_count,
            sum(axis_z_tuple[index] for index in ring_indices) / ring_count,
        )
    else:
        center_suit_ring_center = global_center

    return _LayoutRuntime(
        section_names=tuple(section_names),
        points=tuple(points),
        section_us=section_us_tuple,
        axis_x=axis_x_tuple,
        axis_y=axis_y_tuple,
        axis_z=axis_z_tuple,
        axis_radial=tuple(axis_radial),
        section_indices=section_indices,
        group_indices=group_indices,
        all_indices=tuple(range(len(points))),
        world_x_by_role=world_x_by_role,
        world_y=axis_y_tuple,
        world_z=axis_z_tuple,
        global_x_bounds=global_x_bounds,
        global_y_bounds=global_y_bounds,
        global_z_bounds=global_z_bounds,
        global_center=global_center,
        global_radius_extent=global_radius_extent,
        center_suit_ring_center=center_suit_ring_center,
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
    seed = _clip_seed(clip)
    value = _hash_u32_fast(seed, 0, 0, max(0, int(cycle_index)), 0x41C6CE57)
    return (value / 4294967295.0) * (math.pi * 2.0)


def _clip_seed(clip: ShowClip) -> int:
    payload = (
        f"{clip.effect}|{clip.target_kind}|{clip.target}|"
        f"{clip.layer}|{clip.start_ms}|{clip.end_ms}"
    ).encode("utf-8")
    # The v1 runtime record stores a 16-bit deterministic seed.
    return zlib.crc32(payload) & 0xFFFF


def _hash_u32_fast(seed: int, role_index: int, led_index: int, cycle_index: int, salt: int) -> int:
    value = (
        seed ^
        ((role_index + 1) * 0x9E3779B9) ^
        ((led_index + 1) * 0x85EBCA6B) ^
        ((cycle_index + 1) * 0xC2B2AE35) ^
        salt
    ) & 0xFFFFFFFF
    value ^= value >> 16
    value = (value * 0x7FEB352D) & 0xFFFFFFFF
    value ^= value >> 15
    value = (value * 0x846CA68B) & 0xFFFFFFFF
    value ^= value >> 16
    return value


def _hash_unit_fast(seed: int, role_index: int, led_index: int, cycle_index: int, salt: int) -> float:
    return _hash_u32_fast(seed, role_index, led_index, cycle_index, salt) / 4294967295.0


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


def _world_xyz(runtime: _LayoutRuntime, role: str) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    role_index = ROLE_INDEX.get(role)
    if role_index is None or role_index >= len(runtime.world_x_by_role):
        return ((), (), ())
    return (runtime.world_x_by_role[role_index], runtime.world_y, runtime.world_z)


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


def _random_xy_components(clip: ShowClip | None, cycle_index: int = 0) -> tuple[float, float]:
    angle = _axis_random_angle(clip, cycle_index)
    return (math.cos(angle), math.sin(angle))


def _axis_bounds_random_xy(
    axis_x: tuple[float, ...],
    axis_y: tuple[float, ...],
    indices: tuple[int, ...],
    axis_cos: float,
    axis_sin: float,
) -> tuple[float, float]:
    if not indices:
        return (0.0, 1.0)
    first_index = indices[0]
    lo = axis_x[first_index] * axis_cos + axis_y[first_index] * axis_sin
    hi = lo
    for index in indices[1:]:
        value = axis_x[index] * axis_cos + axis_y[index] * axis_sin
        if value < lo:
            lo = value
        elif value > hi:
            hi = value
    return (lo, hi)


def _global_bounds_random_xy(
    runtime: _LayoutRuntime,
    axis_cos: float,
    axis_sin: float,
) -> tuple[float, float]:
    if not runtime.world_x_by_role or not runtime.world_y:
        return (0.0, 1.0)
    first = True
    lo = 0.0
    hi = 0.0
    for world_x in runtime.world_x_by_role:
        for px, py in zip(world_x, runtime.world_y):
            value = px * axis_cos + py * axis_sin
            if first:
                lo = value
                hi = value
                first = False
            elif value < lo:
                lo = value
            elif value > hi:
                hi = value
    if first:
        return (0.0, 1.0)
    return (lo, hi)


def _global_axis_bounds(
    runtime: _LayoutRuntime,
    axis: str,
    clip: ShowClip | None = None,
    cycle_index: int = 0,
) -> tuple[float, float]:
    if axis == "x":
        return runtime.global_x_bounds
    if axis == "y":
        return runtime.global_y_bounds
    if axis == "z":
        return runtime.global_z_bounds
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
    return runtime.global_center


def _traveling_orb_cross_x(runtime: _LayoutRuntime, clip: ShowClip, cycle_index: int) -> float:
    lo, hi = runtime.global_x_bounds
    if abs(hi - lo) < 1e-6:
        return (lo + hi) * 0.5
    seed = _clip_seed(clip)
    unit = _hash_unit_fast(seed, 0, 0, cycle_index, 0x54A9D93B)
    return _lerp(lo, hi, unit)


def _global_radius_extent(runtime: _LayoutRuntime, center: tuple[float, float, float]) -> float:
    del center
    return runtime.global_radius_extent


def _center_suit_ring_center(runtime: _LayoutRuntime) -> tuple[float, float, float]:
    return runtime.center_suit_ring_center


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


def _color_fit_to_clip_enabled(clip: ShowClip) -> bool:
    return bool(clip.params.get("color_fit_to_clip", False))


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
    if _color_fit_to_clip_enabled(clip):
        return (_clamp01(clip_t) * max(0.0625, rate)) % 1.0
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
            indices.extend(runtime.group_indices.get(group_name, ()))
        return tuple(indices)
    return ()


def _blend_uniform(
    mode: str,
    target_indices: tuple[int, ...],
    src_r: float,
    src_g: float,
    src_b: float,
    src_a: float,
    dst_r: list[float],
    dst_g: list[float],
    dst_b: list[float],
    dst_a: list[float],
) -> None:
    src_r = _clamp01(src_r)
    src_g = _clamp01(src_g)
    src_b = _clamp01(src_b)
    src_a = _clamp01(src_a)
    if not target_indices or src_a <= 0.0:
        return
    if mode == "replace":
        for index in target_indices:
            dst_r[index] = src_r
            dst_g[index] = src_g
            dst_b[index] = src_b
            dst_a[index] = src_a
        return
    if mode == "add":
        premul_r = src_r * src_a
        premul_g = src_g * src_a
        premul_b = src_b * src_a
        for index in target_indices:
            dst_r[index] = _clamp01(dst_r[index] + premul_r)
            dst_g[index] = _clamp01(dst_g[index] + premul_g)
            dst_b[index] = _clamp01(dst_b[index] + premul_b)
            dst_a[index] = _clamp01(max(dst_a[index], src_a))
        return
    if mode == "max":
        premul_r = src_r * src_a
        premul_g = src_g * src_a
        premul_b = src_b * src_a
        for index in target_indices:
            dst_r[index] = max(dst_r[index], premul_r)
            dst_g[index] = max(dst_g[index], premul_g)
            dst_b[index] = max(dst_b[index], premul_b)
            dst_a[index] = max(dst_a[index], src_a)
        return
    inv = 1.0 - src_a
    for index in target_indices:
        dst_r[index] = _clamp01(src_r * src_a + dst_r[index] * inv)
        dst_g[index] = _clamp01(src_g * src_a + dst_g[index] * inv)
        dst_b[index] = _clamp01(src_b * src_a + dst_b[index] * inv)
        dst_a[index] = _clamp01(src_a + dst_a[index] * inv)


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


def _sorted_role_clip_map(project: ShowProject) -> dict[str, tuple[ShowClip, ...]]:
    def clip_key(clip: ShowClip) -> tuple[int, int, int]:
        return (clip.layer, clip.start_ms, clip.end_ms)

    global_clips = tuple(sorted(project.global_clips, key=clip_key))
    role_map: dict[str, tuple[ShowClip, ...]] = {}
    for role in ROLE_NAMES:
        role_clips = tuple(sorted(project.role_clips.get(role, ()), key=clip_key))
        clips = list(global_clips)
        clips.extend(role_clips)
        clips.sort(key=clip_key)
        role_map[role] = tuple(clips)
    return role_map


def _build_preview_frame(
    project: ShowProject,
    role: str,
    time_ms: int,
    runtime: _LayoutRuntime,
    role_clips: tuple[ShowClip, ...] | None = None,
) -> PreviewFrame:
    clip_source = _role_clips(project, role) if role_clips is None else role_clips
    clips = [clip for clip in clip_source if _clip_active(clip, time_ms)]
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

        if effect == "blink":
            frequency_hz = _param_float(clip, "frequency_hz", 2.0, clip_t)
            duty_cycle = _clamp01(_param_float(clip, "duty_cycle", 0.5, clip_t))
            decay = _clamp01(_param_float(clip, "decay", 0.0, clip_t))
            phase = _param_float(clip, "phase", 0.0, clip_t)
            cycle_phase = _effect_phase(project, clip, time_ms, frequency_hz, phase) % 1.0
            if cycle_phase < duty_cycle:
                on = 1.0
            elif decay > 1e-6 and cycle_phase < min(1.0, duty_cycle + decay):
                on = _clamp01(1.0 - ((cycle_phase - duty_cycle) / decay))
            else:
                on = 0.0
            scale = intensity * on
            _blend_uniform(
                blend_mode,
                target_indices,
                base_r * scale,
                base_g * scale,
                base_b * scale,
                base_a * scale,
                color_r,
                color_g,
                color_b,
                color_a,
            )
            continue

        if effect == "pulse":
            frequency_hz = _param_float(clip, "frequency_hz", 1.0, clip_t)
            phase = _param_float(clip, "phase", 0.0, clip_t)
            min_intensity = _param_float(clip, "min_intensity", 0.15, clip_t)
            max_intensity = _param_float(clip, "max_intensity", 1.0, clip_t)
            wave = 0.5 - 0.5 * math.cos(2.0 * math.pi * _effect_phase(project, clip, time_ms, frequency_hz, phase))
            amount = _lerp(min_intensity, max_intensity, wave)
            scale = intensity * amount
            _blend_uniform(
                blend_mode,
                target_indices,
                base_r * scale,
                base_g * scale,
                base_b * scale,
                base_a * scale,
                color_r,
                color_g,
                color_b,
                color_a,
            )
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
            axis_cos = 0.0
            axis_sin = 0.0
            if axis == "random_xy":
                axis_cos, axis_sin = _random_xy_components(clip, cycle_index)
                lo, hi = _axis_bounds_random_xy(runtime.axis_x, runtime.axis_y, target_indices, axis_cos, axis_sin)
                axis_values = ()
            else:
                axis_values = _axis_values(runtime, axis, clip, cycle_index=cycle_index)
                lo, hi = _axis_bounds(runtime, target_indices, axis, clip, cycle_index=cycle_index)
            inv_range = 0.0 if axis == "section_u" or abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            head = _lerp(-travel_margin, 1.0 + travel_margin, travel_t)
            if blend_mode == "max":
                for index in target_indices:
                    if axis == "section_u":
                        pos = runtime.section_us[index]
                    elif axis == "random_xy":
                        pos = 0.5 if inv_range == 0.0 else ((runtime.axis_x[index] * axis_cos + runtime.axis_y[index] * axis_sin) - lo) * inv_range
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
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
            for index in target_indices:
                if axis == "section_u":
                    pos = runtime.section_us[index]
                elif axis == "random_xy":
                    pos = 0.5 if inv_range == 0.0 else ((runtime.axis_x[index] * axis_cos + runtime.axis_y[index] * axis_sin) - lo) * inv_range
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
            if blend_mode == "max":
                for index in target_indices:
                    pos = abs(runtime.section_us[index] - 0.5) * 2.0
                    distance = abs(pos - head)
                    if distance <= core_half_width:
                        falloff = 1.0
                    else:
                        falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
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
            axis_cos = 0.0
            axis_sin = 0.0
            if axis == "random_xy":
                axis_cos, axis_sin = _random_xy_components(clip, cycle_index)
                lo, hi = _axis_bounds_random_xy(runtime.axis_x, runtime.axis_y, target_indices, axis_cos, axis_sin)
                axis_values = ()
            else:
                axis_values = _axis_values(runtime, axis, clip, cycle_index=cycle_index)
                lo, hi = _axis_bounds(runtime, target_indices, axis, clip, cycle_index=cycle_index)
            inv_range = 0.0 if axis == "section_u" or abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            if blend_mode == "max":
                for index in target_indices:
                    if axis == "section_u":
                        pos = runtime.section_us[index]
                    elif axis == "random_xy":
                        pos = 0.5 if inv_range == 0.0 else ((runtime.axis_x[index] * axis_cos + runtime.axis_y[index] * axis_sin) - lo) * inv_range
                    elif inv_range == 0.0:
                        pos = 0.5
                    else:
                        pos = (axis_values[index] - lo) * inv_range
                    local = (pos * repeats + travel_t * repeats) % 1.0
                    distance = min(local, 1.0 - local)
                    falloff = _clamp01(1.0 - distance / width)
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
            for index in target_indices:
                if axis == "section_u":
                    pos = runtime.section_us[index]
                elif axis == "random_xy":
                    pos = 0.5 if inv_range == 0.0 else ((runtime.axis_x[index] * axis_cos + runtime.axis_y[index] * axis_sin) - lo) * inv_range
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
            role_index = ROLE_INDEX.get(role, 0)
            clip_seed = _clip_seed(clip)
            if blend_mode == "max":
                for index in target_indices:
                    hash_value = _hash_u32_fast(clip_seed, role_index, index, cycle_index, 0x5A17)
                    phase_offset = (hash_value & 0xFFFF) / 65535.0
                    amplitude = 0.35 + 0.65 * ((((hash_value >> 16) ^ hash_value) & 0xFFFF) / 65535.0)
                    local_t = (sparkle_t + phase_offset) % 1.0
                    sparkle = max(0.0, 1.0 - abs(local_t * 2.0 - 1.0))
                    sparkle = sparkle * sparkle * amplitude
                    src_r = _clamp01(base_r * intensity * sparkle)
                    src_g = _clamp01(base_g * intensity * sparkle)
                    src_b = _clamp01(base_b * intensity * sparkle)
                    src_a = _clamp01(base_a * intensity * sparkle)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
            for index in target_indices:
                hash_value = _hash_u32_fast(clip_seed, role_index, index, cycle_index, 0x5A17)
                phase_offset = (hash_value & 0xFFFF) / 65535.0
                amplitude = 0.35 + 0.65 * ((((hash_value >> 16) ^ hash_value) & 0xFFFF) / 65535.0)
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

        if effect == "global_sweep":
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
            world_x, world_y, _ = _world_xyz(runtime, role)
            axis_cos = 0.0
            axis_sin = 0.0
            if axis == "random_xy":
                axis_cos, axis_sin = _random_xy_components(clip, cycle_index)
                lo, hi = _global_bounds_random_xy(runtime, axis_cos, axis_sin)
                axis_values = ()
            else:
                axis_values = _world_axis_values(runtime, role, axis, clip, cycle_index)
                lo, hi = _global_axis_bounds(runtime, axis, clip, cycle_index)
            inv_range = 0.0 if abs(hi - lo) < 1e-6 else 1.0 / (hi - lo)
            core_half_width = width * 0.5
            travel_margin = _travel_margin(core_half_width, softness)
            head = _lerp(-travel_margin, 1.0 + travel_margin, travel_t)
            if blend_mode == "max":
                for index in target_indices:
                    if axis == "random_xy":
                        pos = 0.5 if inv_range == 0.0 else ((world_x[index] * axis_cos + world_y[index] * axis_sin) - lo) * inv_range
                    else:
                        pos = 0.5 if inv_range == 0.0 else (axis_values[index] - lo) * inv_range
                    distance = abs(pos - head)
                    if distance <= core_half_width:
                        falloff = 1.0
                    else:
                        falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
            for index in target_indices:
                if axis == "random_xy":
                    pos = 0.5 if inv_range == 0.0 else ((world_x[index] * axis_cos + world_y[index] * axis_sin) - lo) * inv_range
                else:
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
            if blend_mode == "max":
                for index in target_indices:
                    dx = world_x[index] - center[0]
                    dy = world_y[index] - center[1]
                    angle = (math.atan2(dx, -dy) / (math.pi * 2.0)) % 1.0
                    distance = abs(((angle - travel_t + 0.5) % 1.0) - 0.5)
                    if distance <= core_half_width:
                        falloff = 1.0
                    else:
                        falloff = _clamp01(1.0 - (distance - core_half_width) / softness)
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
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
            random_cross_x = bool(clip.params.get("random_cross_x", False))
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
            if axis == "random_xy":
                axis_cos, axis_sin = _random_xy_components(clip, cycle_index)
                lo, hi = _global_bounds_random_xy(runtime, axis_cos, axis_sin)
            else:
                lo, hi = _global_axis_bounds(runtime, axis, clip, cycle_index)
            span = max(1.0, hi - lo)
            radius = span * width * 0.5
            softness_world = span * softness * 0.5
            travel_margin = radius + softness_world * 0.5
            head = _lerp(lo - travel_margin, hi + travel_margin, travel_t)
            if axis == "random_xy":
                direction = (axis_cos, axis_sin, 0.0)
            elif axis == "y":
                direction = (0.0, 1.0, 0.0)
            elif axis == "z":
                direction = (0.0, 0.0, 1.0)
            else:
                direction = (1.0, 0.0, 0.0)
            center = _global_center(runtime)
            anchor = center
            if random_cross_x:
                anchor = (
                    _traveling_orb_cross_x(runtime, clip, cycle_index),
                    center[1],
                    center[2],
                )
            center_proj = anchor[0] * direction[0] + anchor[1] * direction[1] + anchor[2] * direction[2]
            orb_center = (
                anchor[0] + direction[0] * (head - center_proj),
                anchor[1] + direction[1] * (head - center_proj),
                anchor[2] + direction[2] * (head - center_proj),
            )
            world_x, world_y, world_z = _world_xyz(runtime, role)
            radius_sq = radius * radius
            outer_radius = radius + max(0.0, softness_world)
            outer_radius_sq = outer_radius * outer_radius
            if blend_mode == "max":
                for index in target_indices:
                    dx = world_x[index] - orb_center[0]
                    dy = world_y[index] - orb_center[1]
                    dz = world_z[index] - orb_center[2]
                    distance_sq = dx * dx + dy * dy + dz * dz
                    if distance_sq <= radius_sq:
                        falloff = 1.0
                    elif distance_sq >= outer_radius_sq:
                        continue
                    else:
                        distance = math.sqrt(distance_sq)
                        falloff = _clamp01(1.0 - (distance - radius) / max(1e-6, softness_world))
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
            for index in target_indices:
                dx = world_x[index] - orb_center[0]
                dy = world_y[index] - orb_center[1]
                dz = world_z[index] - orb_center[2]
                distance_sq = dx * dx + dy * dy + dz * dz
                if distance_sq <= radius_sq:
                    falloff = 1.0
                elif distance_sq >= outer_radius_sq:
                    falloff = 0.0
                else:
                    distance = math.sqrt(distance_sq)
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
            if blend_mode == "max":
                for index in target_indices:
                    if world_y[index] >= level:
                        falloff = 1.0
                    else:
                        falloff = _clamp01(1.0 - (level - world_y[index]) / edge_softness)
                    src_r = _clamp01(base_r * intensity * falloff)
                    src_g = _clamp01(base_g * intensity * falloff)
                    src_b = _clamp01(base_b * intensity * falloff)
                    src_a = _clamp01(base_a * intensity * falloff)
                    color_r[index] = max(color_r[index], src_r)
                    color_g[index] = max(color_g[index], src_g)
                    color_b[index] = max(color_b[index], src_b)
                    color_a[index] = max(color_a[index], src_a)
                continue
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

        _blend_uniform(
            blend_mode,
            target_indices,
            base_r * intensity,
            base_g * intensity,
            base_b * intensity,
            base_a * intensity,
            color_r,
            color_g,
            color_b,
            color_a,
        )

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
    role_clip_map = _sorted_role_clip_map(project)
    return {
        role: _build_preview_frame(project, role, time_ms, runtime, role_clip_map.get(role))
        for role in roles
    }
