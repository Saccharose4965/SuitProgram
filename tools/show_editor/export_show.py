from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import hashlib
import shutil
import struct
import zlib

from .canonical_layout import GROUP_IDS, ROLE_NAMES, SECTION_IDS
from .project_model import ShowClip, ShowProject

SHOW_FILE_MAGIC_V1 = 0x31535753
SHOW_FILE_VERSION_MAJOR_V1 = 1
SHOW_FILE_VERSION_MINOR_V1 = 0

SHOW_TARGET_KIND_ALL = 1
SHOW_TARGET_KIND_SECTION = 2
SHOW_TARGET_KIND_GROUP = 3

SHOW_BLEND_REPLACE = 0
SHOW_BLEND_ALPHA = 1
SHOW_BLEND_ADD = 2
SHOW_BLEND_MAX = 3
SHOW_CLIP_FLAG_TEMPO_SYNC = 1 << 0

SHOW_AXIS_Y = 0
SHOW_AXIS_X = 1
SHOW_AXIS_Z = 2
SHOW_AXIS_SECTION_U = 3
SHOW_AXIS_RADIAL = 4
SHOW_AXIS_RANDOM_XY = 5

SHOW_EFFECT_SOLID = 1
SHOW_EFFECT_BLINK = 2
SHOW_EFFECT_PULSE = 3
SHOW_EFFECT_SWEEP = 6
SHOW_EFFECT_MIRROR_SWEEP = 7
SHOW_EFFECT_CHASE = 8
SHOW_EFFECT_SPARKLE = 10
SHOW_EFFECT_FANOUT = 11
SHOW_EFFECT_GLOBAL_SWEEP = 13
SHOW_EFFECT_TRAVELING_ORB = 14
SHOW_EFFECT_GROUND_ENERGY = 16
SHOW_EFFECT_RADIAL_RAY = 17

HEADER_STRUCT = struct.Struct("<IHHIIIQIHHHHHHIIIIIII")
ROLE_STRUCT = struct.Struct("<BBHIIII")
BUCKET_STRUCT = struct.Struct("<IIHH")
CLIP_STRUCT = struct.Struct("<IIHBBBBIHHHHIHH")
SOLID_PARAM_STRUCT = struct.Struct("<BBBBHH")
SPATIAL_PARAM_STRUCT = struct.Struct("<BBBBBBBBBBHHHHHHHHHH")

BLEND_IDS = {
    "replace": SHOW_BLEND_REPLACE,
    "alpha": SHOW_BLEND_ALPHA,
    "add": SHOW_BLEND_ADD,
    "max": SHOW_BLEND_MAX,
}

EFFECT_IDS = {
    "solid": SHOW_EFFECT_SOLID,
    "blink": SHOW_EFFECT_BLINK,
    "pulse": SHOW_EFFECT_PULSE,
    "sweep": SHOW_EFFECT_SWEEP,
    "mirror_sweep": SHOW_EFFECT_MIRROR_SWEEP,
    "chase": SHOW_EFFECT_CHASE,
    "sparkle": SHOW_EFFECT_SPARKLE,
    "fanout": SHOW_EFFECT_FANOUT,
    "global_sweep": SHOW_EFFECT_GLOBAL_SWEEP,
    "traveling_orb": SHOW_EFFECT_TRAVELING_ORB,
    "ground_energy": SHOW_EFFECT_GROUND_ENERGY,
    "radialray": SHOW_EFFECT_RADIAL_RAY,
}

AXIS_IDS = {
    "y": SHOW_AXIS_Y,
    "x": SHOW_AXIS_X,
    "z": SHOW_AXIS_Z,
    "section_u": SHOW_AXIS_SECTION_U,
    "radial": SHOW_AXIS_RADIAL,
    "random_xy": SHOW_AXIS_RANDOM_XY,
}


class ShowExportError(RuntimeError):
    pass


@dataclass(frozen=True)
class CompiledClip:
    start_ms: int
    end_ms: int
    layer: int
    effect_kind: int
    blend_mode: int
    target_kind: int
    target_id: int
    palette_id: int
    flags: int
    fade_in_ms_div10: int
    fade_out_ms_div10: int
    param_bytes: bytes
    seed: int


@dataclass(frozen=True)
class ExportResult:
    show_bin_path: Path
    track_path: Path | None
    role_clip_counts: dict[str, int]


def _show_uid(project: ShowProject) -> int:
    digest = hashlib.blake2b(
        f"{project.slug}\n{project.title}\n{project.duration_ms}".encode("utf-8"),
        digest_size=8,
    ).digest()
    return int.from_bytes(digest, "little")


def _target_names(target: str) -> list[str]:
    return [
        name.strip()
        for name in str(target).split(",")
        if name.strip()
    ]


def _clip_targets(clip: ShowClip) -> list[tuple[int, int]]:
    kind = clip.target_kind
    if kind == "all":
        return [(SHOW_TARGET_KIND_ALL, 0)]
    if kind == "section":
        targets = _target_names(clip.target)
        if not targets:
            return []
        compiled: list[tuple[int, int]] = []
        for target_name in targets:
            if target_name not in SECTION_IDS:
                raise ShowExportError(f"unknown section target: {target_name}")
            compiled.append((SHOW_TARGET_KIND_SECTION, SECTION_IDS[target_name]))
        return compiled
    if kind == "group":
        targets = _target_names(clip.target)
        if not targets:
            return []
        compiled = []
        for target_name in targets:
            if target_name not in GROUP_IDS:
                raise ShowExportError(f"unknown group target: {target_name}")
            compiled.append((SHOW_TARGET_KIND_GROUP, GROUP_IDS[target_name]))
        return compiled
    raise ShowExportError(f"unknown target kind: {clip.target_kind}")


def _solid_param_bytes(clip: ShowClip) -> bytes:
    color = clip.params.get("color", [255, 255, 255, 255])
    if len(color) != 4:
        raise ShowExportError("solid.color must contain 4 channels")
    intensity = float(clip.params.get("intensity", 1.0))
    if intensity < 0.0:
        intensity = 0.0
    if intensity > 1.0:
        intensity = 1.0
    return SOLID_PARAM_STRUCT.pack(
        int(color[0]) & 0xFF,
        int(color[1]) & 0xFF,
        int(color[2]) & 0xFF,
        int(color[3]) & 0xFF,
        int(round(intensity * 1024.0)) & 0xFFFF,
        0,
    )


def _u16_scaled(value: float, scale: float = 1024.0) -> int:
    return max(0, min(0xFFFF, int(round(float(value) * scale))))


def _clip_axis_random_mix(clip: ShowClip) -> float:
    mix = clip.params.get("axis_random_mix")
    try:
        if mix is not None:
            return max(0.0, min(1.0, float(mix)))
    except (TypeError, ValueError):
        pass
    payload = (
        f"{clip.effect}|{clip.target_kind}|{clip.target}|"
        f"{clip.layer}|{clip.start_ms}|{clip.end_ms}"
    ).encode("utf-8")
    digest = hashlib.blake2b(payload, digest_size=2).digest()
    return int.from_bytes(digest, "little") / 65535.0


def _spatial_param_bytes(clip: ShowClip) -> bytes:
    color = clip.params.get("color", [255, 255, 255, 255])
    to_color = clip.params.get("to_color", [0, 0, 0, 255])
    if not isinstance(color, (list, tuple)) or len(color) != 4:
        raise ShowExportError("color must contain 4 channels")
    if not isinstance(to_color, (list, tuple)) or len(to_color) != 4:
        raise ShowExportError("to_color must contain 4 channels")
    axis = str(clip.params.get("axis", "y"))
    if axis not in AXIS_IDS:
        raise ShowExportError(f"unknown axis: {axis}")
    return SPATIAL_PARAM_STRUCT.pack(
        int(color[0]) & 0xFF,
        int(color[1]) & 0xFF,
        int(color[2]) & 0xFF,
        int(color[3]) & 0xFF,
        int(to_color[0]) & 0xFF,
        int(to_color[1]) & 0xFF,
        int(to_color[2]) & 0xFF,
        int(to_color[3]) & 0xFF,
        AXIS_IDS[axis] & 0xFF,
        1 if bool(clip.params.get("reverse", False)) else 0,
        _u16_scaled(float(clip.params.get("intensity", 1.0))),
        _u16_scaled(float(clip.params.get("width", 0.22))),
        _u16_scaled(float(clip.params.get("softness", 0.16))),
        _u16_scaled(float(clip.params.get("frequency_hz", 1.0))),
        _u16_scaled(float(clip.params.get("phase", 0.0))),
        max(0, min(0xFFFF, int(clip.params.get("repeats", 1)))),
        _u16_scaled(float(clip.params.get("duty_cycle", 0.5))),
        _u16_scaled(float(clip.params.get("min_intensity", 0.0))),
        _u16_scaled(float(clip.params.get("max_intensity", 1.0))),
        _u16_scaled(_clip_axis_random_mix(clip)),
    )


def _compile_clips(clip: ShowClip) -> list[CompiledClip]:
    if clip.end_ms <= clip.start_ms:
        raise ShowExportError("clip end_ms must be greater than start_ms")
    if clip.effect not in EFFECT_IDS:
        raise ShowExportError(f"unsupported effect for scaffold exporter: {clip.effect}")
    if clip.blend not in BLEND_IDS:
        raise ShowExportError(f"unknown blend mode: {clip.blend}")

    targets = _clip_targets(clip)
    if not targets:
        return []
    param_bytes = _solid_param_bytes(clip) if clip.effect == "solid" else _spatial_param_bytes(clip)
    clip_flags = SHOW_CLIP_FLAG_TEMPO_SYNC if bool(clip.params.get("tempo_sync", False)) else 0

    return [
        CompiledClip(
            start_ms=clip.start_ms,
            end_ms=clip.end_ms,
            layer=clip.layer,
            effect_kind=EFFECT_IDS[clip.effect],
            blend_mode=BLEND_IDS[clip.blend],
            target_kind=target_kind,
            target_id=target_id,
            palette_id=0,
            flags=clip_flags,
            fade_in_ms_div10=0,
            fade_out_ms_div10=0,
            param_bytes=param_bytes,
            seed=0,
        )
        for target_kind, target_id in targets
    ]


def _role_clips(project: ShowProject, role: str) -> list[CompiledClip]:
    items = list(project.global_clips)
    items.extend(project.role_clips.get(role, []))
    compiled: list[CompiledClip] = []
    for clip in items:
        compiled.extend(_compile_clips(clip))
    compiled.sort(key=lambda item: (item.start_ms, item.layer, item.end_ms))
    return compiled


def _build_buckets(clips: list[CompiledClip], duration_ms: int, bucket_ms: int) -> list[tuple[int, int, int]]:
    total_buckets = max(1, (max(duration_ms, 1) + bucket_ms - 1) // bucket_ms)
    buckets: list[tuple[int, int, int]] = []
    for bucket_index in range(total_buckets):
        bucket_start = bucket_index * bucket_ms
        clip_first = len(clips)
        for idx, clip in enumerate(clips):
            if clip.end_ms > bucket_start:
                clip_first = idx
                break
        clip_count = len(clips) - clip_first if clip_first < len(clips) else 0
        buckets.append((bucket_start, clip_first, clip_count))
    return buckets


def export_show(project: ShowProject, output_dir: str | Path, copy_audio: bool = False) -> ExportResult:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    bucket_ms = max(1, int(project.bucket_ms))
    role_clips = {role: _role_clips(project, role) for role in ROLE_NAMES}
    role_buckets = {
        role: _build_buckets(role_clips[role], project.duration_ms, bucket_ms)
        for role in ROLE_NAMES
    }

    role_table_offset = HEADER_STRUCT.size
    palette_table_offset = role_table_offset + len(ROLE_NAMES) * ROLE_STRUCT.size
    group_table_offset = palette_table_offset
    bucket_table_offset = group_table_offset
    role_bucket_first: dict[str, int] = {}
    role_clip_first: dict[str, int] = {}

    bucket_blob = bytearray()
    bucket_index = 0
    for role in ROLE_NAMES:
        role_bucket_first[role] = bucket_index
        for bucket_start, clip_first, clip_count in role_buckets[role]:
            bucket_blob.extend(BUCKET_STRUCT.pack(bucket_start, clip_first, clip_count, 0))
            bucket_index += 1

    clip_table_offset = bucket_table_offset + len(bucket_blob)

    clip_blob = bytearray()
    param_blob = bytearray()
    clip_index = 0
    for role in ROLE_NAMES:
        role_clip_first[role] = clip_index
        for clip in role_clips[role]:
            param_offset = clip_table_offset + 0  # placeholder, fixed below
            clip_blob.extend(
                CLIP_STRUCT.pack(
                    clip.start_ms,
                    clip.end_ms,
                    clip.layer & 0xFFFF,
                    clip.effect_kind & 0xFF,
                    clip.blend_mode & 0xFF,
                    clip.target_kind & 0xFF,
                    0,
                    clip.target_id & 0xFFFFFFFF,
                    clip.palette_id & 0xFFFF,
                    clip.flags & 0xFFFF,
                    clip.fade_in_ms_div10 & 0xFFFF,
                    clip.fade_out_ms_div10 & 0xFFFF,
                    param_offset,
                    len(clip.param_bytes) & 0xFFFF,
                    clip.seed & 0xFFFF,
                )
            )
            param_blob.extend(clip.param_bytes)
            clip_index += 1

    param_blob_offset = clip_table_offset + len(clip_blob)
    string_table_offset = param_blob_offset + len(param_blob)

    patched_clip_blob = bytearray()
    param_cursor = 0
    all_clips: list[CompiledClip] = []
    for role in ROLE_NAMES:
        all_clips.extend(role_clips[role])
    for clip in all_clips:
        patched_clip_blob.extend(
            CLIP_STRUCT.pack(
                clip.start_ms,
                clip.end_ms,
                clip.layer & 0xFFFF,
                clip.effect_kind & 0xFF,
                clip.blend_mode & 0xFF,
                clip.target_kind & 0xFF,
                0,
                clip.target_id & 0xFFFFFFFF,
                clip.palette_id & 0xFFFF,
                clip.flags & 0xFFFF,
                clip.fade_in_ms_div10 & 0xFFFF,
                clip.fade_out_ms_div10 & 0xFFFF,
                param_blob_offset + param_cursor,
                len(clip.param_bytes) & 0xFFFF,
                clip.seed & 0xFFFF,
            )
        )
        param_cursor += len(clip.param_bytes)

    role_blob = bytearray()
    for index, role in enumerate(ROLE_NAMES):
        role_blob.extend(
            ROLE_STRUCT.pack(
                index,
                0,
                0,
                role_bucket_first[role],
                len(role_buckets[role]),
                role_clip_first[role],
                len(role_clips[role]),
            )
        )

    file_bytes = string_table_offset
    header = HEADER_STRUCT.pack(
        SHOW_FILE_MAGIC_V1,
        SHOW_FILE_VERSION_MAJOR_V1,
        SHOW_FILE_VERSION_MINOR_V1,
        HEADER_STRUCT.size,
        file_bytes,
        0,
        _show_uid(project),
        project.duration_ms,
        len(ROLE_NAMES),
        project.fps_hint,
        bucket_ms,
        0,
        0,
        0,
        role_table_offset,
        palette_table_offset,
        group_table_offset,
        bucket_table_offset,
        clip_table_offset,
        param_blob_offset,
        string_table_offset,
    )
    payload = bytearray()
    payload.extend(header)
    payload.extend(role_blob)
    payload.extend(bucket_blob)
    payload.extend(patched_clip_blob)
    payload.extend(param_blob)

    crc32 = zlib.crc32(payload) & 0xFFFFFFFF
    header = HEADER_STRUCT.pack(
        SHOW_FILE_MAGIC_V1,
        SHOW_FILE_VERSION_MAJOR_V1,
        SHOW_FILE_VERSION_MINOR_V1,
        HEADER_STRUCT.size,
        file_bytes,
        crc32,
        _show_uid(project),
        project.duration_ms,
        len(ROLE_NAMES),
        project.fps_hint,
        bucket_ms,
        0,
        0,
        0,
        role_table_offset,
        palette_table_offset,
        group_table_offset,
        bucket_table_offset,
        clip_table_offset,
        param_blob_offset,
        string_table_offset,
    )
    payload[: HEADER_STRUCT.size] = header

    show_bin_path = output_path / "show.bin"
    show_bin_path.write_bytes(payload)

    track_path: Path | None = None
    if copy_audio and project.audio_source:
        src = Path(project.audio_source)
        if src.exists():
            track_path = output_path / "track.wav"
            shutil.copy2(src, track_path)

    return ExportResult(
        show_bin_path=show_bin_path,
        track_path=track_path,
        role_clip_counts={role: len(role_clips[role]) for role in ROLE_NAMES},
    )
