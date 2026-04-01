from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import re
from typing import Any

from .canonical_layout import ROLE_NAMES


@dataclass
class ShowClip:
    start_ms: int
    end_ms: int
    layer: int = 0
    effect: str = "solid"
    blend: str = "replace"
    target_kind: str = "group"
    target: str = "all"
    palette: str | None = None
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class ShowProject:
    slug: str
    title: str
    duration_ms: int
    audio_source: str = ""
    source_path: Path | None = None
    tempo_bpm: float = 120.0
    beat_offset_ms: int = 0
    fps_hint: int = 30
    bucket_ms: int = 1000
    global_clips: list[ShowClip] = field(default_factory=list)
    role_clips: dict[str, list[ShowClip]] = field(
        default_factory=lambda: {role: [] for role in ROLE_NAMES}
    )
    palettes: dict[str, Any] = field(default_factory=dict)


def _clip_from_obj(obj: dict[str, Any]) -> ShowClip:
    return ShowClip(
        start_ms=int(obj["start_ms"]),
        end_ms=int(obj["end_ms"]),
        layer=int(obj.get("layer", 0)),
        effect=str(obj.get("effect", "solid")),
        blend=str(obj.get("blend", "replace")),
        target_kind=str(obj.get("target_kind", "group")),
        target=str(obj.get("target", "all")),
        palette=obj.get("palette"),
        params=dict(obj.get("params", {})),
    )


def _clip_to_obj(clip: ShowClip) -> dict[str, Any]:
    return {
        "start_ms": clip.start_ms,
        "end_ms": clip.end_ms,
        "layer": clip.layer,
        "effect": clip.effect,
        "blend": clip.blend,
        "target_kind": clip.target_kind,
        "target": clip.target,
        "palette": clip.palette,
        "params": clip.params,
    }


def project_from_obj(raw: dict[str, Any], source_path: str | Path | None = None) -> ShowProject:
    meta = raw.get("meta", {})
    audio = raw.get("audio", {})
    roles_raw = raw.get("roles", {})
    role_clips: dict[str, list[ShowClip]] = {role: [] for role in ROLE_NAMES}
    for role in ROLE_NAMES:
        role_entry = roles_raw.get(role, {})
        role_clips[role] = [_clip_from_obj(item) for item in role_entry.get("clips", [])]
    return ShowProject(
        slug=str(meta["slug"]),
        title=str(meta.get("title", meta["slug"])),
        duration_ms=int(meta["duration_ms"]),
        audio_source=str(audio.get("source", "")),
        source_path=Path(source_path).resolve() if source_path is not None else None,
        tempo_bpm=float(meta.get("tempo_bpm", 120.0)),
        beat_offset_ms=int(meta.get("beat_offset_ms", 0)),
        fps_hint=int(meta.get("fps_hint", 30)),
        bucket_ms=int(meta.get("bucket_ms", 1000)),
        global_clips=[_clip_from_obj(item) for item in raw.get("global_clips", [])],
        role_clips=role_clips,
        palettes=dict(raw.get("palettes", {})),
    )


def project_to_obj(project: ShowProject) -> dict[str, Any]:
    return {
        "meta": {
            "slug": project.slug,
            "title": project.title,
            "duration_ms": project.duration_ms,
            "tempo_bpm": project.tempo_bpm,
            "beat_offset_ms": project.beat_offset_ms,
            "fps_hint": project.fps_hint,
            "bucket_ms": project.bucket_ms,
        },
        "audio": {
            "source": project.audio_source,
        },
        "global_clips": [_clip_to_obj(clip) for clip in project.global_clips],
        "roles": {
            role: {
                "clips": [_clip_to_obj(clip) for clip in project.role_clips.get(role, [])],
            }
            for role in ROLE_NAMES
        },
        "palettes": project.palettes,
    }


def load_project(path: str | Path) -> ShowProject:
    project_path = Path(path).resolve()
    raw = json.loads(project_path.read_text())
    return project_from_obj(raw, source_path=project_path)


def save_project(project: ShowProject, path: str | Path) -> None:
    output_path = Path(path).resolve()
    data = project_to_obj(project)
    output_path.write_text(json.dumps(data, indent=2) + "\n")
    project.source_path = output_path


def resolve_audio_path(project: ShowProject) -> Path | None:
    if not project.audio_source:
        return None
    audio_path = Path(project.audio_source)
    if audio_path.is_absolute():
        return audio_path
    if project.source_path is not None:
        project_relative = project.source_path.parent / audio_path
        if project_relative.exists():
            return project_relative
    # Legacy fallback: older projects lived in tools/show_editor, so moving the
    # JSON into shows/ can otherwise break relative audio paths.
    legacy_project_dir = Path(__file__).resolve().parent
    legacy_relative = legacy_project_dir / audio_path
    if legacy_relative.exists():
        return legacy_relative
    if project.source_path is not None:
        return project.source_path.parent / audio_path
    return audio_path


def to_project_relative_path(project: ShowProject, path: str | Path) -> str:
    target = Path(path).resolve()
    if project.source_path is None:
        return str(target)
    return os.path.relpath(target, start=project.source_path.parent.resolve())


def _slugify_title(text: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "-", text.strip().lower())
    normalized = normalized.strip("-")
    return normalized or "untitled-show"


def create_project_from_audio_path(audio_path: str | Path, duration_ms: int = 0) -> ShowProject:
    resolved_audio_path = Path(audio_path).resolve()
    stem = resolved_audio_path.stem.strip()
    title = re.sub(r"[_-]+", " ", stem).strip() or "Untitled Show"
    slug = _slugify_title(stem or title)
    return ShowProject(
        slug=slug,
        title=title,
        duration_ms=max(0, int(duration_ms)),
        audio_source=str(resolved_audio_path),
    )
