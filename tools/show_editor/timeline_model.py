from __future__ import annotations

from dataclasses import dataclass

from .canonical_layout import ROLE_NAMES
from .project_model import ShowClip, ShowProject


@dataclass(frozen=True)
class TimelineRow:
    role: str
    source: str
    clip: ShowClip
    clip_index: int


def build_timeline_rows(project: ShowProject) -> list[TimelineRow]:
    rows: list[TimelineRow] = []
    for clip_index, clip in enumerate(project.global_clips):
        rows.append(TimelineRow(role="*", source="global", clip=clip, clip_index=clip_index))
    for role in ROLE_NAMES:
        for clip_index, clip in enumerate(project.role_clips.get(role, [])):
            rows.append(TimelineRow(role=role, source="role", clip=clip, clip_index=clip_index))
    rows.sort(key=lambda row: (row.clip.start_ms, row.clip.layer, row.role))
    return rows
