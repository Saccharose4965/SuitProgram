from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
from typing import Any

from .project_model import ShowProject, project_from_obj, project_to_obj


@dataclass(frozen=True)
class EditorSessionState:
    project: ShowProject
    current_time_ms: int = 0
    active_role: str = "A"
    preview_scope: str = "Active"
    view_mode: str = "2d"
    show_body: bool = False
    snap_enabled: bool = True
    snap_divisor: int = 1
    camera_preset: str = "iso"
    layout_kind: str = "generated"
    layout_path: str = ""
    presets: tuple[dict[str, Any], ...] = ()


def _state_root() -> Path:
    xdg_state_home = os.environ.get("XDG_STATE_HOME", "").strip()
    if xdg_state_home:
        return Path(xdg_state_home).expanduser()
    return Path.home() / ".local" / "state"


def session_file_path() -> Path:
    session_dir = _state_root() / "suit_show_editor"
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir / "session.json"


def save_editor_session(state: EditorSessionState) -> Path:
    source_path = str(state.project.source_path) if state.project.source_path is not None else ""
    payload = {
        "project": project_to_obj(state.project),
        "project_source_path": source_path,
        "ui": {
            "current_time_ms": int(state.current_time_ms),
            "active_role": state.active_role,
            "preview_scope": state.preview_scope,
            "view_mode": state.view_mode,
            "show_body": state.show_body,
            "snap_enabled": bool(state.snap_enabled),
            "snap_divisor": int(state.snap_divisor),
            "camera_preset": state.camera_preset,
            "layout_kind": state.layout_kind,
            "layout_path": state.layout_path,
            "presets": [dict(entry) for entry in state.presets],
        },
    }
    output_path = session_file_path()
    output_path.write_text(json.dumps(payload, indent=2) + "\n")
    return output_path


def load_editor_session() -> EditorSessionState | None:
    session_path = session_file_path()
    if not session_path.exists():
        return None
    raw = json.loads(session_path.read_text())
    project = project_from_obj(
        dict(raw.get("project", {})),
        source_path=str(raw.get("project_source_path", "")).strip() or None,
    )
    ui = dict(raw.get("ui", {}))
    return EditorSessionState(
        project=project,
        current_time_ms=int(ui.get("current_time_ms", 0)),
        active_role=str(ui.get("active_role", "A")),
        preview_scope=str(ui.get("preview_scope", "Active")),
        view_mode=str(ui.get("view_mode", "2d")),
        show_body=bool(ui.get("show_body", False)),
        snap_enabled=bool(ui.get("snap_enabled", True)),
        snap_divisor=max(1, int(ui.get("snap_divisor", 1))),
        camera_preset=str(ui.get("camera_preset", "iso")),
        layout_kind=str(ui.get("layout_kind", "generated")),
        layout_path=str(ui.get("layout_path", "")),
        presets=tuple(
            dict(entry)
            for entry in ui.get("presets", [])
            if isinstance(entry, dict)
        ),
    )
