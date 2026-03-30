from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import time

from .audio_waveform import AudioWaveform, load_audio_waveform
from .automation_editors import (
    CurveEditorDialog,
    CurveParameterSpec,
    PaletteEditorDialog,
    PaletteStripWidget,
    encode_palette_text,
    parse_palette_text,
)
from .color_palettes import BUILTIN_COLOR_PALETTES, normalize_palette_store_text, resolve_palette_text
from .export_show import export_show
from .layout_io import LoadedLayout, generated_layout, load_layout_file
from .preview_model import PreviewFrame, build_preview_frames, clip_color_at, clip_color_preview_rgba8
from .project_model import (
    ShowProject,
    ShowClip,
    create_project_from_audio_path,
    load_project,
    project_from_obj,
    project_to_obj,
    resolve_audio_path,
    save_project,
    to_project_relative_path,
)
from .session_state import EditorSessionState, save_editor_session
from .qt_compat import (
    IMPORT_ERROR,
    MULTIMEDIA_IMPORT_ERROR,
    PYSIDE_AVAILABLE,
    QT_MULTIMEDIA_AVAILABLE,
    QtCore,
    QtGui,
    QtMultimedia,
    QtWidgets,
)
from .canonical_layout import GROUP_SECTIONS, ROLE_NAMES, SECTION_IDS
from .timeline_model import build_timeline_rows
from tools.layout_model import sample_section_points
from tools.spatial_display import FRONT_BELT_Y, display_point as spatial_display_point


if PYSIDE_AVAILABLE:

    EFFECT_LIBRARY = (
        ("Solid", "solid", "group", "all", "max", 1000, {"color": [255, 255, 255, 255], "intensity": 1.0}, ("Suit Effects", "Basic")),
        ("Blink", "blink", "group", "all", "max", 1000, {"color": [255, 255, 255, 255], "intensity": 1.0, "frequency_hz": 2.0}, ("Suit Effects", "Basic")),
        ("Fade", "fade", "group", "all", "max", 1400, {"color": [255, 255, 255, 255], "to_color": [0, 0, 0, 255], "intensity": 1.0}, ("Suit Effects", "Basic")),
        ("Pulse", "pulse", "group", "all", "max", 1200, {"color": [255, 255, 255, 255], "intensity": 0.8, "frequency_hz": 1.5}, ("Suit Effects", "Basic")),
        ("Strobe", "strobe", "group", "all", "max", 800, {"color": [255, 255, 255, 255], "intensity": 1.0, "frequency_hz": 8.0}, ("Suit Effects", "Basic")),
        ("Sweep", "sweep", "group", "all", "max", 1600, {"color": [255, 255, 255, 255], "intensity": 1.0, "axis": "y", "width": 0.25, "softness": 0.18}, ("Suit Effects", "Motion")),
        ("Mirror Sweep", "mirror_sweep", "group", "all", "max", 1600, {"color": [255, 255, 255, 255], "intensity": 1.0, "axis": "y", "width": 0.25, "softness": 0.18}, ("Suit Effects", "Motion")),
        ("Chase", "chase", "group", "all", "max", 1400, {"color": [255, 255, 255, 255], "intensity": 0.9, "axis": "y", "width": 0.18, "repeats": 4}, ("Suit Effects", "Motion")),
        ("Sparkle", "sparkle", "group", "all", "max", 1200, {"color": [255, 255, 255, 255], "intensity": 1.0, "frequency_hz": 6.0}, ("Suit Effects", "Motion")),
        ("Fanout", "fanout", "group", "all", "max", 1500, {"color": [255, 255, 255, 255], "intensity": 1.0, "width": 0.24, "softness": 0.18}, ("Suit Effects", "Motion")),
        ("Gradient", "gradient", "group", "all", "max", 1600, {"color": [255, 255, 255, 255], "to_color": [0, 128, 255, 255], "intensity": 1.0, "axis": "y"}, ("Suit Effects", "Color")),
        ("Palette Cycle", "palette_cycle", "group", "all", "max", 1800, {"color": [255, 255, 255, 255], "to_color": [0, 128, 255, 255], "intensity": 1.0, "frequency_hz": 1.0}, ("Suit Effects", "Color")),
        ("Global Plane Sweep", "global_plane_sweep", "group", "all", "max", 2000, {"color": [255, 255, 255, 255], "intensity": 1.0, "axis": "x", "width": 0.22, "softness": 0.18}, ("Group Effects", "Spatial")),
        ("Traveling Orb", "traveling_orb", "group", "all", "max", 2000, {"color": [255, 255, 255, 255], "intensity": 1.0, "axis": "x", "width": 0.20, "softness": 0.16}, ("Group Effects", "Spatial")),
        ("Ring Burst", "ring_burst", "group", "all", "max", 2200, {"color": [255, 255, 255, 255], "intensity": 1.0, "width": 0.16, "softness": 0.12}, ("Group Effects", "Spatial")),
        ("Radial Ray", "radialray", "group", "all", "max", 1800, {"color": [255, 255, 255, 255], "intensity": 1.0, "width": 0.10, "softness": 0.08, "frequency_hz": 1.0}, ("Group Effects", "Spatial")),
        ("Ground Energy", "ground_energy", "group", "all", "max", 1800, {"color": [255, 255, 255, 255], "intensity": 1.0, "softness": 0.14}, ("Group Effects", "Spatial")),
    )

    HIDDEN_PICKER_EFFECT_IDS = frozenset({"gradient", "palette_cycle"})
    PICKER_EFFECT_LIBRARY = tuple(item for item in EFFECT_LIBRARY if item[1] not in HIDDEN_PICKER_EFFECT_IDS)
    EFFECT_NAMES = tuple(item[0] for item in PICKER_EFFECT_LIBRARY)
    EFFECT_NAME_TO_SPEC = {item[0]: item for item in EFFECT_LIBRARY}
    EFFECT_ID_TO_NAME = {item[1]: item[0] for item in EFFECT_LIBRARY}
    BLEND_OPTIONS = ("replace", "alpha", "add", "max")
    CLIP_ROLE_OPTIONS = ("*", "A", "B", "C")
    TARGET_MODE_OPTIONS = (
        ("All", "all"),
        ("Groups", "group"),
        ("Sections", "section"),
    )
    PREVIEW_OPTIONS = ("A", "B", "C", "All")
    PREVIEW_ROLE_ORDER = tuple(ROLE_NAMES)
    PREVIEW_ROLE_COLORS = {
        "A": QtGui.QColor("#ff9f45"),
        "B": QtGui.QColor("#ff5d8f"),
        "C": QtGui.QColor("#43d17a"),
    }
    PREVIEW_ROLE_SPACING_2D = 98.0
    PREVIEW_ROLE_SPACING_3D = 62.0
    PREVIEW_BODY_FILL = QtGui.QColor(108, 112, 120, 230)
    PREVIEW_BODY_OUTLINE = QtGui.QColor(142, 148, 156, 240)
    PREVIEW_BODY_TORSO = ((0.0, -18.5, 0.0), (0.0, FRONT_BELT_Y, 0.0), (12.8, 9.5), 0.88)
    PREVIEW_BODY_ARM_CAPSULES = (
        ((19.3, -5.5, 0.0), (22.4, 19.0, 0.0), (4.2, 3.8)),
        ((-19.3, -5.5, 0.0), (-22.4, 19.0, 0.0), (4.2, 3.8)),
    )
    CLIP_RESIZE_HANDLE_PX = 7.0
    TIMELINE_TRACK_COUNT = 4
    EFFECT_FIELDS_COLOR = frozenset({"color", "intensity", "curves", "palette"})
    EFFECT_FIELDS_TO_COLOR = frozenset({"fade", "gradient", "palette_cycle"})
    EFFECT_FIELDS_AXIS = frozenset({"gradient", "sweep", "mirror_sweep", "chase", "global_plane_sweep", "traveling_orb"})
    EFFECT_FIELDS_WIDTH = frozenset({"sweep", "mirror_sweep", "chase", "fanout", "global_plane_sweep", "traveling_orb", "ring_burst", "radialray"})
    EFFECT_FIELDS_SOFTNESS = frozenset({"sweep", "mirror_sweep", "fanout", "global_plane_sweep", "traveling_orb", "ring_burst", "radialray", "ground_energy"})
    EFFECT_FIELDS_RATE = frozenset({"blink", "strobe", "pulse", "sweep", "mirror_sweep", "chase", "sparkle", "fanout", "palette_cycle", "global_plane_sweep", "traveling_orb", "ring_burst", "radialray", "ground_energy"})
    EFFECT_FIELDS_PHASE = frozenset({"blink", "strobe", "pulse", "sweep", "mirror_sweep", "chase", "sparkle", "fanout", "palette_cycle", "global_plane_sweep", "traveling_orb", "ring_burst", "radialray", "ground_energy"})
    EFFECT_FIELDS_REPEATS = frozenset({"chase"})
    EFFECT_FIELDS_DUTY = frozenset({"blink", "strobe"})
    EFFECT_FIELDS_MIN_MAX = frozenset({"pulse"})
    EFFECT_FIELDS_REVERSE = frozenset({"sweep", "mirror_sweep", "fanout", "global_plane_sweep", "traveling_orb", "ring_burst", "radialray", "ground_energy"})

    class PreviewRenderProxy(QtCore.QObject):
        render_requested = QtCore.Signal(object)


    class PreviewRenderWorker(QtCore.QObject):
        rendered = QtCore.Signal(object)

        @QtCore.Slot(object)
        def render_request(self, request: object) -> None:
            payload = dict(request) if isinstance(request, dict) else {}
            project = payload.get("project")
            layout_data = payload.get("layout_data")
            preview_roles = tuple(payload.get("preview_roles", ()))
            current_ms = int(payload.get("time_ms", 0))
            try:
                if project is None or layout_data is None:
                    payload["frames"] = {}
                else:
                    payload["frames"] = build_preview_frames(project, layout_data, preview_roles, current_ms)
                payload["error"] = ""
            except Exception as exc:  # pragma: no cover - worker path requires PySide runtime
                payload["frames"] = {}
                payload["error"] = str(exc)
            self.rendered.emit(payload)


    class SliderFieldWidget(QtWidgets.QWidget):
        valueChanged = QtCore.Signal(float)
        editingFinished = QtCore.Signal()

        def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
            super().__init__(parent)
            self._minimum = 0.0
            self._maximum = 1.0
            self._single_step = 0.01
            self._decimals = 2
            self._value = 0.0
            self._slider_steps = 100

            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.setFocusPolicy(QtCore.Qt.StrongFocus)
            self.line_edit = QtWidgets.QLineEdit()
            self.line_edit.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.line_edit.setFixedWidth(72)

            layout = QtWidgets.QHBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(6)
            layout.addWidget(self.slider, 1)
            layout.addWidget(self.line_edit)

            self.slider.valueChanged.connect(self._on_slider_value_changed)
            self.slider.sliderReleased.connect(self.editingFinished.emit)
            self.line_edit.editingFinished.connect(self._on_line_edit_editing_finished)

            self._update_validator()
            self._update_slider_range()
            self._sync_widgets(self._value)

        def setRange(self, minimum: float, maximum: float) -> None:
            self._minimum = float(minimum)
            self._maximum = max(float(maximum), self._minimum)
            self._value = self._clamp_value(self._value)
            self._update_validator()
            self._update_slider_range()
            self._sync_widgets(self._value)

        def setSingleStep(self, step: float) -> None:
            self._single_step = max(0.0001, float(step))

        def setDecimals(self, decimals: int) -> None:
            self._decimals = max(0, int(decimals))
            self._update_validator()
            self._update_slider_range()
            self._sync_widgets(self._value)

        def value(self) -> float:
            return float(self._value)

        def setValue(self, value: float) -> None:
            normalized = self._clamp_value(value)
            if math.isclose(normalized, self._value, rel_tol=0.0, abs_tol=self._value_tolerance()):
                self._sync_widgets(normalized)
                self._value = normalized
                return
            self._value = normalized
            self._sync_widgets(normalized)

        def _value_tolerance(self) -> float:
            return 1.0 / max(1.0, float(10 ** self._decimals))

        def _clamp_value(self, value: float) -> float:
            try:
                numeric_value = float(value)
            except (TypeError, ValueError):
                numeric_value = self._minimum
            return max(self._minimum, min(self._maximum, numeric_value))

        def _format_value(self, value: float) -> str:
            text = f"{float(value):.{self._decimals}f}"
            if self._decimals > 0:
                text = text.rstrip("0").rstrip(".")
            return text or "0"

        def _update_validator(self) -> None:
            validator = QtGui.QDoubleValidator(self._minimum, self._maximum, self._decimals, self.line_edit)
            validator.setNotation(QtGui.QDoubleValidator.StandardNotation)
            validator.setLocale(QtCore.QLocale.c())
            self.line_edit.setValidator(validator)

        def _update_slider_range(self) -> None:
            span = max(0.0, self._maximum - self._minimum)
            self._slider_steps = max(1, int(round(span * (10 ** self._decimals))))
            self.slider.setRange(0, self._slider_steps)
            if self._single_step > 0.0 and span > 0.0:
                page_step = max(1, int(round(self._single_step * self._slider_steps / span)))
                self.slider.setPageStep(page_step)
                self.slider.setSingleStep(max(1, page_step))

        def _value_to_slider(self, value: float) -> int:
            if self._maximum <= self._minimum:
                return 0
            ratio = (self._clamp_value(value) - self._minimum) / (self._maximum - self._minimum)
            return int(round(ratio * self._slider_steps))

        def _slider_to_value(self, slider_value: int) -> float:
            if self._slider_steps <= 0 or self._maximum <= self._minimum:
                return self._minimum
            ratio = float(slider_value) / float(self._slider_steps)
            raw_value = self._minimum + ratio * (self._maximum - self._minimum)
            return round(raw_value, self._decimals)

        def _sync_widgets(self, value: float) -> None:
            with QtCore.QSignalBlocker(self.slider):
                self.slider.setValue(self._value_to_slider(value))
            self.line_edit.setText(self._format_value(value))

        def _on_slider_value_changed(self, slider_value: int) -> None:
            new_value = self._slider_to_value(slider_value)
            if math.isclose(new_value, self._value, rel_tol=0.0, abs_tol=self._value_tolerance()):
                self._sync_widgets(new_value)
                return
            self._value = new_value
            self.line_edit.setText(self._format_value(new_value))
            self.valueChanged.emit(new_value)

        def _on_line_edit_editing_finished(self) -> None:
            raw_text = self.line_edit.text().strip()
            normalized_text = raw_text.replace(",", ".")
            try:
                parsed_value = float(normalized_text)
            except ValueError:
                parsed_value = self._value
            clamped_value = self._clamp_value(parsed_value)
            changed = not math.isclose(clamped_value, self._value, rel_tol=0.0, abs_tol=self._value_tolerance())
            self._value = clamped_value
            self._sync_widgets(clamped_value)
            if changed:
                self.valueChanged.emit(clamped_value)
            self.editingFinished.emit()


    class ShowTimelineWidget(QtWidgets.QWidget):
        def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
            super().__init__(parent)
            self.setMinimumHeight(480)
            self.setFocusPolicy(QtCore.Qt.ClickFocus)
            self.project: ShowProject | None = None
            self.waveform: AudioWaveform | None = None
            self.current_time_ms = 0
            self.selected_role = "A"
            self.seek_callback = None
            self.viewport_changed_callback = None
            self.clip_selected_callback = None
            self.clip_selection_changed_callback = None
            self.clip_moved_callback = None
            self.clip_create_callback = None
            self.clip_span_create_callback = None
            self.clip_edit_started_callback = None
            self.selected_clip = None
            self.selected_clip_ids: frozenset[int] = frozenset()
            self._drag_clip = None
            self._drag_kind = ""
            self._drag_anchor_time_ms = 0
            self._drag_start_ms = 0
            self._drag_end_ms = 0
            self._drag_history_started = False
            self._zoom_factor = 1.0
            self._visible_start_ms = 0
            self._hover_time_ms: int | None = None
            self._hover_snap_time_ms: int | None = None
            self._hover_role: str | None = None
            self._hover_layer: int | None = None
            self._active_lane_role: str | None = None
            self._active_lane_layer = 0
            self._drag_origin_role: str | None = None
            self._drag_origin_layer = 0
            self.snap_enabled = True
            self.snap_divisor = 1
            self.insert_effect_name = EFFECT_NAMES[0]
            self.insert_effect_duration_ms = 1000
            self._playhead_update_margin_px = 12
            self._last_viewport_state: tuple[int, int, int] | None = None
            self._last_visual_signature: tuple | None = None
            self._manual_view_until_s = 0.0
            self._selection_start_ms: int | None = None
            self._selection_end_ms: int | None = None
            self._range_selecting = False
            self._range_anchor_time_ms = 0
            self._insert_time_ms = 0
            self._span_create_role: str | None = None
            self._span_create_layer = 0
            self._span_create_anchor_time_ms = 0
            self._span_create_current_time_ms = 0
            self._span_create_dragging = False

        def set_timeline_data(
            self,
            project: ShowProject | None,
            waveform: AudioWaveform | None,
            current_time_ms: int,
            selected_role: str,
            auto_scroll: bool = True,
        ) -> None:
            clip_counts = None
            if project is not None:
                clip_counts = (
                    len(project.global_clips),
                    len(project.role_clips.get("A", ())),
                    len(project.role_clips.get("B", ())),
                    len(project.role_clips.get("C", ())),
                )
            visual_signature = (
                id(project),
                id(waveform),
                selected_role,
                None if project is None else int(project.duration_ms),
                None if project is None else round(float(project.tempo_bpm), 6),
                None if project is None else int(project.beat_offset_ms),
                clip_counts,
            )
            static_changed = visual_signature != self._last_visual_signature
            previous_time_ms = self.current_time_ms
            previous_visible_start = self._visible_start_ms
            self.project = project
            self.waveform = waveform
            self.selected_role = selected_role
            self.current_time_ms = current_time_ms
            self._last_visual_signature = visual_signature
            self._clamp_view_window()
            suppress_auto_scroll = (
                self._drag_clip is not None
                or self._range_selecting
                or self._span_create_role is not None
            )
            if auto_scroll and not suppress_auto_scroll and not self._manual_view_active():
                self._ensure_time_visible(current_time_ms)
            self._emit_viewport_changed()
            if static_changed or previous_visible_start != self._visible_start_ms:
                self.update()
                return
            self._update_playhead_region(previous_time_ms, current_time_ms)

        def _update_playhead_region(self, old_time_ms: int, new_time_ms: int) -> None:
            plot_rect = self._plot_rect()
            old_x = self._aligned_x(self._time_to_x_raw(old_time_ms, plot_rect))
            new_x = self._aligned_x(self._time_to_x_raw(new_time_ms, plot_rect))
            left_bound = plot_rect.left()
            right_bound = plot_rect.right()
            if (
                (old_x < left_bound and new_x < left_bound)
                or (old_x > right_bound and new_x > right_bound)
            ):
                return
            clipped_old_x = max(left_bound, min(right_bound, old_x))
            clipped_new_x = max(left_bound, min(right_bound, new_x))
            left = int(max(0.0, min(clipped_old_x, clipped_new_x) - self._playhead_update_margin_px))
            right = int(
                min(float(self.width()), max(clipped_old_x, clipped_new_x) + self._playhead_update_margin_px)
            )
            if right <= left:
                self.update()
                return
            self.update(left, 0, max(1, right - left), self.height())

        def set_snap_settings(self, enabled: bool, divisor: int) -> None:
            normalized_divisor = max(1, int(divisor))
            if self.snap_enabled == enabled and self.snap_divisor == normalized_divisor:
                return
            self.snap_enabled = enabled
            self.snap_divisor = normalized_divisor
            self.update()

        def set_insert_preview(self, effect_name: str, duration_ms: int) -> None:
            normalized_duration = max(1, int(duration_ms))
            if (
                self.insert_effect_name == effect_name
                and self.insert_effect_duration_ms == normalized_duration
            ):
                return
            self.insert_effect_name = effect_name
            self.insert_effect_duration_ms = normalized_duration
            self.update()

        def set_active_lane(self, role: str, layer: int) -> None:
            normalized_role = role if role in {"*", "A", "B", "C"} else "*"
            normalized_layer = max(0, int(layer))
            if (
                self._active_lane_role == normalized_role
                and self._active_lane_layer == normalized_layer
            ):
                return
            self._active_lane_role = normalized_role
            self._active_lane_layer = normalized_layer

        def current_insert_target(self) -> tuple[str, int] | None:
            if self._active_lane_role is None:
                return None
            return (self._active_lane_role, self._active_lane_layer)

        def current_insert_time_ms(self) -> int:
            return max(0, int(self._insert_time_ms))

        def set_insert_cursor(self, role: str, layer: int, time_ms: int) -> None:
            self.set_active_lane(role, layer)
            normalized_time_ms = max(0, int(time_ms))
            if self._insert_time_ms == normalized_time_ms:
                self.update()
                return
            self._insert_time_ms = normalized_time_ms
            self.update()

        def zoom_in(self, anchor_time_ms: int | None = None) -> None:
            self._set_zoom_around_time(1.25, self.current_time_ms if anchor_time_ms is None else anchor_time_ms)

        def zoom_out(self, anchor_time_ms: int | None = None) -> None:
            self._set_zoom_around_time(1.0 / 1.25, self.current_time_ms if anchor_time_ms is None else anchor_time_ms)

        def reset_zoom(self, anchor_time_ms: int | None = None) -> None:
            anchor = self.current_time_ms if anchor_time_ms is None else anchor_time_ms
            if abs(self._zoom_factor - 1.0) < 1e-6:
                return
            self._hold_manual_view()
            old_visible = self._visible_duration_ms()
            anchor_ratio = 0.5
            if old_visible > 0:
                anchor_ratio = (anchor - self._visible_start_ms) / old_visible
            anchor_ratio = max(0.0, min(1.0, anchor_ratio))
            self._zoom_factor = 1.0
            new_visible = self._visible_duration_ms()
            self._visible_start_ms = int(round(anchor - anchor_ratio * new_visible))
            self._clamp_view_window()
            self._emit_viewport_changed()
            self.update()

        def set_selected_clip(self, clip) -> None:
            new_ids = frozenset({id(clip)}) if clip is not None else frozenset()
            if clip is self.selected_clip and new_ids == self.selected_clip_ids:
                return
            self.selected_clip = clip
            self.selected_clip_ids = new_ids
            self.update()

        def set_selected_clips(self, clips) -> None:
            clips_tuple = tuple(clip for clip in clips if clip is not None)
            selected_clip = clips_tuple[0] if clips_tuple else None
            selected_ids = frozenset(id(clip) for clip in clips_tuple)
            if selected_clip is self.selected_clip and selected_ids == self.selected_clip_ids:
                return
            self.selected_clip = selected_clip
            self.selected_clip_ids = selected_ids
            self.update()

        def _all_display_clips(self) -> tuple:
            lanes = self._lane_rows()
            ordered = []
            for role in ("*", "A", "B", "C"):
                ordered.extend(row.clip for row in lanes[role])
            return tuple(ordered)

        def _selected_clips_in_display_order(self) -> tuple:
            selected_ids = self.selected_clip_ids
            if not selected_ids:
                return ()
            return tuple(
                clip for clip in self._all_display_clips()
                if id(clip) in selected_ids
            )

        def _emit_selection_changed(self) -> None:
            selected_clips = self._selected_clips_in_display_order()
            if self.clip_selection_changed_callback is not None:
                self.clip_selection_changed_callback(selected_clips, self.selected_clip)
                return
            if self.clip_selected_callback is not None:
                self.clip_selected_callback(self.selected_clip)

        def viewport_state(self) -> tuple[int, int, int]:
            return (
                int(self._visible_start_ms),
                int(self._visible_duration_ms()),
                int(self._max_visible_start_ms()),
            )

        def set_visible_start_ms(self, start_ms: int) -> None:
            self._hold_manual_view()
            new_start = max(0, min(int(start_ms), self._max_visible_start_ms()))
            if new_start == self._visible_start_ms:
                return
            self._visible_start_ms = new_start
            self._emit_viewport_changed()
            self.update()

        def _duration_ms(self) -> int:
            if self.waveform is not None:
                return max(1, int(self.waveform.duration_ms))
            duration_ms = 1
            if self.project is not None:
                duration_ms = max(duration_ms, self.project.duration_ms)
            return duration_ms

        def _visible_duration_ms(self) -> int:
            duration_ms = self._duration_ms()
            return max(125, int(round(duration_ms / max(1.0, self._zoom_factor))))

        def _max_visible_start_ms(self) -> int:
            return max(0, self._duration_ms() - self._visible_duration_ms())

        def _clamp_view_window(self) -> None:
            self._visible_start_ms = max(0, min(self._visible_start_ms, self._max_visible_start_ms()))

        def _hold_manual_view(self, seconds: float = 1.25) -> None:
            self._manual_view_until_s = max(self._manual_view_until_s, time.monotonic() + max(0.1, seconds))

        def _manual_view_active(self) -> bool:
            return time.monotonic() < self._manual_view_until_s

        def _emit_viewport_changed(self) -> None:
            state = self.viewport_state()
            if state == self._last_viewport_state:
                return
            self._last_viewport_state = state
            if self.viewport_changed_callback is not None:
                self.viewport_changed_callback(*state)

        def _ensure_time_visible(self, time_ms: int) -> None:
            visible = self._visible_duration_ms()
            margin = int(visible * 0.08)
            start = self._visible_start_ms
            end = start + visible
            if time_ms < start + margin:
                self._visible_start_ms = max(0, time_ms - margin)
            elif time_ms > end - margin:
                self._visible_start_ms = min(self._max_visible_start_ms(), time_ms - visible + margin)
            self._clamp_view_window()

        def _follow_time(self, time_ms: int, anchor_ratio: float = 0.18) -> None:
            visible = self._visible_duration_ms()
            self._visible_start_ms = int(round(time_ms - visible * max(0.05, min(0.45, anchor_ratio))))
            self._clamp_view_window()

        def _pan_window_ms(self, delta_ms: int) -> None:
            self._hold_manual_view()
            self._visible_start_ms += int(delta_ms)
            self._clamp_view_window()
            self._emit_viewport_changed()
            self.update()

        def _set_zoom_around_time(self, factor: float, anchor_time_ms: int) -> None:
            self._hold_manual_view()
            new_zoom = max(1.0, min(256.0, self._zoom_factor * factor))
            if abs(new_zoom - self._zoom_factor) < 1e-6:
                return
            old_visible = self._visible_duration_ms()
            anchor_ratio = 0.5
            if old_visible > 0:
                anchor_ratio = (anchor_time_ms - self._visible_start_ms) / old_visible
            anchor_ratio = max(0.0, min(1.0, anchor_ratio))
            self._zoom_factor = new_zoom
            new_visible = self._visible_duration_ms()
            self._visible_start_ms = int(round(anchor_time_ms - anchor_ratio * new_visible))
            self._clamp_view_window()
            self._emit_viewport_changed()
            self.update()

        def _plot_rect(self) -> QtCore.QRectF:
            return QtCore.QRectF(
                80.0,
                18.0,
                max(40.0, self.width() - 100.0),
                max(120.0, self.height() - 36.0),
            )

        def _time_to_x_raw(self, time_ms: int, rect: QtCore.QRectF) -> float:
            visible = self._visible_duration_ms()
            ratio = (time_ms - self._visible_start_ms) / max(1, visible)
            return rect.left() + rect.width() * ratio

        def _time_to_x(self, time_ms: int, rect: QtCore.QRectF) -> float:
            return max(rect.left(), min(rect.right(), self._time_to_x_raw(time_ms, rect)))

        def _x_to_time(self, x: float, rect: QtCore.QRectF) -> int:
            visible = self._visible_duration_ms()
            ratio = (x - rect.left()) / max(1.0, rect.width())
            time_ms = self._visible_start_ms + max(0.0, min(1.0, ratio)) * visible
            return int(round(time_ms))

        def _aligned_x(self, x: float) -> float:
            return float(int(round(x))) + 0.5

        def _tick_ms(self) -> int:
            duration_ms = self._visible_duration_ms()
            for candidate in (250, 500, 1000, 2000, 5000, 10000, 15000, 30000, 60000):
                if duration_ms / candidate <= 14:
                    return candidate
            return 120000

        def _beat_ms(self) -> float | None:
            if self.project is None:
                return None
            if self.project.tempo_bpm <= 0.0:
                return None
            return 60000.0 / self.project.tempo_bpm

        def _hover_text(self) -> str:
            if self._hover_time_ms is None:
                return ""
            parts = [f"{self._hover_time_ms / 1000.0:.3f}s"]
            beat_ms = self._beat_ms()
            if beat_ms is not None:
                beat_value = 1.0 + (self._hover_time_ms - self.project.beat_offset_ms) / beat_ms  # type: ignore[union-attr]
                parts.append(f"beat {beat_value:.2f}")
            if self._hover_snap_time_ms is not None and (
                self.snap_enabled or self._hover_snap_time_ms != self._hover_time_ms
            ):
                parts.append(f"snap {self._hover_snap_time_ms / 1000.0:.3f}s")
            if self._hover_role is not None:
                lane_text = "Global" if self._hover_role == "*" else f"Suit {self._hover_role}"
                if self._hover_layer is not None:
                    lane_text += f" T{self._hover_layer + 1}"
                parts.append(lane_text)
            if self._hover_role is not None and self.insert_effect_name:
                parts.append(f"add {self.insert_effect_name}")
            return " | ".join(parts)

        def _lane_name(self, role: str) -> str:
            return "Global" if role == "*" else f"Suit {role}"

        def _lane_color(self, role: str) -> QtGui.QColor:
            colors = {
                "*": QtGui.QColor("#8b949e"),
                "A": QtGui.QColor("#ff9f45"),
                "B": QtGui.QColor("#ff5d8f"),
                "C": QtGui.QColor("#43d17a"),
            }
            return colors.get(role, QtGui.QColor("#8b949e"))

        def _clip_fill_color(self, role: str, clip) -> QtGui.QColor:
            del role
            rgba = clip.params.get("color", [255, 255, 255, 255]) if clip is not None else [255, 255, 255, 255]
            if not isinstance(rgba, (list, tuple)) or len(rgba) != 4:
                rgba = [255, 255, 255, 255]
            try:
                red, green, blue, alpha = (max(0, min(255, int(channel))) for channel in rgba)
            except (TypeError, ValueError):
                red, green, blue, alpha = (255, 255, 255, 255)
            return QtGui.QColor(red, green, blue, max(72, alpha))

        def _text_color_for_fill(self, fill: QtGui.QColor) -> QtGui.QColor:
            luminance = 0.2126 * fill.red() + 0.7152 * fill.green() + 0.0722 * fill.blue()
            if luminance >= 150.0:
                return QtGui.QColor("#091018")
            return QtGui.QColor("#f2f6fb")

        def _clip_preview_colors(self, clip, sample_count: int = 7) -> tuple[QtGui.QColor, ...]:
            if self.project is None or clip is None:
                return (QtGui.QColor(255, 255, 255, 255),)
            return tuple(
                QtGui.QColor(r, g, b, max(72, a))
                for r, g, b, a in clip_color_preview_rgba8(self.project, clip, sample_count)
            )

        def _clip_fill_sample_count(self, clip_rect: QtCore.QRectF, clip, visible_fraction: float = 1.0) -> int:
            if clip is None or self.project is None:
                return 1
            width_px = max(1, int(round(clip_rect.width())))
            color_mode = str(clip.params.get("color_mode", "hold")).strip().lower()
            if color_mode != "cycle":
                return max(1, min(24, max(3, width_px // 10)))
            duration_ms = max(1, int(clip.end_ms) - int(clip.start_ms))
            rate = max(0.0, float(clip.params.get("color_rate", 1.0) or 0.0))
            tempo_sync = bool(clip.params.get("color_tempo_sync", True))
            estimated_cycles = 1.0
            if tempo_sync and float(self.project.tempo_bpm) > 0.0:
                beat_ms = 60000.0 / float(self.project.tempo_bpm)
                estimated_cycles = max(1.0, (duration_ms / beat_ms) * max(0.0625, rate))
            elif rate > 0.0:
                estimated_cycles = max(1.0, (duration_ms / 1000.0) * rate)
            estimated_cycles = max(1.0, estimated_cycles * max(0.0, min(1.0, visible_fraction)))
            samples_from_cycles = int(math.ceil(estimated_cycles * 24.0))
            samples_from_width = max(16, width_px // 2)
            return max(8, min(256, max(samples_from_cycles, samples_from_width)))

        def _clip_fill_brush(self, clip_rect: QtCore.QRectF, clip) -> tuple[QtGui.QBrush, QtGui.QColor]:
            colors = self._clip_preview_colors(clip, self._clip_fill_sample_count(clip_rect, clip))
            if len(colors) <= 1:
                base_color = colors[0]
                return (QtGui.QBrush(base_color), base_color)
            gradient = QtGui.QLinearGradient(clip_rect.left(), clip_rect.top(), clip_rect.right(), clip_rect.top())
            count = len(colors)
            for index, color in enumerate(colors):
                stop_t = 0.0 if count <= 1 else index / (count - 1)
                gradient.setColorAt(stop_t, color)
            return (QtGui.QBrush(gradient), colors[count // 2])

        def _paint_clip_fill(
            self,
            painter: QtGui.QPainter,
            clip_rect: QtCore.QRectF,
            clip,
            plot_rect: QtCore.QRectF,
        ) -> QtGui.QColor:
            inner_rect = clip_rect.adjusted(1.0, 1.0, -1.0, -1.0)
            if self.project is None or clip is None or inner_rect.width() <= 1.0 or inner_rect.height() <= 1.0:
                fill_brush, preview_color = self._clip_fill_brush(inner_rect, clip)
                painter.fillRect(inner_rect, fill_brush)
                return preview_color

            visible_rect = inner_rect.intersected(
                QtCore.QRectF(plot_rect.left(), inner_rect.top(), plot_rect.width(), inner_rect.height())
            )
            if visible_rect.isEmpty() or visible_rect.width() <= 0.0:
                fill_brush, preview_color = self._clip_fill_brush(inner_rect, clip)
                painter.fillRect(inner_rect, fill_brush)
                return preview_color

            full_width = max(1e-6, inner_rect.width())
            visible_fraction = visible_rect.width() / full_width
            sample_count = self._clip_fill_sample_count(visible_rect, clip, visible_fraction)
            duration_ms = max(1, int(clip.end_ms) - int(clip.start_ms))
            preview_color = QtGui.QColor(255, 255, 255, 255)
            for sample_index in range(sample_count):
                left_t = sample_index / sample_count
                right_t = (sample_index + 1) / sample_count
                x0 = visible_rect.left() + visible_rect.width() * left_t
                x1 = visible_rect.left() + visible_rect.width() * right_t
                sample_center_x = 0.5 * (x0 + x1)
                sample_t = (sample_center_x - inner_rect.left()) / full_width
                sample_t = max(0.0, min(1.0, sample_t))
                time_ms = int(round(clip.start_ms + duration_ms * sample_t))
                color = clip_color_at(self.project, clip, time_ms, sample_t)
                qcolor = QtGui.QColor(
                    int(round(max(0.0, min(1.0, color[0])) * 255.0)),
                    int(round(max(0.0, min(1.0, color[1])) * 255.0)),
                    int(round(max(0.0, min(1.0, color[2])) * 255.0)),
                    max(72, int(round(max(0.0, min(1.0, color[3])) * 255.0))),
                )
                painter.fillRect(
                    QtCore.QRectF(x0, visible_rect.top(), max(1.0, x1 - x0 + 0.5), visible_rect.height()),
                    qcolor,
                )
                if sample_index == sample_count // 2:
                    preview_color = qcolor
            return preview_color

        def _insert_effect_color(self) -> QtGui.QColor:
            spec = EFFECT_NAME_TO_SPEC.get(self.insert_effect_name, EFFECT_LIBRARY[0])
            params = spec[6] if len(spec) >= 7 else {}
            rgba = params.get("color", [255, 255, 255, 255]) if isinstance(params, dict) else [255, 255, 255, 255]
            if not isinstance(rgba, (list, tuple)) or len(rgba) != 4:
                rgba = [255, 255, 255, 255]
            try:
                red, green, blue, alpha = (max(0, min(255, int(channel))) for channel in rgba)
            except (TypeError, ValueError):
                red, green, blue, alpha = (255, 255, 255, 255)
            return QtGui.QColor(red, green, blue, max(72, alpha))

        def _lane_rows(self) -> dict[str, list]:
            lanes = {"*": [], "A": [], "B": [], "C": []}
            if self.project is None:
                return lanes
            for row in build_timeline_rows(self.project):
                lane_key = row.role if row.role in lanes else "*"
                lanes[lane_key].append(row)
            return lanes

        def _waveform_rect(self, plot_rect: QtCore.QRectF) -> QtCore.QRectF:
            return QtCore.QRectF(plot_rect.left(), plot_rect.top(), plot_rect.width(), 96.0)

        def _time_selection_zone_rect(self, plot_rect: QtCore.QRectF) -> QtCore.QRectF:
            waveform_rect = self._waveform_rect(plot_rect)
            return QtCore.QRectF(
                plot_rect.left(),
                plot_rect.top(),
                plot_rect.width(),
                waveform_rect.bottom() - plot_rect.top() + 18.0,
            )

        def selection_range(self) -> tuple[int, int] | None:
            if self._selection_start_ms is None or self._selection_end_ms is None:
                return None
            start_ms = int(self._selection_start_ms)
            end_ms = int(self._selection_end_ms)
            if end_ms <= start_ms:
                return None
            return (start_ms, end_ms)

        def clear_selection_range(self) -> None:
            if self._selection_start_ms is None and self._selection_end_ms is None:
                return
            self._selection_start_ms = None
            self._selection_end_ms = None
            self.update()

        def _set_selection_range(self, start_ms: int, end_ms: int) -> None:
            start_ms = max(0, int(start_ms))
            end_ms = max(0, int(end_ms))
            self._selection_start_ms = min(start_ms, end_ms)
            self._selection_end_ms = max(start_ms, end_ms)
            self.update()

        def _snap_step_ms(self) -> int:
            return max(1, int(round(self._snap_step_value_ms())))

        def _snap_step_value_ms(self) -> float:
            divisor = max(1, self.snap_divisor)
            beat_ms = self._beat_ms()
            if beat_ms is None:
                if self.project is None:
                    return 1.0
                return max(1.0, float(self.project.bucket_ms) / divisor)
            return max(1.0, float(beat_ms) / divisor)

        def _snap_time_ms(self, time_ms: int) -> int:
            if not self.snap_enabled:
                return max(0, int(time_ms))
            step_ms = self._snap_step_value_ms()
            offset_ms = float(self.project.beat_offset_ms) if self.project is not None else 0.0
            snapped = offset_ms + round((float(time_ms) - offset_ms) / step_ms) * step_ms
            return max(0, int(round(snapped)))

        def _lane_rects(self, plot_rect: QtCore.QRectF) -> dict[str, QtCore.QRectF]:
            top = self._waveform_rect(plot_rect).bottom() + 18.0
            lane_gap = 6.0
            lanes = ("*", "A", "B", "C")
            lane_height = max(34.0, (plot_rect.bottom() - top - lane_gap * (len(lanes) - 1)) / len(lanes))
            return {
                role: QtCore.QRectF(
                    plot_rect.left(),
                    top + index * (lane_height + lane_gap),
                    plot_rect.width(),
                    lane_height,
                )
                for index, role in enumerate(lanes)
            }

        def _lane_layout_metrics(
            self,
            rows: list,
            rect: QtCore.QRectF,
            extra_layer: int | None = None,
        ) -> tuple[int, float, float, float]:
            layer_count = TIMELINE_TRACK_COUNT
            if rows:
                layer_count = max(TIMELINE_TRACK_COUNT, max(row.clip.layer for row in rows) + 1)
            if extra_layer is not None:
                layer_count = max(layer_count, extra_layer + 1)
            inner_top = rect.top() + 6.0
            layer_gap = 3.0
            layer_height = max(10.0, (rect.height() - 12.0 - layer_gap * (layer_count - 1)) / layer_count)
            return layer_count, inner_top, layer_gap, layer_height

        def _preview_clip_rect(
            self,
            role: str,
            rows: list,
            rect: QtCore.QRectF,
            plot_rect: QtCore.QRectF,
            start_ms: int,
            layer: int,
        ) -> QtCore.QRectF:
            _, inner_top, layer_gap, layer_height = self._lane_layout_metrics(rows, rect, extra_layer=layer)
            end_ms = start_ms + max(1, self.insert_effect_duration_ms)
            x0 = self._time_to_x(start_ms, plot_rect)
            x1 = self._time_to_x(end_ms, plot_rect)
            if x1 < plot_rect.left() or x0 > plot_rect.right():
                return QtCore.QRectF()
            clip_rect = QtCore.QRectF(
                max(plot_rect.left(), x0),
                inner_top + layer * (layer_height + layer_gap),
                max(6.0, min(plot_rect.right(), x1) - max(plot_rect.left(), x0)),
                layer_height,
            )
            return clip_rect

        def _span_create_preview_rect(self, role: str, layer: int, plot_rect: QtCore.QRectF) -> QtCore.QRectF:
            lane_rects = self._lane_rects(plot_rect)
            rect = lane_rects.get(role)
            if rect is None:
                return QtCore.QRectF()
            rows = self._lane_rows().get(role, [])
            _, inner_top, layer_gap, layer_height = self._lane_layout_metrics(rows, rect, extra_layer=layer)
            start_ms = min(self._span_create_anchor_time_ms, self._span_create_current_time_ms)
            end_ms = max(self._span_create_anchor_time_ms, self._span_create_current_time_ms)
            x0 = self._time_to_x_raw(start_ms, plot_rect)
            x1 = self._time_to_x_raw(end_ms, plot_rect)
            return QtCore.QRectF(
                x0,
                inner_top + layer * (layer_height + layer_gap),
                max(2.0, x1 - x0),
                layer_height,
            )

        def _draw_waveform(self, painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:
            painter.fillRect(rect, QtGui.QColor("#161b22"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#2d3644"), 1.0))
            painter.drawRect(rect)

            if self.waveform is None or not self.waveform.peaks:
                painter.setPen(QtGui.QColor("#7d8590"))
                painter.drawText(
                    rect.adjusted(10, 10, -10, -10),
                    int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop),
                    "No WAV waveform loaded",
                )
                return

            center_y = rect.center().y()
            visible_start = self._visible_start_ms
            visible_end = visible_start + self._visible_duration_ms()
            column_count = max(2, int(rect.width() * 1.5))
            frame_count = max(1, int(self.waveform.frame_count))
            sample_rate = max(1, int(self.waveform.sample_rate))
            visible_frame_start = max(0, min(frame_count - 1, int(math.floor(visible_start * sample_rate / 1000.0))))
            visible_frame_end = max(
                visible_frame_start + 1,
                min(frame_count, int(math.ceil(visible_end * sample_rate / 1000.0))),
            )
            visible_frame_count = max(1, visible_frame_end - visible_frame_start)
            samples_per_column = visible_frame_count / max(1, column_count)

            if samples_per_column <= 512.0 and len(self.waveform.mono_samples) >= frame_count:
                painter.save()
                painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
                painter.setPen(QtGui.QPen(QtGui.QColor("#7aa2f7"), 1.0))
                amplitude_scale = rect.height() * 0.42
                samples = self.waveform.mono_samples
                for column in range(column_count):
                    column_start = visible_frame_start + int(math.floor(visible_frame_count * (column / column_count)))
                    column_end = visible_frame_start + int(math.ceil(visible_frame_count * ((column + 1) / column_count)))
                    column_start = max(0, min(frame_count - 1, column_start))
                    column_end = max(column_start + 1, min(frame_count, column_end))
                    min_value = 1.0
                    max_value = -1.0
                    for sample_index in range(column_start, column_end):
                        sample_value = float(samples[sample_index])
                        if sample_value < min_value:
                            min_value = sample_value
                        if sample_value > max_value:
                            max_value = sample_value
                    x = rect.left() + rect.width() * ((column + 0.5) / column_count)
                    x = self._aligned_x(x)
                    y0 = center_y - max_value * amplitude_scale
                    y1 = center_y - min_value * amplitude_scale
                    painter.drawLine(QtCore.QPointF(x, y0), QtCore.QPointF(x, y1))
                painter.restore()
            else:
                peaks = self.waveform.peaks
                total_peak_count = len(peaks)
                bucket_size = max(1, int(math.ceil(frame_count / max(1, total_peak_count))))
                amplitude_scale = rect.height() * 0.42
                painter.save()
                painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
                painter.setPen(QtGui.QPen(QtGui.QColor(122, 162, 247, 160), 1.0))
                for column in range(column_count):
                    column_start = visible_frame_start + int(math.floor(visible_frame_count * (column / column_count)))
                    column_end = visible_frame_start + int(math.ceil(visible_frame_count * ((column + 1) / column_count)))
                    column_start = max(0, min(frame_count - 1, column_start))
                    column_end = max(column_start + 1, min(frame_count, column_end))
                    index0 = max(0, min(total_peak_count - 1, column_start // bucket_size))
                    index1 = max(index0 + 1, min(total_peak_count, int(math.ceil(column_end / bucket_size))))
                    peak_value = 0.0
                    for peak in peaks[index0:index1]:
                        if peak > peak_value:
                            peak_value = peak
                    x = rect.left() + rect.width() * ((column + 0.5) / column_count)
                    x = self._aligned_x(x)
                    y0 = center_y - peak_value * amplitude_scale
                    y1 = center_y + peak_value * amplitude_scale
                    painter.drawLine(QtCore.QPointF(x, y0), QtCore.QPointF(x, y1))
                painter.restore()
            painter.drawLine(rect.left(), center_y, rect.right(), center_y)

        def _draw_beat_grid(self, painter: QtGui.QPainter, plot_rect: QtCore.QRectF) -> None:
            if self.project is None:
                return
            beat_ms = self._beat_ms()
            if beat_ms is None:
                return
            divisor = max(1, self.snap_divisor if self.snap_enabled else 1)
            step_ms = self._snap_step_value_ms() if self.snap_enabled else max(1.0, float(beat_ms))
            visible_start = self._visible_start_ms
            visible_end = visible_start + self._visible_duration_ms()
            offset_ms = float(self.project.beat_offset_ms)
            first_index = int(math.floor((visible_start - offset_ms) / step_ms))
            last_index = int(math.ceil((visible_end - offset_ms) / step_ms))
            primary_spacing_ms = step_ms * divisor
            beat_pixel_spacing = plot_rect.width() * (primary_spacing_ms / max(1, self._visible_duration_ms()))
            for step_index in range(first_index, last_index + 1):
                time_ms = offset_ms + step_index * step_ms
                x = self._aligned_x(self._time_to_x(time_ms, plot_rect))
                is_primary = (step_index % divisor) == 0
                color = QtGui.QColor("#2f4358" if is_primary else "#223041")
                if not is_primary:
                    color.setAlpha(110 if self.snap_enabled else 70)
                painter.setPen(QtGui.QPen(color, 1.0))
                painter.drawLine(QtCore.QPointF(x, plot_rect.top()), QtCore.QPointF(x, plot_rect.bottom()))
                if is_primary and beat_pixel_spacing >= 44.0:
                    beat_number = 1 + int(round(step_index / divisor))
                    painter.setPen(QtGui.QColor("#8fa4ba"))
                    painter.drawText(
                        QtCore.QRectF(x + 2.0, self._waveform_rect(plot_rect).top() + 2.0, 42.0, 14.0),
                        int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter),
                        f"b{beat_number}",
                    )

        def _draw_lane(
            self,
            painter: QtGui.QPainter,
            rect: QtCore.QRectF,
            role: str,
            rows: list,
            plot_rect: QtCore.QRectF,
        ) -> None:
            base_fill = QtGui.QColor("#0f1620")
            if role == self.selected_role:
                base_fill = QtGui.QColor("#17202b")
            painter.fillRect(rect, base_fill)
            painter.setPen(QtGui.QPen(QtGui.QColor("#263241"), 1.0))
            painter.drawRect(rect)

            label_rect = QtCore.QRectF(8.0, rect.top(), 60.0, rect.height())
            painter.setPen(QtGui.QColor("#d7dfeb"))
            painter.drawText(
                label_rect,
                int(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight),
                self._lane_name(role),
            )

            layer_count, inner_top, layer_gap, layer_height = self._lane_layout_metrics(rows, rect)
            painter.setPen(QtGui.QPen(QtGui.QColor("#223041"), 1.0))
            for layer_index in range(layer_count):
                layer_top = inner_top + layer_index * (layer_height + layer_gap)
                layer_rect = QtCore.QRectF(rect.left(), layer_top, rect.width(), layer_height)
                if layer_index > 0:
                    painter.drawLine(
                        QtCore.QPointF(rect.left(), layer_top - layer_gap * 0.5),
                        QtCore.QPointF(rect.right(), layer_top - layer_gap * 0.5),
                    )
                painter.setPen(QtGui.QColor("#7d8590"))
                painter.drawText(
                    QtCore.QRectF(rect.left() + 6.0, layer_top, 32.0, layer_height),
                    int(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft),
                    f"T{layer_index + 1}",
                )
                painter.setPen(QtGui.QPen(QtGui.QColor("#223041"), 1.0))

            for row, clip_rect in self._lane_entries(role, rows, rect, plot_rect):
                clip = row.clip
                visible_clip_rect = clip_rect.intersected(
                    QtCore.QRectF(plot_rect.left(), clip_rect.top(), plot_rect.width(), clip_rect.height())
                )
                if visible_clip_rect.isEmpty() or visible_clip_rect.width() <= 0.0:
                    continue
                painter.fillRect(visible_clip_rect, QtGui.QColor("#18212d"))
                preview_color = self._paint_clip_fill(painter, clip_rect, clip, plot_rect)
                if id(row.clip) in self.selected_clip_ids:
                    painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 2.0))
                else:
                    painter.setPen(QtGui.QPen(preview_color.lighter(145), 1.0))
                painter.drawRect(visible_clip_rect)
                if id(row.clip) in self.selected_clip_ids and visible_clip_rect.width() >= 10.0:
                    painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff"), 1.1))
                    left_x = visible_clip_rect.left() + CLIP_RESIZE_HANDLE_PX * 0.5
                    right_x = visible_clip_rect.right() - CLIP_RESIZE_HANDLE_PX * 0.5
                    painter.drawLine(
                        QtCore.QPointF(left_x, visible_clip_rect.top() + 2.0),
                        QtCore.QPointF(left_x, visible_clip_rect.bottom() - 2.0),
                    )
                    painter.drawLine(
                        QtCore.QPointF(right_x, visible_clip_rect.top() + 2.0),
                        QtCore.QPointF(right_x, visible_clip_rect.bottom() - 2.0),
                    )
                if visible_clip_rect.width() > 68.0:
                    painter.setPen(self._text_color_for_fill(preview_color))
                    painter.drawText(
                        visible_clip_rect.adjusted(4.0, 0.0, -4.0, 0.0),
                        int(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft),
                        f"{clip.effect} {clip.target}",
                    )

        def _lane_entries(self, role: str, rows: list, rect: QtCore.QRectF, plot_rect: QtCore.QRectF):
            _, inner_top, layer_gap, layer_height = self._lane_layout_metrics(rows, rect)
            entries = []
            for row in rows:
                x0 = self._time_to_x_raw(row.clip.start_ms, plot_rect)
                x1 = self._time_to_x_raw(row.clip.end_ms, plot_rect)
                clip_rect = QtCore.QRectF(
                    x0,
                    inner_top + row.clip.layer * (layer_height + layer_gap),
                    max(2.0, x1 - x0),
                    layer_height,
                )
                entries.append((row, clip_rect))
            return entries

        def _clip_hit_entries(self):
            plot_rect = self._plot_rect()
            lane_rects = self._lane_rects(plot_rect)
            lanes = self._lane_rows()
            entries = []
            for role in ("*", "A", "B", "C"):
                entries.extend(self._lane_entries(role, lanes[role], lane_rects[role], plot_rect))
            return entries

        def _clip_hit_test(self, pos: QtCore.QPointF):
            for row, clip_rect in reversed(self._clip_hit_entries()):
                if not clip_rect.contains(pos):
                    continue
                hit_kind = "move"
                handle_width = min(CLIP_RESIZE_HANDLE_PX, clip_rect.width() * 0.35)
                if clip_rect.width() >= 8.0:
                    if abs(pos.x() - clip_rect.left()) <= handle_width:
                        hit_kind = "resize_start"
                    elif abs(pos.x() - clip_rect.right()) <= handle_width:
                        hit_kind = "resize_end"
                return row, clip_rect, hit_kind
            return None

        def _lane_hit(self, pos: QtCore.QPointF):
            plot_rect = self._plot_rect()
            lane_rects = self._lane_rects(plot_rect)
            lanes = self._lane_rows()
            for role in ("*", "A", "B", "C"):
                rect = lane_rects[role]
                if not rect.contains(pos):
                    continue
                rows = lanes[role]
                layer_count, inner_top, layer_gap, layer_height = self._lane_layout_metrics(rows, rect)
                relative_y = pos.y() - inner_top
                if relative_y < 0.0:
                    layer = 0
                else:
                    layer = int(relative_y // max(1.0, layer_height + layer_gap))
                layer = max(0, min(layer_count - 1, layer))
                return (
                    role,
                    layer,
                    self._x_to_time(pos.x(), plot_rect),
                )
            return None

        def _request_seek(self, x: float) -> None:
            rect = self._plot_rect()
            if x < rect.left() or x > rect.right():
                return
            if self.seek_callback is not None:
                self.seek_callback(self._x_to_time(x, rect))

        def mousePressEvent(self, event) -> None:  # type: ignore[override]
            self.setFocus(QtCore.Qt.MouseFocusReason)
            if event.button() == QtCore.Qt.LeftButton:
                plot_rect = self._plot_rect()
                if self._time_selection_zone_rect(plot_rect).contains(event.position()):
                    anchor_time_ms = self._x_to_time(event.position().x(), plot_rect)
                    self._range_selecting = True
                    self._range_anchor_time_ms = anchor_time_ms
                    self._set_selection_range(anchor_time_ms, anchor_time_ms)
                    self.setCursor(QtCore.Qt.SizeHorCursor)
                    event.accept()
                    return
                hit = self._clip_hit_test(event.position())
                if hit is not None:
                    row, clip_rect, hit_kind = hit
                    self.set_active_lane(row.role, row.clip.layer)
                    if event.modifiers() & QtCore.Qt.ControlModifier:
                        selected_ids = set(self.selected_clip_ids)
                        clip_id = id(row.clip)
                        if clip_id in selected_ids:
                            selected_ids.remove(clip_id)
                        else:
                            selected_ids.add(clip_id)
                        selected_clips = [
                            clip for clip in self._all_display_clips()
                            if id(clip) in selected_ids
                        ]
                        self.selected_clip_ids = frozenset(selected_ids)
                        self.selected_clip = row.clip if clip_id in selected_ids else (selected_clips[0] if selected_clips else None)
                        self._drag_clip = None
                        self._drag_kind = ""
                        self._drag_origin_role = None
                        self._drag_origin_layer = 0
                        self._drag_history_started = False
                        self._emit_selection_changed()
                        self.update()
                        event.accept()
                        return
                    self.selected_clip = row.clip
                    self.selected_clip_ids = frozenset({id(row.clip)})
                    self._drag_clip = row.clip
                    self._drag_kind = hit_kind
                    self._drag_origin_role = row.role
                    self._drag_origin_layer = row.clip.layer
                    self._drag_anchor_time_ms = self._x_to_time(event.position().x(), self._plot_rect())
                    self._drag_start_ms = row.clip.start_ms
                    self._drag_end_ms = row.clip.end_ms
                    self._drag_history_started = False
                    self._emit_selection_changed()
                    self.update()
                    event.accept()
                    return
                lane_hit = self._lane_hit(event.position())
                if lane_hit is not None:
                    self.set_active_lane(lane_hit[0], lane_hit[1])
                    anchor_time_ms = self._snap_time_ms(lane_hit[2])
                    self._insert_time_ms = anchor_time_ms
                    self._span_create_role = lane_hit[0]
                    self._span_create_layer = lane_hit[1]
                    self._span_create_anchor_time_ms = anchor_time_ms
                    self._span_create_current_time_ms = anchor_time_ms
                    self._span_create_dragging = False
                    self.update()
                    event.accept()
                    return
                self._request_seek(event.position().x())
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
            lane_hit = self._lane_hit(event.position())
            self._hover_role = lane_hit[0] if lane_hit is not None else None
            self._hover_layer = lane_hit[1] if lane_hit is not None else None
            plot_rect = self._plot_rect()
            if plot_rect.contains(event.position()):
                self._hover_time_ms = self._x_to_time(event.position().x(), plot_rect)
                self._hover_snap_time_ms = (
                    self._snap_time_ms(self._hover_time_ms) if lane_hit is not None else None
                )
            else:
                self._hover_time_ms = None
                self._hover_snap_time_ms = None
            if self._range_selecting and (event.buttons() & QtCore.Qt.LeftButton):
                current_time_ms = self._x_to_time(event.position().x(), plot_rect)
                self._set_selection_range(self._range_anchor_time_ms, current_time_ms)
                self.setCursor(QtCore.Qt.SizeHorCursor)
                event.accept()
                return
            if self._span_create_role is not None and (event.buttons() & QtCore.Qt.LeftButton):
                current_time_ms = self._snap_time_ms(self._x_to_time(event.position().x(), self._plot_rect()))
                if lane_hit is not None:
                    self._span_create_role = lane_hit[0]
                    self._span_create_layer = lane_hit[1]
                    self.set_active_lane(lane_hit[0], lane_hit[1])
                self._span_create_current_time_ms = current_time_ms
                minimum_span_ms = max(20, self._snap_step_ms() // 2)
                self._span_create_dragging = abs(self._span_create_current_time_ms - self._span_create_anchor_time_ms) >= minimum_span_ms
                self.setCursor(QtCore.Qt.CrossCursor)
                self.update()
                event.accept()
                return
            if self._drag_clip is not None and (event.buttons() & QtCore.Qt.LeftButton):
                current_time_ms = self._x_to_time(event.position().x(), self._plot_rect())
                max_duration = self._duration_ms()
                target_role = self._drag_origin_role or "*"
                target_layer = self._drag_origin_layer
                if lane_hit is not None:
                    target_role = lane_hit[0]
                    target_layer = lane_hit[1]
                if self._drag_kind == "resize_start":
                    new_start = max(0, min(current_time_ms, self._drag_end_ms - 1))
                    new_end = self._drag_end_ms
                elif self._drag_kind == "resize_end":
                    new_start = self._drag_start_ms
                    new_end = max(self._drag_start_ms + 1, min(max_duration, current_time_ms))
                else:
                    delta = current_time_ms - self._drag_anchor_time_ms
                    duration = max(1, self._drag_end_ms - self._drag_start_ms)
                    new_start = max(0, self._drag_start_ms + delta)
                    if new_start + duration > max_duration:
                        new_start = max(0, max_duration - duration)
                    new_end = new_start + duration
                if not self._drag_history_started and self.clip_edit_started_callback is not None:
                    self.clip_edit_started_callback(self._drag_clip, self._drag_kind or "move")
                    self._drag_history_started = True
                if self.clip_moved_callback is not None:
                    self.clip_moved_callback(
                        self._drag_clip,
                        new_start,
                        new_end,
                        self._drag_kind or "move",
                        target_role,
                        target_layer,
                    )
                self.set_active_lane(target_role, target_layer)
                if self._drag_kind in {"resize_start", "resize_end"}:
                    self.setCursor(QtCore.Qt.SizeHorCursor)
                else:
                    self.setCursor(QtCore.Qt.ClosedHandCursor)
                self.update()
                event.accept()
                return
            hover_hit = self._clip_hit_test(event.position())
            if hover_hit is not None:
                _, _, hit_kind = hover_hit
                if hit_kind in {"resize_start", "resize_end"}:
                    self.setCursor(QtCore.Qt.SizeHorCursor)
                else:
                    self.setCursor(QtCore.Qt.OpenHandCursor)
            elif lane_hit is not None:
                self.setCursor(QtCore.Qt.CrossCursor)
            else:
                self.unsetCursor()
            if event.buttons() & QtCore.Qt.LeftButton:
                self._request_seek(event.position().x())
            self.update()
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
            if self._range_selecting and event.button() == QtCore.Qt.LeftButton:
                self._range_selecting = False
                selection_range = self.selection_range()
                minimum_span_ms = max(20, self._snap_step_ms() // 2)
                if selection_range is None or (selection_range[1] - selection_range[0]) < minimum_span_ms:
                    click_time_ms = self._range_anchor_time_ms
                    self.clear_selection_range()
                    self._request_seek(self._time_to_x(click_time_ms, self._plot_rect()))
                self.unsetCursor()
                event.accept()
                return
            if self._span_create_role is not None and event.button() == QtCore.Qt.LeftButton:
                role = self._span_create_role
                layer = self._span_create_layer
                start_ms = min(self._span_create_anchor_time_ms, self._span_create_current_time_ms)
                end_ms = max(self._span_create_anchor_time_ms, self._span_create_current_time_ms)
                should_create = self._span_create_dragging and (end_ms - start_ms) >= max(20, self._snap_step_ms() // 2)
                self._span_create_role = None
                self._span_create_layer = 0
                self._span_create_anchor_time_ms = 0
                self._span_create_current_time_ms = 0
                self._span_create_dragging = False
                self.unsetCursor()
                if should_create and self.clip_span_create_callback is not None:
                    self.clip_span_create_callback(role, layer, start_ms, max(start_ms + 1, end_ms))
                else:
                    self.set_insert_cursor(role, layer, start_ms)
                self.update()
                event.accept()
                return
            self._drag_clip = None
            self._drag_kind = ""
            self._drag_origin_role = None
            self._drag_origin_layer = 0
            self._drag_history_started = False
            self.unsetCursor()
            super().mouseReleaseEvent(event)

        def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[override]
            if event.button() == QtCore.Qt.LeftButton:
                hit = self._clip_hit_test(event.position())
                if hit is not None:
                    row, clip_rect, hit_kind = hit
                    self.set_active_lane(row.role, row.clip.layer)
                    if self.seek_callback is not None:
                        if hit_kind == "resize_start":
                            boundary_time_ms = row.clip.start_ms
                        elif hit_kind == "resize_end":
                            boundary_time_ms = row.clip.end_ms
                        else:
                            boundary_time_ms = row.clip.end_ms if event.position().x() >= clip_rect.center().x() else row.clip.start_ms
                        self.seek_callback(boundary_time_ms)
                        event.accept()
                        return
                hit = self._lane_hit(event.position())
                if hit is not None and self.clip_create_callback is not None:
                    role, layer, time_ms = hit
                    self.set_active_lane(role, layer)
                    self.clip_create_callback(role, layer, self._snap_time_ms(time_ms))
                    event.accept()
                    return
            super().mouseDoubleClickEvent(event)

        def leaveEvent(self, event) -> None:  # type: ignore[override]
            self._hover_time_ms = None
            self._hover_snap_time_ms = None
            self._hover_role = None
            self._hover_layer = None
            self.unsetCursor()
            self.update()
            super().leaveEvent(event)

        def wheelEvent(self, event) -> None:  # type: ignore[override]
            pixel_delta = event.pixelDelta()
            angle_delta = event.angleDelta()
            delta_x = float(pixel_delta.x()) if not pixel_delta.isNull() else float(angle_delta.x())
            delta_y = float(pixel_delta.y()) if not pixel_delta.isNull() else float(angle_delta.y())
            if delta_x == 0.0 and delta_y == 0.0:
                return
            plot_rect = self._plot_rect()
            anchor_x = max(plot_rect.left(), min(plot_rect.right(), event.position().x()))
            anchor_time_ms = self._x_to_time(anchor_x, plot_rect)
            horizontal_pan = abs(delta_x) > abs(delta_y) and abs(delta_x) > 0.0
            if horizontal_pan or (event.modifiers() & QtCore.Qt.ShiftModifier):
                scroll_delta = delta_x if horizontal_pan else delta_y
                if not pixel_delta.isNull():
                    pixels_per_view = max(1.0, plot_rect.width())
                    pan_delta = -scroll_delta * (self._visible_duration_ms() / pixels_per_view) * 0.65
                else:
                    pan_ratio = 0.18
                    direction = -1.0 if scroll_delta > 0.0 else 1.0
                    pan_delta = self._visible_duration_ms() * pan_ratio * direction
                self._pan_window_ms(int(round(pan_delta)))
            else:
                if not pixel_delta.isNull():
                    zoom_factor = math.pow(1.0015, delta_y)
                else:
                    zoom_factor = 1.25 if delta_y > 0.0 else 1.0 / 1.25
                self._set_zoom_around_time(zoom_factor, anchor_time_ms)
            event.accept()

        def paintEvent(self, event) -> None:  # type: ignore[override]
            del event
            painter = QtGui.QPainter(self)
            painter.fillRect(self.rect(), QtGui.QColor("#0b0f14"))
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

            plot_rect = self._plot_rect()

            self._draw_beat_grid(painter, plot_rect)
            self._draw_waveform(painter, self._waveform_rect(plot_rect))
            selection_range = self.selection_range()
            if selection_range is not None:
                select_x0 = self._aligned_x(self._time_to_x_raw(selection_range[0], plot_rect))
                select_x1 = self._aligned_x(self._time_to_x_raw(selection_range[1], plot_rect))
                selection_rect = QtCore.QRectF(
                    min(select_x0, select_x1),
                    plot_rect.top(),
                    max(1.0, abs(select_x1 - select_x0)),
                    plot_rect.height(),
                ).intersected(plot_rect)
                if not selection_rect.isEmpty():
                    painter.fillRect(selection_rect, QtGui.QColor(122, 162, 247, 36))
                    painter.setPen(QtGui.QPen(QtGui.QColor("#7aa2f7"), 1.2))
                    painter.drawLine(
                        QtCore.QPointF(selection_rect.left(), plot_rect.top()),
                        QtCore.QPointF(selection_rect.left(), plot_rect.bottom()),
                    )
                    painter.drawLine(
                        QtCore.QPointF(selection_rect.right(), plot_rect.top()),
                        QtCore.QPointF(selection_rect.right(), plot_rect.bottom()),
                    )
            lane_rects = self._lane_rects(plot_rect)
            lanes = self._lane_rows()
            for role in ("*", "A", "B", "C"):
                self._draw_lane(painter, lane_rects[role], role, lanes[role], plot_rect)

            insert_target = self.current_insert_target()
            if insert_target is not None and insert_target[0] in lane_rects:
                insert_x = self._aligned_x(self._time_to_x_raw(self.current_insert_time_ms(), plot_rect))
                lane_rect = lane_rects[insert_target[0]]
                if lane_rect.left() <= insert_x <= lane_rect.right():
                    painter.save()
                    painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
                    painter.fillRect(
                        QtCore.QRectF(insert_x - 1.0, lane_rect.top(), 2.0, lane_rect.height()),
                        QtGui.QColor("#ffb347"),
                    )
                    painter.restore()

            if self._span_create_role is not None and self._span_create_dragging:
                preview_rect = self._span_create_preview_rect(self._span_create_role, self._span_create_layer, plot_rect)
                visible_rect = preview_rect.intersected(
                    QtCore.QRectF(plot_rect.left(), preview_rect.top(), plot_rect.width(), preview_rect.height())
                )
                if not visible_rect.isEmpty() and visible_rect.width() > 0.0:
                    ghost_color = self._insert_effect_color()
                    ghost_fill = QtGui.QColor(ghost_color)
                    ghost_fill.setAlpha(88)
                    ghost_pen = QtGui.QPen(ghost_color.lighter(130), 1.4, QtCore.Qt.DashLine)
                    painter.setPen(ghost_pen)
                    painter.setBrush(ghost_fill)
                    painter.drawRoundedRect(visible_rect, 3.0, 3.0)
                    if visible_rect.width() > 80.0:
                        painter.setPen(self._text_color_for_fill(ghost_color))
                        painter.drawText(
                            visible_rect.adjusted(4.0, 0.0, -4.0, 0.0),
                            int(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft),
                            self.insert_effect_name,
                        )

            playhead_x = self._aligned_x(self._time_to_x_raw(self.current_time_ms, plot_rect))
            if plot_rect.left() <= playhead_x <= plot_rect.right():
                painter.save()
                painter.setRenderHint(QtGui.QPainter.Antialiasing, False)
                painter.fillRect(
                    QtCore.QRectF(playhead_x - 1.0, plot_rect.top(), 2.0, plot_rect.height()),
                    QtGui.QColor("#ff4d5a"),
                )
                painter.restore()
            painter.end()


    class PreviewViewportWidget(QtWidgets.QWidget):
        def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
            super().__init__(parent)
            self.setMinimumSize(520, 360)
            self.setFocusPolicy(QtCore.Qt.ClickFocus)
            self._layout_data: LoadedLayout = generated_layout()
            self._frames: dict[str, PreviewFrame] = {}
            self._active_role = "A"
            self._show_all_roles = False
            self._show_body = False
            self._view_mode = "2d"
            self._yaw_deg = 28.0
            self._pitch_deg = -18.0
            self._zoom_factor = 1.0
            self._pan = QtCore.QPointF(0.0, 0.0)
            self._last_drag_pos: QtCore.QPointF | None = None
            self._drag_mode = ""
            self._scene_points_cache: tuple[tuple[float, float, float], ...] = ()
            self._led_geometry_cache: dict[str, tuple[tuple[float, float, float], ...]] = {}
            self._role_label_point_cache: dict[str, tuple[float, float, float]] = {}
            self._geometry_key: tuple[object, ...] | None = None
            self._projection_draw_key: tuple[object, ...] | None = None
            self._projected_led_draw_cache: tuple[tuple[str, int, QtCore.QPointF, float], ...] = ()
            self._projected_body_shape_cache: tuple[tuple[object, ...], ...] = ()
            self._role_body_depth_cache: dict[str, float] = {}
            self._projected_role_label_cache: dict[str, QtCore.QPointF] = {}
            self._dot_pen_cache: dict[tuple[int, int, int, int], QtGui.QPen] = {}
            self._frame_times: deque[float] = deque(maxlen=30)
            self._fps_value = 0.0
            self.setMouseTracking(True)
            self._rebuild_scene_cache()

        def set_preview_data(
            self,
            layout_data: LoadedLayout,
            frames: dict[str, PreviewFrame],
            active_role: str,
            show_all_roles: bool,
        ) -> None:
            self._layout_data = layout_data
            self._frames = dict(frames)
            self._active_role = active_role if active_role in PREVIEW_ROLE_ORDER else PREVIEW_ROLE_ORDER[0]
            self._show_all_roles = show_all_roles
            geometry_key = (
                id(layout_data),
                self._view_mode,
                self._show_all_roles,
                self._active_role,
                tuple(role for role in PREVIEW_ROLE_ORDER if role in self._frames),
            )
            if geometry_key != self._geometry_key:
                self._geometry_key = geometry_key
                self._rebuild_scene_cache()
            self.update()

        def view_mode(self) -> str:
            return self._view_mode

        def show_body(self) -> bool:
            return self._show_body

        def interaction_hint(self) -> str:
            if self._view_mode == "3d":
                return "3D: drag orbit, shift-drag pan, wheel zoom"
            return "2D: drag pan, wheel zoom"

        def set_view_mode(self, mode: str) -> None:
            if mode not in {"2d", "3d"}:
                return
            self._view_mode = mode
            if mode == "2d":
                self._zoom_factor = 1.0
                self._pan = QtCore.QPointF(0.0, 0.0)
            self._rebuild_scene_cache()
            self.update()

        def set_show_body(self, show_body: bool) -> None:
            show_body = bool(show_body)
            if show_body == self._show_body:
                return
            self._show_body = show_body
            self._projection_draw_key = None
            self.update()

        def set_camera_preset(self, preset: str) -> None:
            self._view_mode = "3d"
            if preset == "front":
                self._yaw_deg = 0.0
                self._pitch_deg = 0.0
            elif preset == "back":
                self._yaw_deg = 180.0
                self._pitch_deg = 0.0
            elif preset == "left":
                self._yaw_deg = -90.0
                self._pitch_deg = 0.0
            elif preset == "right":
                self._yaw_deg = 90.0
                self._pitch_deg = 0.0
            else:
                self._yaw_deg = 28.0
                self._pitch_deg = -18.0
            self._zoom_factor = 1.0
            self._pan = QtCore.QPointF(0.0, 0.0)
            self._projection_draw_key = None
            self.update()

        def reset_view(self) -> None:
            if self._view_mode == "3d":
                self.set_camera_preset("iso")
                return
            self._zoom_factor = 1.0
            self._pan = QtCore.QPointF(0.0, 0.0)
            self._projection_draw_key = None
            self.update()

        def _display_point(
            self,
            section_name: str,
            point: tuple[float, float, float],
            section_u: float,
        ) -> tuple[float, float, float]:
            return spatial_display_point(self._view_mode, section_name, point, section_u)

        def _visible_roles(self) -> tuple[str, ...]:
            if not self._frames:
                return ()
            if self._show_all_roles:
                return tuple(role for role in PREVIEW_ROLE_ORDER if role in self._frames)
            if self._active_role in self._frames:
                return (self._active_role,)
            return tuple(role for role in PREVIEW_ROLE_ORDER if role in self._frames)

        def _rebuild_scene_cache(self) -> None:
            visible_roles = self._visible_roles()
            led_geometry_cache: dict[str, tuple[tuple[float, float, float], ...]] = {}
            label_points_cache: dict[str, tuple[float, float, float]] = {}
            scene_points: list[tuple[float, float, float]] = []

            for role in visible_roles:
                role_points: list[tuple[float, float, float]] = []
                for section in self._layout_data.sections:
                    raw_points = list(sample_section_points(section))
                    if section.reversed:
                        raw_points.reverse()
                    total = len(raw_points)
                    if total == 0:
                        continue
                    for index, point in enumerate(raw_points):
                        section_u = 0.5 if total <= 1 else index / (total - 1)
                        display_point = self._apply_role_offset(
                            self._display_point(section.name, point, section_u),
                            role,
                        )
                        role_points.append(display_point)

                led_geometry_cache[role] = tuple(role_points)
                scene_points.extend(role_points)
                if role_points:
                    xs = [point[0] for point in role_points]
                    ys = [point[1] for point in role_points]
                    zs = [point[2] for point in role_points]
                    label_points_cache[role] = (
                        (min(xs) + max(xs)) * 0.5,
                        min(ys) - 8.0,
                        (min(zs) + max(zs)) * 0.5,
                    )

            self._led_geometry_cache = led_geometry_cache
            self._role_label_point_cache = label_points_cache
            self._scene_points_cache = tuple(scene_points)
            self._projection_draw_key = None
            self._projected_led_draw_cache = ()
            self._projected_body_shape_cache = ()
            self._role_body_depth_cache = {}
            self._projected_role_label_cache = {}

        def _dot_pen(self, rgba: tuple[int, int, int, int]) -> QtGui.QPen:
            pen = self._dot_pen_cache.get(rgba)
            if pen is not None:
                return pen
            pen = QtGui.QPen(QtGui.QColor(*rgba), 7.2, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin)
            self._dot_pen_cache[rgba] = pen
            return pen

        def _role_offset(self, role: str) -> tuple[float, float, float]:
            if not self._show_all_roles:
                return (0.0, 0.0, 0.0)
            index = PREVIEW_ROLE_ORDER.index(role)
            center_index = (len(PREVIEW_ROLE_ORDER) - 1) * 0.5
            spacing = PREVIEW_ROLE_SPACING_3D if self._view_mode == "3d" else PREVIEW_ROLE_SPACING_2D
            return ((index - center_index) * spacing, 0.0, 0.0)

        def _apply_role_offset(
            self,
            point: tuple[float, float, float],
            role: str,
        ) -> tuple[float, float, float]:
            offset_x, offset_y, offset_z = self._role_offset(role)
            return (point[0] + offset_x, point[1] + offset_y, point[2] + offset_z)

        def _role_label_point(self, role: str) -> tuple[float, float, float]:
            return self._role_label_point_cache.get(role, self._apply_role_offset((0.0, 0.0, 0.0), role))

        def _projection_cache(self) -> dict[str, object]:
            points = self._scene_points_cache
            if not points:
                return {
                    "center": (0.0, 0.0, 0.0),
                    "scale": 1.0,
                    "offset_x": self.width() * 0.5,
                    "offset_y": self.height() * 0.5,
                    "cos_yaw": 1.0,
                    "sin_yaw": 0.0,
                    "cos_pitch": 1.0,
                    "sin_pitch": 0.0,
                }

            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            zs = [point[2] for point in points]
            center = (
                (min(xs) + max(xs)) * 0.5,
                (min(ys) + max(ys)) * 0.5,
                (min(zs) + max(zs)) * 0.5,
            )

            yaw = math.radians(self._yaw_deg)
            pitch = math.radians(self._pitch_deg)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            cos_pitch = math.cos(pitch)
            sin_pitch = math.sin(pitch)

            projected = [self._transform_point(point, center, cos_yaw, sin_yaw, cos_pitch, sin_pitch) for point in points]
            proj_xs = [point[0] for point in projected]
            proj_ys = [point[1] for point in projected]
            min_x = min(proj_xs)
            max_x = max(proj_xs)
            min_y = min(proj_ys)
            max_y = max(proj_ys)
            span_x = max(1.0, max_x - min_x)
            span_y = max(1.0, max_y - min_y)
            scale = min((self.width() - 40.0) / span_x, (self.height() - 40.0) / span_y)
            scale *= self._zoom_factor
            offset_x = self.width() * 0.5 - (min_x + span_x * 0.5) * scale + self._pan.x()
            offset_y = self.height() * 0.5 - (min_y + span_y * 0.5) * scale + self._pan.y()

            return {
                "center": center,
                "scale": scale,
                "offset_x": offset_x,
                "offset_y": offset_y,
                "cos_yaw": cos_yaw,
                "sin_yaw": sin_yaw,
                "cos_pitch": cos_pitch,
                "sin_pitch": sin_pitch,
            }

        def _transform_point(
            self,
            point: tuple[float, float, float],
            center: tuple[float, float, float],
            cos_yaw: float,
            sin_yaw: float,
            cos_pitch: float,
            sin_pitch: float,
        ) -> tuple[float, float, float]:
            x = point[0] - center[0]
            y = point[1] - center[1]
            z = point[2] - center[2]
            if self._view_mode == "2d":
                return (x, y, z)

            x1 = x * cos_yaw + z * sin_yaw
            z1 = -x * sin_yaw + z * cos_yaw
            y1 = y * cos_pitch - z1 * sin_pitch
            z2 = y * sin_pitch + z1 * cos_pitch
            return (x1, y1, z2)

        def _map_point(
            self,
            point: tuple[float, float, float],
            cache: dict[str, object],
        ) -> tuple[QtCore.QPointF, float]:
            tx, ty, tz = self._transform_point(
                point,
                cache["center"],      # type: ignore[arg-type]
                float(cache["cos_yaw"]),
                float(cache["sin_yaw"]),
                float(cache["cos_pitch"]),
                float(cache["sin_pitch"]),
            )
            scale = float(cache["scale"])
            offset_x = float(cache["offset_x"])
            offset_y = float(cache["offset_y"])
            return (QtCore.QPointF(offset_x + tx * scale, offset_y + ty * scale), tz)

        def _project_body_capsule(
            self,
            start: tuple[float, float, float],
            end: tuple[float, float, float],
            radii: tuple[float, float],
            cache: dict[str, object],
        ) -> tuple[float, tuple[object, ...]] | None:
            start_point, start_depth = self._map_point(start, cache)
            end_point, end_depth = self._map_point(end, cache)
            mid = (
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            )
            mid_point, _ = self._map_point(mid, cache)
            rx, rz = radii
            radius_x_point, _ = self._map_point((mid[0] + rx, mid[1], mid[2]), cache)
            radius_z_point, _ = self._map_point((mid[0], mid[1], mid[2] + rz), cache)
            radius_px = max(
                1.0,
                math.dist(
                    (mid_point.x(), mid_point.y()),
                    (radius_x_point.x(), radius_x_point.y()),
                ),
                math.dist(
                    (mid_point.x(), mid_point.y()),
                    (radius_z_point.x(), radius_z_point.y()),
                ),
            )
            depth = (start_depth + end_depth) * 0.5
            return (depth, ("capsule", start_point, end_point, radius_px))

        def _project_body_torso(
            self,
            top: tuple[float, float, float],
            bottom: tuple[float, float, float],
            radii: tuple[float, float],
            top_round_ratio: float,
            cache: dict[str, object],
        ) -> tuple[float, tuple[object, ...]] | None:
            top_point, top_depth = self._map_point(top, cache)
            bottom_point, bottom_depth = self._map_point(bottom, cache)
            mid = (
                (top[0] + bottom[0]) * 0.5,
                (top[1] + bottom[1]) * 0.5,
                (top[2] + bottom[2]) * 0.5,
            )
            mid_point, _ = self._map_point(mid, cache)
            rx, rz = radii
            radius_x_point, _ = self._map_point((mid[0] + rx, mid[1], mid[2]), cache)
            radius_z_point, _ = self._map_point((mid[0], mid[1], mid[2] + rz), cache)
            half_width_px = max(
                1.0,
                math.dist(
                    (mid_point.x(), mid_point.y()),
                    (radius_x_point.x(), radius_x_point.y()),
                ),
                math.dist(
                    (mid_point.x(), mid_point.y()),
                    (radius_z_point.x(), radius_z_point.y()),
                ),
            )
            top_round_px = max(1.0, half_width_px * max(0.35, min(1.5, float(top_round_ratio))))
            depth = (top_depth + bottom_depth) * 0.5
            return (depth, ("torso", top_point, bottom_point, half_width_px, top_round_px))

        def _point_color_key(self, role: str, led_index: int) -> tuple[int, int, int, int] | None:
            frame = self._frames.get(role)
            if frame is None or led_index >= len(frame.colors_rgba8):
                return None
            color_rgba8 = frame.colors_rgba8[led_index]
            if color_rgba8[3] == 0 or color_rgba8[:3] == (0, 0, 0):
                return (0, 0, 0, 255)
            return (
                int(color_rgba8[0]),
                int(color_rgba8[1]),
                int(color_rgba8[2]),
                int(max(72, color_rgba8[3])),
            )

        def _draw_point_groups(
            self,
            painter: QtGui.QPainter,
            point_groups: dict[tuple[int, int, int, int], list[QtCore.QPointF]],
        ) -> None:
            for color_key, points in point_groups.items():
                if not points:
                    continue
                painter.setPen(self._dot_pen(color_key))
                painter.drawPoints(QtGui.QPolygonF(points))

        def _rebuild_projected_draw_cache(self, visible_roles: tuple[str, ...]) -> None:
            projection_key = (
                self._geometry_key,
                self.width(),
                self.height(),
                self._view_mode,
                self._show_body,
                round(self._yaw_deg, 3),
                round(self._pitch_deg, 3),
                round(self._zoom_factor, 4),
                round(self._pan.x(), 2),
                round(self._pan.y(), 2),
            )
            if projection_key == self._projection_draw_key:
                return
            cache = self._projection_cache()
            draw_items: list[tuple[float, str, int, QtCore.QPointF]] = []
            for role in visible_roles:
                for led_index, point in enumerate(self._led_geometry_cache.get(role, ())):
                    mapped_point, depth = self._map_point(point, cache)
                    draw_items.append((depth, role, led_index, mapped_point))
            draw_items.sort(key=lambda item: item[0])
            self._projected_led_draw_cache = tuple(
                (role, led_index, mapped_point, depth)
                for depth, role, led_index, mapped_point in draw_items
            )
            self._projected_role_label_cache = {
                role: self._map_point(self._role_label_point(role), cache)[0]
                for role in visible_roles
            }
            self._projected_body_shape_cache = ()
            self._role_body_depth_cache = {}
            if self._show_body and self._view_mode == "3d":
                body_shapes: list[tuple[float, tuple[object, ...]]] = []
                role_body_depth: dict[str, float] = {}
                for role in visible_roles:
                    torso_start, torso_end, torso_radii, torso_top_round = PREVIEW_BODY_TORSO
                    torso_mid = (
                        (torso_start[0] + torso_end[0]) * 0.5,
                        (torso_start[1] + torso_end[1]) * 0.5,
                        (torso_start[2] + torso_end[2]) * 0.5,
                    )
                    role_body_depth[role] = self._map_point(self._apply_role_offset(torso_mid, role), cache)[1]
                    torso_shape = self._project_body_torso(
                        self._apply_role_offset(torso_start, role),
                        self._apply_role_offset(torso_end, role),
                        torso_radii,
                        torso_top_round,
                        cache,
                    )
                    if torso_shape is not None:
                        body_shapes.append(torso_shape)
                    for start, end, radii in PREVIEW_BODY_ARM_CAPSULES:
                        projected = self._project_body_capsule(
                            self._apply_role_offset(start, role),
                            self._apply_role_offset(end, role),
                            radii,
                            cache,
                        )
                        if projected is not None:
                            body_shapes.append(projected)
                body_shapes.sort(key=lambda item: item[0])
                self._projected_body_shape_cache = tuple(shape for _, shape in body_shapes)
                self._role_body_depth_cache = role_body_depth
            self._projection_draw_key = projection_key

        def wheelEvent(self, event) -> None:  # type: ignore[override]
            delta = event.angleDelta().y()
            if delta == 0:
                return
            factor = 1.12 if delta > 0 else 1.0 / 1.12
            self._zoom_factor = max(0.2, min(12.0, self._zoom_factor * factor))
            self._projection_draw_key = None
            self.update()
            event.accept()

        def mousePressEvent(self, event) -> None:  # type: ignore[override]
            self.setFocus(QtCore.Qt.MouseFocusReason)
            self._last_drag_pos = event.position()
            if self._view_mode == "3d" and event.button() == QtCore.Qt.LeftButton and not (event.modifiers() & QtCore.Qt.ShiftModifier):
                self._drag_mode = "orbit"
            elif event.button() in (QtCore.Qt.LeftButton, QtCore.Qt.MiddleButton):
                self._drag_mode = "pan"
            else:
                self._drag_mode = ""
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
            if self._last_drag_pos is None:
                super().mouseMoveEvent(event)
                return
            delta = event.position() - self._last_drag_pos
            if self._drag_mode == "orbit":
                self._yaw_deg += delta.x() * 0.45
                self._pitch_deg = max(-85.0, min(85.0, self._pitch_deg + delta.y() * 0.35))
                self._projection_draw_key = None
                self.update()
            elif self._drag_mode == "pan":
                self._pan = self._pan + QtCore.QPointF(delta.x(), delta.y())
                self._projection_draw_key = None
                self.update()
            self._last_drag_pos = event.position()
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
            self._last_drag_pos = None
            self._drag_mode = ""
            super().mouseReleaseEvent(event)

        def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[override]
            self.reset_view()
            super().mouseDoubleClickEvent(event)

        def _update_fps(self) -> None:
            now = time.monotonic()
            self._frame_times.append(now)
            if len(self._frame_times) < 2:
                self._fps_value = 0.0
                return
            elapsed = self._frame_times[-1] - self._frame_times[0]
            if elapsed <= 1e-6:
                self._fps_value = 0.0
                return
            self._fps_value = (len(self._frame_times) - 1) / elapsed

        def paintEvent(self, event) -> None:  # type: ignore[override]
            del event
            self._update_fps()
            painter = QtGui.QPainter(self)
            try:
                painter.fillRect(self.rect(), QtGui.QColor("#11151c"))
                painter.setRenderHint(QtGui.QPainter.Antialiasing, False)

                visible_roles = self._visible_roles()
                if not visible_roles:
                    painter.setPen(QtGui.QColor("#d7dfeb"))
                    painter.drawText(
                        self.rect().adjusted(16, 16, -16, -16),
                        int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop),
                        "No project loaded",
                    )
                    return

                self._rebuild_projected_draw_cache(visible_roles)

                if self._show_body and self._view_mode == "3d" and self._projected_body_shape_cache:
                    back_point_groups: dict[tuple[int, int, int, int], list[QtCore.QPointF]] = {}
                    front_point_groups: dict[tuple[int, int, int, int], list[QtCore.QPointF]] = {}
                    for role, led_index, mapped_point, depth in self._projected_led_draw_cache:
                        color_key = self._point_color_key(role, led_index)
                        if color_key is None:
                            continue
                        threshold = self._role_body_depth_cache.get(role, 0.0)
                        target_groups = front_point_groups if depth >= threshold else back_point_groups
                        target_groups.setdefault(color_key, []).append(mapped_point)

                    self._draw_point_groups(painter, back_point_groups)
                    painter.setBrush(QtCore.Qt.NoBrush)
                    for shape in self._projected_body_shape_cache:
                        if not shape:
                            continue
                        kind = shape[0]
                        if kind == "torso":
                            _, top_point, bottom_point, half_width_px, top_round_px = shape
                            top_point = top_point  # type: ignore[assignment]
                            bottom_point = bottom_point  # type: ignore[assignment]
                            axis_dx = bottom_point.x() - top_point.x()
                            axis_dy = bottom_point.y() - top_point.y()
                            axis_len = math.hypot(axis_dx, axis_dy)
                            if axis_len <= 1e-6:
                                continue
                            axis_x = axis_dx / axis_len
                            axis_y = axis_dy / axis_len
                            normal_x = -axis_y
                            normal_y = axis_x
                            top_base = QtCore.QPointF(
                                top_point.x() + axis_x * top_round_px,
                                top_point.y() + axis_y * top_round_px,
                            )
                            left_bottom = QtCore.QPointF(
                                bottom_point.x() + normal_x * half_width_px,
                                bottom_point.y() + normal_y * half_width_px,
                            )
                            right_bottom = QtCore.QPointF(
                                bottom_point.x() - normal_x * half_width_px,
                                bottom_point.y() - normal_y * half_width_px,
                            )
                            left_top = QtCore.QPointF(
                                top_base.x() + normal_x * half_width_px,
                                top_base.y() + normal_y * half_width_px,
                            )
                            right_top = QtCore.QPointF(
                                top_base.x() - normal_x * half_width_px,
                                top_base.y() - normal_y * half_width_px,
                            )
                            torso_path = QtGui.QPainterPath(left_bottom)
                            torso_path.lineTo(left_top)
                            torso_path.quadTo(top_point, right_top)
                            torso_path.lineTo(right_bottom)
                            torso_path.closeSubpath()
                            painter.setPen(QtGui.QPen(PREVIEW_BODY_OUTLINE, 1.5))
                            painter.setBrush(PREVIEW_BODY_FILL)
                            painter.drawPath(torso_path)
                            continue

                        _, start_point, end_point, radius_px = shape
                        outline_pen = QtGui.QPen(
                            PREVIEW_BODY_OUTLINE,
                            radius_px * 2.0 + 2.0,
                            QtCore.Qt.SolidLine,
                            QtCore.Qt.RoundCap,
                            QtCore.Qt.RoundJoin,
                        )
                        fill_pen = QtGui.QPen(
                            PREVIEW_BODY_FILL,
                            radius_px * 2.0,
                            QtCore.Qt.SolidLine,
                            QtCore.Qt.RoundCap,
                            QtCore.Qt.RoundJoin,
                        )
                        painter.setPen(outline_pen)
                        painter.drawLine(start_point, end_point)
                        painter.setPen(fill_pen)
                        painter.drawLine(start_point, end_point)
                    self._draw_point_groups(painter, front_point_groups)
                else:
                    point_groups: dict[tuple[int, int, int, int], list[QtCore.QPointF]] = {}
                    for role, led_index, mapped_point, _ in self._projected_led_draw_cache:
                        color_key = self._point_color_key(role, led_index)
                        if color_key is None:
                            continue
                        point_groups.setdefault(color_key, []).append(mapped_point)
                    self._draw_point_groups(painter, point_groups)

                if self._show_all_roles:
                    for role in visible_roles:
                        label_point = self._projected_role_label_cache.get(role, QtCore.QPointF(self.width() * 0.5, 20.0))
                        painter.setPen(QtGui.QColor(PREVIEW_ROLE_COLORS.get(role, QtGui.QColor("#d7dfeb"))))
                        painter.drawText(
                            QtCore.QRectF(label_point.x() - 18.0, label_point.y() - 10.0, 36.0, 20.0),
                            int(QtCore.Qt.AlignCenter),
                            role,
                        )

                painter.setPen(QtGui.QColor("#d7dfeb"))
                painter.drawText(
                    QtCore.QRectF(12.0, self.height() - 28.0, self.width() - 24.0, 20.0),
                    int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter),
                    self.interaction_hint(),
                )
                fps_rect = QtCore.QRectF(self.width() - 120.0, 10.0, 108.0, 20.0)
                painter.fillRect(fps_rect, QtGui.QColor(17, 21, 28, 210))
                painter.setPen(QtGui.QColor("#d7dfeb"))
                painter.drawText(
                    fps_rect.adjusted(6.0, 0.0, -6.0, 0.0),
                    int(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter),
                    f"{self._fps_value:5.1f} FPS",
                )
            finally:
                painter.end()


    class ShowEditorMainWindow(QtWidgets.QMainWindow):
        def __init__(
            self,
            project: ShowProject | None = None,
            startup_session: EditorSessionState | None = None,
        ) -> None:
            super().__init__()
            self.setWindowTitle("Suit Show Editor")
            self.resize(1440, 920)
            self.project = project
            self._startup_session = startup_session
            self.preview_layout_data: LoadedLayout = generated_layout()
            self._layout_source_kind = "generated"
            self._layout_source_path: Path | None = None
            self.audio_waveform: AudioWaveform | None = None
            self.audio_player = None
            self.audio_output = None
            self._ignore_time_slider_change = False
            self._ignore_clip_editor_changes = False
            self.selected_clip = None
            self.timeline_rows = []
            self.clip_clipboard: dict | None = None
            self.effect_presets: list[dict] = []
            self._selected_clip_ids: set[int] = set()
            self._last_single_preview_role = ROLE_NAMES[0]
            self._last_audio_ui_update_s = 0.0
            self._playback_anchor_pos_ms = 0
            self._playback_anchor_monotonic_s = 0.0
            self._playback_anchor_valid = False
            self._last_backend_audio_pos_ms = 0
            self._playback_range_ms: tuple[int, int] | None = None
            self._time_slider_scrubbing = False
            self._ignore_timeline_scrollbar_change = False
            self._pending_preview_time_ms = 0
            self._preview_refresh_queued = False
            self._preview_request_counter = 0
            self._preview_worker_busy = False
            self._pending_worker_request: dict | None = None
            self._undo_stack: list[dict] = []
            self._redo_stack: list[dict] = []
            self._history_limit = 128
            self._history_restoring = False
            self._pending_history_entry: dict | None = None
            self._clip_editor_dirty = False
            self._saved_project_signature: str | None = None
            self._preview_refresh_timer = QtCore.QTimer(self)
            self._preview_refresh_timer.setSingleShot(True)
            self._preview_refresh_timer.setInterval(8)
            self._preview_refresh_timer.timeout.connect(self._flush_preview_refresh)
            self._playback_sync_timer = QtCore.QTimer(self)
            self._playback_sync_timer.setSingleShot(False)
            self._playback_sync_timer.setInterval(16)
            self._playback_sync_timer.timeout.connect(self._on_playback_sync_timer)
            self._session_save_timer = QtCore.QTimer(self)
            self._session_save_timer.setSingleShot(True)
            self._session_save_timer.setInterval(1200)
            self._session_save_timer.timeout.connect(self._write_editor_session)

            self._setup_audio_backend()
            self._setup_preview_worker()
            self._build_ui()
            self._create_actions()
            self._create_menus()
            self._wire_controls()
            self._update_view_buttons()
            self._update_transport_state()
            self._set_target_list_items("all")
            self._refresh_preset_list()
            self._refresh_color_palette_preset_combo()
            self._update_clip_color_buttons()
            self._update_color_mode_ui()
            self.statusBar().showMessage("Ready")

            if self.project is not None:
                self.set_project(self.project)
                if self._startup_session is not None:
                    self._apply_editor_session_state(self._startup_session)

        def _setup_audio_backend(self) -> None:
            if not QT_MULTIMEDIA_AVAILABLE or QtMultimedia is None:
                return
            queued = (
                QtCore.Qt.ConnectionType.QueuedConnection
                if hasattr(QtCore.Qt, "ConnectionType")
                else QtCore.Qt.QueuedConnection
            )
            self.audio_output = QtMultimedia.QAudioOutput(self)
            self.audio_output.setVolume(0.85)
            self.audio_player = QtMultimedia.QMediaPlayer(self)
            self.audio_player.setAudioOutput(self.audio_output)
            self.audio_player.positionChanged.connect(self._on_audio_position_changed, queued)
            self.audio_player.durationChanged.connect(self._on_audio_duration_changed, queued)
            self.audio_player.playbackStateChanged.connect(self._on_audio_playback_state_changed, queued)
            if hasattr(self.audio_player, "errorOccurred"):
                self.audio_player.errorOccurred.connect(self._on_audio_error, queued)

        def _setup_preview_worker(self) -> None:
            queued = (
                QtCore.Qt.ConnectionType.QueuedConnection
                if hasattr(QtCore.Qt, "ConnectionType")
                else QtCore.Qt.QueuedConnection
            )
            self._preview_render_thread = QtCore.QThread(self)
            self._preview_render_proxy = PreviewRenderProxy()
            self._preview_render_worker = PreviewRenderWorker()
            self._preview_render_worker.moveToThread(self._preview_render_thread)
            self._preview_render_proxy.render_requested.connect(self._preview_render_worker.render_request, queued)
            self._preview_render_worker.rendered.connect(self._on_preview_rendered, queued)
            self._preview_render_thread.start()

        def _build_ui(self) -> None:
            self.effects_list = QtWidgets.QTreeWidget()
            self.effects_list.setHeaderHidden(True)
            self.effects_list.setRootIsDecorated(True)
            self.effects_list.setUniformRowHeights(True)
            self._populate_effect_tree()
            self.new_clip_lane_combo = QtWidgets.QComboBox()
            for role_code in CLIP_ROLE_OPTIONS:
                self.new_clip_lane_combo.addItem("Global" if role_code == "*" else f"Suit {role_code}", role_code)
            self.add_effect_button = QtWidgets.QPushButton("Add At Playhead")
            self.presets_list = QtWidgets.QListWidget()
            self.save_preset_button = QtWidgets.QPushButton("Save Selected")
            self.add_preset_button = QtWidgets.QPushButton("Add Preset")
            self.delete_preset_button = QtWidgets.QPushButton("Delete Preset")
            self.snap_checkbox = QtWidgets.QCheckBox("Snap")
            self.snap_checkbox.setChecked(True)
            self.snap_division_combo = QtWidgets.QComboBox()
            self.snap_division_combo.addItem("Beat", 1)
            self.snap_division_combo.addItem("1/2", 2)
            self.snap_division_combo.addItem("1/4", 4)
            self.snap_division_combo.addItem("1/8", 8)

            effects_panel = QtWidgets.QWidget()
            effects_layout = QtWidgets.QVBoxLayout(effects_panel)
            effects_layout.setContentsMargins(8, 8, 8, 8)
            effects_layout.addWidget(QtWidgets.QLabel("Effects"))
            effects_layout.addWidget(self.effects_list, 1)
            lane_row = QtWidgets.QHBoxLayout()
            lane_row.addWidget(QtWidgets.QLabel("Track"))
            lane_row.addWidget(self.new_clip_lane_combo, 1)
            effects_layout.addLayout(lane_row)
            effects_layout.addWidget(self.add_effect_button)
            effects_layout.addSpacing(10)
            effects_layout.addWidget(QtWidgets.QLabel("Presets"))
            effects_layout.addWidget(self.presets_list, 1)
            preset_buttons = QtWidgets.QHBoxLayout()
            preset_buttons.addWidget(self.save_preset_button)
            preset_buttons.addWidget(self.add_preset_button)
            preset_buttons.addWidget(self.delete_preset_button)
            effects_layout.addLayout(preset_buttons)

            self.clip_status_label = QtWidgets.QLabel("No clip selected")
            self.clip_role_combo = QtWidgets.QComboBox()
            for role_code in CLIP_ROLE_OPTIONS:
                self.clip_role_combo.addItem("Global" if role_code == "*" else f"Suit {role_code}", role_code)
            self.clip_effect_value_label = QtWidgets.QLabel("-")
            self.clip_target_mode_combo = QtWidgets.QComboBox()
            for label, value in TARGET_MODE_OPTIONS:
                self.clip_target_mode_combo.addItem(label, value)
            self.clip_target_list = QtWidgets.QListWidget()
            self.clip_target_list.setAlternatingRowColors(True)
            self.clip_target_list.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
            self.clip_target_list.setMinimumHeight(88)
            self.clip_blend_combo = QtWidgets.QComboBox()
            self.clip_blend_combo.addItems(BLEND_OPTIONS)
            self.clip_blend_combo.setCurrentText("max")
            self.clip_start_spin = QtWidgets.QSpinBox()
            self.clip_start_spin.setRange(0, 3_600_000)
            self.clip_end_spin = QtWidgets.QSpinBox()
            self.clip_end_spin.setRange(1, 3_600_000)
            self.clip_layer_spin = QtWidgets.QSpinBox()
            self.clip_layer_spin.setRange(0, TIMELINE_TRACK_COUNT - 1)
            self.clip_r_spin = QtWidgets.QSpinBox()
            self.clip_g_spin = QtWidgets.QSpinBox()
            self.clip_b_spin = QtWidgets.QSpinBox()
            self.clip_a_spin = QtWidgets.QSpinBox()
            self.clip_to_r_spin = QtWidgets.QSpinBox()
            self.clip_to_g_spin = QtWidgets.QSpinBox()
            self.clip_to_b_spin = QtWidgets.QSpinBox()
            self.clip_to_a_spin = QtWidgets.QSpinBox()
            self.clip_curve_text_edit = QtWidgets.QLineEdit()
            self.clip_curve_text_edit.setPlaceholderText("intensity=0:0,0.2:1,1:0; width=0:0.12,1:0.28")
            self.clip_curve_text_edit.setToolTip("Manual parameter curves over clip time. Format: key=t:v,t:v; key=t:v,t:v")
            self.clip_curve_edit_button = QtWidgets.QPushButton("Edit...")
            self.clip_palette_text_edit = QtWidgets.QLineEdit()
            self.clip_palette_text_edit.setPlaceholderText("#ff5500; #00aaff; #ffffff")
            self.clip_palette_text_edit.setToolTip("Palette colors in order. Stops are evenly spaced automatically.")
            self.clip_palette_summary_label = QtWidgets.QLabel("No palette")
            self.clip_palette_summary_label.setWordWrap(True)
            for spin in (
                self.clip_r_spin,
                self.clip_g_spin,
                self.clip_b_spin,
                self.clip_a_spin,
                self.clip_to_r_spin,
                self.clip_to_g_spin,
                self.clip_to_b_spin,
                self.clip_to_a_spin,
            ):
                spin.setRange(0, 255)
            self.clip_intensity_spin = SliderFieldWidget()
            self.clip_intensity_spin.setRange(0.0, 4.0)
            self.clip_intensity_spin.setSingleStep(0.05)
            self.clip_intensity_spin.setDecimals(2)
            self.clip_axis_combo = QtWidgets.QComboBox()
            self.clip_axis_combo.addItems(["y", "x", "z", "section_u", "radial", "random_xy"])
            self.clip_width_spin = SliderFieldWidget()
            self.clip_width_spin.setRange(0.01, 2.0)
            self.clip_width_spin.setSingleStep(0.01)
            self.clip_width_spin.setDecimals(3)
            self.clip_softness_spin = SliderFieldWidget()
            self.clip_softness_spin.setRange(0.0, 2.0)
            self.clip_softness_spin.setSingleStep(0.01)
            self.clip_softness_spin.setDecimals(3)
            self.clip_frequency_spin = SliderFieldWidget()
            self.clip_frequency_spin.setRange(0.0, 40.0)
            self.clip_frequency_spin.setSingleStep(0.1)
            self.clip_frequency_spin.setDecimals(3)
            self.clip_tempo_sync_checkbox = QtWidgets.QCheckBox("Use Project BPM")
            self.clip_tempo_sync_checkbox.setToolTip("Lock this effect's cycle timing to the project BPM and beat offset.")
            self.clip_beats_per_cycle_spin = QtWidgets.QDoubleSpinBox()
            self.clip_beats_per_cycle_spin.setRange(0.0625, 64.0)
            self.clip_beats_per_cycle_spin.setSingleStep(0.25)
            self.clip_beats_per_cycle_spin.setDecimals(3)
            self.clip_beats_per_cycle_spin.setToolTip("How many beats each effect cycle should last when BPM sync is enabled.")
            self.clip_phase_spin = SliderFieldWidget()
            self.clip_phase_spin.setRange(0.0, 1.0)
            self.clip_phase_spin.setSingleStep(0.05)
            self.clip_phase_spin.setDecimals(3)
            self.clip_repeats_spin = QtWidgets.QSpinBox()
            self.clip_repeats_spin.setRange(1, 64)
            self.clip_duty_cycle_spin = SliderFieldWidget()
            self.clip_duty_cycle_spin.setRange(0.0, 1.0)
            self.clip_duty_cycle_spin.setSingleStep(0.05)
            self.clip_duty_cycle_spin.setDecimals(3)
            self.clip_min_intensity_spin = SliderFieldWidget()
            self.clip_min_intensity_spin.setRange(0.0, 4.0)
            self.clip_min_intensity_spin.setSingleStep(0.05)
            self.clip_min_intensity_spin.setDecimals(3)
            self.clip_max_intensity_spin = SliderFieldWidget()
            self.clip_max_intensity_spin.setRange(0.0, 4.0)
            self.clip_max_intensity_spin.setSingleStep(0.05)
            self.clip_max_intensity_spin.setDecimals(3)
            self.clip_reverse_checkbox = QtWidgets.QCheckBox("Reverse")
            self.clip_color_mode_combo = QtWidgets.QComboBox()
            self.clip_color_mode_combo.addItem("Hold", "hold")
            self.clip_color_mode_combo.addItem("Linear", "linear")
            self.clip_color_mode_combo.addItem("Smooth", "smooth")
            self.clip_color_mode_combo.addItem("Cycle", "cycle")
            self.clip_color_from_button = QtWidgets.QPushButton("From Color")
            self.clip_color_to_button = QtWidgets.QPushButton("To Color")
            self.clip_effect_to_color_button = QtWidgets.QPushButton("Effect Target")
            self.clip_color_from_button.setCheckable(True)
            self.clip_color_to_button.setCheckable(True)
            self.clip_color_palette_preset_combo = QtWidgets.QComboBox()
            self.clip_color_palette_save_button = QtWidgets.QPushButton("Save Preset...")
            self.clip_color_palette_delete_button = QtWidgets.QPushButton("Delete Preset")
            self.clip_color_tempo_sync_checkbox = QtWidgets.QCheckBox("Use Project BPM")
            self.clip_color_tempo_sync_checkbox.setChecked(True)
            self.clip_color_rate_spin = SliderFieldWidget()
            self.clip_color_rate_spin.setRange(0.0, 40.0)
            self.clip_color_rate_spin.setSingleStep(0.1)
            self.clip_color_rate_spin.setDecimals(3)
            self.clip_color_rate_label = QtWidgets.QLabel("Cycles / Beat")
            self.clip_color_picker_target = "from"
            self.clip_embedded_color_picker = QtWidgets.QColorDialog(self)
            self.clip_embedded_color_picker.setOption(QtWidgets.QColorDialog.DontUseNativeDialog, True)
            self.clip_embedded_color_picker.setOption(QtWidgets.QColorDialog.NoButtons, True)
            self.clip_embedded_color_picker.setOption(QtWidgets.QColorDialog.ShowAlphaChannel, True)
            self.clip_embedded_color_picker.setWindowFlags(QtCore.Qt.Widget)
            self.clip_embedded_color_picker.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Preferred,
            )
            self.clip_palette_strip = PaletteStripWidget()
            self.clip_palette_position_spin = QtWidgets.QDoubleSpinBox()
            self.clip_palette_position_spin.setRange(0.0, 1.0)
            self.clip_palette_position_spin.setSingleStep(0.01)
            self.clip_palette_position_spin.setDecimals(3)
            self.clip_palette_position_spin.setPrefix("t=")
            self.clip_palette_position_spin.setEnabled(False)
            self.clip_palette_position_spin.setVisible(False)
            self.clip_palette_delete_stop_button = QtWidgets.QPushButton("Delete Stop")
            self.clip_palette_hint_label = QtWidgets.QLabel(
                "Click the palette bar to add colors, drag stops to reorder them, right-click a stop to remove."
            )
            self.clip_palette_hint_label.setWordWrap(True)
            self.reset_clip_params_button = QtWidgets.QPushButton("Reset Effect Defaults")
            self.delete_clip_button = QtWidgets.QPushButton("Delete")

            self.clip_panel = QtWidgets.QWidget()
            clip_layout = QtWidgets.QVBoxLayout(self.clip_panel)
            clip_layout.setContentsMargins(8, 8, 8, 8)
            clip_layout.addWidget(self.clip_status_label)
            self.clip_tabs = QtWidgets.QTabWidget()

            self.clip_effect_panel = QtWidgets.QWidget()
            effect_layout = QtWidgets.QVBoxLayout(self.clip_effect_panel)
            effect_layout.setContentsMargins(0, 0, 0, 0)
            clip_form = QtWidgets.QFormLayout()
            self.clip_form = clip_form
            clip_form.addRow("Track", self.clip_role_combo)
            clip_form.addRow("Effect", self.clip_effect_value_label)
            clip_form.addRow("Target", self.clip_target_mode_combo)
            clip_form.addRow("Sections / Groups", self.clip_target_list)
            clip_form.addRow("Blend", self.clip_blend_combo)
            clip_form.addRow("Start ms", self.clip_start_spin)
            clip_form.addRow("End ms", self.clip_end_spin)
            clip_form.addRow("Layer", self.clip_layer_spin)
            to_color_row = QtWidgets.QHBoxLayout()
            to_color_row.addWidget(self.clip_effect_to_color_button)
            self.clip_to_color_widget = QtWidgets.QWidget()
            self.clip_to_color_widget.setLayout(to_color_row)
            clip_form.addRow("Effect Target", self.clip_to_color_widget)
            curve_row = QtWidgets.QHBoxLayout()
            curve_row.addWidget(self.clip_curve_text_edit, 1)
            curve_row.addWidget(self.clip_curve_edit_button)
            self.clip_curve_widget = QtWidgets.QWidget()
            self.clip_curve_widget.setLayout(curve_row)
            clip_form.addRow("Curves", self.clip_curve_widget)
            clip_form.addRow("Intensity", self.clip_intensity_spin)
            clip_form.addRow("Axis", self.clip_axis_combo)
            clip_form.addRow("Width", self.clip_width_spin)
            clip_form.addRow("Softness", self.clip_softness_spin)
            self.clip_frequency_label = QtWidgets.QLabel("Frequency Hz")
            clip_form.addRow(self.clip_frequency_label, self.clip_frequency_spin)
            clip_form.addRow("", self.clip_tempo_sync_checkbox)
            self.clip_beats_per_cycle_label = QtWidgets.QLabel("Beats / Cycle")
            clip_form.addRow(self.clip_beats_per_cycle_label, self.clip_beats_per_cycle_spin)
            clip_form.addRow("Phase", self.clip_phase_spin)
            clip_form.addRow("Repeats", self.clip_repeats_spin)
            clip_form.addRow("Duty Cycle", self.clip_duty_cycle_spin)
            clip_form.addRow("Min Intensity", self.clip_min_intensity_spin)
            clip_form.addRow("Max Intensity", self.clip_max_intensity_spin)
            clip_form.addRow("", self.clip_reverse_checkbox)
            effect_layout.addLayout(clip_form)

            self.clip_color_panel = QtWidgets.QWidget()
            color_layout = QtWidgets.QVBoxLayout(self.clip_color_panel)
            color_layout.setContentsMargins(0, 0, 0, 0)
            color_form = QtWidgets.QFormLayout()
            self.clip_color_form = color_form
            from_color_row = QtWidgets.QHBoxLayout()
            from_color_row.addWidget(self.clip_color_from_button)
            self.clip_color_widget = QtWidgets.QWidget()
            self.clip_color_widget.setLayout(from_color_row)
            color_form.addRow("From", self.clip_color_widget)
            to_color_pick_row = QtWidgets.QHBoxLayout()
            to_color_pick_row.addWidget(self.clip_color_to_button)
            self.clip_color_to_widget = QtWidgets.QWidget()
            self.clip_color_to_widget.setLayout(to_color_pick_row)
            color_form.addRow("To", self.clip_color_to_widget)
            color_form.addRow("Mode", self.clip_color_mode_combo)
            color_form.addRow("", self.clip_color_tempo_sync_checkbox)
            color_form.addRow(self.clip_color_rate_label, self.clip_color_rate_spin)
            color_form.addRow("Palette Preset", self.clip_color_palette_preset_combo)
            color_form.addRow("Palette", self.clip_palette_summary_label)
            palette_button_row = QtWidgets.QHBoxLayout()
            palette_button_row.addWidget(self.clip_color_palette_save_button)
            palette_button_row.addWidget(self.clip_color_palette_delete_button)
            self.clip_palette_widget = QtWidgets.QWidget()
            self.clip_palette_widget.setLayout(palette_button_row)
            color_form.addRow("", self.clip_palette_widget)
            color_layout.addLayout(color_form)
            self.clip_color_picker_widget = self.clip_embedded_color_picker
            color_layout.addWidget(self.clip_color_picker_widget, 1)
            self.clip_palette_editor_widget = QtWidgets.QWidget()
            palette_editor_layout = QtWidgets.QVBoxLayout(self.clip_palette_editor_widget)
            palette_editor_layout.setContentsMargins(0, 0, 0, 0)
            palette_editor_layout.setSpacing(6)
            palette_editor_layout.addWidget(self.clip_palette_strip)
            palette_control_row = QtWidgets.QHBoxLayout()
            palette_control_row.addWidget(self.clip_palette_position_spin)
            palette_control_row.addWidget(self.clip_palette_delete_stop_button)
            palette_control_row.addStretch(1)
            palette_editor_layout.addLayout(palette_control_row)
            palette_editor_layout.addWidget(self.clip_palette_hint_label)
            color_layout.addWidget(self.clip_palette_editor_widget)

            self.clip_tabs.addTab(self.clip_effect_panel, "Effect")
            self.clip_tabs.addTab(self.clip_color_panel, "Color")
            clip_layout.addWidget(self.clip_tabs, 1)
            clip_button_row = QtWidgets.QHBoxLayout()
            clip_button_row.addWidget(self.reset_clip_params_button)
            clip_button_row.addWidget(self.delete_clip_button)
            clip_layout.addLayout(clip_button_row)
            clip_layout.addStretch(1)

            self.sidebar_tabs = QtWidgets.QTabWidget()
            self.sidebar_tabs.addTab(effects_panel, "Effects")
            self.sidebar_tabs.addTab(self.clip_panel, "Inspector")

            self.timeline_view = ShowTimelineWidget()
            self.preview_view = PreviewViewportWidget()

            self.preview_combo = QtWidgets.QComboBox()
            self.preview_combo.addItems(list(PREVIEW_OPTIONS))
            self.preview_body_checkbox = QtWidgets.QCheckBox("Body")
            self.mode_combo = QtWidgets.QComboBox()
            self.mode_combo.addItems(["2D", "3D"])
            self.tempo_bpm_spin = QtWidgets.QDoubleSpinBox()
            self.tempo_bpm_spin.setRange(1.0, 300.0)
            self.tempo_bpm_spin.setDecimals(4)
            self.tempo_bpm_spin.setSingleStep(0.01)
            self.beat_offset_spin = QtWidgets.QSpinBox()
            self.beat_offset_spin.setRange(-3_600_000, 3_600_000)
            self.beat_offset_spin.setSingleStep(10)

            self.time_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.time_slider.setRange(0, 0)

            self.time_label = QtWidgets.QLabel("0.000 s")
            self.timeline_zoom_out_button = QtWidgets.QPushButton("-")
            self.timeline_zoom_out_button.setFixedWidth(28)
            self.timeline_zoom_in_button = QtWidgets.QPushButton("+")
            self.timeline_zoom_in_button.setFixedWidth(28)
            self.timeline_zoom_fit_button = QtWidgets.QPushButton("Fit")
            self.timeline_zoom_fit_button.setFixedWidth(42)
            self.timeline_scrollbar = QtWidgets.QScrollBar(QtCore.Qt.Horizontal)
            self.layout_label = QtWidgets.QLabel("layout: generated")
            self.audio_label = QtWidgets.QLabel("audio: none")

            self.play_button = self._make_tool_button(QtWidgets.QStyle.SP_MediaPlay, "Play", self._play_audio)
            self.pause_button = self._make_tool_button(QtWidgets.QStyle.SP_MediaPause, "Pause", self._pause_audio)
            self.stop_button = self._make_tool_button(QtWidgets.QStyle.SP_MediaStop, "Stop", self._stop_audio)
            self.restart_button = self._make_tool_button(QtWidgets.QStyle.SP_MediaSkipBackward, "Restart", self._restart_audio)

            transport_bar = QtWidgets.QWidget()
            transport_layout = QtWidgets.QHBoxLayout(transport_bar)
            transport_layout.setContentsMargins(0, 0, 0, 0)
            transport_layout.setSpacing(6)
            transport_layout.addWidget(self.play_button)
            transport_layout.addWidget(self.pause_button)
            transport_layout.addWidget(self.stop_button)
            transport_layout.addWidget(self.restart_button)
            transport_layout.addSpacing(12)
            transport_layout.addWidget(QtWidgets.QLabel("Mode"))
            transport_layout.addWidget(self.mode_combo)
            transport_layout.addSpacing(12)
            transport_layout.addWidget(QtWidgets.QLabel("Preview"))
            transport_layout.addWidget(self.preview_combo)
            transport_layout.addWidget(self.preview_body_checkbox)
            transport_layout.addSpacing(12)
            transport_layout.addWidget(self.snap_checkbox)
            transport_layout.addWidget(self.snap_division_combo)
            transport_layout.addSpacing(12)
            transport_layout.addWidget(QtWidgets.QLabel("BPM"))
            transport_layout.addWidget(self.tempo_bpm_spin)
            transport_layout.addWidget(QtWidgets.QLabel("Beat Offset"))
            transport_layout.addWidget(self.beat_offset_spin)
            transport_layout.addStretch(1)
            transport_layout.addWidget(self.layout_label)
            transport_layout.addSpacing(12)
            transport_layout.addWidget(self.audio_label)

            scrub_bar = QtWidgets.QWidget()
            scrub_layout = QtWidgets.QHBoxLayout(scrub_bar)
            scrub_layout.setContentsMargins(0, 0, 0, 0)
            scrub_layout.setSpacing(8)
            scrub_layout.addWidget(QtWidgets.QLabel("Timeline"))
            scrub_layout.addWidget(self.time_slider, 1)
            scrub_layout.addWidget(self.time_label)
            scrub_layout.addSpacing(12)
            scrub_layout.addWidget(QtWidgets.QLabel("Zoom"))
            scrub_layout.addWidget(self.timeline_zoom_out_button)
            scrub_layout.addWidget(self.timeline_zoom_in_button)
            scrub_layout.addWidget(self.timeline_zoom_fit_button)

            preview_panel = QtWidgets.QWidget()
            preview_layout = QtWidgets.QVBoxLayout(preview_panel)
            preview_layout.setContentsMargins(0, 0, 0, 0)
            preview_layout.addWidget(self.preview_view)

            timeline_panel = QtWidgets.QWidget()
            timeline_layout = QtWidgets.QVBoxLayout(timeline_panel)
            timeline_layout.setContentsMargins(0, 0, 0, 0)
            timeline_layout.setSpacing(6)
            timeline_layout.addWidget(self.timeline_view, 1)
            timeline_layout.addWidget(self.timeline_scrollbar)

            editor_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
            editor_splitter.addWidget(preview_panel)
            editor_splitter.addWidget(timeline_panel)
            editor_splitter.setStretchFactor(0, 2)
            editor_splitter.setStretchFactor(1, 1)
            editor_splitter.setSizes([580, 300])

            main_splitter = QtWidgets.QSplitter()
            main_splitter.addWidget(self.sidebar_tabs)
            main_splitter.addWidget(editor_splitter)
            main_splitter.setStretchFactor(0, 0)
            main_splitter.setStretchFactor(1, 1)
            main_splitter.setSizes([320, 1100])

            central = QtWidgets.QWidget()
            root_layout = QtWidgets.QVBoxLayout(central)
            root_layout.setContentsMargins(8, 8, 8, 8)
            root_layout.setSpacing(8)
            root_layout.addWidget(transport_bar)
            root_layout.addWidget(scrub_bar)
            root_layout.addWidget(main_splitter, 1)
            self.setCentralWidget(central)

        def _wire_controls(self) -> None:
            self.time_slider.valueChanged.connect(self._on_time_slider_changed)
            self.time_slider.sliderPressed.connect(self._on_time_slider_pressed)
            self.time_slider.sliderReleased.connect(self._on_time_slider_released)
            self.preview_combo.currentTextChanged.connect(self._on_preview_combo_changed)
            self.preview_body_checkbox.toggled.connect(self._on_preview_body_toggled)
            self.mode_combo.currentTextChanged.connect(self._on_mode_combo_changed)
            self.mode_combo.currentTextChanged.connect(lambda *_: self._schedule_session_save())
            self.tempo_bpm_spin.valueChanged.connect(self._on_tempo_changed)
            self.beat_offset_spin.valueChanged.connect(self._on_beat_offset_changed)
            self.snap_checkbox.toggled.connect(self._refresh_timeline)
            self.snap_division_combo.currentIndexChanged.connect(self._refresh_timeline)
            self.snap_checkbox.toggled.connect(lambda *_: self._schedule_session_save())
            self.snap_division_combo.currentIndexChanged.connect(lambda *_: self._schedule_session_save())
            self.effects_list.itemSelectionChanged.connect(self._refresh_timeline)
            self.timeline_zoom_out_button.clicked.connect(self._zoom_timeline_out)
            self.timeline_zoom_in_button.clicked.connect(self._zoom_timeline_in)
            self.timeline_zoom_fit_button.clicked.connect(self._zoom_timeline_fit)
            self.timeline_scrollbar.valueChanged.connect(self._on_timeline_scrollbar_changed)
            self.timeline_view.seek_callback = self._set_timeline_position
            self.timeline_view.viewport_changed_callback = self._on_timeline_viewport_changed
            self.timeline_view.clip_selected_callback = self._select_clip
            self.timeline_view.clip_selection_changed_callback = self._set_clip_selection
            self.timeline_view.clip_moved_callback = self._move_clip
            self.timeline_view.clip_create_callback = self._create_clip_from_timeline
            self.timeline_view.clip_span_create_callback = self._create_clip_span_from_timeline
            self.timeline_view.clip_edit_started_callback = self._begin_clip_drag_edit
            self.effects_list.itemDoubleClicked.connect(self._add_selected_effect)
            self.add_effect_button.clicked.connect(self._add_selected_effect)
            self.presets_list.itemDoubleClicked.connect(self._add_selected_preset)
            self.save_preset_button.clicked.connect(self._save_selected_preset)
            self.add_preset_button.clicked.connect(self._add_selected_preset)
            self.delete_preset_button.clicked.connect(self._delete_selected_preset)
            self.reset_clip_params_button.clicked.connect(self._reset_clip_params)
            self.delete_clip_button.clicked.connect(self._delete_selected_clip)
            self.clip_role_combo.currentIndexChanged.connect(self._on_clip_combo_changed)
            self.clip_target_mode_combo.currentIndexChanged.connect(self._on_target_mode_changed)
            self.clip_target_list.itemChanged.connect(self._on_target_list_item_changed)
            self.clip_blend_combo.currentIndexChanged.connect(self._on_clip_combo_changed)
            self.clip_axis_combo.currentIndexChanged.connect(self._on_clip_combo_changed)
            self.clip_tempo_sync_checkbox.toggled.connect(self._on_tempo_sync_toggled)
            self.clip_reverse_checkbox.toggled.connect(self._on_clip_toggle_changed)
            self.clip_curve_text_edit.editingFinished.connect(self._mark_clip_dirty)
            self.clip_curve_text_edit.editingFinished.connect(self._apply_clip_editor_if_dirty)
            self.clip_curve_edit_button.clicked.connect(self._open_curve_editor)
            self.clip_color_mode_combo.currentIndexChanged.connect(self._on_color_mode_changed)
            self.clip_color_tempo_sync_checkbox.toggled.connect(self._on_color_tempo_sync_toggled)
            self.clip_color_palette_preset_combo.currentIndexChanged.connect(self._on_color_palette_preset_changed)
            self.clip_color_palette_save_button.clicked.connect(self._save_color_palette_preset)
            self.clip_color_palette_delete_button.clicked.connect(self._delete_color_palette_preset)
            self.clip_color_from_button.clicked.connect(lambda: self._set_clip_color_picker_target("from"))
            self.clip_color_to_button.clicked.connect(lambda: self._set_clip_color_picker_target("to"))
            self.clip_effect_to_color_button.clicked.connect(lambda: self._choose_clip_color("effect_to"))
            self.clip_embedded_color_picker.currentColorChanged.connect(self._on_embedded_color_picker_changed)
            self.clip_palette_strip.stops_changed.connect(self._on_palette_strip_changed)
            self.clip_palette_strip.selection_changed.connect(self._on_palette_stop_selection_changed)
            self.clip_palette_position_spin.valueChanged.connect(self._on_palette_stop_position_changed)
            self.clip_palette_delete_stop_button.clicked.connect(self._delete_selected_palette_stop)
            for widget in (
                self.clip_start_spin,
                self.clip_end_spin,
                self.clip_layer_spin,
                self.clip_intensity_spin,
                self.clip_width_spin,
                self.clip_softness_spin,
                self.clip_frequency_spin,
                self.clip_phase_spin,
                self.clip_repeats_spin,
                self.clip_duty_cycle_spin,
                self.clip_min_intensity_spin,
                self.clip_max_intensity_spin,
                self.clip_color_rate_spin,
            ):
                widget.valueChanged.connect(self._mark_clip_dirty)
                widget.editingFinished.connect(self._apply_clip_editor_if_dirty)
            self._set_clip_editor_enabled(False)

        def eventFilter(self, watched, event) -> bool:
            return super().eventFilter(watched, event)

        def _make_tool_button(self, icon_enum, tooltip: str, callback) -> QtWidgets.QPushButton:
            button = QtWidgets.QPushButton()
            button.setIcon(self.style().standardIcon(icon_enum))
            button.setToolTip(tooltip)
            button.setFixedWidth(36)
            button.clicked.connect(callback)
            return button

        def _create_actions(self) -> None:
            self.new_project_from_audio_action = QtGui.QAction("New Project From Audio...", self)
            self.new_project_from_audio_action.setShortcut(QtGui.QKeySequence("Ctrl+Shift+N"))
            self.new_project_from_audio_action.triggered.connect(self._new_project_from_audio_dialog)
            self.open_action = QtGui.QAction("Open Project...", self)
            self.open_action.triggered.connect(self._open_project_dialog)
            self.save_action = QtGui.QAction("Save Project", self)
            self.save_action.setShortcut(QtGui.QKeySequence.Save)
            self.save_action.triggered.connect(self._save_project)
            self.undo_action = QtGui.QAction("Undo", self)
            self.undo_action.setShortcut(QtGui.QKeySequence.Undo)
            self.undo_action.setShortcutContext(QtCore.Qt.ApplicationShortcut)
            self.undo_action.triggered.connect(self._undo)
            self.redo_action = QtGui.QAction("Redo", self)
            self.redo_action.setShortcuts([QtGui.QKeySequence("Ctrl+Y"), QtGui.QKeySequence.Redo])
            self.redo_action.setShortcutContext(QtCore.Qt.ApplicationShortcut)
            self.redo_action.triggered.connect(self._redo)
            self.save_as_action = QtGui.QAction("Save Project As...", self)
            self.save_as_action.triggered.connect(self._save_project_as)
            self.export_action = QtGui.QAction("Export Package...", self)
            self.export_action.triggered.connect(self._export_dialog)
            self.load_example_action = QtGui.QAction("Load Example", self)
            self.load_example_action.triggered.connect(self._load_example)
            self.load_layout_action = QtGui.QAction("Load Layout...", self)
            self.load_layout_action.triggered.connect(self._load_layout_dialog)
            self.use_generated_layout_action = QtGui.QAction("Use Generated Layout", self)
            self.use_generated_layout_action.triggered.connect(self._use_generated_layout)
            self.reload_audio_action = QtGui.QAction("Reload Audio", self)
            self.reload_audio_action.triggered.connect(self._load_project_audio)
            self.set_audio_action = QtGui.QAction("Set Audio File...", self)
            self.set_audio_action.triggered.connect(self._set_audio_file_dialog)
            self.open_project_dir_action = QtGui.QAction("Open Project Directory", self)
            self.open_project_dir_action.triggered.connect(self._open_project_directory)
            self.play_pause_action = QtGui.QAction("Play/Pause From Cursor", self)
            self.play_pause_action.setShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space))
            self.play_pause_action.setShortcutContext(QtCore.Qt.ApplicationShortcut)
            self.play_pause_action.triggered.connect(self._toggle_play_pause_from_cursor)
            self.quick_add_action = QtGui.QAction("Add Effect At Playhead", self)
            self.quick_add_action.setShortcut(QtGui.QKeySequence("Ctrl+Return"))
            self.quick_add_action.triggered.connect(self._quick_add_at_playhead)
            self.copy_clip_action = QtGui.QAction("Copy Clip", self)
            self.copy_clip_action.setShortcut(QtGui.QKeySequence.Copy)
            self.copy_clip_action.triggered.connect(self._copy_selected_clip)
            self.cut_clip_action = QtGui.QAction("Cut Clip", self)
            self.cut_clip_action.setShortcut(QtGui.QKeySequence.Cut)
            self.cut_clip_action.triggered.connect(self._cut_selected_clip)
            self.select_all_clips_action = QtGui.QAction("Select All Clips", self)
            self.select_all_clips_action.setShortcut(QtGui.QKeySequence.SelectAll)
            self.select_all_clips_action.setShortcutContext(QtCore.Qt.ApplicationShortcut)
            self.select_all_clips_action.triggered.connect(self._select_all_clips)
            self.paste_clip_action = QtGui.QAction("Paste Clip", self)
            self.paste_clip_action.setShortcut(QtGui.QKeySequence.Paste)
            self.paste_clip_action.triggered.connect(self._paste_clip)
            self.duplicate_clip_action = QtGui.QAction("Duplicate Clip", self)
            self.duplicate_clip_action.setShortcut(QtGui.QKeySequence("Ctrl+D"))
            self.duplicate_clip_action.triggered.connect(self._duplicate_selected_clip)
            self.delete_clip_action = QtGui.QAction("Delete Clip", self)
            self.delete_clip_action.setShortcut(QtGui.QKeySequence.Delete)
            self.delete_clip_action.triggered.connect(self._delete_selected_clip)
            self.timeline_zoom_in_action = QtGui.QAction("Zoom Timeline In", self)
            self.timeline_zoom_in_action.setShortcut(QtGui.QKeySequence("Ctrl+="))
            self.timeline_zoom_in_action.triggered.connect(self._zoom_timeline_in)
            self.timeline_zoom_out_action = QtGui.QAction("Zoom Timeline Out", self)
            self.timeline_zoom_out_action.setShortcut(QtGui.QKeySequence("Ctrl+-"))
            self.timeline_zoom_out_action.triggered.connect(self._zoom_timeline_out)
            self.quit_action = QtGui.QAction("Quit", self)
            self.quit_action.setShortcut(QtGui.QKeySequence.Quit)
            self.quit_action.triggered.connect(self.close)
            for action in (
                self.new_project_from_audio_action,
                self.open_action,
                self.save_action,
                self.undo_action,
                self.redo_action,
                self.save_as_action,
                self.export_action,
                self.load_example_action,
                self.load_layout_action,
                self.use_generated_layout_action,
                self.reload_audio_action,
                self.set_audio_action,
                self.open_project_dir_action,
                self.play_pause_action,
                self.quick_add_action,
                self.copy_clip_action,
                self.cut_clip_action,
                self.select_all_clips_action,
                self.paste_clip_action,
                self.duplicate_clip_action,
                self.delete_clip_action,
                self.timeline_zoom_in_action,
                self.timeline_zoom_out_action,
                self.quit_action,
            ):
                self.addAction(action)
            self._update_history_actions()

        def _create_menus(self) -> None:
            file_menu = self.menuBar().addMenu("File")
            file_menu.addAction(self.new_project_from_audio_action)
            file_menu.addAction(self.open_action)
            file_menu.addAction(self.save_action)
            file_menu.addAction(self.save_as_action)
            file_menu.addSeparator()
            file_menu.addAction(self.load_example_action)
            file_menu.addAction(self.set_audio_action)
            file_menu.addAction(self.open_project_dir_action)
            file_menu.addAction(self.load_layout_action)
            file_menu.addAction(self.use_generated_layout_action)
            file_menu.addAction(self.reload_audio_action)
            file_menu.addSeparator()
            file_menu.addAction(self.export_action)
            file_menu.addSeparator()
            file_menu.addAction(self.quit_action)

            edit_menu = self.menuBar().addMenu("Edit")
            edit_menu.addAction(self.undo_action)
            edit_menu.addAction(self.redo_action)
            edit_menu.addSeparator()
            edit_menu.addAction(self.play_pause_action)
            edit_menu.addSeparator()
            edit_menu.addAction(self.quick_add_action)
            edit_menu.addAction(self.copy_clip_action)
            edit_menu.addAction(self.cut_clip_action)
            edit_menu.addAction(self.select_all_clips_action)
            edit_menu.addAction(self.paste_clip_action)
            edit_menu.addAction(self.duplicate_clip_action)
            edit_menu.addAction(self.delete_clip_action)
            edit_menu.addSeparator()
            edit_menu.addAction(self.timeline_zoom_in_action)
            edit_menu.addAction(self.timeline_zoom_out_action)

        def _timeline_max_ms(self) -> int:
            if self.audio_waveform is not None:
                return max(0, int(self.audio_waveform.duration_ms))
            duration_ms = 0
            if self.project is not None:
                duration_ms = max(duration_ms, self.project.duration_ms)
            if self.audio_player is not None:
                duration_ms = max(duration_ms, self.audio_player.duration())
            return max(0, duration_ms)

        def _has_audio_source(self) -> bool:
            if self.audio_player is None or self.project is None:
                return False
            audio_path = resolve_audio_path(self.project)
            return audio_path is not None and audio_path.exists()

        def _update_time_range(self) -> None:
            current_value = self.time_slider.value()
            max_time = self._timeline_max_ms()
            self.time_slider.setRange(0, max(0, max_time))
            if current_value > self.time_slider.maximum():
                self._set_timeline_position(self.time_slider.maximum(), sync_player=False)

        def _update_view_buttons(self) -> None:
            is_3d = self.preview_view.view_mode() == "3d"
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentText("3D" if is_3d else "2D")
            self.mode_combo.blockSignals(False)

        def _on_mode_combo_changed(self, text: str) -> None:
            self._set_preview_mode("3d" if text == "3D" else "2d")

        def _on_preview_body_toggled(self, checked: bool) -> None:
            self.preview_view.set_show_body(bool(checked))
            self.preview_view.update()
            self._schedule_session_save()

        def _on_preview_combo_changed(self, text: str) -> None:
            if text in ROLE_NAMES:
                self._last_single_preview_role = text
            self._refresh_preview()
            self._schedule_session_save()

        def _update_transport_state(self) -> None:
            has_audio = self._has_audio_source()
            if self.audio_player is not None and QtMultimedia is not None:
                playing = self.audio_player.playbackState() == QtMultimedia.QMediaPlayer.PlayingState
            else:
                playing = False
            self.play_button.setEnabled(has_audio and not playing)
            self.pause_button.setEnabled(has_audio and playing)
            self.stop_button.setEnabled(has_audio or self.time_slider.value() > 0)
            self.restart_button.setEnabled(self.project is not None or has_audio)

        def _audio_is_playing(self) -> bool:
            if self.audio_player is None or QtMultimedia is None:
                return False
            return self.audio_player.playbackState() == QtMultimedia.QMediaPlayer.PlayingState

        def _preview_selection(self) -> str:
            selection = self.preview_combo.currentText().strip()
            if selection in PREVIEW_OPTIONS:
                return selection
            return self._last_single_preview_role

        def _show_all_preview_roles(self) -> bool:
            return self._preview_selection() == "All"

        def _active_preview_role(self) -> str:
            selection = self._preview_selection()
            if selection in ROLE_NAMES:
                return selection
            if self.selected_clip is not None:
                location = self._find_clip_location(self.selected_clip)
                if location is not None and location[0] in ROLE_NAMES:
                    return location[0]
            lane_role = self.new_clip_lane_combo.currentData()
            if lane_role in ROLE_NAMES:
                return str(lane_role)
            return self._last_single_preview_role

        def _timeline_focus_role(self) -> str:
            return self._active_preview_role()

        def _build_editor_session_state(self) -> EditorSessionState | None:
            if self.project is None:
                return None
            return EditorSessionState(
                project=self.project,
                current_time_ms=int(self.time_slider.value()),
                active_role=self._active_preview_role(),
                preview_scope="All" if self._show_all_preview_roles() else "Active",
                view_mode=self.preview_view.view_mode(),
                show_body=self.preview_view.show_body(),
                snap_enabled=self.snap_checkbox.isChecked(),
                snap_divisor=max(1, int(self.snap_division_combo.currentData() or 1)),
                layout_kind=self._layout_source_kind,
                layout_path=str(self._layout_source_path) if self._layout_source_path is not None else "",
                presets=tuple(dict(entry) for entry in self.effect_presets),
            )

        def _schedule_session_save(self) -> None:
            if self._history_restoring:
                return
            self._session_save_timer.start()

        def _write_editor_session(self) -> None:
            session_state = self._build_editor_session_state()
            if session_state is None:
                return
            try:
                save_editor_session(session_state)
            except Exception as exc:
                self.statusBar().showMessage(f"Session save failed: {exc}")

        def _project_signature(self, project: ShowProject | None = None) -> str | None:
            target = self.project if project is None else project
            if target is None:
                return None
            return json.dumps(project_to_obj(target), sort_keys=True, separators=(",", ":"))

        def _saved_signature_from_disk(self, project: ShowProject | None = None) -> str | None:
            target = self.project if project is None else project
            if target is None or target.source_path is None:
                return None
            try:
                return json.dumps(
                    json.loads(target.source_path.read_text()),
                    sort_keys=True,
                    separators=(",", ":"),
                )
            except Exception:
                return None

        def _has_unsaved_project_changes(self) -> bool:
            if self.project is None:
                return False
            if self._clip_editor_dirty:
                return True
            current_signature = self._project_signature()
            if self.project.source_path is None:
                return bool(current_signature)
            return current_signature != self._saved_project_signature

        def _confirm_close_with_unsaved_changes(self) -> bool:
            if self.project is None:
                return True
            if not self._flush_pending_clip_apply():
                self.statusBar().showMessage("Fix inspector changes before closing")
                return False
            if not self._has_unsaved_project_changes():
                return True
            title = self.project.title.strip() or self.project.slug.strip() or "Untitled Show"
            message_box = QtWidgets.QMessageBox(self)
            message_box.setIcon(QtWidgets.QMessageBox.Warning)
            message_box.setWindowTitle("Unsaved Project")
            message_box.setText(f'Save changes to "{title}" before closing?')
            message_box.setInformativeText("Your project has unsaved changes.")
            save_button = message_box.addButton("Save", QtWidgets.QMessageBox.AcceptRole)
            discard_button = message_box.addButton("Discard", QtWidgets.QMessageBox.DestructiveRole)
            cancel_button = message_box.addButton("Cancel", QtWidgets.QMessageBox.RejectRole)
            message_box.setDefaultButton(save_button)
            message_box.setEscapeButton(cancel_button)
            message_box.exec()
            clicked = message_box.clickedButton()
            if clicked is save_button:
                return self._save_project()
            if clicked is discard_button:
                return True
            return False

        def _apply_editor_session_state(self, session: EditorSessionState) -> None:
            self.preview_combo.blockSignals(True)
            self.preview_body_checkbox.blockSignals(True)
            self.mode_combo.blockSignals(True)
            self.snap_checkbox.blockSignals(True)
            self.snap_division_combo.blockSignals(True)
            try:
                role = session.active_role if session.active_role in ROLE_NAMES else ROLE_NAMES[0]
                self._last_single_preview_role = role
                self.preview_combo.setCurrentText("All" if session.preview_scope == "All" else role)
                self.preview_body_checkbox.setChecked(bool(session.show_body))
                view_mode = "3d" if session.view_mode == "3d" else "2d"
                self.mode_combo.setCurrentText("3D" if view_mode == "3d" else "2D")
                self.snap_checkbox.setChecked(bool(getattr(session, "snap_enabled", True)))
                snap_divisor = max(1, int(getattr(session, "snap_divisor", 1)))
                snap_index = self.snap_division_combo.findData(snap_divisor)
                if snap_index >= 0:
                    self.snap_division_combo.setCurrentIndex(snap_index)
            finally:
                self.preview_combo.blockSignals(False)
                self.preview_body_checkbox.blockSignals(False)
                self.mode_combo.blockSignals(False)
                self.snap_checkbox.blockSignals(False)
                self.snap_division_combo.blockSignals(False)

            if session.layout_kind == "file" and session.layout_path:
                layout_path = Path(session.layout_path).expanduser()
                if layout_path.exists():
                    self.preview_layout_data = load_layout_file(layout_path)
                    self._layout_source_kind = "file"
                    self._layout_source_path = layout_path.resolve()
                    self.layout_label.setText(f"layout: {layout_path.name}")
                else:
                    self._layout_source_kind = "generated"
                    self._layout_source_path = None
                    self.preview_layout_data = generated_layout()
                    self.layout_label.setText("layout: generated")
            else:
                self._layout_source_kind = "generated"
                self._layout_source_path = None
                self.preview_layout_data = generated_layout()
                self.layout_label.setText("layout: generated")

            self._set_preview_mode("3d" if session.view_mode == "3d" else "2d")
            self.preview_view.set_show_body(bool(session.show_body))
            self.effect_presets = [dict(entry) for entry in getattr(session, "presets", ())]
            self._refresh_preset_list()
            self._set_timeline_position(int(session.current_time_ms), sync_player=False)
            self._refresh_timeline()
            self._schedule_session_save()

        def _set_clip_editor_enabled(self, enabled: bool) -> None:
            for widget in (
                self.clip_tabs,
                self.clip_role_combo,
                self.clip_target_mode_combo,
                self.clip_target_list,
                self.clip_blend_combo,
                self.clip_start_spin,
                self.clip_end_spin,
                self.clip_layer_spin,
                self.clip_curve_text_edit,
                self.clip_curve_edit_button,
                self.clip_intensity_spin,
                self.clip_axis_combo,
                self.clip_width_spin,
                self.clip_softness_spin,
                self.clip_frequency_spin,
                self.clip_tempo_sync_checkbox,
                self.clip_phase_spin,
                self.clip_repeats_spin,
                self.clip_duty_cycle_spin,
                self.clip_min_intensity_spin,
                self.clip_max_intensity_spin,
                self.clip_reverse_checkbox,
                self.clip_color_mode_combo,
                self.clip_color_from_button,
                self.clip_color_to_button,
                self.clip_effect_to_color_button,
                self.clip_color_tempo_sync_checkbox,
                self.clip_color_rate_spin,
                self.clip_color_palette_preset_combo,
                self.clip_color_palette_save_button,
                self.clip_color_palette_delete_button,
                self.clip_color_picker_widget,
                self.clip_palette_editor_widget,
                self.clip_palette_strip,
                self.clip_palette_position_spin,
                self.clip_palette_delete_stop_button,
                self.reset_clip_params_button,
                self.delete_clip_button,
            ):
                widget.setEnabled(enabled)
            self._update_tempo_sync_ui()
            self._update_color_tempo_sync_ui()
            self._update_color_palette_summary()

        def _set_form_row_visible(self, form_layout, field_widget, visible: bool) -> None:
            if field_widget is None:
                return
            label_widget = form_layout.labelForField(field_widget)
            if label_widget is not None:
                label_widget.setVisible(visible)
            field_widget.setVisible(visible)

        def _set_clip_form_row_visible(self, field_widget, visible: bool) -> None:
            self._set_form_row_visible(self.clip_form, field_widget, visible)

        def _apply_effect_row_visibility(self, effect_name: str) -> None:
            effect_id = EFFECT_NAME_TO_SPEC.get(effect_name, EFFECT_LIBRARY[0])[1]
            self._set_clip_form_row_visible(self.clip_intensity_spin, True)
            self._set_clip_form_row_visible(self.clip_curve_widget, True)
            self._set_clip_form_row_visible(self.clip_to_color_widget, effect_id in EFFECT_FIELDS_TO_COLOR)
            self._set_clip_form_row_visible(self.clip_axis_combo, effect_id in EFFECT_FIELDS_AXIS)
            self._set_clip_form_row_visible(self.clip_width_spin, effect_id in EFFECT_FIELDS_WIDTH)
            self._set_clip_form_row_visible(self.clip_softness_spin, effect_id in EFFECT_FIELDS_SOFTNESS)
            show_rate = effect_id in EFFECT_FIELDS_RATE
            self._set_clip_form_row_visible(self.clip_tempo_sync_checkbox, show_rate)
            self._set_clip_form_row_visible(self.clip_frequency_spin, show_rate)
            self._set_clip_form_row_visible(self.clip_phase_spin, effect_id in EFFECT_FIELDS_PHASE)
            self._set_clip_form_row_visible(self.clip_repeats_spin, effect_id in EFFECT_FIELDS_REPEATS)
            self._set_clip_form_row_visible(self.clip_duty_cycle_spin, effect_id in EFFECT_FIELDS_DUTY)
            show_min_max = effect_id in EFFECT_FIELDS_MIN_MAX
            self._set_clip_form_row_visible(self.clip_min_intensity_spin, show_min_max)
            self._set_clip_form_row_visible(self.clip_max_intensity_spin, show_min_max)
            self._set_clip_form_row_visible(self.clip_reverse_checkbox, effect_id in EFFECT_FIELDS_REVERSE)
            self._update_color_mode_ui()

        def _set_clip_editor_dirty(self, dirty: bool) -> None:
            self._clip_editor_dirty = bool(dirty and self.selected_clip is not None)
            if self.selected_clip is not None:
                base_text = self.clip_status_label.text().replace(" (pending changes)", "")
                if self._clip_editor_dirty:
                    self.clip_status_label.setText(f"{base_text} (pending changes)")
                else:
                    self.clip_status_label.setText(base_text)
            self._update_history_actions()

        def _mark_clip_dirty(self, *args) -> None:
            del args
            if self._ignore_clip_editor_changes or self.selected_clip is None:
                return
            self._set_clip_editor_dirty(True)
            self._begin_pending_history("Edit clip")
            self.statusBar().showMessage("Inspector changed")

        def _apply_clip_editor_if_dirty(self, *args) -> None:
            del args
            if not self._clip_editor_dirty:
                return
            self._apply_clip_changes()

        def _on_clip_combo_changed(self, *args) -> None:
            del args
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _on_clip_toggle_changed(self, *args) -> None:
            del args
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _update_tempo_sync_ui(self) -> None:
            enabled = self.selected_clip is not None
            tempo_sync = enabled and self.clip_tempo_sync_checkbox.isChecked()
            self.clip_frequency_spin.setEnabled(enabled)
            self.clip_frequency_label.setText("Cycles / Beat" if tempo_sync else "Frequency Hz")
            self.clip_frequency_spin.setToolTip(
                "Effect cycles per beat when BPM sync is enabled."
                if tempo_sync else
                "Effect cycles per second."
            )
            self.clip_beats_per_cycle_label.setVisible(False)
            self.clip_beats_per_cycle_spin.setVisible(False)

        def _on_tempo_sync_toggled(self, checked: bool) -> None:
            if checked:
                self.clip_frequency_spin.setValue(1.0)
            self._update_tempo_sync_ui()
            self._on_clip_toggle_changed()

        def _available_section_names(self) -> tuple[str, ...]:
            names = tuple(section.name for section in self.preview_layout_data.sections)
            if names:
                return names
            return tuple(SECTION_IDS.keys())

        def _available_group_names(self) -> tuple[str, ...]:
            return tuple(GROUP_SECTIONS.keys())

        def _parsed_target_names(self, target: str) -> tuple[str, ...]:
            return tuple(
                name.strip()
                for name in target.split(",")
                if name.strip()
            )

        def _set_target_list_items(self, target_kind: str, checked_names: tuple[str, ...] = ()) -> None:
            checked = set(checked_names)
            if target_kind == "group":
                names = self._available_group_names()
            elif target_kind == "section":
                names = self._available_section_names()
            else:
                names = ()
            self.clip_target_list.blockSignals(True)
            self.clip_target_list.clear()
            for name in names:
                item = QtWidgets.QListWidgetItem(name)
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable | QtCore.Qt.ItemIsEnabled)
                item.setCheckState(QtCore.Qt.Checked if name in checked else QtCore.Qt.Unchecked)
                self.clip_target_list.addItem(item)
            self.clip_target_list.setEnabled(target_kind != "all")
            self.clip_target_list.blockSignals(False)

        def _selected_target_config(self) -> tuple[str, str]:
            target_kind = str(self.clip_target_mode_combo.currentData() or "all")
            if target_kind == "all":
                return ("all", "all")
            names: list[str] = []
            for index in range(self.clip_target_list.count()):
                item = self.clip_target_list.item(index)
                if item is not None and item.checkState() == QtCore.Qt.Checked:
                    names.append(item.text().strip())
            return (target_kind, ",".join(names))

        def _set_target_mode(self, target_kind: str, target: str) -> None:
            normalized_kind = target_kind if target_kind in {"all", "group", "section"} else "all"
            combo_index = self.clip_target_mode_combo.findData(normalized_kind)
            if combo_index >= 0:
                self.clip_target_mode_combo.setCurrentIndex(combo_index)
            self._set_target_list_items(normalized_kind, self._parsed_target_names(target))

        def _on_target_mode_changed(self, *args) -> None:
            del args
            if self._ignore_clip_editor_changes:
                return
            target_kind = str(self.clip_target_mode_combo.currentData() or "all")
            self._set_target_list_items(target_kind)
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _on_target_list_item_changed(self, item) -> None:
            del item
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _refresh_preset_list(self) -> None:
            self.presets_list.clear()
            for entry in self.effect_presets:
                name = str(entry.get("name", "Preset")).strip() or "Preset"
                clip_obj = entry.get("clip")
                effect_name = ""
                if isinstance(clip_obj, dict):
                    effect_name = EFFECT_ID_TO_NAME.get(str(clip_obj.get("effect", "")), str(clip_obj.get("effect", "")))
                item = QtWidgets.QListWidgetItem(name if not effect_name else f"{name} [{effect_name}]")
                item.setData(QtCore.Qt.UserRole, entry)
                self.presets_list.addItem(item)

        def _save_selected_preset(self) -> None:
            if self.selected_clip is None:
                self.statusBar().showMessage("No clip selected")
                return
            if not self._flush_pending_clip_apply():
                return
            default_name = EFFECT_ID_TO_NAME.get(self.selected_clip.effect, self.selected_clip.effect.title())
            name, accepted = QtWidgets.QInputDialog.getText(
                self,
                "Save Preset",
                "Preset name",
                text=default_name,
            )
            if not accepted:
                return
            preset_name = name.strip() or default_name
            preset_entry = {
                "name": preset_name,
                "clip": {
                    "start_ms": 0,
                    "end_ms": max(1, self.selected_clip.end_ms - self.selected_clip.start_ms),
                    "layer": int(self.selected_clip.layer),
                    "effect": str(self.selected_clip.effect),
                    "blend": str(self.selected_clip.blend),
                    "target_kind": str(self.selected_clip.target_kind),
                    "target": str(self.selected_clip.target),
                    "palette": self.selected_clip.palette,
                    "params": json.loads(json.dumps(self.selected_clip.params)),
                },
            }
            self.effect_presets.append(preset_entry)
            self._refresh_preset_list()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Saved preset {preset_name}")

        def _selected_preset_entry(self) -> dict | None:
            item = self.presets_list.currentItem()
            if item is None:
                return None
            entry = item.data(QtCore.Qt.UserRole)
            if isinstance(entry, dict):
                return dict(entry)
            return None

        def _add_selected_preset(self, *args) -> None:
            del args
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            entry = self._selected_preset_entry()
            if entry is None:
                self.statusBar().showMessage("No preset selected")
                return
            clip_obj = entry.get("clip")
            if not isinstance(clip_obj, dict):
                self.statusBar().showMessage("Preset is invalid")
                return
            if not self._flush_pending_clip_apply():
                return
            self._record_undo_state("Add preset clip")
            lane_role = str(self.new_clip_lane_combo.currentData() or "*")
            duration = max(1, int(clip_obj.get("end_ms", 1000)) - int(clip_obj.get("start_ms", 0)))
            start_ms = self._snap_time_ms(self.time_slider.value())
            new_clip = ShowClip(
                start_ms=start_ms,
                end_ms=start_ms + duration,
                layer=int(clip_obj.get("layer", 0)),
                effect=str(clip_obj.get("effect", "solid")),
                blend=str(clip_obj.get("blend", "max")),
                target_kind=str(clip_obj.get("target_kind", "all")),
                target=str(clip_obj.get("target", "all")),
                palette=clip_obj.get("palette"),
                params=json.loads(json.dumps(dict(clip_obj.get("params", {})))),
            )
            self._insert_clip(lane_role, new_clip)
            self._populate_timeline_table()
            self._select_clip(new_clip)
            self._set_timeline_position(new_clip.end_ms, sync_player=False)
            self._refresh_preview()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Added preset {entry.get('name', 'Preset')}")

        def _delete_selected_preset(self) -> None:
            row = self.presets_list.currentRow()
            if row < 0 or row >= len(self.effect_presets):
                return
            deleted_name = str(self.effect_presets[row].get("name", "Preset"))
            del self.effect_presets[row]
            self._refresh_preset_list()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Deleted preset {deleted_name}")

        def _find_clip_location(self, clip):
            if self.project is None or clip is None:
                return None
            for index, item in enumerate(self.project.global_clips):
                if item is clip:
                    return ("*", index)
            for role in ("A", "B", "C"):
                role_clips = self.project.role_clips.get(role, [])
                for index, item in enumerate(role_clips):
                    if item is clip:
                        return (role, index)
            return None

        def _clip_locator(self, clip) -> dict | None:
            location = self._find_clip_location(clip)
            if location is None:
                return None
            return {"role": location[0], "index": location[1]}

        def _all_project_clips(self) -> list[ShowClip]:
            if self.project is None:
                return []
            clips = list(self.project.global_clips)
            for role in ("A", "B", "C"):
                clips.extend(self.project.role_clips.get(role, []))
            return clips

        def _clips_from_id_set(self, clip_ids: set[int] | frozenset[int]) -> list[ShowClip]:
            if not clip_ids:
                return []
            return [clip for clip in self._all_project_clips() if id(clip) in clip_ids]

        def _clip_from_locator(self, locator: dict | None):
            if self.project is None or not locator:
                return None
            role = str(locator.get("role", "*"))
            try:
                index = int(locator.get("index", -1))
            except (TypeError, ValueError):
                return None
            if index < 0:
                return None
            if role == "*":
                clips = self.project.global_clips
            else:
                clips = self.project.role_clips.get(role, [])
            if 0 <= index < len(clips):
                return clips[index]
            return None

        def _history_entry(self, label: str) -> dict | None:
            if self.project is None:
                return None
            return {
                "label": label,
                "source_path": str(self.project.source_path) if self.project.source_path is not None else "",
                "project": project_to_obj(self.project),
                "selected_clip": self._clip_locator(self.selected_clip),
                "time_ms": int(self.time_slider.value()),
            }

        def _history_entries_equal(self, left: dict | None, right: dict | None) -> bool:
            if left is None or right is None:
                return left is right
            return (
                left.get("source_path") == right.get("source_path")
                and left.get("project") == right.get("project")
                and left.get("selected_clip") == right.get("selected_clip")
                and int(left.get("time_ms", 0)) == int(right.get("time_ms", 0))
            )

        def _clear_pending_history(self) -> None:
            self._pending_history_entry = None

        def _discard_pending_clip_edit(self) -> None:
            self._clear_pending_history()
            self._set_clip_editor_dirty(False)

        def _begin_pending_history(self, label: str) -> None:
            if self._history_restoring or self.project is None:
                return
            if self._pending_history_entry is not None:
                return
            self._pending_history_entry = self._history_entry(label)

        def _push_undo_entry(self, entry: dict | None) -> None:
            if entry is None or self._history_restoring:
                return
            if self._undo_stack and self._history_entries_equal(self._undo_stack[-1], entry):
                return
            self._undo_stack.append(entry)
            if len(self._undo_stack) > self._history_limit:
                self._undo_stack = self._undo_stack[-self._history_limit:]
            self._redo_stack.clear()
            self._update_history_actions()

        def _record_undo_state(self, label: str) -> None:
            if self._history_restoring:
                return
            self._push_undo_entry(self._history_entry(label))

        def _commit_pending_history(self) -> None:
            if self._pending_history_entry is None:
                return
            entry = self._pending_history_entry
            self._pending_history_entry = None
            current = self._history_entry(str(entry.get("label", "Edit")))
            if self._history_entries_equal(entry, current):
                self._update_history_actions()
                return
            self._push_undo_entry(entry)

        def _cancel_pending_history_ui(self) -> bool:
            if self._pending_history_entry is None and not self._clip_editor_dirty:
                return False
            self._discard_pending_clip_edit()
            self._refresh_clip_editor()
            self.statusBar().showMessage("Canceled pending edit")
            return True

        def _flush_pending_clip_apply(self) -> bool:
            if not self._clip_editor_dirty:
                return True
            applied = self._apply_clip_changes(quiet=True)
            if not applied:
                self._discard_pending_clip_edit()
            return applied

        def _reset_history(self) -> None:
            self._undo_stack.clear()
            self._redo_stack.clear()
            self._discard_pending_clip_edit()
            self._update_history_actions()

        def _update_history_actions(self) -> None:
            if not hasattr(self, "undo_action"):
                return
            self.undo_action.setEnabled(bool(self._undo_stack or self._pending_history_entry or self._clip_editor_dirty))
            self.redo_action.setEnabled(bool(self._redo_stack))

        def _restore_history_entry(self, entry: dict) -> None:
            source_path = str(entry.get("source_path", "")).strip()
            project = project_from_obj(
                dict(entry.get("project", {})),
                source_path=source_path or None,
            )
            current_audio_path = resolve_audio_path(self.project) if self.project is not None else None
            target_audio_path = resolve_audio_path(project)
            reuse_audio_state = current_audio_path == target_audio_path
            restore_time_ms = self.time_slider.value() if reuse_audio_state else int(entry.get("time_ms", 0))
            self._history_restoring = True
            try:
                self.set_project(
                    project,
                    reset_history=False,
                    time_ms=restore_time_ms,
                    selected_locator=entry.get("selected_clip"),
                    reload_audio=not reuse_audio_state,
                )
            finally:
                self._history_restoring = False
            self._update_history_actions()

        def _undo(self) -> None:
            if self._cancel_pending_history_ui():
                self._update_history_actions()
                return
            if self.project is None or not self._undo_stack:
                self.statusBar().showMessage("Nothing to undo")
                self._update_history_actions()
                return
            current = self._history_entry("Current")
            target = self._undo_stack.pop()
            if current is not None:
                self._redo_stack.append(current)
            self._restore_history_entry(target)
            self.statusBar().showMessage(f"Undo: {target.get('label', 'Edit')}")
            self._update_history_actions()

        def _redo(self) -> None:
            if self._cancel_pending_history_ui():
                self._update_history_actions()
                return
            if self.project is None or not self._redo_stack:
                self.statusBar().showMessage("Nothing to redo")
                self._update_history_actions()
                return
            current = self._history_entry("Current")
            target = self._redo_stack.pop()
            if current is not None:
                self._undo_stack.append(current)
            self._restore_history_entry(target)
            self.statusBar().showMessage(f"Redo: {target.get('label', 'Edit')}")
            self._update_history_actions()

        def _populate_effect_tree(self) -> None:
            self.effects_list.clear()
            category_items: dict[tuple[str, ...], QtWidgets.QTreeWidgetItem] = {}
            first_effect_item = None
            for spec in PICKER_EFFECT_LIBRARY:
                effect_name = str(spec[0])
                category_path = tuple(spec[7]) if len(spec) > 7 else ("Effects",)
                parent_item = None
                path_so_far: list[str] = []
                for category_label in category_path:
                    path_so_far.append(str(category_label))
                    key = tuple(path_so_far)
                    category_item = category_items.get(key)
                    if category_item is None:
                        category_item = QtWidgets.QTreeWidgetItem([str(category_label)])
                        category_item.setFlags(category_item.flags() & ~QtCore.Qt.ItemIsSelectable)
                        if parent_item is None:
                            self.effects_list.addTopLevelItem(category_item)
                        else:
                            parent_item.addChild(category_item)
                        category_item.setExpanded(True)
                        category_items[key] = category_item
                    parent_item = category_item
                effect_item = QtWidgets.QTreeWidgetItem([effect_name])
                effect_item.setData(0, QtCore.Qt.UserRole, effect_name)
                if parent_item is None:
                    self.effects_list.addTopLevelItem(effect_item)
                else:
                    parent_item.addChild(effect_item)
                if first_effect_item is None:
                    first_effect_item = effect_item
            self.effects_list.expandAll()
            if first_effect_item is not None:
                self.effects_list.setCurrentItem(first_effect_item)

        def _effect_name_from_item(self, item) -> str:
            if item is None:
                return ""
            data = item.data(0, QtCore.Qt.UserRole)
            if isinstance(data, str) and data.strip():
                return data.strip()
            text = item.text(0).strip()
            return text if text in EFFECT_NAME_TO_SPEC else ""

        def _selected_effect_name(self) -> str:
            item = self.effects_list.currentItem()
            effect_name = self._effect_name_from_item(item)
            if effect_name:
                return effect_name
            return EFFECT_NAMES[0]

        def _curve_parameter_specs_for_effect(self, effect_name: str) -> list[CurveParameterSpec]:
            effect_id = EFFECT_NAME_TO_SPEC.get(effect_name, EFFECT_LIBRARY[0])[1]
            specs = [CurveParameterSpec("intensity", "Intensity", 0.0, 4.0)]
            if effect_id in EFFECT_FIELDS_WIDTH:
                specs.append(CurveParameterSpec("width", "Width", 0.01, 2.0))
            if effect_id in EFFECT_FIELDS_SOFTNESS:
                specs.append(CurveParameterSpec("softness", "Softness", 0.0, 2.0))
            if effect_id in EFFECT_FIELDS_RATE:
                rate_label = "Cycles / Beat" if self.clip_tempo_sync_checkbox.isChecked() else "Frequency"
                specs.append(CurveParameterSpec("frequency_hz", rate_label, 0.0, 40.0))
            if effect_id in EFFECT_FIELDS_PHASE:
                specs.append(CurveParameterSpec("phase", "Phase", 0.0, 1.0))
            if effect_id in EFFECT_FIELDS_REPEATS:
                specs.append(CurveParameterSpec("repeats", "Repeats", 1.0, 64.0))
            if effect_id in EFFECT_FIELDS_DUTY:
                specs.append(CurveParameterSpec("duty_cycle", "Duty Cycle", 0.0, 1.0))
            if effect_id in EFFECT_FIELDS_MIN_MAX:
                specs.append(CurveParameterSpec("min_intensity", "Min Intensity", 0.0, 4.0))
                specs.append(CurveParameterSpec("max_intensity", "Max Intensity", 0.0, 4.0))
            return specs

        def _open_curve_editor(self) -> None:
            if self.selected_clip is None:
                self.statusBar().showMessage("No clip selected")
                return
            effect_name = EFFECT_ID_TO_NAME.get(self.selected_clip.effect, self.selected_clip.effect.title())
            dialog = CurveEditorDialog(
                self.clip_curve_text_edit.text(),
                self._curve_parameter_specs_for_effect(effect_name),
                self,
            )
            if dialog.exec() != QtWidgets.QDialog.Accepted:
                return
            self.clip_curve_text_edit.setText(dialog.result_text())
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _normalized_rgba(self, rgba) -> tuple[int, int, int, int]:
            if not isinstance(rgba, (list, tuple)) or len(rgba) != 4:
                return (255, 255, 255, 255)
            try:
                return tuple(max(0, min(255, int(channel))) for channel in rgba)  # type: ignore[return-value]
            except (TypeError, ValueError):
                return (255, 255, 255, 255)

        def _color_button_style(self, rgba: tuple[int, int, int, int], *, selected: bool = False) -> str:
            border = "#ffffff" if selected else "#263241"
            text = "#091018" if sum(rgba[:3]) >= 384 else "#e6edf3"
            return (
                "QPushButton {"
                f"background-color: rgba({rgba[0]}, {rgba[1]}, {rgba[2]}, {rgba[3]});"
                f"border: 2px solid {border};"
                "border-radius: 4px;"
                f"color: {text};"
                "padding: 2px 6px;"
                "}"
            )

        def _format_rgba_label(self, rgba: tuple[int, int, int, int]) -> str:
            hex_text = f"#{rgba[0]:02X}{rgba[1]:02X}{rgba[2]:02X}"
            if rgba[3] == 255:
                return hex_text
            return f"{hex_text} / {rgba[3]}"

        def _set_hidden_rgba_spins(self, spins, rgba: tuple[int, int, int, int]) -> None:
            color = self._normalized_rgba(rgba)
            for spin, value in zip(spins, color):
                previous = spin.blockSignals(True)
                spin.setValue(int(value))
                spin.blockSignals(previous)

        def _button_rgba(self, button: QtWidgets.QPushButton, fallback) -> tuple[int, int, int, int]:
            stored = button.property("rgba")
            if stored is None:
                return self._normalized_rgba(fallback)
            return self._normalized_rgba(stored)

        def _set_button_rgba(self, button: QtWidgets.QPushButton, rgba, *, selected: bool = False) -> None:
            color = self._normalized_rgba(rgba)
            button.setProperty("rgba", color)
            button.setStyleSheet(self._color_button_style(color, selected=selected))
            button.setText(self._format_rgba_label(color))

        def _clip_color_from_rgba(self) -> tuple[int, int, int, int]:
            return self._button_rgba(
                self.clip_color_from_button,
                (
                    int(self.clip_r_spin.value()),
                    int(self.clip_g_spin.value()),
                    int(self.clip_b_spin.value()),
                    int(self.clip_a_spin.value()),
                ),
            )

        def _set_clip_color_from_rgba(self, rgba) -> None:
            color = self._normalized_rgba(rgba)
            self._set_hidden_rgba_spins(
                (self.clip_r_spin, self.clip_g_spin, self.clip_b_spin, self.clip_a_spin),
                color,
            )
            self._set_button_rgba(self.clip_color_from_button, color)

        def _clip_color_to_rgba(self) -> tuple[int, int, int, int]:
            return self._button_rgba(self.clip_color_to_button, self._clip_color_from_rgba())

        def _set_clip_color_to_rgba(self, rgba) -> None:
            self._set_button_rgba(self.clip_color_to_button, rgba)

        def _clip_effect_target_rgba(self) -> tuple[int, int, int, int]:
            return self._button_rgba(
                self.clip_effect_to_color_button,
                (
                    int(self.clip_to_r_spin.value()),
                    int(self.clip_to_g_spin.value()),
                    int(self.clip_to_b_spin.value()),
                    int(self.clip_to_a_spin.value()),
                ),
            )

        def _set_clip_effect_target_rgba(self, rgba) -> None:
            color = self._normalized_rgba(rgba)
            self._set_hidden_rgba_spins(
                (self.clip_to_r_spin, self.clip_to_g_spin, self.clip_to_b_spin, self.clip_to_a_spin),
                color,
            )
            self._set_button_rgba(self.clip_effect_to_color_button, color)

        def _update_clip_color_buttons(self) -> None:
            self._set_button_rgba(
                self.clip_color_from_button,
                self._clip_color_from_rgba(),
                selected=self.clip_color_picker_target == "from",
            )
            self._set_button_rgba(
                self.clip_color_to_button,
                self._clip_color_to_rgba(),
                selected=self.clip_color_picker_target == "to",
            )
            self._set_button_rgba(self.clip_effect_to_color_button, self._clip_effect_target_rgba())
            self.clip_color_from_button.setChecked(self.clip_color_picker_target == "from")
            self.clip_color_to_button.setChecked(self.clip_color_picker_target == "to")

        def _set_clip_color_picker_target(self, target: str) -> None:
            if target not in {"from", "to"}:
                target = "from"
            self.clip_color_picker_target = target
            self._update_clip_color_buttons()
            self._sync_embedded_color_picker_from_selection()

        def _current_picker_target_rgba(self) -> tuple[int, int, int, int]:
            if self._color_mode_value() == "cycle":
                return self._selected_palette_stop_rgba()
            if self.clip_color_picker_target == "to":
                return self._clip_color_to_rgba()
            return self._clip_color_from_rgba()

        def _sync_embedded_color_picker_from_selection(self) -> None:
            target_rgba = self._current_picker_target_rgba()
            color = QtGui.QColor(*target_rgba)
            previous = self.clip_embedded_color_picker.blockSignals(True)
            self.clip_embedded_color_picker.setCurrentColor(color)
            self.clip_embedded_color_picker.blockSignals(previous)

        def _on_embedded_color_picker_changed(self, color: QtGui.QColor) -> None:
            if self._ignore_clip_editor_changes or self.selected_clip is None or not color.isValid():
                return
            rgba = (color.red(), color.green(), color.blue(), color.alpha())
            if self._color_mode_value() == "cycle":
                self.clip_palette_strip.set_selected_stop_color(rgba)
                return
            elif self.clip_color_picker_target == "to":
                self._set_clip_color_to_rgba(rgba)
            else:
                self._set_clip_color_from_rgba(rgba)
            self._update_clip_color_buttons()
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _selected_palette_stop_rgba(self) -> tuple[int, int, int, int]:
            stops = self.clip_palette_strip.stops()
            index = self.clip_palette_strip.selected_index()
            if 0 <= index < len(stops):
                return self._normalized_rgba(stops[index][1])
            return self._clip_color_from_rgba()

        def _set_palette_strip_text(self, palette_text: str) -> None:
            stops = parse_palette_text(palette_text)
            if not stops:
                stops = [(0.0, self._clip_color_from_rgba()), (1.0, self._clip_color_to_rgba())]
            previous_strip = self.clip_palette_strip.blockSignals(True)
            previous_pos = self.clip_palette_position_spin.blockSignals(True)
            self.clip_palette_strip.set_stops(stops)
            self.clip_palette_position_spin.blockSignals(previous_pos)
            self.clip_palette_strip.blockSignals(previous_strip)
            self._sync_palette_controls_from_selection()

        def _sync_palette_controls_from_selection(self, *args) -> None:
            del args
            stops = self.clip_palette_strip.stops()
            index = self.clip_palette_strip.selected_index()
            has_selection = 0 <= index < len(stops)
            self.clip_palette_delete_stop_button.setEnabled(has_selection and len(stops) > 1 and self.selected_clip is not None)
            if has_selection:
                previous = self.clip_palette_position_spin.blockSignals(True)
                self.clip_palette_position_spin.setValue(float(stops[index][0]))
                self.clip_palette_position_spin.blockSignals(previous)
            self._sync_embedded_color_picker_from_selection()

        def _on_palette_strip_changed(self, stops: object) -> None:
            if self._ignore_clip_editor_changes or self.selected_clip is None:
                return
            stop_list = list(stops) if isinstance(stops, list) else self.clip_palette_strip.stops()
            self.clip_palette_text_edit.setText(encode_palette_text(stop_list))
            self._refresh_color_palette_preset_combo("__custom__")
            self._sync_palette_controls_from_selection()
            self._update_color_palette_summary()
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _on_palette_stop_selection_changed(self, index: int) -> None:
            del index
            self._sync_palette_controls_from_selection()

        def _on_palette_stop_position_changed(self, value: float) -> None:
            del value

        def _delete_selected_palette_stop(self) -> None:
            if self._ignore_clip_editor_changes or self.selected_clip is None:
                return
            stops = self.clip_palette_strip.stops()
            index = self.clip_palette_strip.selected_index()
            if not (0 <= index < len(stops)) or len(stops) <= 1:
                return
            del stops[index]
            self.clip_palette_strip.set_stops(stops)

        def _choose_color_value(self, title: str, initial_rgba: tuple[int, int, int, int]) -> tuple[int, int, int, int] | None:
            initial = QtGui.QColor(*self._normalized_rgba(initial_rgba))
            color = QtWidgets.QColorDialog.getColor(
                initial,
                self,
                title,
                QtWidgets.QColorDialog.ShowAlphaChannel,
            )
            if not color.isValid():
                return None
            return (color.red(), color.green(), color.blue(), color.alpha())

        def _choose_clip_color(self, target: str) -> None:
            if self.selected_clip is None:
                self.statusBar().showMessage("No clip selected")
                return
            if target == "from":
                rgba = self._choose_color_value("Select Clip Color", self._clip_color_from_rgba())
                if rgba is None:
                    return
                self._set_clip_color_from_rgba(rgba)
            elif target == "to":
                rgba = self._choose_color_value("Select Target Color", self._clip_color_to_rgba())
                if rgba is None:
                    return
                self._set_clip_color_to_rgba(rgba)
            else:
                rgba = self._choose_color_value("Select Effect Target Color", self._clip_effect_target_rgba())
                if rgba is None:
                    return
                self._set_clip_effect_target_rgba(rgba)
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _selected_color_palette_preset_name(self) -> str:
            data = self.clip_color_palette_preset_combo.currentData()
            if not isinstance(data, str) or data == "__custom__":
                return ""
            return data

        def _refresh_color_palette_preset_combo(self, preferred_preset: str | None = None) -> None:
            current_custom_text = self.clip_palette_text_edit.text().strip()
            project_palette_names: list[str] = []
            if self.project is not None:
                for name in sorted(self.project.palettes):
                    if name in BUILTIN_COLOR_PALETTES:
                        continue
                    if normalize_palette_store_text(self.project.palettes.get(name)):
                        project_palette_names.append(name)
            target_name = preferred_preset if preferred_preset is not None else self._selected_color_palette_preset_name()
            if target_name and target_name not in BUILTIN_COLOR_PALETTES and target_name not in project_palette_names:
                target_name = ""
            if not target_name and not current_custom_text:
                target_name = "Rainbow"
            if not target_name and current_custom_text:
                target_name = "__custom__"
            blocked = self.clip_color_palette_preset_combo.blockSignals(True)
            self.clip_color_palette_preset_combo.clear()
            for name in BUILTIN_COLOR_PALETTES:
                self.clip_color_palette_preset_combo.addItem(name, name)
            for name in project_palette_names:
                self.clip_color_palette_preset_combo.addItem(name, name)
            self.clip_color_palette_preset_combo.addItem("Custom", "__custom__")
            index = self.clip_color_palette_preset_combo.findData(target_name)
            if index < 0:
                index = self.clip_color_palette_preset_combo.findData("__custom__" if current_custom_text else "Rainbow")
            self.clip_color_palette_preset_combo.setCurrentIndex(max(0, index))
            self.clip_color_palette_preset_combo.blockSignals(blocked)

        def _resolved_color_palette_text(self) -> str:
            project_palettes = self.project.palettes if self.project is not None else None
            return resolve_palette_text(
                project_palettes,
                self._selected_color_palette_preset_name(),
                self.clip_palette_text_edit.text(),
            )

        def _update_color_palette_summary(self) -> None:
            resolved_text = self._resolved_color_palette_text()
            if not resolved_text:
                self.clip_palette_summary_label.setText("No palette")
            else:
                label = self._selected_color_palette_preset_name() or "Custom"
                stop_count = len([chunk for chunk in resolved_text.split(";") if chunk.strip()])
                stop_suffix = "stop" if stop_count == 1 else "stops"
                self.clip_palette_summary_label.setText(f"{label} · {stop_count} {stop_suffix}")
            selected_name = self._selected_color_palette_preset_name()
            can_delete = (
                self.project is not None
                and bool(selected_name)
                and selected_name not in BUILTIN_COLOR_PALETTES
                and selected_name in self.project.palettes
            )
            self.clip_color_palette_delete_button.setEnabled(can_delete and self.selected_clip is not None)
            self.clip_color_palette_save_button.setEnabled(self.selected_clip is not None and bool(resolved_text))

        def _update_color_tempo_sync_ui(self) -> None:
            tempo_sync = self.selected_clip is not None and self.clip_color_tempo_sync_checkbox.isChecked()
            self.clip_color_rate_label.setText("Cycles / Beat" if tempo_sync else "Frequency Hz")
            self.clip_color_rate_spin.setToolTip(
                "Color cycles per beat when BPM sync is enabled."
                if tempo_sync else
                "Color cycles per second."
            )

        def _color_mode_value(self) -> str:
            data = self.clip_color_mode_combo.currentData()
            if isinstance(data, str) and data:
                return data
            return "hold"

        def _update_color_mode_ui(self) -> None:
            mode = self._color_mode_value()
            show_from = mode in {"hold", "linear", "smooth"}
            show_to = mode in {"linear", "smooth"}
            show_cycle = mode == "cycle"
            self._set_form_row_visible(self.clip_color_form, self.clip_color_widget, show_from)
            self._set_form_row_visible(self.clip_color_form, self.clip_color_to_widget, show_to)
            self._set_form_row_visible(self.clip_color_form, self.clip_color_tempo_sync_checkbox, show_cycle)
            self._set_form_row_visible(self.clip_color_form, self.clip_color_rate_spin, show_cycle)
            self._set_form_row_visible(self.clip_color_form, self.clip_color_palette_preset_combo, show_cycle)
            self._set_form_row_visible(self.clip_color_form, self.clip_palette_summary_label, show_cycle)
            self._set_form_row_visible(self.clip_color_form, self.clip_palette_widget, show_cycle)
            self.clip_color_picker_widget.setVisible(show_from or show_to or show_cycle)
            self.clip_palette_editor_widget.setVisible(show_cycle)
            if show_from and not show_to and self.clip_color_picker_target != "from":
                self.clip_color_picker_target = "from"
                self._update_clip_color_buttons()
            self._sync_embedded_color_picker_from_selection()
            self._update_color_tempo_sync_ui()
            self._update_color_palette_summary()

        def _on_color_mode_changed(self, *args) -> None:
            del args
            if self._ignore_clip_editor_changes:
                return
            if self._color_mode_value() == "cycle" and not self._selected_color_palette_preset_name() and not self.clip_palette_text_edit.text().strip():
                self._refresh_color_palette_preset_combo("Rainbow")
            self._update_color_mode_ui()
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _on_color_tempo_sync_toggled(self, checked: bool) -> None:
            if checked:
                self.clip_color_rate_spin.setValue(1.0)
            self._update_color_tempo_sync_ui()
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _on_color_palette_preset_changed(self, *args) -> None:
            del args
            self._set_palette_strip_text(self._resolved_color_palette_text())
            self._update_color_palette_summary()
            if self._ignore_clip_editor_changes:
                return
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _open_color_palette_editor(self) -> None:
            if self.selected_clip is None:
                self.statusBar().showMessage("No clip selected")
                return
            dialog = PaletteEditorDialog(self._resolved_color_palette_text(), self)
            if dialog.exec() != QtWidgets.QDialog.Accepted:
                return
            self.clip_palette_text_edit.setText(dialog.result_text())
            self._refresh_color_palette_preset_combo("__custom__")
            self._update_color_palette_summary()
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _save_color_palette_preset(self) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            palette_text = self._resolved_color_palette_text()
            normalized = normalize_palette_store_text(palette_text)
            if not normalized:
                self.statusBar().showMessage("No palette to save")
                return
            suggested = self._selected_color_palette_preset_name()
            if not suggested or suggested in BUILTIN_COLOR_PALETTES:
                suggested = "My Palette"
            name, accepted = QtWidgets.QInputDialog.getText(self, "Save Palette Preset", "Preset name:", text=suggested)
            if not accepted:
                return
            preset_name = str(name).strip()
            if not preset_name:
                self.statusBar().showMessage("Palette preset name required")
                return
            self.project.palettes[preset_name] = normalized
            self._refresh_color_palette_preset_combo(preset_name)
            self._set_palette_strip_text(self._resolved_color_palette_text())
            self._update_color_palette_summary()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Saved palette preset {preset_name}")
            if self.selected_clip is not None:
                self._mark_clip_dirty()
                self._apply_clip_editor_if_dirty()

        def _delete_color_palette_preset(self) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            preset_name = self._selected_color_palette_preset_name()
            if not preset_name or preset_name in BUILTIN_COLOR_PALETTES:
                self.statusBar().showMessage("Selected palette is built-in")
                return
            if preset_name not in self.project.palettes:
                self.statusBar().showMessage("Palette preset not found")
                return
            del self.project.palettes[preset_name]
            fallback_name = "__custom__" if self.clip_palette_text_edit.text().strip() else "Rainbow"
            self._refresh_color_palette_preset_combo(fallback_name)
            self._set_palette_strip_text(self._resolved_color_palette_text())
            self._update_color_palette_summary()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Deleted palette preset {preset_name}")
            if self.selected_clip is not None:
                self._mark_clip_dirty()
                self._apply_clip_editor_if_dirty()

        def _snap_step_ms(self) -> int:
            if self.project is None:
                return 1
            return max(1, int(round(self._snap_step_value_ms())))

        def _snap_step_value_ms(self) -> float:
            if self.project is None:
                return 1.0
            divisor = int(self.snap_division_combo.currentData() or 1)
            divisor = max(1, divisor)
            if self.project.tempo_bpm > 0.0:
                return max(1.0, (60000.0 / self.project.tempo_bpm) / divisor)
            return max(1.0, float(self.project.bucket_ms) / divisor)

        def _snap_time_ms(self, time_ms: int) -> int:
            if not self.snap_checkbox.isChecked():
                return max(0, int(time_ms))
            step_ms = self._snap_step_value_ms()
            offset_ms = float(self.project.beat_offset_ms) if self.project is not None else 0.0
            snapped = offset_ms + round((float(time_ms) - offset_ms) / step_ms) * step_ms
            return max(0, int(round(snapped)))

        def _clone_clip(self, clip: ShowClip) -> ShowClip:
            return ShowClip(
                start_ms=int(clip.start_ms),
                end_ms=int(clip.end_ms),
                layer=int(clip.layer),
                effect=str(clip.effect),
                blend=str(clip.blend),
                target_kind=str(clip.target_kind),
                target=str(clip.target),
                palette=clip.palette,
                params=json.loads(json.dumps(clip.params)),
            )

        def _insert_clip(self, role: str, clip: ShowClip) -> None:
            if self.project is None:
                return
            if role == "*":
                self.project.global_clips.append(clip)
            else:
                self.project.role_clips.setdefault(role, []).append(clip)

        def _build_default_clip(
            self,
            effect_name: str,
            start_ms: int | None = None,
            layer: int = 0,
            end_ms: int | None = None,
        ):
            label, effect_id, target_kind, target, blend, _duration_ms, _params, _category = EFFECT_NAME_TO_SPEC[effect_name]
            del label
            if start_ms is None:
                start_ms = self._insert_reference_time_ms()
            start_ms = self._snap_time_ms(start_ms)
            if end_ms is None:
                default_duration_ms = self._default_insert_duration_ms()
                end_ms = min(self._timeline_max_ms() or start_ms + default_duration_ms, start_ms + default_duration_ms)
                if end_ms <= start_ms:
                    end_ms = start_ms + default_duration_ms
            else:
                end_ms = max(start_ms + 1, self._snap_time_ms(end_ms))
            return ShowClip(
                start_ms=start_ms,
                end_ms=end_ms,
                layer=max(0, layer),
                effect=effect_id,
                blend=blend,
                target_kind=target_kind,
                target=target,
                params=self._effect_default_params(effect_name),
            )

        def _default_insert_duration_ms(self) -> int:
            if self.project is None:
                return 1000
            if self.project.tempo_bpm > 0.0:
                return max(1, int(round(60000.0 / self.project.tempo_bpm)))
            return max(1, int(self.project.bucket_ms))

        def _insert_reference(self) -> tuple[str, int, int]:
            target = self.timeline_view.current_insert_target()
            if target is not None:
                return (target[0], target[1], self.timeline_view.current_insert_time_ms())
            fallback_role = str(self.new_clip_lane_combo.currentData() or "*")
            return (fallback_role, 0, self.time_slider.value())

        def _insert_reference_time_ms(self) -> int:
            return int(self._insert_reference()[2])

        def _insert_effect_clip(self, effect_name: str, lane_role: str, start_ms: int, layer: int = 0) -> ShowClip | None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return None
            if not self._flush_pending_clip_apply():
                return None
            self._record_undo_state(f"Add {effect_name} clip")
            new_clip = self._build_default_clip(effect_name, start_ms=start_ms, layer=layer)
            self._insert_clip(lane_role, new_clip)
            self._populate_timeline_table()
            self._select_clip(new_clip)
            self._refresh_preview()
            self.timeline_view.set_insert_cursor(lane_role, new_clip.layer, new_clip.end_ms)
            self._schedule_session_save()
            self.statusBar().showMessage(
                f"Added {effect_name} clip to {'Global' if lane_role == '*' else f'Suit {lane_role}'}"
            )
            return new_clip

        def _add_selected_effect(self, *args) -> None:
            del args
            item = self.effects_list.currentItem()
            effect_name = self._effect_name_from_item(item) or self._selected_effect_name()
            lane_role, layer, start_ms = self._insert_reference()
            self._insert_effect_clip(effect_name, lane_role, start_ms, layer=layer)

        def _quick_add_at_playhead(self) -> None:
            lane_role, layer, start_ms = self._insert_reference()
            self._insert_effect_clip(self._selected_effect_name(), lane_role, start_ms, layer=layer)

        def _create_clip_from_timeline(self, role: str, layer: int, time_ms: int) -> None:
            self._insert_effect_clip(self._selected_effect_name(), role, time_ms, layer=layer)

        def _create_clip_span_from_timeline(self, role: str, layer: int, start_ms: int, end_ms: int) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            if not self._flush_pending_clip_apply():
                return
            effect_name = self._selected_effect_name()
            self._record_undo_state(f"Draw {effect_name} clip")
            new_clip = self._build_default_clip(
                effect_name,
                start_ms=min(start_ms, end_ms),
                layer=layer,
                end_ms=max(start_ms + 1, end_ms),
            )
            self._insert_clip(role, new_clip)
            self._populate_timeline_table()
            self._select_clip(new_clip)
            self._refresh_preview()
            self.timeline_view.set_insert_cursor(role, new_clip.layer, new_clip.end_ms)
            self._schedule_session_save()
            self.statusBar().showMessage(
                f"Drew {effect_name} clip on {'Global' if role == '*' else f'Suit {role}'}"
            )

        def _begin_clip_drag_edit(self, clip, drag_kind: str) -> None:
            del clip
            if drag_kind == "resize_start":
                label = "Resize clip start"
            elif drag_kind == "resize_end":
                label = "Resize clip end"
            else:
                label = "Move clip"
            self._begin_pending_history(label)

        def _copy_selected_clip(self) -> None:
            selected_clips = self._selected_timeline_clips()
            if not selected_clips:
                self.statusBar().showMessage("No clip selected")
                return
            entries: list[dict] = []
            base_start_ms = min(int(clip.start_ms) for clip in selected_clips)
            base_layer = min(int(clip.layer) for clip in selected_clips)
            for clip in selected_clips:
                location = self._find_clip_location(clip)
                if location is None:
                    continue
                entries.append(
                    {
                        "role": location[0],
                        "clip": self._clone_clip(clip),
                        "start_offset_ms": int(clip.start_ms) - base_start_ms,
                        "layer_offset": int(clip.layer) - base_layer,
                    }
                )
            if not entries:
                self.statusBar().showMessage("Clip not found")
                return
            self.clip_clipboard = {
                "items": entries,
                "base_start_ms": base_start_ms,
                "base_layer": base_layer,
            }
            self.statusBar().showMessage("Clip copied" if len(entries) == 1 else f"Copied {len(entries)} clips")

        def _cut_selected_clip(self) -> None:
            if not self._selected_timeline_clips():
                self.statusBar().showMessage("No clip selected")
                return
            self._copy_selected_clip()
            if not self._selected_timeline_clips():
                return
            self._delete_selected_clip()
            copied_items = 0 if self.clip_clipboard is None else len(self.clip_clipboard.get("items", ()))
            self.statusBar().showMessage("Clip cut" if copied_items <= 1 else f"Cut {copied_items} clips")

        def _paste_clip(self) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            if not self._flush_pending_clip_apply():
                return
            if self.clip_clipboard is None:
                self.statusBar().showMessage("Clipboard is empty")
                return
            self._record_undo_state("Paste clip")
            clipboard_items = tuple(self.clip_clipboard.get("items", ()))
            if not clipboard_items:
                self.statusBar().showMessage("Clipboard is empty")
                return
            target = self.timeline_view.current_insert_target()
            insert_time_ms = self._insert_reference_time_ms()
            if target is None and self.selected_clip is not None:
                location = self._find_clip_location(self.selected_clip)
                if location is not None:
                    target = (location[0], self.selected_clip.layer)
            if target is None:
                fallback_role = str(self.new_clip_lane_combo.currentData() or clipboard_items[0].get("role", "*"))
                fallback_layer = int(clipboard_items[0].get("clip").layer if clipboard_items[0].get("clip") is not None else 0)
                target = (fallback_role, fallback_layer)
            base_paste_start_ms = self._snap_time_ms(insert_time_ms)
            target_role, target_layer = target
            pasted_clips: list[ShowClip] = []
            max_end_ms = base_paste_start_ms
            for entry in clipboard_items:
                clip_template = entry.get("clip")
                if not isinstance(clip_template, ShowClip):
                    continue
                clip = self._clone_clip(clip_template)
                duration = max(1, clip.end_ms - clip.start_ms)
                start_offset_ms = int(entry.get("start_offset_ms", 0))
                layer_offset = int(entry.get("layer_offset", 0))
                clip.start_ms = max(0, base_paste_start_ms + start_offset_ms)
                clip.end_ms = clip.start_ms + duration
                clip.layer = max(0, int(target_layer) + layer_offset)
                paste_role = target_role if target_role != "*" else str(entry.get("role", "*"))
                if target_role != "*" and str(entry.get("role", "*")) == "*":
                    paste_role = target_role
                self._insert_clip(paste_role, clip)
                pasted_clips.append(clip)
                max_end_ms = max(max_end_ms, clip.end_ms)
            if not pasted_clips:
                self.statusBar().showMessage("Clipboard is empty")
                return
            self._populate_timeline_table()
            self._set_clip_selection(pasted_clips, pasted_clips[0])
            self.timeline_view.set_active_lane(target_role, pasted_clips[0].layer)
            self.timeline_view.set_insert_cursor(target_role, pasted_clips[0].layer, max_end_ms)
            self._schedule_session_save()
            if len(pasted_clips) == 1:
                self.statusBar().showMessage(
                    f"Pasted to {'Global' if target_role == '*' else f'Suit {target_role}'} layer {pasted_clips[0].layer + 1}"
                )
            else:
                self.statusBar().showMessage(f"Pasted {len(pasted_clips)} clips")

        def _duplicate_selected_clip(self) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            if not self._flush_pending_clip_apply():
                return
            if self.selected_clip is None:
                self.statusBar().showMessage("No clip selected")
                return
            location = self._find_clip_location(self.selected_clip)
            if location is None:
                self.statusBar().showMessage("Clip not found")
                return
            self._record_undo_state("Duplicate clip")
            role, _ = location
            clip = self._clone_clip(self.selected_clip)
            duration = max(1, clip.end_ms - clip.start_ms)
            clip.start_ms = self._snap_time_ms(self.selected_clip.end_ms)
            clip.end_ms = clip.start_ms + duration
            self._insert_clip(role, clip)
            self._populate_timeline_table()
            self._select_clip(clip)
            self._set_timeline_position(clip.end_ms, sync_player=False)
            self._schedule_session_save()
            self.statusBar().showMessage("Clip duplicated")

        def _select_clip(self, clip, sync_table: bool = True) -> None:
            del sync_table
            clips = (clip,) if clip is not None else ()
            self._set_clip_selection(clips, clip)

        def _set_clip_selection(self, clips, focus_clip=None) -> None:
            clips_tuple = tuple(clip for clip in clips if clip is not None)
            normalized_focus = focus_clip if focus_clip in clips_tuple else (clips_tuple[0] if clips_tuple else None)
            if normalized_focus is not self.selected_clip:
                self._discard_pending_clip_edit()
            self.selected_clip = normalized_focus
            self._selected_clip_ids = {id(clip) for clip in clips_tuple}
            location = self._find_clip_location(normalized_focus)
            if location is not None:
                self.timeline_view.set_active_lane(location[0], normalized_focus.layer)
            self.timeline_view.set_selected_clips(clips_tuple)
            self._refresh_clip_editor()
            self.sidebar_tabs.setCurrentIndex(1)

        def _selected_timeline_clips(self) -> list[ShowClip]:
            clips = self._clips_from_id_set(self._selected_clip_ids)
            if clips:
                return clips
            return [self.selected_clip] if self.selected_clip is not None else []

        def _select_all_clips(self) -> None:
            clips = self._all_project_clips()
            if not clips:
                self.statusBar().showMessage("No clips to select")
                return
            self.selected_clip = clips[0]
            self._selected_clip_ids = {id(clip) for clip in clips}
            self.timeline_view.set_selected_clips(clips)
            self._refresh_clip_editor()
            self.statusBar().showMessage(f"Selected {len(clips)} clips")

        def _effect_default_params(self, effect_name: str) -> dict:
            spec = EFFECT_NAME_TO_SPEC[effect_name]
            params = spec[6]
            defaults = {
                key: (list(value) if isinstance(value, list) else value)
                for key, value in params.items()
            }
            effect_id = spec[1]
            if effect_id in EFFECT_FIELDS_RATE:
                defaults["tempo_sync"] = True
                defaults["frequency_hz"] = 1.0
                defaults["cycles_per_beat"] = 1.0
                defaults.pop("beats_per_cycle", None)
            base_color = defaults.get("color", [255, 255, 255, 255])
            if not isinstance(base_color, (list, tuple)) or len(base_color) != 4:
                base_color = [255, 255, 255, 255]
            to_color = defaults.get("to_color", base_color)
            if not isinstance(to_color, (list, tuple)) or len(to_color) != 4:
                to_color = base_color
            defaults["color"] = list(base_color)
            defaults["color_from"] = list(defaults.get("color_from", base_color))
            defaults["color_to"] = list(defaults.get("color_to", to_color))
            defaults["color_mode"] = str(defaults.get("color_mode", "hold"))
            defaults["color_tempo_sync"] = bool(defaults.get("color_tempo_sync", True))
            defaults["color_rate"] = float(defaults.get("color_rate", 1.0))
            defaults["color_palette_preset"] = str(defaults.get("color_palette_preset", "Rainbow"))
            return defaults

        def _set_effect_param_widgets(self, params: dict, effect_name: str) -> None:
            self.clip_axis_combo.setCurrentText(str(params.get("axis", "y")))
            self.clip_width_spin.setValue(float(params.get("width", 0.22)))
            self.clip_softness_spin.setValue(float(params.get("softness", params.get("width", 0.16))))
            tempo_sync = bool(params.get("tempo_sync", False))
            frequency_value = float(params.get("frequency_hz", 2.0))
            if tempo_sync:
                frequency_value = 1.0
                cycles_per_beat = float(params.get("cycles_per_beat", 0.0))
                if cycles_per_beat > 0.0:
                    frequency_value = cycles_per_beat
                else:
                    beats_per_cycle = float(params.get("beats_per_cycle", 0.0))
                    if beats_per_cycle > 0.0:
                        frequency_value = 1.0 / beats_per_cycle
            self.clip_frequency_spin.setValue(frequency_value)
            self.clip_tempo_sync_checkbox.setChecked(tempo_sync)
            self.clip_beats_per_cycle_spin.setValue(
                float(params.get("beats_per_cycle", 0.0 if tempo_sync else 1.0))
            )
            self.clip_phase_spin.setValue(float(params.get("phase", 0.0)))
            self.clip_repeats_spin.setValue(int(params.get("repeats", 4)))
            self.clip_duty_cycle_spin.setValue(float(params.get("duty_cycle", 0.5)))
            self.clip_min_intensity_spin.setValue(float(params.get("min_intensity", 0.15)))
            self.clip_max_intensity_spin.setValue(float(params.get("max_intensity", 1.0)))
            self.clip_reverse_checkbox.setChecked(bool(params.get("reverse", False)))
            self._sync_effect_param_enabled(effect_name)
            self._update_tempo_sync_ui()

        def _set_color_param_widgets(self, params: dict) -> None:
            base_color = params.get("color_from", params.get("color", [255, 255, 255, 255]))
            if self.clip_color_picker_target not in {"from", "to"}:
                self.clip_color_picker_target = "from"
            self._set_clip_color_from_rgba(base_color)
            self._set_clip_color_to_rgba(params.get("color_to", base_color))
            self._set_clip_effect_target_rgba(params.get("to_color", [0, 0, 0, 255]))
            self.clip_palette_text_edit.setText(str(params.get("palette_text", "")))
            color_mode = str(params.get("color_mode", "hold")).strip().lower()
            color_mode_index = self.clip_color_mode_combo.findData(color_mode)
            if color_mode_index < 0:
                color_mode_index = self.clip_color_mode_combo.findData("hold")
            self.clip_color_mode_combo.setCurrentIndex(max(0, color_mode_index))
            self.clip_color_tempo_sync_checkbox.setChecked(bool(params.get("color_tempo_sync", True)))
            self.clip_color_rate_spin.setValue(float(params.get("color_rate", 1.0)))
            preferred_preset = str(params.get("color_palette_preset", "")).strip()
            if not preferred_preset:
                preferred_preset = "__custom__" if self.clip_palette_text_edit.text().strip() else "Rainbow"
            self._refresh_color_palette_preset_combo(preferred_preset)
            self._set_palette_strip_text(self._resolved_color_palette_text())
            self._update_clip_color_buttons()
            self._update_color_mode_ui()

        def _sync_effect_param_enabled(self, effect_name: str) -> None:
            enabled = self.selected_clip is not None
            for widget in (
                self.clip_axis_combo,
                self.clip_width_spin,
                self.clip_softness_spin,
                self.clip_frequency_spin,
                self.clip_tempo_sync_checkbox,
                self.clip_beats_per_cycle_spin,
                self.clip_phase_spin,
                self.clip_repeats_spin,
                self.clip_duty_cycle_spin,
                self.clip_min_intensity_spin,
                self.clip_max_intensity_spin,
                self.clip_reverse_checkbox,
            ):
                widget.setEnabled(enabled)
            self._apply_effect_row_visibility(effect_name)
            self._update_tempo_sync_ui()

        def _merge_common_params(self, params: dict, effect_name: str, current_params: dict | None = None) -> dict:
            merged = dict(params)
            del effect_name
            del current_params
            merged["axis"] = self.clip_axis_combo.currentText()
            merged.pop("axis_random_mix", None)
            merged["width"] = float(self.clip_width_spin.value())
            merged["softness"] = float(self.clip_softness_spin.value())
            frequency_value = max(0.0, float(self.clip_frequency_spin.value()))
            tempo_sync = bool(self.clip_tempo_sync_checkbox.isChecked())
            merged["frequency_hz"] = frequency_value
            merged["tempo_sync"] = tempo_sync
            if tempo_sync:
                merged["cycles_per_beat"] = frequency_value
            else:
                merged.pop("cycles_per_beat", None)
            merged.pop("beats_per_cycle", None)
            merged["phase"] = float(self.clip_phase_spin.value())
            merged["repeats"] = int(self.clip_repeats_spin.value())
            merged["duty_cycle"] = float(self.clip_duty_cycle_spin.value())
            merged["min_intensity"] = float(self.clip_min_intensity_spin.value())
            merged["max_intensity"] = float(self.clip_max_intensity_spin.value())
            merged["reverse"] = bool(self.clip_reverse_checkbox.isChecked())
            merged["to_color"] = list(self._clip_effect_target_rgba())
            curve_text = self.clip_curve_text_edit.text().strip()
            if curve_text:
                merged["curve_text"] = curve_text
            else:
                merged.pop("curve_text", None)
            return merged

        def _on_clip_effect_changed(self, effect_name: str) -> None:
            if self._ignore_clip_editor_changes or self.selected_clip is None:
                return
            defaults = self._effect_default_params(effect_name)
            self._ignore_clip_editor_changes = True
            try:
                self._set_effect_param_widgets(defaults, effect_name)
                default_blend = EFFECT_NAME_TO_SPEC[effect_name][4]
                self.clip_blend_combo.setCurrentText(default_blend)
            finally:
                self._ignore_clip_editor_changes = False
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _refresh_clip_editor(self) -> None:
            self._set_clip_editor_dirty(False)
            self._ignore_clip_editor_changes = True
            try:
                clip = self.selected_clip
                if clip is None:
                    self.clip_status_label.setText("No clip selected")
                    self.clip_effect_value_label.setText("-")
                    self._set_clip_editor_enabled(False)
                    self._sync_effect_param_enabled(EFFECT_NAMES[0])
                    return
                self._set_clip_editor_enabled(True)
                location = self._find_clip_location(clip)
                lane_role = location[0] if location is not None else "*"
                self.clip_status_label.setText(
                    f"Selected: {EFFECT_ID_TO_NAME.get(clip.effect, clip.effect)} on "
                    f"{'Global' if lane_role == '*' else f'Suit {lane_role}'}"
                )
                self.clip_role_combo.setCurrentIndex(max(0, CLIP_ROLE_OPTIONS.index(lane_role)))
                self.clip_effect_value_label.setText(EFFECT_ID_TO_NAME.get(clip.effect, clip.effect.title()))
                self._set_target_mode(clip.target_kind, clip.target)
                self.clip_blend_combo.setCurrentText(clip.blend)
                self.clip_start_spin.setValue(max(0, clip.start_ms))
                self.clip_end_spin.setValue(max(1, clip.end_ms))
                self.clip_layer_spin.setValue(max(0, clip.layer))
                self.clip_curve_text_edit.setText(str(clip.params.get("curve_text", "")))
                self.clip_intensity_spin.setValue(float(clip.params.get("intensity", 1.0)))
                effect_name = EFFECT_ID_TO_NAME.get(clip.effect, clip.effect.title())
                self._set_effect_param_widgets(clip.params, effect_name)
                self._set_color_param_widgets(clip.params)
            finally:
                self._ignore_clip_editor_changes = False

        def _reset_clip_params(self) -> None:
            if self.selected_clip is None:
                return
            self._begin_pending_history("Reset clip params")
            effect_name = EFFECT_ID_TO_NAME.get(self.selected_clip.effect, self.selected_clip.effect.title())
            params = self._effect_default_params(effect_name)
            self._ignore_clip_editor_changes = True
            try:
                self._set_effect_param_widgets(params, effect_name)
            finally:
                self._ignore_clip_editor_changes = False
            self._ignore_clip_editor_changes = True
            try:
                self._set_color_param_widgets(params)
                self.clip_curve_text_edit.setText("")
            finally:
                self._ignore_clip_editor_changes = False
            self.clip_intensity_spin.setValue(float(params.get("intensity", 1.0)))
            self._mark_clip_dirty()
            self._apply_clip_editor_if_dirty()

        def _apply_clip_changes(self, quiet: bool = False) -> bool:
            if self.project is None or self.selected_clip is None or self._ignore_clip_editor_changes:
                return False
            clip = self.selected_clip
            location = self._find_clip_location(clip)
            new_role = self.clip_role_combo.currentData()
            start_ms = int(self.clip_start_spin.value())
            end_ms = max(start_ms + 1, int(self.clip_end_spin.value()))
            effect_name = EFFECT_ID_TO_NAME.get(clip.effect, clip.effect.title())
            new_effect = clip.effect
            params = self._effect_default_params(effect_name)
            new_target_kind, new_target = self._selected_target_config()
            new_blend = self.clip_blend_combo.currentText()
            new_layer = int(self.clip_layer_spin.value())
            params["color"] = list(self._clip_color_from_rgba())
            params["intensity"] = float(self.clip_intensity_spin.value())
            new_params = self._merge_common_params(params, effect_name, clip.params)
            new_params["color_from"] = list(self._clip_color_from_rgba())
            new_params["color_to"] = list(self._clip_color_to_rgba())
            new_params["color_mode"] = self._color_mode_value()
            new_params["color_tempo_sync"] = bool(self.clip_color_tempo_sync_checkbox.isChecked())
            new_params["color_rate"] = max(0.0, float(self.clip_color_rate_spin.value()))
            palette_preset_name = self._selected_color_palette_preset_name()
            if not palette_preset_name and self._color_mode_value() == "cycle" and not self.clip_palette_text_edit.text().strip():
                palette_preset_name = "Rainbow"
            if palette_preset_name:
                new_params["color_palette_preset"] = palette_preset_name
            else:
                new_params.pop("color_palette_preset", None)
            palette_text = self.clip_palette_text_edit.text().strip()
            if palette_text:
                new_params["palette_text"] = palette_text
            else:
                new_params.pop("palette_text", None)
            old_location = location[0] if location is not None else None
            changed = (
                clip.start_ms != start_ms
                or clip.end_ms != end_ms
                or clip.layer != new_layer
                or clip.effect != new_effect
                or clip.target_kind != new_target_kind
                or clip.target != new_target
                or clip.blend != new_blend
                or clip.params != new_params
                or new_role != old_location
            )
            if not changed:
                self._discard_pending_clip_edit()
                return False
            clip.start_ms = start_ms
            clip.end_ms = end_ms
            clip.layer = new_layer
            clip.effect = new_effect
            clip.target_kind = new_target_kind
            clip.target = new_target
            clip.blend = new_blend
            clip.params = new_params

            if location is not None and new_role != location[0]:
                old_role, old_index = location
                if old_role == "*":
                    self.project.global_clips.pop(old_index)
                else:
                    self.project.role_clips[old_role].pop(old_index)
                if new_role == "*":
                    self.project.global_clips.append(clip)
                else:
                    self.project.role_clips.setdefault(new_role, []).append(clip)

            self._populate_timeline_table()
            self._selected_clip_ids = {id(clip)}
            self.timeline_view.set_selected_clip(clip)
            self._refresh_clip_editor()
            self._refresh_preview()
            self._commit_pending_history()
            self._set_clip_editor_dirty(False)
            self._schedule_session_save()
            if not quiet:
                self.statusBar().showMessage("Clip updated")
            return True

        def _delete_selected_clip(self) -> None:
            if self.project is None:
                return
            if not self._flush_pending_clip_apply():
                return
            selected_clips = self._selected_timeline_clips()
            if not selected_clips and self.selected_clip is not None:
                selected_clips = [self.selected_clip]
            if not selected_clips:
                return
            self._record_undo_state("Delete clip" if len(selected_clips) == 1 else "Delete clips")
            selected_ids = {id(clip) for clip in selected_clips}
            self.project.global_clips = [
                clip for clip in self.project.global_clips
                if id(clip) not in selected_ids
            ]
            for role in ("A", "B", "C"):
                self.project.role_clips[role] = [
                    clip for clip in self.project.role_clips.get(role, [])
                    if id(clip) not in selected_ids
                ]
            self.selected_clip = None
            self._selected_clip_ids.clear()
            self.timeline_view.set_selected_clips(())
            self._populate_timeline_table()
            self._refresh_clip_editor()
            self._refresh_preview()
            self._schedule_session_save()
            if len(selected_clips) == 1:
                self.statusBar().showMessage("Clip deleted")
            else:
                self.statusBar().showMessage(f"Deleted {len(selected_clips)} clips")

        def _move_clip(
            self,
            clip,
            new_start_ms: int,
            new_end_ms: int,
            drag_kind: str = "move",
            new_role: str | None = None,
            new_layer: int | None = None,
        ) -> None:
            if clip is None:
                return
            location = self._find_clip_location(clip)
            current_role = location[0] if location is not None else "*"
            old_start = clip.start_ms
            old_end = clip.end_ms
            old_layer = clip.layer
            if drag_kind == "resize_start":
                snapped_start = self._snap_time_ms(int(new_start_ms))
                clip.start_ms = min(snapped_start, clip.end_ms - 1)
            elif drag_kind == "resize_end":
                snapped_end = self._snap_time_ms(int(new_end_ms))
                clip.end_ms = max(clip.start_ms + 1, snapped_end)
            else:
                duration = max(1, int(new_end_ms) - int(new_start_ms))
                snapped_start = self._snap_time_ms(int(new_start_ms))
                clip.start_ms = snapped_start
                clip.end_ms = max(snapped_start + 1, snapped_start + duration)
                if new_layer is not None:
                    clip.layer = max(0, int(new_layer))
            target_role = current_role if new_role is None else str(new_role)
            if clip.start_ms == old_start and clip.end_ms == old_end and clip.layer == old_layer and target_role == current_role:
                return
            if location is not None and target_role != current_role:
                old_role, old_index = location
                if old_role == "*":
                    self.project.global_clips.pop(old_index)
                else:
                    self.project.role_clips[old_role].pop(old_index)
                self._insert_clip(target_role, clip)
            self.selected_clip = clip
            self.timeline_view.set_active_lane(target_role, clip.layer)
            self._populate_timeline_table()
            self._refresh_clip_editor()
            self._refresh_preview()
            self._commit_pending_history()
            self._schedule_session_save()

        def _set_preview_mode(self, mode: str) -> None:
            self.preview_view.set_view_mode(mode)
            self._update_view_buttons()
            self.preview_view.update()

        def _set_camera_preset(self, preset: str) -> None:
            self.preview_view.set_camera_preset(preset)
            self._update_view_buttons()

        def _seek_audio_player(self, time_ms: int) -> None:
            if not self._has_audio_source() or self.audio_player is None:
                return
            if self._time_slider_scrubbing:
                return
            if abs(self.audio_player.position() - time_ms) > 40:
                self._set_playback_anchor(time_ms)
                self.audio_player.setPosition(time_ms)

        def _set_playback_anchor(self, position_ms: int, now_s: float | None = None) -> None:
            if now_s is None:
                now_s = time.monotonic()
            self._playback_anchor_pos_ms = max(0, int(position_ms))
            self._last_backend_audio_pos_ms = self._playback_anchor_pos_ms
            self._playback_anchor_monotonic_s = float(now_s)
            self._playback_anchor_valid = True

        def _clear_playback_anchor(self) -> None:
            self._playback_anchor_pos_ms = 0
            self._last_backend_audio_pos_ms = 0
            self._playback_anchor_monotonic_s = 0.0
            self._playback_anchor_valid = False

        def _playback_rate(self) -> float:
            if self.audio_player is None or not hasattr(self.audio_player, "playbackRate"):
                return 1.0
            try:
                return max(0.01, float(self.audio_player.playbackRate()))
            except (TypeError, ValueError):
                return 1.0

        def _predicted_audio_position_ms(self, observed_position_ms: int | None = None) -> int:
            now_s = time.monotonic()
            if observed_position_ms is None and self.audio_player is not None:
                observed_position_ms = int(self.audio_player.position())
            observed = max(0, int(observed_position_ms or 0))
            if not self._playback_anchor_valid:
                self._set_playback_anchor(observed, now_s)
            predicted_raw = self._playback_anchor_pos_ms + int(
                round((now_s - self._playback_anchor_monotonic_s) * 1000.0 * self._playback_rate())
            )
            error_ms = observed - predicted_raw
            if abs(error_ms) > 180:
                self._set_playback_anchor(observed, now_s)
                predicted_raw = observed
            elif abs(error_ms) > 2:
                corrected = predicted_raw + int(round(error_ms * 0.16))
                self._set_playback_anchor(corrected, now_s)
                predicted_raw = corrected
            self._last_backend_audio_pos_ms = observed
            return max(0, min(self.time_slider.maximum(), predicted_raw))

        def _set_timeline_position(self, time_ms: int, sync_player: bool = True) -> None:
            clamped = max(0, min(self.time_slider.maximum(), int(time_ms)))
            if self.time_slider.value() != clamped:
                self._ignore_time_slider_change = True
                self.time_slider.setValue(clamped)
                self._ignore_time_slider_change = False
            self._refresh_preview()
            if sync_player:
                self._seek_audio_player(clamped)
            self._update_transport_state()
            if not self._audio_is_playing():
                self._schedule_session_save()

        def _on_time_slider_changed(self, value: int) -> None:
            if self._ignore_time_slider_change:
                return
            self._refresh_preview(value)
            if not self._time_slider_scrubbing:
                self._seek_audio_player(value)
            self._update_transport_state()

        def _on_time_slider_pressed(self) -> None:
            self._time_slider_scrubbing = True

        def _on_time_slider_released(self) -> None:
            self._time_slider_scrubbing = False
            value = self.time_slider.value()
            self._refresh_preview(value)
            self._seek_audio_player(value)
            self._update_transport_state()
            self._schedule_session_save()

        def _on_audio_position_changed(self, position: int) -> None:
            if self._time_slider_scrubbing:
                return
            if self._audio_is_playing():
                if not self._playback_anchor_valid:
                    self._set_playback_anchor(position)
                else:
                    self._last_backend_audio_pos_ms = int(position)
                return
            now = time.monotonic()
            current_slider = self.time_slider.value()
            if (
                abs(position - current_slider) < 120
                and (now - self._last_audio_ui_update_s) < (1.0 / 30.0)
                and position < self.time_slider.maximum()
            ):
                return
            self._last_audio_ui_update_s = now
            self._set_timeline_position(position, sync_player=False)

        def _on_playback_sync_timer(self) -> None:
            if self.audio_player is None or not self._audio_is_playing():
                self._playback_sync_timer.stop()
                return
            position = self._predicted_audio_position_ms(int(self.audio_player.position()))
            if self._playback_range_ms is not None:
                range_start_ms, range_end_ms = self._playback_range_ms
                if position >= range_end_ms:
                    self.audio_player.pause()
                    self._set_playback_anchor(range_end_ms)
                    self._playback_range_ms = None
                    self._set_timeline_position(range_end_ms, sync_player=False)
                    self._update_transport_state()
                    return
                if position < range_start_ms:
                    position = range_start_ms
            self._set_timeline_position(position, sync_player=False)

        def _on_audio_duration_changed(self, duration: int) -> None:
            if self.project is not None:
                target_duration = int(duration)
                if self.audio_waveform is not None:
                    target_duration = int(self.audio_waveform.duration_ms)
                if target_duration > 0 and self.project.duration_ms != target_duration:
                    self.project.duration_ms = target_duration
                    self._schedule_session_save()
            del duration
            self._update_time_range()

        def _on_audio_playback_state_changed(self, state) -> None:
            if QtMultimedia is not None and state == QtMultimedia.QMediaPlayer.PlayingState:
                self._last_audio_ui_update_s = 0.0
                if self.audio_player is not None:
                    self._set_playback_anchor(int(self.audio_player.position()))
                if not self._playback_sync_timer.isActive():
                    self._playback_sync_timer.start()
            else:
                self._playback_sync_timer.stop()
                if self.audio_player is not None:
                    self._set_playback_anchor(int(self.audio_player.position()))
                else:
                    self._clear_playback_anchor()
                if QtMultimedia is not None and state == QtMultimedia.QMediaPlayer.StoppedState:
                    self._playback_range_ms = None
            self._update_transport_state()

        def _on_audio_error(self, *args) -> None:
            if self.audio_player is None:
                return
            error_text = self.audio_player.errorString()
            if error_text:
                self.statusBar().showMessage(f"Audio error: {error_text}")

        def _zoom_timeline_in(self) -> None:
            self.timeline_view.zoom_in(self.time_slider.value())

        def _zoom_timeline_out(self) -> None:
            self.timeline_view.zoom_out(self.time_slider.value())

        def _zoom_timeline_fit(self) -> None:
            self.timeline_view.reset_zoom(self.time_slider.value())

        def _on_timeline_viewport_changed(self, visible_start_ms: int, visible_duration_ms: int, max_visible_start_ms: int) -> None:
            self._ignore_timeline_scrollbar_change = True
            try:
                self.timeline_scrollbar.setRange(0, max(0, int(max_visible_start_ms)))
                self.timeline_scrollbar.setPageStep(max(1, int(visible_duration_ms)))
                self.timeline_scrollbar.setSingleStep(max(1, self._snap_step_ms()))
                self.timeline_scrollbar.setValue(int(visible_start_ms))
                self.timeline_scrollbar.setEnabled(max_visible_start_ms > 0)
            finally:
                self._ignore_timeline_scrollbar_change = False

        def _on_timeline_scrollbar_changed(self, value: int) -> None:
            if self._ignore_timeline_scrollbar_change:
                return
            self.timeline_view.set_visible_start_ms(int(value))

        def _on_tempo_changed(self, value: float) -> None:
            if self.project is None:
                return
            if abs(self.project.tempo_bpm - float(value)) < 1e-9:
                return
            self._record_undo_state("Change tempo")
            self.project.tempo_bpm = float(value)
            self._refresh_preview()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Tempo set to {value:.2f} BPM")

        def _on_beat_offset_changed(self, value: int) -> None:
            if self.project is None:
                return
            if self.project.beat_offset_ms == int(value):
                return
            self._record_undo_state("Change beat offset")
            self.project.beat_offset_ms = int(value)
            self._refresh_preview()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Beat offset set to {value} ms")

        def _play_audio(self, playback_range_ms: tuple[int, int] | None = None) -> None:
            if not self._has_audio_source() or self.audio_player is None:
                if not QT_MULTIMEDIA_AVAILABLE:
                    self.statusBar().showMessage(
                        f"Qt multimedia unavailable: {MULTIMEDIA_IMPORT_ERROR}"
                    )
                else:
                    self.statusBar().showMessage("No audio file loaded")
                return
            if playback_range_ms is not None:
                start_ms, end_ms = playback_range_ms
                start_ms = max(0, int(start_ms))
                end_ms = max(start_ms + 1, int(end_ms))
                self._playback_range_ms = (start_ms, end_ms)
                self._set_timeline_position(start_ms, sync_player=False)
                self._seek_audio_player(start_ms)
                self._set_playback_anchor(start_ms)
            else:
                self._playback_range_ms = None
                self._seek_audio_player(self.time_slider.value())
                self._set_playback_anchor(self.time_slider.value())
            self.audio_player.play()
            self._update_transport_state()

        def _pause_audio(self) -> None:
            if self.audio_player is None:
                return
            self._playback_range_ms = None
            self.audio_player.pause()
            self._set_playback_anchor(int(self.audio_player.position()))
            self._update_transport_state()

        def _stop_audio(self) -> None:
            self._playback_range_ms = None
            if self.audio_player is not None:
                self.audio_player.stop()
            self._clear_playback_anchor()
            self._set_timeline_position(0, sync_player=False)

        def _restart_audio(self) -> None:
            self._playback_range_ms = None
            self._set_timeline_position(0, sync_player=True)
            self._set_playback_anchor(0)
            if self._has_audio_source() and self.audio_player is not None:
                self.audio_player.play()
            self._update_transport_state()

        def _toggle_play_pause_from_cursor(self) -> None:
            if not self._has_audio_source() or self.audio_player is None or QtMultimedia is None:
                self._play_audio()
                return
            if self.audio_player.playbackState() == QtMultimedia.QMediaPlayer.PlayingState:
                self._pause_audio()
                return
            selected_range = self.timeline_view.selection_range()
            if selected_range is not None:
                self._play_audio(selected_range)
                return
            self._play_audio()

        def _load_example(self) -> None:
            example = Path(__file__).resolve().parent / "example_project.json"
            self.set_project(load_project(example))

        def _repo_root_directory(self) -> Path:
            return Path(__file__).resolve().parents[2]

        def _default_project_directory(self) -> Path:
            target = self._repo_root_directory() / "shows"
            target.mkdir(parents=True, exist_ok=True)
            return target

        def _new_project_from_audio_dialog(self) -> None:
            if self.project is not None and not self._flush_pending_clip_apply():
                return
            start_dir = self._default_project_directory()
            if self.project is not None:
                current_audio = resolve_audio_path(self.project)
                if current_audio is not None:
                    start_dir = current_audio.parent
                elif self.project.source_path is not None:
                    start_dir = self.project.source_path.parent
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Create Project From Audio",
                str(start_dir),
                "Audio Files (*.wav *.mp3 *.flac *.ogg *.m4a);;All Files (*)",
            )
            if not path:
                return
            duration_ms = 0
            try:
                duration_ms = load_audio_waveform(path).duration_ms
            except Exception:
                duration_ms = 0
            self.set_project(create_project_from_audio_path(path, duration_ms=duration_ms))
            self.statusBar().showMessage(f"Created project from {Path(path).name}")

        def _open_project_dialog(self) -> None:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Open Show Project",
                str(self._default_project_directory()),
                "JSON Files (*.json);;All Files (*)",
            )
            if not path:
                return
            self.set_project(load_project(path))

        def _open_project_directory(self) -> None:
            target_dir: Path | None = None
            if self.project is not None and self.project.source_path is not None:
                target_dir = self.project.source_path.parent
            elif self.project is not None:
                audio_path = resolve_audio_path(self.project)
                if audio_path is not None:
                    target_dir = audio_path.parent
            if target_dir is None:
                target_dir = self._default_project_directory()
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(target_dir.resolve())))
            self.statusBar().showMessage(f"Opened {target_dir.resolve()}")

        def _save_project(self) -> bool:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return False
            if not self._flush_pending_clip_apply():
                return False
            if self.project.source_path is None:
                return self._save_project_as()
            return self._save_project_to(self.project.source_path)

        def _save_project_as(self) -> bool:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return False
            if not self._flush_pending_clip_apply():
                return False
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "Save Show Project",
                str(self.project.source_path or (self._default_project_directory() / f"{self.project.slug}.json")),
                "JSON Files (*.json);;All Files (*)",
            )
            if not path:
                self.statusBar().showMessage("Save canceled")
                return False
            return self._save_project_to(path)

        def _save_project_to(self, path: str | Path) -> bool:
            if self.project is None:
                return False
            previous_audio_path = resolve_audio_path(self.project)
            target_path = Path(path).resolve()
            self.project.source_path = target_path
            if previous_audio_path is not None:
                self.project.audio_source = to_project_relative_path(self.project, previous_audio_path)
            save_project(self.project, target_path)
            self._saved_project_signature = self._project_signature(self.project)
            current_audio_path = resolve_audio_path(self.project)
            if current_audio_path != previous_audio_path:
                self._load_project_audio()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Saved {target_path.name}")
            return True

        def _load_layout_dialog(self) -> None:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Open LED Layout",
                str(Path.cwd()),
                "Layout Files (*.txt);;All Files (*)",
            )
            if not path:
                return
            resolved_path = Path(path).resolve()
            self.preview_layout_data = load_layout_file(resolved_path)
            self._layout_source_kind = "file"
            self._layout_source_path = resolved_path
            self.layout_label.setText(f"layout: {Path(path).name}")
            self.statusBar().showMessage(f"Loaded layout {Path(path).name}")
            self._refresh_preview()
            self._schedule_session_save()

        def _use_generated_layout(self) -> None:
            self.preview_layout_data = generated_layout()
            self._layout_source_kind = "generated"
            self._layout_source_path = None
            self.layout_label.setText("layout: generated")
            self.statusBar().showMessage("Using generated layout")
            self._refresh_preview()
            self._schedule_session_save()

        def _set_audio_file_dialog(self) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            start_dir = Path.cwd()
            current_audio = resolve_audio_path(self.project)
            if current_audio is not None:
                start_dir = current_audio.parent
            elif self.project.source_path is not None:
                start_dir = self.project.source_path.parent
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Set Audio File",
                str(start_dir),
                "Audio Files (*.wav *.mp3 *.flac *.ogg *.m4a);;All Files (*)",
            )
            if not path:
                return
            if not self._flush_pending_clip_apply():
                return
            self._record_undo_state("Change audio file")
            self.project.audio_source = to_project_relative_path(self.project, path)
            self._load_project_audio()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Audio set to {Path(path).name}")

        def _export_dialog(self) -> None:
            if self.project is None:
                self.statusBar().showMessage("No project loaded")
                return
            if not self._flush_pending_clip_apply():
                return
            out_dir = QtWidgets.QFileDialog.getExistingDirectory(
                self,
                "Export Show Package",
                str(Path.cwd()),
            )
            if not out_dir:
                return
            result = export_show(self.project, out_dir, copy_audio=False)
            self.statusBar().showMessage(f"Exported {result.show_bin_path.name}")

        def set_project(
            self,
            project: ShowProject,
            reset_history: bool = True,
            time_ms: int | None = None,
            selected_locator: dict | None = None,
            reload_audio: bool = True,
        ) -> None:
            self.project = project
            self._saved_project_signature = self._saved_signature_from_disk(project)
            self.selected_clip = None
            self._selected_clip_ids.clear()
            self.timeline_rows = []
            self._discard_pending_clip_edit()
            self.timeline_view.set_selected_clips(())
            if reload_audio and self.audio_player is not None:
                self.audio_player.stop()
            self.tempo_bpm_spin.blockSignals(True)
            self.beat_offset_spin.blockSignals(True)
            self.tempo_bpm_spin.setValue(project.tempo_bpm)
            self.beat_offset_spin.setValue(project.beat_offset_ms)
            self.tempo_bpm_spin.blockSignals(False)
            self.beat_offset_spin.blockSignals(False)
            if reload_audio:
                self._load_project_audio()
            self._populate_timeline_table()
            self._update_time_range()
            selected_clip = self._clip_from_locator(selected_locator)
            if selected_clip is not None:
                self.selected_clip = selected_clip
                self._selected_clip_ids = {id(selected_clip)}
                self.timeline_view.set_selected_clip(selected_clip)
            if reset_history:
                self._reset_history()
            self._set_timeline_position(0 if time_ms is None else int(time_ms), sync_player=False)
            self._refresh_clip_editor()
            self._update_history_actions()
            self._schedule_session_save()
            self.statusBar().showMessage(f"Loaded {project.slug}")

        def _load_project_audio(self) -> None:
            self.audio_waveform = None
            if self.audio_player is not None:
                self.audio_player.stop()
                self.audio_player.setSource(QtCore.QUrl())

            if self.project is None:
                self.audio_label.setText("audio: none")
                self._update_time_range()
                self._refresh_timeline()
                self._update_transport_state()
                return

            audio_path = resolve_audio_path(self.project)
            if audio_path is None:
                self.audio_label.setText("audio: none")
                self._update_time_range()
                self._refresh_timeline()
                self._update_transport_state()
                return

            if not audio_path.exists():
                self.audio_label.setText(f"audio: missing {audio_path.name}")
                self._update_time_range()
                self._refresh_timeline()
                self._update_transport_state()
                return

            label_suffix = ""
            if audio_path.suffix.lower() == ".wav":
                try:
                    self.audio_waveform = load_audio_waveform(audio_path)
                    if self.project is not None and self.project.duration_ms != int(self.audio_waveform.duration_ms):
                        self.project.duration_ms = int(self.audio_waveform.duration_ms)
                    label_suffix = f" {self.audio_waveform.duration_ms / 1000.0:.2f}s"
                except Exception as exc:
                    label_suffix = f" ({exc})"
            else:
                label_suffix = " (no waveform)"

            if self.audio_player is not None:
                self.audio_player.setSource(QtCore.QUrl.fromLocalFile(str(audio_path.resolve())))

            self.audio_label.setText(f"audio: {audio_path.name}{label_suffix}")
            self._update_time_range()
            self._refresh_timeline()
            self._update_transport_state()

        def _populate_timeline_table(self) -> None:
            self.timeline_rows = []
            if self.project is None:
                return
            self.timeline_rows = build_timeline_rows(self.project)

        def _refresh_timeline(self) -> None:
            role = self._timeline_focus_role()
            divisor = int(self.snap_division_combo.currentData() or 1)
            effect_name = self._selected_effect_name()
            effect_duration_ms = EFFECT_NAME_TO_SPEC[effect_name][5]
            self.timeline_view.set_timeline_data(
                self.project,
                self.audio_waveform,
                self.time_slider.value(),
                role,
                auto_scroll=not self._audio_is_playing(),
            )
            self.timeline_view.set_snap_settings(self.snap_checkbox.isChecked(), divisor)
            self.timeline_view.set_insert_preview(effect_name, effect_duration_ms)
            selected_clips = self._selected_timeline_clips()
            if len(selected_clips) > 1:
                self.timeline_view.set_selected_clips(selected_clips)
            else:
                self.timeline_view.set_selected_clip(self.selected_clip)
            self._on_timeline_viewport_changed(*self.timeline_view.viewport_state())

        def _dispatch_preview_request(self, request: dict) -> None:
            self._preview_worker_busy = True
            self._preview_render_proxy.render_requested.emit(request)

        def _on_preview_rendered(self, result: object) -> None:
            payload = dict(result) if isinstance(result, dict) else {}
            self._preview_worker_busy = False
            request_id = int(payload.get("request_id", 0))
            if request_id != self._preview_request_counter:
                if self._pending_worker_request is not None:
                    pending = self._pending_worker_request
                    self._pending_worker_request = None
                    self._dispatch_preview_request(pending)
                return
            error_text = str(payload.get("error", "")).strip()
            if error_text:
                self.statusBar().showMessage(f"Preview render error: {error_text}")
            self.preview_view.set_preview_data(
                payload.get("layout_data", self.preview_layout_data),
                dict(payload.get("frames", {})),
                str(payload.get("active_role", self._active_preview_role())),
                bool(payload.get("show_all_roles", self._show_all_preview_roles())),
            )
            if self._pending_worker_request is not None:
                pending = self._pending_worker_request
                self._pending_worker_request = None
                self._dispatch_preview_request(pending)

        def _flush_preview_refresh(self) -> None:
            self._preview_refresh_queued = False
            self._preview_request_counter += 1
            request_id = self._preview_request_counter
            current_ms = int(self._pending_preview_time_ms)
            active_role = self._active_preview_role()
            show_all_roles = self._show_all_preview_roles()
            if self.project is None:
                self._pending_worker_request = None
                self.preview_view.set_preview_data(
                    self.preview_layout_data,
                    {},
                    active_role,
                    show_all_roles,
                )
                return
            preview_roles = PREVIEW_ROLE_ORDER if show_all_roles else (active_role,)
            request = {
                "request_id": request_id,
                "project": self.project,
                "layout_data": self.preview_layout_data,
                "preview_roles": preview_roles,
                "time_ms": current_ms,
                "active_role": active_role,
                "show_all_roles": show_all_roles,
            }
            if self._preview_worker_busy:
                self._pending_worker_request = request
                return
            self._dispatch_preview_request(request)

        def _refresh_preview(self, time_ms: int | None = None) -> None:
            current_ms = self.time_slider.value() if time_ms is None else int(time_ms)
            self.time_label.setText(f"{current_ms / 1000.0:.3f} s")
            self._refresh_timeline()
            self._pending_preview_time_ms = current_ms
            if self._preview_refresh_queued:
                return
            self._preview_refresh_queued = True
            self._preview_refresh_timer.start()

        def _focused_text_widget(self):
            focus_widget = self.focusWidget()
            if focus_widget is None:
                return None
            if isinstance(
                focus_widget,
                (
                    QtWidgets.QAbstractItemView,
                    QtWidgets.QComboBox,
                    QtWidgets.QLineEdit,
                    QtWidgets.QPlainTextEdit,
                    QtWidgets.QTextEdit,
                    QtWidgets.QAbstractSpinBox,
                ),
            ):
                return focus_widget
            parent_widget = focus_widget.parentWidget()
            if isinstance(parent_widget, QtWidgets.QComboBox):
                return parent_widget
            return None

        def _timeline_step_ms(self, multiplier: int = 1) -> int:
            base_step = self._snap_step_ms() if self.snap_checkbox.isChecked() else 50
            return max(1, int(base_step) * max(1, multiplier))

        def _navigate_clip_selection(self, direction: int) -> bool:
            if not self.timeline_rows:
                return False
            rows = self.timeline_rows
            if self.selected_clip is not None:
                try:
                    current_index = next(index for index, row in enumerate(rows) if row.clip is self.selected_clip)
                except StopIteration:
                    current_index = -1
                next_index = current_index + direction
                if 0 <= next_index < len(rows):
                    target_clip = rows[next_index].clip
                    self._select_clip(target_clip)
                    self._set_timeline_position(target_clip.start_ms, sync_player=False)
                    return True
                return False
            current_time = self.time_slider.value()
            if direction < 0:
                candidates = [row for row in rows if row.clip.start_ms < current_time]
                if not candidates:
                    return False
                target_clip = candidates[-1].clip
            else:
                candidates = [row for row in rows if row.clip.start_ms > current_time]
                if not candidates:
                    return False
                target_clip = candidates[0].clip
            self._select_clip(target_clip)
            self._set_timeline_position(target_clip.start_ms, sync_player=False)
            return True

        def keyPressEvent(self, event) -> None:  # type: ignore[override]
            if self._focused_text_widget() is not None:
                super().keyPressEvent(event)
                return
            key = event.key()
            modifiers = event.modifiers()
            if key in (QtCore.Qt.Key_Left, QtCore.Qt.Key_Right):
                step = self._timeline_step_ms(4 if modifiers & QtCore.Qt.ShiftModifier else 1)
                delta = -step if key == QtCore.Qt.Key_Left else step
                self._set_timeline_position(self.time_slider.value() + delta, sync_player=False)
                event.accept()
                return
            if key == QtCore.Qt.Key_Up:
                if self._navigate_clip_selection(-1):
                    event.accept()
                    return
            if key == QtCore.Qt.Key_Down:
                if self._navigate_clip_selection(1):
                    event.accept()
                    return
            if key == QtCore.Qt.Key_Home:
                self._set_timeline_position(0, sync_player=False)
                event.accept()
                return
            if key == QtCore.Qt.Key_End:
                self._set_timeline_position(self.time_slider.maximum(), sync_player=False)
                event.accept()
                return
            super().keyPressEvent(event)

        def closeEvent(self, event) -> None:  # type: ignore[override]
            if not self._confirm_close_with_unsaved_changes():
                event.ignore()
                return
            self._preview_refresh_timer.stop()
            self._session_save_timer.stop()
            self._write_editor_session()
            if hasattr(self, "_preview_render_thread"):
                self._preview_render_thread.quit()
                self._preview_render_thread.wait(1500)
            event.accept()
            super().closeEvent(event)


else:

    class ShowEditorMainWindow:
        def __init__(
            self,
            project: ShowProject | None = None,
            startup_session: EditorSessionState | None = None,
        ) -> None:
            del project, startup_session
            raise RuntimeError(
                "PySide6 is required to run the editor UI scaffold. "
                f"Current import error: {IMPORT_ERROR}"
            )
