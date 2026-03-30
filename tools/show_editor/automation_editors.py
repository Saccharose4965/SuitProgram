from __future__ import annotations

from dataclasses import dataclass

from .color_palettes import encode_palette_text_even, normalize_palette_stops_even, parse_palette_text_even
from .qt_compat import PYSIDE_AVAILABLE, QtCore, QtGui, QtWidgets


if PYSIDE_AVAILABLE:

    @dataclass(frozen=True)
    class CurveParameterSpec:
        key: str
        label: str
        min_value: float
        max_value: float


    def _fmt_float(value: float) -> str:
        text = f"{float(value):.4f}"
        text = text.rstrip("0").rstrip(".")
        return text or "0"


    def parse_curve_text(text: str) -> dict[str, list[tuple[float, float]]]:
        curves: dict[str, list[tuple[float, float]]] = {}
        for segment in str(text).split(";"):
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
                    point_t = max(0.0, min(1.0, float(time_text.strip())))
                    point_value = float(value_text.strip())
                except ValueError:
                    continue
                points.append((point_t, point_value))
            if points:
                points.sort(key=lambda item: item[0])
                curves[key] = points
        return curves


    def encode_curve_text(curves: dict[str, list[tuple[float, float]]]) -> str:
        parts: list[str] = []
        for key in sorted(curves.keys()):
            points = curves[key]
            if not points:
                continue
            point_text = ",".join(f"{_fmt_float(t)}:{_fmt_float(v)}" for t, v in sorted(points, key=lambda item: item[0]))
            parts.append(f"{key}={point_text}")
        return "; ".join(parts)


    def parse_palette_text(text: str) -> list[tuple[float, tuple[int, int, int, int]]]:
        return parse_palette_text_even(text)


    def encode_palette_text(stops: list[tuple[float, tuple[int, int, int, int]]]) -> str:
        return encode_palette_text_even(stops)


    def _sample_palette_color(stops: list[tuple[float, tuple[int, int, int, int]]], t: float) -> tuple[int, int, int, int]:
        if not stops:
            return (255, 255, 255, 255)
        t = max(0.0, min(1.0, float(t)))
        if t <= stops[0][0]:
            return stops[0][1]
        if t >= stops[-1][0]:
            return stops[-1][1]
        for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
            if t0 <= t <= t1:
                if abs(t1 - t0) <= 1e-6:
                    return c1
                local_t = (t - t0) / (t1 - t0)
                return tuple(
                    max(0, min(255, int(round(c0[index] + (c1[index] - c0[index]) * local_t))))
                    for index in range(4)
                )  # type: ignore[return-value]
        return stops[-1][1]


    class CurveGraphWidget(QtWidgets.QWidget):
        points_changed = QtCore.Signal(object)

        def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
            super().__init__(parent)
            self.setMinimumSize(420, 240)
            self.setFocusPolicy(QtCore.Qt.StrongFocus)
            self._points: list[tuple[float, float]] = []
            self._selected_index = -1
            self._dragging = False
            self._min_value = 0.0
            self._max_value = 1.0

        def set_value_range(self, min_value: float, max_value: float) -> None:
            self._min_value = float(min_value)
            self._max_value = float(max(max_value, min_value + 1e-6))
            self.update()

        def set_points(self, points: list[tuple[float, float]]) -> None:
            self._points = sorted(
                [(max(0.0, min(1.0, float(t))), float(v)) for t, v in points],
                key=lambda item: item[0],
            )
            self._selected_index = min(self._selected_index, len(self._points) - 1)
            self.update()

        def points(self) -> list[tuple[float, float]]:
            return list(self._points)

        def _plot_rect(self) -> QtCore.QRectF:
            return QtCore.QRectF(40.0, 12.0, max(120.0, self.width() - 56.0), max(100.0, self.height() - 34.0))

        def _point_to_widget(self, point_t: float, point_v: float) -> QtCore.QPointF:
            rect = self._plot_rect()
            x = rect.left() + point_t * rect.width()
            value_span = max(1e-6, self._max_value - self._min_value)
            normalized = (point_v - self._min_value) / value_span
            y = rect.bottom() - normalized * rect.height()
            return QtCore.QPointF(x, y)

        def _widget_to_point(self, pos: QtCore.QPointF) -> tuple[float, float]:
            rect = self._plot_rect()
            point_t = (pos.x() - rect.left()) / max(1.0, rect.width())
            point_t = max(0.0, min(1.0, point_t))
            normalized = (rect.bottom() - pos.y()) / max(1.0, rect.height())
            normalized = max(0.0, min(1.0, normalized))
            point_v = self._min_value + normalized * (self._max_value - self._min_value)
            return (point_t, point_v)

        def _hit_point_index(self, pos: QtCore.QPointF) -> int:
            for index, (point_t, point_v) in enumerate(self._points):
                widget_point = self._point_to_widget(point_t, point_v)
                if QtCore.QLineF(widget_point, pos).length() <= 8.0:
                    return index
            return -1

        def _emit_points_changed(self) -> None:
            self.points_changed.emit(list(self._points))

        def paintEvent(self, event) -> None:  # type: ignore[override]
            del event
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            rect = self._plot_rect()
            painter.fillRect(self.rect(), QtGui.QColor("#0f1620"))
            painter.fillRect(rect, QtGui.QColor("#131b26"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#263241"), 1.0))
            painter.drawRect(rect)
            for index in range(1, 4):
                x = rect.left() + rect.width() * index / 4.0
                y = rect.top() + rect.height() * index / 4.0
                painter.drawLine(QtCore.QPointF(x, rect.top()), QtCore.QPointF(x, rect.bottom()))
                painter.drawLine(QtCore.QPointF(rect.left(), y), QtCore.QPointF(rect.right(), y))
            painter.setPen(QtGui.QColor("#8fa4ba"))
            painter.drawText(QtCore.QRectF(4.0, rect.top() - 2.0, 32.0, 16.0), int(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter), _fmt_float(self._max_value))
            painter.drawText(QtCore.QRectF(4.0, rect.bottom() - 14.0, 32.0, 16.0), int(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter), _fmt_float(self._min_value))
            painter.drawText(QtCore.QRectF(rect.left() - 10.0, rect.bottom() + 4.0, 20.0, 16.0), int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter), "0")
            painter.drawText(QtCore.QRectF(rect.right() - 10.0, rect.bottom() + 4.0, 20.0, 16.0), int(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter), "1")
            if self._points:
                path = QtGui.QPainterPath(self._point_to_widget(*self._points[0]))
                for point_t, point_v in self._points[1:]:
                    path.lineTo(self._point_to_widget(point_t, point_v))
                painter.setPen(QtGui.QPen(QtGui.QColor("#7aa2f7"), 2.0))
                painter.drawPath(path)
            for index, (point_t, point_v) in enumerate(self._points):
                widget_point = self._point_to_widget(point_t, point_v)
                radius = 5.5 if index == self._selected_index else 4.0
                painter.setBrush(QtGui.QColor("#ffffff") if index == self._selected_index else QtGui.QColor("#7aa2f7"))
                painter.setPen(QtGui.QPen(QtGui.QColor("#0b0f14"), 1.0))
                painter.drawEllipse(widget_point, radius, radius)
            painter.end()

        def mousePressEvent(self, event) -> None:  # type: ignore[override]
            if event.button() == QtCore.Qt.LeftButton:
                self.setFocus(QtCore.Qt.MouseFocusReason)
                hit_index = self._hit_point_index(event.position())
                if hit_index >= 0:
                    self._selected_index = hit_index
                    self._dragging = True
                else:
                    self._points.append(self._widget_to_point(event.position()))
                    self._points.sort(key=lambda item: item[0])
                    self._selected_index = min(range(len(self._points)), key=lambda idx: abs(self._points[idx][0] - self._widget_to_point(event.position())[0]))
                    self._dragging = True
                    self._emit_points_changed()
                self.update()
                event.accept()
                return
            if event.button() == QtCore.Qt.RightButton:
                hit_index = self._hit_point_index(event.position())
                if hit_index >= 0:
                    del self._points[hit_index]
                    self._selected_index = min(hit_index, len(self._points) - 1)
                    self._emit_points_changed()
                    self.update()
                    event.accept()
                    return
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
            if self._dragging and 0 <= self._selected_index < len(self._points):
                self._points[self._selected_index] = self._widget_to_point(event.position())
                current_point = self._points[self._selected_index]
                self._points.sort(key=lambda item: item[0])
                self._selected_index = min(range(len(self._points)), key=lambda idx: abs(self._points[idx][0] - current_point[0]) + abs(self._points[idx][1] - current_point[1]))
                self._emit_points_changed()
                self.update()
                event.accept()
                return
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
            self._dragging = False
            super().mouseReleaseEvent(event)


    class PaletteStripWidget(QtWidgets.QWidget):
        stops_changed = QtCore.Signal(object)
        selection_changed = QtCore.Signal(int)

        def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
            super().__init__(parent)
            self.setMinimumSize(420, 120)
            self._stops: list[tuple[float, tuple[int, int, int, int]]] = [(0.0, (255, 255, 255, 255)), (1.0, (0, 0, 0, 255))]
            self._selected_index = 0
            self._dragging = False

        def set_stops(self, stops: list[tuple[float, tuple[int, int, int, int]]]) -> None:
            normalized = normalize_palette_stops_even(stops or [(0.0, (255, 255, 255, 255)), (1.0, (0, 0, 0, 255))])
            self._stops = normalized or [(0.0, (255, 255, 255, 255))]
            self._selected_index = min(self._selected_index, len(self._stops) - 1)
            self.selection_changed.emit(self._selected_index)
            self.update()

        def stops(self) -> list[tuple[float, tuple[int, int, int, int]]]:
            return list(self._stops)

        def selected_index(self) -> int:
            return self._selected_index

        def set_selected_stop_color(self, rgba: tuple[int, int, int, int]) -> None:
            if 0 <= self._selected_index < len(self._stops):
                t, _ = self._stops[self._selected_index]
                self._stops[self._selected_index] = (t, rgba)
                self.stops_changed.emit(list(self._stops))
                self.update()

        def _bar_rect(self) -> QtCore.QRectF:
            return QtCore.QRectF(20.0, 16.0, max(180.0, self.width() - 40.0), 44.0)

        def _marker_rect(self, stop_t: float) -> QtCore.QRectF:
            bar = self._bar_rect()
            x = bar.left() + stop_t * bar.width()
            return QtCore.QRectF(x - 7.0, bar.bottom() + 8.0, 14.0, 18.0)

        def _hit_stop_index(self, pos: QtCore.QPointF) -> int:
            for index, (stop_t, _) in enumerate(self._stops):
                if self._marker_rect(stop_t).adjusted(-4.0, -4.0, 4.0, 4.0).contains(pos):
                    return index
            return -1

        def paintEvent(self, event) -> None:  # type: ignore[override]
            del event
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            painter.fillRect(self.rect(), QtGui.QColor("#0f1620"))
            bar = self._bar_rect()
            gradient = QtGui.QLinearGradient(bar.topLeft(), bar.topRight())
            for stop_t, rgba in self._stops:
                gradient.setColorAt(max(0.0, min(1.0, stop_t)), QtGui.QColor(*rgba))
            painter.fillRect(bar, gradient)
            painter.setPen(QtGui.QPen(QtGui.QColor("#263241"), 1.0))
            painter.drawRect(bar)
            for index, (stop_t, rgba) in enumerate(self._stops):
                marker = self._marker_rect(stop_t)
                painter.setBrush(QtGui.QColor(*rgba))
                painter.setPen(QtGui.QPen(QtGui.QColor("#ffffff" if index == self._selected_index else "#0b0f14"), 2.0 if index == self._selected_index else 1.0))
                painter.drawRoundedRect(marker, 3.0, 3.0)
            painter.setPen(QtGui.QColor("#8fa4ba"))
            painter.drawText(QtCore.QRectF(bar.left() - 6.0, bar.bottom() + 32.0, 20.0, 16.0), int(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter), "0")
            painter.drawText(QtCore.QRectF(bar.right() - 12.0, bar.bottom() + 32.0, 20.0, 16.0), int(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter), "1")
            painter.end()

        def mousePressEvent(self, event) -> None:  # type: ignore[override]
            if event.button() == QtCore.Qt.LeftButton:
                hit_index = self._hit_stop_index(event.position())
                if hit_index >= 0:
                    self._selected_index = hit_index
                    self._dragging = True
                    self.selection_changed.emit(self._selected_index)
                    self.update()
                    event.accept()
                    return
                bar = self._bar_rect()
                if bar.adjusted(-2.0, -2.0, 2.0, 2.0).contains(event.position()):
                    stop_t = max(0.0, min(1.0, (event.position().x() - bar.left()) / max(1.0, bar.width())))
                    rgba = _sample_palette_color(self._stops, stop_t)
                    self._stops.append((stop_t, rgba))
                    self._stops.sort(key=lambda item: item[0])
                    self._stops = normalize_palette_stops_even(self._stops)
                    self._selected_index = min(range(len(self._stops)), key=lambda idx: abs(self._stops[idx][0] - stop_t))
                    self._dragging = True
                    self.selection_changed.emit(self._selected_index)
                    self.stops_changed.emit(list(self._stops))
                    self.update()
                    event.accept()
                    return
            if event.button() == QtCore.Qt.RightButton:
                hit_index = self._hit_stop_index(event.position())
                if hit_index >= 0 and len(self._stops) > 1:
                    del self._stops[hit_index]
                    self._selected_index = min(hit_index, len(self._stops) - 1)
                    self.selection_changed.emit(self._selected_index)
                    self.stops_changed.emit(list(self._stops))
                    self.update()
                    event.accept()
                    return
            super().mousePressEvent(event)

        def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
            if self._dragging and 0 <= self._selected_index < len(self._stops):
                bar = self._bar_rect()
                stop_t = max(0.0, min(1.0, (event.position().x() - bar.left()) / max(1.0, bar.width())))
                _, rgba = self._stops[self._selected_index]
                self._stops[self._selected_index] = (stop_t, rgba)
                current_t = stop_t
                self._stops.sort(key=lambda item: item[0])
                self._stops = normalize_palette_stops_even(self._stops)
                self._selected_index = min(range(len(self._stops)), key=lambda idx: abs(self._stops[idx][0] - current_t))
                self.selection_changed.emit(self._selected_index)
                self.stops_changed.emit(list(self._stops))
                self.update()
                event.accept()
                return
            super().mouseMoveEvent(event)

        def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
            self._dragging = False
            super().mouseReleaseEvent(event)


    class CurveEditorDialog(QtWidgets.QDialog):
        def __init__(
            self,
            curve_text: str,
            specs: list[CurveParameterSpec],
            parent: QtWidgets.QWidget | None = None,
        ) -> None:
            super().__init__(parent)
            self.setWindowTitle("Curve Editor")
            self.resize(560, 420)
            self._specs = {spec.key: spec for spec in specs}
            self._curves = parse_curve_text(curve_text)

            self.param_combo = QtWidgets.QComboBox()
            keys = list(self._specs.keys())
            for key in self._curves.keys():
                if key not in self._specs:
                    self._specs[key] = CurveParameterSpec(key, key, 0.0, 1.0)
                    keys.append(key)
            for key in keys:
                self.param_combo.addItem(self._specs[key].label, key)
            self.delete_button = QtWidgets.QPushButton("Clear Parameter")
            self.graph = CurveGraphWidget()
            self.hint_label = QtWidgets.QLabel("Click to add points, drag to move, right-click a point to remove.")
            self.value_range_label = QtWidgets.QLabel("")

            header = QtWidgets.QHBoxLayout()
            header.addWidget(QtWidgets.QLabel("Parameter"))
            header.addWidget(self.param_combo, 1)
            header.addWidget(self.delete_button)

            buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)

            layout = QtWidgets.QVBoxLayout(self)
            layout.addLayout(header)
            layout.addWidget(self.value_range_label)
            layout.addWidget(self.graph, 1)
            layout.addWidget(self.hint_label)
            layout.addWidget(buttons)

            self.param_combo.currentIndexChanged.connect(self._load_selected_curve)
            self.delete_button.clicked.connect(self._clear_selected_curve)
            self.graph.points_changed.connect(self._on_points_changed)
            buttons.accepted.connect(self.accept)
            buttons.rejected.connect(self.reject)

            self._load_selected_curve()

        def _selected_key(self) -> str:
            return str(self.param_combo.currentData() or "")

        def _load_selected_curve(self) -> None:
            key = self._selected_key()
            spec = self._specs.get(key, CurveParameterSpec(key, key, 0.0, 1.0))
            self.graph.set_value_range(spec.min_value, spec.max_value)
            self.graph.set_points(list(self._curves.get(key, [])))
            self.value_range_label.setText(f"Range: {spec.min_value:g} .. {spec.max_value:g}")

        def _on_points_changed(self, points: object) -> None:
            key = self._selected_key()
            value = list(points) if isinstance(points, list) else []
            if value:
                self._curves[key] = value
            else:
                self._curves.pop(key, None)

        def _clear_selected_curve(self) -> None:
            key = self._selected_key()
            self._curves.pop(key, None)
            self.graph.set_points([])

        def result_text(self) -> str:
            key = self._selected_key()
            self._curves[key] = self.graph.points()
            return encode_curve_text(self._curves)


    class PaletteEditorDialog(QtWidgets.QDialog):
        def __init__(self, palette_text: str, parent: QtWidgets.QWidget | None = None) -> None:
            super().__init__(parent)
            self.setWindowTitle("Palette Editor")
            self.resize(560, 260)

            self.strip = PaletteStripWidget()
            self.strip.set_stops(parse_palette_text(palette_text) or [(0.0, (255, 255, 255, 255)), (1.0, (0, 0, 0, 255))])
            self.color_button = QtWidgets.QPushButton("Pick Color...")
            self.delete_button = QtWidgets.QPushButton("Delete Stop")
            self.position_spin = QtWidgets.QDoubleSpinBox()
            self.position_spin.setRange(0.0, 1.0)
            self.position_spin.setSingleStep(0.01)
            self.position_spin.setDecimals(3)
            self.position_spin.setPrefix("t=")
            self.position_spin.setEnabled(False)
            self.position_spin.setVisible(False)
            self.hint_label = QtWidgets.QLabel("Click on the bar to add colors, drag stops to reorder them, right-click a stop to remove.")

            controls = QtWidgets.QHBoxLayout()
            controls.addWidget(self.position_spin)
            controls.addWidget(self.color_button)
            controls.addWidget(self.delete_button)
            controls.addStretch(1)

            buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)

            layout = QtWidgets.QVBoxLayout(self)
            layout.addWidget(self.strip)
            layout.addLayout(controls)
            layout.addWidget(self.hint_label)
            layout.addWidget(buttons)

            self.strip.selection_changed.connect(self._sync_controls_from_selection)
            self.strip.stops_changed.connect(self._sync_controls_from_selection)
            self.position_spin.valueChanged.connect(self._on_position_changed)
            self.color_button.clicked.connect(self._pick_color)
            self.delete_button.clicked.connect(self._delete_selected_stop)
            buttons.accepted.connect(self.accept)
            buttons.rejected.connect(self.reject)

            self._sync_controls_from_selection()

        def _selected_index(self) -> int:
            return self.strip.selected_index()

        def _sync_controls_from_selection(self, *args) -> None:
            del args
            stops = self.strip.stops()
            index = self._selected_index()
            if not (0 <= index < len(stops)):
                return
            self.position_spin.blockSignals(True)
            self.position_spin.setValue(float(stops[index][0]))
            self.position_spin.blockSignals(False)

        def _on_position_changed(self, value: float) -> None:
            del value

        def _pick_color(self) -> None:
            stops = self.strip.stops()
            index = self._selected_index()
            if not (0 <= index < len(stops)):
                return
            _, rgba = stops[index]
            color = QtWidgets.QColorDialog.getColor(QtGui.QColor(*rgba), self, "Select Palette Stop Color", QtWidgets.QColorDialog.ShowAlphaChannel)
            if not color.isValid():
                return
            self.strip.set_selected_stop_color((color.red(), color.green(), color.blue(), color.alpha()))

        def _delete_selected_stop(self) -> None:
            stops = self.strip.stops()
            index = self._selected_index()
            if not (0 <= index < len(stops)) or len(stops) <= 1:
                return
            del stops[index]
            self.strip.set_stops(stops)

        def result_text(self) -> str:
            return encode_palette_text(self.strip.stops())
