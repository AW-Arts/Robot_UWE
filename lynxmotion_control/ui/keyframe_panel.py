"""Reusable Qt widget providing waypoint management controls."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

try:  # pragma: no cover - import depends on available Qt binding
    from PySide6 import QtCore, QtWidgets
except ImportError:  # pragma: no cover - fallback when PySide6 unavailable
    from PyQt5 import QtCore, QtWidgets  # type: ignore[no-redef]

from ..interactive import Waypoint


@dataclass(frozen=True)
class _WaypointDisplayData:
    """Light-weight container used when rendering waypoint rows."""

    index: int
    position: Sequence[float]
    duration: float


class KeyframePanel(QtWidgets.QWidget):
    """Timeline, duration control, and transport buttons for waypoints."""

    addWaypointRequested = QtCore.Signal(float)
    playRequested = QtCore.Signal()
    clearRequested = QtCore.Signal()
    selectionChanged = QtCore.Signal(int)
    durationChanged = QtCore.Signal(float)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._updating_table = False
        self._updating_selection = False
        self._updating_duration = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)

        self._timeline = QtWidgets.QTableWidget(0, 5, self)
        self._timeline.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._timeline.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self._timeline.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._timeline.setAlternatingRowColors(True)
        self._timeline.verticalHeader().setVisible(False)
        self._timeline.horizontalHeader().setStretchLastSection(True)
        self._timeline.setHorizontalHeaderLabels(
            ["#", "X (m)", "Y (m)", "Z (m)", "Duration (s)"]
        )
        self._timeline.selectionModel().currentRowChanged.connect(
            self._on_current_row_changed
        )
        layout.addWidget(self._timeline)

        self._empty_label = QtWidgets.QLabel("No waypoints")
        self._empty_label.setAlignment(QtCore.Qt.AlignCenter)
        self._empty_label.setStyleSheet("color: palette(mid);")
        layout.addWidget(self._empty_label)

        duration_layout = QtWidgets.QHBoxLayout()
        duration_label = QtWidgets.QLabel("Duration (s):")
        duration_layout.addWidget(duration_label)

        self._duration_spin = QtWidgets.QDoubleSpinBox()
        self._duration_spin.setRange(0.1, 60.0)
        self._duration_spin.setDecimals(2)
        self._duration_spin.setSingleStep(0.1)
        self._duration_spin.valueChanged.connect(self._on_duration_changed)
        duration_layout.addWidget(self._duration_spin, 1)
        layout.addLayout(duration_layout)

        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addStretch(1)

        self._add_button = QtWidgets.QPushButton("Add")
        self._add_button.clicked.connect(self._emit_add_request)
        button_layout.addWidget(self._add_button)

        self._play_button = QtWidgets.QPushButton("Play path")
        self._play_button.clicked.connect(self.playRequested)
        button_layout.addWidget(self._play_button)

        self._clear_button = QtWidgets.QPushButton("Clear")
        self._clear_button.clicked.connect(self.clearRequested)
        button_layout.addWidget(self._clear_button)

        layout.addLayout(button_layout)

        self._update_empty_state(has_waypoints=False)

    # ------------------------------------------------------------------
    # Public API used by the QtInteractiveArm window
    # ------------------------------------------------------------------
    def set_waypoints(
        self, waypoints: Iterable[Waypoint], *, selected_index: int | None
    ) -> None:
        """Populate the timeline with the provided waypoint collection."""

        data = [
            _WaypointDisplayData(index=idx, position=wp.position, duration=wp.duration)
            for idx, wp in enumerate(waypoints)
        ]

        self._updating_table = True
        self._timeline.setRowCount(len(data))
        for row, entry in enumerate(data):
            row_values = [
                f"{entry.index + 1}",
                f"{entry.position[0]:.3f}",
                f"{entry.position[1]:.3f}",
                f"{entry.position[2]:.3f}",
                f"{entry.duration:.2f}",
            ]
            for column, text in enumerate(row_values):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                if column == 0:
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                else:
                    item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                self._timeline.setItem(row, column, item)
        self._updating_table = False

        self._update_empty_state(has_waypoints=bool(data))
        self.set_selected_index(selected_index)

    def set_selected_index(self, index: int | None) -> None:
        """Synchronise the selection in the timeline with the backend state."""

        self._updating_selection = True
        if index is None or index < 0 or index >= self._timeline.rowCount():
            self._timeline.clearSelection()
            self._timeline.setCurrentItem(None)
        else:
            self._timeline.selectRow(index)
            self._timeline.setCurrentCell(index, 0)
        self._updating_selection = False

    def set_duration(self, value: float | None) -> None:
        """Update the duration spinbox to match the active waypoint."""

        self._updating_duration = True
        if value is None:
            default = max(
                self._duration_spin.minimum(),
                min(self._duration_spin.maximum(), 2.0),
            )
            self._duration_spin.setValue(default)
            self._duration_spin.setEnabled(False)
        else:
            self._duration_spin.setEnabled(True)
            self._duration_spin.setValue(value)
        self._updating_duration = False

    def set_playing(self, running: bool) -> None:
        """Update the play button label to reflect playback state."""

        label = "Stop" if running else "Play path"
        self._play_button.setText(label)

    def current_duration(self) -> float:
        """Return the currently selected duration value."""

        return self._duration_spin.value()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _emit_add_request(self) -> None:
        self.addWaypointRequested.emit(max(0.1, self.current_duration()))

    def _on_duration_changed(self, value: float) -> None:
        if self._updating_duration:
            return
        self.durationChanged.emit(max(0.1, value))

    def _on_current_row_changed(
        self, current: QtCore.QModelIndex, _previous: QtCore.QModelIndex
    ) -> None:
        if self._updating_selection:
            return
        if current.isValid():
            self.selectionChanged.emit(current.row())
        else:
            self.selectionChanged.emit(-1)

    def _update_empty_state(self, *, has_waypoints: bool) -> None:
        self._timeline.setVisible(has_waypoints)
        self._empty_label.setVisible(not has_waypoints)
        self._clear_button.setEnabled(has_waypoints)
        self._play_button.setEnabled(has_waypoints)
        if not has_waypoints:
            self.set_duration(None)

