"""Qt-based main window embedding the Matplotlib viewport."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

try:  # pragma: no cover - import depends on available Qt binding
    from PySide6 import QtCore, QtWidgets
except ImportError:  # pragma: no cover - fallback when PySide6 unavailable
    from PyQt5 import QtCore, QtWidgets  # type: ignore[no-redef]

try:  # pragma: no cover - Matplotlib backend module varies
    from matplotlib.backends.backend_qtagg import (
        FigureCanvasQTAgg as FigureCanvas,
        NavigationToolbar2QT,
    )
except ImportError:  # pragma: no cover - fallback for older Matplotlib releases
    from matplotlib.backends.backend_qt5agg import (  # type: ignore[no-redef]
        FigureCanvasQTAgg as FigureCanvas,
        NavigationToolbar2QT,
    )

from ..interactive import InteractiveArm, Waypoint, _DEFAULT_VERTICAL_JOINTS
from .keyframe_panel import KeyframePanel


@dataclass
class ServoRowWidgets:
    """Widgets associated with a single servo entry."""

    commanded_label: QtWidgets.QLabel
    actual_label: QtWidgets.QLabel
    invert_button: QtWidgets.QPushButton
    min_spin: QtWidgets.QDoubleSpinBox
    max_spin: QtWidgets.QDoubleSpinBox
    info_label: QtWidgets.QLabel


class _QtInteractiveArmBackend(InteractiveArm):
    """InteractiveArm variant that forwards UI updates to a Qt window."""

    def __init__(self, ui: "QtInteractiveArm", *args, **kwargs) -> None:
        self._ui = ui
        super().__init__(*args, build_matplotlib_controls=False, **kwargs)

    def _create_controls(self) -> None:  # pragma: no cover - intentionally unused
        # Skip Matplotlib widget creation; Qt owns the controls instead.
        return

    def _update_servo_readouts(self) -> None:
        self._ui.schedule_servo_update()

    def _update_inversion_button_visual(self, index: int) -> None:
        self._ui.schedule_inversion_update(index)

    def _update_limit_box_display(self, index: int) -> None:
        self._ui.schedule_limit_update(index)

    def _update_calibration_button_visual(self) -> None:
        self._ui.schedule_calibration_update()

    def _update_raw_angle_button_visual(self) -> None:
        self._ui.schedule_raw_angle_update()

    def _refresh_waypoint_display(self) -> None:
        self._ui.schedule_waypoint_refresh()

    def _update_waypoint_duration_box(self) -> None:
        self._ui.schedule_waypoint_duration_update()

    def _update_play_button_label(self, *, running: bool) -> None:
        self._ui.schedule_play_button_update(running=running)

    def _highlight_selected_waypoint(self) -> None:
        self._ui.schedule_waypoint_selection_update()


class QtInteractiveArm(QtWidgets.QMainWindow):
    """Qt main window embedding the InteractiveArm Matplotlib viewport."""

    def __init__(self, controller, **kwargs) -> None:
        super().__init__()
        self.setWindowTitle("Lynxmotion AL5A Controller")
        self.resize(1280, 720)

        self._updating_limits = False
        self._updating_calibration_controls = False
        self._updating_raw_checkbox = False
        self._updating_waypoint_list = False
        self._updating_waypoint_duration = False

        central_widget = QtWidgets.QWidget(self)
        central_layout = QtWidgets.QVBoxLayout(central_widget)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)

        self.backend = _QtInteractiveArmBackend(self, controller, **kwargs)

        self.canvas = FigureCanvas(self.backend.figure)
        toolbar = NavigationToolbar2QT(self.canvas, self)
        central_layout.addWidget(toolbar)
        central_layout.addWidget(self.canvas)
        self.setCentralWidget(central_widget)

        self._view_menu = self.menuBar().addMenu("&View")
        self._view_toolbar = QtWidgets.QToolBar("View", self)
        self._view_toolbar.setObjectName("viewToolbar")
        self.addToolBar(self._view_toolbar)

        self._servo_rows: list[ServoRowWidgets] = []
        self._build_servo_dock()
        self._build_calibration_dock()
        self._build_waypoint_dock()

        # Populate UI with the backend state.
        self.schedule_servo_update()
        for index in range(len(self.backend._SERVO_METADATA)):
            self.schedule_inversion_update(index)
            self.schedule_limit_update(index)
        self.schedule_calibration_update()
        self.schedule_raw_angle_update()
        self.schedule_waypoint_refresh()
        self.schedule_waypoint_duration_update()
        self.schedule_play_button_update(running=False)

    # ------------------------------------------------------------------
    # Qt helper utilities
    # ------------------------------------------------------------------
    def _invoke_in_main_thread(self, callback: Callable[[], None]) -> None:
        if QtCore.QThread.currentThread() is self.thread():
            callback()
        else:  # pragma: no cover - invoked when backend threads emit updates
            QtCore.QTimer.singleShot(0, callback)

    # ------------------------------------------------------------------
    # Servo dock construction and updates
    # ------------------------------------------------------------------
    def _build_servo_dock(self) -> None:
        dock = QtWidgets.QDockWidget("Servo controls", self)
        dock.setObjectName("servoControlsDock")
        dock.setAllowedAreas(
            QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea
        )

        container = QtWidgets.QWidget(dock)
        layout = QtWidgets.QGridLayout(container)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(6)

        header_font = container.font()
        header_font.setBold(True)

        headers = [
            ("Servo", 0),
            ("Commanded", 1),
            ("Actual", 2),
            ("Adjust", 3),
            ("Invert", 5),
            ("Min (°)", 6),
            ("Max (°)", 7),
        ]
        for text, column in headers:
            label = QtWidgets.QLabel(text)
            label.setFont(header_font)
            layout.addWidget(label, 0, column)

        for index, (name, model, location) in enumerate(self.backend._SERVO_METADATA):
            row = index + 1
            info_label = QtWidgets.QLabel(f"{name}\n{model}\n{location}")
            info_label.setWordWrap(True)
            layout.addWidget(info_label, row, 0)

            commanded_label = QtWidgets.QLabel("Cmd: 0.0°")
            layout.addWidget(commanded_label, row, 1)

            actual_label = QtWidgets.QLabel("Actual: 0.0°")
            layout.addWidget(actual_label, row, 2)

            minus_button = QtWidgets.QPushButton("−")
            minus_button.setAutoDefault(False)
            minus_button.clicked.connect(
                lambda _=False, idx=index: self.backend._adjust_servo(
                    idx, -math.radians(5)
                )
            )
            layout.addWidget(minus_button, row, 3)

            plus_button = QtWidgets.QPushButton("+")
            plus_button.setAutoDefault(False)
            plus_button.clicked.connect(
                lambda _=False, idx=index: self.backend._adjust_servo(
                    idx, math.radians(5)
                )
            )
            layout.addWidget(plus_button, row, 4)

            invert_button = QtWidgets.QPushButton("Inv")
            invert_button.setCheckable(True)
            invert_button.clicked.connect(
                lambda checked=False, idx=index: self.backend._toggle_servo_inversion(idx)
            )
            layout.addWidget(invert_button, row, 5)

            min_spin = QtWidgets.QDoubleSpinBox()
            min_spin.setRange(-360.0, 360.0)
            min_spin.setDecimals(1)
            min_spin.setSingleStep(1.0)
            min_spin.editingFinished.connect(
                lambda idx=index, spin=min_spin: self._on_limit_spin_finished(
                    idx, "min", spin
                )
            )
            layout.addWidget(min_spin, row, 6)

            max_spin = QtWidgets.QDoubleSpinBox()
            max_spin.setRange(-360.0, 360.0)
            max_spin.setDecimals(1)
            max_spin.setSingleStep(1.0)
            max_spin.editingFinished.connect(
                lambda idx=index, spin=max_spin: self._on_limit_spin_finished(
                    idx, "max", spin
                )
            )
            layout.addWidget(max_spin, row, 7)

            self._servo_rows.append(
                ServoRowWidgets(
                    commanded_label=commanded_label,
                    actual_label=actual_label,
                    invert_button=invert_button,
                    min_spin=min_spin,
                    max_spin=max_spin,
                    info_label=info_label,
                )
            )

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

        container.setLayout(layout)

        scroll = QtWidgets.QScrollArea(dock)
        scroll.setWidgetResizable(True)
        scroll.setWidget(container)
        dock.setWidget(scroll)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

    def _on_limit_spin_finished(
        self, index: int, bound: str, spin_box: QtWidgets.QDoubleSpinBox
    ) -> None:
        if self._updating_limits:
            return
        value = spin_box.value()
        radians_value = math.radians(value)
        if bound == "min":
            self.backend._update_servo_limit(index, min_angle=radians_value)
        else:
            self.backend._update_servo_limit(index, max_angle=radians_value)

    def schedule_servo_update(self) -> None:
        def update() -> None:
            for idx, row in enumerate(self._servo_rows):
                if idx >= len(self.backend.commanded_joints):
                    continue
                zero_angle = self.backend.zero_reference.get(
                    idx,
                    _DEFAULT_VERTICAL_JOINTS[idx]
                    if idx < len(_DEFAULT_VERTICAL_JOINTS)
                    else 0.0,
                )
                zero_deg = math.degrees(zero_angle)
                commanded_raw = math.degrees(self.backend.commanded_joints[idx])
                actual_source = (
                    self.backend.feedback_joints
                    if self.backend.feedback_joints
                    and idx < len(self.backend.feedback_joints)
                    else self.backend.current_joints
                )
                actual_raw = math.degrees(actual_source[idx])
                commanded_deg = commanded_raw - zero_deg
                actual_deg = actual_raw - zero_deg
                raw_suffix_cmd = (
                    f" (raw {commanded_raw:.1f}°)" if self.backend._show_raw_angles else ""
                )
                raw_suffix_actual = (
                    f" (raw {actual_raw:.1f}°)" if self.backend._show_raw_angles else ""
                )
                row.commanded_label.setText(f"Cmd: {commanded_deg:.1f}°{raw_suffix_cmd}")
                row.actual_label.setText(f"Actual: {actual_deg:.1f}°{raw_suffix_actual}")

        self._invoke_in_main_thread(update)

    def schedule_inversion_update(self, index: int) -> None:
        def update() -> None:
            if index >= len(self._servo_rows):
                return
            inverted = self.backend.servo_inversions[index]
            button = self._servo_rows[index].invert_button
            button.blockSignals(True)
            button.setChecked(inverted)
            button.setText("Inv✓" if inverted else "Inv")
            if inverted:
                button.setStyleSheet("background-color: #90ee90;")
            else:
                button.setStyleSheet("")
            button.blockSignals(False)

        self._invoke_in_main_thread(update)

    def schedule_limit_update(self, index: int) -> None:
        def update() -> None:
            if index >= len(self._servo_rows):
                return
            base_config = self.backend._base_servo_configs.get(index)
            if base_config is None:
                return
            row = self._servo_rows[index]
            self._updating_limits = True
            try:
                row.min_spin.blockSignals(True)
                row.max_spin.blockSignals(True)
                row.min_spin.setValue(math.degrees(base_config.min_angle))
                row.max_spin.setValue(math.degrees(base_config.max_angle))
            finally:
                row.min_spin.blockSignals(False)
                row.max_spin.blockSignals(False)
                self._updating_limits = False

        self._invoke_in_main_thread(update)

    # ------------------------------------------------------------------
    # Calibration dock
    # ------------------------------------------------------------------
    def _build_calibration_dock(self) -> None:
        dock = QtWidgets.QDockWidget("Calibration", self)
        dock.setObjectName("calibrationDock")
        dock.setAllowedAreas(
            QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea
        )

        widget = QtWidgets.QWidget(dock)
        layout = QtWidgets.QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        self._calibrate_button = QtWidgets.QPushButton("Calibrate")
        self._calibrate_button.setCheckable(True)
        self._calibrate_button.clicked.connect(self._on_calibrate_clicked)
        layout.addWidget(self._calibrate_button)

        self._set_vertical_button = QtWidgets.QPushButton("Set vertical")
        self._set_vertical_button.clicked.connect(self.backend._handle_set_vertical)
        layout.addWidget(self._set_vertical_button)

        self._raw_angle_checkbox = QtWidgets.QCheckBox("Show raw servo angles")
        self._raw_angle_checkbox.toggled.connect(self._on_raw_checkbox_toggled)
        layout.addWidget(self._raw_angle_checkbox)

        layout.addStretch()

        widget.setLayout(layout)
        dock.setWidget(widget)
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, dock)

    def _on_calibrate_clicked(self) -> None:
        if self._updating_calibration_controls:
            return
        self.backend._toggle_calibration()

    def _on_raw_checkbox_toggled(self, checked: bool) -> None:
        if self._updating_raw_checkbox:
            return
        self.backend._show_raw_angles = bool(checked)
        self.backend._update_servo_readouts()

    def schedule_calibration_update(self) -> None:
        def update() -> None:
            active = self.backend._calibration_active
            self._updating_calibration_controls = True
            try:
                self._calibrate_button.setChecked(active)
                self._calibrate_button.setText(
                    "Calibrating" if active else "Calibrate"
                )
                self._set_vertical_button.setEnabled(active)
            finally:
                self._updating_calibration_controls = False

        self._invoke_in_main_thread(update)

    def schedule_raw_angle_update(self) -> None:
        def update() -> None:
            self._updating_raw_checkbox = True
            try:
                self._raw_angle_checkbox.setChecked(self.backend._show_raw_angles)
            finally:
                self._updating_raw_checkbox = False

        self._invoke_in_main_thread(update)

    # ------------------------------------------------------------------
    # Waypoint dock and updates
    # ------------------------------------------------------------------
    def _build_waypoint_dock(self) -> None:
        dock = QtWidgets.QDockWidget("Waypoints", self)
        dock.setObjectName("waypointDock")
        dock.setAllowedAreas(
            QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea
        )

        self._keyframe_panel = KeyframePanel(dock)
        self._keyframe_panel.addWaypointRequested.connect(self._on_add_waypoint)
        self._keyframe_panel.playRequested.connect(
            self.backend._handle_play_waypoints
        )
        self._keyframe_panel.clearRequested.connect(
            self.backend._handle_clear_waypoints
        )
        self._keyframe_panel.selectionChanged.connect(self._on_waypoint_selected)
        self._keyframe_panel.durationChanged.connect(
            self._on_waypoint_duration_changed
        )
        dock.setWidget(self._keyframe_panel)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

        toggle_action = dock.toggleViewAction()
        toggle_action.setText("Waypoints panel")
        toggle_action.setStatusTip("Show or hide the waypoint manager")
        self._view_menu.addAction(toggle_action)
        self._view_toolbar.addAction(toggle_action)

    def _on_add_waypoint(self, duration: float) -> None:
        duration = max(0.1, duration)
        position = self.backend._clamp_target(np.array(self.backend.target))
        self.backend.waypoints.append(
            Waypoint(position=position.copy(), duration=duration)
        )
        self.backend._selected_waypoint_index = len(self.backend.waypoints) - 1
        self.backend._refresh_waypoint_display()
        self.backend._update_waypoint_duration_box()

    def _on_waypoint_selected(self, row: int) -> None:
        if self._updating_waypoint_list:
            return
        if row < 0 or row >= len(self.backend.waypoints):
            self.backend._selected_waypoint_index = None
        else:
            self.backend._selected_waypoint_index = row
        self.backend._update_waypoint_duration_box()

    def _on_waypoint_duration_changed(self, value: float) -> None:
        if self._updating_waypoint_duration:
            return
        index = self.backend._selected_waypoint_index
        if index is None or index >= len(self.backend.waypoints):
            return
        value = max(0.1, value)
        self.backend.waypoints[index].duration = value
        self.backend._refresh_waypoint_display()

    def schedule_waypoint_refresh(self) -> None:
        def update() -> None:
            self._updating_waypoint_list = True
            try:
                self._keyframe_panel.set_waypoints(
                    self.backend.waypoints,
                    selected_index=self.backend._selected_waypoint_index,
                )
            finally:
                self._updating_waypoint_list = False

        self._invoke_in_main_thread(update)

    def schedule_waypoint_duration_update(self) -> None:
        def update() -> None:
            self._updating_waypoint_duration = True
            try:
                index = self.backend._selected_waypoint_index
                duration = (
                    None
                    if index is None or index >= len(self.backend.waypoints)
                    else self.backend.waypoints[index].duration
                )
                self._keyframe_panel.set_duration(duration)
            finally:
                self._updating_waypoint_duration = False

        self._invoke_in_main_thread(update)

    def schedule_play_button_update(self, *, running: bool) -> None:
        def update() -> None:
            self._keyframe_panel.set_playing(running)

        self._invoke_in_main_thread(update)

    def schedule_waypoint_selection_update(self) -> None:
        def update() -> None:
            index = self.backend._selected_waypoint_index
            self._updating_waypoint_list = True
            try:
                self._keyframe_panel.set_selected_index(index)
            finally:
                self._updating_waypoint_list = False

        self._invoke_in_main_thread(update)


def create_qt_application(args: list[str] | None = None) -> QtWidgets.QApplication:
    """Return an existing QApplication or create a new one."""

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(args or [])
    return app
