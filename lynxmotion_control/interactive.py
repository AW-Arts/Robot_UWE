"""Interactive matplotlib UI for commanding the Lynxmotion AL5A."""
from __future__ import annotations

import json
import logging
import math
import queue
import threading
import time
from copy import deepcopy
from typing import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from matplotlib.widgets import Button, Slider, TextBox
from mpl_toolkits.mplot3d import proj3d

from .al5a_kinematics import (
    AL5AKinematics,
    DEFAULT_SERVO_CHANNELS,
    DEFAULT_SERVO_CONFIGS,
    ServoConfig,
)


_LOGGER = logging.getLogger(__name__)


CALIBRATION_CONFIG_PATH = Path.home() / ".config" / "lynxmotion_al5a" / "servo_offsets.json"
PATH_STORAGE_PATH = CALIBRATION_CONFIG_PATH.with_name("saved_path.json")
SUBROUTINE_STORAGE_DIR = PATH_STORAGE_PATH.with_name("subroutines")
TIMELINE_STORAGE_PATH = PATH_STORAGE_PATH.with_name("timeline.json")


_DEFAULT_VERTICAL_JOINTS = [
    0.0,
    math.pi / 2,
    0.0,
    0.0,
    0.0,
    0.0,
]


def _ensure_interactive_backend() -> None:
    """Ensure Matplotlib is using a backend that supports mouse interaction."""

    interactive_backends = {
        "gtk3agg",
        "macosx",
        "nbagg",
        "qtagg",
        "qt5agg",
        "tkagg",
        "wxagg",
    }
    backend = matplotlib.get_backend()
    backend_normalised = backend.lower()
    if backend_normalised.startswith("module://"):
        backend_normalised = backend_normalised.split("module://", 1)[1]

    if backend_normalised in interactive_backends:
        return

    for candidate in ("qtagg", "qt5agg", "tkagg", "nbagg", "gtk3agg", "wxagg"):
        try:
            matplotlib.use(candidate, force=True)
        except Exception:  # pragma: no cover - backend availability varies
            continue
        else:
            _LOGGER.info("Switched Matplotlib backend to %s for interactivity", candidate)
            return

    raise RuntimeError(
        "No interactive Matplotlib backend available. Install PyQt, Tk, or another "
        "GUI toolkit to enable dragging the target in the controller UI."
    )


_ensure_interactive_backend()

import matplotlib.pyplot as plt  # noqa: E402  (import after backend selection)


@dataclass
class DragState:
    dragging: bool = False
    last_event: object | None = None
    active_axis: str | None = None
    start_target: np.ndarray | None = None


@dataclass
class Waypoint:
    position: np.ndarray
    duration: float
    wrist_pitch: float


@dataclass
class WaypointDragState:
    index: int | None = None
    offset: float = 0.0


@dataclass
class SubroutineSummary:
    name: str
    slug: str
    path: Path
    duration: float
    waypoint_count: int


@dataclass
class TimelineEntry:
    name: str
    slug: str
    duration: float
    missing: bool = False


@dataclass
class TimelineDragState:
    index: int | None = None
    offset: float = 0.0


class InteractiveArm:
    """Matplotlib based interactive controller."""

    _SERVO_METADATA = [
        ("Base rotation", "HS-755HB", "lower servo, inside the base"),
        ("Shoulder pitch", "HS-645MG", "mounted between ASB-06 and ASB-10"),
        ("Elbow pitch", "HS-422", "mid-arm servo"),
        ("Wrist pitch", "HS-422", "near wrist"),
        ("Wrist rotation", "HS-85BB", "wrist roll servo"),
        ("Gripper", "HS-422/HS-225MG", "gripper open/close"),
    ]

    DEFAULT_HOME_PULSES: dict[int, int] = {
        0: 1500,
        1: 1580,
        2: 950,
        3: 1400,
        4: 500,
        5: 1500,
    }

    def __init__(
        self,
        controller,
        kinematics: AL5AKinematics | None = None,
        wrist_pitch: float = 0.0,
        move_time_ms: int = 1000,
        step_xy: float = 0.01,
        step_z: float = 0.01,
        *,
        build_matplotlib_controls: bool = True,
    ) -> None:
        self.controller = controller
        self.kin = kinematics or AL5AKinematics()
        self._wrist_pitch_limits = (
            math.radians(-120.0),
            math.radians(120.0),
        )
        self._wrist_slider_limits = (-90.0, 90.0)
        self.wrist_pitch = float(np.clip(wrist_pitch, *self._wrist_pitch_limits))
        self._last_wrist_pitch = self.wrist_pitch
        self.move_time_ms = move_time_ms
        self.step_xy = step_xy
        self.step_z = step_z
        self._base_servo_configs = deepcopy(DEFAULT_SERVO_CONFIGS)
        self.servo_configs: dict[int, ServoConfig] = dict(self._base_servo_configs)
        self._soft_servo_limits: dict[int, tuple[float, float]] = {
            idx: (config.min_angle, config.max_angle)
            for idx, config in self._base_servo_configs.items()
        }
        self.servo_inversions: list[bool] = [False] * len(self._SERVO_METADATA)
        self._invert_button_inactive_color = "0.85"
        self._invert_button_active_color = "#90ee90"
        self._calibration_path = CALIBRATION_CONFIG_PATH
        self._wrist_slider: Slider | None = None
        self._updating_wrist_slider = False
        self.zero_reference: dict[int, float] = {
            index: angle for index, angle in enumerate(_DEFAULT_VERTICAL_JOINTS)
        }
        self._show_raw_angles = False
        self._raw_angle_button: Button | None = None
        self._soft_start_min_time_ms = 4_000
        self._soft_start_min_segments = 18
        self._home_move_time_ms = move_time_ms
        self._loaded_servo_limits: dict[int, tuple[float, float]] = {}
        self._loaded_workspace: dict[str, tuple[float, float]] = {}
        self.servo_offsets: dict[int, float] = self._load_calibration_data()
        self.workspace_limits = {
            "x": (-0.25, 0.25),
            "y": (-0.25, 0.25),
            "z": (
                self.kin.links.base_height + 0.02,
                self.kin.links.base_height
                + self.kin.links.shoulder
                + self.kin.links.elbow,
            ),
        }
        self._apply_loaded_servo_limits()

        for index, inverted in enumerate(self.servo_inversions):
            if inverted:
                self._apply_servo_inversion(index)

        self.current_joints = self._default_joint_configuration()
        self.commanded_joints: list[float] = list(self.current_joints)
        self.feedback_joints: list[float] | None = None

        if len(self.current_joints) >= 5:
            self.wrist_rotation = self.current_joints[4]
        else:
            self.wrist_rotation = 0.0
        if len(self.current_joints) >= 6:
            self.gripper_angle = self.current_joints[5]
        else:
            self.gripper_angle = 0.0

        if len(self.current_joints) >= 4:
            self._last_wrist_pitch = (
                self.current_joints[1]
                + self.current_joints[2]
                + self.current_joints[3]
            )
            forward_pose = self.kin.forward(self.current_joints)
            target_position = forward_pose[:3, 3]
        else:
            target_position = np.array([0.18, 0.0, 0.18])

        self.target = np.array(target_position, dtype=float)
        self._setpoint_position: np.ndarray | None = np.array(target_position, dtype=float)
        self._setpoint_joints: list[float] | None = None
        self._load_workspace_limits_from_config()
        self._last_commanded_raw: tuple[float, ...] | None = tuple(
            self._apply_offsets(self.current_joints, direction="raw")
        )
        self._initial_feedback_move_pending = False
        self._initial_feedback_move_time_ms = 10_000
        self._skip_next_command = False
        self._smoothing_step_ms = 60
        self._min_smoothing_segments = 5
        self._default_max_joint_speed = math.radians(60.0) / 0.2  # ~300°/s
        self._joint_max_speeds: dict[int, float] = {
            0: math.radians(60.0) / 0.23,  # HS-755HB
            1: math.radians(60.0) / 0.20,  # HS-645MG
            2: math.radians(60.0) / 0.16,  # HS-422
            3: math.radians(60.0) / 0.16,  # HS-422
            4: math.radians(60.0) / 0.16,  # HS-85BB
            5: math.radians(60.0) / 0.14,  # HS-422/HS-225MG
        }

        # Serial writes can block when the controller is busy. Offload them to a
        # dedicated worker so the Matplotlib event loop stays responsive.
        self._command_queue: queue.Queue[
            tuple[tuple[float, ...], int | None, bool]
        ] = queue.Queue(maxsize=32)
        self._command_thread = threading.Thread(
            target=self._command_worker, name="al5a-command-worker", daemon=True
        )
        self._command_thread.start()

        self.drag_state = DragState()
        self._modifiers: set[str] = set()
        self._drag_z_scale = 0.0008
        self.waypoints: list[Waypoint] = []
        self._waypoint_patches: list[Rectangle] = []
        self._waypoint_texts: list = []
        self._waypoint_drag = WaypointDragState()
        self._waypoint_reordered = False
        self._selected_waypoint_index: int | None = None
        self._waypoint_playback_thread: threading.Thread | None = None
        self._waypoint_stop_event = threading.Event()
        self._updating_duration_box = False
        self._subroutine_catalog: dict[str, SubroutineSummary] = {}
        self._subroutine_list_ax = None
        self._subroutine_patches: list[Rectangle] = []
        self._subroutine_texts: list = []
        self._subroutine_display_slugs: list[str] = []
        self._subroutine_item_height = 0.16
        self._subroutine_item_gap = 0.05
        self._selected_subroutine_slug: str | None = None
        self._timeline_entries: list[TimelineEntry] = []
        self._timeline_ax = None
        self._timeline_patches: list[Rectangle] = []
        self._timeline_texts: list = []
        self._timeline_item_height = 0.16
        self._timeline_item_gap = 0.05
        self._timeline_drag = TimelineDragState()
        self._timeline_reordered = False
        self._selected_timeline_index: int | None = None
        self._timeline_playback_thread: threading.Thread | None = None
        self._timeline_stop_event = threading.Event()
        self._timeline_repeat = False
        self._timeline_speed = 1.0
        self._timeline_repeat_button: Button | None = None
        self._timeline_speed_slider: Slider | None = None
        self.figure = plt.figure("Lynxmotion AL5A Controller")
        try:
            # Provide extra breathing room for the on-figure panels while keeping the
            # 3D viewport generous enough for manipulation.
            self.figure.set_size_inches(14.5, 8.5, forward=True)
        except Exception:  # pragma: no cover - backend quirks when sizing figures
            pass
        try:
            self.figure.set_tight_layout(True)
        except AttributeError:  # pragma: no cover - Matplotlib < 3.1
            self.figure.tight_layout()
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self._recompute_camera_framing()
        self.ax.view_init(elev=25, azim=-60)

        (self.base_line,) = self.ax.plot([], [], [], "-o", lw=3, label="Arm (actual)")
        (self.setpoint_line,) = self.ax.plot(
            [],
            [],
            [],
            "--o",
            lw=2,
            color="#ff9f1c",
            alpha=0.7,
            label="Arm (setpoint)",
        )
        self._gizmo_vectors = {
            "x": np.array([1.0, 0.0, 0.0]),
            "y": np.array([0.0, 1.0, 0.0]),
            "z": np.array([0.0, 0.0, 1.0]),
        }
        self._gizmo_colors = {"x": "#ff595e", "y": "#1982c4", "z": "#8ac926"}
        self.target_artist = self.ax.scatter(
            [self.target[0]],
            [self.target[1]],
            [self.target[2]],
            c="red",
            s=100,
            label="Target",
        )
        self._gizmo_length = 0.06
        self._gizmo_lines = self._create_gizmo()
        self.text = self.ax.text2D(
            0.02,
            0.95,
            "",
            transform=self.ax.transAxes,
            bbox=dict(facecolor="white", alpha=0.7),
        )
        self.figure.canvas.mpl_connect("button_press_event", self._on_press)
        self.figure.canvas.mpl_connect("button_release_event", self._on_release)
        self.figure.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.figure.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.figure.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.figure.canvas.mpl_connect("key_release_event", self._on_key_release)
        self.figure.canvas.mpl_connect("resize_event", self._on_canvas_resized)

        self._calibration_active = False
        self._calibration_button: Button | None = None
        self._set_vertical_button: Button | None = None
        self._calibration_timer = None

        # UI placeholders populated when using the Matplotlib-based controls.
        self.buttons: dict[str, Button] = {}
        self.servo_value_texts: list = []
        self.servo_buttons: list[Button] = []
        self.servo_invert_buttons: list[Button] = []
        self.servo_limit_boxes_min: list[TextBox] = []
        self.servo_limit_boxes_max: list[TextBox] = []
        self.servo_pulse_sliders: list[Slider] = []
        self._pulse_slider_limit_lines: dict[int, dict[str, Line2D]] = {}
        self._home_button: Button | None = None
        self._waypoint_duration_box: TextBox | None = None
        self._add_waypoint_button: Button | None = None
        self._play_waypoints_button: Button | None = None
        self._clear_waypoints_button: Button | None = None
        self._subroutine_name_box: TextBox | None = None
        self._save_subroutine_button: Button | None = None
        self._load_subroutine_button: Button | None = None
        self._refresh_subroutine_button: Button | None = None
        self._add_timeline_button: Button | None = None
        self._remove_timeline_button: Button | None = None
        self._clear_timeline_button: Button | None = None
        self._play_timeline_button: Button | None = None
        self.waypoint_ax = None

        self._load_saved_waypoints()
        self._load_subroutine_catalog()
        self._load_saved_timeline()

        if build_matplotlib_controls:
            self._create_controls()
        self._initialise_from_feedback()
        self._skip_next_command = True
        self.update_robot()
        self._run_home_sequence()

    def _on_canvas_resized(self, _event) -> None:
        try:
            self.figure.tight_layout()
        except Exception:  # pragma: no cover - backend specific layout errors
            pass
        self._recompute_camera_framing()
        self.figure.canvas.draw_idle()

    def _recompute_camera_framing(self) -> None:
        x_limits = self.workspace_limits.get("x", (-0.25, 0.25))
        y_limits = self.workspace_limits.get("y", (-0.25, 0.25))
        z_limits = self.workspace_limits.get("z", (0.0, 0.35))

        pad_x = (x_limits[1] - x_limits[0]) * 0.05 or 0.01
        pad_y = (y_limits[1] - y_limits[0]) * 0.05 or 0.01
        pad_z = (z_limits[1] - z_limits[0]) * 0.05 or 0.01

        self.ax.set_xlim(x_limits[0] - pad_x, x_limits[1] + pad_x)
        self.ax.set_ylim(y_limits[0] - pad_y, y_limits[1] + pad_y)
        self.ax.set_zlim(z_limits[0] - pad_z, z_limits[1] + pad_z)

        try:
            span_x = max(x_limits[1] - x_limits[0], 1e-6)
            span_y = max(y_limits[1] - y_limits[0], 1e-6)
            span_z = max(z_limits[1] - z_limits[0], 1e-6)
            self.ax.set_box_aspect((span_x, span_y, span_z))
        except AttributeError:  # Matplotlib < 3.4
            pass

    def _default_joint_configuration(self) -> list[float]:
        joints: list[float] = []
        for idx in range(len(self._SERVO_METADATA)):
            config = self.servo_configs.get(idx)
            pulse = self.DEFAULT_HOME_PULSES.get(idx)
            if config is None or pulse is None:
                joints.append(0.0)
                continue
            joints.append(config.pulse_to_angle(pulse))
        return joints

    def _get_home_joints(self) -> list[float] | None:
        if not self.zero_reference:
            return list(_DEFAULT_VERTICAL_JOINTS)

        home: list[float] = []
        for idx in range(len(self._SERVO_METADATA)):
            if idx in self.zero_reference:
                home.append(self.zero_reference[idx])
            elif idx < len(_DEFAULT_VERTICAL_JOINTS):
                home.append(_DEFAULT_VERTICAL_JOINTS[idx])
            else:
                home.append(0.0)
        return home

    def _initialise_from_feedback(self) -> None:
        read_positions = getattr(self.controller, "read_positions", None)
        if not callable(read_positions):
            return

        try:
            feedback = read_positions(
                servo_configs=self._get_servo_configs_for_controller(),
                servo_channels=DEFAULT_SERVO_CHANNELS,
            )
        except Exception:  # pragma: no cover - runtime safety net
            _LOGGER.warning(
                "Failed to obtain initial feedback from controller", exc_info=True
            )
            self._skip_next_command = True
            return

        if not feedback:
            return

        corrected = self._apply_offsets(list(feedback), direction="correct")
        for idx, angle in enumerate(corrected):
            if idx < len(self.current_joints):
                self.current_joints[idx] = angle
            else:
                self.current_joints.append(angle)

        if len(self.current_joints) >= 5:
            self.wrist_rotation = self.current_joints[4]
        if len(self.current_joints) >= 6:
            self.gripper_angle = self.current_joints[5]

        if len(self.current_joints) >= 4:
            try:
                pose = self.kin.forward(self.current_joints)
            except Exception:  # pragma: no cover - safety net
                _LOGGER.warning(
                    "Failed to compute pose from controller feedback", exc_info=True
                )
            else:
                self.target[:3] = pose[:3, 3]
                actual_pitch = (
                    self.current_joints[1]
                    + self.current_joints[2]
                    + self.current_joints[3]
                )
                self._last_wrist_pitch = actual_pitch
                self._set_wrist_pitch_target(actual_pitch)
                if len(self.current_joints) >= 4:
                    self._setpoint_joints = list(self.current_joints[:4])
                    self._setpoint_position = np.array(self.target)
                self._update_visuals(self.current_joints)

        self.feedback_joints = list(corrected)
        self._update_servo_readouts()
        self._initial_feedback_move_pending = True

    def _run_home_sequence(self) -> None:
        home = self._get_home_joints()
        if not home:
            return

        clamped_home = self._clamp_joint_list(home)
        self.commanded_joints = list(clamped_home)
        if len(clamped_home) >= 4:
            try:
                pose = self.kin.forward(clamped_home)
            except Exception:  # pragma: no cover - runtime safety net
                pose = None
            else:
                position = np.array(pose[:3, 3], dtype=float)
                self.target[:] = self._clamp_target(position)
                actual_pitch = sum(clamped_home[1:4])
                self._last_wrist_pitch = actual_pitch
                self._set_wrist_pitch_target(actual_pitch)
                self._setpoint_joints = list(clamped_home[:4])
                self._setpoint_position = np.array(self.target)
                self._update_visuals(self.current_joints)
        if len(clamped_home) >= 5:
            self.wrist_rotation = clamped_home[4]
        if len(clamped_home) >= 6:
            self.gripper_angle = clamped_home[5]

        self._update_servo_readouts()
        # Ensure the home move is not skipped even if initial feedback failed.
        self._skip_next_command = False
        self._send_move_command(
            clamped_home, move_time_ms=self._home_move_time_ms, soft_start=False
        )

        # Refresh the plot using the commanded home pose so the displayed arm
        # matches the setpoint immediately after homing.
        self._update_visuals(clamped_home)

    def update_robot(self) -> None:
        requested = self._clamp_target(self.target)
        self.target[:] = requested
        joints = self.kin.inverse(requested[[0, 1, 2]], self.wrist_pitch)
        joints = self._clamp_joint_list(list(joints))
        self._last_wrist_pitch = joints[1] + joints[2] + joints[3]
        try:
            pose = self.kin.forward(joints)
        except Exception:
            self._setpoint_position = None
        else:
            position = np.array(pose[:3, 3], dtype=float)
            self._setpoint_position = position
            self.target[:] = position
        self._setpoint_joints = list(joints)
        full_joints = joints + [self.wrist_rotation, self.gripper_angle]
        full_joints = self._clamp_joint_list(full_joints)
        self.commanded_joints = list(full_joints)
        self._send_move_command(
            full_joints, move_time_ms=self.move_time_ms, soft_start=True
        )
        self.figure.canvas.draw_idle()

        self._update_raw_angle_button_visual()

    def _slider_value_from_pitch(self, pitch: float) -> float:
        clamped = float(np.clip(pitch, *self._wrist_pitch_limits))
        slider_value = 90.0 - math.degrees(clamped)
        return float(np.clip(slider_value, *self._wrist_slider_limits))

    def _pitch_from_slider_value(self, value: float) -> float:
        clamped_value = float(np.clip(value, *self._wrist_slider_limits))
        pitch = math.radians(90.0 - clamped_value)
        return float(np.clip(pitch, *self._wrist_pitch_limits))

    def _set_wrist_pitch_target(self, pitch: float, *, update_slider: bool = True) -> None:
        clamped = float(np.clip(pitch, *self._wrist_pitch_limits))
        self.wrist_pitch = clamped
        if update_slider and self._wrist_slider is not None:
            slider_value = self._slider_value_from_pitch(clamped)
            try:
                self._updating_wrist_slider = True
                self._wrist_slider.set_val(slider_value)
            finally:
                self._updating_wrist_slider = False

    def _handle_wrist_slider_change(self, value: float) -> None:  # pragma: no cover - UI interaction
        if self._updating_wrist_slider:
            return
        desired_pitch = self._pitch_from_slider_value(value)
        self._set_wrist_pitch_target(desired_pitch, update_slider=False)
        self.update_robot()
        if (
            self._selected_waypoint_index is not None
            and 0 <= self._selected_waypoint_index < len(self.waypoints)
        ):
            self.waypoints[self._selected_waypoint_index].wrist_pitch = desired_pitch
            self._refresh_waypoint_display()
            self._save_waypoints()

    def _update_wrist_slider_display(self) -> None:
        if self._wrist_slider is None:
            return
        try:
            self._updating_wrist_slider = True
            self._wrist_slider.set_val(self._slider_value_from_pitch(self.wrist_pitch))
        finally:
            self._updating_wrist_slider = False

    def _update_raw_angle_button_visual(self) -> None:
        if self._raw_angle_button is None:
            return
        if self._show_raw_angles:
            self._raw_angle_button.color = "#add8e6"
            self._raw_angle_button.hovercolor = "#bde0fe"
            self._raw_angle_button.label.set_text("Hide raw")
        else:
            self._raw_angle_button.color = "0.85"
            self._raw_angle_button.hovercolor = "0.95"
            self._raw_angle_button.label.set_text("Show raw")
        self._raw_angle_button.ax.set_facecolor(self._raw_angle_button.color)
        self.figure.canvas.draw_idle()

    def _on_press(self, event) -> None:
        if self._handle_subroutine_press(event):
            return
        if self._handle_timeline_press(event):
            return
        if self._handle_waypoint_press(event):
            return
        if event.inaxes != self.ax:
            return
        if event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        # ``contains`` on ``scatter`` can be unreliable depending on the backend,
        # so fall back to a simple distance based check in data coordinates. This
        # makes grabbing the red target dot consistent across platforms. A
        # slightly larger tolerance keeps the dot easy to grab while still
        # preventing accidental drags from distant clicks.
        tolerance = 0.02  # metres
        distance = math.hypot(event.xdata - self.target[0], event.ydata - self.target[1])
        axis = self._pick_gizmo_axis(event)
        if axis is None and distance <= tolerance:
            axis = "xy"
        if axis is not None:
            self.drag_state.dragging = True
            self.drag_state.last_event = event
            self.drag_state.active_axis = axis
            self.drag_state.start_target = self.target.copy()
            return

        # If the click is outside the tolerance treat it as a request to jump
        # the target to the clicked location.  This provides an easy way to
        # reposition the end-effector even if the user misses the dot on the
        # first try, after which standard dragging takes over.
        self.target[0] = event.xdata
        self.target[1] = event.ydata
        self.drag_state.dragging = True
        self.drag_state.last_event = event
        self.drag_state.active_axis = "xy"
        self.drag_state.start_target = self.target.copy()
        self.update_robot()

    def _on_release(self, event) -> None:
        if self._handle_timeline_release(event):
            return
        if self._handle_waypoint_release(event):
            return
        self.drag_state.dragging = False
        self.drag_state.last_event = None
        self.drag_state.active_axis = None
        self.drag_state.start_target = None

    def _on_motion(self, event) -> None:
        if self._handle_timeline_motion(event):
            return
        if self._handle_waypoint_motion(event):
            return
        if not self.drag_state.dragging or event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        axis = self.drag_state.active_axis or "xy"
        new_target = self.target.copy()
        if "shift" in self._modifiers or "ctrl" in self._modifiers or "control" in self._modifiers:
            direction = 0.0
            if any(mod in self._modifiers for mod in {"shift"}):
                direction += 1.0
            if any(mod in self._modifiers for mod in {"ctrl", "control"}):
                direction -= 1.0
            if direction != 0.0 and self.drag_state.last_event is not None:
                delta_pixels = self.drag_state.last_event.y - event.y
                new_target[2] += direction * delta_pixels * self._drag_z_scale
        else:
            if axis in {"x", "xy"}:
                new_target[0] = event.xdata
            if axis in {"y", "xy"}:
                new_target[1] = event.ydata
            if axis == "z" and self.drag_state.last_event is not None:
                delta_pixels = self.drag_state.last_event.y - event.y
                new_target[2] += delta_pixels * self._drag_z_scale
        self.target[:] = new_target
        self.update_robot()
        self.drag_state.last_event = event

    def _on_scroll(self, event) -> None:
        if event.inaxes != self.ax:
            return
        step = 0.01 if event.button == "up" else -0.01
        updated = self.target.copy()
        updated[2] += step
        self.target[:] = self._clamp_target(updated)
        self.update_robot()

    def _on_key_press(self, event) -> None:
        if event.key is None:
            return
        self._modifiers.add(event.key.lower())

    def _on_key_release(self, event) -> None:
        if event.key is None:
            return
        self._modifiers.discard(event.key.lower())

    # ------------------------------------------------------------------
    # UI helpers
    def _create_gizmo(self) -> dict[str, Line2D]:
        lines: dict[str, Line2D] = {}
        for axis, direction in self._gizmo_vectors.items():
            color = self._gizmo_colors.get(axis, "black")
            endpoint = self.target + direction * self._gizmo_length
            (line,) = self.ax.plot(
                [self.target[0], endpoint[0]],
                [self.target[1], endpoint[1]],
                [self.target[2], endpoint[2]],
                color=color,
                linewidth=2,
                marker="o",
                markersize=6,
                markerfacecolor=color,
                markeredgecolor=color,
                alpha=0.9,
                picker=5,
            )
            lines[axis] = line
        return lines

    def _update_gizmo(self) -> None:
        for axis, line in self._gizmo_lines.items():
            direction = self._gizmo_vectors.get(axis)
            if direction is None:
                continue
            endpoint = self.target + direction * self._gizmo_length
            line.set_data_3d(
                [self.target[0], endpoint[0]],
                [self.target[1], endpoint[1]],
                [self.target[2], endpoint[2]],
            )

    def _project_point(self, point: np.ndarray) -> tuple[float, float]:
        x2, y2, _ = proj3d.proj_transform(point[0], point[1], point[2], self.ax.get_proj())
        sx, sy = self.ax.transData.transform((x2, y2))
        return sx, sy

    def _pick_gizmo_axis(self, event) -> str | None:
        if event.x is None or event.y is None:
            return None
        tolerance = 18.0
        target_screen = self._project_point(self.target)
        for axis, direction in self._gizmo_vectors.items():
            endpoint = self.target + direction * self._gizmo_length
            screen = self._project_point(endpoint)
            if math.hypot(event.x - screen[0], event.y - screen[1]) <= tolerance:
                return axis
        if math.hypot(event.x - target_screen[0], event.y - target_screen[1]) <= tolerance:
            return "xy"
        return None

    def _create_controls(self) -> None:
        """Create on-figure UI elements organised into collapsible panels."""

        self.figure.subplots_adjust(left=0.045, right=0.575, top=0.965, bottom=0.08)

        self._panel_left = 0.61
        self._panel_bottom = 0.08
        self._panel_width = 0.36
        self._panel_height = 0.88
        self._panel_margin = 0.035
        self._panel_menu_height = 0.16
        self._panel_content_top = 1.0 - self._panel_menu_height - self._panel_margin

        background = self._panel_axes(0.0, 0.0, 1.0, 1.0)
        background.set_xticks([])
        background.set_yticks([])
        background.set_facecolor("#f4f4f4")
        for spine in background.spines.values():
            spine.set_visible(False)
        background.set_zorder(-10)

        menu_entries = [
            ("Movement", "movement"),
            ("Servos", "servos"),
            ("Motor config", "motor_config"),
            ("Path", "waypoints"),
            ("Timeline", "timeline"),
        ]

        self._panel_widgets = {key: [] for _, key in menu_entries}
        self._panel_interactive_widgets: dict[str, list] = {
            key: [] for _, key in menu_entries
        }
        self._panel_menu_buttons: dict[str, Button] = {}
        self._active_panel: str | None = None

        button_height = self._panel_menu_height * 0.65
        menu_bottom = 1.0 - self._panel_menu_height + (
            self._panel_menu_height - button_height
        ) / 2
        available_width = 1.0 - 2 * self._panel_margin
        button_width = (
            available_width - (len(menu_entries) - 1) * self._panel_margin
        ) / len(menu_entries)

        for idx, (label, key) in enumerate(menu_entries):
            left = self._panel_margin + idx * (button_width + self._panel_margin)
            ax = self._panel_axes(left, menu_bottom, button_width, button_height)
            button = Button(ax, label, hovercolor="#d9e8ff")
            button.on_clicked(lambda _event, name=key: self._set_active_panel(name))
            self._panel_menu_buttons[key] = button

        self.buttons = {}
        self.servo_value_texts = []
        self.servo_buttons = []
        self.servo_invert_buttons = []
        self.servo_limit_boxes_min = []
        self.servo_limit_boxes_max = []
        self.servo_pulse_sliders = []

        self._panel_widgets["movement"] = self._build_movement_panel()
        self._panel_widgets["servos"] = self._build_servo_panel()
        self._panel_widgets["motor_config"] = self._build_motor_config_panel()
        self._panel_widgets["waypoints"] = self._build_waypoint_panel()
        self._panel_widgets["timeline"] = self._build_timeline_panel()

        self._set_active_panel("movement")
        self._refresh_waypoint_display()
        self._update_waypoint_duration_box()

    def _panel_axes(
        self, rel_left: float, rel_bottom: float, rel_width: float, rel_height: float
    ):
        return self.figure.add_axes(
            [
                self._panel_left + rel_left * self._panel_width,
                self._panel_bottom + rel_bottom * self._panel_height,
                rel_width * self._panel_width,
                rel_height * self._panel_height,
            ]
        )

    def _set_active_panel(self, panel: str) -> None:
        if self._active_panel == panel:
            return

        for name, axes in self._panel_widgets.items():
            visible = name == panel
            for axis in axes:
                axis.set_visible(visible)
        for name, widgets in self._panel_interactive_widgets.items():
            active = name == panel
            for widget in widgets:
                if hasattr(widget, "set_active"):
                    widget.set_active(active)
                elif hasattr(widget, "eventson"):
                    widget.eventson = active
        for name, button in self._panel_menu_buttons.items():
            if name == panel:
                button.color = "#aac8ff"
                button.hovercolor = "#aac8ff"
            else:
                button.color = "0.85"
                button.hovercolor = "0.95"
            button.ax.set_facecolor(button.color)
        self._active_panel = panel
        self.figure.canvas.draw_idle()

    def _build_movement_panel(self) -> list:
        panel_key = "movement"
        axes: list = []

        title_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.08,
            1.0 - 2 * self._panel_margin,
            0.06,
        )
        title_ax.axis("off")
        title_ax.text(
            0.0,
            0.5,
            "Target nudging",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        axes.append(title_ax)

        pad_left = self._panel_margin + 0.045
        pad_bottom = self._panel_margin + 0.33
        pad_size = 0.19
        pad_gap = 0.03

        button_defs = {
            "up": (
                pad_left,
                pad_bottom + pad_size + pad_gap,
                "▲",
                (0.0, self.step_xy, 0.0),
            ),
            "down": (
                pad_left,
                pad_bottom - pad_size - pad_gap,
                "▼",
                (0.0, -self.step_xy, 0.0),
            ),
            "left": (
                pad_left - pad_size - pad_gap,
                pad_bottom,
                "◀",
                (-self.step_xy, 0.0, 0.0),
            ),
            "right": (
                pad_left + pad_size + pad_gap,
                pad_bottom,
                "▶",
                (self.step_xy, 0.0, 0.0),
            ),
            "raise": (
                pad_left + 2 * (pad_size + pad_gap),
                pad_bottom + pad_size + pad_gap,
                "Z+",
                (0.0, 0.0, self.step_z),
            ),
            "lower": (
                pad_left + 2 * (pad_size + pad_gap),
                pad_bottom - pad_size - pad_gap,
                "Z-",
                (0.0, 0.0, -self.step_z),
            ),
        }

        for name, (x, y, label, delta) in button_defs.items():
            axes_obj = self._panel_axes(x, y, pad_size, pad_size)
            button = Button(axes_obj, label, hovercolor="#e8f0ff")
            button.on_clicked(self._make_move_callback(delta))
            self.buttons[name] = button
            self._panel_interactive_widgets[panel_key].append(button)
            axes.append(axes_obj)

        centre_ax = self._panel_axes(pad_left, pad_bottom, pad_size, pad_size)
        centre_ax.axis("off")
        centre_ax.text(
            0.5,
            0.5,
            "XY",
            ha="center",
            va="center",
            fontsize=10,
            transform=centre_ax.transAxes,
        )
        axes.append(centre_ax)

        slider_ax = self._panel_axes(
            self._panel_margin,
            self._panel_margin + 0.2,
            1.0 - 2 * self._panel_margin,
            0.08,
        )
        self._wrist_slider = Slider(
            slider_ax,
            "Wrist pitch (°)",
            valmin=self._wrist_slider_limits[0],
            valmax=self._wrist_slider_limits[1],
            valinit=self._slider_value_from_pitch(self.wrist_pitch),
        )
        self._wrist_slider.on_changed(self._handle_wrist_slider_change)
        self._panel_interactive_widgets[panel_key].append(self._wrist_slider)
        axes.append(slider_ax)

        home_ax = self._panel_axes(
            pad_left,
            self._panel_margin + 0.1,
            pad_size * 2 + pad_gap,
            0.12,
        )
        self._home_button = Button(home_ax, "Home", hovercolor="0.95")
        self._home_button.on_clicked(self._handle_home_button)
        self._panel_interactive_widgets[panel_key].append(self._home_button)
        axes.append(home_ax)

        self._update_wrist_slider_display()

        return axes

    def _build_servo_panel(self) -> list:
        panel_key = "servos"
        axes: list = []

        title_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.08,
            1.0 - 2 * self._panel_margin,
            0.06,
        )
        title_ax.axis("off")
        title_ax.text(
            0.0,
            0.5,
            "Servo tuning",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        axes.append(title_ax)

        top_edge = self._panel_content_top - 0.12
        bottom_edge = self._panel_margin + 0.18
        row_gap = 0.02
        available_height = max(
            top_edge - bottom_edge - (len(self._SERVO_METADATA) - 1) * row_gap, 0.001
        )
        row_height = available_height / max(len(self._SERVO_METADATA), 1)

        value_left = self._panel_margin
        value_width = 0.52
        gap_small = 0.02
        invert_gap = 0.03
        minus_width = 0.08
        plus_width = 0.08
        invert_width = 0.1

        for index, (name, model, location) in enumerate(self._SERVO_METADATA):
            row_bottom = bottom_edge + (
                len(self._SERVO_METADATA) - index - 1
            ) * (row_height + row_gap)
            control_height = row_height

            value_ax = self._panel_axes(
                value_left,
                row_bottom,
                value_width,
                control_height,
            )
            value_ax.axis("off")
            text = value_ax.text(
                0.0,
                0.5,
                f"{name} ({model})\n{location}\nAngle: 0.0°",
                va="center",
                ha="left",
                fontsize=9,
                linespacing=1.5,
                wrap=True,
                transform=value_ax.transAxes,
            )
            self.servo_value_texts.append(text)
            axes.append(value_ax)

            minus_left = value_left + value_width + gap_small
            plus_left = minus_left + minus_width + gap_small
            invert_left = plus_left + plus_width + gap_small
            minus_ax = self._panel_axes(
                minus_left,
                row_bottom,
                minus_width,
                control_height,
            )
            plus_ax = self._panel_axes(
                plus_left,
                row_bottom,
                plus_width,
                control_height,
            )
            invert_ax = self._panel_axes(
                invert_left,
                row_bottom,
                invert_width,
                control_height,
            )
            minus_button = Button(minus_ax, "-", hovercolor="0.975")
            plus_button = Button(plus_ax, "+", hovercolor="0.975")
            invert_button = Button(invert_ax, "Inv", hovercolor="0.975")

            minus_button.on_clicked(self._make_servo_adjust_callback(index, -math.radians(5)))
            plus_button.on_clicked(self._make_servo_adjust_callback(index, math.radians(5)))
            invert_button.on_clicked(self._make_inversion_toggle_callback(index))

            self.servo_buttons.extend([minus_button, plus_button])
            self.servo_invert_buttons.append(invert_button)
            self._panel_interactive_widgets[panel_key].extend(
                [minus_button, plus_button, invert_button]
            )
            self._update_inversion_button_visual(index)

            axes.extend([minus_ax, plus_ax, invert_ax])

        control_bottom = self._panel_margin + 0.06
        control_height = 0.085
        control_gap = 0.022
        control_width = (
            1.0 - 2 * self._panel_margin - 2 * control_gap
        ) / 3

        calibrate_ax = self._panel_axes(
            self._panel_margin,
            control_bottom,
            control_width,
            control_height,
        )
        set_vertical_ax = self._panel_axes(
            self._panel_margin + control_width + control_gap,
            control_bottom,
            control_width,
            control_height,
        )
        raw_toggle_ax = self._panel_axes(
            self._panel_margin + 2 * (control_width + control_gap),
            control_bottom,
            control_width,
            control_height,
        )

        self._calibration_button = Button(calibrate_ax, "Calibrate", hovercolor="0.95")
        self._calibration_button.on_clicked(self._toggle_calibration)
        self._panel_interactive_widgets[panel_key].append(self._calibration_button)
        self._set_vertical_button = Button(
            set_vertical_ax, "Set vertical", hovercolor="0.95"
        )
        self._set_vertical_button.on_clicked(self._handle_set_vertical)
        self._panel_interactive_widgets[panel_key].append(self._set_vertical_button)
        self._raw_angle_button = Button(raw_toggle_ax, "Show raw", hovercolor="0.95")
        self._raw_angle_button.on_clicked(self._toggle_raw_angle_display)
        self._panel_interactive_widgets[panel_key].append(self._raw_angle_button)

        self._update_calibration_button_visual()
        self._update_raw_angle_button_visual()

        axes.extend([calibrate_ax, set_vertical_ax, raw_toggle_ax])

        return axes

    def _build_motor_config_panel(self) -> list:
        panel_key = "motor_config"
        axes: list = []

        title_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.08,
            1.0 - 2 * self._panel_margin,
            0.06,
        )
        title_ax.axis("off")
        title_ax.text(
            0.0,
            0.5,
            "Motor configuration",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        axes.append(title_ax)

        guide_height = 0.085
        guide_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.16,
            1.0 - 2 * self._panel_margin,
            guide_height,
        )
        guide_ax.axis("off")
        guide_ax.text(
            0.0,
            1.0,
            "Calibration helper:\n"
            "1) Tap Calibrate to relax the motors.\n"
            "2) Move each servo with +/- or the Pulse slider until the arm looks right.\n"
            "3) Press Set vertical to store that upright pose.\n"
            "Soft min°/Soft max° below are safety stops only—"
            "they do not change the calibration.",
            va="top",
            ha="left",
            fontsize=8,
            wrap=True,
            linespacing=1.4,
        )
        axes.append(guide_ax)

        top_edge = self._panel_content_top - guide_height - 0.13
        bottom_edge = self._panel_margin + 0.12
        row_gap = 0.02
        available_height = max(
            top_edge - bottom_edge - (len(self._SERVO_METADATA) - 1) * row_gap, 0.001
        )
        row_height = available_height / max(len(self._SERVO_METADATA), 1)

        label_width = 0.32
        slider_width = 0.38
        limit_width = 0.1
        gap_small = 0.012

        for index, (name, model, location) in enumerate(self._SERVO_METADATA):
            row_bottom = bottom_edge + (
                len(self._SERVO_METADATA) - index - 1
            ) * (row_height + row_gap)

            label_ax = self._panel_axes(
                self._panel_margin,
                row_bottom + row_height * 0.45,
                label_width,
                row_height * 0.5,
            )
            label_ax.axis("off")
            label_ax.text(
                0.0,
                0.5,
                f"{name} ({model})\n{location}",
                va="center",
                ha="left",
                fontsize=9,
                linespacing=1.5,
                wrap=True,
                transform=label_ax.transAxes,
            )
            axes.append(label_ax)

            slider_left = self._panel_margin + label_width + gap_small
            slider_ax = self._panel_axes(
                slider_left,
                row_bottom + row_height * 0.05,
                slider_width,
                row_height * 0.55,
            )

            config = self.servo_configs.get(index)
            current_source = self.feedback_joints or self.current_joints
            current_angle = (
                current_source[index]
                if current_source and index < len(current_source)
                else 0.0
            )
            current_pulse = (
                config.angle_to_pulse(current_angle) if config is not None else 0.0
            )
            slider = Slider(
                slider_ax,
                "Pulse (µs)",
                valmin=min(config.min_pulse, config.max_pulse) if config else 0.0,
                valmax=max(config.min_pulse, config.max_pulse) if config else 0.0,
                valinit=current_pulse,
                valfmt="%0.0f µs",
            )
            slider.on_changed(self._make_pulse_slider_callback(index))
            self.servo_pulse_sliders.append(slider)
            self._panel_interactive_widgets[panel_key].append(slider)
            axes.append(slider_ax)

            min_left = slider_left + slider_width + gap_small
            max_left = min_left + limit_width + gap_small

            min_ax = self._panel_axes(
                min_left,
                row_bottom + row_height * 0.05,
                limit_width,
                row_height * 0.5,
            )
            max_ax = self._panel_axes(
                max_left,
                row_bottom + row_height * 0.05,
                limit_width,
                row_height * 0.5,
            )

            min_box = TextBox(min_ax, "Soft min°", initial="0.0")
            max_box = TextBox(max_ax, "Soft max°", initial="0.0")
            min_box.on_submit(self._make_limit_submit_callback(index, "min"))
            max_box.on_submit(self._make_limit_submit_callback(index, "max"))

            self.servo_limit_boxes_min.append(min_box)
            self.servo_limit_boxes_max.append(max_box)
            self._panel_interactive_widgets[panel_key].extend([min_box, max_box])
            self._update_limit_box_display(index)
            self._refresh_pulse_slider_limits(index)

            axes.extend([min_ax, max_ax])

        return axes

    def _build_waypoint_panel(self) -> list:
        panel_key = "waypoints"
        axes: list = []

        title_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.08,
            1.0 - 2 * self._panel_margin,
            0.06,
        )
        title_ax.axis("off")
        title_ax.text(
            0.0,
            0.5,
            "Path planning",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        axes.append(title_ax)

        duration_height = 0.08
        duration_bottom = self._panel_content_top - 0.12
        duration_ax = self._panel_axes(
            self._panel_margin,
            duration_bottom,
            1.0 - 2 * self._panel_margin,
            duration_height,
        )
        self._waypoint_duration_box = TextBox(
            duration_ax, "Duration (s)", initial="2.0"
        )
        self._waypoint_duration_box.on_submit(self._handle_duration_submit)
        self._panel_interactive_widgets[panel_key].append(self._waypoint_duration_box)
        axes.append(duration_ax)

        button_height = 0.085
        button_gap = 0.025
        button_width = (
            1.0 - 2 * self._panel_margin - 2 * button_gap
        ) / 3
        button_bottom = duration_bottom - button_height - 0.03

        add_ax = self._panel_axes(
            self._panel_margin,
            button_bottom,
            button_width,
            button_height,
        )
        play_ax = self._panel_axes(
            self._panel_margin + button_width + button_gap,
            button_bottom,
            button_width,
            button_height,
        )
        clear_ax = self._panel_axes(
            self._panel_margin + 2 * (button_width + button_gap),
            button_bottom,
            button_width,
            button_height,
        )

        self._add_waypoint_button = Button(add_ax, "Add waypoint", hovercolor="0.95")
        self._add_waypoint_button.on_clicked(self._handle_add_waypoint)
        self._panel_interactive_widgets[panel_key].append(self._add_waypoint_button)
        self._play_waypoints_button = Button(play_ax, "Play path", hovercolor="0.95")
        self._play_waypoints_button.on_clicked(self._handle_play_waypoints)
        self._panel_interactive_widgets[panel_key].append(self._play_waypoints_button)
        self._clear_waypoints_button = Button(clear_ax, "Clear path", hovercolor="0.95")
        self._clear_waypoints_button.on_clicked(self._handle_clear_waypoints)
        self._panel_interactive_widgets[panel_key].append(self._clear_waypoints_button)

        axes.extend([add_ax, play_ax, clear_ax])

        waypoint_bottom = self._panel_margin + 0.02
        waypoint_height = button_bottom - waypoint_bottom - 0.04
        self._waypoint_item_height = 0.15
        self._waypoint_item_gap = 0.045

        self.waypoint_ax = self._panel_axes(
            self._panel_margin,
            waypoint_bottom,
            1.0 - 2 * self._panel_margin,
            waypoint_height,
        )
        self.waypoint_ax.set_xlim(0, 1)
        self.waypoint_ax.set_ylim(0, 1)
        self.waypoint_ax.set_xticks([])
        self.waypoint_ax.set_yticks([])
        self.waypoint_ax.set_facecolor("#f7f7f7")
        self.waypoint_ax.set_title("Waypoints", pad=8)
        axes.append(self.waypoint_ax)

        self._refresh_waypoint_display()

        return axes

    def _build_timeline_panel(self) -> list:
        panel_key = "timeline"
        axes: list = []

        title_offset = 0.02
        title_height = 0.05
        name_gap = 0.015
        name_height = 0.07
        button_gap = 0.02
        button_height = 0.07
        subroutine_gap = 0.03
        timeline_buttons_gap = 0.025
        timeline_buttons_height = 0.07
        slider_gap = 0.025
        slider_height = 0.045
        min_subroutine_height = 0.14
        preferred_subroutine_height = 0.22
        min_timeline_height = 0.12

        title_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - title_offset - title_height,
            1.0 - 2 * self._panel_margin,
            title_height,
        )
        title_ax.axis("off")
        title_ax.text(
            0.0,
            0.5,
            "Subroutines & timeline",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        axes.append(title_ax)

        timeline_bottom_base = self._panel_margin + 0.03
        available_height = self._panel_content_top - timeline_bottom_base
        base_height = (
            title_offset
            + title_height
            + name_gap
            + name_height
            + button_gap
            + button_height
            + subroutine_gap
            + timeline_buttons_gap
            + timeline_buttons_height
            + slider_gap
            + slider_height
        )

        remaining_height = max(available_height - base_height, 0.0)
        min_required_height = min_subroutine_height + min_timeline_height

        if remaining_height < min_required_height:
            scale = remaining_height / min_required_height if min_required_height else 1.0
            subroutine_height = min_subroutine_height * scale
            timeline_height = min_timeline_height * scale
        else:
            subroutine_height = np.clip(
                remaining_height - min_timeline_height,
                min_subroutine_height,
                preferred_subroutine_height,
            )
            timeline_height = remaining_height - subroutine_height
            if timeline_height < min_timeline_height:
                timeline_height = min_timeline_height
                subroutine_height = max(
                    min_subroutine_height, remaining_height - timeline_height
                )

        y_cursor = self._panel_content_top

        y_cursor -= title_offset + title_height
        name_bottom = y_cursor - name_gap - name_height
        name_ax = self._panel_axes(
            self._panel_margin,
            name_bottom,
            1.0 - 2 * self._panel_margin,
            name_height,
        )
        self._subroutine_name_box = TextBox(name_ax, "Subroutine", initial="")
        self._panel_interactive_widgets[panel_key].append(self._subroutine_name_box)
        axes.append(name_ax)

        y_cursor = name_bottom
        button_width = (
            1.0 - 2 * self._panel_margin - 2 * button_gap
        ) / 3
        button_bottom = y_cursor - button_gap - button_height
        save_ax = self._panel_axes(
            self._panel_margin,
            button_bottom,
            button_width,
            button_height,
        )
        load_ax = self._panel_axes(
            self._panel_margin + button_width + button_gap,
            button_bottom,
            button_width,
            button_height,
        )
        refresh_ax = self._panel_axes(
            self._panel_margin + 2 * (button_width + button_gap),
            button_bottom,
            button_width,
            button_height,
        )
        self._save_subroutine_button = Button(save_ax, "Save current", hovercolor="0.95")
        self._save_subroutine_button.on_clicked(self._handle_save_subroutine)
        self._load_subroutine_button = Button(load_ax, "Load subroutine", hovercolor="0.95")
        self._load_subroutine_button.on_clicked(self._handle_load_subroutine)
        self._refresh_subroutine_button = Button(
            refresh_ax, "Refresh list", hovercolor="0.95"
        )
        self._refresh_subroutine_button.on_clicked(self._handle_refresh_subroutines)
        self._panel_interactive_widgets[panel_key].extend(
            [
                self._save_subroutine_button,
                self._load_subroutine_button,
                self._refresh_subroutine_button,
            ]
        )
        axes.extend([save_ax, load_ax, refresh_ax])

        y_cursor = button_bottom - subroutine_gap
        subroutine_bottom = y_cursor - subroutine_height
        self._subroutine_list_ax = self._panel_axes(
            self._panel_margin,
            subroutine_bottom,
            1.0 - 2 * self._panel_margin,
            subroutine_height,
        )
        self._subroutine_list_ax.set_xlim(0, 1)
        self._subroutine_list_ax.set_ylim(0, 1)
        self._subroutine_list_ax.set_xticks([])
        self._subroutine_list_ax.set_yticks([])
        self._subroutine_list_ax.set_facecolor("#f7f7f7")
        self._subroutine_list_ax.set_title("Saved subroutines", pad=8)
        axes.append(self._subroutine_list_ax)

        timeline_button_count = 5
        timeline_button_width = (
            1.0 - 2 * self._panel_margin - (timeline_button_count - 1) * timeline_buttons_gap
        ) / timeline_button_count
        timeline_button_bottom = (
            subroutine_bottom - timeline_buttons_gap - timeline_buttons_height
        )
        add_ax = self._panel_axes(
            self._panel_margin,
            timeline_button_bottom,
            timeline_button_width,
            timeline_buttons_height,
        )
        remove_ax = self._panel_axes(
            self._panel_margin + timeline_button_width + timeline_buttons_gap,
            timeline_button_bottom,
            timeline_button_width,
            timeline_buttons_height,
        )
        clear_ax = self._panel_axes(
            self._panel_margin + 2 * (timeline_button_width + timeline_buttons_gap),
            timeline_button_bottom,
            timeline_button_width,
            timeline_buttons_height,
        )
        play_ax = self._panel_axes(
            self._panel_margin + 3 * (timeline_button_width + timeline_buttons_gap),
            timeline_button_bottom,
            timeline_button_width,
            timeline_buttons_height,
        )
        repeat_ax = self._panel_axes(
            self._panel_margin + 4 * (timeline_button_width + timeline_buttons_gap),
            timeline_button_bottom,
            timeline_button_width,
            timeline_buttons_height,
        )
        self._add_timeline_button = Button(add_ax, "Add to timeline", hovercolor="0.95")
        self._add_timeline_button.on_clicked(self._handle_add_to_timeline)
        self._remove_timeline_button = Button(remove_ax, "Remove entry", hovercolor="0.95")
        self._remove_timeline_button.on_clicked(self._handle_remove_timeline_entry)
        self._clear_timeline_button = Button(clear_ax, "Clear timeline", hovercolor="0.95")
        self._clear_timeline_button.on_clicked(self._handle_clear_timeline)
        self._play_timeline_button = Button(play_ax, "Play timeline", hovercolor="0.95")
        self._play_timeline_button.on_clicked(self._handle_play_timeline)
        self._timeline_repeat_button = Button(
            repeat_ax, "Repeat: Off", hovercolor="0.95"
        )
        self._timeline_repeat_button.on_clicked(self._handle_toggle_timeline_repeat)
        self._panel_interactive_widgets[panel_key].extend(
            [
                self._add_timeline_button,
                self._remove_timeline_button,
                self._clear_timeline_button,
                self._play_timeline_button,
                self._timeline_repeat_button,
            ]
        )
        axes.extend([add_ax, remove_ax, clear_ax, play_ax, repeat_ax])

        speed_slider_bottom = timeline_button_bottom - slider_gap - slider_height
        speed_ax = self._panel_axes(
            self._panel_margin,
            speed_slider_bottom,
            1.0 - 2 * self._panel_margin,
            slider_height,
        )
        self._timeline_speed_slider = Slider(
            speed_ax,
            "Speed",
            0.1,
            5.0,
            valinit=self._timeline_speed,
            valfmt="%0.1fx",
        )
        self._timeline_speed_slider.on_changed(self._handle_timeline_speed_change)
        axes.append(speed_ax)
        self._update_timeline_repeat_button()

        timeline_height = max(timeline_height, 0.05)
        timeline_bottom = timeline_bottom_base
        self._timeline_ax = self._panel_axes(
            self._panel_margin,
            timeline_bottom,
            1.0 - 2 * self._panel_margin,
            timeline_height,
        )
        self._timeline_ax.set_xlim(0, 1)
        self._timeline_ax.set_ylim(0, 1)
        self._timeline_ax.set_xticks([])
        self._timeline_ax.set_yticks([])
        self._timeline_ax.set_facecolor("#f7f7f7")
        self._timeline_ax.set_title("Timeline schedule", pad=8)
        axes.append(self._timeline_ax)

        self._refresh_subroutine_list_display()
        self._refresh_timeline_display()

        return axes

    def _refresh_subroutine_list_display(self) -> None:
        if self._subroutine_list_ax is None:
            return
        ax = self._subroutine_list_ax
        ax.cla()
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_facecolor("#f7f7f7")
        ax.set_title("Saved subroutines", pad=8)
        self._subroutine_patches = []
        self._subroutine_texts = []
        self._subroutine_display_slugs = []

        summaries = sorted(
            self._subroutine_catalog.values(), key=lambda item: item.name.lower()
        )
        if not summaries:
            ax.text(
                0.5,
                0.5,
                "No subroutines saved",
                ha="center",
                va="center",
                fontsize=10,
                color="#666666",
            )
            self.figure.canvas.draw_idle()
            return

        for index, summary in enumerate(summaries):
            y = 1.0 - (index + 1) * self._subroutine_item_height
            y -= index * self._subroutine_item_gap
            y = max(y, 0.02)
            rect = Rectangle(
                (0.02, y),
                0.96,
                min(self._subroutine_item_height, 0.9),
                facecolor="#ffffff",
                edgecolor="#cccccc",
                linewidth=1,
            )
            if summary.slug == self._selected_subroutine_slug:
                rect.set_facecolor("#cfe8fc")
            ax.add_patch(rect)
            text = ax.text(
                0.04,
                y + self._subroutine_item_height / 2,
                f"{summary.name}\n{summary.waypoint_count} keyframes, {summary.duration:.2f} s",
                va="center",
                ha="left",
                fontsize=9,
                color="#333333",
            )
            self._subroutine_patches.append(rect)
            self._subroutine_texts.append(text)
            self._subroutine_display_slugs.append(summary.slug)

        self.figure.canvas.draw_idle()

    def _refresh_timeline_display(self) -> None:
        if self._timeline_ax is None:
            return
        ax = self._timeline_ax
        ax.cla()
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_facecolor("#f7f7f7")
        ax.set_title("Timeline schedule", pad=8)
        self._timeline_patches = []
        self._timeline_texts = []

        if not self._timeline_entries:
            ax.text(
                0.5,
                0.5,
                "No scheduled subroutines",
                ha="center",
                va="center",
                fontsize=10,
                color="#666666",
            )
            self.figure.canvas.draw_idle()
            return

        for index, entry in enumerate(self._timeline_entries):
            y = 1.0 - (index + 1) * self._timeline_item_height
            y -= index * self._timeline_item_gap
            y = max(y, 0.02)
            rect = Rectangle(
                (0.02, y),
                0.96,
                min(self._timeline_item_height, 0.9),
                facecolor="#ffffff",
                edgecolor="#cccccc",
                linewidth=1,
            )
            if index == self._selected_timeline_index:
                rect.set_facecolor("#cfe8fc")
            ax.add_patch(rect)
            duration_label = f"{entry.duration:.2f} s"
            if entry.missing:
                status = "Missing file"
                text_color = "#b00020"
            else:
                status = "Ready"
                text_color = "#333333"
            text = ax.text(
                0.04,
                y + self._timeline_item_height / 2,
                f"#{index + 1}: {entry.name}\n{duration_label} • {status}",
                va="center",
                ha="left",
                fontsize=9,
                color=text_color,
            )
            self._timeline_patches.append(rect)
            self._timeline_texts.append(text)

        self.figure.canvas.draw_idle()

    def _highlight_selected_subroutine(self) -> None:
        for slug, patch in zip(self._subroutine_display_slugs, self._subroutine_patches):
            if slug == self._selected_subroutine_slug:
                patch.set_facecolor("#cfe8fc")
            else:
                patch.set_facecolor("#ffffff")
        self.figure.canvas.draw_idle()

    def _highlight_selected_timeline(self) -> None:
        for idx, patch in enumerate(self._timeline_patches):
            if idx == self._selected_timeline_index:
                patch.set_facecolor("#cfe8fc")
            else:
                patch.set_facecolor("#ffffff")
        self.figure.canvas.draw_idle()

    def _update_subroutine_name_box(self) -> None:
        if self._subroutine_name_box is None:
            return
        target = ""
        if self._selected_subroutine_slug:
            summary = self._subroutine_catalog.get(self._selected_subroutine_slug)
            if summary is not None:
                target = summary.name
        try:
            self._subroutine_name_box.set_val(target)
        except Exception:
            pass

    def _refresh_waypoint_display(self) -> None:
        if not hasattr(self, "waypoint_ax"):
            return
        self.waypoint_ax.cla()
        self.waypoint_ax.set_xlim(0, 1)
        self.waypoint_ax.set_ylim(0, 1)
        self.waypoint_ax.set_xticks([])
        self.waypoint_ax.set_yticks([])
        self.waypoint_ax.set_facecolor("#f7f7f7")
        self.waypoint_ax.set_title("Waypoints", pad=8)
        self._waypoint_patches = []
        self._waypoint_texts = []

        if not self.waypoints:
            self.waypoint_ax.text(
                0.5,
                0.5,
                "No waypoints",
                ha="center",
                va="center",
                fontsize=10,
                color="#666666",
            )
            self.figure.canvas.draw_idle()
            return

        for index, waypoint in enumerate(self.waypoints):
            y = 1.0 - (index + 1) * self._waypoint_item_height - index * self._waypoint_item_gap
            y = max(y, 0.02)
            rect = Rectangle(
                (0.02, y),
                0.96,
                min(self._waypoint_item_height, 0.9),
                facecolor="#ffffff",
                edgecolor="#cccccc",
                linewidth=1,
            )
            if index == self._selected_waypoint_index:
                rect.set_facecolor("#cfe8fc")
            self.waypoint_ax.add_patch(rect)
            self._waypoint_patches.append(rect)

            position_text = (
                f"#{index + 1}: x={waypoint.position[0]:.3f}, "
                f"y={waypoint.position[1]:.3f}, z={waypoint.position[2]:.3f}"
            )
            pitch_text = (
                f"Wrist: {self._slider_value_from_pitch(waypoint.wrist_pitch):.1f}°"
            )
            text = self.waypoint_ax.text(
                0.04,
                y + self._waypoint_item_height / 2,
                f"{position_text}\n{waypoint.duration:.2f} s, {pitch_text}",
                va="center",
                ha="left",
                fontsize=9,
                color="#333333",
            )
            self._waypoint_texts.append(text)

        self.figure.canvas.draw_idle()

    def _highlight_selected_waypoint(self) -> None:
        for idx, patch in enumerate(self._waypoint_patches):
            if idx == self._selected_waypoint_index:
                patch.set_facecolor("#cfe8fc")
            else:
                patch.set_facecolor("#ffffff")
        self.figure.canvas.draw_idle()

    def _update_waypoint_duration_box(self) -> None:
        if not hasattr(self, "_waypoint_duration_box"):
            return
        if self._waypoint_duration_box is None:
            return
        if self._updating_duration_box:
            return
        value = "2.0"
        if (
            self._selected_waypoint_index is not None
            and 0 <= self._selected_waypoint_index < len(self.waypoints)
        ):
            value = f"{self.waypoints[self._selected_waypoint_index].duration:.2f}"
        try:
            self._updating_duration_box = True
            self._waypoint_duration_box.set_val(value)
        finally:
            self._updating_duration_box = False

    def _waypoints_from_serialised(self, data: object) -> list[Waypoint]:
        if not isinstance(data, list):
            return []

        loaded: list[Waypoint] = []
        for entry in data:
            if not isinstance(entry, dict):
                continue
            position = entry.get("position")
            duration = entry.get("duration")
            wrist_pitch = entry.get("wrist_pitch")
            try:
                position_array = np.array(position, dtype=float)
                if position_array.shape != (3,):
                    continue
                duration_value = float(duration)
                pitch_value = float(wrist_pitch)
            except (TypeError, ValueError):
                continue
            loaded.append(
                Waypoint(
                    position=position_array,
                    duration=max(0.1, duration_value),
                    wrist_pitch=float(np.clip(pitch_value, *self._wrist_pitch_limits)),
                )
            )
        return loaded

    def _load_saved_waypoints(self) -> None:
        try:
            if not PATH_STORAGE_PATH.exists():
                return
            data = json.loads(PATH_STORAGE_PATH.read_text())
        except Exception:
            _LOGGER.warning(
                "Failed to load saved path from %s", PATH_STORAGE_PATH, exc_info=True
            )
            return

        loaded = self._waypoints_from_serialised(data)
        self.waypoints = loaded if loaded else []

    def _compute_waypoint_total_duration(self, waypoints: list[Waypoint]) -> float:
        return sum(max(0.1, waypoint.duration) for waypoint in waypoints)

    def _slugify_subroutine_name(self, name: str) -> str:
        cleaned = " ".join(str(name or "").split())
        cleaned = cleaned.strip()
        safe = "".join(
            ch if ch.isalnum() or ch in {"_", "-"} else "_" for ch in cleaned
        )
        safe = safe.strip("_")
        if not safe:
            safe = "subroutine"
        return safe.lower()

    def _load_subroutine_catalog(self) -> None:
        catalog: dict[str, SubroutineSummary] = {}
        if SUBROUTINE_STORAGE_DIR.exists():
            for path in sorted(SUBROUTINE_STORAGE_DIR.glob("*.json")):
                slug = path.stem
                try:
                    data = json.loads(path.read_text())
                except Exception:
                    _LOGGER.warning("Failed to read subroutine %s", path, exc_info=True)
                    continue
                waypoints = self._waypoints_from_serialised(data.get("waypoints"))
                if not waypoints:
                    continue
                name = str(data.get("name") or slug)
                summary = SubroutineSummary(
                    name=name,
                    slug=slug,
                    path=path,
                    duration=self._compute_waypoint_total_duration(waypoints),
                    waypoint_count=len(waypoints),
                )
                catalog[slug] = summary
        self._subroutine_catalog = catalog
        if self._selected_subroutine_slug not in self._subroutine_catalog:
            self._selected_subroutine_slug = None
        self._refresh_subroutine_list_display()

    def _load_subroutine_waypoints(self, slug: str) -> list[Waypoint]:
        if not slug:
            return []
        path = SUBROUTINE_STORAGE_DIR / f"{slug}.json"
        try:
            data = json.loads(path.read_text())
        except Exception:
            _LOGGER.warning("Failed to load subroutine %s", path, exc_info=True)
            return []
        return self._waypoints_from_serialised(data.get("waypoints"))

    def _load_saved_timeline(self) -> None:
        entries: list[TimelineEntry] = []
        if TIMELINE_STORAGE_PATH.exists():
            try:
                data = json.loads(TIMELINE_STORAGE_PATH.read_text())
            except Exception:
                _LOGGER.warning(
                    "Failed to load saved timeline from %s",
                    TIMELINE_STORAGE_PATH,
                    exc_info=True,
                )
            else:
                if isinstance(data, list):
                    for entry in data:
                        if not isinstance(entry, dict):
                            continue
                        slug = str(entry.get("slug") or "").strip()
                        if not slug:
                            continue
                        summary = self._subroutine_catalog.get(slug)
                        if summary is not None:
                            entries.append(
                                TimelineEntry(
                                    name=summary.name,
                                    slug=slug,
                                    duration=summary.duration,
                                    missing=False,
                                )
                            )
                        else:
                            entries.append(
                                TimelineEntry(
                                    name=str(entry.get("name") or slug),
                                    slug=slug,
                                    duration=float(entry.get("duration") or 0.0),
                                    missing=True,
                                )
                            )
        self._timeline_entries = entries
        self._selected_timeline_index = None
        self._refresh_timeline_display()

    def _save_timeline(self) -> None:
        try:
            TIMELINE_STORAGE_PATH.parent.mkdir(parents=True, exist_ok=True)
            serialisable = [
                {"name": entry.name, "slug": entry.slug} for entry in self._timeline_entries
            ]
            TIMELINE_STORAGE_PATH.write_text(json.dumps(serialisable, indent=2))
        except Exception:
            _LOGGER.warning(
                "Failed to persist timeline data to %s",
                TIMELINE_STORAGE_PATH,
                exc_info=True,
            )

    def _update_timeline_entries_for_slug(self, slug: str) -> None:
        summary = self._subroutine_catalog.get(slug)
        if summary is None:
            return
        changed = False
        for entry in self._timeline_entries:
            if entry.slug == slug:
                entry.name = summary.name
                entry.duration = summary.duration
                entry.missing = False
                changed = True
        if changed:
            self._refresh_timeline_display()
            self._save_timeline()

    def _save_waypoints(self) -> None:
        try:
            PATH_STORAGE_PATH.parent.mkdir(parents=True, exist_ok=True)
            serialisable = self._serialise_waypoints(self.waypoints)
            PATH_STORAGE_PATH.write_text(json.dumps(serialisable, indent=2))
        except Exception:
            _LOGGER.warning(
                "Failed to persist path data to %s", PATH_STORAGE_PATH, exc_info=True
            )

    def _serialise_waypoints(self, waypoints: list[Waypoint]) -> list[dict[str, object]]:
        return [
            {
                "position": waypoint.position.tolist(),
                "duration": float(waypoint.duration),
                "wrist_pitch": float(waypoint.wrist_pitch),
            }
            for waypoint in waypoints
        ]


    def _make_move_callback(self, delta: tuple[float, float, float]):
        def _callback(event) -> None:  # pragma: no cover - UI interaction
            self._nudge_target(*delta)

        return _callback

    def _make_servo_adjust_callback(self, index: int, delta: float):
        def _callback(event) -> None:  # pragma: no cover - UI interaction
            self._adjust_servo(index, delta)

        return _callback

    def _make_inversion_toggle_callback(self, index: int):
        def _callback(event) -> None:  # pragma: no cover - UI interaction
            self._toggle_servo_inversion(index)

        return _callback

    def _make_limit_submit_callback(self, index: int, bound: str):
        def _callback(text: str) -> None:  # pragma: no cover - UI interaction
            try:
                value = float(text)
            except ValueError:
                name, model, _ = self._SERVO_METADATA[index]
                _LOGGER.warning(
                    "Ignoring invalid soft %s limit for %s (%s)", bound, name, model
                )
                self._update_limit_box_display(index)
                return

            radians_value = math.radians(value)
            if bound == "min":
                self._update_servo_limit(index, min_angle=radians_value)
            else:
                self._update_servo_limit(index, max_angle=radians_value)

        return _callback

    def _make_pulse_slider_callback(self, index: int):
        def _callback(value: float) -> None:  # pragma: no cover - UI interaction
            config = self.servo_configs.get(index)
            if config is None:
                return
            self._set_servo_angle(
                index, config.pulse_to_angle(value), use_soft_limits=False
            )

        return _callback

    def _nudge_target(self, dx: float, dy: float, dz: float) -> None:
        updated = self.target.copy()
        updated[0] += dx
        updated[1] += dy
        updated[2] += dz
        self.target[:] = self._clamp_target(updated)
        self.update_robot()

    def _adjust_servo(self, index: int, delta: float) -> None:
        source = self.feedback_joints or self.current_joints
        if not source:
            return

        updated = list(source)
        new_angle = updated[index] + delta
        self._set_servo_angle(index, new_angle)

    def _set_servo_angle(
        self, index: int, angle: float, *, use_soft_limits: bool = True
    ) -> None:
        config = self.servo_configs.get(index)
        if config is None:
            return

        source = self.feedback_joints or self.current_joints
        if not source:
            return

        limits = self._active_angle_limits(index, use_soft_limits=use_soft_limits)
        updated = list(source)
        old_angle = updated[index]
        new_angle = float(np.clip(angle, *limits))
        if math.isclose(new_angle, old_angle, abs_tol=1e-6):
            name, model, _ = self._SERVO_METADATA[index]
            _LOGGER.warning(
                "%s (%s) servo adjustment hit the configured limit (%.1f° to %.1f°).",
                name,
                model,
                math.degrees(limits[0]),
                math.degrees(limits[1]),
            )
        updated[index] = new_angle

        if index >= 4:
            if index == 4:
                self.wrist_rotation = new_angle
            else:
                self.gripper_angle = new_angle
            self.commanded_joints = list(updated)
            self.current_joints = list(updated)
            self._update_servo_readouts()
            self._cancel_pending_commands()
            self._send_move_command(
                updated, move_time_ms=self.move_time_ms, soft_start=False
            )
            return

        self.commanded_joints = list(updated)
        self.current_joints = list(updated)
        forward_pose = self.kin.forward(self.commanded_joints)
        self.target[:] = forward_pose[:3, 3]
        actual_pitch = sum(self.commanded_joints[1:4])
        self._last_wrist_pitch = actual_pitch
        self._set_wrist_pitch_target(actual_pitch)
        self._update_visuals(self.commanded_joints[:4])
        self._update_servo_readouts()
        self._cancel_pending_commands()
        self._send_move_command(
            self.commanded_joints,
            move_time_ms=self.move_time_ms,
            soft_start=False,
        )

    def _toggle_servo_inversion(self, index: int) -> None:
        if index >= len(self.servo_inversions):
            return

        self.servo_inversions[index] = not self.servo_inversions[index]
        self._apply_servo_inversion(index)
        self._update_inversion_button_visual(index)
        self._save_calibration_data()

        name, model, _ = self._SERVO_METADATA[index]
        state = "enabled" if self.servo_inversions[index] else "disabled"
        _LOGGER.info("%s (%s) servo inversion %s", name, model, state)
        self._send_move_command(self.commanded_joints, move_time_ms=self.move_time_ms)

    def _apply_servo_inversion(self, index: int) -> None:
        base_config = self._base_servo_configs.get(index)
        if base_config is None:
            return

        if self.servo_inversions[index]:
            self.servo_configs[index] = ServoConfig(
                min_angle=base_config.min_angle,
                max_angle=base_config.max_angle,
                min_pulse=base_config.max_pulse,
                max_pulse=base_config.min_pulse,
            )
        else:
            self.servo_configs[index] = base_config
        self._update_limit_box_display(index)
        self._update_pulse_slider_range(index)
        self._refresh_pulse_slider_limits(index)

    def _update_limit_box_display(self, index: int) -> None:
        if not hasattr(self, "servo_limit_boxes_min"):
            return
        if index >= len(self.servo_limit_boxes_min):
            return
        base_config = self._base_servo_configs.get(index)
        if base_config is None:
            return
        min_box = self.servo_limit_boxes_min[index]
        max_box = self.servo_limit_boxes_max[index]
        try:
            min_box.eventson = False
            max_box.eventson = False
        except AttributeError:  # pragma: no cover - depends on Matplotlib
            pass
        soft_limits = self._soft_servo_limits.get(
            index, (base_config.min_angle, base_config.max_angle)
        )
        min_box.set_val(f"{math.degrees(soft_limits[0]):.1f}")
        max_box.set_val(f"{math.degrees(soft_limits[1]):.1f}")
        try:
            min_box.eventson = True
            max_box.eventson = True
        except AttributeError:  # pragma: no cover - depends on Matplotlib
            pass

    def _update_pulse_slider_display(self, index: int) -> None:
        if not hasattr(self, "servo_pulse_sliders"):
            return
        if index >= len(self.servo_pulse_sliders):
            return
        slider = self.servo_pulse_sliders[index]
        config = self.servo_configs.get(index)
        if slider is None or config is None:
            return
        if not self.commanded_joints:
            return
        if index >= len(self.commanded_joints):
            return
        pulse = config.angle_to_pulse(self.commanded_joints[index])
        low = min(slider.valmin, slider.valmax)
        high = max(slider.valmin, slider.valmax)
        try:
            slider.eventson = False
        except AttributeError:  # pragma: no cover - Matplotlib implementation detail
            pass
        try:
            slider.set_val(float(np.clip(pulse, low, high)))
        finally:
            try:
                slider.eventson = True
            except AttributeError:  # pragma: no cover - Matplotlib implementation detail
                pass

    def _update_pulse_slider_range(self, index: int) -> None:
        if not hasattr(self, "servo_pulse_sliders"):
            return
        if index >= len(self.servo_pulse_sliders):
            return
        slider = self.servo_pulse_sliders[index]
        config = self.servo_configs.get(index)
        if slider is None or config is None:
            return
        slider.valmin = min(config.min_pulse, config.max_pulse)
        slider.valmax = max(config.min_pulse, config.max_pulse)
        slider.ax.set_xlim(slider.valmin, slider.valmax)
        self._update_pulse_slider_display(index)
        self._refresh_pulse_slider_limits(index)

    def _refresh_pulse_slider_limits(self, index: int) -> None:
        if not hasattr(self, "servo_pulse_sliders"):
            return
        if index >= len(self.servo_pulse_sliders):
            return
        slider = self.servo_pulse_sliders[index]
        config = self.servo_configs.get(index)
        if slider is None or config is None:
            return

        lines = self._pulse_slider_limit_lines.setdefault(index, {})

        def _get_line(key: str, **line_kwargs) -> Line2D:
            line = lines.get(key)
            if line is None:
                line = slider.ax.axvline(**line_kwargs)
                lines[key] = line
            else:
                for attr, value in line_kwargs.items():
                    setter = getattr(line, f"set_{attr}", None)
                    if callable(setter):
                        setter(value)
            line.set_visible(True)
            return line

        motor_min = config.min_pulse
        motor_max = config.max_pulse
        _get_line(
            "motor_min",
            x=motor_min,
            color="#8d99ae",
            linestyle="-",
            linewidth=1.2,
            alpha=0.8,
        )
        _get_line(
            "motor_max",
            x=motor_max,
            color="#8d99ae",
            linestyle="-",
            linewidth=1.2,
            alpha=0.8,
        )

        soft_limits = self._soft_servo_limits.get(index)
        if soft_limits is None:
            return

        try:
            soft_min_pulse = config.angle_to_pulse(soft_limits[0])
            soft_max_pulse = config.angle_to_pulse(soft_limits[1])
        except ValueError:
            return

        _get_line(
            "soft_min",
            x=soft_min_pulse,
            color="#ef476f",
            linestyle="--",
            linewidth=1.1,
            alpha=0.9,
        )
        _get_line(
            "soft_max",
            x=soft_max_pulse,
            color="#ef476f",
            linestyle="--",
            linewidth=1.1,
            alpha=0.9,
        )

    def _active_angle_limits(
        self, index: int, *, use_soft_limits: bool = True
    ) -> tuple[float, float]:
        config = self.servo_configs.get(index)
        if config is None:
            return (-math.inf, math.inf)

        min_angle = config.min_angle
        max_angle = config.max_angle

        if use_soft_limits:
            soft_limits = self._soft_servo_limits.get(index)
            if soft_limits is not None:
                min_angle = max(min_angle, soft_limits[0])
                max_angle = min(max_angle, soft_limits[1])
                if min_angle > max_angle:
                    min_angle, max_angle = config.min_angle, config.max_angle

        return (min_angle, max_angle)

    def _clamp_joint_list(
        self,
        joints: list[float] | tuple[float, ...],
        *,
        use_soft_limits: bool = True,
    ) -> list[float]:
        clamped = list(joints)
        for idx, angle in enumerate(clamped):
            limits = self._active_angle_limits(idx, use_soft_limits=use_soft_limits)
            clamped[idx] = float(np.clip(angle, *limits))
        return clamped

    def _clamp_target(self, target: np.ndarray | list[float]) -> np.ndarray:
        result = np.array(target, dtype=float)
        x_limits = self.workspace_limits.get("x", (-0.3, 0.3))
        y_limits = self.workspace_limits.get("y", (-0.3, 0.3))
        z_limits = self.workspace_limits.get("z", (0.0, 0.4))
        result[0] = float(np.clip(result[0], *x_limits))
        result[1] = float(np.clip(result[1], *y_limits))
        result[2] = float(np.clip(result[2], *z_limits))
        return result

    def _apply_offsets(
        self, joints: list[float] | tuple[float, ...], *, direction: str
    ) -> list[float]:
        result: list[float] = []
        for idx, angle in enumerate(joints):
            offset = self.servo_offsets.get(idx, 0.0)
            if direction == "correct":
                result.append(angle + offset)
            elif direction == "raw":
                result.append(angle - offset)
            else:  # pragma: no cover - defensive
                raise ValueError(f"Unknown offset direction {direction}")
        return result

    def _load_calibration_data(self) -> dict[int, float]:
        path = self._calibration_path
        if not path.exists():
            return {}
        try:
            data = json.loads(path.read_text())
        except Exception:  # pragma: no cover - configuration robustness
            _LOGGER.warning(
                "Failed to load calibration data from %s", path, exc_info=True
            )
            return {}

        offsets_raw: dict[str, object] | None = None
        vertical_raw: dict[str, object] | None = None
        inversion_raw: dict[str, object] | None = None
        workspace_raw: dict[str, object] | None = None
        limits_raw: dict[str, object] | None = None

        if isinstance(data, dict):
            if "offsets" in data or "vertical_angles" in data:
                offsets_candidate = data.get("offsets")
                if isinstance(offsets_candidate, dict):
                    offsets_raw = offsets_candidate
                vertical_candidate = data.get("vertical_angles")
                if isinstance(vertical_candidate, dict):
                    vertical_raw = vertical_candidate
                inversion_candidate = data.get("inverted")
                if isinstance(inversion_candidate, dict):
                    inversion_raw = inversion_candidate
                workspace_candidate = data.get("workspace")
                if isinstance(workspace_candidate, dict):
                    workspace_raw = workspace_candidate
            else:
                offsets_raw = data

            limits_candidate = data.get("servo_limits")
            if isinstance(limits_candidate, dict):
                limits_raw = limits_candidate

        offsets: dict[int, float] = {}
        if offsets_raw:
            for key, value in offsets_raw.items():
                try:
                    index = int(key)
                    offsets[index] = float(value)
                except (TypeError, ValueError):
                    _LOGGER.warning("Ignoring invalid servo offset entry for %s", key)

        if vertical_raw:
            for key, value in vertical_raw.items():
                try:
                    index = int(key)
                    self.zero_reference[index] = float(value)
                except (TypeError, ValueError):
                    _LOGGER.warning("Ignoring invalid zero reference entry for %s", key)

        if inversion_raw:
            for key, value in inversion_raw.items():
                try:
                    index = int(key)
                except (TypeError, ValueError):
                    _LOGGER.warning("Ignoring invalid inversion entry for %s", key)
                    continue
                if index >= len(self.servo_inversions):
                    continue
                self.servo_inversions[index] = bool(value)

        if workspace_raw:
            loaded_workspace: dict[str, tuple[float, float]] = {}
            for axis, bounds in workspace_raw.items():
                if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                    continue
                try:
                    lower = float(bounds[0])
                    upper = float(bounds[1])
                except (TypeError, ValueError):
                    continue
                if lower > upper:
                    lower, upper = upper, lower
                loaded_workspace[axis] = (lower, upper)
            if loaded_workspace:
                self._loaded_workspace.update(loaded_workspace)

        if limits_raw:
            parsed_limits: dict[int, tuple[float, float]] = {}
            for key, payload in limits_raw.items():
                if not isinstance(payload, dict):
                    continue
                try:
                    index = int(key)
                except (TypeError, ValueError):
                    continue
                try:
                    min_deg = float(payload.get("min_deg"))
                    max_deg = float(payload.get("max_deg"))
                except (TypeError, ValueError):
                    continue
                if min_deg >= max_deg:
                    continue
                parsed_limits[index] = (
                    math.radians(min_deg),
                    math.radians(max_deg),
                )
            if parsed_limits:
                self._loaded_servo_limits.update(parsed_limits)

        return offsets

    def _save_calibration_data(self) -> None:
        path = self._calibration_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            servo_limits = {
                str(idx): {
                    "min_deg": math.degrees(limits[0]),
                    "max_deg": math.degrees(limits[1]),
                }
                for idx, limits in self._soft_servo_limits.items()
            }
            offsets_serialised = {
                str(idx): offset for idx, offset in self.servo_offsets.items()
            }
            vertical_serialised = {
                str(idx): angle for idx, angle in self.zero_reference.items()
            }
            serialisable = dict(offsets_serialised)
            serialisable.update(
                {
                    "offsets": dict(offsets_serialised),
                    "vertical_angles": dict(vertical_serialised),
                    "inverted": {
                        str(idx): state for idx, state in enumerate(self.servo_inversions)
                    },
                    "servo_limits": servo_limits,
                    "workspace": {
                        axis: [bounds[0], bounds[1]]
                        for axis, bounds in self.workspace_limits.items()
                    },
                }
            )
            path.write_text(json.dumps(serialisable, indent=2, sort_keys=True))
        except Exception:  # pragma: no cover - configuration robustness
            _LOGGER.warning(
                "Failed to persist calibration data to %s", path, exc_info=True
            )

    def _load_workspace_limits_from_config(self) -> None:
        if not self._loaded_workspace:
            return
        for axis, bounds in self._loaded_workspace.items():
            if len(bounds) != 2:
                continue
            lower, upper = bounds
            if lower >= upper:
                continue
            if axis in self.workspace_limits:
                self.workspace_limits[axis] = (lower, upper)
            else:
                self.workspace_limits[axis] = (lower, upper)
        if hasattr(self, "target"):
            self.target[:] = self._clamp_target(self.target)
        if hasattr(self, "ax"):
            self._recompute_camera_framing()

    def _apply_loaded_servo_limits(self) -> None:
        if not self._loaded_servo_limits:
            return
        for idx, (min_angle, max_angle) in self._loaded_servo_limits.items():
            if min_angle >= max_angle:
                continue
            self._soft_servo_limits[idx] = (min_angle, max_angle)
            self._update_limit_box_display(idx)
            self._refresh_pulse_slider_limits(idx)

    def _update_calibration_button_visual(self) -> None:
        if self._calibration_button is None:
            return
        if self._calibration_active:
            self._calibration_button.color = "#add8e6"
            self._calibration_button.hovercolor = "#bde0fe"
            self._calibration_button.label.set_text("Calibrate✓")
        else:
            self._calibration_button.color = "0.85"
            self._calibration_button.hovercolor = "0.95"
            self._calibration_button.label.set_text("Calibrate")
        self._calibration_button.ax.set_facecolor(self._calibration_button.color)

        if self._set_vertical_button is not None:
            self._set_vertical_button.eventson = self._calibration_active
            if self._calibration_active:
                self._set_vertical_button.color = "#ffe29a"
                self._set_vertical_button.hovercolor = "#ffe7b8"
            else:
                self._set_vertical_button.color = "0.85"
                self._set_vertical_button.hovercolor = "0.95"
            self._set_vertical_button.ax.set_facecolor(self._set_vertical_button.color)

        self.figure.canvas.draw_idle()

    def _toggle_calibration(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if self._calibration_active:
            self._exit_calibration_mode()
        else:
            self._enter_calibration_mode()

    def _toggle_raw_angle_display(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._show_raw_angles = not self._show_raw_angles
        self._update_raw_angle_button_visual()
        self._update_servo_readouts()

    def _handle_home_button(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._run_home_sequence()

    def _handle_duration_submit(self, text: str) -> None:  # pragma: no cover - UI interaction
        if self._updating_duration_box:
            return
        try:
            value = float(text)
        except ValueError:
            _LOGGER.warning("Ignoring invalid waypoint duration entry: %s", text)
            self._update_waypoint_duration_box()
            return
        value = max(0.1, value)
        if (
            self._selected_waypoint_index is None
            or self._selected_waypoint_index >= len(self.waypoints)
        ):
            return
        self.waypoints[self._selected_waypoint_index].duration = value
        self._refresh_waypoint_display()
        self._update_waypoint_duration_box()
        self._save_waypoints()

    def _handle_add_waypoint(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if self._waypoint_duration_box is None:
            duration = 2.0
        else:
            try:
                duration = float(self._waypoint_duration_box.text)
            except ValueError:
                duration = 2.0
        duration = max(0.1, duration)
        position = self._clamp_target(np.array(self.target))
        self.waypoints.append(
            Waypoint(
                position=position.copy(),
                duration=duration,
                wrist_pitch=self.wrist_pitch,
            )
        )
        self._selected_waypoint_index = len(self.waypoints) - 1
        self._refresh_waypoint_display()
        self._update_waypoint_duration_box()
        self._save_waypoints()

    def _handle_clear_waypoints(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._stop_waypoint_playback()
        self.waypoints.clear()
        self._selected_waypoint_index = None
        self._refresh_waypoint_display()
        self._update_waypoint_duration_box()
        self._save_waypoints()

    def _handle_save_subroutine(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if not self.waypoints:
            _LOGGER.info("No waypoints available to save as a subroutine")
            return
        name = ""
        if self._subroutine_name_box is not None:
            name = self._subroutine_name_box.text
        if not name.strip():
            name = "Subroutine"
        slug = self._slugify_subroutine_name(name)
        serialised = self._serialise_waypoints(self.waypoints)
        data = {"name": name, "slug": slug, "waypoints": serialised}
        try:
            SUBROUTINE_STORAGE_DIR.mkdir(parents=True, exist_ok=True)
            path = SUBROUTINE_STORAGE_DIR / f"{slug}.json"
            path.write_text(json.dumps(data, indent=2))
        except Exception:
            _LOGGER.warning("Failed to save subroutine %s", name, exc_info=True)
            return
        self._selected_subroutine_slug = slug
        self._load_subroutine_catalog()
        self._update_subroutine_name_box()
        self._highlight_selected_subroutine()
        self._update_timeline_entries_for_slug(slug)

    def _handle_load_subroutine(self, _event=None) -> None:  # pragma: no cover - UI interaction
        slug = self._selected_subroutine_slug
        name_text = self._subroutine_name_box.text if self._subroutine_name_box else ""
        if not slug and name_text.strip():
            slug = self._slugify_subroutine_name(name_text)
        if not slug:
            _LOGGER.info("Select or name a subroutine before loading")
            return
        waypoints = self._load_subroutine_waypoints(slug)
        if not waypoints:
            _LOGGER.warning("No subroutine data found for %s", slug)
            return
        self.waypoints = list(waypoints)
        self._selected_waypoint_index = None
        self._refresh_waypoint_display()
        self._update_waypoint_duration_box()
        self._selected_subroutine_slug = slug
        self._highlight_selected_subroutine()
        self._update_subroutine_name_box()

    def _handle_refresh_subroutines(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._load_subroutine_catalog()

    def _handle_add_to_timeline(self, _event=None) -> None:  # pragma: no cover - UI interaction
        slug = self._selected_subroutine_slug
        name_text = self._subroutine_name_box.text if self._subroutine_name_box else ""
        if not slug and name_text.strip():
            slug = self._slugify_subroutine_name(name_text)
        if not slug:
            _LOGGER.info("Select or name a subroutine to schedule")
            return
        summary = self._subroutine_catalog.get(slug)
        if summary is None:
            _LOGGER.warning("Unknown subroutine '%s'", name_text or slug)
            return
        entry = TimelineEntry(
            name=summary.name,
            slug=summary.slug,
            duration=summary.duration,
            missing=False,
        )
        self._timeline_entries.append(entry)
        self._selected_timeline_index = len(self._timeline_entries) - 1
        self._refresh_timeline_display()
        self._highlight_selected_timeline()
        self._save_timeline()

    def _handle_remove_timeline_entry(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if self._selected_timeline_index is None:
            _LOGGER.info("Select a timeline entry to remove")
            return
        if not (0 <= self._selected_timeline_index < len(self._timeline_entries)):
            return
        self._timeline_entries.pop(self._selected_timeline_index)
        if self._selected_timeline_index >= len(self._timeline_entries):
            self._selected_timeline_index = len(self._timeline_entries) - 1
        if self._selected_timeline_index < 0:
            self._selected_timeline_index = None
        self._refresh_timeline_display()
        self._highlight_selected_timeline()
        self._save_timeline()

    def _handle_clear_timeline(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._stop_timeline_playback()
        self._timeline_entries.clear()
        self._selected_timeline_index = None
        self._refresh_timeline_display()
        self._highlight_selected_timeline()
        self._save_timeline()

    def _handle_play_timeline(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if self._timeline_playback_thread and self._timeline_playback_thread.is_alive():
            self._stop_timeline_playback()
            return
        valid_entries = [entry for entry in self._timeline_entries if not entry.missing]
        if not valid_entries:
            _LOGGER.info("No playable timeline entries; add subroutines first")
            return
        self._stop_waypoint_playback()
        self._timeline_stop_event.clear()
        self._timeline_playback_thread = threading.Thread(
            target=self._timeline_playback_worker,
            name="al5a-timeline-playback",
            daemon=True,
        )
        self._timeline_playback_thread.start()
        self._update_timeline_play_button(running=True)

    def _handle_toggle_timeline_repeat(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._timeline_repeat = not self._timeline_repeat
        self._update_timeline_repeat_button()

    def _handle_timeline_speed_change(self, value: float) -> None:
        self._timeline_speed = float(np.clip(value, 0.1, 5.0))

    def _handle_play_waypoints(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if self._waypoint_playback_thread and self._waypoint_playback_thread.is_alive():
            self._stop_waypoint_playback()
            return
        if not self.waypoints:
            _LOGGER.info("No waypoints queued; add at least one before playback")
            return
        self._waypoint_stop_event.clear()
        self._waypoint_playback_thread = threading.Thread(
            target=self._waypoint_playback_worker,
            name="al5a-waypoint-playback",
            daemon=True,
        )
        self._waypoint_playback_thread.start()
        self._update_play_button_label(running=True)

    def _update_play_button_label(self, *, running: bool) -> None:
        button = getattr(self, "_play_waypoints_button", None)
        if button is None:
            return
        label = "Stop" if running else "Play path"
        button.label.set_text(label)
        button.ax.figure.canvas.draw_idle()

    def _stop_waypoint_playback(self) -> None:
        if (
            self._waypoint_playback_thread
            and self._waypoint_playback_thread.is_alive()
        ):
            self._waypoint_stop_event.set()
            self._waypoint_playback_thread.join(timeout=1.0)
        self._waypoint_playback_thread = None
        self._waypoint_stop_event.clear()
        self._update_play_button_label(running=False)

    def _waypoint_playback_worker(self) -> None:
        try:
            self._execute_waypoint_sequence(
                list(self.waypoints),
                stop_event=self._waypoint_stop_event,
                selection_callback=self._handle_waypoint_playback_step,
            )
        finally:
            self._update_play_button_label(running=False)
            self._waypoint_stop_event.clear()
            self._waypoint_playback_thread = None

    def _handle_waypoint_playback_step(self, index: int, _waypoint: Waypoint) -> None:
        self._selected_waypoint_index = index
        self._highlight_selected_waypoint()
        self._update_waypoint_duration_box()

    def _update_timeline_play_button(self, *, running: bool) -> None:
        button = self._play_timeline_button
        if button is None:
            return
        label = "Stop" if running else "Play timeline"
        button.label.set_text(label)
        button.ax.figure.canvas.draw_idle()

    def _update_timeline_repeat_button(self) -> None:
        button = self._timeline_repeat_button
        if button is None:
            return
        label = "Repeat: On" if self._timeline_repeat else "Repeat: Off"
        facecolor = "#cfe8fc" if self._timeline_repeat else "0.85"
        button.label.set_text(label)
        button.ax.set_facecolor(facecolor)
        button.ax.figure.canvas.draw_idle()

    def _stop_timeline_playback(self) -> None:
        if self._timeline_playback_thread and self._timeline_playback_thread.is_alive():
            self._timeline_stop_event.set()
            self._timeline_playback_thread.join(timeout=1.0)
        self._timeline_playback_thread = None
        self._timeline_stop_event.clear()
        self._update_timeline_play_button(running=False)

    def _timeline_playback_worker(self) -> None:
        try:
            while not self._timeline_stop_event.is_set():
                for index, entry in enumerate(list(self._timeline_entries)):
                    if self._timeline_stop_event.is_set():
                        break
                    if entry.missing:
                        continue
                    waypoints = self._load_subroutine_waypoints(entry.slug)
                    if not waypoints:
                        _LOGGER.warning(
                            "Skipping timeline entry %s; subroutine file missing", entry.name
                        )
                        continue
                    self._selected_timeline_index = index
                    self._highlight_selected_timeline()
                    self._selected_subroutine_slug = entry.slug
                    self._highlight_selected_subroutine()
                    self._update_subroutine_name_box()
                    self._execute_waypoint_sequence(
                        waypoints,
                        stop_event=self._timeline_stop_event,
                        selection_callback=None,
                        playback_speed=self._timeline_speed,
                    )
                    if self._timeline_stop_event.is_set():
                        break
                if not self._timeline_repeat:
                    break
        finally:
            self._timeline_stop_event.clear()
            self._timeline_playback_thread = None
            self._update_timeline_play_button(running=False)

    def _execute_waypoint_sequence(
        self,
        waypoints: list[Waypoint],
        *,
        stop_event: threading.Event,
        selection_callback: Callable[[int, Waypoint], None] | None = None,
        playback_speed: float = 1.0,
    ) -> None:
        for index, waypoint in enumerate(waypoints):
            if stop_event.is_set():
                break
            target = self._clamp_target(np.array(waypoint.position, dtype=float))
            scaled_duration = waypoint.duration / max(playback_speed, 0.1)
            duration_ms = int(max(0.02, scaled_duration) * 1000)
            desired_pitch = float(np.clip(waypoint.wrist_pitch, *self._wrist_pitch_limits))
            self._set_wrist_pitch_target(desired_pitch, update_slider=False)
            try:
                joints = list(self.kin.inverse(target[[0, 1, 2]], desired_pitch))
            except Exception:
                _LOGGER.exception("Failed to solve IK for scheduled waypoint %d", index + 1)
                continue
            self._last_wrist_pitch = joints[1] + joints[2] + joints[3]
            full_joints = joints + [self.wrist_rotation, self.gripper_angle]
            full_joints = self._clamp_joint_list(full_joints)
            self.target[:] = target
            self._send_move_command(
                full_joints,
                move_time_ms=duration_ms,
                soft_start=True,
                replace=False,
            )
            self._command_queue.join()
            if stop_event.is_set():
                break
            if selection_callback is not None:
                try:
                    selection_callback(index, waypoint)
                except Exception:
                    pass

    def _handle_subroutine_press(self, event) -> bool:
        if self._subroutine_list_ax is None or event.inaxes != self._subroutine_list_ax:
            return False
        if event.xdata is None or event.ydata is None:
            return True
        handled = False
        for slug, patch in zip(self._subroutine_display_slugs, self._subroutine_patches):
            contains, _ = patch.contains(event)
            if contains:
                self._selected_subroutine_slug = slug
                self._highlight_selected_subroutine()
                self._update_subroutine_name_box()
                handled = True
                break
        if not handled:
            self._selected_subroutine_slug = None
            self._highlight_selected_subroutine()
            self._update_subroutine_name_box()
        return True

    def _handle_timeline_press(self, event) -> bool:
        if self._timeline_ax is None or event.inaxes != self._timeline_ax:
            return False
        if event.xdata is None or event.ydata is None:
            return True
        handled = False
        for idx, patch in enumerate(self._timeline_patches):
            contains, _ = patch.contains(event)
            if contains:
                self._timeline_drag = TimelineDragState(index=idx, offset=event.ydata)
                self._selected_timeline_index = idx
                self._timeline_reordered = False
                if 0 <= idx < len(self._timeline_entries):
                    self._selected_subroutine_slug = self._timeline_entries[idx].slug
                self._highlight_selected_timeline()
                self._highlight_selected_subroutine()
                self._update_subroutine_name_box()
                handled = True
                break
        if not handled:
            self._timeline_drag = TimelineDragState()
            self._selected_timeline_index = None
            self._highlight_selected_timeline()
        return True

    def _handle_waypoint_press(self, event) -> bool:
        waypoint_ax = getattr(self, "waypoint_ax", None)
        if waypoint_ax is None or event.inaxes != waypoint_ax:
            return False
        if event.xdata is None or event.ydata is None:
            return True
        handled = False
        for idx, patch in enumerate(self._waypoint_patches):
            contains, _ = patch.contains(event)
            if contains:
                self._waypoint_drag = WaypointDragState(index=idx, offset=event.ydata)
                self._selected_waypoint_index = idx
                self._waypoint_reordered = False
                if 0 <= idx < len(self.waypoints):
                    self._set_wrist_pitch_target(self.waypoints[idx].wrist_pitch)
                self._highlight_selected_waypoint()
                self._update_waypoint_duration_box()
                handled = True
                break
        if not handled:
            self._waypoint_drag = WaypointDragState()
            self._selected_waypoint_index = None
            self._refresh_waypoint_display()
            self._update_waypoint_duration_box()
        return True

    def _handle_timeline_motion(self, event) -> bool:
        if self._timeline_drag.index is None:
            return False
        if self._timeline_ax is None or event.inaxes != self._timeline_ax:
            return True
        if event.ydata is None or not self._timeline_entries:
            return True
        centers = [
            patch.get_y() + self._timeline_item_height / 2 for patch in self._timeline_patches
        ]
        if not centers:
            return True
        distances = [abs(event.ydata - center) for center in centers]
        new_index = int(min(range(len(distances)), key=distances.__getitem__))
        old_index = self._timeline_drag.index
        if new_index != old_index:
            entry = self._timeline_entries.pop(old_index)
            self._timeline_entries.insert(new_index, entry)
            self._timeline_drag.index = new_index
            self._selected_timeline_index = new_index
            self._refresh_timeline_display()
            self._timeline_reordered = True
        return True

    def _handle_waypoint_motion(self, event) -> bool:
        if self._waypoint_drag.index is None:
            return False
        waypoint_ax = getattr(self, "waypoint_ax", None)
        if waypoint_ax is None or event.inaxes != waypoint_ax:
            return True
        if event.ydata is None or not self.waypoints:
            return True
        centers = [
            patch.get_y() + self._waypoint_item_height / 2 for patch in self._waypoint_patches
        ]
        if not centers:
            return True
        distances = [abs(event.ydata - center) for center in centers]
        new_index = int(min(range(len(distances)), key=distances.__getitem__))
        old_index = self._waypoint_drag.index
        if new_index != old_index:
            waypoint = self.waypoints.pop(old_index)
            self.waypoints.insert(new_index, waypoint)
            self._waypoint_drag.index = new_index
            self._selected_waypoint_index = new_index
            self._refresh_waypoint_display()
            self._update_waypoint_duration_box()
            self._waypoint_reordered = True
        return True

    def _handle_timeline_release(self, event) -> bool:
        if self._timeline_drag.index is None:
            return False
        self._timeline_drag = TimelineDragState()
        if self._timeline_reordered:
            self._timeline_reordered = False
            self._save_timeline()
        return True

    def _handle_waypoint_release(self, event) -> bool:
        if self._waypoint_drag.index is None:
            return False
        self._waypoint_drag = WaypointDragState()
        if self._waypoint_reordered:
            self._save_waypoints()
        self._waypoint_reordered = False
        return True

    def _enter_calibration_mode(self) -> None:
        if self._calibration_active:
            return
        self._calibration_active = True
        relax = getattr(self.controller, "relax_servos", None)
        should_relax = getattr(self.controller, "AUTO_RELAX_ON_CALIBRATION", False)
        if callable(relax) and should_relax:
            try:
                relax()
            except Exception:  # pragma: no cover - runtime safety net
                _LOGGER.warning("Failed to relax servos for calibration", exc_info=True)
        commands_buffer = getattr(self._command_queue, "commands", None)
        if isinstance(commands_buffer, list):
            commands_buffer.clear()
        else:
            while True:
                try:
                    self._command_queue.get_nowait()
                except queue.Empty:
                    break
                else:  # pragma: no cover - runtime safety net
                    try:
                        self._command_queue.task_done()
                    except Exception:
                        pass
        if self._calibration_timer is not None:
            try:
                self._calibration_timer.cancel()
            except Exception:  # pragma: no cover - depends on timer implementation
                pass
            self._calibration_timer = None
        self._update_calibration_button_visual()

    def _exit_calibration_mode(self) -> None:
        if not self._calibration_active:
            return
        self._calibration_active = False
        self._update_calibration_button_visual()
    def _handle_set_vertical(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if not self._calibration_active:
            return
        source = self.feedback_joints or self.current_joints
        if not source:
            return
        desired_pose = [0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0]
        desired: list[float] = []
        for idx in range(len(source)):
            if idx < len(desired_pose):
                desired.append(desired_pose[idx])
            else:
                desired.append(0.0)

        updated_feedback: list[float] = []
        base_override_offset: float | None = None
        base_index = 0
        base_config = self.servo_configs.get(base_index)
        if base_index < len(desired) and base_config is not None:
            # The base joint is treated specially during vertical calibration so
            # that a zero command maps to the midpoint between the configured
            # minimum and maximum pulse widths. This only applies when the
            # limits are symmetric around zero; asymmetric limits should retain
            # their configured extremes to avoid shrinking the usable workspace.
            base_mid_angle = base_config.min_angle + (
                (base_config.max_angle - base_config.min_angle) / 2.0
            )
            limits_are_symmetric = math.isclose(
                abs(base_config.min_angle),
                abs(base_config.max_angle),
                rel_tol=1e-6,
                abs_tol=1e-6,
            )
            if limits_are_symmetric:
                base_override_offset = desired[base_index] - base_mid_angle
        for idx, actual in enumerate(source):
            target_angle = desired[idx]
            if idx == base_index and base_override_offset is not None:
                self.servo_offsets[idx] = base_override_offset
                updated_feedback.append(target_angle)
                continue
            delta = target_angle - actual
            self.servo_offsets[idx] = self.servo_offsets.get(idx, 0.0) + delta
            updated_feedback.append(actual + delta)

        for idx, angle in enumerate(updated_feedback):
            self.zero_reference[idx] = angle

        self._save_calibration_data()
        self.feedback_joints = list(updated_feedback)
        self.current_joints = list(updated_feedback)
        self.commanded_joints = list(updated_feedback)
        self._last_commanded_raw = tuple(
            self._apply_offsets(self.current_joints, direction="raw")
        )
        if len(self.current_joints) >= 5:
            self.wrist_rotation = self.current_joints[4]
        if len(self.current_joints) >= 6:
            self.gripper_angle = self.current_joints[5]
        if len(self.current_joints) >= 4:
            actual_pitch = (
                self.current_joints[1]
                + self.current_joints[2]
                + self.current_joints[3]
            )
            self._last_wrist_pitch = actual_pitch
            self._set_wrist_pitch_target(actual_pitch)
        self._update_servo_readouts()
        if len(self.current_joints) >= 4:
            self._update_visuals(self.current_joints)

    def _apply_feedback(self, corrected: list[float]) -> None:
        self.feedback_joints = list(corrected)
        for idx, angle in enumerate(corrected):
            if idx < len(self.current_joints):
                self.current_joints[idx] = angle
            else:
                self.current_joints.append(angle)

        if len(self.current_joints) >= 5:
            self.wrist_rotation = self.current_joints[4]
        if len(self.current_joints) >= 6:
            self.gripper_angle = self.current_joints[5]

        if len(self.current_joints) >= 4:
            try:
                pose = self.kin.forward(self.current_joints)
            except Exception:  # pragma: no cover - runtime safety net
                _LOGGER.warning(
                    "Failed to compute pose from controller feedback", exc_info=True
                )
            else:
                self.target[:3] = pose[:3, 3]
                actual_pitch = (
                    self.current_joints[1]
                    + self.current_joints[2]
                    + self.current_joints[3]
                )
                self._last_wrist_pitch = actual_pitch
                self._set_wrist_pitch_target(actual_pitch)
                self._update_visuals(self.current_joints[:4])

        self._update_servo_readouts()

    def _read_feedback_from_controller(self) -> list[float] | None:
        read_positions = getattr(self.controller, "read_positions", None)
        if not callable(read_positions):
            return None

        try:
            feedback = read_positions(
                servo_configs=self._get_servo_configs_for_controller(),
                servo_channels=DEFAULT_SERVO_CHANNELS,
            )
        except Exception:  # pragma: no cover - runtime safety net
            _LOGGER.warning(
                "Failed to obtain feedback from controller", exc_info=True
            )
            return None

        if not feedback:
            return None
        return list(feedback)

    def _update_inversion_button_visual(self, index: int) -> None:
        if index >= len(self.servo_invert_buttons):
            return

        button = self.servo_invert_buttons[index]
        inverted = self.servo_inversions[index]
        if inverted:
            button.color = self._invert_button_active_color
            button.hovercolor = "#b9f6b9"
            button.label.set_text("Inv✓")
        else:
            button.color = self._invert_button_inactive_color
            button.hovercolor = "0.95"
            button.label.set_text("Inv")
        button.ax.set_facecolor(button.color)
        self.figure.canvas.draw_idle()

    def _compute_link_positions(
        self, joints: Sequence[float]
    ) -> tuple[list[float], list[float], list[float]]:
        if len(joints) < 4:
            raise ValueError("Expected at least 4 joint angles for visualisation")

        shoulder = joints[1]
        elbow = joints[2]
        wrist = joints[3]

        base = joints[0]
        links = self.kin.links

        base_origin = np.array([0.0, 0.0, 0.0])
        shoulder_pivot = np.array([0.0, 0.0, links.base_height])

        base_cos = math.cos(base)
        base_sin = math.sin(base)

        shoulder_horizontal = links.shoulder * math.cos(shoulder)
        shoulder_vertical = links.shoulder * math.sin(shoulder)
        elbow_joint = shoulder_pivot + np.array(
            [
                base_cos * shoulder_horizontal,
                base_sin * shoulder_horizontal,
                shoulder_vertical,
            ]
        )

        elbow_angle = shoulder + elbow
        elbow_horizontal = links.elbow * math.cos(elbow_angle)
        elbow_vertical = links.elbow * math.sin(elbow_angle)
        wrist_joint = elbow_joint + np.array(
            [
                base_cos * elbow_horizontal,
                base_sin * elbow_horizontal,
                elbow_vertical,
            ]
        )

        wrist_angle = elbow_angle + wrist
        wrist_horizontal = links.wrist * math.cos(wrist_angle)
        wrist_vertical = links.wrist * math.sin(wrist_angle)
        tool_tip = wrist_joint + np.array(
            [
                base_cos * wrist_horizontal,
                base_sin * wrist_horizontal,
                wrist_vertical,
            ]
        )

        xs = [
            base_origin[0],
            shoulder_pivot[0],
            elbow_joint[0],
            wrist_joint[0],
            tool_tip[0],
        ]
        ys = [
            base_origin[1],
            shoulder_pivot[1],
            elbow_joint[1],
            wrist_joint[1],
            tool_tip[1],
        ]
        zs = [
            base_origin[2],
            shoulder_pivot[2],
            elbow_joint[2],
            wrist_joint[2],
            tool_tip[2],
        ]
        return xs, ys, zs

    def _update_visuals(self, joints: list[float]) -> None:
        if len(joints) < 4:
            self.base_line.set_data([], [])
            self.base_line.set_3d_properties([])
            self.setpoint_line.set_data([], [])
            self.setpoint_line.set_3d_properties([])
            return

        xs, ys, zs = self._compute_link_positions(joints)
        self.base_line.set_data(xs, ys)
        self.base_line.set_3d_properties(zs)

        if self._setpoint_joints:
            try:
                sp_xs, sp_ys, sp_zs = self._compute_link_positions(self._setpoint_joints)
            except Exception:
                self.setpoint_line.set_data([], [])
                self.setpoint_line.set_3d_properties([])
            else:
                self.setpoint_line.set_data(sp_xs, sp_ys)
                self.setpoint_line.set_3d_properties(sp_zs)
        else:
            self.setpoint_line.set_data([], [])
            self.setpoint_line.set_3d_properties([])

        self.target_artist._offsets3d = (
            [self.target[0]],
            [self.target[1]],
            [self.target[2]],
        )
        self._update_gizmo()
        actual_tip = (xs[-1], ys[-1], zs[-1])
        target_text = (
            f"Target: x={self.target[0]:.3f} m, y={self.target[1]:.3f} m, z={self.target[2]:.3f} m"
        )
        if self._setpoint_position is not None:
            target_text += (
                "\nSetpoint tip: "
                f"x={self._setpoint_position[0]:.3f} m, "
                f"y={self._setpoint_position[1]:.3f} m, "
                f"z={self._setpoint_position[2]:.3f} m"
            )
        target_text += (
            "\nActual tip: "
            f"x={actual_tip[0]:.3f} m, "
            f"y={actual_tip[1]:.3f} m, "
            f"z={actual_tip[2]:.3f} m"
        )
        target_text += (
            f"\nWrist pitch: {self._slider_value_from_pitch(self.wrist_pitch):.1f}°"
        )
        self.text.set_text(target_text)

    def _update_servo_readouts(self) -> None:
        if not self.servo_value_texts:
            return
        for idx, text in enumerate(self.servo_value_texts):
            name, model, location = self._SERVO_METADATA[idx]
            zero_angle = self.zero_reference.get(
                idx,
                _DEFAULT_VERTICAL_JOINTS[idx]
                if idx < len(_DEFAULT_VERTICAL_JOINTS)
                else 0.0,
            )
            zero_deg = math.degrees(zero_angle)
            commanded_raw_deg = math.degrees(self.commanded_joints[idx])
            actual_source = (
                self.feedback_joints
                if self.feedback_joints and idx < len(self.feedback_joints)
                else self.current_joints
            )
            actual_raw_deg = math.degrees(actual_source[idx])
            commanded_deg = commanded_raw_deg - zero_deg
            actual_deg = actual_raw_deg - zero_deg
            inversion_note = " (inv)" if self.servo_inversions[idx] else ""
            soft_limits = self._soft_servo_limits.get(idx)
            config = self._base_servo_configs.get(idx)
            limits_text = ""
            if soft_limits is not None:
                limits_text = (
                    f"Limits: {math.degrees(soft_limits[0]):.0f}° to "
                    f"{math.degrees(soft_limits[1]):.0f}°"
                )
            elif config is not None:
                limits_text = (
                    f"Limits: {math.degrees(config.min_angle):.0f}° to "
                    f"{math.degrees(config.max_angle):.0f}°"
                )
            commanded_raw_suffix = (
                f" (raw {commanded_raw_deg:.1f}°)" if self._show_raw_angles else ""
            )
            actual_raw_suffix = (
                f" (raw {actual_raw_deg:.1f}°)" if self._show_raw_angles else ""
            )
            text.set_text(
                f"{name} ({model})\n{location}\nCmd: {commanded_deg:.1f}°"
                f"{commanded_raw_suffix} | Actual: {actual_deg:.1f}°"
                f"{actual_raw_suffix}{inversion_note}\n{limits_text}"
            )
            self._update_pulse_slider_display(idx)
        self.figure.canvas.draw_idle()

    def _update_servo_limit(
        self,
        index: int,
        *,
        min_angle: float | None = None,
        max_angle: float | None = None,
    ) -> None:
        """Apply soft safety limits without altering the pulse calibration."""
        base_config = self._base_servo_configs.get(index)
        if base_config is None:
            return

        new_min = base_config.min_angle if min_angle is None else min_angle
        new_max = base_config.max_angle if max_angle is None else max_angle
        if new_min >= new_max:
            name, model, _ = self._SERVO_METADATA[index]
            _LOGGER.warning(
                "Ignored invalid limit update for %s (%s): min %.1f° >= max %.1f°",
                name,
                model,
                math.degrees(new_min),
                math.degrees(new_max),
            )
            self._update_limit_box_display(index)
            return

        self._soft_servo_limits[index] = (new_min, new_max)
        self._refresh_pulse_slider_limits(index)

        # Keep the loaded limits cache in sync so future persistence uses the
        # most recent values. This is particularly important when the limits
        # originated from a configuration file because users expect their
        # adjustments to overwrite the stored values instead of reverting on
        # restart.
        self._loaded_servo_limits[index] = (new_min, new_max)

        # Persist the calibration data immediately so the adjustments are not
        # lost if the application exits before another save opportunity.
        self._save_calibration_data()

        def _clamp_list(values: list[float] | None) -> None:
            if values is None:
                return
            if index >= len(values):
                return
            limits = self._active_angle_limits(index)
            values[index] = float(np.clip(values[index], *limits))

        _clamp_list(self.current_joints)
        _clamp_list(self.commanded_joints)
        self._last_commanded_raw = tuple(
            self._apply_offsets(self.commanded_joints, direction="raw")
        )
        self._update_servo_readouts()
        self._update_limit_box_display(index)

    def _cancel_pending_commands(self) -> None:
        while True:
            try:
                queued = self._command_queue.get_nowait()
            except queue.Empty:
                break
            else:
                self._command_queue.task_done()

    def _store_setpoint(self, joints: Sequence[float]) -> None:
        if len(joints) < 4:
            self._setpoint_joints = None
            self._setpoint_position = None
            return

        self._setpoint_joints = list(joints[:4])
        try:
            pose = self.kin.forward(self._setpoint_joints)
        except Exception:
            self._setpoint_position = None
            return

        position = np.array(pose[:3, 3], dtype=float)
        self._setpoint_position = position
        self.target[:] = position

    def _command_worker(self) -> None:
        while True:
            joints_raw, move_time, soft_start = self._command_queue.get()
            try:
                interrupted = False
                aborted_for_calibration = False
                for segment_raw, segment_time in self._generate_smooth_segments(
                    self._last_commanded_raw, joints_raw, move_time, soft_start=soft_start
                ):
                    if self._calibration_active:
                        relax = getattr(self.controller, "relax_servos", None)
                        if callable(relax):
                            try:
                                relax()
                            except Exception:  # pragma: no cover - runtime safety net
                                _LOGGER.warning(
                                    "Failed to relax servos when calibration became active",
                                    exc_info=True,
                                )
                        aborted_for_calibration = True
                        break
                    corrected_segment = self._apply_offsets(
                        segment_raw, direction="correct"
                    )
                    self.commanded_joints = list(corrected_segment)
                    self.controller.move_joints(
                        segment_raw,
                        move_time_ms=segment_time,
                        servo_configs=self._get_servo_configs_for_controller(),
                    )
                    self._last_commanded_raw = tuple(segment_raw)
                    self._update_servo_readouts()
                    interrupted = False
                    if segment_time and segment_time > 0:
                        interrupted = self._wait_for_segment(segment_time / 1000.0)
                    if not interrupted:
                        self.current_joints = list(corrected_segment)
                        self._update_servo_readouts()
                        if len(self.current_joints) >= 4:
                            self._update_visuals(self.current_joints)
                    if interrupted:
                        break

                if interrupted:
                    continue

                if aborted_for_calibration:
                    self.feedback_joints = None
                    self.commanded_joints = list(self.current_joints)
                    self._last_commanded_raw = tuple(
                        self._apply_offsets(self.current_joints, direction="raw")
                    )
                    self._update_servo_readouts()
                    continue

                feedback_raw = self._read_feedback_from_controller()
                if feedback_raw:
                    corrected_feedback = self._apply_offsets(
                        feedback_raw, direction="correct"
                    )
                    self._apply_feedback(corrected_feedback)
                else:
                    self.feedback_joints = None
            except Exception:  # pragma: no cover - runtime safety net
                _LOGGER.exception("Failed to send move command to controller")
            finally:
                self._command_queue.task_done()

    def _send_move_command(
        self,
        joints: list[float] | tuple[float, ...],
        move_time_ms: int | None,
        *,
        soft_start: bool = True,
        replace: bool = True,
    ) -> None:
        adjusted_move_time = move_time_ms
        if self._initial_feedback_move_pending:
            if adjusted_move_time is None:
                adjusted_move_time = self._initial_feedback_move_time_ms
            else:
                adjusted_move_time = max(
                    adjusted_move_time, self._initial_feedback_move_time_ms
                )
            self._initial_feedback_move_pending = False

        if soft_start:
            if adjusted_move_time is None:
                adjusted_move_time = self._soft_start_min_time_ms
            else:
                adjusted_move_time = max(adjusted_move_time, self._soft_start_min_time_ms)

        self._store_setpoint(joints)
        self.commanded_joints = list(joints)
        if not self._calibration_active:
            self.feedback_joints = None
        self._update_visuals(self.current_joints)
        self._update_servo_readouts()

        if self._calibration_active:
            return

        if self._skip_next_command:
            self._skip_next_command = False
            return

        raw_command = tuple(self._apply_offsets(joints, direction="raw"))
        command = (raw_command, adjusted_move_time, soft_start)
        if replace:
            try:
                self._command_queue.put_nowait(command)
            except queue.Full:
                try:
                    self._command_queue.get_nowait()
                    self._command_queue.task_done()
                except queue.Empty:  # pragma: no cover - defensive
                    pass
                self._command_queue.put_nowait(command)
        else:
            self._command_queue.put(command)

    def _get_servo_configs_for_controller(self) -> dict[int, ServoConfig]:
        return dict(self.servo_configs)

    def _generate_smooth_segments(
        self,
        start: tuple[float, ...] | None,
        target: tuple[float, ...],
        move_time_ms: int | None,
        *,
        soft_start: bool = False,
    ) -> list[tuple[tuple[float, ...], int | None]]:
        if start is None or len(start) != len(target):
            return [(target, move_time_ms)]

        required_time_s = 0.0
        for idx, (start_angle, target_angle) in enumerate(zip(start, target)):
            speed = self._joint_max_speeds.get(idx, self._default_max_joint_speed)
            if speed <= 0:
                continue
            delta = abs(target_angle - start_angle)
            required_time_s = max(required_time_s, delta / speed)

        requested_time_s = 0.0 if move_time_ms is None else max(0.0, move_time_ms / 1000.0)
        total_time_s = max(requested_time_s, required_time_s)
        if soft_start:
            total_time_s = max(total_time_s, self._soft_start_min_time_ms / 1000.0)
        if total_time_s <= 0:
            return [(target, None)]

        move_time_ms = int(round(total_time_s * 1000))
        min_segments = self._min_smoothing_segments
        if soft_start:
            min_segments = max(min_segments, self._soft_start_min_segments)
        segments = max(
            min_segments,
            int(math.ceil(max(move_time_ms, self._smoothing_step_ms) / self._smoothing_step_ms)),
        )
        step_time = move_time_ms / segments
        result: list[tuple[tuple[float, ...], int | None]] = []
        accumulated = 0
        for step in range(1, segments + 1):
            t = step / segments
            eased = t * t * (3 - 2 * t)
            values = tuple(
                start[idx] + (target[idx] - start[idx]) * eased for idx in range(len(target))
            )
            if step < segments:
                segment_time = int(round(step_time))
                accumulated += segment_time
            else:
                segment_time = max(0, move_time_ms - accumulated)
            result.append((values, segment_time if segment_time > 0 else None))
        return result

    def _wait_for_segment(self, duration_s: float) -> bool:
        end_time = time.monotonic() + duration_s
        poll_interval = max(0.01, self._smoothing_step_ms / 1000.0)
        while True:
            if not self._command_queue.empty():
                return True
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return False
            time.sleep(min(remaining, poll_interval))


def run_demo(controller, move_time_ms: int = 1000) -> None:
    """Launch the Matplotlib-based demo interface."""

    InteractiveArm(controller, move_time_ms=move_time_ms)
    plt.legend()
    plt.show()


__all__ = ["InteractiveArm", "run_demo"]
