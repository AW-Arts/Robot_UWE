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
from matplotlib.patches import Circle, Rectangle, Wedge
from matplotlib.widgets import Button, Slider, TextBox
from mpl_toolkits.mplot3d import proj3d

from .al5a_kinematics import (
    AL5AKinematics,
    DEFAULT_SERVO_CHANNELS,
    DEFAULT_SERVO_CONFIGS,
    ServoConfig,
)
from .led_driver import build_drive_leds, load_led_config
from .status_leds import LEDState, RobotISOState, StatusLEDController


_LOGGER = logging.getLogger(__name__)


CALIBRATION_CONFIG_PATH = Path.home() / ".config" / "lynxmotion_al5a" / "servo_offsets.json"
PATH_STORAGE_PATH = CALIBRATION_CONFIG_PATH.with_name("saved_path.json")
SUBROUTINE_STORAGE_DIR = PATH_STORAGE_PATH.with_name("subroutines")
TIMELINE_STORAGE_PATH = PATH_STORAGE_PATH.with_name("timeline.json")
TEACH_SESSION_DIR = CALIBRATION_CONFIG_PATH.with_name("teach_sessions")


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
    wrist_rotation: float
    gripper_angle: float


@dataclass
class TeachSample:
    timestamp: float
    joints: tuple[float, ...]
    cartesian: tuple[float, float, float] | None
    wrist_pitch: float
    wrist_rotation: float
    gripper_angle: float
    move_time_ms: int | None
    dwell_ms: int | None = None
    label: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "timestamp_s": float(self.timestamp),
            "joints": list(self.joints),
            "cartesian": list(self.cartesian) if self.cartesian is not None else None,
            "wrist_pitch": float(self.wrist_pitch),
            "wrist_rotation": float(self.wrist_rotation),
            "gripper_angle": float(self.gripper_angle),
            "move_time_ms": self.move_time_ms,
            "dwell_ms": self.dwell_ms,
            "label": self.label,
        }


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
        self.servo_multipliers: dict[int, float] = {
            index: 1.0 for index in self._base_servo_configs
        }
        self.servo_inversions: list[bool] = [False] * len(self._SERVO_METADATA)
        self._invert_button_inactive_color = "0.85"
        self._invert_button_active_color = "#90ee90"
        self._operation_mode: str = "live"
        self._teach_session_start: float | None = None
        self._last_teach_timestamp: float | None = None
        self._teach_samples: list[TeachSample] = []
        self._smoothed_teach_samples: list[TeachSample] = []
        self._last_teach_save_path: Path | None = None
        self._last_teach_subroutine_path: Path | None = None
        self._last_teach_status: str = ""
        self._teach_recording: bool = False
        self._pre_teach_target_position: np.ndarray | None = None
        self._pre_teach_joints: list[float] | None = None
        self._pre_teach_full_pose: list[float] | None = None
        self._mode_toggle_buttons: dict[str, Button] = {}
        self._recording_indicator: Circle | None = None
        self._recording_indicator_text = None
        self._recording_indicator_label = None
        self._record_teach_button: Button | None = None
        TEACH_SESSION_DIR.mkdir(parents=True, exist_ok=True)
        self._fault_active = False
        self._motion_active = False
        self._playback_active = False
        self._base_iso_state = RobotISOState.CONNECTED_NOT_ENABLED
        led_config = load_led_config()
        self._drive_leds_callback = build_drive_leds(
            led_config,
            serial_writer=self._build_serial_writer(),
        )
        self.status_leds = StatusLEDController(on_change=self._on_leds_changed)
        self._latest_led_states: dict[str, LEDState] = {}
        self._led_panel_ax = None
        self._led_patches: dict[str, Circle] = {}
        self._led_reason_texts: dict[str, object] = {}
        self._led_pattern_texts: dict[str, object] = {}
        self._led_color_map: dict[str, str] = {}
        self._led_tick_count = 0
        self._led_flash_timer = None
        self._calibration_path = CALIBRATION_CONFIG_PATH
        self._calibration_loaded = False
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
        self._max_teach_samples_per_second = 10
        self._teach_sample_min_interval = 1.0 / self._max_teach_samples_per_second
        self._loaded_hard_limits: dict[int, tuple[float, float]] = {}
        self._loaded_workspace: dict[str, tuple[float, float]] = {}
        self._servo_multiplier_limits = (0.8, 1.2)
        self.wrist_extension_compensation: list[tuple[float, float]] = [
            (0.0, 1.0),
            (0.55, 1.0),
            (0.75, 1.05),
            (1.0, 1.12),
        ]
        self.servo_offsets: dict[int, float] = self._load_calibration_data()
        if not self._calibration_loaded:
            self.zero_reference = {
                index: angle
                for index, angle in enumerate(self._default_joint_configuration())
            }
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
        self._apply_loaded_hard_limits()

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
        self._return_from_teach_move_time_ms = 10_000
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

        self._calibration_active = False
        self._calibration_button: Button | None = None
        self._set_vertical_button: Button | None = None
        self._calibration_timer = None
        self._calibration_guide_active = False
        self._calibration_steps: list[tuple[int, str]] = []
        self._calibration_step_index: int | None = None
        self._calibration_status_text = None
        self._calibration_progress_text = None
        self._calibration_angle_box: TextBox | None = None
        self._updating_calibration_angle_box = False

        self._initialise_status_leds()

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
        self._zero_reference_lines: list[Line2D] = []
        for _ in range(4):
            (line,) = self.ax.plot(
                [], [], [],
                color="#6c757d",
                linestyle=":",
                lw=1.6,
                alpha=0.9,
            )
            self._zero_reference_lines.append(line)
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
        self.figure.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.figure.canvas.mpl_connect("key_release_event", self._on_key_release)
        self.figure.canvas.mpl_connect("resize_event", self._on_canvas_resized)

        self._calibration_overlay = self.ax.text2D(
            0.98,
            0.95,
            "",
            transform=self.ax.transAxes,
            ha="right",
            va="top",
            bbox=dict(facecolor="#eef6ff", edgecolor="#aac8ff", alpha=0.9),
        )
        self._calibration_arc: Wedge | None = None
        self._calibration_arc_text = None

        # UI placeholders populated when using the Matplotlib-based controls.
        self.buttons: dict[str, Button] = {}
        self.servo_value_texts: list = []
        self.servo_buttons: list[Button] = []
        self.servo_invert_buttons: list[Button] = []
        self.servo_limit_boxes_min: list[TextBox] = []
        self.servo_limit_boxes_max: list[TextBox] = []
        self.servo_pulse_sliders: list[Slider] = []
        self.servo_multiplier_sliders: list[Slider] = []
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
        self._mode_status_text = None
        self._smooth_teach_button: Button | None = None
        self._save_teach_button: Button | None = None
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

    def _build_serial_writer(self) -> Callable[[bytes], None] | None:
        controller = getattr(self, "controller", None)
        if controller is None or not hasattr(controller, "ensure_connection"):
            return None

        def _writer(data: bytes) -> None:
            serial_port = controller.ensure_connection()
            serial_port.write(data)

        return _writer

    def _initialise_status_leds(self) -> None:
        self._base_iso_state = RobotISOState.CONNECTED_NOT_ENABLED
        self._sync_leds()

    def _on_leds_changed(self, state: dict[str, LEDState]) -> None:
        self._latest_led_states = state
        self._render_led_states()
        if self._drive_leds_callback:
            try:
                self._drive_leds_callback(state)
            except Exception:
                # Hardware LED updates are best-effort and must not break the UI.
                pass

    def _update_idle_leds(self) -> None:
        if not (self._calibration_active or self._motion_active or self._playback_active):
            self._base_iso_state = RobotISOState.READY_IDLE
        self._sync_leds()

    def _mark_motion_active(self) -> None:
        if self._fault_active:
            self._fault_active = False
        self._motion_active = True
        self._base_iso_state = RobotISOState.RUNNING
        self._sync_leds()

    def _mark_motion_complete(self) -> None:
        if hasattr(self._command_queue, "empty") and not self._command_queue.empty():
            return
        if self._playback_active:
            return
        self._motion_active = False
        self._base_iso_state = RobotISOState.READY_IDLE
        self._sync_leds()

    def _mark_fault(self) -> None:
        self._fault_active = True
        self._motion_active = False
        self._sync_leds(fault_just_triggered=True)

    def _update_calibration_leds(self) -> None:
        if self._calibration_active:
            self._base_iso_state = RobotISOState.TEACH_MODE
        elif not (self._motion_active or self._playback_active):
            self._base_iso_state = RobotISOState.READY_IDLE
        self._sync_leds()

    def _sync_leds(self, *, fault_just_triggered: bool = False) -> None:
        teach_mode_active = self._operation_mode == "teach" or self._calibration_active
        teach_recording = self._teach_recording or self._calibration_guide_active
        calibration_active = self._calibration_active
        if self._fault_active:
            state = RobotISOState.FAULT
        elif teach_mode_active:
            state = RobotISOState.TEACH_MODE
        elif self._motion_active or self._playback_active:
            state = RobotISOState.RUNNING
        else:
            state = self._base_iso_state

        self.status_leds.set_iso_state(
            state,
            teach_active=teach_mode_active,
            teach_recording=teach_recording,
            fault_just_triggered=fault_just_triggered,
        )

        if calibration_active:
            self.status_leds.set_attention(
                active=True, transitioning=self._motion_active or self._playback_active
            )
            self.status_leds.set_ready(active=False)

    # ------------------------------------------------------------------
    # LED status display
    def _build_led_status_panel(self) -> None:
        """Create a left-hand summary showing simulated stack lights."""

        self._led_panel_ax = self.figure.add_axes([0.02, 0.08, 0.12, 0.8])
        self._led_panel_ax.set_xlim(0.0, 1.0)
        self._led_panel_ax.set_ylim(0.0, 1.0)
        self._led_panel_ax.axis("off")
        self._led_panel_ax.set_facecolor("#f8fafc")
        self._led_panel_ax.set_aspect("equal")

        self._led_panel_ax.text(
            0.5,
            0.96,
            "LED mirror",
            ha="center",
            va="center",
            fontsize=10,
            fontweight="bold",
            color="#0f172a",
        )
        self._led_panel_ax.text(
            0.5,
            0.92,
            "(live signals)",
            ha="center",
            va="center",
            fontsize=8,
            color="#475569",
        )

        led_order = [
            "red",
            "amber",
            "green",
            "yellow",
            "blue",
            "white",
        ]
        led_colors = {
            "red": "#ef4444",
            "amber": "#f59e0b",
            "green": "#22c55e",
            "yellow": "#fbbf24",
            "blue": "#3b82f6",
            "white": "#e5e7eb",
        }
        self._led_color_map = led_colors

        y_positions = np.linspace(0.82, 0.08, len(led_order))
        for name, y in zip(led_order, y_positions, strict=True):
            color = led_colors.get(name, "#94a3b8")
            circle = Circle(
                (0.18, y),
                0.065,
                facecolor="#0f172a",
                edgecolor=color,
                lw=2,
                alpha=0.2,
            )
            self._led_panel_ax.add_patch(circle)
            label = name.capitalize()
            self._led_panel_ax.text(
                0.34,
                y + 0.05,
                label,
                ha="left",
                va="center",
                fontsize=9,
                color="#0f172a",
                fontweight="bold",
            )
            reason_text = self._led_panel_ax.text(
                0.34,
                y,
                "idle",
                ha="left",
                va="center",
                fontsize=11,
                color="#0f172a",
                fontweight="bold",
            )
            pattern_text = self._led_panel_ax.text(
                0.34,
                y - 0.05,
                "off",
                ha="left",
                va="center",
                fontsize=8,
                color="#475569",
            )
            self._led_patches[name] = circle
            self._led_reason_texts[name] = reason_text
            self._led_pattern_texts[name] = pattern_text

        self._led_tick_count = 0
        self._led_flash_timer = self.figure.canvas.new_timer(interval=250)
        self._led_flash_timer.add_callback(self._tick_led_timer)
        self._led_flash_timer.start()
        self._render_led_states()

    def _tick_led_timer(self) -> None:
        self._led_tick_count += 1
        self._render_led_states()

    def _render_led_states(self) -> None:
        if self._led_panel_ax is None:
            return
        states = self._latest_led_states or self.status_leds.snapshot()
        for name, state in states.items():
            patch = self._led_patches.get(name)
            reason_text = self._led_reason_texts.get(name)
            pattern_text = self._led_pattern_texts.get(name)
            if patch is None or reason_text is None or pattern_text is None:
                continue

            base_color = self._led_color_map.get(name, "#94a3b8")
            pattern = state.pattern
            tick = self._led_tick_count
            if pattern == "blink_fast":
                on = (tick % 2) == 0
            elif pattern == "blink_slow":
                on = (tick % 6) < 3
            else:
                on = pattern != "off"

            face = base_color if on else "#0f172a"
            alpha = 1.0
            if pattern == "off":
                alpha = 0.18
            elif pattern == "dim":
                alpha = 0.35
            elif pattern.startswith("blink") and not on:
                alpha = 0.28

            patch.set_facecolor(face)
            patch.set_edgecolor(base_color)
            patch.set_alpha(alpha)

            reason_text.set_text(self._summarise_led_reason(state))
            pattern_text.set_text(pattern.replace("_", " "))
        if self.figure.canvas is not None:
            self.figure.canvas.draw_idle()

    def _summarise_led_reason(self, state: LEDState) -> str:
        meaning = state.meaning.lower()
        if "fault" in meaning or "stop" in meaning:
            return "error"
        if "attention" in meaning or "interlock" in meaning:
            return "hold"
        if "teach" in meaning or "calibration" in meaning:
            return "teach"
        if "ready" in meaning or "running" in meaning:
            if state.pattern in {"blink_slow", "blink_fast"}:
                return "running"
            if state.pattern == "off":
                return "idle"
            return "ready"
        if "usb" in meaning or "host" in meaning or "heartbeat" in meaning:
            return "link"
        if "illumination" in meaning or "presence" in meaning:
            return "light"
        return "status"

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

    def _zero_angle_for_servo(self, index: int) -> float:
        if index in self.zero_reference:
            return self.zero_reference[index]

        default_zero = (
            _DEFAULT_VERTICAL_JOINTS[index]
            if index < len(_DEFAULT_VERTICAL_JOINTS)
            else 0.0
        )
        return default_zero

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
        if not self._calibration_loaded:
            return self._default_joint_configuration()

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

    def update_robot(self, *, move_time_ms: int | None = None) -> None:
        requested = self._clamp_target(self.target)
        self.target[:] = requested
        compensated_pitch = self._apply_wrist_extension_compensation(
            requested, self.wrist_pitch
        )
        joints = self._apply_hard_limits_to_ik(
            list(self.kin.inverse(requested[[0, 1, 2]], compensated_pitch))
        )
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
            full_joints,
            move_time_ms=self.move_time_ms if move_time_ms is None else move_time_ms,
            soft_start=True,
        )
        self.figure.canvas.draw_idle()

        self._update_raw_angle_button_visual()

    def _resume_live_mode_from_teach(self) -> None:
        final_target = np.array(self.target, dtype=float)
        if self._pre_teach_target_position is not None:
            self.target[:] = self._pre_teach_target_position
            self._setpoint_position = np.array(self._pre_teach_target_position)
        if self._pre_teach_joints is not None:
            self._setpoint_joints = list(self._pre_teach_joints)

        if self._pre_teach_full_pose is not None:
            self.commanded_joints = list(self._pre_teach_full_pose)
            self._last_commanded_raw = tuple(self._pre_teach_full_pose)

        self.target[:] = final_target
        self.update_robot(move_time_ms=self._return_from_teach_move_time_ms)

    def _slider_value_from_pitch(self, pitch: float) -> float:
        clamped = float(np.clip(pitch, *self._wrist_pitch_limits))
        slider_value = 90.0 - math.degrees(clamped)
        return float(np.clip(slider_value, *self._wrist_slider_limits))

    def _pitch_from_slider_value(self, value: float) -> float:
        clamped_value = float(np.clip(value, *self._wrist_slider_limits))
        pitch = math.radians(90.0 - clamped_value)
        return float(np.clip(pitch, *self._wrist_pitch_limits))

    def _extension_ratio(self, position: Sequence[float]) -> float:
        planar_radius = math.hypot(float(position[0]), float(position[1]))
        max_reach = self.kin.links.shoulder + self.kin.links.elbow
        if max_reach <= 0:
            return 0.0
        return float(np.clip(planar_radius / max_reach, 0.0, 1.0))

    def _wrist_extension_multiplier(self, position: Sequence[float]) -> float:
        profile = sorted(self.wrist_extension_compensation, key=lambda item: item[0])
        if not profile:
            return 1.0

        ratio = self._extension_ratio(position)

        for index, (reach_fraction, multiplier) in enumerate(profile):
            if ratio <= reach_fraction:
                if index == 0:
                    return float(multiplier)
                prev_fraction, prev_multiplier = profile[index - 1]
                span = reach_fraction - prev_fraction
                blend = 0.0 if span == 0 else (ratio - prev_fraction) / span
                return float(prev_multiplier + blend * (multiplier - prev_multiplier))

        return float(profile[-1][1])

    def _apply_wrist_extension_compensation(
        self, position: Sequence[float], desired_pitch: float
    ) -> float:
        multiplier = self._wrist_extension_multiplier(position)
        compensated = desired_pitch * multiplier
        return float(np.clip(compensated, *self._wrist_pitch_limits))

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
        # Manipulating the end-effector directly from the 3D graph led to frequent
        # accidental drags. Ignore clicks in the IK viewport so movement comes
        # solely from the dedicated controls instead of the plot.
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

        self.figure.subplots_adjust(left=0.16, right=0.575, top=0.955, bottom=0.08)

        self._build_mode_toggle_strip()
        self._build_led_status_panel()
        self._build_recording_indicator()

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
            ("Modes", "modes"),
            ("Movement", "movement"),
            ("Servos", "servos"),
            ("Motor config", "motor_config"),
            ("Calibration tour", "calibration_tour"),
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

        self._panel_widgets["modes"] = self._build_mode_panel()
        self._panel_widgets["movement"] = self._build_movement_panel()
        self._panel_widgets["servos"] = self._build_servo_panel()
        self._panel_widgets["motor_config"] = self._build_motor_config_panel()
        self._panel_widgets["calibration_tour"] = (
            self._build_calibration_tour_panel()
        )
        self._panel_widgets["waypoints"] = self._build_waypoint_panel()
        self._panel_widgets["timeline"] = self._build_timeline_panel()

        self._set_active_panel("movement")
        self._refresh_waypoint_display()
        self._update_waypoint_duration_box()

    def _build_mode_toggle_strip(self) -> None:
        """Place always-visible Live/Teach toggle above the LED stack."""

        self._mode_toggle_buttons = {}
        label_ax = self.figure.add_axes([0.02, 0.94, 0.14, 0.03])
        label_ax.axis("off")
        label_ax.text(
            0.0,
            0.5,
            "Mode",
            ha="left",
            va="center",
            fontsize=9,
            fontweight="bold",
            color="#0f172a",
        )

        toggle_bottom = 0.885
        button_width = 0.065
        button_height = 0.05
        gap = 0.01
        for index, (mode, label) in enumerate((("live", "Live"), ("teach", "Teach"))):
            left = 0.02 + index * (button_width + gap)
            ax = self.figure.add_axes([left, toggle_bottom, button_width, button_height])
            button = Button(ax, label, hovercolor="#d9e8ff")
            button.on_clicked(lambda _event, selected=mode: self._set_operation_mode(selected))
            self._mode_toggle_buttons[mode] = button

    def _build_recording_indicator(self) -> None:
        indicator_ax = self.figure.add_axes([0.87, 0.93, 0.12, 0.05])
        indicator_ax.axis("off")
        indicator_ax.set_facecolor("#f8fafc")

        indicator = Circle((0.12, 0.5), 0.09, transform=indicator_ax.transAxes)
        indicator_ax.add_patch(indicator)
        text = indicator_ax.text(
            0.26,
            0.5,
            "REC",
            ha="left",
            va="center",
            fontsize=9,
            fontweight="bold",
        )
        label = indicator_ax.text(
            0.5,
            0.5,
            "Recording paused",
            ha="left",
            va="center",
            fontsize=8,
            color="#0f172a",
        )

        self._recording_indicator = indicator
        self._recording_indicator_text = text
        self._recording_indicator_label = label

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

    def _build_mode_panel(self) -> list:
        panel_key = "modes"
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
            0.6,
            "Teach tools & status",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        title_ax.text(
            0.0,
            0.0,
            "Use the top-left toggle to flip between Live (hardware) and Teach (virtual recording).",
            va="center",
            ha="left",
            fontsize=8,
            color="#334155",
        )
        axes.append(title_ax)

        info_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.24,
            1.0 - 2 * self._panel_margin,
            0.12,
        )
        info_ax.axis("off")
        info_text = (
            "Live Mode streams commands to the arm.\n"
            "Teach Mode mirrors every move to the digital twin only. Use Start recording to capture routines for later playback.\n"
            "Watch the top-right REC badge and the yellow LED overlay while teaching; save recordings into subroutines to use them in the timeline."
        )
        info_ax.text(
            0.0,
            0.95,
            info_text,
            va="top",
            ha="left",
            fontsize=8,
            color="#0f172a",
            wrap=True,
        )
        axes.append(info_ax)

        button_height = 0.08
        button_bottom = self._panel_content_top - 0.3

        record_ax = self._panel_axes(
            self._panel_margin,
            button_bottom,
            1.0 - 2 * self._panel_margin,
            button_height,
        )
        self._record_teach_button = Button(
            record_ax, "Start recording", hovercolor="#d9e8ff"
        )
        self._record_teach_button.on_clicked(self._toggle_teach_recording)
        self._panel_interactive_widgets[panel_key].append(self._record_teach_button)
        axes.append(record_ax)

        status_ax = self._panel_axes(
            self._panel_margin,
            button_bottom - 0.12,
            1.0 - 2 * self._panel_margin,
            0.1,
        )
        status_ax.axis("off")
        self._mode_status_text = status_ax.text(
            0.0,
            0.6,
            "Mode: Live (hardware commands enabled)",
            va="center",
            ha="left",
            fontsize=9,
            color="#0f172a",
        )
        status_ax.text(
            0.0,
            0.05,
            "Teach recordings auto-save with timestamps, gripper state, and dwell metadata.",
            va="bottom",
            ha="left",
            fontsize=8,
            color="#334155",
        )
        axes.append(status_ax)

        smooth_ax = self._panel_axes(
            self._panel_margin,
            self._panel_margin + 0.16,
            (1.0 - 3 * self._panel_margin) / 2,
            button_height,
        )
        self._smooth_teach_button = Button(
            smooth_ax, "Smooth recording", hovercolor="#d9e8ff"
        )
        self._smooth_teach_button.on_clicked(self._smooth_teach_session)
        self._panel_interactive_widgets[panel_key].append(self._smooth_teach_button)
        axes.append(smooth_ax)

        save_ax = self._panel_axes(
            self._panel_margin * 2 + (1.0 - 3 * self._panel_margin) / 2,
            self._panel_margin + 0.16,
            (1.0 - 3 * self._panel_margin) / 2,
            button_height,
        )
        self._save_teach_button = Button(save_ax, "Save routine", hovercolor="#d9e8ff")
        self._save_teach_button.on_clicked(self._finalise_teach_session)
        self._panel_interactive_widgets[panel_key].append(self._save_teach_button)
        axes.append(save_ax)

        summary_ax = self._panel_axes(
            self._panel_margin,
            self._panel_margin,
            1.0 - 2 * self._panel_margin,
            0.12,
        )
        summary_ax.axis("off")
        summary_ax.text(
            0.0,
            0.8,
            "Post-process smoothing trims jitter (averages neighbours) and keeps timestamps intact.",
            va="center",
            ha="left",
            fontsize=8,
            color="#0f172a",
        )
        summary_ax.text(
            0.0,
            0.3,
            "Saved teach recordings are written to Subroutines so they can be dropped onto the timeline immediately.",
            va="center",
            ha="left",
            fontsize=8,
            color="#0f172a",
            fontweight="bold",
        )
        axes.append(summary_ax)

        self._update_mode_controls()
        return axes

    def _set_operation_mode(self, mode: str) -> None:
        if mode not in {"live", "teach"}:
            return

        previous_mode = self._operation_mode
        self._operation_mode = mode

        if previous_mode == "teach" and mode != "teach":
            self._pause_teach_recording(status="Switched to Live mode; recording paused.")
            self._finalise_teach_session(auto_save=True)
            self._resume_live_mode_from_teach()
        elif mode == "teach" and previous_mode != "teach":
            self._remember_pose_before_teach()
            self._enter_teach_mode()

        self._update_mode_controls()
        self._sync_leds()

    def _remember_pose_before_teach(self) -> None:
        self._pre_teach_target_position = np.array(self.target, dtype=float)
        if self.commanded_joints:
            self._pre_teach_joints = list(self.commanded_joints[:4])
        elif self._setpoint_joints:
            self._pre_teach_joints = list(self._setpoint_joints)
        elif self.current_joints:
            self._pre_teach_joints = list(self.current_joints[:4])
        else:
            self._pre_teach_joints = None

        if self._pre_teach_joints is not None:
            if self.commanded_joints and len(self.commanded_joints) >= 6:
                wrist_rotation = self.commanded_joints[4]
                gripper_angle = self.commanded_joints[5]
            else:
                wrist_rotation = self.wrist_rotation if hasattr(self, "wrist_rotation") else 0.0
                gripper_angle = self.gripper_angle if hasattr(self, "gripper_angle") else 0.0

            self._pre_teach_full_pose = list(self._pre_teach_joints) + [
                wrist_rotation,
                gripper_angle,
            ]
        else:
            self._pre_teach_full_pose = None

    def _enter_teach_mode(self) -> None:
        self._teach_recording = False
        self._teach_samples = []
        self._smoothed_teach_samples = []
        self._teach_session_start = None
        self._last_teach_timestamp = None
        self._last_teach_status = (
            "Teach Mode: ready to record. Press Start recording to capture moves."
        )
        self._update_mode_controls()

    def _start_teach_session(self) -> None:
        self._teach_samples = []
        self._smoothed_teach_samples = []
        self._teach_session_start = None
        self._last_teach_timestamp = None
        self._teach_recording = True
        self._last_teach_status = "Teach Mode: recording virtual moves."
        self._update_mode_controls()

    def _pause_teach_recording(self, *, status: str | None = None) -> None:
        if not self._teach_recording:
            return
        self._teach_recording = False
        if status is not None:
            self._last_teach_status = status
        elif self._teach_samples:
            self._last_teach_status = (
                f"Recording paused ({len(self._teach_samples)} samples captured)."
            )
        else:
            self._last_teach_status = "Recording paused."
        self._update_mode_controls()

    def _toggle_teach_recording(self, _event=None) -> None:
        if self._operation_mode != "teach":
            self._last_teach_status = "Switch to Teach mode to record."
            self._update_mode_controls()
            return

        if self._teach_recording:
            self._pause_teach_recording()
        else:
            self._start_teach_session()

    def _record_teach_sample(
        self, joints: Sequence[float], move_time_ms: int | None
    ) -> None:
        if self._operation_mode != "teach" or not self._teach_recording:
            return
        now = time.monotonic()
        if self._teach_samples:
            if self._teach_sample_min_interval > 0:
                elapsed = now - (self._last_teach_timestamp or now)
                if elapsed < self._teach_sample_min_interval:
                    return
            if self._teach_session_start is None:
                self._teach_session_start = now
            timestamp = now - self._teach_session_start
            dwell_ms = int((now - (self._last_teach_timestamp or now)) * 1000)
        else:
            self._teach_session_start = now
            timestamp = 0.0
            dwell_ms = 0
        cartesian = (
            tuple(map(float, self._setpoint_position))
            if self._setpoint_position is not None
            else None
        )
        sample = TeachSample(
            timestamp=timestamp,
            joints=tuple(float(value) for value in joints),
            cartesian=cartesian,
            wrist_pitch=float(self.wrist_pitch),
            wrist_rotation=float(self.wrist_rotation),
            gripper_angle=float(self.gripper_angle),
            move_time_ms=move_time_ms,
            dwell_ms=dwell_ms,
            label=None,
        )
        self._teach_samples.append(sample)
        self._last_teach_timestamp = now
        self._last_teach_status = (
            f"Recording: {len(self._teach_samples)} samples captured"
        )
        self._update_mode_controls()

    def _smooth_teach_session(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if not self._teach_samples:
            self._last_teach_status = "No teach samples available to smooth."
            self._update_mode_controls()
            return
        if len(self._teach_samples) < 3:
            self._smoothed_teach_samples = list(self._teach_samples)
            self._last_teach_status = "Not enough samples for smoothing; using raw routine."
            self._update_mode_controls()
            return

        smoothed: list[TeachSample] = []
        window_size = 3
        for index, sample in enumerate(self._teach_samples):
            if index == 0 or index == len(self._teach_samples) - 1:
                smoothed.append(sample)
                continue
            window = self._teach_samples[index - 1 : index + 2]
            avg_joints = tuple(
                float(np.mean([frame.joints[idx] for frame in window]))
                for idx in range(len(sample.joints))
            )
            avg_cartesian = None
            if all(frame.cartesian is not None for frame in window):
                avg_cartesian = tuple(
                    float(np.mean([frame.cartesian[idx] for frame in window]))
                    for idx in range(3)
                )
            avg_pitch = float(np.mean([frame.wrist_pitch for frame in window]))
            avg_rotation = float(np.mean([frame.wrist_rotation for frame in window]))
            avg_gripper = float(np.mean([frame.gripper_angle for frame in window]))
            avg_move_time = int(
                round(np.mean([frame.move_time_ms or 0 for frame in window]))
            )
            avg_dwell = int(round(np.mean([frame.dwell_ms or 0 for frame in window])))
            smoothed.append(
                TeachSample(
                    timestamp=sample.timestamp,
                    joints=avg_joints,
                    cartesian=avg_cartesian,
                    wrist_pitch=avg_pitch,
                    wrist_rotation=avg_rotation,
                    gripper_angle=avg_gripper,
                    move_time_ms=avg_move_time,
                    dwell_ms=avg_dwell,
                    label=sample.label,
                )
            )

        self._smoothed_teach_samples = smoothed
        self._last_teach_status = (
            f"Smoothed {len(smoothed)} samples (window={window_size}) for cleaner playback."
        )
        self._update_mode_controls()

    def _teach_samples_to_waypoints(self, samples: list[TeachSample]) -> list[Waypoint]:
        waypoints: list[Waypoint] = []
        if not samples:
            return waypoints

        for index, sample in enumerate(samples):
            if sample.cartesian is not None:
                position = np.array(sample.cartesian, dtype=float)
            else:
                position = np.array(self.target, dtype=float)

            move_ms = sample.move_time_ms or 0
            dwell_ms = sample.dwell_ms or 0
            duration_ms = move_ms + dwell_ms

            if index > 0:
                previous = samples[index - 1]
                elapsed_ms = int(
                    max(0.0, (sample.timestamp - previous.timestamp) * 1000)
                )
                if elapsed_ms > 0:
                    # Prefer the recorded timing between samples so teach playback
                    # mirrors the original recording rate. Any move/dwell metadata
                    # longer than the real elapsed time is clamped to the recorded
                    # interval to avoid stretching the routine when move_time_ms
                    # defaults to 1s per keyframe.
                    duration_ms = min(duration_ms, elapsed_ms) if duration_ms > 0 else elapsed_ms
            if duration_ms <= 0:
                duration_ms = 200

            waypoints.append(
                Waypoint(
                    position=position,
                    duration=max(0.1, duration_ms / 1000.0),
                    wrist_pitch=sample.wrist_pitch,
                    wrist_rotation=sample.wrist_rotation,
                    gripper_angle=sample.gripper_angle,
                )
            )

        return waypoints

    def _save_teach_session(self, *, include_smoothed: bool = True) -> Path | None:
        if not self._teach_samples:
            return None
        self._last_teach_subroutine_path = None
        TEACH_SESSION_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        path = TEACH_SESSION_DIR / f"teach_session_{timestamp}.json"
        routine_duration = (
            self._teach_samples[-1].timestamp - self._teach_samples[0].timestamp
            if len(self._teach_samples) > 1
            else 0.0
        )
        payload: dict[str, object] = {
            "mode": self._operation_mode,
            "sample_count": len(self._teach_samples),
            "duration_s": routine_duration,
            "samples": [sample.to_dict() for sample in self._teach_samples],
        }
        export_samples: list[TeachSample] = list(self._teach_samples)
        if include_smoothed and self._smoothed_teach_samples:
            payload.update(
                {
                    "smoothing": {
                        "method": "moving_average",
                        "window": 3,
                    },
                    "smoothed_samples": [
                        sample.to_dict() for sample in self._smoothed_teach_samples
                    ],
                }
            )
            export_samples = list(self._smoothed_teach_samples)

        path.write_text(json.dumps(payload, indent=2))
        self._last_teach_save_path = path
        self._last_teach_subroutine_path = self._save_teach_subroutine(
            export_samples, timestamp
        )
        return path

    def _save_teach_subroutine(
        self, samples: list[TeachSample], timestamp: str
    ) -> Path | None:
        waypoints = self._teach_samples_to_waypoints(samples)
        if not waypoints:
            return None
        try:
            SUBROUTINE_STORAGE_DIR.mkdir(parents=True, exist_ok=True)
            name = f"Teach session {timestamp}"
            slug = self._slugify_subroutine_name(name)
            data = {
                "name": name,
                "slug": slug,
                "waypoints": self._serialise_waypoints(waypoints),
            }
            path = SUBROUTINE_STORAGE_DIR / f"{slug}.json"
            path.write_text(json.dumps(data, indent=2))
        except Exception:
            _LOGGER.warning(
                "Failed to save teach recording as a subroutine", exc_info=True
            )
            return None

        self._selected_subroutine_slug = slug
        self._load_subroutine_catalog()
        self._highlight_selected_subroutine()
        self._update_subroutine_name_box()
        self._update_timeline_entries_for_slug(slug)
        return path

    def _finalise_teach_session(self, _event=None, *, auto_save: bool = False) -> None:
        if not self._teach_samples:
            self._last_teach_status = "No teach samples to save yet."
            self._update_mode_controls()
            return

        saved_path = self._save_teach_session(include_smoothed=True)
        if saved_path:
            subroutine_note = (
                f" + {self._last_teach_subroutine_path.name}"
                if self._last_teach_subroutine_path
                else ""
            )
            self._last_teach_status = (
                f"Saved routine to {saved_path.name}{subroutine_note}"
            )

        if auto_save:
            self._teach_samples = []
            self._smoothed_teach_samples = []
        self._teach_recording = False
        self._teach_session_start = None
        self._last_teach_timestamp = None
        self._update_mode_controls()

    def _update_mode_controls(self) -> None:
        def _style_button(button: Button | None, active: bool, *, danger: bool = False) -> None:
            if button is None:
                return
            if active:
                button.color = "#aac8ff" if not danger else "#ffdec3"
                button.hovercolor = button.color
            else:
                button.color = "0.85"
                button.hovercolor = "0.95"
            button.ax.set_facecolor(button.color)

        for mode, button in self._mode_toggle_buttons.items():
            _style_button(button, self._operation_mode == mode)

        sample_count = len(self._teach_samples)
        smoothed_count = len(self._smoothed_teach_samples)
        status_parts = [f"Mode: {self._operation_mode.capitalize()}"]
        if self._operation_mode == "teach":
            status_parts.append("virtual outputs only")
            status_parts.append(
                "recording" if self._teach_recording else "recording paused"
            )
        if sample_count:
            status_parts.append(f"{sample_count} recorded moves")
        if smoothed_count:
            status_parts.append(f"smoothed {smoothed_count} points")
        if self._mode_status_text is not None:
            details = ", ".join(status_parts)
            if self._last_teach_status:
                details = f"{details}\n{self._last_teach_status}"
            if self._last_teach_save_path:
                details = f"{details}\nLast save: {self._last_teach_save_path.name}"
            if self._last_teach_subroutine_path:
                details = (
                    f"{details}\nSubroutine: {self._last_teach_subroutine_path.name}"
                )
            self._mode_status_text.set_text(details)

        self._update_recording_indicator()

        if self._record_teach_button is not None:
            _style_button(self._record_teach_button, self._teach_recording)
            self._record_teach_button.label.set_text(
                "Stop recording" if self._teach_recording else "Start recording"
            )
            enabled = self._operation_mode == "teach"
            self._record_teach_button.eventson = enabled
            self._record_teach_button.ax.set_alpha(1.0 if enabled else 0.4)

        smooth_enabled = sample_count > 0
        if self._smooth_teach_button is not None:
            self._smooth_teach_button.eventson = smooth_enabled
            self._smooth_teach_button.ax.set_alpha(1.0 if smooth_enabled else 0.4)
        if self._save_teach_button is not None:
            self._save_teach_button.eventson = smooth_enabled
            self._save_teach_button.ax.set_alpha(1.0 if smooth_enabled else 0.4)

        self.figure.canvas.draw_idle()

    def _update_recording_indicator(self) -> None:
        if (
            self._recording_indicator is None
            or self._recording_indicator_text is None
            or self._recording_indicator_label is None
        ):
            return

        active = self._teach_recording
        color = "#ef4444" if active else "#94a3b8"
        edge = "#7f1d1d" if active else "#cbd5e1"
        self._recording_indicator.set_facecolor(color)
        self._recording_indicator.set_edgecolor(edge)
        self._recording_indicator.set_alpha(0.95 if active else 0.3)
        self._recording_indicator_text.set_text("REC" if active else "IDLE")
        self._recording_indicator_text.set_color("#7f1d1d" if active else "#475569")

        teach_mode = self._operation_mode == "teach"
        if teach_mode and self._smoothed_teach_samples and not active:
            label_text = f"Teach paused – {len(self._smoothed_teach_samples)} smoothed pts"
        elif active and self._smoothed_teach_samples:
            label_text = f"Smoothed {len(self._smoothed_teach_samples)} pts ready"
        elif active:
            label_text = "Recording (teach mode)"
        elif teach_mode:
            label_text = "Recording paused (teach mode)"
        else:
            label_text = "Recording paused"
        self._recording_indicator_label.set_text(label_text)
        self._recording_indicator_label.set_color("#0f172a")

        if self.figure.canvas is not None:
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

    def _build_calibration_tour_panel(self) -> list:
        panel_key = "calibration_tour"
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
            "Guided calibration",
            va="center",
            ha="left",
            fontsize=10,
            fontweight="bold",
        )
        axes.append(title_ax)

        guide_height = 0.18
        guide_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.22,
            1.0 - 2 * self._panel_margin,
            guide_height,
        )
        guide_ax.axis("off")
        guide_ax.text(
            0.0,
            1.0,
            "Use the tour to step through each motor. The app will move to the\n"
            "center and each hard limit for every servo, showing the angle\n"
            "on the IK diagram. Confirm each stage before continuing; if the\n"
            "diagram does not match, nudge the joint until it does.",
            va="top",
            ha="left",
            fontsize=8,
            wrap=True,
            linespacing=1.4,
        )
        axes.append(guide_ax)

        status_height = 0.22
        status_ax = self._panel_axes(
            self._panel_margin,
            self._panel_content_top - 0.22 - status_height - 0.02,
            1.0 - 2 * self._panel_margin,
            status_height,
        )
        status_ax.axis("off")
        self._calibration_status_text = status_ax.text(
            0.0,
            0.75,
            "Start the tour to begin motor-by-motor guidance.",
            va="top",
            ha="left",
            fontsize=9,
            wrap=True,
            linespacing=1.5,
        )
        self._calibration_progress_text = status_ax.text(
            0.0,
            0.2,
            "",
            va="bottom",
            ha="left",
            fontsize=9,
            color="#444444",
        )
        axes.append(status_ax)

        button_height = 0.09
        button_gap = 0.025
        button_width = (
            1.0 - 2 * self._panel_margin - 2 * button_gap
        ) / 3
        button_bottom = self._panel_margin + 0.18

        start_ax = self._panel_axes(
            self._panel_margin,
            button_bottom,
            button_width,
            button_height,
        )
        confirm_ax = self._panel_axes(
            self._panel_margin + button_width + button_gap,
            button_bottom,
            button_width,
            button_height,
        )
        restart_ax = self._panel_axes(
            self._panel_margin + 2 * (button_width + button_gap),
            button_bottom,
            button_width,
            button_height,
        )

        start_button = Button(start_ax, "Start tour", hovercolor="0.95")
        start_button.on_clicked(self._start_calibration_tour)
        confirm_button = Button(confirm_ax, "Confirm step", hovercolor="0.95")
        confirm_button.on_clicked(self._confirm_calibration_step)
        restart_button = Button(restart_ax, "Restart", hovercolor="0.95")
        restart_button.on_clicked(self._restart_calibration_tour)

        self._panel_interactive_widgets[panel_key].extend(
            [start_button, confirm_button, restart_button]
        )
        axes.extend([start_ax, confirm_ax, restart_ax])

        angle_ax = self._panel_axes(
            self._panel_margin,
            self._panel_margin + 0.12,
            1.0 - 2 * self._panel_margin,
            0.055,
        )
        self._calibration_angle_box = TextBox(
            angle_ax,
            "Set angle (°)",
            initial="",
        )
        self._calibration_angle_box.on_submit(self._apply_calibration_angle_from_text)
        self._panel_interactive_widgets[panel_key].append(self._calibration_angle_box)
        axes.append(angle_ax)

        nudge_height = 0.085
        nudge_gap = 0.03
        nudge_width = (
            1.0 - 2 * self._panel_margin - nudge_gap
        ) / 2
        nudge_bottom = self._panel_margin + 0.04

        nudge_minus_ax = self._panel_axes(
            self._panel_margin,
            nudge_bottom,
            nudge_width,
            nudge_height,
        )
        nudge_plus_ax = self._panel_axes(
            self._panel_margin + nudge_width + nudge_gap,
            nudge_bottom,
            nudge_width,
            nudge_height,
        )

        nudge_minus = Button(nudge_minus_ax, "Nudge -5°", hovercolor="0.95")
        nudge_minus.on_clicked(self._make_calibration_nudge_callback(-5.0))
        nudge_plus = Button(nudge_plus_ax, "Nudge +5°", hovercolor="0.95")
        nudge_plus.on_clicked(self._make_calibration_nudge_callback(5.0))
        self._panel_interactive_widgets[panel_key].extend([nudge_minus, nudge_plus])
        axes.extend([nudge_minus_ax, nudge_plus_ax])

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
            "1) Tap Calibrate to start the calibration flow.\n"
            "2) Move each servo with +/- or the Pulse slider until the arm looks right.\n"
            "3) Press Set vertical to store that upright pose.\n"
            "Hard min°/Hard max° below update the same limits used during the"
            " calibration tour.",
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

            config = self._servo_config_with_multiplier(index)
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

            multiplier_ax = self._panel_axes(
                slider_left,
                row_bottom + row_height * 0.65,
                slider_width,
                row_height * 0.25,
            )
            multiplier_slider = Slider(
                multiplier_ax,
                "Fudge ×",
                valmin=self._servo_multiplier_limits[0],
                valmax=self._servo_multiplier_limits[1],
                valinit=self.servo_multipliers.get(index, 1.0),
                valfmt="%0.2f×",
            )
            multiplier_slider.on_changed(self._make_multiplier_slider_callback(index))
            self.servo_multiplier_sliders.append(multiplier_slider)
            self._panel_interactive_widgets[panel_key].append(multiplier_slider)
            axes.append(multiplier_ax)

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

            min_box = TextBox(min_ax, "Hard min°", initial="0.0")
            max_box = TextBox(max_ax, "Hard max°", initial="0.0")
            min_box.on_submit(self._make_limit_submit_callback(index, "min"))
            max_box.on_submit(self._make_limit_submit_callback(index, "max"))

            self.servo_limit_boxes_min.append(min_box)
            self.servo_limit_boxes_max.append(max_box)
            self._panel_interactive_widgets[panel_key].extend([min_box, max_box])
            self._update_limit_box_display(index)

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
            rotation_text = f"Rotation: {math.degrees(waypoint.wrist_rotation):.1f}°"
            gripper_text = f"Gripper: {math.degrees(waypoint.gripper_angle):.1f}°"
            text = self.waypoint_ax.text(
                0.04,
                y + self._waypoint_item_height / 2,
                f"{position_text}\n{waypoint.duration:.2f} s, {pitch_text}\n"
                f"{rotation_text}, {gripper_text}",
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
            wrist_rotation = entry.get("wrist_rotation", self.wrist_rotation)
            gripper_angle = entry.get("gripper_angle", self.gripper_angle)
            try:
                position_array = np.array(position, dtype=float)
                if position_array.shape != (3,):
                    continue
                duration_value = float(duration)
                pitch_value = float(wrist_pitch)
                rotation_value = float(wrist_rotation)
                gripper_value = float(gripper_angle)
            except (TypeError, ValueError):
                continue

            rotation_config = self.servo_configs.get(4)
            if rotation_config is not None:
                rotation_value = rotation_config.clamp_angle(rotation_value)
            gripper_config = self.servo_configs.get(5)
            if gripper_config is not None:
                gripper_value = gripper_config.clamp_angle(gripper_value)

            loaded.append(
                Waypoint(
                    position=position_array,
                    duration=max(0.1, duration_value),
                    wrist_pitch=float(np.clip(pitch_value, *self._wrist_pitch_limits)),
                    wrist_rotation=rotation_value,
                    gripper_angle=gripper_value,
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
                "wrist_rotation": float(waypoint.wrist_rotation),
                "gripper_angle": float(waypoint.gripper_angle),
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
                    "Ignoring invalid hard %s limit for %s (%s)", bound, name, model
                )
                self._update_limit_box_display(index)
                return

            zero_angle = self._zero_angle_for_servo(index)
            radians_value = math.radians(value) + zero_angle
            if bound == "min":
                self._update_servo_limit(index, min_angle=radians_value)
            else:
                self._update_servo_limit(index, max_angle=radians_value)

        return _callback

    def _make_pulse_slider_callback(self, index: int):
        def _callback(value: float) -> None:  # pragma: no cover - UI interaction
            config = self._servo_config_with_multiplier(index)
            if config is None:
                return
            self._set_servo_angle(index, config.pulse_to_angle(value))

        return _callback

    def _scaled_servo_config(
        self, config: ServoConfig, multiplier: float
    ) -> ServoConfig:
        mid_pulse = (config.min_pulse + config.max_pulse) / 2.0
        half_span = (config.max_pulse - config.min_pulse) / 2.0
        scaled_half_span = half_span * multiplier
        min_pulse = mid_pulse - scaled_half_span
        max_pulse = mid_pulse + scaled_half_span
        return ServoConfig(
            min_angle=config.min_angle,
            max_angle=config.max_angle,
            min_pulse=int(round(min_pulse)),
            max_pulse=int(round(max_pulse)),
        )

    def _servo_config_with_multiplier(self, index: int) -> ServoConfig | None:
        config = self.servo_configs.get(index)
        if config is None:
            return None
        multiplier = self.servo_multipliers.get(index, 1.0)
        clamped = float(np.clip(multiplier, *self._servo_multiplier_limits))
        if math.isclose(clamped, 1.0, rel_tol=0.0, abs_tol=1e-6):
            return config
        return self._scaled_servo_config(config, clamped)

    def _set_servo_multiplier(self, index: int, multiplier: float) -> None:
        clamped = float(np.clip(multiplier, *self._servo_multiplier_limits))
        self.servo_multipliers[index] = clamped
        self._update_multiplier_slider_display(index)
        self._update_pulse_slider_range(index)
        self._save_calibration_data()
        self._send_move_command(self.commanded_joints, move_time_ms=self.move_time_ms)

    def _make_multiplier_slider_callback(self, index: int):
        def _callback(value: float) -> None:  # pragma: no cover - UI interaction
            self._set_servo_multiplier(index, value)

        return _callback

    def _make_calibration_nudge_callback(self, delta_degrees: float):
        def _callback(_event=None) -> None:  # pragma: no cover - UI interaction
            self._nudge_current_calibration_step(math.radians(delta_degrees))

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

    def _set_servo_angle(self, index: int, angle: float) -> None:
        config = self.servo_configs.get(index)
        if config is None:
            return

        source = self.feedback_joints or self.current_joints
        if not source:
            return

        updated = list(source)
        old_angle = updated[index]
        new_angle = config.clamp_angle(angle)
        if math.isclose(new_angle, old_angle, abs_tol=1e-6):
            name, model, _ = self._SERVO_METADATA[index]
            zero_angle = self._zero_angle_for_servo(index)
            _LOGGER.warning(
                "%s (%s) servo adjustment hit the configured limit (%.1f° to %.1f°).",
                name,
                model,
                math.degrees(config.min_angle - zero_angle),
                math.degrees(config.max_angle - zero_angle),
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
        zero_angle = self._zero_angle_for_servo(index)
        min_box.set_val(f"{math.degrees(base_config.min_angle - zero_angle):.1f}")
        max_box.set_val(f"{math.degrees(base_config.max_angle - zero_angle):.1f}")
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
        config = self._servo_config_with_multiplier(index)
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
        config = self._servo_config_with_multiplier(index)
        if slider is None or config is None:
            return
        slider.valmin = min(config.min_pulse, config.max_pulse)
        slider.valmax = max(config.min_pulse, config.max_pulse)
        slider.ax.set_xlim(slider.valmin, slider.valmax)
        self._update_pulse_slider_display(index)

    def _update_multiplier_slider_display(self, index: int) -> None:
        if not hasattr(self, "servo_multiplier_sliders"):
            return
        if index >= len(self.servo_multiplier_sliders):
            return
        slider = self.servo_multiplier_sliders[index]
        if slider is None:
            return
        value = self.servo_multipliers.get(index, 1.0)
        low = min(slider.valmin, slider.valmax)
        high = max(slider.valmin, slider.valmax)
        try:
            slider.eventson = False
        except AttributeError:  # pragma: no cover - Matplotlib implementation detail
            pass
        try:
            slider.set_val(float(np.clip(value, low, high)))
        finally:
            try:
                slider.eventson = True
            except AttributeError:  # pragma: no cover - Matplotlib implementation detail
                pass

    def _apply_hard_limits_to_ik(self, joints: list[float]) -> list[float]:
        clamped = self._clamp_joint_list(joints)
        if len(clamped) != len(joints):
            return clamped
        for idx, (original, limited) in enumerate(zip(joints, clamped)):
            if not math.isclose(original, limited, rel_tol=0.0, abs_tol=1e-6):
                name, model, _ = self._SERVO_METADATA[idx]
                zero_angle = self._zero_angle_for_servo(idx)
                _LOGGER.info(
                    "%s (%s) IK solution clipped to hard limits (%.1f° → %.1f°)",
                    name,
                    model,
                    math.degrees(original - zero_angle),
                    math.degrees(limited - zero_angle),
                )
                break
        return clamped

    def _clamp_joint_list(
        self, joints: list[float] | tuple[float, ...]
    ) -> list[float]:
        clamped = list(joints)
        for idx, angle in enumerate(clamped):
            config = self.servo_configs.get(idx)
            if config is not None:
                clamped[idx] = config.clamp_angle(angle)
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
            self._calibration_loaded = False
            return {}
        try:
            data = json.loads(path.read_text())
        except Exception:  # pragma: no cover - configuration robustness
            _LOGGER.warning(
                "Failed to load calibration data from %s", path, exc_info=True
            )
            self._calibration_loaded = False
            return {}

        self._calibration_loaded = True

        offsets_raw: dict[str, object] | None = None
        vertical_raw: dict[str, object] | None = None
        inversion_raw: dict[str, object] | None = None
        workspace_raw: dict[str, object] | None = None
        limits_raw: dict[str, object] | None = None
        multipliers_raw: dict[str, object] | None = None

        if isinstance(data, dict):
            structured_keys = {
                "offsets",
                "vertical_angles",
                "inverted",
                "workspace",
                "servo_limits",
                "multipliers",
            }
            if structured_keys.intersection(data):
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
                multiplier_candidate = data.get("multipliers")
                if isinstance(multiplier_candidate, dict):
                    multipliers_raw = multiplier_candidate
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
                self._loaded_hard_limits.update(parsed_limits)

        if multipliers_raw:
            for key, value in multipliers_raw.items():
                try:
                    index = int(key)
                    multiplier = float(value)
                except (TypeError, ValueError):
                    _LOGGER.warning("Ignoring invalid servo multiplier entry for %s", key)
                    continue
                self.servo_multipliers[index] = float(
                    np.clip(multiplier, *self._servo_multiplier_limits)
                )

        return offsets

    def _save_calibration_data(self) -> None:
        path = self._calibration_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            servo_limits = {
                str(idx): {
                    "min_deg": math.degrees(config.min_angle),
                    "max_deg": math.degrees(config.max_angle),
                }
                for idx, config in self._base_servo_configs.items()
            }
            offsets_serialised = {
                str(idx): offset for idx, offset in self.servo_offsets.items()
            }
            vertical_serialised = {
                str(idx): angle for idx, angle in self.zero_reference.items()
            }
            multipliers_serialised = {
                str(idx): value for idx, value in self.servo_multipliers.items()
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
                    "multipliers": dict(multipliers_serialised),
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

    def _apply_loaded_hard_limits(self) -> None:
        if not self._loaded_hard_limits:
            return
        for idx, (min_angle, max_angle) in self._loaded_hard_limits.items():
            base = self._base_servo_configs.get(idx)
            if base is None:
                continue
            if min_angle >= max_angle:
                continue
            updated = ServoConfig(
                min_angle=min_angle,
                max_angle=max_angle,
                min_pulse=base.min_pulse,
                max_pulse=base.max_pulse,
            )
            self._base_servo_configs[idx] = updated
            self.servo_configs[idx] = updated

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
                wrist_rotation=self.wrist_rotation,
                gripper_angle=self.gripper_angle,
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
        self._playback_active = True
        self._mark_motion_active()
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
        self._playback_active = True
        self._mark_motion_active()
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
        self._playback_active = False
        self._update_idle_leds()
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
            self._playback_active = False
            self._update_idle_leds()
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
        self._playback_active = False
        self._update_idle_leds()
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
            self._playback_active = False
            self._update_idle_leds()
            self._update_timeline_play_button(running=False)

    def _execute_waypoint_sequence(
        self,
        waypoints: list[Waypoint],
        *,
        stop_event: threading.Event,
        selection_callback: Callable[[int, Waypoint], None] | None = None,
        playback_speed: float = 1.0,
    ) -> None:
        previous_playback_state = self._playback_active
        self._playback_active = True
        self._mark_motion_active()
        try:
            for index, waypoint in enumerate(waypoints):
                if stop_event.is_set():
                    break
                target = self._clamp_target(np.array(waypoint.position, dtype=float))
                scaled_duration = waypoint.duration / max(playback_speed, 0.1)
                duration_ms = int(max(0.02, scaled_duration) * 1000)
                desired_pitch = float(np.clip(waypoint.wrist_pitch, *self._wrist_pitch_limits))
                desired_rotation = waypoint.wrist_rotation
                desired_gripper = waypoint.gripper_angle
                self._set_wrist_pitch_target(desired_pitch, update_slider=False)
                try:
                    compensated_pitch = self._apply_wrist_extension_compensation(
                        target, desired_pitch
                    )
                    joints = self._apply_hard_limits_to_ik(
                        list(self.kin.inverse(target[[0, 1, 2]], compensated_pitch))
                    )
                except Exception:
                    _LOGGER.exception("Failed to solve IK for scheduled waypoint %d", index + 1)
                    continue
                self._last_wrist_pitch = joints[1] + joints[2] + joints[3]
                self.wrist_rotation = desired_rotation
                self.gripper_angle = desired_gripper
                full_joints = joints + [desired_rotation, desired_gripper]
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
        finally:
            self._playback_active = previous_playback_state
            if not self._playback_active:
                self._update_idle_leds()

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
        if self._calibration_active and not self._calibration_guide_active:
            return
        self._calibration_active = True
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
        self._update_calibration_leds()

    def _exit_calibration_mode(self) -> None:
        if not self._calibration_active:
            return
        self._calibration_active = False
        self._update_calibration_button_visual()
        self._update_calibration_leds()

    def _generate_calibration_steps(self) -> list[tuple[int, str]]:
        steps: list[tuple[int, str]] = []
        for idx in range(len(self._SERVO_METADATA)):
            steps.extend([(idx, "zero"), (idx, "min"), (idx, "max"), (idx, "center")])
        return steps

    def _target_angle_for_phase(self, index: int, phase: str) -> float:
        config = self.servo_configs.get(index)
        if phase == "zero":
            return self._zero_angle_for_servo(index)
        if phase == "center":
            if config is not None:
                return config.min_angle + (config.max_angle - config.min_angle) / 2.0
            return 0.0
        if config is None:
            return 0.0
        if phase == "min":
            return config.min_angle
        return config.max_angle

    def _calibration_neutral_joints(self) -> list[float]:
        neutral: list[float] = []
        for idx in range(len(self._SERVO_METADATA)):
            if idx in self.zero_reference:
                neutral.append(self.zero_reference[idx])
            elif idx < len(_DEFAULT_VERTICAL_JOINTS):
                neutral.append(_DEFAULT_VERTICAL_JOINTS[idx])
            else:
                neutral.append(0.0)
        return neutral

    def _start_calibration_tour(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._calibration_steps = self._generate_calibration_steps()
        self._calibration_guide_active = True
        self._calibration_step_index = None
        self._enter_calibration_mode()
        self._advance_calibration_step(force_index=0)
        self._update_calibration_leds()

    def _restart_calibration_tour(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._start_calibration_tour()

    def _confirm_calibration_step(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if not self._calibration_guide_active:
            self._start_calibration_tour()
            return
        self._advance_calibration_step()

    def _advance_calibration_step(self, *, force_index: int | None = None) -> None:
        if not self._calibration_steps:
            self._calibration_steps = self._generate_calibration_steps()
        if force_index is None:
            next_index = 0 if self._calibration_step_index is None else self._calibration_step_index + 1
        else:
            next_index = force_index

        if next_index >= len(self._calibration_steps):
            if self._calibration_step_index is not None:
                previous_servo, _ = self._calibration_steps[self._calibration_step_index]
                zero_angle = self._zero_angle_for_servo(previous_servo)
                self._move_servo_for_calibration(
                    previous_servo,
                    zero_angle,
                    base_joints=self._calibration_neutral_joints(),
                )
            self._calibration_guide_active = False
            self._calibration_step_index = None
            self._update_calibration_status("Guided calibration complete.")
            self._update_calibration_overlay()
            self._exit_calibration_mode()
            return

        previous_step_index = self._calibration_step_index
        self._calibration_step_index = next_index
        servo_index, phase = self._calibration_steps[next_index]
        baseline = self._calibration_neutral_joints()
        target_angle = self._target_angle_for_phase(servo_index, phase)
        self._move_servo_for_calibration(
            servo_index, target_angle, base_joints=baseline
        )
        self._update_calibration_status()
        self._update_calibration_overlay()
        self._update_calibration_leds()

    def _move_servo_for_calibration(
        self, index: int, target_angle: float, *, base_joints: Sequence[float] | None = None
    ) -> None:
        source = base_joints or self.feedback_joints or self.current_joints
        if not source:
            source = self._default_joint_configuration()
        joints = list(source)
        while len(joints) < len(self._SERVO_METADATA):
            joints.append(0.0)

        joints[index] = target_angle
        joints = self._clamp_joint_list(joints)
        self.commanded_joints = list(joints)
        self.current_joints = list(joints)

        if len(joints) >= 4:
            try:
                pose = self.kin.forward(self.commanded_joints)
            except Exception:
                pose = None
            if pose is not None:
                self.target[:3] = pose[:3, 3]
                actual_pitch = (
                    self.commanded_joints[1]
                    + self.commanded_joints[2]
                    + self.commanded_joints[3]
                )
                self._last_wrist_pitch = actual_pitch
                self._set_wrist_pitch_target(actual_pitch)
                self._update_visuals(self.commanded_joints[:4])

        self._update_servo_readouts()
        self._cancel_pending_commands()
        self._send_move_command(
            self.commanded_joints, move_time_ms=self.move_time_ms, soft_start=False
        )

    def _update_calibration_status(self, message: str | None = None) -> None:
        if self._calibration_status_text is None:
            return

        if message is None and self._calibration_step_index is not None:
            servo_index, phase = self._calibration_steps[self._calibration_step_index]
            name, model, _ = self._SERVO_METADATA[servo_index]
            target_angle = self._target_angle_for_phase(servo_index, phase)
            config = self._servo_config_with_multiplier(servo_index)
            zero_angle = self._zero_angle_for_servo(servo_index)
            relative_target_deg = math.degrees(target_angle - zero_angle)
            message = (
                f"{name} ({model}) — {phase.capitalize()}\n"
                f"Target: {relative_target_deg:.1f}°"
            )
            if self._show_raw_angles:
                message += f" (raw {math.degrees(target_angle):.1f}°)"
            if config is not None:
                mid_pulse = (config.min_pulse + config.max_pulse) // 2
                message += (
                    f"\nPulse: {config.angle_to_pulse(target_angle)}µs"
                    f" (min {config.min_pulse}µs / mid {mid_pulse}µs / max {config.max_pulse}µs)"
                )
        elif message is None:
            message = "Start the tour to begin motor-by-motor guidance."

        self._calibration_status_text.set_text(message)

        if self._calibration_progress_text is not None:
            if self._calibration_step_index is None:
                progress = ""
            else:
                progress = (
                    f"Step {self._calibration_step_index + 1} of {len(self._calibration_steps)}"
                )
            self._calibration_progress_text.set_text(progress)
        self.figure.canvas.draw_idle()

    def _clear_calibration_arc_display(self) -> None:
        if self._calibration_arc is not None:
            self._calibration_arc.set_visible(False)
        if self._calibration_arc_text is not None:
            self._calibration_arc_text.set_visible(False)

    def _update_calibration_arc_display(
        self, servo_index: int, target_angle: float
    ) -> None:
        zero_angle = self._zero_angle_for_servo(servo_index)
        relative_deg = math.degrees(target_angle - zero_angle)
        theta_start = 90.0
        theta_end = theta_start + relative_deg

        # Matplotlib's 3D axes do not support 2D patch projection out of the
        # box. Adding a Wedge triggered an AttributeError for
        # ``do_3d_projection`` during draw, which cleared the figure when the
        # calibration tour began. Keep the text indicator but skip the patch to
        # avoid backend crashes.
        if self._calibration_arc is not None:
            try:
                self._calibration_arc.remove()
            except Exception:
                pass
            self._calibration_arc = None

        if self._calibration_arc_text is None:
            # Use text2D instead of text so we do not need to supply a Z value
            # when annotating on a 3D axis. A missing Z argument caused a
            # TypeError as soon as the calibration tour started, blanking the
            # UI with an uncaught exception from the Matplotlib callback.
            self._calibration_arc_text = self.ax.text2D(
                0.85,
                0.18,
                "",
                ha="center",
                va="center",
                fontsize=8,
                transform=self.ax.transAxes,
                bbox=dict(facecolor="white", edgecolor="#aac8ff", alpha=0.8),
            )

        self._calibration_arc_text.set_text(f"{relative_deg:+.1f}°")
        self._calibration_arc_text.set_visible(True)

    def _update_calibration_angle_box(self, actual_angle: float | None) -> None:
        if self._calibration_angle_box is None:
            return

        self._updating_calibration_angle_box = True
        try:
            if actual_angle is None:
                self._calibration_angle_box.set_val("")
            else:
                zero_angle = 0.0
                if self._calibration_step_index is not None:
                    servo_index, _ = self._calibration_steps[self._calibration_step_index]
                    zero_angle = self._zero_angle_for_servo(servo_index)
                relative_angle_deg = math.degrees(actual_angle - zero_angle)
                self._calibration_angle_box.set_val(f"{relative_angle_deg:.2f}")
        finally:
            self._updating_calibration_angle_box = False

    def _update_calibration_overlay(self) -> None:
        if self._calibration_overlay is None:
            return
        if not self._calibration_guide_active or self._calibration_step_index is None:
            self._calibration_overlay.set_text("")
            self._clear_calibration_arc_display()
            self._update_calibration_angle_box(None)
            self.figure.canvas.draw_idle()
            return

        servo_index, phase = self._calibration_steps[self._calibration_step_index]
        name, model, _ = self._SERVO_METADATA[servo_index]
        target_angle = self._target_angle_for_phase(servo_index, phase)
        actual_angle = None
        source = self.feedback_joints or self.current_joints
        if source and servo_index < len(source):
            actual_angle = source[servo_index]
        config = self._servo_config_with_multiplier(servo_index)
        phase_label = {
            "center": "Center", "min": "Hard min", "max": "Hard max", "zero": "Zero"
        }.get(phase, phase.capitalize())
        zero_angle = self._zero_angle_for_servo(servo_index)
        relative_target_deg = math.degrees(target_angle - zero_angle)
        overlay_lines = [
            f"Calibration tour: {phase_label}",
            f"{name} ({model})",
            f"Target {relative_target_deg:.1f}°",
        ]
        if self._show_raw_angles:
            overlay_lines[-1] += f" (raw {math.degrees(target_angle):.1f}°)"
        if actual_angle is not None:
            relative_actual_deg = math.degrees(actual_angle - zero_angle)
            current_line = f"Current {relative_actual_deg:.1f}°"
            if self._show_raw_angles:
                current_line += f" (raw {math.degrees(actual_angle):.1f}°)"
            overlay_lines.append(current_line)
        if config is not None:
            mid_pulse = (config.min_pulse + config.max_pulse) // 2
            overlay_lines.extend(
                [
                    f"Target pulse {config.angle_to_pulse(target_angle)}µs",
                    f"Pulse range {config.min_pulse}–{config.max_pulse}µs (mid {mid_pulse}µs)",
                ]
            )
        self._calibration_overlay.set_text("\n".join(overlay_lines))
        self._update_calibration_angle_box(actual_angle)
        self._update_calibration_arc_display(servo_index, target_angle)
        self.figure.canvas.draw_idle()

    def _apply_calibration_angle_from_text(self, text: str) -> None:
        if self._updating_calibration_angle_box:
            return
        if not self._calibration_guide_active or self._calibration_step_index is None:
            return

        try:
            value = float(text)
        except ValueError:
            _LOGGER.warning("Invalid calibration angle entry: %s", text)
            return

        radians_value = math.radians(value)
        servo_index, phase = self._calibration_steps[self._calibration_step_index]
        zero_angle = self._zero_angle_for_servo(servo_index)
        absolute_angle = zero_angle + radians_value

        if phase == "min":
            self._update_servo_limit(servo_index, min_angle=absolute_angle)
        elif phase == "max":
            self._update_servo_limit(servo_index, max_angle=absolute_angle)
        elif phase == "center":
            self.zero_reference[servo_index] = absolute_angle
            self._save_calibration_data()
            self._update_zero_reference_lines()
        else:
            self._set_servo_angle(servo_index, absolute_angle)
        self._update_calibration_status()
        self._update_calibration_overlay()

    def _nudge_current_calibration_step(self, delta: float) -> None:
        if not self._calibration_guide_active or self._calibration_step_index is None:
            return
        source = self.feedback_joints or self.current_joints
        if not source:
            return
        servo_index, phase = self._calibration_steps[self._calibration_step_index]
        zero_angle = self._zero_angle_for_servo(servo_index)
        config = self.servo_configs.get(servo_index)

        if phase == "min" and config is not None:
            self._update_servo_limit(servo_index, min_angle=config.min_angle + delta)
        elif phase == "max" and config is not None:
            self._update_servo_limit(servo_index, max_angle=config.max_angle + delta)
        elif phase == "center":
            current_zero = self.zero_reference.get(servo_index, zero_angle)
            self.zero_reference[servo_index] = current_zero + delta
            self._save_calibration_data()
            self._update_zero_reference_lines()
        else:
            self._adjust_servo(servo_index, delta)
        self._update_calibration_status()
        self._update_calibration_overlay()
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

    def _zero_pose_joints(self) -> list[float]:
        zeros: list[float] = []
        for idx in range(4):
            zeros.append(
                self.zero_reference.get(
                    idx,
                    _DEFAULT_VERTICAL_JOINTS[idx]
                    if idx < len(_DEFAULT_VERTICAL_JOINTS)
                    else 0.0,
                )
            )
        return zeros

    def _update_zero_reference_lines(self) -> None:
        if len(self._zero_reference_lines) < 4:
            return

        zero_joints = self._zero_pose_joints()
        anchor_joints: Sequence[float] | None = None
        if self.commanded_joints and len(self.commanded_joints) >= 4:
            anchor_joints = self.commanded_joints
        elif self.current_joints and len(self.current_joints) >= 4:
            anchor_joints = self.current_joints

        try:
            zx, zy, zz = self._compute_link_positions(zero_joints)
        except Exception:
            for line in self._zero_reference_lines:
                line.set_data([], [])
                line.set_3d_properties([])
            return

        anchor_points: list[np.ndarray] | None = None
        anchor_base_height = zz[1]
        if anchor_joints is not None:
            try:
                ax, ay, az = self._compute_link_positions(anchor_joints)
            except Exception:
                pass
            else:
                anchor_points = [
                    np.array([ax[1], ay[1], az[1]]),
                    np.array([ax[2], ay[2], az[2]]),
                    np.array([ax[3], ay[3], az[3]]),
                ]
                anchor_base_height = az[1]

        base_zero = zero_joints[0]
        base_dir = np.array([math.cos(base_zero), math.sin(base_zero), 0.0])
        base_start = np.array([0.0, 0.0, zz[1]])
        base_start[2] = anchor_base_height
        base_length = max(self.kin.links.shoulder * 0.35, 0.04)
        base_end = base_start + base_dir * base_length
        self._zero_reference_lines[0].set_data(
            [base_start[0], base_end[0]], [base_start[1], base_end[1]]
        )
        self._zero_reference_lines[0].set_3d_properties([base_start[2], base_end[2]])

        joint_points = [
            np.array([zx[1], zy[1], zz[1]]),
            np.array([zx[2], zy[2], zz[2]]),
            np.array([zx[3], zy[3], zz[3]]),
        ]
        next_points = [
            np.array([zx[2], zy[2], zz[2]]),
            np.array([zx[3], zy[3], zz[3]]),
            np.array([zx[4], zy[4], zz[4]]),
        ]
        if anchor_points is None:
            anchor_points = joint_points

        for line, start, end in zip(
            self._zero_reference_lines[1:], anchor_points, next_points
        ):
            direction = end - start
            shortened = start + direction * 0.35
            line.set_data([start[0], shortened[0]], [start[1], shortened[1]])
            line.set_3d_properties([start[2], shortened[2]])

    def _update_visuals(self, joints: list[float]) -> None:
        if len(joints) < 4:
            self.base_line.set_data([], [])
            self.base_line.set_3d_properties([])
            self.setpoint_line.set_data([], [])
            self.setpoint_line.set_3d_properties([])
            for line in self._zero_reference_lines:
                line.set_data([], [])
                line.set_3d_properties([])
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
        self._update_zero_reference_lines()

    def _update_servo_readouts(self) -> None:
        if not self.servo_value_texts:
            return
        for idx, text in enumerate(self.servo_value_texts):
            name, model, location = self._SERVO_METADATA[idx]
            zero_angle = self._zero_angle_for_servo(idx)
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
            config = self._base_servo_configs.get(idx)
            limits_text = ""
            if config is not None:
                limits_text = (
                    f"Limits: {math.degrees(config.min_angle - zero_angle):.0f}° to "
                    f"{math.degrees(config.max_angle - zero_angle):.0f}°"
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
        self._update_calibration_overlay()

    def _update_servo_limit(
        self,
        index: int,
        *,
        min_angle: float | None = None,
        max_angle: float | None = None,
    ) -> None:
        """Update the calibrated hard limits without altering the pulse mapping."""
        base_config = self._base_servo_configs.get(index)
        if base_config is None:
            return

        new_min = base_config.min_angle if min_angle is None else min_angle
        new_max = base_config.max_angle if max_angle is None else max_angle
        if new_min >= new_max:
            name, model, _ = self._SERVO_METADATA[index]
            zero_angle = self._zero_angle_for_servo(index)
            _LOGGER.warning(
                "Ignored invalid limit update for %s (%s): min %.1f° >= max %.1f°",
                name,
                model,
                math.degrees(new_min - zero_angle),
                math.degrees(new_max - zero_angle),
            )
            self._update_limit_box_display(index)
            return

        updated_base = ServoConfig(
            min_angle=new_min,
            max_angle=new_max,
            min_pulse=base_config.min_pulse,
            max_pulse=base_config.max_pulse,
        )
        self._base_servo_configs[index] = updated_base
        self._apply_servo_inversion(index)
        self.servo_configs[index] = (
            self.servo_configs.get(index) or updated_base
        )

        # Keep the loaded limits cache in sync so future persistence uses the
        # most recent values. This is particularly important when the limits
        # originated from a configuration file because users expect their
        # adjustments to overwrite the stored values instead of reverting on
        # restart.
        self._loaded_hard_limits[index] = (new_min, new_max)

        # Persist the calibration data immediately so the adjustments are not
        # lost if the application exits before another save opportunity.
        self._save_calibration_data()

        def _clamp_list(values: list[float] | None) -> None:
            if values is None:
                return
            if index >= len(values):
                return
            values[index] = self.servo_configs[index].clamp_angle(values[index])

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
                self._mark_motion_active()
                interrupted = False
                aborted_for_calibration = False
                for segment_raw, segment_time in self._generate_smooth_segments(
                    self._last_commanded_raw, joints_raw, move_time, soft_start=soft_start
                ):
                    if self._calibration_active and not self._calibration_guide_active:
                        aborted_for_calibration = True
                        break
                    corrected_segment = self._apply_offsets(
                        segment_raw, direction="correct"
                    )
                    self.commanded_joints = list(corrected_segment)
                    if self._operation_mode == "teach":
                        self.current_joints = list(corrected_segment)
                        self._last_commanded_raw = tuple(segment_raw)
                        self._update_servo_readouts()
                        self._update_visuals(self.current_joints)
                        self._record_teach_sample(corrected_segment, segment_time)
                        continue

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

                if self._operation_mode != "teach":
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
                self._mark_fault()
            finally:
                self._command_queue.task_done()
                self._mark_motion_complete()

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

        if self._calibration_active and not self._calibration_guide_active:
            return

        if self._skip_next_command:
            self._skip_next_command = False
            return

        self._mark_motion_active()
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
        configs: dict[int, ServoConfig] = {}
        for index in self.servo_configs:
            config = self._servo_config_with_multiplier(index)
            if config is not None:
                configs[index] = config
        return configs

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
