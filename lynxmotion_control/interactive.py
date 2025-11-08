"""Interactive matplotlib UI for commanding the Lynxmotion AL5A."""
from __future__ import annotations

import json
import logging
import math
import queue
import threading
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.widgets import Button, TextBox

from .al5a_kinematics import (
    AL5AKinematics,
    DEFAULT_SERVO_CHANNELS,
    DEFAULT_SERVO_CONFIGS,
    ServoConfig,
)


_LOGGER = logging.getLogger(__name__)


CALIBRATION_CONFIG_PATH = Path.home() / ".config" / "lynxmotion_al5a" / "servo_offsets.json"


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
        wrist_pitch: float = math.radians(30),
        move_time_ms: int = 1000,
        step_xy: float = 0.01,
        step_z: float = 0.01,
    ) -> None:
        self.controller = controller
        self.kin = kinematics or AL5AKinematics()
        self.wrist_pitch = wrist_pitch
        self.move_time_ms = move_time_ms
        self.step_xy = step_xy
        self.step_z = step_z
        self._base_servo_configs = deepcopy(DEFAULT_SERVO_CONFIGS)
        self.servo_configs: dict[int, ServoConfig] = dict(self._base_servo_configs)
        self.servo_inversions: list[bool] = [False] * len(self._SERVO_METADATA)
        self._invert_button_inactive_color = "0.85"
        self._invert_button_active_color = "#90ee90"
        self._calibration_path = CALIBRATION_CONFIG_PATH
        self.zero_reference: dict[int, float] = {
            index: angle for index, angle in enumerate(_DEFAULT_VERTICAL_JOINTS)
        }
        self._show_raw_angles = False
        self._raw_angle_button: Button | None = None
        self._soft_start_min_time_ms = 4_000
        self._soft_start_min_segments = 18
        self._home_move_time_ms = 4_000
        self.servo_offsets: dict[int, float] = self._load_calibration_data()

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
            self.wrist_pitch = (
                self.current_joints[1]
                + self.current_joints[2]
                + self.current_joints[3]
            )
            forward_pose = self.kin.forward(self.current_joints)
            target_position = forward_pose[:3, 3]
        else:
            target_position = np.array([0.18, 0.0, 0.18])

        self.target = np.array(target_position, dtype=float)
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
        ] = (
            queue.Queue(maxsize=1)
        )
        self._command_thread = threading.Thread(
            target=self._command_worker, name="al5a-command-worker", daemon=True
        )
        self._command_thread.start()

        self.drag_state = DragState()
        self.figure = plt.figure("Lynxmotion AL5A Controller")
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_position([0.05, 0.12, 0.5, 0.78])
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_xlim(-0.25, 0.25)
        self.ax.set_ylim(-0.25, 0.25)
        self.ax.set_zlim(0.0, 0.35)
        try:
            self.ax.set_box_aspect((1.0, 1.0, 0.6))
        except AttributeError:  # Matplotlib < 3.4
            pass
        self.ax.view_init(elev=25, azim=-60)

        (self.base_line,) = self.ax.plot([], [], [], "-o", lw=3)
        self.target_artist = self.ax.scatter(
            [self.target[0]],
            [self.target[1]],
            [self.target[2]],
            c="red",
            s=100,
            label="Target",
        )
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

        self._calibration_active = False
        self._calibration_button: Button | None = None
        self._set_vertical_button: Button | None = None
        self._calibration_timer = None

        self._create_controls()
        self._initialise_from_feedback()
        self._skip_next_command = True
        self.update_robot()
        self._start_homing_sequence()

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

        self.commanded_joints = list(self.current_joints)
        self.feedback_joints = list(self.current_joints)
        self._last_commanded_raw = tuple(
            self._apply_offsets(self.current_joints, direction="raw")
        )

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
                self.wrist_pitch = (
                    self.current_joints[1]
                    + self.current_joints[2]
                    + self.current_joints[3]
                )
                self._update_visuals(self.current_joints)

        self._update_servo_readouts()
        self._initial_feedback_move_pending = True

    def _start_homing_sequence(self) -> None:
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
                self.target = pose[:3, 3]
                self.wrist_pitch = sum(clamped_home[1:4])
                self._update_visuals(clamped_home[:4])
        if len(clamped_home) >= 5:
            self.wrist_rotation = clamped_home[4]
        if len(clamped_home) >= 6:
            self.gripper_angle = clamped_home[5]

        self._update_servo_readouts()
        self._send_move_command(
            clamped_home, move_time_ms=self._home_move_time_ms, soft_start=True
        )

    def update_robot(self) -> None:
        joints = self.kin.inverse(self.target[[0, 1, 2]], self.wrist_pitch)
        joints = self._clamp_joint_list(list(joints))
        self.wrist_pitch = joints[1] + joints[2] + joints[3]
        full_joints = joints + [self.wrist_rotation, self.gripper_angle]
        full_joints = self._clamp_joint_list(full_joints)
        self.commanded_joints = list(full_joints)
        self._update_visuals(joints)
        self._update_servo_readouts()
        self._send_move_command(full_joints, move_time_ms=self.move_time_ms)
        self.figure.canvas.draw_idle()

    def _on_press(self, event) -> None:
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
        if distance <= tolerance:
            self.drag_state.dragging = True
            self.drag_state.last_event = event
            return

        # If the click is outside the tolerance treat it as a request to jump
        # the target to the clicked location.  This provides an easy way to
        # reposition the end-effector even if the user misses the dot on the
        # first try, after which standard dragging takes over.
        self.target[0] = event.xdata
        self.target[1] = event.ydata
        self.drag_state.dragging = True
        self.drag_state.last_event = event
        self.update_robot()

    def _on_release(self, event) -> None:
        self.drag_state.dragging = False
        self.drag_state.last_event = None

    def _on_motion(self, event) -> None:
        if not self.drag_state.dragging or event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        self.target[0] = event.xdata
        self.target[1] = event.ydata
        self.update_robot()

    def _on_scroll(self, event) -> None:
        if event.inaxes != self.ax:
            return
        step = 0.01 if event.button == "up" else -0.01
        self.target[2] = np.clip(
            self.target[2] + step,
            self.kin.links.base_height + 0.02,
            self.kin.links.base_height + self.kin.links.shoulder + self.kin.links.elbow,
        )
        self.update_robot()

    # ------------------------------------------------------------------
    # UI helpers
    def _create_controls(self) -> None:
        """Create on-figure UI elements such as the D-pad."""

        pad_left = 0.56
        pad_bottom = 0.18
        pad_size = 0.07
        pad_gap = 0.005

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

        self.buttons: dict[str, Button] = {}

        for name, (x, y, label, delta) in button_defs.items():
            axes = self.figure.add_axes([x, y, pad_size, pad_size])
            button = Button(axes, label)
            button.on_clicked(self._make_move_callback(delta))
            self.buttons[name] = button

        centre_ax = self.figure.add_axes([pad_left, pad_bottom, pad_size, pad_size])
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

        self.servo_value_texts: list = []
        self.servo_buttons: list[Button] = []
        self.servo_invert_buttons: list[Button] = []
        self.servo_limit_boxes_min: list[TextBox] = []
        self.servo_limit_boxes_max: list[TextBox] = []

        servo_value_left = 0.56
        servo_value_width = 0.16
        servo_button_width = 0.04
        servo_row_height = 0.055
        servo_gap = 0.01
        servo_top = 0.9

        for index, (name, model, location) in enumerate(self._SERVO_METADATA):
            row_bottom = servo_top - servo_row_height - index * (servo_row_height + servo_gap)

            value_ax = self.figure.add_axes(
                [servo_value_left, row_bottom, servo_value_width, servo_row_height]
            )
            value_ax.axis("off")
            text = value_ax.text(
                0.0,
                0.5,
                f"{name} ({model})\n{location}\nAngle: 0.0°",
                va="center",
                ha="left",
                fontsize=8,
                transform=value_ax.transAxes,
            )
            self.servo_value_texts.append(text)

            minus_left = servo_value_left + servo_value_width + 0.008
            plus_left = minus_left + servo_button_width + 0.008
            invert_left = plus_left + servo_button_width + 0.008
            limit_left = invert_left + servo_button_width + 0.012
            limit_width = 0.05
            max_left = limit_left + limit_width + 0.008

            minus_ax = self.figure.add_axes(
                [minus_left, row_bottom, servo_button_width, servo_row_height]
            )
            plus_ax = self.figure.add_axes(
                [plus_left, row_bottom, servo_button_width, servo_row_height]
            )
            invert_ax = self.figure.add_axes(
                [invert_left, row_bottom, servo_button_width, servo_row_height]
            )
            min_ax = self.figure.add_axes([limit_left, row_bottom, limit_width, servo_row_height])
            max_ax = self.figure.add_axes([max_left, row_bottom, limit_width, servo_row_height])

            minus_button = Button(minus_ax, "-", hovercolor="0.975")
            plus_button = Button(plus_ax, "+", hovercolor="0.975")
            invert_button = Button(invert_ax, "Inv", hovercolor="0.975")
            min_box = TextBox(
                min_ax,
                "Min°",
                initial=f"{math.degrees(self.servo_configs[index].min_angle):.0f}",
            )
            max_box = TextBox(
                max_ax,
                "Max°",
                initial=f"{math.degrees(self.servo_configs[index].max_angle):.0f}",
            )

            minus_button.on_clicked(self._make_servo_adjust_callback(index, -math.radians(5)))
            plus_button.on_clicked(self._make_servo_adjust_callback(index, math.radians(5)))
            invert_button.on_clicked(self._make_inversion_toggle_callback(index))
            min_box.on_submit(self._make_limit_submit_callback(index, "min"))
            max_box.on_submit(self._make_limit_submit_callback(index, "max"))

            self.servo_buttons.extend([minus_button, plus_button])
            self.servo_invert_buttons.append(invert_button)
            self.servo_limit_boxes_min.append(min_box)
            self.servo_limit_boxes_max.append(max_box)
            self._update_inversion_button_visual(index)

        calibration_left = servo_value_left
        calibration_bottom = pad_bottom - 2 * (pad_size + pad_gap)
        calibration_width = 0.12
        calibration_height = 0.045

        calibrate_ax = self.figure.add_axes(
            [calibration_left, calibration_bottom, calibration_width, calibration_height]
        )
        self._calibration_button = Button(calibrate_ax, "Calibrate", hovercolor="0.95")
        self._calibration_button.on_clicked(self._toggle_calibration)

        set_vertical_left = calibration_left + calibration_width + 0.01
        set_vertical_ax = self.figure.add_axes(
            [set_vertical_left, calibration_bottom, calibration_width, calibration_height]
        )
        self._set_vertical_button = Button(set_vertical_ax, "Set vertical", hovercolor="0.95")
        self._set_vertical_button.on_clicked(self._handle_set_vertical)
        self._update_calibration_button_visual()

        raw_toggle_left = set_vertical_left + calibration_width + 0.01
        raw_toggle_ax = self.figure.add_axes(
            [raw_toggle_left, calibration_bottom, calibration_width, calibration_height]
        )
        self._raw_angle_button = Button(raw_toggle_ax, "Show raw", hovercolor="0.95")
        self._raw_angle_button.on_clicked(self._toggle_raw_angle_display)
        self._update_raw_angle_button_visual()


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
                _LOGGER.warning("Ignoring invalid %s limit for %s (%s)", bound, name, model)
                self._update_limit_box_display(index)
                return

            radians_value = math.radians(value)
            if bound == "min":
                self._update_servo_limit(index, min_angle=radians_value)
            else:
                self._update_servo_limit(index, max_angle=radians_value)

        return _callback

    def _nudge_target(self, dx: float, dy: float, dz: float) -> None:
        limits = (
            (-0.25, 0.25),
            (-0.25, 0.25),
            (
                self.kin.links.base_height + 0.02,
                self.kin.links.base_height
                + self.kin.links.shoulder
                + self.kin.links.elbow,
            ),
        )

        self.target[0] = np.clip(self.target[0] + dx, *limits[0])
        self.target[1] = np.clip(self.target[1] + dy, *limits[1])
        self.target[2] = np.clip(self.target[2] + dz, *limits[2])
        self.update_robot()

    def _adjust_servo(self, index: int, delta: float) -> None:
        config = self.servo_configs.get(index)
        if config is None:
            return

        source = self.feedback_joints or self.commanded_joints
        if not source:
            source = self.current_joints
        updated = list(source)
        old_angle = updated[index]
        new_angle = config.clamp_angle(old_angle + delta)
        if math.isclose(new_angle, old_angle, abs_tol=1e-6):
            name, model, _ = self._SERVO_METADATA[index]
            _LOGGER.warning(
                "%s (%s) servo adjustment hit the configured limit (%.1f° to %.1f°).",
                name,
                model,
                math.degrees(config.min_angle),
                math.degrees(config.max_angle),
            )
        updated[index] = new_angle

        if index >= 4:
            if index == 4:
                self.wrist_rotation = new_angle
            else:
                self.gripper_angle = new_angle
            self.commanded_joints = list(updated)
            self._update_servo_readouts()
            self._send_move_command(updated, move_time_ms=self.move_time_ms)
            return

        self.commanded_joints = list(updated)
        forward_pose = self.kin.forward(self.commanded_joints)
        self.target = forward_pose[:3, 3]
        self.wrist_pitch = sum(self.commanded_joints[1:4])
        self._update_visuals(self.commanded_joints[:4])
        self._update_servo_readouts()
        self._send_move_command(self.commanded_joints, move_time_ms=self.move_time_ms)

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
        min_box.set_val(f"{math.degrees(base_config.min_angle):.1f}")
        max_box.set_val(f"{math.degrees(base_config.max_angle):.1f}")
        try:
            min_box.eventson = True
            max_box.eventson = True
        except AttributeError:  # pragma: no cover - depends on Matplotlib
            pass

    def _clamp_joint_list(
        self, joints: list[float] | tuple[float, ...]
    ) -> list[float]:
        clamped = list(joints)
        for idx, angle in enumerate(clamped):
            config = self.servo_configs.get(idx)
            if config is not None:
                clamped[idx] = config.clamp_angle(angle)
        return clamped

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

        if isinstance(data, dict) and (
            "offsets" in data or "vertical_angles" in data
        ):
            offsets_candidate = data.get("offsets")
            if isinstance(offsets_candidate, dict):
                offsets_raw = offsets_candidate
            vertical_candidate = data.get("vertical_angles")
            if isinstance(vertical_candidate, dict):
                vertical_raw = vertical_candidate
            inversion_candidate = data.get("inverted")
            if isinstance(inversion_candidate, dict):
                inversion_raw = inversion_candidate
        elif isinstance(data, dict):
            offsets_raw = data

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

        return offsets

    def _save_calibration_data(self) -> None:
        path = self._calibration_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            serialisable = {
                "offsets": {str(idx): offset for idx, offset in self.servo_offsets.items()},
                "vertical_angles": {
                    str(idx): angle for idx, angle in self.zero_reference.items()
                },
                "inverted": {
                    str(idx): state for idx, state in enumerate(self.servo_inversions)
                },
            }
            path.write_text(json.dumps(serialisable, indent=2, sort_keys=True))
        except Exception:  # pragma: no cover - configuration robustness
            _LOGGER.warning(
                "Failed to persist calibration data to %s", path, exc_info=True
            )

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

        self._update_raw_angle_button_visual()

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

    def _toggle_calibration(self, _event=None) -> None:  # pragma: no cover - UI interaction
        if self._calibration_active:
            self._exit_calibration_mode()
        else:
            self._enter_calibration_mode()

    def _toggle_raw_angle_display(self, _event=None) -> None:  # pragma: no cover - UI interaction
        self._show_raw_angles = not self._show_raw_angles
        self._update_raw_angle_button_visual()
        self._update_servo_readouts()

    def _enter_calibration_mode(self) -> None:
        if self._calibration_active:
            return
        self._calibration_active = True
        relax = getattr(self.controller, "relax_servos", None)
        if callable(relax):
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
                self._calibration_timer.stop()
            except Exception:  # pragma: no cover - backend specific
                pass
        self._calibration_timer = self.figure.canvas.new_timer(interval=300)
        self._calibration_timer.add_callback(self._poll_calibration_feedback)
        self._calibration_timer.start()
        self._update_calibration_button_visual()

    def _exit_calibration_mode(self) -> None:
        if not self._calibration_active:
            return
        self._calibration_active = False
        if self._calibration_timer is not None:
            try:
                self._calibration_timer.stop()
            except Exception:  # pragma: no cover - backend specific
                pass
            self._calibration_timer = None
        self._update_calibration_button_visual()

    def _poll_calibration_feedback(self) -> None:
        if not self._calibration_active:
            return
        feedback_raw = self._read_feedback_from_controller()
        if feedback_raw:
            corrected = self._apply_offsets(feedback_raw, direction="correct")
            self._apply_feedback(corrected)

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
        for idx, actual in enumerate(source):
            target_angle = desired[idx]
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
            self.wrist_pitch = (
                self.current_joints[1]
                + self.current_joints[2]
                + self.current_joints[3]
            )
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
        self._update_servo_readouts()
        if len(self.current_joints) >= 4:
            self._update_visuals(self.current_joints)

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

    def _update_visuals(self, joints: list[float]) -> None:
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
        self.base_line.set_data(xs, ys)
        self.base_line.set_3d_properties(zs)

        self.target_artist._offsets3d = (
            [self.target[0]],
            [self.target[1]],
            [self.target[2]],
        )
        self.text.set_text(
            f"Target: x={self.target[0]:.3f} m, y={self.target[1]:.3f} m, z={self.target[2]:.3f} m\n"
            + f"Wrist pitch: {math.degrees(self.wrist_pitch):.1f}°"
        )

    def _update_servo_readouts(self) -> None:
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
            config = self._base_servo_configs.get(idx)
            limits_text = ""
            if config is not None:
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
        self.figure.canvas.draw_idle()

    def _update_servo_limit(
        self,
        index: int,
        *,
        min_angle: float | None = None,
        max_angle: float | None = None,
    ) -> None:
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

        def _clamp_list(values: list[float] | None) -> None:
            if values is None:
                return
            if index >= len(values):
                return
            values[index] = self.servo_configs[index].clamp_angle(values[index])

        _clamp_list(self.current_joints)
        _clamp_list(self.commanded_joints)
        _clamp_list(self.feedback_joints)
        self._last_commanded_raw = tuple(
            self._apply_offsets(self.commanded_joints, direction="raw")
        )
        self._update_servo_readouts()
        self._update_limit_box_display(index)

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
                    self.feedback_joints = None
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
        soft_start: bool = False,
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

        self.commanded_joints = list(joints)
        if not self._calibration_active:
            self.feedback_joints = None
        self._update_servo_readouts()

        if self._calibration_active:
            return

        if self._skip_next_command:
            self._skip_next_command = False
            return

        raw_command = tuple(self._apply_offsets(joints, direction="raw"))
        command = (raw_command, adjusted_move_time, soft_start)
        try:
            self._command_queue.put_nowait(command)
        except queue.Full:
            try:
                self._command_queue.get_nowait()
                self._command_queue.task_done()
            except queue.Empty:  # pragma: no cover - defensive
                pass
            self._command_queue.put_nowait(command)

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
    InteractiveArm(controller, move_time_ms=move_time_ms)
    plt.legend()
    plt.show()


__all__ = ["InteractiveArm", "run_demo"]
