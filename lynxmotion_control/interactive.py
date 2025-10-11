"""Interactive matplotlib UI for commanding the Lynxmotion AL5A."""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import matplotlib
import numpy as np
from matplotlib.widgets import Button

from .al5a_kinematics import AL5AKinematics, DEFAULT_SERVO_CONFIGS


_LOGGER = logging.getLogger(__name__)


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
        ("Gripper", "HS-422/HS-225MG", "gripper open/close"),
    ]

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
        self.gripper_angle = 0.0
        self.current_joints: list[float] = [0.0] * 5

        self.target = np.array([0.18, 0.0, 0.18])
        self.drag_state = DragState()
        self.figure = plt.figure("Lynxmotion AL5A Controller")
        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_position([0.08, 0.1, 0.65, 0.8])
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

        self._create_controls()
        self.update_robot()

    def update_robot(self) -> None:
        joints = self.kin.inverse(self.target[[0, 1, 2]], self.wrist_pitch)
        self.wrist_pitch = joints[1] + joints[2] + joints[3]
        full_joints = list(joints) + [self.gripper_angle]
        self.current_joints = full_joints
        self._update_visuals(joints)
        self._update_servo_readouts()
        self.controller.move_joints(full_joints, move_time_ms=self.move_time_ms)
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

        pad_left = 0.78
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

        servo_value_left = 0.72
        servo_value_width = 0.14
        servo_button_width = 0.05
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

            minus_left = servo_value_left + servo_value_width + 0.01
            plus_left = minus_left + servo_button_width + 0.01

            minus_ax = self.figure.add_axes([minus_left, row_bottom, servo_button_width, servo_row_height])
            plus_ax = self.figure.add_axes([plus_left, row_bottom, servo_button_width, servo_row_height])

            minus_button = Button(minus_ax, "-", hovercolor="0.975")
            plus_button = Button(plus_ax, "+", hovercolor="0.975")

            minus_button.on_clicked(self._make_servo_adjust_callback(index, -math.radians(5)))
            plus_button.on_clicked(self._make_servo_adjust_callback(index, math.radians(5)))

            self.servo_buttons.extend([minus_button, plus_button])


    def _make_move_callback(self, delta: tuple[float, float, float]):
        def _callback(event) -> None:  # pragma: no cover - UI interaction
            self._nudge_target(*delta)

        return _callback

    def _make_servo_adjust_callback(self, index: int, delta: float):
        def _callback(event) -> None:  # pragma: no cover - UI interaction
            self._adjust_servo(index, delta)

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
        config = DEFAULT_SERVO_CONFIGS.get(index)
        if config is None:
            return

        updated = list(self.current_joints)
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

        if index == 4:
            self.gripper_angle = new_angle
            self.current_joints = updated
            self._update_servo_readouts()
            self.controller.move_joints(updated, move_time_ms=self.move_time_ms)
            return

        self.current_joints = updated
        forward_pose = self.kin.forward(self.current_joints)
        self.target = forward_pose[:3, 3]
        self.wrist_pitch = sum(self.current_joints[1:4])
        self._update_visuals(self.current_joints[:4])
        self._update_servo_readouts()
        self.controller.move_joints(self.current_joints, move_time_ms=self.move_time_ms)

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
            angle_deg = math.degrees(self.current_joints[idx])
            text.set_text(
                f"{name} ({model})\n{location}\nAngle: {angle_deg:.1f}°"
            )
        self.figure.canvas.draw_idle()


def run_demo(controller, move_time_ms: int = 1000) -> None:
    InteractiveArm(controller, move_time_ms=move_time_ms)
    plt.legend()
    plt.show()


__all__ = ["InteractiveArm", "run_demo"]
