"""Interactive matplotlib UI for commanding the Lynxmotion AL5A."""
from __future__ import annotations

import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from .al5a_kinematics import AL5AKinematics


@dataclass
class DragState:
    dragging: bool = False
    last_event: object | None = None


class InteractiveArm:
    """Matplotlib based interactive controller."""

    def __init__(
        self,
        controller,
        kinematics: AL5AKinematics | None = None,
        wrist_pitch: float = math.radians(30),
        move_time_ms: int = 1000,
    ) -> None:
        self.controller = controller
        self.kin = kinematics or AL5AKinematics()
        self.wrist_pitch = wrist_pitch
        self.move_time_ms = move_time_ms

        self.target = np.array([0.18, 0.0, 0.18])
        self.drag_state = DragState()
        self.figure = plt.figure("Lynxmotion AL5A Controller")
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-0.25, 0.25)
        self.ax.set_ylim(-0.25, 0.25)
        self.ax.grid(True)

        (self.base_line,) = self.ax.plot([], [], "-o", lw=3)
        self.target_artist = self.ax.scatter(
            [self.target[0]], [self.target[1]], c="red", s=100, label="Target"
        )
        self.text = self.ax.text(
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

        self.update_robot()

    def update_robot(self) -> None:
        joints = self.kin.inverse(self.target[[0, 1, 2]], self.wrist_pitch)
        shoulder = joints[1]
        elbow = joints[2]

        # Compute planar geometry for display (top-down view of XY plane)
        base = joints[0]
        base_point = np.array([0.0, 0.0])
        shoulder_point = np.array(
            [
                math.cos(base) * self.kin.links.shoulder * math.cos(shoulder),
                math.sin(base) * self.kin.links.shoulder * math.cos(shoulder),
            ]
        )
        elbow_angle = shoulder + elbow
        elbow_point = shoulder_point + np.array(
            [
                math.cos(base) * self.kin.links.elbow * math.cos(elbow_angle),
                math.sin(base) * self.kin.links.elbow * math.cos(elbow_angle),
            ]
        )
        wrist_angle = elbow_angle + joints[3]
        wrist_point = elbow_point + np.array(
            [
                math.cos(base) * self.kin.links.wrist * math.cos(wrist_angle),
                math.sin(base) * self.kin.links.wrist * math.cos(wrist_angle),
            ]
        )

        xs = [base_point[0], shoulder_point[0], elbow_point[0], wrist_point[0]]
        ys = [base_point[1], shoulder_point[1], elbow_point[1], wrist_point[1]]
        self.base_line.set_data(xs, ys)

        self.target_artist.set_offsets([[self.target[0], self.target[1]]])
        self.text.set_text(
            f"Target: x={self.target[0]:.3f} m, y={self.target[1]:.3f} m, z={self.target[2]:.3f} m\n"
            + f"Wrist pitch: {math.degrees(self.wrist_pitch):.1f}°"
        )

        self.controller.move_joints(joints, move_time_ms=self.move_time_ms)
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
        # makes grabbing the red target dot consistent across platforms.
        tolerance = 0.01  # metres
        distance = math.hypot(event.xdata - self.target[0], event.ydata - self.target[1])
        if distance <= tolerance:
            self.drag_state.dragging = True
            self.drag_state.last_event = event

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


def run_demo(controller, move_time_ms: int = 1000) -> None:
    InteractiveArm(controller, move_time_ms=move_time_ms)
    plt.legend()
    plt.show()


__all__ = ["InteractiveArm", "run_demo"]
