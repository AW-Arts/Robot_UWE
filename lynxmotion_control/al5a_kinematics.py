"""Kinematics and servo mapping utilities for the Lynxmotion AL5A arm."""
from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, sin, sqrt
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class ServoConfig:
    """Conversion between joint angles (radians) and servo pulses."""

    min_angle: float
    max_angle: float
    min_pulse: int
    max_pulse: int

    def clamp_angle(self, angle: float) -> float:
        return max(self.min_angle, min(self.max_angle, angle))

    def angle_to_pulse(self, angle: float) -> int:
        """Convert an angle in radians to a servo pulse width."""
        clamped = self.clamp_angle(angle)
        span_angle = self.max_angle - self.min_angle
        span_pulse = self.max_pulse - self.min_pulse
        if span_angle == 0:
            raise ValueError("Servo configuration has zero angle span")
        proportion = (clamped - self.min_angle) / span_angle
        return int(round(self.min_pulse + proportion * span_pulse))

    def pulse_to_angle(self, pulse: int) -> float:
        span_angle = self.max_angle - self.min_angle
        span_pulse = self.max_pulse - self.min_pulse
        if span_pulse == 0:
            raise ValueError("Servo configuration has zero pulse span")
        proportion = (pulse - self.min_pulse) / span_pulse
        return self.min_angle + proportion * span_angle


@dataclass(frozen=True)
class AL5ALinkLengths:
    base_height: float = 0.070  # metres
    shoulder: float = 0.105
    elbow: float = 0.105
    wrist: float = 0.082


class AL5AKinematics:
    """Planar kinematics for the Lynxmotion AL5A arm."""

    def __init__(self, links: AL5ALinkLengths | None = None) -> None:
        self.links = links or AL5ALinkLengths()

    def forward(self, joints: Sequence[float]) -> np.ndarray:
        """Return the 4x4 pose matrix of the tool tip."""
        if len(joints) < 4:
            raise ValueError("Expected at least 4 joint angles")
        base, shoulder, elbow, wrist = joints[:4]
        L = self.links

        # Base rotation around Z
        cb, sb = cos(base), sin(base)

        # Position of the wrist relative to base frame in plane
        shoulder_angle = shoulder
        elbow_angle = elbow
        wrist_angle = wrist

        # Compute planar coordinates
        z = L.base_height
        r = 0.0

        # Shoulder link
        r += L.shoulder * cos(shoulder_angle)
        z += L.shoulder * sin(shoulder_angle)

        # Elbow link
        r += L.elbow * cos(shoulder_angle + elbow_angle)
        z += L.elbow * sin(shoulder_angle + elbow_angle)

        # Wrist link / tool offset
        r += L.wrist * cos(shoulder_angle + elbow_angle + wrist_angle)
        z += L.wrist * sin(shoulder_angle + elbow_angle + wrist_angle)

        x = cb * r
        y = sb * r

        # Orientation: yaw = base, pitch = total planar angle, roll = 0
        pitch = shoulder_angle + elbow_angle + wrist_angle

        ct = cos(pitch)
        st = sin(pitch)

        rot = np.array(
            [
                [cb * ct, -sb, cb * st, 0.0],
                [sb * ct, cb, sb * st, 0.0],
                [-st, 0.0, ct, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        rot[:3, 3] = [x, y, z]
        return rot

    def inverse(self, position: Iterable[float], wrist_pitch: float) -> list[float]:
        """Inverse kinematics for XYZ position and wrist pitch.

        Parameters
        ----------
        position:
            Iterable of (x, y, z) coordinates in metres.
        wrist_pitch:
            Desired pitch of the wrist relative to the base frame (radians).
        """

        x, y, z = position
        L = self.links

        base = atan2(y, x)
        planar_radius = sqrt(x * x + y * y)

        # Position of the wrist centre after compensating the wrist link
        wx = planar_radius - L.wrist * cos(wrist_pitch)
        wz = z - L.base_height - L.wrist * sin(wrist_pitch)

        d_sq = wx * wx + wz * wz
        d = sqrt(d_sq)

        # Guard against unreachable targets
        max_reach = L.shoulder + L.elbow
        if d > max_reach + 1e-9:
            scale = max_reach / d if d != 0 else 0.0
            wx *= scale
            wz *= scale
            d_sq = wx * wx + wz * wz
            d = sqrt(d_sq)

        # Law of cosines for elbow angle
        cos_elbow = (d_sq - L.shoulder**2 - L.elbow**2) / (2 * L.shoulder * L.elbow)
        cos_elbow = min(1.0, max(-1.0, cos_elbow))
        elbow = -acos(cos_elbow)

        # Compute shoulder angle
        k1 = L.shoulder + L.elbow * cos(elbow)
        k2 = L.elbow * sin(elbow)
        shoulder = atan2(wz, wx) - atan2(k2, k1)

        wrist = wrist_pitch - shoulder - elbow

        return [base, shoulder, elbow, wrist]


DEFAULT_SERVO_CONFIGS = {
    # Channel: ServoConfig(min_angle, max_angle, min_pulse, max_pulse)
    0: ServoConfig(min_angle=-np.pi / 2, max_angle=np.pi / 2, min_pulse=500, max_pulse=2500),
    1: ServoConfig(min_angle=-0.35, max_angle=2.0, min_pulse=500, max_pulse=2500),
    2: ServoConfig(min_angle=-2.4, max_angle=0.35, min_pulse=500, max_pulse=2500),
    3: ServoConfig(min_angle=-2.0, max_angle=2.0, min_pulse=500, max_pulse=2500),
    4: ServoConfig(min_angle=-1.0, max_angle=1.0, min_pulse=800, max_pulse=2200),
}


def joints_to_pulses(joints: Sequence[float], servo_configs: dict[int, ServoConfig] | None = None) -> list[int]:
    configs = servo_configs or DEFAULT_SERVO_CONFIGS
    pulses: list[int] = []
    for channel, angle in enumerate(joints):
        if channel not in configs:
            raise KeyError(f"No servo configuration for channel {channel}")
        pulses.append(configs[channel].angle_to_pulse(angle))
    return pulses


def pulses_to_joints(pulses: Sequence[int], servo_configs: dict[int, ServoConfig] | None = None) -> list[float]:
    configs = servo_configs or DEFAULT_SERVO_CONFIGS
    joints: list[float] = []
    for channel, pulse in enumerate(pulses):
        if channel not in configs:
            raise KeyError(f"No servo configuration for channel {channel}")
        joints.append(configs[channel].pulse_to_angle(pulse))
    return joints


__all__ = [
    "AL5AKinematics",
    "AL5ALinkLengths",
    "ServoConfig",
    "DEFAULT_SERVO_CONFIGS",
    "joints_to_pulses",
    "pulses_to_joints",
]
