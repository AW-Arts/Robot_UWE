"""Utilities for interactive control of the Lynxmotion AL5A arm."""

from importlib import import_module
from typing import Any

from .al5a_kinematics import (
    AL5AKinematics,
    AL5ALinkLengths,
    DEFAULT_SERVO_CONFIGS,
    DEFAULT_SERVO_CHANNELS,
    ServoConfig,
    joints_to_pulses,
    pulses_to_joints,
)
from .serial_comm import AL5ASerialController, PrintController
from .status_leds import LEDState, StatusLEDController

__all__ = [
    "AL5AKinematics",
    "AL5ALinkLengths",
    "DEFAULT_SERVO_CONFIGS",
    "DEFAULT_SERVO_CHANNELS",
    "ServoConfig",
    "joints_to_pulses",
    "pulses_to_joints",
    "InteractiveArm",
    "run_demo",
    "AL5ASerialController",
    "PrintController",
    "LEDState",
    "StatusLEDController",
]


def __getattr__(name: str) -> Any:
    if name in {"InteractiveArm", "run_demo"}:
        module = import_module(".interactive", __name__)
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(name)
