"""Status LED helper following the industrial stack light scheme."""
from __future__ import annotations

from enum import Enum
from dataclasses import dataclass
from typing import Callable, Dict


@dataclass
class LEDState:
    """Represents the current drive pattern and meaning for an LED."""

    pattern: str
    meaning: str


class RobotISOState(Enum):
    """ISO-style stack light truth table for common robot states."""

    POWERED_OFF = "powered_off"
    CONNECTED_NOT_ENABLED = "connected_not_enabled"
    ENABLED_NOT_HOMED = "enabled_not_homed"
    HOMING = "homing"
    READY_IDLE = "ready_idle"
    RUNNING = "running"
    TEACH_MODE = "teach_mode"
    FAULT = "fault"


class StatusLEDController:
    """Utility for coordinating stack light and auxiliary indicators.

    The patterns map directly to the documented industrial scheme:

    - Red: fault / e-stop (blinking fast if the fault just occurred)
    - Amber: attention / interlock / transition (blinking slow while transitioning)
    - Green: ready / running (blinking slow while motion is active)
    - Yellow: teach / calibration / manual jog (blinking when actively stepping)
    - Blue: controller heartbeat and USB/host presence (blinking slow as a heartbeat)
    - White: illumination / presence (solid or PWM dim)
    """

    def __init__(self, on_change: Callable[[Dict[str, LEDState]], None] | None = None) -> None:
        self._on_change = on_change
        self._state: Dict[str, LEDState] = {
            "red": LEDState("off", "Fault / E-Stop"),
            "amber": LEDState("off", "Attention / Interlock / Transition"),
            "green": LEDState("off", "Ready / Running"),
            "yellow": LEDState("off", "Teach / Calibration / Manual jog"),
            "blue": LEDState("off", "USB / Host link / Firmware heartbeat"),
            "white": LEDState("off", "Illumination / Presence"),
        }

    def snapshot(self) -> Dict[str, LEDState]:
        """Return a shallow copy of the current LED state."""

        return {name: LEDState(state.pattern, state.meaning) for name, state in self._state.items()}

    def set_fault(self, active: bool, *, just_triggered: bool = False) -> None:
        pattern = "blink_fast" if active and just_triggered else "solid" if active else "off"
        self._set("red", pattern, "Fault / E-Stop")

    def set_attention(self, *, active: bool, transitioning: bool = False) -> None:
        pattern = "blink_slow" if active and transitioning else "solid" if active else "off"
        self._set("amber", pattern, "Attention / Interlock / Transition")

    def set_ready(self, *, active: bool, running: bool = False) -> None:
        pattern = "blink_slow" if active and running else "solid" if active else "off"
        self._set("green", pattern, "Ready / Running")

    def set_teach_mode(self, *, active: bool, blinking: bool = False) -> None:
        pattern = "blink_slow" if active and blinking else "solid" if active else "off"
        self._set("yellow", pattern, "Teach / Calibration / Manual jog")

    def set_controller_link(self, *, connected: bool, heartbeat: bool = False) -> None:
        # Per the ISO truth table we surface host presence as solid blue; heartbeat
        # is intentionally ignored to avoid mirroring the motion (green) LED.
        pattern = "solid" if connected else "off"
        self._set("blue", pattern, "USB / Host link / Firmware heartbeat")

    def set_illumination(self, *, on: bool, dim: bool = False) -> None:
        pattern = "dim" if on and dim else "solid" if on else "off"
        self._set("white", pattern, "Illumination / Presence")

    def set_iso_state(
        self,
        state: RobotISOState | str,
        *,
        teach_active: bool = False,
        teach_recording: bool = False,
        fault_just_triggered: bool = False,
    ) -> None:
        """Apply ISO-style stack light defaults for the given robot state."""

        if isinstance(state, str):
            state = RobotISOState(state)

        patterns: dict[str, str]
        if state == RobotISOState.POWERED_OFF:
            patterns = {color: "off" for color in self._state.keys()}
        elif state == RobotISOState.CONNECTED_NOT_ENABLED:
            patterns = {
                "red": "off",
                "amber": "solid",
                "green": "off",
                "blue": "solid",
                "yellow": "off",
                "white": "solid",
            }
        elif state == RobotISOState.ENABLED_NOT_HOMED:
            patterns = {
                "red": "off",
                "amber": "solid",
                "green": "off",
                "blue": "solid",
                "yellow": "off",
                "white": "solid",
            }
        elif state == RobotISOState.HOMING:
            patterns = {
                "red": "off",
                "amber": "blink_slow",
                "green": "off",
                "blue": "solid",
                "yellow": "off",
                "white": "solid",
            }
        elif state == RobotISOState.READY_IDLE:
            patterns = {
                "red": "off",
                "amber": "off",
                "green": "solid",
                "blue": "solid",
                "yellow": "off",
                "white": "solid",
            }
        elif state == RobotISOState.RUNNING:
            patterns = {
                "red": "off",
                "amber": "off",
                "green": "blink_slow",
                "blue": "solid",
                "yellow": "off",
                "white": "solid",
            }
        elif state == RobotISOState.TEACH_MODE:
            patterns = {
                "red": "off",
                "amber": "off",
                "green": "solid",
                "blue": "solid",
                "yellow": "solid",
                "white": "solid",
            }
        elif state == RobotISOState.FAULT:
            patterns = {
                "red": "blink_fast" if fault_just_triggered else "solid",
                "amber": "off",
                "green": "off",
                "blue": "solid",
                "yellow": "off",
                "white": "solid",
            }
        else:
            raise ValueError(f"Unrecognised ISO state: {state}")

        # Apply overlays after the base ISO state.
        yellow_pattern = "blink_slow" if teach_active and teach_recording else "solid"
        if teach_active:
            patterns["yellow"] = yellow_pattern

        for color, pattern in patterns.items():
            meaning = self._state[color].meaning
            self._set(color, pattern, meaning)

    def _set(self, name: str, pattern: str, meaning: str) -> None:
        current = self._state.get(name)
        if current and current.pattern == pattern and current.meaning == meaning:
            return
        self._state[name] = LEDState(pattern, meaning)
        if self._on_change:
            try:
                self._on_change(self.snapshot())
            except Exception:
                # LED updates should never break motion control; swallow callbacks that fail.
                pass


__all__ = ["LEDState", "RobotISOState", "StatusLEDController"]
