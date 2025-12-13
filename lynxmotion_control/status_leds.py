"""Status LED helper following the industrial stack light scheme."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict


@dataclass
class LEDState:
    """Represents the current drive pattern and meaning for an LED."""

    pattern: str
    meaning: str


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
        pattern = "blink_slow" if connected and heartbeat else "solid" if connected else "off"
        self._set("blue", pattern, "USB / Host link / Firmware heartbeat")

    def set_illumination(self, *, on: bool, dim: bool = False) -> None:
        pattern = "dim" if on and dim else "solid" if on else "off"
        self._set("white", pattern, "Illumination / Presence")

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


__all__ = ["LEDState", "StatusLEDController"]
