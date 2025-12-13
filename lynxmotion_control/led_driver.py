"""Hardware driver for status LEDs configured via a JSON pin map."""
from __future__ import annotations

import importlib
import importlib.util
import json
import logging
from pathlib import Path
from typing import Callable, Dict

from .status_leds import LEDState

_LOGGER = logging.getLogger(__name__)

CONFIG_PATH = Path.home() / ".config" / "lynxmotion_al5a" / "led_pins.json"
_ALLOWED_LEDS = {"red", "amber", "green", "yellow", "blue", "white"}

_PWM_FREQUENCIES = {
    "blink_slow": 1.0,
    "blink_fast": 4.0,
    "dim": 200.0,
}

_PWM_DUTY_CYCLES = {
    "blink_slow": 50.0,
    "blink_fast": 50.0,
    "dim": 22.0,
}


class _NoOpGPIO:
    """Fallback GPIO shim used when hardware access is unavailable."""

    OUT = "OUT"
    BCM = "BCM"
    HIGH = 1
    LOW = 0

    def __init__(self) -> None:
        self.setup_calls: list[tuple[int, str]] = []
        self.output_calls: list[tuple[int, int]] = []

    def setmode(self, _mode) -> None:  # pragma: no cover - trivial shim
        return

    def setup(self, pin: int, mode) -> None:  # pragma: no cover - trivial shim
        self.setup_calls.append((pin, mode))

    def output(self, pin: int, level: int) -> None:  # pragma: no cover - trivial shim
        self.output_calls.append((pin, level))

    def PWM(self, pin: int, frequency: float):  # pragma: no cover - trivial shim
        return _NoOpPWM(pin, frequency)


class _NoOpPWM:
    """Minimal PWM stand-in used for documentation builds and tests."""

    def __init__(self, _pin: int, _frequency: float) -> None:
        self.frequency = _frequency
        self.duty_cycle = 0.0
        self.active = False

    def start(self, duty_cycle: float) -> None:
        self.active = True
        self.duty_cycle = duty_cycle

    def ChangeFrequency(self, frequency: float) -> None:  # noqa: N802
        self.frequency = frequency

    def ChangeDutyCycle(self, duty_cycle: float) -> None:  # noqa: N802
        self.duty_cycle = duty_cycle

    def stop(self) -> None:
        self.active = False


class LEDDriver:
    """Encapsulates GPIO initialisation and LED driving logic."""

    def __init__(self, pin_map: Dict[str, int], gpio_module=None) -> None:
        self.pin_map = pin_map
        self.gpio = gpio_module or _load_gpio_module()
        self._pwm_channels: dict[int, object] = {}
        if not self.gpio:
            _LOGGER.info(
                "Status LED pin map present but no GPIO backend available; skipping hardware drive"
            )
            return
        self.gpio.setmode(self.gpio.BCM)
        for pin in self.pin_map.values():
            self.gpio.setup(pin, self.gpio.OUT)
        _LOGGER.info("Initialised %d status LED pins", len(self.pin_map))

    def drive(self, state: Dict[str, LEDState]) -> None:
        if not self.pin_map or not self.gpio:
            return
        for name, led_state in state.items():
            pin = self.pin_map.get(name)
            if pin is None:
                continue
            pattern = led_state.pattern
            if pattern == "off":
                self._stop_pwm(pin)
                self.gpio.output(pin, self.gpio.LOW)
                continue
            if pattern == "solid":
                self._stop_pwm(pin)
                self.gpio.output(pin, self.gpio.HIGH)
                continue
            frequency = _PWM_FREQUENCIES.get(pattern)
            duty_cycle = _PWM_DUTY_CYCLES.get(pattern)
            if frequency is None or duty_cycle is None:
                _LOGGER.warning("Unknown LED pattern '%s' for %s", pattern, name)
                continue
            pwm = self._pwm_channels.get(pin)
            if pwm is None:
                pwm = self.gpio.PWM(pin, frequency)
                pwm.start(duty_cycle)
                self._pwm_channels[pin] = pwm
            else:
                pwm.ChangeFrequency(frequency)
                pwm.ChangeDutyCycle(duty_cycle)
            self.gpio.output(pin, self.gpio.HIGH)

    def _stop_pwm(self, pin: int) -> None:
        pwm = self._pwm_channels.pop(pin, None)
        if pwm is not None:
            pwm.stop()


def load_led_pin_map(config_path: Path = CONFIG_PATH) -> Dict[str, int] | None:
    """Load and validate the user-provided LED pin map if present."""

    if not config_path.exists():
        _LOGGER.info(
            "No LED pin map found at %s; hardware LEDs will be disabled", config_path
        )
        return None
    try:
        raw = json.loads(config_path.read_text())
    except Exception as exc:  # pragma: no cover - exercised via log path
        _LOGGER.warning("Failed to parse LED pin map at %s: %s", config_path, exc)
        return None
    if not isinstance(raw, dict):
        _LOGGER.warning(
            "LED pin map at %s must be a JSON object mapping LED names to pin numbers",
            config_path,
        )
        return None
    valid_entries: dict[str, int] = {}
    for led_name, pin in raw.items():
        if led_name not in _ALLOWED_LEDS:
            _LOGGER.warning("Ignoring unsupported LED name '%s' in %s", led_name, config_path)
            continue
        if not isinstance(pin, int):
            _LOGGER.warning(
                "LED pin for %s must be an integer in %s; skipping entry", led_name, config_path
            )
            continue
        valid_entries[led_name] = pin
    if not valid_entries:
        _LOGGER.warning("No valid LED mappings found in %s; hardware LEDs disabled", config_path)
        return None
    return valid_entries


def build_drive_leds(pin_map: Dict[str, int] | None) -> Callable[[Dict[str, LEDState]], None]:
    """Construct a drive_leds callback suitable for StatusLEDController."""

    if not pin_map:
        _LOGGER.info("LED driver running in no-op mode; no pin map available")
        return lambda _state: None
    driver = LEDDriver(pin_map)
    return driver.drive


def _load_gpio_module():
    spec = importlib.util.find_spec("RPi.GPIO")
    if spec is None:
        _LOGGER.info("RPi.GPIO not available; status LEDs will not drive hardware")
        return None
    module = importlib.import_module("RPi.GPIO")
    return module


__all__ = ["LEDDriver", "build_drive_leds", "load_led_pin_map", "CONFIG_PATH"]
