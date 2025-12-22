"""Hardware drivers for status LEDs configured via a JSON pin map or SSC-32."""
from __future__ import annotations

import importlib
import importlib.util
import json
import logging
import shutil
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Dict

from .config_paths import CONFIG_BUNDLE_DIR, CONFIG_ROOT

from .serial_comm import SSC32Command
from .status_leds import LEDState

_LOGGER = logging.getLogger(__name__)

CONFIG_PATH = CONFIG_ROOT / "led_pins.json"
_ALLOWED_LEDS = {"red", "amber", "green", "yellow", "blue", "white"}
_DEFAULT_SSC32_PULSES = {"off": 0, "on": 2000, "dim": 1300}
_DEFAULT_SSC32_BLINK = {"slow_hz": 1.0, "fast_hz": 4.0}
_DEFAULT_SSC32_RESERVED = set(range(0, 8))
_NOOP_CONFIG: dict[str, object] = {"backend": "noop"}

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
        self._last_state_snapshot: Dict[str, LEDState] = {}
        self._log_interval_s = 5.0
        self._log_stop_event = threading.Event()
        self._log_thread = threading.Thread(
            target=self._log_signals_periodically,
            name="led-gpio-signal-logger",
            daemon=True,
        )
        if not self.gpio:
            _LOGGER.info(
                "Status LED pin map present but no GPIO backend available; skipping hardware drive"
            )
        self._log_thread.start()
        if not self.gpio:
            return
        self.gpio.setmode(self.gpio.BCM)
        for pin in self.pin_map.values():
            self.gpio.setup(pin, self.gpio.OUT)
        _LOGGER.info("Initialised %d status LED pins", len(self.pin_map))

    def drive(self, state: Dict[str, LEDState]) -> None:
        if not self.pin_map or not self.gpio:
            return
        self._last_state_snapshot = {
            name: LEDState(led_state.pattern, led_state.meaning)
            for name, led_state in state.items()
        }
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

    def _describe_gpio_signal(self, pattern: str) -> str:
        if pattern == "off":
            return "LOW (off)"
        if pattern == "solid":
            return "HIGH (solid)"
        frequency = _PWM_FREQUENCIES.get(pattern)
        duty_cycle = _PWM_DUTY_CYCLES.get(pattern)
        if frequency is not None and duty_cycle is not None:
            return f"PWM {frequency:.2f} Hz at {duty_cycle:.1f}% duty"
        return f"Unknown pattern '{pattern}'"

    def _log_signals_periodically(self) -> None:
        while not self._log_stop_event.is_set():
            if self.pin_map and self._last_state_snapshot:
                entries = []
                for name, led_state in sorted(self._last_state_snapshot.items()):
                    pin = self.pin_map.get(name)
                    if pin is None:
                        continue
                    signal = self._describe_gpio_signal(led_state.pattern)
                    entries.append(
                        f"{name}: pin {pin}, pattern={led_state.pattern}, signal={signal}"
                    )
                if entries:
                    _LOGGER.info("GPIO LED pin signals -> %s", "; ".join(entries))
            self._log_stop_event.wait(self._log_interval_s)


def load_led_pin_map(config_path: Path = CONFIG_PATH) -> Dict[str, int] | None:
    """Load and validate the user-provided LED pin map if present."""

    _LOGGER.info(
        "`load_led_pin_map` is deprecated; use `load_led_config` and include a backend"
    )

    config = load_led_config(config_path)
    if config.get("backend") != "gpio":
        return None
    return config.get("pin_map")  # type: ignore[return-value]


def load_led_config(config_path: Path = CONFIG_PATH) -> Dict[str, object]:
    """Load and validate the LED driver configuration.

    The loader accepts both the legacy GPIO-only JSON map and the new structured
    configuration that supports SSC-32(U) PWM outputs. Missing or invalid files
    cause the LEDs to run in a no-op mode while still updating logical state.
    """

    if not config_path.exists():
        bundled_default = CONFIG_BUNDLE_DIR / config_path.name
        if bundled_default.exists():
            try:
                config_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(bundled_default, config_path)
                _LOGGER.info(
                    "Seeded default LED configuration from %s to %s",
                    bundled_default,
                    config_path,
                )
            except Exception as exc:  # pragma: no cover - depends on filesystem state
                _LOGGER.warning(
                    "Failed to seed default LED configuration from %s: %s; hardware LEDs will be disabled",
                    bundled_default,
                    exc,
                )
                return _NOOP_CONFIG
        else:
            _LOGGER.info(
                "No LED pin map found at %s; hardware LEDs will be disabled", config_path
            )
            return _NOOP_CONFIG
    try:
        config_text = config_path.read_text()
        _LOGGER.info("Reading LED config from %s", config_path)
        for idx, line in enumerate(config_text.splitlines(), start=1):
            _LOGGER.info("LED config line %d: %s", idx, line)
        raw = json.loads(config_text)
    except Exception as exc:  # pragma: no cover - exercised via log path
        _LOGGER.warning("Failed to parse LED pin map at %s: %s", config_path, exc)
        return _NOOP_CONFIG
    if not isinstance(raw, dict):
        _LOGGER.warning(
            "LED configuration at %s must be a JSON object; disabling hardware LEDs",
            config_path,
        )
        return _NOOP_CONFIG
    if "backend" not in raw:
        config = _parse_legacy_gpio_config(raw, config_path)
    else:
        backend = raw.get("backend")
        if backend == "gpio":
            config = _parse_gpio_config(raw, config_path)
        elif backend == "ssc32_pwm_led":
            config = _parse_ssc32_config(raw, config_path)
        else:
            _LOGGER.warning(
                "Unsupported LED backend '%s' in %s; disabling hardware LEDs", backend, config_path
            )
            return _NOOP_CONFIG

    config["_config_path"] = str(config_path)
    _log_loaded_config(config)
    return config


def build_drive_leds(
    config: Dict[str, object],
    *,
    serial_writer: Callable[[bytes], None] | None = None,
) -> Callable[[Dict[str, LEDState]], None]:
    """Construct a drive_leds callback suitable for StatusLEDController."""

    backend = config.get("backend")
    if backend == "gpio":
        pin_map = config.get("pin_map")
        if not pin_map:
            _LOGGER.info("LED driver running in no-op mode; GPIO pin map missing")
            return lambda _state: None
        driver = LEDDriver(pin_map)  # type: ignore[arg-type]
        return driver.drive
    if backend == "ssc32_pwm_led":
        channel_map = config.get("led_channels")
        if not channel_map:
            _LOGGER.info("LED driver running in no-op mode; no SSC-32 channel map")
            return lambda _state: None
        if serial_writer is None:
            _LOGGER.info(
                "SSC-32 LED backend requested but no serial writer provided; running in no-op mode"
            )
            return lambda _state: None
        pulses = config.get("pulse_us", _DEFAULT_SSC32_PULSES)
        blink = config.get("blink", _DEFAULT_SSC32_BLINK)
        reserved = set(config.get("servo_reserved_channels", _DEFAULT_SSC32_RESERVED))
        driver = SSC32PWMLEDDriver(
            channel_map=channel_map,  # type: ignore[arg-type]
            reserved_channels=reserved,
            pulse_us=pulses,  # type: ignore[arg-type]
            blink=blink,  # type: ignore[arg-type]
            serial_writer=serial_writer,
        )
        return driver.drive
    _LOGGER.info("LED driver running in no-op mode; backend %s not configured", backend)
    return lambda _state: None


def _load_gpio_module():
    if not sys.platform.startswith("linux"):
        _LOGGER.info(
            "RPi.GPIO is only available on Raspberry Pi; skipping hardware LEDs on %s",
            sys.platform,
        )
        return None

    spec = importlib.util.find_spec("RPi.GPIO")
    if spec is None:
        _LOGGER.info("RPi.GPIO not available; status LEDs will not drive hardware")
        return None

    try:
        module = importlib.import_module("RPi.GPIO")
    except ImportError:
        _LOGGER.info("Failed to import RPi.GPIO; status LEDs will not drive hardware")
        return None

    return module


def _parse_legacy_gpio_config(raw: dict, config_path: Path) -> dict[str, object]:
    _LOGGER.info(
        "Using legacy LED pin map format in %s; consider adding a 'backend' field",
        config_path,
    )
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
        return _NOOP_CONFIG
    return {"backend": "gpio", "pin_map": valid_entries}


def _parse_gpio_config(raw: dict, config_path: Path) -> dict[str, object]:
    pin_map = raw.get("pin_map")
    if not isinstance(pin_map, dict):
        _LOGGER.warning(
            "GPIO LED backend in %s requires a 'pin_map' object; disabling hardware LEDs",
            config_path,
        )
        return _NOOP_CONFIG
    return _parse_legacy_gpio_config(pin_map, config_path)


def _parse_ssc32_config(raw: dict, config_path: Path) -> dict[str, object]:
    led_channels = raw.get("led_channels")
    if not isinstance(led_channels, dict):
        _LOGGER.warning(
            "SSC-32 LED backend in %s requires an 'led_channels' object; disabling hardware LEDs",
            config_path,
        )
        return _NOOP_CONFIG
    validated_channels: dict[str, int] = {}
    for name, channel in led_channels.items():
        if name not in _ALLOWED_LEDS:
            _LOGGER.warning("Ignoring unsupported LED name '%s' in %s", name, config_path)
            continue
        if not isinstance(channel, int) or channel < 0:
            _LOGGER.warning(
                "LED channel for %s must be a non-negative integer in %s", name, config_path
            )
            continue
        validated_channels[name] = channel
    if not validated_channels:
        _LOGGER.warning("No valid SSC-32 LED channels defined in %s; hardware LEDs disabled", config_path)
        return _NOOP_CONFIG
    pulses = raw.get("pulse_us", _DEFAULT_SSC32_PULSES)
    blink = raw.get("blink", _DEFAULT_SSC32_BLINK)
    reserved = set(raw.get("servo_reserved_channels", _DEFAULT_SSC32_RESERVED))
    return {
        "backend": "ssc32_pwm_led",
        "led_channels": validated_channels,
        "pulse_us": {
            "off": int(pulses.get("off", _DEFAULT_SSC32_PULSES["off"])),
            "on": int(pulses.get("on", _DEFAULT_SSC32_PULSES["on"])),
            "dim": int(pulses.get("dim", _DEFAULT_SSC32_PULSES["dim"])),
        },
        "blink": {
            "slow_hz": float(blink.get("slow_hz", _DEFAULT_SSC32_BLINK["slow_hz"])),
            "fast_hz": float(blink.get("fast_hz", _DEFAULT_SSC32_BLINK["fast_hz"])),
        },
        "servo_reserved_channels": reserved,
    }


def _log_loaded_config(config: dict[str, object]) -> None:
    backend = config.get("backend", "noop")
    config_path = config.get("_config_path")
    if backend == "ssc32_pwm_led":
        channels = config.get("led_channels", {})
        pulses = config.get("pulse_us", {})
        blink = config.get("blink", {})
        reserved = config.get("servo_reserved_channels", set())
        _LOGGER.info(
            "Loaded SSC-32 LED config from %s -> channels=%s, pulses=%s, blink=%s, reserved=%s",
            config_path,
            channels,
            pulses,
            blink,
            reserved,
        )
    elif backend == "gpio":
        pin_map = config.get("pin_map", {})
        _LOGGER.info("Loaded GPIO LED config from %s -> pins=%s", config_path, pin_map)
    else:
        _LOGGER.info("Loaded LED config from %s -> backend=%s (no hardware drive)", config_path, backend)


class SSC32PWMLEDDriver:
    """Drive LEDs using SSC-32(U) servo PWM channels.

    This backend converts logical LED patterns into servo pulse widths following
    the prototype scheme described in ``docs/status_led_plan.md``. Blink
    patterns are implemented via a background timer that toggles between off/on
    pulse widths.
    """

    def __init__(
        self,
        *,
        channel_map: Dict[str, int],
        reserved_channels: set[int],
        pulse_us: Dict[str, int],
        blink: Dict[str, float],
        serial_writer: Callable[[bytes], None],
        tick_s: float = 0.05,
    ) -> None:
        self.channel_map = self._filter_channel_map(channel_map, reserved_channels)
        self.pulse_us = {**_DEFAULT_SSC32_PULSES, **pulse_us}
        self.blink = {**_DEFAULT_SSC32_BLINK, **blink}
        self.serial_writer = serial_writer
        self.tick_s = tick_s
        self._patterns: dict[str, str] = {name: "off" for name in self.channel_map}
        self._lock = threading.Lock()
        self._update_event = threading.Event()
        self._stop_event = threading.Event()
        self._last_pulses: dict[int, int] = {}
        self._last_log_time = 0.0
        self._log_interval_s = 5.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def drive(self, state: Dict[str, LEDState]) -> None:
        with self._lock:
            updated = False
            for name, led_state in state.items():
                if name not in self.channel_map:
                    continue
                pattern = led_state.pattern
                if self._patterns.get(name) != pattern:
                    self._patterns[name] = pattern
                    updated = True
            if updated:
                self._update_event.set()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            now = time.monotonic()
            pulses = self._compute_pulses(now)
            if pulses != self._last_pulses:
                self._send_pulses(pulses)
                self._last_pulses = pulses
            if now - self._last_log_time >= self._log_interval_s:
                self._log_current_signals(pulses)
                self._last_log_time = now
            self._update_event.wait(self.tick_s)
            self._update_event.clear()

    def _compute_pulses(self, now: float) -> dict[int, int]:
        pulses: dict[int, int] = {}
        with self._lock:
            patterns = dict(self._patterns)
        for name, pattern in patterns.items():
            channel = self.channel_map[name]
            pulses[channel] = self._pattern_to_pulse(pattern, now)
        return pulses

    def _pattern_to_pulse(self, pattern: str, now: float) -> int:
        if pattern == "off":
            return self.pulse_us["off"]
        if pattern == "solid":
            return self.pulse_us["on"]
        if pattern == "dim":
            return self.pulse_us["dim"]
        if pattern == "blink_slow":
            return self.pulse_us["on"] if self._blink_on(self.blink["slow_hz"], now) else self.pulse_us["off"]
        if pattern == "blink_fast":
            return self.pulse_us["on"] if self._blink_on(self.blink["fast_hz"], now) else self.pulse_us["off"]
        _LOGGER.warning("Unknown LED pattern '%s'; defaulting to off", pattern)
        return self.pulse_us["off"]

    @staticmethod
    def _blink_on(frequency_hz: float, now: float) -> bool:
        if frequency_hz <= 0:
            return False
        period = 1.0 / frequency_hz
        return (now % period) < (period / 2.0)

    def _send_pulses(self, pulses: dict[int, int]) -> None:
        if not pulses:
            return
        max_channel = max(self.channel_map.values())
        command_pulses = [None] * (max_channel + 1)
        for channel, pulse in pulses.items():
            command_pulses[channel] = pulse
        command = SSC32Command(command_pulses).to_bytes()
        try:
            self.serial_writer(command)
        except Exception as exc:  # pragma: no cover - depends on runtime IO
            _LOGGER.warning("Failed to write SSC-32 LED command: %s", exc)

    def _log_current_signals(self, pulses: dict[int, int]) -> None:
        with self._lock:
            patterns = dict(self._patterns)
        entries = []
        for name, channel in sorted(self.channel_map.items()):
            pulse = pulses.get(channel, self.pulse_us["off"])
            pattern = patterns.get(name, "off")
            entries.append(
                f"{name}: channel {channel}, pattern={pattern}, pulse_us={pulse}"
            )
        if entries:
            _LOGGER.info("SSC-32 LED channel signals -> %s", "; ".join(entries))

    @staticmethod
    def _filter_channel_map(channel_map: Dict[str, int], reserved: set[int]) -> Dict[str, int]:
        filtered: dict[str, int] = {}
        for name, channel in channel_map.items():
            if channel in reserved:
                _LOGGER.warning(
                    "LED '%s' requested reserved SSC-32 channel %d; skipping entry", name, channel
                )
                continue
            filtered[name] = channel
        return filtered


__all__ = [
    "LEDDriver",
    "SSC32PWMLEDDriver",
    "build_drive_leds",
    "load_led_config",
    "load_led_pin_map",
    "CONFIG_PATH",
]
