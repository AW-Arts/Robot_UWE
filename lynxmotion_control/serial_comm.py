"""Serial communication helpers for the Lynxmotion AL5A arm."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

from .al5a_kinematics import (
    DEFAULT_SERVO_CHANNELS,
    DEFAULT_SERVO_CONFIGS,
    joints_to_pulses,
)

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency at runtime
    serial = None


class SerialLike(Protocol):
    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


@dataclass
class SSC32Command:
    pulses: Sequence[int | None]
    move_time_ms: int | None = None

    def to_bytes(self) -> bytes:
        parts: list[str] = []
        for channel, pulse in enumerate(self.pulses):
            if pulse is None:
                continue
            parts.append(f"#{channel}P{pulse}")
        if self.move_time_ms is not None:
            parts.append(f"T{self.move_time_ms}")
        return ("".join(parts) + "\r").encode("ascii")


class AL5ASerialController:
    """High-level interface to the SSC-32/SSC-32U controller."""

    def __init__(self, port: str | None = None, baudrate: int = 115200) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self._serial: SerialLike | None = None

    def connect(self) -> None:
        if self._serial is not None:
            return
        if self.port_name is None:
            raise RuntimeError("Serial port not specified")
        if serial is None:
            raise RuntimeError(
                "pyserial is not available. Install it or run in simulation mode."
            )
        try:
            self._serial = serial.Serial(self.port_name, self.baudrate, timeout=1)
        except Exception as exc:  # pragma: no cover - depends on hardware state
            serial_exception = getattr(serial, "SerialException", Exception)
            if isinstance(exc, serial_exception):
                raise RuntimeError(
                    "Unable to open serial port "
                    f"{self.port_name!r}. Ensure the controller is connected, "
                    "the correct port is selected, and no other program is using "
                    "the device."
                ) from exc
            raise

    def disconnect(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def ensure_connection(self) -> SerialLike:
        if self._serial is None:
            self.connect()
        if self._serial is None:
            raise RuntimeError("Unable to establish serial connection")
        return self._serial

    def move_joints(
        self,
        joints: Sequence[float],
        move_time_ms: int | None = None,
        servo_configs: dict[int, object] | None = None,
        servo_channels: dict[int, int] | None = None,
    ) -> None:
        serial_port = self.ensure_connection()
        pulses = joints_to_pulses(
            joints,
            servo_configs=servo_configs or DEFAULT_SERVO_CONFIGS,
            servo_channels=servo_channels or DEFAULT_SERVO_CHANNELS,
        )
        command = SSC32Command(pulses, move_time_ms)
        serial_port.write(command.to_bytes())


class PrintController:
    """Fallback controller that prints commands instead of sending them."""

    def move_joints(
        self,
        joints: Sequence[float],
        move_time_ms: int | None = None,
        servo_configs: dict[int, object] | None = None,
        servo_channels: dict[int, int] | None = None,
    ) -> None:
        pulses = joints_to_pulses(
            joints,
            servo_configs=servo_configs or DEFAULT_SERVO_CONFIGS,
            servo_channels=servo_channels or DEFAULT_SERVO_CHANNELS,
        )
        command = SSC32Command(pulses, move_time_ms)
        print(command.to_bytes().decode("ascii").strip())


__all__ = ["AL5ASerialController", "PrintController", "SSC32Command"]
