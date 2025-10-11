"""Serial communication helpers for the Lynxmotion AL5A arm."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Protocol, Sequence

from .al5a_kinematics import DEFAULT_SERVO_CONFIGS, joints_to_pulses

try:
    import serial  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - optional dependency at runtime
    serial = None


class SerialLike(Protocol):
    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


@dataclass
class SSC32Command:
    pulses: Sequence[int]
    move_time_ms: int | None = None

    def to_bytes(self) -> bytes:
        parts: list[str] = []
        for channel, pulse in enumerate(self.pulses):
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
        self._serial = serial.Serial(self.port_name, self.baudrate, timeout=1)

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
    ) -> None:
        serial_port = self.ensure_connection()
        pulses = joints_to_pulses(joints, servo_configs or DEFAULT_SERVO_CONFIGS)
        command = SSC32Command(pulses, move_time_ms)
        serial_port.write(command.to_bytes())

    def scan_channels(
        self,
        channels: Iterable[int] | None = None,
        *,
        centre_pulse: int = 1500,
        pulse_span: int = 300,
        move_time_ms: int = 750,
        pause_s: float = 0.6,
    ) -> None:
        """Sweep potential channels to help locate miswired servos.

        The SSC-32 controllers do not provide electrical feedback, so the scan
        simply jogs each candidate channel back and forth.  Watch the physical
        arm while the scan runs and note which servo responds to each channel.

        Parameters
        ----------
        channels:
            Iterable of candidate channel numbers.  When ``None`` every channel
            between 0 and 31 is exercised.
        centre_pulse:
            The neutral pulse width used for the sweep, typically 1500 μs.
        pulse_span:
            The number of microseconds added/subtracted from ``centre_pulse``
            to create the sweep limits.
        move_time_ms:
            Duration supplied to the controller for each movement.
        pause_s:
            Time to wait between individual sweep commands so servos have time
            to move before the next channel is tested.
        """

        serial_port = self.ensure_connection()

        if channels is None:
            ordered = range(32)
        else:
            ordered = dict.fromkeys(channels).keys()
        candidate_channels = tuple(ch for ch in ordered if 0 <= ch < 32)
        if not candidate_channels:
            return

        low = max(500, centre_pulse - abs(pulse_span))
        high = min(2500, centre_pulse + abs(pulse_span))
        pause_s = max(0.0, pause_s)

        print(
            "Sweeping candidate channels to locate attached servos. Observe the arm "
            "and note which joints move for each channel."
        )

        import time

        for channel in candidate_channels:
            print(
                f"Channel {channel}: moving to pulses {low}, {high} and back to {centre_pulse}"
            )
            command_low = f"#{channel}P{low}T{move_time_ms}\r".encode("ascii")
            serial_port.write(command_low)
            if pause_s:
                time.sleep(pause_s)

            command_high = f"#{channel}P{high}T{move_time_ms}\r".encode("ascii")
            serial_port.write(command_high)
            if pause_s:
                time.sleep(pause_s)

            command_centre = f"#{channel}P{centre_pulse}T{move_time_ms}\r".encode("ascii")
            serial_port.write(command_centre)
            if pause_s:
                time.sleep(pause_s)

        print("Scan complete. Re-map your servo channels based on the observed motion.")


class PrintController:
    """Fallback controller that prints commands instead of sending them."""

    def move_joints(
        self,
        joints: Sequence[float],
        move_time_ms: int | None = None,
        servo_configs: dict[int, object] | None = None,
    ) -> None:
        pulses = joints_to_pulses(joints, servo_configs or DEFAULT_SERVO_CONFIGS)
        command = SSC32Command(pulses, move_time_ms)
        print(command.to_bytes().decode("ascii").strip())

    def scan_channels(
        self,
        channels: Iterable[int] | None = None,
        *,
        centre_pulse: int = 1500,
        pulse_span: int = 300,
        move_time_ms: int = 750,
        pause_s: float = 0.6,
    ) -> None:
        if channels is None:
            ordered = range(32)
        else:
            ordered = dict.fromkeys(channels).keys()
        candidate_channels = tuple(ch for ch in ordered if 0 <= ch < 32)
        if not candidate_channels:
            print("No channels supplied for scanning.")
            return

        low = max(500, centre_pulse - abs(pulse_span))
        high = min(2500, centre_pulse + abs(pulse_span))

        print(
            "Simulation scan. The following commands would be sent to help locate "
            "servo wiring:"
        )
        for channel in candidate_channels:
            print(f"  #{channel}P{low}T{move_time_ms}")
            if pause_s:
                print(f"  (wait {pause_s:.2f}s)")
            print(f"  #{channel}P{high}T{move_time_ms}")
            if pause_s:
                print(f"  (wait {pause_s:.2f}s)")
            print(f"  #{channel}P{centre_pulse}T{move_time_ms}")
            if pause_s:
                print(f"  (wait {pause_s:.2f}s)")
        print("Scan complete.")


__all__ = ["AL5ASerialController", "PrintController", "SSC32Command"]
