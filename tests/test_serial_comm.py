from __future__ import annotations

from typing import Sequence
import pytest

from lynxmotion_control import serial_comm


class FakeSerial:
    def __init__(self) -> None:
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def read_until(self, terminator: bytes = b"\n") -> bytes:  # pragma: no cover - unused in tests
        return b""

    def close(self) -> None:  # pragma: no cover - unused in tests
        pass


def test_move_joints_sends_serial_command(monkeypatch: pytest.MonkeyPatch) -> None:
    controller = serial_comm.AL5ASerialController(port="loopback")
    fake_serial = FakeSerial()
    controller._serial = fake_serial

    monkeypatch.setattr(
        serial_comm,
        "joints_to_pulses",
        lambda *_args, **_kwargs: [1000, 1500],
    )

    controller.move_joints([0.0, 0.0], move_time_ms=250)

    assert fake_serial.writes == [b"#0P1000#1P1500T250\r"]


def test_print_controller_move_outputs_command(
    capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    controller = serial_comm.PrintController()

    monkeypatch.setattr(
        serial_comm,
        "joints_to_pulses",
        lambda *_args, **_kwargs: [1200],
    )

    controller.move_joints([0.0], move_time_ms=None)

    captured = capsys.readouterr().out.strip()
    assert captured == "#0P1200\r"
