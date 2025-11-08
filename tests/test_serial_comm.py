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


@pytest.fixture
def patched_channels(monkeypatch: pytest.MonkeyPatch) -> dict[int, int]:
    mapping = {0: 0, 1: 2, 2: 4}
    monkeypatch.setattr(serial_comm, "DEFAULT_SERVO_CHANNELS", mapping)
    return mapping


@pytest.mark.parametrize("servo_indices", [None, [0, 2]])
def test_relax_servos_generates_expected_command(
    servo_indices: Sequence[int] | None, patched_channels: dict[int, int]
) -> None:
    controller = serial_comm.AL5ASerialController(port="loopback")
    fake_serial = FakeSerial()
    controller._serial = fake_serial

    controller.relax_servos(servo_indices=servo_indices)

    expected_order = (
        [0, 1, 2] if servo_indices is None else list(servo_indices)
    )
    expected = (
        "".join(f"#{patched_channels[index]}PO" for index in expected_order) + "\r"
    ).encode(
        "ascii"
    )
    assert fake_serial.writes == [expected]


def test_print_controller_relax_outputs_command(
    capsys: pytest.CaptureFixture[str], patched_channels: dict[int, int]
) -> None:
    controller = serial_comm.PrintController()

    controller.relax_servos(servo_indices=[1])

    captured = capsys.readouterr().out.strip()
    assert captured == f"#{patched_channels[1]}PO"
