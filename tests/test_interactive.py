"""Tests for the interactive controller UI."""
from __future__ import annotations

import contextlib
import json
import importlib
import math
import sys
from typing import Any

import pytest

matplotlib = pytest.importorskip("matplotlib")


@pytest.fixture
def interactive_module(monkeypatch):
    """Import the interactive module with a benign backend and patched thread."""

    monkeypatch.setattr(matplotlib, "get_backend", lambda: "nbagg")
    sys.modules.pop("lynxmotion_control.interactive", None)
    module = importlib.import_module("lynxmotion_control.interactive")

    class DummyThread:
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            self.args = args
            self.kwargs = kwargs

        def start(self) -> None:  # pragma: no cover - no behaviour to verify
            return

    monkeypatch.setattr(module.threading, "Thread", DummyThread)
    return module


class _BasicController:
    def __init__(self) -> None:
        self.moves: list[tuple[tuple[float, ...], int | None]] = []
        self.relax_calls = 0

    def move_joints(self, joints, **kwargs) -> None:  # pragma: no cover - simple recording
        self.moves.append((tuple(joints), kwargs.get("move_time_ms")))

    def relax_servos(self, *_args, **_kwargs) -> None:  # pragma: no cover - compatibility
        self.relax_calls += 1


@contextlib.contextmanager
def _prepare_arm(monkeypatch, interactive_module, controller, *, move_time_ms: int = 1234):
    class FakeQueue:
        def __init__(self, *_, **__) -> None:
            self.commands: list[tuple[tuple[float, ...], int | None]] = []

        def put_nowait(self, item: tuple[tuple[float, ...], int | None]) -> None:
            self.commands.append(item)

        def get_nowait(self):  # pragma: no cover - defensive, unused in tests
            raise interactive_module.queue.Empty

        def task_done(self) -> None:  # pragma: no cover - compatibility stub
            return

        def get(self, *_args, **_kwargs):
            if not self.commands:
                raise AssertionError("Unexpected blocking get on fake queue")
            return self.commands.pop(0)

        def empty(self) -> bool:
            return not self.commands

    monkeypatch.setattr(interactive_module.queue, "Queue", FakeQueue)
    arm = interactive_module.InteractiveArm(controller, move_time_ms=move_time_ms)
    try:
        yield arm
    finally:
        interactive_module.plt.close(arm.figure)


def test_initialisation_queues_initial_move(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        assert arm._command_queue.commands  # type: ignore[attr-defined]
        queued_move_time = arm._command_queue.commands[0][1]  # type: ignore[attr-defined]
        assert queued_move_time == 1234


def test_enter_calibration_mode_clears_pending_commands(
    monkeypatch, interactive_module
) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._command_queue.put_nowait(((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), None))  # type: ignore[attr-defined]
        arm._enter_calibration_mode()
        assert arm._calibration_active
        assert arm._command_queue.commands == []  # type: ignore[attr-defined]
        assert controller.relax_calls == 0


def test_set_vertical_persists_offsets(monkeypatch, interactive_module, tmp_path) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._enter_calibration_mode()
        arm.current_joints = [0.0, 0.4, -0.2, 0.1, 0.0, 0.0]
        arm.commanded_joints = list(arm.current_joints)
        arm._handle_set_vertical()
        arm._exit_calibration_mode()

    assert calibration_file.exists()
    stored = json.loads(calibration_file.read_text())
    expected_shoulder = math.pi / 2 - 0.4
    expected_elbow = 0.0 - (-0.2)
    assert math.isclose(float(stored["1"]), expected_shoulder, rel_tol=1e-6)
    assert math.isclose(float(stored["2"]), expected_elbow, rel_tol=1e-6)

    new_controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, new_controller) as arm:
        assert math.isclose(
            arm.servo_offsets.get(1, 0.0), expected_shoulder, rel_tol=1e-6
        )
        assert math.isclose(arm.servo_offsets.get(2, 0.0), expected_elbow, rel_tol=1e-6)


def test_servo_limits_loaded_and_persisted(
    monkeypatch, interactive_module, tmp_path
) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )
    calibration_file.write_text(
        json.dumps(
            {
                "servo_limits": {
                    "0": {"min_deg": -45.0, "max_deg": 30.0},
                    "3": {"min_deg": -100.0, "max_deg": 95.0},
                }
            }
        )
    )

    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        config = arm.servo_configs[0]
        assert math.isclose(
            config.min_angle, math.radians(-45.0), rel_tol=1e-6
        )
        assert math.isclose(
            config.max_angle, math.radians(30.0), rel_tol=1e-6
        )

        new_min = -55.0
        arm._update_servo_limit(0, min_angle=math.radians(new_min))

    stored = json.loads(calibration_file.read_text())
    stored_limits = stored["servo_limits"]["0"]
    assert math.isclose(float(stored_limits["min_deg"]), new_min, rel_tol=1e-6)
    assert math.isclose(float(stored_limits["max_deg"]), 30.0, rel_tol=1e-6)


def test_command_worker_processes_commands_without_feedback(
    monkeypatch, interactive_module
) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._command_queue.commands.clear()  # type: ignore[attr-defined]
        baseline_current = list(arm.current_joints)
        baseline_last_raw = arm._last_commanded_raw

        target = [angle + 0.1 for angle in baseline_current]
        arm._send_move_command(target, move_time_ms=250)

        assert arm._command_queue.commands  # type: ignore[attr-defined]
        original_get = arm._command_queue.get  # type: ignore[attr-defined]

        def single_use_get(*args, **kwargs):
            if single_use_get.calls == 0:
                single_use_get.calls += 1
                return original_get(*args, **kwargs)
            raise KeyboardInterrupt

        single_use_get.calls = 0

        arm._command_queue.get = single_use_get  # type: ignore[attr-defined]

        with pytest.raises(KeyboardInterrupt):
            arm._command_worker()

        arm._command_queue.get = original_get  # type: ignore[attr-defined]

        assert controller.moves  # command sent to controller
        assert controller.relax_calls == 0
        assert arm._last_commanded_raw != baseline_last_raw
        assert arm.commanded_joints != baseline_current
        assert arm.current_joints != baseline_current
