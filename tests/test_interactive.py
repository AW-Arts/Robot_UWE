"""Tests for the interactive controller UI."""
from __future__ import annotations

import contextlib
import json
import importlib
import math
import sys
from typing import Any

import pytest
import numpy as np

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

    def move_joints(self, joints, **kwargs) -> None:  # pragma: no cover - simple recording
        self.moves.append((tuple(joints), kwargs.get("move_time_ms")))


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


def _install_fake_clock(monkeypatch, interactive_module, *, start: float = 100.0):
    current = {"now": start}

    def monotonic():
        return current["now"]

    def advance(seconds: float) -> None:
        current["now"] += seconds

    monkeypatch.setattr(interactive_module.time, "monotonic", monotonic)
    return advance


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


def test_set_vertical_aligns_base_zero_to_mid_pulse(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._enter_calibration_mode()
        arm.current_joints = [0.35, 0.4, -0.2, 0.1, 0.0, 0.0]
        arm.commanded_joints = list(arm.current_joints)

        base_config = arm.servo_configs[0]
        arm._handle_set_vertical()

        expected_mid_angle = base_config.min_angle + (
            (base_config.max_angle - base_config.min_angle) / 2.0
        )
        expected_offset = 0.0 - expected_mid_angle

        assert math.isclose(arm.servo_offsets[0], expected_offset, rel_tol=1e-6)
        assert math.isclose(arm.zero_reference[0], 0.0, rel_tol=1e-6)
        assert math.isclose(arm.current_joints[0], 0.0, rel_tol=1e-6)

        zero_raw = arm._apply_offsets([0.0], direction="raw")[0]
        min_raw = arm._apply_offsets([base_config.min_angle], direction="raw")[0]
        max_raw = arm._apply_offsets([base_config.max_angle], direction="raw")[0]

        assert math.isclose(zero_raw, expected_mid_angle, rel_tol=1e-6)
        assert math.isclose(min_raw, base_config.min_angle, rel_tol=1e-6)
        assert math.isclose(max_raw, base_config.max_angle, rel_tol=1e-6)

        zero_pulse = base_config.angle_to_pulse(zero_raw)
        expected_mid_pulse = int(round((base_config.min_pulse + base_config.max_pulse) / 2.0))
        assert zero_pulse == expected_mid_pulse


def test_set_vertical_preserves_asymmetric_base_limits(
    monkeypatch, interactive_module, tmp_path
) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    calibration_file.write_text(
        json.dumps(
            {
                "servo_limits": {
                    "0": {"min_deg": -45.0, "max_deg": 30.0},
                }
            }
        )
    )
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )

    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._enter_calibration_mode()
        original_base_angle = 0.35
        arm.current_joints = [original_base_angle, 0.4, -0.2, 0.1, 0.0, 0.0]
        arm.commanded_joints = list(arm.current_joints)

        base_config = arm.servo_configs[0]
        assert not math.isclose(
            abs(base_config.min_angle), abs(base_config.max_angle), rel_tol=1e-6
        )

        arm._handle_set_vertical()

        assert math.isclose(
            arm.servo_offsets[0], -original_base_angle, rel_tol=1e-6
        )
        zero_raw = arm._apply_offsets([0.0], direction="raw")[0]
        assert math.isclose(zero_raw, original_base_angle, rel_tol=1e-6)

        max_raw = arm._apply_offsets([base_config.max_angle], direction="raw")[0]
        assert max_raw >= base_config.max_angle


def test_leds_follow_calibration_state(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._command_queue.commands.clear()  # type: ignore[attr-defined]
        arm._mark_motion_complete()
        initial = arm.status_leds.snapshot()
        assert initial["green"].pattern == "solid"

        arm._enter_calibration_mode()
        during_calibration = arm.status_leds.snapshot()
        assert during_calibration["yellow"].pattern == "solid"
        assert during_calibration["amber"].pattern == "solid"
        assert during_calibration["green"].pattern == "off"

        arm._exit_calibration_mode()
        after = arm.status_leds.snapshot()
        assert after["yellow"].pattern == "off"
        assert after["amber"].pattern == "off"
        assert after["green"].pattern == "solid"


def test_teach_recording_starts_at_zero(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    advance = _install_fake_clock(monkeypatch, interactive_module, start=50.0)

    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._set_operation_mode("teach")
        arm._start_teach_session()

        advance(5.0)
        arm._record_teach_sample((1.0, 2.0, 3.0, 4.0, 5.0, 6.0), move_time_ms=None)

        assert len(arm._teach_samples) == 1
        first = arm._teach_samples[0]
        assert first.timestamp == 0.0
        assert first.dwell_ms == 0


def test_teach_recording_throttles_samples(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    advance = _install_fake_clock(monkeypatch, interactive_module, start=80.0)

    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._set_operation_mode("teach")
        arm._start_teach_session()

        arm._record_teach_sample((0.0, 0.1, 0.2, 0.3, 0.4, 0.5), move_time_ms=None)
        assert len(arm._teach_samples) == 1

        advance(0.05)
        arm._record_teach_sample((0.5, 0.4, 0.3, 0.2, 0.1, 0.0), move_time_ms=None)
        assert len(arm._teach_samples) == 1

        advance(0.25)
        arm._record_teach_sample((0.6, 0.5, 0.4, 0.3, 0.2, 0.1), move_time_ms=None)
        assert len(arm._teach_samples) == 2

        second = arm._teach_samples[1]
        assert second.timestamp == pytest.approx(0.3)
        assert second.dwell_ms == pytest.approx(300, rel=0.01)


def test_teach_waypoints_preserve_move_and_dwell(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        samples = [
            interactive_module.TeachSample(
                timestamp=0.0,
                joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                cartesian=(0.0, 0.0, 0.0),
                wrist_pitch=0.0,
                wrist_rotation=0.0,
                gripper_angle=0.0,
                move_time_ms=50,
                dwell_ms=0,
                label=None,
            ),
            interactive_module.TeachSample(
                timestamp=1.315,
                joints=(0.1, 0.1, 0.1, 0.1, 0.1, 0.1),
                cartesian=(0.1, 0.0, 0.0),
                wrist_pitch=5.0,
                wrist_rotation=10.0,
                gripper_angle=15.0,
                move_time_ms=60,
                dwell_ms=1250,
                label=None,
            ),
        ]

        waypoints = arm._teach_samples_to_waypoints(samples)

        assert len(waypoints) == 2
        # First waypoint clamps to minimum duration
        assert waypoints[0].duration == pytest.approx(0.1)
        # Second waypoint should combine move and dwell time
        assert waypoints[1].duration == pytest.approx(1.31, rel=1e-3)


def test_returning_from_teach_moves_slowly(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(
        monkeypatch, interactive_module, controller, move_time_ms=500
    ) as arm:
        arm._command_queue.commands.clear()  # type: ignore[attr-defined]
        arm._set_operation_mode("teach")

        new_target = np.array([0.12, -0.03, 0.18])
        arm.target[:] = new_target

        arm._set_operation_mode("live")

        assert arm._command_queue.commands  # type: ignore[attr-defined]
        _, move_time, _ = arm._command_queue.commands[-1]  # type: ignore[attr-defined]
        assert move_time == 10_000


def test_manual_move_cancels_playback(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        class _FakeThread:
            def __init__(self) -> None:
                self.join_called = False

            def is_alive(self) -> bool:
                return True

            def join(self, timeout=None) -> None:  # pragma: no cover - simple flag
                self.join_called = True

        waypoint_thread = _FakeThread()
        timeline_thread = _FakeThread()
        arm._waypoint_playback_thread = waypoint_thread
        arm._timeline_playback_thread = timeline_thread

        arm.update_robot()

        assert arm._waypoint_playback_thread is None
        assert arm._timeline_playback_thread is None
        assert waypoint_thread.join_called
        assert timeline_thread.join_called
        assert not arm._waypoint_stop_event.is_set()
        assert not arm._timeline_stop_event.is_set()


def test_update_waypoint_uses_latest_joints(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm.waypoints = [
            interactive_module.Waypoint(
                name="1",
                position=np.zeros(3),
                duration=1.0,
                wrist_pitch=0.0,
                wrist_rotation=0.0,
                gripper_angle=0.0,
            )
        ]
        arm._selected_waypoint_index = 0

        joint_state = [0.2, 0.4, -0.25, 0.1, 0.3, -0.2]
        arm.feedback_joints = list(joint_state)
        arm.commanded_joints = list(joint_state)
        arm.wrist_pitch = -1.0
        arm.wrist_rotation = -1.0
        arm.gripper_angle = 0.5
        arm.target[:] = np.array([0.05, 0.05, 0.05])

        arm._handle_update_waypoint()

        updated = arm.waypoints[0]
        expected_pose = arm.kin.forward(joint_state[:4])
        np.testing.assert_allclose(updated.position, expected_pose[:3, 3])
        expected_pitch = joint_state[1] + joint_state[2] + joint_state[3]
        assert math.isclose(updated.wrist_pitch, expected_pitch, rel_tol=1e-6)
        assert math.isclose(updated.wrist_rotation, joint_state[4], rel_tol=1e-6)
        assert math.isclose(updated.gripper_angle, joint_state[5], rel_tol=1e-6)


def test_zero_reference_lines_follow_current_pose(monkeypatch, interactive_module) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        commanded = [
            math.radians(30.0),
            math.radians(20.0),
            math.radians(-15.0),
            math.radians(10.0),
            0.0,
            0.0,
        ]
        arm.commanded_joints = list(commanded)
        arm._update_zero_reference_lines()

        x_data, y_data, z_data = arm._zero_reference_lines[1].get_data_3d()
        ax, ay, az = arm._compute_link_positions(commanded)

        assert math.isclose(x_data[0], ax[1], rel_tol=0.0, abs_tol=1e-6)
        assert math.isclose(y_data[0], ay[1], rel_tol=0.0, abs_tol=1e-6)
        assert math.isclose(z_data[0], az[1], rel_tol=0.0, abs_tol=1e-6)


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


def test_servo_multipliers_loaded_and_persisted(
    monkeypatch, interactive_module, tmp_path
) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    calibration_file.write_text(
        json.dumps(
            {
                "multipliers": {
                    "0": 0.5,
                    "2": 1.3,
                }
            }
        )
    )
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )

    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        assert arm.servo_multipliers[0] == pytest.approx(0.8)
        assert arm.servo_multipliers[2] == pytest.approx(1.2)
        arm._set_servo_multiplier(0, 1.05)

    stored = json.loads(calibration_file.read_text())
    assert stored["multipliers"]["0"] == pytest.approx(1.05)
    assert stored["multipliers"]["2"] == pytest.approx(1.2)


def test_hard_limit_clamping_uses_servo_configs(
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
                    "1": {"min_deg": -10.0, "max_deg": 20.0},
                }
            }
        )
    )

    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        config = arm.servo_configs[1]
        assert math.isclose(
            config.min_angle, math.radians(-10.0), rel_tol=1e-6
        )
        assert math.isclose(config.max_angle, math.radians(20.0), rel_tol=1e-6)

        joints = [0.0, math.radians(90.0), 0.0, 0.0]
        clamped = arm._apply_hard_limits_to_ik(joints)
        assert math.isclose(
            clamped[1], config.max_angle, rel_tol=0.0, abs_tol=1e-6
        )

        arm.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        arm.commanded_joints = list(arm.current_joints)
        arm._set_servo_angle(1, math.radians(-90.0))

        assert math.isclose(
            arm.current_joints[1], config.min_angle, rel_tol=0.0, abs_tol=1e-6
        )
        queued_command = arm._command_queue.commands[-1][0]  # type: ignore[attr-defined]
        assert math.isclose(
            queued_command[1], config.min_angle, rel_tol=0.0, abs_tol=1e-6
        )


def test_calibration_text_updates_limits_without_motion(
    monkeypatch, interactive_module, tmp_path
) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._calibration_guide_active = True
        arm._calibration_steps = [(0, "min")]
        arm._calibration_step_index = 0
        arm.current_joints = [0.0] * 6
        arm.feedback_joints = list(arm.current_joints)

        starting_zero = arm._zero_angle_for_servo(0)
        arm._apply_calibration_angle_from_text("-15")

        updated_config = arm.servo_configs[0]
        expected_min = starting_zero + math.radians(-15)
        assert math.isclose(updated_config.min_angle, expected_min, rel_tol=1e-6)
        assert controller.moves == []
        stored = json.loads(calibration_file.read_text())
        assert stored["servo_limits"]["0"]["min_deg"] == pytest.approx(-15.0)


def test_calibration_center_updates_zero_reference_only(
    monkeypatch, interactive_module, tmp_path
) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._calibration_guide_active = True
        arm._calibration_steps = [(1, "center")]
        arm._calibration_step_index = 0
        arm.current_joints = [0.0] * 6
        arm.feedback_joints = list(arm.current_joints)

        starting_zero = arm._zero_angle_for_servo(1)
        arm._apply_calibration_angle_from_text("5")

        expected_zero = starting_zero + math.radians(5)
        assert math.isclose(arm.zero_reference[1], expected_zero, rel_tol=1e-6)
        assert controller.moves == []
        stored = json.loads(calibration_file.read_text())
        assert stored["vertical_angles"]["1"] == pytest.approx(expected_zero)


def test_calibration_nudge_updates_limits_without_motion(
    monkeypatch, interactive_module, tmp_path
) -> None:
    calibration_file = tmp_path / "servo_offsets.json"
    monkeypatch.setattr(
        interactive_module,
        "CALIBRATION_CONFIG_PATH",
        calibration_file,
        raising=False,
    )
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm._calibration_guide_active = True
        arm._calibration_steps = [(2, "max")]
        arm._calibration_step_index = 0
        arm.current_joints = [0.0] * 6
        arm.feedback_joints = list(arm.current_joints)

        starting_max = arm.servo_configs[2].max_angle
        arm._nudge_current_calibration_step(math.radians(3))

        updated_max = arm.servo_configs[2].max_angle
        assert math.isclose(
            updated_max, starting_max + math.radians(3), rel_tol=1e-6
        )
        assert controller.moves == []
        stored = json.loads(calibration_file.read_text())
        assert stored["servo_limits"]["2"]["max_deg"] == pytest.approx(
            math.degrees(updated_max)
        )


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
        assert arm._last_commanded_raw != baseline_last_raw
        assert arm.commanded_joints != baseline_current
        assert arm.current_joints != baseline_current


def test_wrist_extension_compensation_scales_with_reach(
    monkeypatch, interactive_module
) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm.wrist_extension_compensation = [
            (0.0, 1.0),
            (0.5, 1.0),
            (1.0, 1.2),
        ]
        base_pitch = math.radians(15)
        close_target = np.array([0.05, 0.0, 0.05])
        far_target = np.array([arm.kin.links.shoulder + arm.kin.links.elbow, 0.0, 0.05])

        close_pitch = arm._apply_wrist_extension_compensation(close_target, base_pitch)
        far_pitch = arm._apply_wrist_extension_compensation(far_target, base_pitch)

        assert math.isclose(close_pitch, base_pitch, rel_tol=1e-6)
        assert far_pitch > close_pitch


def test_wrist_extension_compensation_respects_limits(
    monkeypatch, interactive_module
) -> None:
    controller = _BasicController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        arm.wrist_extension_compensation = [(0.0, 10.0)]
        arm._wrist_pitch_limits = (math.radians(-20), math.radians(20))
        base_pitch = math.radians(18)
        target = np.array([arm.kin.links.shoulder, 0.0, 0.0])

        compensated = arm._apply_wrist_extension_compensation(target, base_pitch)

        assert math.isclose(compensated, arm._wrist_pitch_limits[1], rel_tol=1e-6)
