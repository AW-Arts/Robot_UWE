"""Tests for the interactive controller UI."""
from __future__ import annotations

import contextlib
import importlib
import sys
from typing import Any

import matplotlib
import pytest


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


class _NoFeedbackController:
    def move_joints(self, *_args, **_kwargs) -> None:
        return


class _FeedbackController:
    def __init__(self, feedback):
        self._feedback = feedback

    def move_joints(self, *_args, **_kwargs) -> None:
        return

    def read_positions(self, **_kwargs):
        return list(self._feedback)


class _FailingFeedbackController:
    def move_joints(self, *_args, **_kwargs) -> None:
        return

    def read_positions(self, **_kwargs):
        raise RuntimeError("serial fault")


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

        def get(self, *_args, **_kwargs):  # pragma: no cover - compatibility stub
            raise AssertionError("Unexpected blocking get on fake queue")

        def empty(self) -> bool:
            return not self.commands

    monkeypatch.setattr(interactive_module.queue, "Queue", FakeQueue)
    arm = interactive_module.InteractiveArm(controller, move_time_ms=move_time_ms)
    try:
        yield arm
    finally:
        interactive_module.plt.close(arm.figure)


def test_initialisation_without_feedback_uses_default_duration(
    monkeypatch, interactive_module
) -> None:
    controller = _NoFeedbackController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        assert arm._command_queue.commands  # type: ignore[attr-defined]
        queued_move_time = arm._command_queue.commands[0][1]  # type: ignore[attr-defined]
        assert queued_move_time == 1234


def test_initialisation_with_feedback_extends_first_move(monkeypatch, interactive_module) -> None:
    controller = _FeedbackController([0.1, 0.2, -0.1, 0.05, 0.0, 0.0])
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        assert arm._command_queue.commands  # type: ignore[attr-defined]
        queued_move_time = arm._command_queue.commands[0][1]  # type: ignore[attr-defined]
        assert queued_move_time == 10_000


def test_initial_feedback_failure_skips_first_command(monkeypatch, interactive_module) -> None:
    controller = _FailingFeedbackController()
    with _prepare_arm(monkeypatch, interactive_module, controller) as arm:
        assert arm._command_queue.commands == []  # type: ignore[attr-defined]
