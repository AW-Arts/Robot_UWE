import json
from pathlib import Path

import pytest

from motor_control import MotorInversionManager


@pytest.fixture()
def default_config(tmp_path: Path) -> Path:
    config_path = tmp_path / "motors.json"
    defaults = [
        {"id": "joint1", "label": "Joint 1"},
        {"id": "joint2", "label": "Joint 2"},
    ]
    MotorInversionManager(config_path, default_motors=defaults)
    return config_path


def test_toggle_persists_state(default_config: Path) -> None:
    manager = MotorInversionManager(default_config)
    assert manager.is_inverted("joint1") is False

    manager.toggle("joint1")
    assert manager.is_inverted("joint1") is True
    assert manager.apply("joint1", 3.5) == pytest.approx(-3.5)

    saved = json.loads(default_config.read_text())
    joint1 = next(item for item in saved["motors"] if item["id"] == "joint1")
    assert joint1["inverted"] is True


def test_set_inverted_roundtrip(default_config: Path) -> None:
    manager = MotorInversionManager(default_config)
    manager.set_inverted("joint2", True)
    assert manager.is_inverted("joint2") is True

    # Reload from disk to ensure persistence
    reloaded = MotorInversionManager(default_config)
    assert reloaded.is_inverted("joint2") is True


def test_unknown_motor_raises(default_config: Path) -> None:
    manager = MotorInversionManager(default_config)
    with pytest.raises(KeyError):
        manager.toggle("missing")
