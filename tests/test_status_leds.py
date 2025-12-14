from __future__ import annotations

from lynxmotion_control.status_leds import RobotISOState, StatusLEDController


def test_status_led_controller_patterns() -> None:
    leds = StatusLEDController()

    leds.set_ready(active=True, running=False)
    snapshot = leds.snapshot()
    assert snapshot["green"].pattern == "solid"
    assert snapshot["green"].meaning == "Ready / Running"

    leds.set_ready(active=True, running=True)
    snapshot = leds.snapshot()
    assert snapshot["green"].pattern == "blink_slow"

    leds.set_fault(active=True, just_triggered=True)
    snapshot = leds.snapshot()
    assert snapshot["red"].pattern == "blink_fast"

    leds.set_fault(active=False)
    snapshot = leds.snapshot()
    assert snapshot["red"].pattern == "off"

    leds.set_teach_mode(active=True, blinking=True)
    snapshot = leds.snapshot()
    assert snapshot["yellow"].pattern == "blink_slow"


def test_iso_truth_table_states() -> None:
    leds = StatusLEDController()

    leds.set_iso_state(RobotISOState.CONNECTED_NOT_ENABLED)
    snapshot = leds.snapshot()
    assert snapshot["amber"].pattern == "solid"
    assert snapshot["blue"].pattern == "solid"
    assert snapshot["green"].pattern == "off"

    leds.set_iso_state(RobotISOState.RUNNING)
    snapshot = leds.snapshot()
    assert snapshot["green"].pattern == "blink_slow"
    assert snapshot["blue"].pattern == "solid"

    leds.set_iso_state(RobotISOState.TEACH_MODE, teach_active=True)
    snapshot = leds.snapshot()
    assert snapshot["green"].pattern == "solid"
    assert snapshot["yellow"].pattern == "solid"
    assert snapshot["amber"].pattern == "off"

