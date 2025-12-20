from __future__ import annotations

import json
import time

from lynxmotion_control.led_driver import (
    SSC32PWMLEDDriver,
    load_led_config,
)
from lynxmotion_control.status_leds import LEDState


def test_load_led_config_legacy_gpio(tmp_path) -> None:
    config_path = tmp_path / "led_pins.json"
    config_path.write_text(json.dumps({"red": 17, "green": 22}))

    config = load_led_config(config_path)

    assert config["backend"] == "gpio"
    assert config["pin_map"] == {"red": 17, "green": 22}


def test_load_led_config_ssc32_structured(tmp_path) -> None:
    payload = {
        "backend": "ssc32_pwm_led",
        "servo_reserved_channels": [0, 1, 2, 3, 4, 5, 6, 7],
        "led_channels": {"red": 8, "amber": 9, "green": 10},
        "pulse_us": {"off": 500, "on": 2000, "dim": 1300},
        "blink": {"slow_hz": 1, "fast_hz": 4},
    }
    config_path = tmp_path / "led_pins.json"
    config_path.write_text(json.dumps(payload))

    config = load_led_config(config_path)

    assert config["backend"] == "ssc32_pwm_led"
    assert config["led_channels"] == {"red": 8, "amber": 9, "green": 10}
    assert config["pulse_us"]["on"] == 2000
    assert config["blink"]["fast_hz"] == 4.0


def test_load_led_config_seeds_defaults_when_missing(tmp_path) -> None:
    config_root = tmp_path / "config"
    config_root.mkdir()
    (config_root / "servo_offsets.json").write_text("{}")
    config_path = config_root / "led_pins.json"

    config = load_led_config(config_path)

    assert config_path.exists()
    assert config["backend"] == "ssc32_pwm_led"
    assert config["led_channels"]["red"] == 8


def test_ssc32_driver_filters_reserved_and_drives() -> None:
    sent: list[bytes] = []
    driver = SSC32PWMLEDDriver(
        channel_map={"red": 8, "amber": 0},
        reserved_channels={0},
        pulse_us={"off": 500, "on": 2000, "dim": 1300},
        blink={"slow_hz": 1.0, "fast_hz": 4.0},
        serial_writer=sent.append,
        tick_s=0.01,
    )

    driver.drive({"red": LEDState("solid", ""), "amber": LEDState("solid", "")})
    time.sleep(0.03)

    assert sent, "Driver should emit at least one SSC-32 command"
    command = sent[-1].decode("ascii")
    assert "#8P2000" in command
    assert "#0P" not in command
