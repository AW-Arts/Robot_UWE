from __future__ import annotations

from pathlib import Path

from lynxmotion_control.led_driver import (
    LEDDriver,
    build_drive_leds,
    load_led_pin_map,
)
from lynxmotion_control.status_leds import LEDState


class FakePWM:
    def __init__(self, pin: int, frequency: float) -> None:
        self.pin = pin
        self.frequency = frequency
        self.duty_cycle: float | None = None
        self.started = False
        self.stopped = False

    def start(self, duty_cycle: float) -> None:
        self.started = True
        self.duty_cycle = duty_cycle

    def ChangeFrequency(self, frequency: float) -> None:  # noqa: N802
        self.frequency = frequency

    def ChangeDutyCycle(self, duty_cycle: float) -> None:  # noqa: N802
        self.duty_cycle = duty_cycle

    def stop(self) -> None:
        self.stopped = True


class FakeGPIO:
    OUT = "out"
    BCM = "bcm"
    HIGH = 1
    LOW = 0

    def __init__(self) -> None:
        self.setmode_calls: list[str] = []
        self.setup_calls: list[tuple[int, str]] = []
        self.output_calls: list[tuple[int, int]] = []
        self.pwms: list[FakePWM] = []

    def setmode(self, mode) -> None:
        self.setmode_calls.append(mode)

    def setup(self, pin: int, mode) -> None:
        self.setup_calls.append((pin, mode))

    def output(self, pin: int, level: int) -> None:
        self.output_calls.append((pin, level))

    def PWM(self, pin: int, frequency: float) -> FakePWM:
        pwm = FakePWM(pin, frequency)
        self.pwms.append(pwm)
        return pwm


def test_load_led_pin_map_filters_invalid_entries(tmp_path: Path) -> None:
    config = tmp_path / "led_pins.json"
    config.write_text(
        '{"red": 5, "amber": "not-a-pin", "orange": 12, "green": 7}'
    )

    pin_map = load_led_pin_map(config)

    assert pin_map == {"red": 5, "green": 7}


def test_build_drive_leds_no_config() -> None:
    driver_fn = build_drive_leds(None)

    driver_fn({
        "red": LEDState("solid", ""),
    })

    assert callable(driver_fn)


def test_led_driver_translates_patterns_to_gpio(tmp_path: Path) -> None:
    gpio = FakeGPIO()
    driver = LEDDriver({"red": 17, "green": 18}, gpio_module=gpio)

    driver.drive(
        {
            "red": LEDState("solid", "Fault"),
            "green": LEDState("blink_fast", "Ready"),
        }
    )

    assert gpio.setmode_calls == [gpio.BCM]
    assert (17, gpio.OUT) in gpio.setup_calls
    assert (18, gpio.OUT) in gpio.setup_calls
    assert (17, gpio.HIGH) in gpio.output_calls
    assert (18, gpio.HIGH) in gpio.output_calls

    assert len(gpio.pwms) == 1
    pwm = gpio.pwms[0]
    assert pwm.started is True
    assert pwm.frequency == 4.0
    assert pwm.duty_cycle == 50.0

    driver.drive({"green": LEDState("off", "Ready"), "red": LEDState("off", "Fault")})

    assert pwm.stopped is True
    assert (17, gpio.LOW) in gpio.output_calls
    assert (18, gpio.LOW) in gpio.output_calls
