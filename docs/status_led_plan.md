# SSC-32(U) status light prototype plan

This document is the authoritative plan for driving DMG-style status lights from an SSC-32(U) **without** any auxiliary GPIO hardware. It applies to the prototype stage only and intentionally reuses unused servo PWM channels for LEDs.

## 1. Objective

Implement a CNC-style traffic-light system for the robot using only the SSC-32(U). The lights clearly communicate robot state, follow industrial colour semantics, and acknowledge the hardware limitations of the prototype.

## 2. Design constraints (fixed)

- **Controller:** Lynxmotion SSC-32(U)
- **Channels 0–7:** used by servos and must not be touched
- **No extra GPIO/PLC/Raspberry Pi:** SSC-32(U) outputs only
- **Indicators are informational:** not safety-rated
- **Prototype acceptance:** servo PWM driving LEDs is acceptable even if flicker is visible

## 3. Available outputs

| Output type | Count | Usage |
| --- | --- | --- |
| Servo PWM signal pins | 32 | Servos and LEDs |
| Safe LED channels | 24 | Channels 8–31 |
| True GPIO | None | Not available |
| Power rail for LEDs | None | LEDs driven from signal pins only |

## 4. Channel allocation (locked)

Servo channels:

- 0–7 → Servos (motion only)

LED channels (prototype):

- 8 → Red (fault / unsafe)
- 9 → Amber (attention / transition)
- 10 → Green (ready / running)
- 11 → Blue (controller / automation)
- 12 → Yellow (teach / calibration)
- 13 → White (utility / awake)

Channels 14–31 remain unused.

## 5. Electrical implementation

Per-LED wiring:

```
SSC-32 SIGNAL (CHx) ──[220–470 Ω]──▶|── GND
```

Rules:

- One resistor per LED
- LED cathode to GND
- Do not use servo V+
- Do not share resistors
- Do not connect LEDs between signal pins

This wiring keeps current limited, avoids loading the servo power rail, and keeps each LED isolated.

## 6. LED meaning & behaviour (industrial-correct)

Priority hierarchy: **RED > AMBER > GREEN**. Blue, Yellow, and White are overlays that can be active alongside the stack lights.

### 🔴 Red — Fault / Unsafe (priority 1)

- Meaning: unsafe condition exists now
- Triggers: E-stop, hard fault, limit hit, servo/controller error, safety chain open
- Behaviour: **Solid** = fault present; **Blink fast (4 Hz)** = fault just occurred / operator action required
- Rules: if red ≠ OFF → amber = OFF, green = OFF; red may remain on while overlays are active

### 🟠 Amber — Attention / Transition (priority 2)

- Meaning: not ready or transitioning
- Triggers: connected but not enabled; enabled but not homed; waiting for operator confirmation; homing in progress
- Behaviour: **Solid** = waiting / blocked; **Blink slow (1 Hz)** = transitioning
- Rules: amber is never idle; if amber ≠ OFF → green = OFF

### 🟢 Green — Ready / Running (priority 3)

- Meaning: normal, permissive operation
- Triggers: enabled, homed, no faults
- Behaviour: **Solid** = idle but ready; **Blink slow (1 Hz)** = motion active / program running

### 🔵 Blue — Controller / Automation (overlay)

- Meaning: controller/software state
- Behaviour: **Off** = no host / inactive; **Solid** = host connected & controlling; **Blink slow** = firmware heartbeat (optional)

### 🟡 Yellow — Mode (overlay)

- Meaning: special operating mode
- Triggers: teach mode, calibration mode, manual jog
- Behaviour: **Solid** = mode active; **Blink slow** = recording / active calibration step

### ⚪ White — Utility (overlay)

- Meaning: non-safety information
- Behaviour: **Solid** = machine awake / workspace light; optional “dim” via lower PWM value

## 7. State → LED truth table

| Robot state | Red | Amber | Green | Blue | Yellow | White |
| --- | --- | --- | --- | --- | --- | --- |
| Powered off | OFF | OFF | OFF | OFF | OFF | OFF |
| Connected, not enabled | OFF | SOLID | OFF | SOLID | OFF | SOLID |
| Enabled, not homed | OFF | SOLID | OFF | SOLID | OFF | SOLID |
| Homing | OFF | BLINK | OFF | SOLID | OFF | SOLID |
| Ready idle | OFF | OFF | SOLID | SOLID | OFF | SOLID |
| Running | OFF | OFF | BLINK | SOLID | OFF | SOLID |
| Teach mode | OFF | OFF | SOLID | SOLID | SOLID | SOLID |
| Fault | SOLID / FAST BLINK | OFF | OFF | SOLID | OFF | SOLID |

## 8. How LEDs are driven in software

Because we are intentionally reusing servo outputs, logical states map to servo pulse widths:

| Logical state | Pulse width |
| --- | --- |
| OFF | 500 µs |
| ON (“solid”) | 2000 µs |
| DIM (optional) | 1200–1500 µs (1300 µs default) |
| BLINK | Toggle OFF / ON using the target pulse widths |

Blink timing is implemented in software using a timer-based loop (20–50 ms tick). This avoids touching reserved servo channels 0–7 and only emits pulses for the LED channels.

## 9. Example configuration (authoritative)

Place the configuration at `~/.config/lynxmotion_al5a/led_pins.json`:

```json
{
  "backend": "ssc32_pwm_led",
  "servo_reserved_channels": [0, 1, 2, 3, 4, 5, 6, 7],
  "led_channels": {
    "red": 8,
    "amber": 9,
    "green": 10,
    "blue": 11,
    "yellow": 12,
    "white": 13
  },
  "pulse_us": {
    "off": 500,
    "on": 2000,
    "dim": 1300
  },
  "blink": {
    "slow_hz": 1,
    "fast_hz": 4
  }
}
```

The LED driver validates this mapping, ignores reserved channels, and falls back to a no-op driver if the serial writer is unavailable.

## 10. Explicit prototype disclaimer (include verbatim)

> Prototype limitation: Status indicators are driven using unused servo PWM channels on the SSC-32(U). These outputs are not true GPIO and may exhibit flicker or brightness variation. This implementation is acceptable for prototype demonstration and functional validation only and is not intended for production or safety-rated use.

