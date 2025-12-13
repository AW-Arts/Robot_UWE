# Status LED Scheme

This design treats status lights like an industrial stack light so each color has a single, clear meaning. The three primary lights are vertically stacked and always visible, while auxiliary indicators are used orthogonally for mode and controller information.

## Traffic-light stack (always visible)
- **Red (fault / unsafe)**
  - Solid: E-stop pressed, hard fault, limit hit, servo or controller error.
  - Blinking (fast): Fault just occurred and requires operator action.
  - Rule: if red is on, ignore other indicators.
- **Amber (attention / transitional)**
  - Solid: Enabled but not safe to run, homing required, interlock open, waiting for user confirmation.
  - Blinking (slow): Transitioning states such as enabling/disabling or homing in progress.
  - Meaning: do not touch yet.
- **Green (ready / running)**
  - Solid: Enabled, homed, ready for motion.
  - Blinking (slow): Program running or motion active.
  - Meaning: behaving as expected.

## Auxiliary indicators (orthogonal to the stack)
- **Yellow (mode / teach)**
  - Solid: Teach or calibration mode, manual jog mode.
  - Blinking: Recording waypoints, editing path, or a calibration step is active.
- **Blue (controller / software state)**
  - Solid: PC connected via USB and control software running.
  - Blinking (heartbeat): Firmware alive and main loop executing.
  - Off: No host or firmware not running.
- **Clear / White (utility / illumination)**
  - Solid: Workspace light, "machine awake" indicator, or panel backlight (can be PWM dim when idle).
  - Note: White never indicates errors.

## Final mapping
| LED | Role | Meaning |
| --- | --- | --- |
| Red | Stack | Fault / E-stop |
| Amber | Stack | Attention / interlock / transition |
| Green | Stack | Ready / running |
| Yellow | Mode | Teach / calibration |
| Blue | System | USB + firmware alive |
| Clear | Utility | Power / illumination |

## Wiring and configuration

Hardware pin assignments live in `~/.config/lynxmotion_al5a/led_pins.json` as a JSON
object mapping LED names to BCM/GPIO pin numbers:

```json
{
  "red": 17,
  "amber": 27,
  "green": 22,
  "yellow": 23,
  "blue": 24,
  "white": 25
}
```

If the config file is missing or contains no valid entries, the controller falls back
to a no-op hardware driver while still updating the logical LED states inside the UI.

### Electrical expectations

The driver interprets patterns as follows:

- `solid` &rarr; output driven high.
- `off` &rarr; output driven low.
- `blink_slow` &rarr; 1 Hz PWM at 50% duty cycle.
- `blink_fast` &rarr; 4 Hz PWM at 50% duty cycle.
- `dim` &rarr; ~200 Hz PWM at ~22% duty cycle.

Assuming common-cathode LEDs with individual current-limiting resistors (e.g. 220–470 Ω),
each pin drives the anode directly from the MCU or SBC GPIO header. Adjust the JSON map
to match your carrier board layout.
