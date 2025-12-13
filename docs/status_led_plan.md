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
