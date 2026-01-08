# Lynxmotion AL5A Interactive Controller

This repository now includes both a Python package, `lynxmotion_control`, and a
MATLAB utility for commanding a Lynxmotion AL5A manipulator.  The tools offer:

- Basic forward and inverse kinematics for the Lynxmotion AL5A geometry.
- Conversion between joint angles and servo pulses for SSC-32/SSC-32U controllers.
- Serial communication helpers with both hardware and simulation backends.
- A matplotlib-based UI that allows you to drag the end-effector target in the
  XY plane while using the mouse wheel to adjust height.  Every interaction is
  translated into servo commands for the AL5A.
- A MATLAB `teach`-based interface that mirrors joint slider movements to an
  SSC-32/SSC-32U controller, letting you keep your workflow inside the
  Robotics Toolbox environment.

## Requirements

- Python 3.10+
- `numpy`
- `matplotlib`
- `pyserial` (only when talking to real hardware)

You can install the dependencies with:

```bash
pip install -r requirements.txt
```

## Running the interactive controller (Python)

To run in simulation mode (no hardware required) simply execute:

```bash
python -m lynxmotion_control --simulate
```

When you are ready to connect to a physical Lynxmotion arm via an SSC-32 or
SSC-32U controller specify the serial port, for example on Windows:

```bash
python -m lynxmotion_control --port COM3
```

or on Linux/macOS:

```bash
python -m lynxmotion_control --port /dev/ttyUSB0
```

The window shows a top-down view of the arm.  Drag the red target to reposition
the end-effector in the XY plane and use the mouse wheel to move it up or down.
Commands are streamed to the controller with the configured travel time.

Use the **Modes** panel to switch between:
- **Live Mode:** sends IK jogs, joint jogs, gripper changes, and subroutines to the real robot in real time.
- **Teach Mode:** mirrors the Live UI but drives only the digital twin while auto-recording every move with timestamps, gripper states, and dwell metadata; recordings auto-save and can be smoothed before finalizing.
- **Run Mode:** the only mode that replays routines on hardware, gated by an explicit **Arm + confirm run** step to prevent accidental motion.

## Notes

- The kinematic model uses the standard Lynxmotion AL5B link lengths (4.75"
  shoulder-link, 5.00" forearm-link).  If your hardware differs you can adjust
  the values in `al5a_kinematics.py`.
- Servo ranges are approximate.  Calibrate the min/max pulse widths to match
  your individual servos.
- The UI sends commands immediately when the target changes.  Consider slowing
  the command rate or batching updates if your hardware requires it.
- When a manual servo adjustment reaches a configured limit the controller logs
  a warning but still re-sends the command so the hardware stays in sync.
- See `docs/teach_mode.md` for a breakdown of Live Mode vs. Teach Mode, the
  post-teach smoothing step, and the Run Mode confirmation rule that prevents
  accidental motion on hardware.

### Configuration folder

- Config files live in the per-OS app data folder:
  - Windows: `%APPDATA%\lynxmotion_al5a`
  - macOS: `~/Library/Application Support/lynxmotion_al5a`
  - Linux/other: `~/.config/lynxmotion_al5a`
- The Motor configuration panel includes an **Open config folder** button that
  opens the directory in your OS file manager.
- On first launch the app seeds that folder **only if it is empty**, copying the
  entire bundled defaults directory (servo offsets, LED pin map, saved
  subroutines/timelines, etc.) so users start with a working configuration they
  can edit or replace.

## Status LEDs

Stack lights are driven directly from unused SSC-32(U) servo channels (no extra
GPIO). Follow the locked channel map, wiring rules, and behaviours in
`docs/status_led_plan.md`, and configure the mapping in the app data folder
(`led_pins.json` alongside the other config files listed above).

> Prototype limitation: Status indicators are driven using unused servo PWM
> channels on the SSC-32(U). These outputs are not true GPIO and may exhibit
> flicker or brightness variation. This implementation is acceptable for
> prototype demonstration and functional validation only and is not intended for
> production or safety-rated use.
