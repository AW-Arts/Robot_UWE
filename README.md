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

## Running the interactive controller (MATLAB)

If you prefer to remain in MATLAB, use the `al5a_teach.m` helper which is built
on Peter Corke's Robotics Toolbox (`rvctools`).  Ensure the toolbox is on your
path, then launch the teach pendant:

```matlab
al5a_teach
```

Pass a serial port to stream the slider commands to an SSC-32/SSC-32U
controller:

```matlab
al5a_teach("COM3", 'TravelTime', 1.0);
```

Use the slider controls to manipulate the arm.  Joint commands are converted to
servo pulses with the same limits as the Python implementation, keeping the two
workflows aligned.

## Notes

- The kinematic model uses approximate link lengths suitable for the stock
  Lynxmotion AL5A.  If your hardware differs you can adjust the values in
  `al5a_kinematics.py`.
- Servo ranges are approximate.  Calibrate the min/max pulse widths to match
  your individual servos.
- The UI sends commands immediately when the target changes.  Consider slowing
  the command rate or batching updates if your hardware requires it.
