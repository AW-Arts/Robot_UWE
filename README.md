# Robot UWE Motor Visualisation Controls

This project provides a lightweight Flask application that keeps the physical
robot arm and its visualisation in sync by exposing per-motor inversion
controls.  Each motor can be toggled through a simple web interface, ensuring
that clockwise/counter-clockwise differences between the simulation and the
real hardware are corrected globally.

## Features

- Persisted inversion state for every motor using a JSON configuration file.
- REST API for querying and updating motor inversion flags.
- Web dashboard with buttons that act as global inverters for each motor.
- Python utilities (`MotorInversionManager`) that can be reused in control
  scripts to apply the inversion state before sending commands to the robot.

## Getting started

1. Install the dependencies (preferably in a virtual environment):

   ```bash
   pip install -r requirements.txt
   ```

2. Launch the Flask development server:

   ```bash
   flask --app app run --debug
   ```

3. Open <http://localhost:5000> in your browser.  Use the buttons to invert
   any motor that behaves opposite to the visualisation.  The settings are
   saved in `config/motors.json` so the rest of your tooling can read the same
   configuration.

## Tests

Run the automated tests with:

```bash
pytest
```
