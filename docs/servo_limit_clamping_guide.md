# Servo Limit and Clamping Guide

This document explains how the Lynxmotion controller maps joint angles to servo pulses, how the hard limits are enforced, and what information you need to characterise each motor. The goal is to make limit configuration reproducible rather than confusing.

## Core definitions

- **Hard limits (mechanical):** Physical end stops of the servo or linkage. Approaching them risks damage. Hard limits set the safe pulse span the controller is allowed to command.
- **Command saturation:** Requests outside the hard limits are clipped to the nearest safe pulse so the controller never exceeds the calibrated mechanical range.
- **Pulse span:** `pulse_span = max_pulse - min_pulse` (microseconds).
- **Angle span:** `angle_span = max_angle - min_angle` (radians).
- **Pulses per radian:** `ppr = pulse_span / angle_span`.
- **Pulses per degree:** `ppd = ppr * (π / 180)`.

## How mapping and clamping work today

The `ServoConfig` class drives every conversion between angles and pulse widths:

- `clamp_angle(angle)` enforces the calibrated hard limits by bounding `angle` to `min_angle <= angle <= max_angle`.
- `angle_to_pulse(angle)` first clamps the angle, then linearly maps it into the pulse span using the proportion of the angle span.
- `pulse_to_angle(pulse)` performs the inverse linear mapping from pulse width to angle.

All runtime clamping in the interactive controller (inverse kinematics saturation, manual joint nudges, and calibration limit updates) routes through these `ServoConfig` entries. That keeps the IK solver, UI feedback, and persisted configuration on the same hard-limit system instead of duplicating bound logic elsewhere.

Formally, for any joint:

```text
clamped = clamp(angle, min_angle, max_angle)
proportion = (clamped - min_angle) / angle_span
pulse = round(min_pulse + proportion * pulse_span)
```

The default servo table in `al5a_kinematics.py` stores the hard-limit data used for this mapping. For example, servo 0 (base) uses ±90° with 500–2500 µs pulses. 【F:lynxmotion_control/al5a_kinematics.py†L15-L75】【F:lynxmotion_control/al5a_kinematics.py†L96-L124】

## Data to gather per motor

To configure limits confidently, collect the following for each servo/channel:

1. **Mechanical angle span (hard):** The usable rotation between physical stops in radians or degrees.
2. **Matching pulse widths (hard):** Pulse widths (µs) at each stop. These define `min_pulse` and `max_pulse`.
3. **Neutral or reference pulse:** Pulse width that yields a known angle (often 1500 µs for “zero”); useful for sanity checks and for tuning symmetrical ranges.
4. **Resolution needs:** Minimum commanded increment you care about, so you can confirm `ppd` is sufficient for your application.
5. **Load/torque considerations:** Note if a joint regularly back-drives or stalls near a limit, suggesting additional margin or slower travel times.

## Recommended improvement process

1. **Bench-calibrate each servo:** With the linkage disconnected, sweep pulses to locate safe mechanical stops and record `(min_pulse, max_pulse)` and the corresponding angles.
2. **Compute mapping ratios:** Use `ppd = (max_pulse - min_pulse) / angle_span * (π / 180)` to understand command resolution; aim for consistent ratios across joints when possible.
3. **Update `ServoConfig` entries:** Enter the calibrated pulses and hard limits in `DEFAULT_SERVO_CONFIGS` (or your custom config file) so both Python and MATLAB workflows stay aligned.
4. **Validate conversions:** For each joint, pick test angles near and beyond the hard limits. Confirm `angle_to_pulse` clamps correctly and that `pulse_to_angle` round-trips within tolerance.
5. **Document per-servo specs:** Record the chosen limits, margins, and `ppd` so future changes remain transparent.
6. **Add runtime warnings:** Ensure UI and command paths surface clear messages when a request hits a hard limit, pointing to the per-servo documentation.

## Action checklist

- [ ] Measure hard-stop angles and pulses for every servo (base, shoulder, elbow, wrist, wrist rotate, gripper).
- [ ] Derive `angle_span`, `pulse_span`, `ppr`, and `ppd` for each motor and log the results.
- [ ] Decide safety margins and record the hard limits that avoid the measured stops while preserving the same `ppd`.
- [ ] Update `DEFAULT_SERVO_CONFIGS` (or a new config file) with the calibrated values.
- [ ] Add a short per-servo note (limit table) to the repo so future users can see the calibrated spans and margins.
- [ ] Run end-to-end tests that command angles near limits to verify clamping and logging behaviour.
