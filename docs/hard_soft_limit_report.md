# Hard Limits, Soft Limits, and Inverse Kinematics in the Lynxmotion AL5A Control Stack

## Overview
This report describes how mechanical hard limits, software soft limits, and the inverse kinematics (IK) pipeline interact in the Lynxmotion AL5A controller by unifying the mapping logic in `ServoConfig`, the kinematics flow in `AL5AKinematics`, and the calibration steps from the servo limit guide.

## Key Definitions
- **Hard limits (mechanical):** Physical end stops of a servo or linkage that cannot be exceeded without risking damage. They define the widest angle span and pulse span that the controller can safely command. 【F:docs/servo_limit_clamping_guide.md†L8-L23】
- **Soft limits (software):** Intentionally tighter angle bounds used by the planner. They keep commanded motion away from hardware stops while preserving the original angle-to-pulse scaling derived from the hard limits. 【F:docs/servo_limit_clamping_guide.md†L8-L34】
- **Pulse span:** Difference between `max_pulse` and `min_pulse`, expressed in microseconds, describing the electrical range mapped to the mechanical span. 【F:docs/servo_limit_clamping_guide.md†L8-L23】
- **Pulses per radian/degree:** Resolution of the mapping from mechanical rotation to commandable pulses. The controller keeps this ratio constant even when soft limits are tightened. 【F:docs/servo_limit_clamping_guide.md†L26-L35】

## ServoConfig: Mapping Angles to Pulses
Each servo channel is described by a `ServoConfig` dataclass containing `min_angle`, `max_angle`, `min_pulse`, and `max_pulse`. When a target angle arrives, the controller clamps it to the soft limits, scales it linearly across the hard-limit span to produce a pulse, and can perform the inverse mapping when feedback pulses need to become angles again. 【F:lynxmotion_control/al5a_kinematics.py†L16-L38】

## Default Limits per Joint
The default configuration in `DEFAULT_SERVO_CONFIGS` establishes spans and pulses for six servos. Representative values include ±90° for the base (500–2500 µs) and a symmetric ±π span for wrist rotation. 【F:lynxmotion_control/al5a_kinematics.py†L95-L116】 These serve as the hard-limit basis; soft limits can tighten the angles without changing pulses. 【F:docs/servo_limit_clamping_guide.md†L26-L35】

## Inverse Kinematics Workflow
The IK pipeline translates target tool poses into joint angles that subsequently pass through limit enforcement:
1. `AL5AKinematics.inverse` receives a Cartesian position and wrist pitch. 【F:lynxmotion_control/al5a_kinematics.py†L62-L120】
2. The base angle comes from `atan2`; wrist offset compensation then forms a planar triangle for the shoulder and elbow. 【F:lynxmotion_control/al5a_kinematics.py†L85-L118】
3. The joint vector flows into `joints_to_pulses`, which clamps and converts each angle per its `ServoConfig`. 【F:lynxmotion_control/al5a_kinematics.py†L120-L161】

## Interaction Between Hard Limits, Soft Limits, and IK
- **Hard limits anchor scaling.** The min/max pulses and angles in `ServoConfig` define the linear conversion ratio. IK outputs are evaluated against these spans when converted to pulses. 【F:lynxmotion_control/al5a_kinematics.py†L16-L38】
- **Soft limits provide runtime safety.** During `angle_to_pulse`, angles exceeding `min_angle` or `max_angle` are clamped before scaling, preventing commands that would push the servo to its physical stops. 【F:lynxmotion_control/al5a_kinematics.py†L16-L29】
- **IK respects reachable workspace but not hardware margin.** The inverse solver guards against unreachable points by scaling targets back to maximum reach, but it does not embed per-joint margins. The soft limits applied after IK supply that margin. 【F:lynxmotion_control/al5a_kinematics.py†L72-L118】
- **Consistent resolution.** Because the pulse-per-degree ratio is derived from the hard limits, tightening soft limits does not change command resolution; planners can reuse the same trajectory discretization. 【F:docs/servo_limit_clamping_guide.md†L26-L35】

## Calibration Data Required per Motor
Accurate limits require per-motor data: mechanical span, pulses at each stop, desired safety margin, neutral pulse references, and any load behavior near stops. 【F:docs/servo_limit_clamping_guide.md†L37-L61】 These details enable reliable updates to `DEFAULT_SERVO_CONFIGS` across Python and MATLAB workflows. 【F:docs/servo_limit_clamping_guide.md†L63-L78】

## Motor Limit Diagrams
Below are simplified ASCII diagrams showing the relationship between hard and soft limits for a typical rotational servo.

```
Hard limit span (physical):
[min_angle_hard]------------------------------[max_angle_hard]
                      ^ neutral/zero

Soft limit span (software safety margin):
         [min_angle_soft]--------------[max_angle_soft]
                      ^ neutral/zero

Pulse mapping (electrical command range):
[min_pulse]-----------------------------------------[max_pulse]
         |------------- same span factor ------------|
```

For the base servo (index 0), the hard span is approximately -π/2 to +π/2 with pulses 500–2500 µs. A soft limit could narrow that to -1.4 to +1.4 rad while preserving the 2000 µs pulse span. 【F:lynxmotion_control/al5a_kinematics.py†L95-L116】

## System Flow Diagram
The flow from Cartesian targets to hardware pulses is:

```
Cartesian target (x, y, z, wrist_pitch)
          |
          v
 AL5AKinematics.inverse
          |
 [base, shoulder, elbow, wrist]
          |
          v
 joints_to_pulses (clamp -> scale -> pulse)
          |
          v
 SSC-32 channel pulse array
```

## How Limits Influence Planning and Safety
- **Trajectory planning:** Keep waypoints inside soft limits to avoid clamping artifacts; the pulse ratio stays fixed for timing calculations. 【F:docs/servo_limit_clamping_guide.md†L26-L35】
- **Hardware protection:** Soft limits absorb IK overshoot while hard-limit spans anchor scaling. 【F:docs/servo_limit_clamping_guide.md†L8-L35】
- **Feedback interpretation:** `pulse_to_angle` uses the same ratio as forward mapping, aligning outbound and inbound conversions. 【F:lynxmotion_control/al5a_kinematics.py†L31-L38】

## Practical Calibration and Implementation Steps
To refine limits or add new servos, follow the process distilled in the servo guide: bench-calibrate each servo to locate mechanical stops, compute pulses-per-degree from measured spans, choose soft limits by trimming 5–10° from each hard stop while keeping the same pulses, and validate with round-trip tests near the limits. 【F:docs/servo_limit_clamping_guide.md†L37-L78】【F:lynxmotion_control/al5a_kinematics.py†L95-L116】

## Example: Shoulder Joint Limit Configuration
The shoulder servo (index 1) spans roughly -0.35 to 2.0 rad with 500–2500 µs pulses. 【F:lynxmotion_control/al5a_kinematics.py†L95-L116】 If binding appears near the upper stop, soften to -0.25 and 1.85 rad while retaining the pulses so clamping protects the joint without changing resolution.

## Edge Cases and Failure Modes
- **Zero spans:** The code raises a `ValueError` if either angle or pulse span is zero, preventing division by zero and signaling misconfiguration. 【F:lynxmotion_control/al5a_kinematics.py†L24-L38】
- **Missing channels/configs:** `joints_to_pulses` and `pulses_to_joints` throw `KeyError` when servo indices or channel mappings are absent, ensuring every joint has explicit limits before command generation. 【F:lynxmotion_control/al5a_kinematics.py†L126-L159】
- **Unreachable Cartesian targets:** The IK solver scales targets back to the arm’s maximum reach rather than failing, which can produce angles near soft limits; clamping then protects the hardware. 【F:lynxmotion_control/al5a_kinematics.py†L72-L118】

## Recommendations for Deployment
- Keep a per-servo calibration table in version control and update `DEFAULT_SERVO_CONFIGS` after hardware changes. 【F:docs/servo_limit_clamping_guide.md†L63-L94】
- Surface UI warnings when clamping occurs so operators see when commands are constrained. 【F:docs/servo_limit_clamping_guide.md†L79-L94】
- Use conservative soft margins for joints with heavy loads or back-driving, and re-validate mappings after firmware or geometry changes. 【F:docs/servo_limit_clamping_guide.md†L37-L94】

## Conclusion
Hard limits set the conversion ratio between angles and pulses, while soft limits provide the safety margin. IK supplies joint targets, and the clamping/mapping path keeps them inside safe electrical and mechanical bounds.
