# Operating Modes and Teach Workflow

## Live Mode
- All existing controls are available (IK jog, joint jog, gripper, subroutines, keyframes, etc.).
- Commands are streamed directly to the physical robot in real time.
- Intended for hands-on operation and quick hardware testing.

## Teach Mode
Teach Mode mirrors the Live Mode UI and tools but with two crucial differences:

1. **No live output to hardware**
   - No commands are transmitted to the robot.
   - Motion is reflected only in the local simulation/digital twin.
   - IK, jog controls, and gripper actions still work, but they manipulate the virtual state.

2. **Automatic recording enabled**
   - Every move and action is captured as it occurs—no need to start or stop recording manually.
   - Sessions produce a routine containing time-stamped joint angles or Cartesian targets, gripper state changes, speed/dwell metadata, and optional labels/markers.
   - "Real time" here refers to the timing of user interactions, not hardware execution.
   - The Matplotlib UI surfaces this as a **Modes** tab with Live/Teach/Run buttons. Entering Teach automatically starts a new recording and updates the status line with the running sample count.

## Smoothing (post-process)
Offer an optional smoothing step after a teach session, before saving/finalizing. Choose one approach:

- **Trajectory smoothing:** Fit splines or filters to reduce jitter and promote continuous motion.
- **Waypoint simplification:** Reduce the number of samples by keeping only key points (e.g., Douglas–Peucker or threshold-based pruning).
- **Velocity/acceleration limiting:** Apply jerk/acceleration limits for realistic, safer playback.

### Smoothing output
- Produces a cleaned routine optimized for Run Mode with fewer twitchy points, more consistent speed, and reduced IK noise.
- The **Smooth recording** button in the Modes panel applies a 3-point moving-average blend to joints, wrist poses, and timing metadata while preserving timestamps, giving you a quick post-teach cleanup pass without altering interaction timing.

### Saving the routine

- Teach sessions auto-save to `~/.config/lynxmotion_al5a/teach_sessions/teach_session_<timestamp>.json` when you leave Teach Mode or click **Save routine**.
- Saved files include raw samples plus the optional smoothed variant when smoothing is applied.

## Safety and UX rule
Only Run Mode executes commands on hardware. Entering Run Mode must require an explicit action (e.g., "ARM + RUN" or a clear confirmation) to prevent accidental motion and maintain a defensible separation between simulation and physical control.

The Modes tab enforces this by requiring an **Arm + confirm run** click before the Run button will activate hardware playback. If Run is disarmed, the UI reverts to Live and blocks hardware motion, keeping Teach sessions safely virtual.
