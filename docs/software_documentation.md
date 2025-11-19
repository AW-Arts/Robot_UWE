# Lynxmotion AL5A Software Documentation

This document summarises the existing codebase, how the modules collaborate, and the operational flows requested in the project brief. Diagrams use Mermaid syntax for readability.

## Software Architecture Diagram
```mermaid
flowchart LR
    User[(Operator)] --> CLI[CLI launcher (__main__.py)]
    CLI --> Interactive[InteractiveArm UI (interactive.py)]
    Interactive -->|inverse/forward kinematics| Kinematics[AL5AKinematics (al5a_kinematics.py)]
    Interactive -->|move_joints & relax_servos| Controllers[AL5ASerialController / PrintController (serial_comm.py)]
    Controllers -->|SSC-32 command bytes| Hardware[SSC-32/SSC-32U + servos]
    Interactive -->|calibration, waypoints, timeline| LocalFiles[~/.config/lynxmotion_al5a/*.json]
    Tests -->|fixtures & behaviours| Interactive
    Tests --> Kinematics
```

## Flowcharts

### Teleoperation
```mermaid
flowchart TD
    start([Start UI]) --> setpoint[Default home pose]
    setpoint --> input{User input?}
    input -->|Drag gizmo / scroll / sliders| target[Clamp target to workspace]
    input -->|Keyboard modifiers| target
    target --> ik[Inverse kinematics (wrist pitch + XYZ)]
    ik --> queue[Queue smoothed joint command]
    queue --> worker[Command worker thread]
    worker --> controller[move_joints to SSC-32 or simulator]
    controller --> feedback[Optional joint feedback -> visuals]
    feedback --> input
```

### Teach mode (waypoints/subroutines)
```mermaid
flowchart TD
    start([Enter Path panel]) --> record[Adjust pose + wrist]
    record --> addWaypoint[Add waypoint (position, duration, wrist pitch)]
    addWaypoint --> repeat{More waypoints?}
    repeat -->|Yes| record
    repeat -->|No| save[Save subroutine JSON]
    save --> playback[Play waypoints locally or add to timeline]
```

### Autoplay (timeline)
```mermaid
flowchart TD
    load[Load saved timeline JSON] --> select[Select subroutine entry]
    select --> hydrate[Load subroutine path + duration metadata]
    hydrate --> play[Queue waypoints sequentially]
    play --> done{Finished?}
    done -->|No & repeat enabled| select
    done -->|Yes| stop([Return to idle state])
```

### Safety & calibration
```mermaid
flowchart TD
    start([Command requested]) --> limits[Clamp against workspace + servo limits]
    limits --> smooth[Enforce joint speed + soft-start smoothing]
    smooth --> queue[Queue command]
    queue --> calib{Calibration active?}
    calib -->|Yes| relax[Relax servos + cancel pending]
    relax --> waitCalib[Await user exit from calibration]
    waitCalib --> limits
    calib -->|No| execute[Send to controller]
    execute --> monitor[Read feedback when available]
    monitor --> start
```

## State Machine Diagram
```mermaid
stateDiagram-v2
    [*] --> Home
    Home --> Idle: home move completes
    Idle --> Teleop: drag/scroll/slider input
    Teleop --> Idle: no input / command queue empty
    Idle --> Teaching: Path panel add/save subroutines
    Teaching --> Idle: subroutine saved or cleared
    Idle --> Playback: waypoint playback or timeline start
    Playback --> Idle: playback complete or stop pressed
    Idle --> Calibration: calibration button toggled
    Calibration --> Idle: exit calibration + restore offsets
    Calibration --> Relaxed: servos relaxed for safety
    Relaxed --> Calibration
```

## Pseudocode / Class Diagram

### Class overview
```mermaid
classDiagram
    class InteractiveArm {
        +target: np.ndarray
        +waypoints: list
        +timeline: list
        -_command_queue: queue
        +update_robot()
        +_send_move_command()
        +_command_worker()
    }
    class AL5AKinematics {
        +forward(joints)
        +inverse(position, wrist_pitch)
    }
    class AL5ASerialController {
        +move_joints(joints, move_time_ms)
        +relax_servos(indices)
    }
    class PrintController {
        +move_joints(...)
        +relax_servos(...)
    }
    InteractiveArm --> AL5AKinematics : uses
    InteractiveArm --> AL5ASerialController : sends pulses
    InteractiveArm --> PrintController : simulation
```

### Command path pseudocode
```
User adjusts target (drag, scroll, sliders)
    -> clamp to workspace and enforce wrist limits
    -> compute joints = kinematics.inverse(target, wrist_pitch)
    -> _send_move_command(joints, move_time_ms, soft_start)
        -> store setpoint + update visuals
        -> push (raw_joints, move_time, soft_start) to _command_queue
    -> _command_worker thread pops commands
        -> _generate_smooth_segments for soft start
        -> controller.move_joints(segment)
        -> mirror commanded joints into UI and optional feedback
```

## Inline Code Documentation

Recent updates add clarifying docstrings around the main entrypoint and critical control paths (queue management and smoothing) inside `InteractiveArm`. The class-level description now explains ownership of UI elements, calibration data, and threading, while helper methods document how commands flow from user input into the controller.

## Full Code Listing

See [`full_code_listing.md`](./full_code_listing.md) for the complete, current source for all Python modules and tests.
