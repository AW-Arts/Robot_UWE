window.SSR_DATA = [
  {
    "id": "1.1.1.1",
    "sheet_row": 5,
    "text": "Teleop view shall allow setting and updating a target position in the XY plane via dedicated UI controls (e.g., click-to-set and/or nudge controls) with clear target/cursor feedback.",
    "verification": "Test",
    "pass": "Setting/updating the XY target via the teleop UI controls (e.g. nudge controls) updates the target marker smoothly and provides clear on-screen feedback of the current target.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.1.2",
    "sheet_row": 6,
    "text": "The system shall support direct move-to-target in teleop using incremental updates (not a single step).",
    "verification": "Test",
    "pass": "Incremental teleoperation target updates result in continuous motion toward the latest target",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.1.3",
    "sheet_row": 7,
    "text": "The UI shall provide a calibration/setup panel for servo offsets and limits.",
    "verification": "Test",
    "pass": "Adjust and save offset; restart keeps it.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.1.4",
    "sheet_row": 8,
    "text": "Teleop view shall allow adjusting the Z target via dedicated UI controls (e.g., slider and/or increment/decrement controls) with configurable step size.",
    "verification": "Test",
    "pass": "Z target changes using the Z controls (e.g., +/- step and/or slider) adjust by the configured step size and the displayed Z value matches the applied step.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.2.1",
    "sheet_row": 9,
    "text": "Comms loss shall be detected via timeout and transition to fault state.",
    "verification": "Test",
    "pass": "Unplug triggers fault within timeout.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.2.2",
    "sheet_row": 10,
    "text": "The UI shall show joint angles and/or PWM values in a diagnostics pane.",
    "verification": "Test",
    "pass": "Values update during motion.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.2.3",
    "sheet_row": 11,
    "text": "Forward kinematics shall compute end-effector position from joint angles.",
    "verification": "Test",
    "pass": "FK matches known pose within tolerance.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.2.4",
    "sheet_row": 12,
    "text": "Inverse kinematics shall compute joint angles from XYZ target and return success/failure.",
    "verification": "Test",
    "pass": "Unreachable returns failure.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.1.2.5",
    "sheet_row": 13,
    "text": "IK shall reject solutions that violate joint limits.",
    "verification": "Test",
    "pass": "Solution outside limits rejected.",
    "evidence": ""
  },
  {
    "id": "1.1.3.1",
    "sheet_row": 14,
    "text": "The planner shall avoid queue buildup by dropping intermediate teleop targets when new target arrives.",
    "verification": "Test",
    "pass": "Rapid commands do not create backlog.",
    "evidence": ""
  },
  {
    "id": "1.1.3.2",
    "sheet_row": 15,
    "text": "The system shall allow specifying gripper open/close actions at waypoints.",
    "verification": "Test",
    "pass": "Playback actuates gripper",
    "evidence": ""
  },
  {
    "id": "1.1.3.3",
    "sheet_row": 16,
    "text": "Timeline engine shall execute items sequentially and stop on fault or stop command.",
    "verification": "Test",
    "pass": "Fault stops remaining items.",
    "evidence": ""
  },
  {
    "id": "1.2.2.1",
    "sheet_row": 17,
    "text": "The repository shall include pinned dependencies (requirements.txt or lockfile) and setup instructions.",
    "verification": "Test",
    "pass": "Install on clean machine using docs.",
    "evidence": ""
  },
  {
    "id": "2.1.1.1",
    "sheet_row": 18,
    "text": "The UI shall group controls by mode (Teleop/Teach/Playback) and hide irrelevant controls when not active.",
    "verification": "Test",
    "pass": "Snapshot of grouped controls in all functional modes",
    "evidence": ""
  },
  {
    "id": "2.1.1.2",
    "sheet_row": 19,
    "text": "Teach mode shall show a waypoint list with add/edit/remove controls.",
    "verification": "Test",
    "pass": "Can add and remove waypoints.",
    "evidence": ""
  },
  {
    "id": "2.1.1.3",
    "sheet_row": 20,
    "text": "Waypoint editor shall allow inline editing of dwell time and validate ranges.",
    "verification": "Test",
    "pass": "Can add dwell and edit dwell and Invalid dwell rejected.",
    "evidence": ""
  },
  {
    "id": "2.1.1.4",
    "sheet_row": 21,
    "text": "Waypoint list shall support reordering via buttons or drag and drop.",
    "verification": "Test",
    "pass": "Reorder changes playback order.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.1.1.5",
    "sheet_row": 22,
    "text": "Waypoint names with identical text shall share a consistent colour tag in the UI.",
    "verification": "Test",
    "pass": "Same name outputs same colour; persisted across sessions.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.1.1.6",
    "sheet_row": 23,
    "text": "The UI shall allow grouping waypoints into named subroutines,  stored in a consistent representation (joint angles or XYZ) with explicit flag.",
    "verification": "Test",
    "pass": "Group and save subroutine, flag present",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.1.1.7",
    "sheet_row": 24,
    "text": "The UI shall provide per-joint jog buttons with configurable increment step.",
    "verification": "Test",
    "pass": "Increment size changes movement.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "1.2.2.1",
    "sheet_row": 25,
    "text": "The UI shall provide a 'dry-run' toggle that simulates motion and logs commands without actuating motors.",
    "verification": "Test",
    "pass": "Dry-run runs without sending PWM outputs.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.1.2.1",
    "sheet_row": 26,
    "text": "Controller shall support LED control channels or a status byte for LED driver.",
    "verification": "Inspection",
    "pass": "Switch mode; verify only relevant controls visible, LED displaying current state",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.1.2.2",
    "sheet_row": 27,
    "text": "Wiring diagram shall include mapping of controller pins/channels to servos, gripper, LEDs.",
    "verification": "Review",
    "pass": "Pin map included",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.1.2.3",
    "sheet_row": 28,
    "text": "The LED control show set logical states: ready, running, teach, fault.",
    "verification": "Test",
    "pass": "Set states updates snapshot.",
    "evidence": "Snapshot"
  },
  {
    "id": "2.1.2.4",
    "sheet_row": 29,
    "text": "Calibration panel shall include a required steps checklist.",
    "verification": "Document Review",
    "pass": "Checklist exists",
    "evidence": "Document Review"
  },
  {
    "id": "2.2.1.1",
    "sheet_row": 30,
    "text": "The UI shall include a routine list with create/rename/delete actions.",
    "verification": "Test",
    "pass": "Create/rename/delete works and persists.",
    "evidence": "Snapshot"
  },
  {
    "id": "2.2.1.2",
    "sheet_row": 31,
    "text": "The UI shall prevent destructive actions without confirmation (delete routine, reset config).",
    "verification": "Test",
    "pass": "Confirm dialog appears.",
    "evidence": "Snapshot"
  },
  {
    "id": "2.2.1.3",
    "sheet_row": 32,
    "text": "Timeline editor shall allow adding,  reordering routines & forming subroutines, stored in human readable JSON format",
    "verification": "Test",
    "pass": "Timeline list displayed and editable.",
    "evidence": "Snapshot"
  },
  {
    "id": "2.2.1.4",
    "sheet_row": 33,
    "text": "Timeline engine shall support looping, pause, resume and cancel",
    "verification": "Test",
    "pass": "Pause mid-run then resume completes.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.2.1.5",
    "sheet_row": 34,
    "text": "The application shall store configuration in a dedicated config directory and load it at startup.",
    "verification": "Test",
    "pass": "Configuration loaded on startup.",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "2.2.1.6",
    "sheet_row": 35,
    "text": "The planner shall publish commanded joint targets and computed XYZ for UI updates in consistent units (degrees/radians)",
    "verification": "Test",
    "pass": "UI updates at least 10 Hz.",
    "evidence": "Snapshot"
  },
  {
    "id": "3.1.1.1",
    "sheet_row": 36,
    "text": "The arm shall include standardized mounting holes (e.g., M3 pattern) for attaching alternative end-effectors",
    "verification": "Inspection",
    "pass": "Hole spacing of gripper mount matches drawings",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.1.2.1",
    "sheet_row": 37,
    "text": "All added mechanical components shall be reversible using standard fasteners",
    "verification": "Test",
    "pass": "Robot arm converted back to its original mechanical configuration with standard fastners",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.1.3.1",
    "sheet_row": 38,
    "text": "The arm extension component shall allow link replacement to increase reach without exceeding 5 mm tip deflection under 0.2 kg payload.[TBC]",
    "verification": "FEA",
    "pass": "TBC",
    "evidence": ""
  },
  {
    "id": "3.1.3.2",
    "sheet_row": 39,
    "text": "The arm structure shall withstand the maximum payload (including gripper) with a factor of safety \u2265 2",
    "verification": "Test",
    "pass": "Physical load test demonstrate movement of object, will mass equivalent to a factor of saftey \u22652 [TBC]",
    "evidence": "Test report [TBC]"
  },
  {
    "id": "3.1.3.3",
    "sheet_row": 40,
    "text": "The arm shall maintain tip deflection \u2264 5 mm under full payload during reach extension, validated",
    "verification": "FEA",
    "pass": "TBC",
    "evidence": ""
  },
  {
    "id": "3.1.3.4",
    "sheet_row": 41,
    "text": "The control panel shall include at least two mode-indicator bulbs and one error-status LED.",
    "verification": "Inspection",
    "pass": "Inspection of LED and their functionality",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.1.3.5",
    "sheet_row": 42,
    "text": "Switches shall allow selection between teleoperation and teach mode without requiring system reboot",
    "verification": "Test",
    "pass": "Perform a switch from one mode to another without rebooting, UI reflects change",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.2.1.1",
    "sheet_row": 43,
    "text": "The UI shall validate numeric inputs and show inline errors without crashing.",
    "verification": "Test",
    "pass": "Invalid inputs are rejected, UI remains operative",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.2.1.2",
    "sheet_row": 44,
    "text": "The UI shall provide a speed scaling control with numeric readout.",
    "verification": "Test",
    "pass": "Scale speed while in motion and record visible and recorded speed increase",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.2.1.3",
    "sheet_row": 45,
    "text": "The UI shall provide a Home button and show when home is in progress/completed.",
    "verification": "Test",
    "pass": "Selecting the home button is functional, UI displays 'in progress' and 'completed'",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.2.1.4",
    "sheet_row": 46,
    "text": "The UI shall support full-screen mode and responsive layout scaling.",
    "verification": "Test",
    "pass": "Toggle UI between full screen and modified layout scalling with no errors",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "3.2.1.5",
    "sheet_row": 47,
    "text": "The UI shall use small compact input fields with labels above inputs to maximize list space.",
    "verification": "Inspection",
    "pass": "UI input fields are compact and functional, without compromising screen space",
    "evidence": "Snapshot"
  },
  {
    "id": "4.1.1.1",
    "sheet_row": 48,
    "text": "A Work Breakdown Structure (WBS) shall be produced including the major tasks of the project, deliverables formatted clearly",
    "verification": "Document Review",
    "pass": "WBS exists, includes all majors tasks qwith dedicated owners",
    "evidence": "Document Review"
  },
  {
    "id": "4.1.1.2",
    "sheet_row": 49,
    "text": "A Gantt chart will be produced to document task durations and milestones along with a regularly updated critical path",
    "verification": "Document Review",
    "pass": "Gantt includes durations, milestones and baselines",
    "evidence": "Document Review"
  },
  {
    "id": "4.1.2.1",
    "sheet_row": 50,
    "text": "All formal project meetings should have documented minutes, along with the atendees, adjenda, notes and action items",
    "verification": "Document Review",
    "pass": "Minutes for all project meetings collated to one document",
    "evidence": "Document Review"
  },
  {
    "id": "4.1.3.1",
    "sheet_row": 51,
    "text": "A Risk Register should be produced and maintained, identifying risks, likelyhood, impact and mitigations",
    "verification": "Document Review",
    "pass": "Risk regiister exists, is up to date,  provides an overall assesment of risk",
    "evidence": "Document Review"
  },
  {
    "id": "5.1.1.1",
    "sheet_row": 52,
    "text": "A timeline shall be implemented that minimises dwell time, reducing unproductiveness of sequences.",
    "verification": "Test",
    "pass": "Time 3 consecutive assemblies at <25 seconds",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "5.1.1.2",
    "sheet_row": 53,
    "text": "Gripper shall release in <2 seconds to avoid delayed pick or placement.",
    "verification": "Test",
    "pass": "Time 3 consecutive removals of the end effector (gripper)\nStat conditions: Required tooling to hand, robot assembled.\nStop conditions: Gripper removed",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "5.1.1.3",
    "sheet_row": 54,
    "text": "UI shall display accurate elapsed time per assembly.",
    "verification": "Test",
    "pass": "Video evidence of timer functioning in demonstrated assembly test",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "5.1.2.1",
    "sheet_row": 55,
    "text": "Joint workspace limits to avoid overlapping envelopes",
    "verification": "",
    "pass": "",
    "evidence": ""
  },
  {
    "id": "5.1.3.1",
    "sheet_row": 56,
    "text": "Timestamp e.g. start/finish, Ready/busy on UI display",
    "verification": "Test",
    "pass": "Video evidence of timestamp change from start/finish on UI display",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "5.1.3.2",
    "sheet_row": 57,
    "text": "Daily production logging capability",
    "verification": "Test",
    "pass": "Photo evidence  of daily log following T5A testing",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "5.2.1.1",
    "sheet_row": 58,
    "text": "Preset speed selection with identifiable UI display buttons [TBC]",
    "verification": "Test",
    "pass": "Recorded speed change to predicted value when available speed presets are interchanged",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "5.2.1.2",
    "sheet_row": 59,
    "text": "Preset parameters stored in configuration file, loaded upon restart",
    "verification": "Test",
    "pass": "Run cancelled sequence and record position reverted to by robot is as expected from the previously taught process",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "6.1.1.1",
    "sheet_row": 60,
    "text": "Buttons to be high contrast and recognisable.",
    "verification": "Test",
    "pass": "Operate UI interface, record state changes",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "6.1.2.1",
    "sheet_row": 61,
    "text": "Current routine index serialization, joint targets and gripper state.",
    "verification": "Test",
    "pass": "Pause mid routine, resume, record success of state retention",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "6.1.2.2",
    "sheet_row": 62,
    "text": "Confirmation of next step (resume) with available preview before confirmation",
    "verification": "Test",
    "pass": "Pause mid routine, record previewed next end effector location before resuming, record resumed position matches predicted position",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "6.2.1.1",
    "sheet_row": 63,
    "text": "UI interface is variable in positioning to maximise ergonomics",
    "verification": "Test",
    "pass": "Adjust UI positioning to improve ergonomics, evidence will be provided verbally by the operator for recording in test documentation",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "6.2.1.2",
    "sheet_row": 64,
    "text": "UI layout and ergonomic use diagrams produced [TBC]",
    "verification": "Document Review",
    "pass": "Review of ergonomic documentation",
    "evidence": "Document Review"
  },
  {
    "id": "7.1.1.1",
    "sheet_row": 65,
    "text": "Colour coded and separately located raw and RTE produce/ tools",
    "verification": "Inspection",
    "pass": "Photograph of colour coded tools/consumable pickup locations",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "7.1.1.2",
    "sheet_row": 66,
    "text": "End effector protection against contamination",
    "verification": "",
    "pass": "",
    "evidence": ""
  },
  {
    "id": "7.1.2.1",
    "sheet_row": 67,
    "text": "The cleaning checklist shall include a step by step guides with dedicated sign-off by operator.",
    "verification": "Document Review",
    "pass": "Cleaning checklist exists and details the proceedure of cleaning, acceptable cleaning products, and a statement of conformity [TBC]",
    "evidence": "Document Review"
  },
  {
    "id": "7.1.2.2",
    "sheet_row": 68,
    "text": "Acceptable cleaning agents defined, to be included in cleaning checklist document, see 7.1.2.1",
    "verification": "Document Review",
    "pass": "Document review of Cleaning checklist, ensure acceptable cleaning products identified",
    "evidence": "Document Review"
  },
  {
    "id": "7.2.1.1",
    "sheet_row": 69,
    "text": "Fillet radius on internal corners, inspection for remaining residue to be addressed in cleaning checklist, see 7.1.2.1",
    "verification": "Inspection",
    "pass": "Photos to be taken of all internal corners radius to ensure fillet",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "9.1.1.1",
    "sheet_row": 70,
    "text": "PWM/ driver channels to be sized to LED current. Emergency override/fault forces red LED.",
    "verification": "Test",
    "pass": "LED operative on power up",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "10.1.1.1",
    "sheet_row": 72,
    "text": "Fasteners should not be hidden to ensure ease of access",
    "verification": "Inspection",
    "pass": "Photographs proving screws securing external covers are exposed",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "10.1.1.2",
    "sheet_row": 73,
    "text": "External cover removal sequence to be photographed",
    "verification": "Inspection",
    "pass": "Photographs of removing external covers",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "10.1.2.1",
    "sheet_row": 74,
    "text": "Diagnostics pane to feature sliders to jog motors and provide life positional feedback",
    "verification": "Test",
    "pass": "Jog each individual motor via UI sliders, record angle changes are max limits",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "10.2.1.1",
    "sheet_row": 75,
    "text": "Full parts list and full system maintenance diagrams",
    "verification": "Document review",
    "pass": "Review of part list and maintenance diagram",
    "evidence": "Document review"
  },
  {
    "id": "10.2.1.2",
    "sheet_row": 76,
    "text": "Where applicable serial numbers are to be used to identify parts",
    "verification": "Document review",
    "pass": "Serial numbers used are to be recorded in the parts list",
    "evidence": "Document review"
  },
  {
    "id": "11.1.1.1",
    "sheet_row": 78,
    "text": "Where applicable seals will be utilised to prevent the ingress of particulates",
    "verification": "Inspection",
    "pass": "The use of seals will be recorded",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "11.1.1.2",
    "sheet_row": 79,
    "text": "Where applicable the exposure of threads to the working environment will be minimised",
    "verification": "Inspection",
    "pass": "Exposed threads are to identified and included in the risk register",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "11.1.1.3",
    "sheet_row": 80,
    "text": "Pre use checklist to ensure food safety referencing EN 1672 2, see 7.1.2.1",
    "verification": "Document review",
    "pass": "See 7.1.2.1",
    "evidence": "Document review"
  },
  {
    "id": "11.1.2.1",
    "sheet_row": 81,
    "text": "Certificates of conformity for exposed metals and polymers stored",
    "verification": "Document review",
    "pass": "Review of statements of compliance/ certificates of conformities to be documented",
    "evidence": "Document review"
  },
  {
    "id": "11.1.2.2",
    "sheet_row": 82,
    "text": "Suppliers are required to supply a CofC, compliance statement etc.. with parts delivery",
    "verification": "Document review",
    "pass": "Review of statements of compliance/ certificates of conformities to be documented",
    "evidence": "Document review"
  },
  {
    "id": "12.1.1.1",
    "sheet_row": 84,
    "text": "Design adaptations are to utilise current bolt patterns and be interchangeable with stock configuration",
    "verification": "Inspection",
    "pass": "Photographs of old and new configuration mounting points documenting no change",
    "evidence": "Photographic/ Video recorded evidence"
  },
  {
    "id": "12.1.1.2",
    "sheet_row": 85,
    "text": "Pre and post modification audit documented to ensure compatibility",
    "verification": "Inspection",
    "pass": "Photographs of old and new configuration mounting points in assembled state, documenting no change",
    "evidence": "Photographic/ Video recorded evidence"
  }
];
