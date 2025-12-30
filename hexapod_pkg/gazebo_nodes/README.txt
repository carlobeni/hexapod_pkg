============================================================
GAZEBO_NODES — SIMULATION-ONLY CONTROL & SENSOR EMULATION
============================================================

FOLDER:
-------
gazebo_nodes


============================================================
1) gz_hexapod_low_level_control.py
============================================================

NODE NAME:
----------
gz_hexapod_low_level_control


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Low-level motion controller for the hexapod **inside Gazebo only**.

This node converts high-level textual commands into either:
- continuous locomotion gaits, or
- discrete interpolated “social poses”

and publishes joint trajectories directly to the
Gazebo joint_trajectory_controller.

It is NOT designed to drive the real robot.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- std_msgs/String
  High-level symbolic commands, grouped into:

  LOCOMOTION:
    forward, backward,
    turn_left, turn_right,
    lateral_left, lateral_right,
    stop

  SOCIAL (gesture-level):
    rock,
    al_pelo,
    (future: hand-based gestures via MediaPipe)

OUTPUT:
-------
- trajectory_msgs/JointTrajectory
  Sent directly to the joint trajectory controller


------------------------------------------------------------
ARCHITECTURE
------------------------------------------------------------

The node internally separates motion into TWO DISTINCT
control paradigms:

1 LOCOMOTION (GAITS)


- BaseGait
    Abstract open-loop gait engine handling:
    - joint state feedback (position only)
    - phase timing
    - continuous trajectory streaming

- Concrete analytical gaits:
    - TripodForwardGait
    - TripodBackwardGait
    - RotateCCW / RotateCW
    - LateralLeftGait / LateralRightGait

Characteristics:
- Periodic
- Phase-based
- Continuous motion
- Designed to emulate walking dynamics



2 SOCIAL MOTION (POSE INTERPOLATION)


- BaseMotion
    Non-periodic interpolated motion engine:
    - single-shot target poses
    - smooth cosine interpolation
    - time-bounded execution

- SocialFrontLegsMotion
    Prototype class defining expressive poses
    using mainly front and middle legs.

Examples:
- "rock"      → expressive front-leg posture
- "al_pelo"   → neutral / reset pose

Characteristics:
- Discrete
- Non-cyclic
- Gesture-oriented
- No locomotion intent


------------------------------------------------------------
SOCIAL POSES & MEDIAPIPE CONTEXT
------------------------------------------------------------

Social poses are intended to be triggered by
**hand gesture recognition**, currently prototyped using
MediaPipe (hands / landmarks).

Conceptual pipeline (intended):

  Hand gesture (MediaPipe)
        ↓
  High-level symbolic command (e.g. "rock")
        ↓
  gz_hexapod_low_level_control
        ↓
  Interpolated expressive pose (SOCIAL mode)

CURRENT STATE:
- Fully functional and visually correct in Gazebo.
- Social poses are clearly distinguishable from locomotion.
- Smooth interpolation avoids gait interference.

LIMITATION (IMPORTANT):
- These poses ONLY EXIST in the simulation layer.
- The real robot does NOT implement joint-space poses.
- When sent to the real robot, these same commands are
  interpreted downstream as generic motion commands
  (forward / turn / stop), due to the Arduino firmware
  and serial protocol limitations.

Result:
→ On the real robot, “social gestures” collapse into
  basic locomotion behaviors.


------------------------------------------------------------
MODE MANAGEMENT
------------------------------------------------------------

The node operates in three exclusive modes:

- IDLE
    No active gait or pose.

- LOCOMOTION
    One continuous gait enabled.

- SOCIAL
    One-shot interpolated pose execution.

Rules:
- Entering SOCIAL disables all gaits.
- Entering LOCOMOTION exits SOCIAL automatically.
- STOP exits any mode and returns to IDLE.

This prevents gait–pose interference.


------------------------------------------------------------
GAIT / MOTION MODEL
------------------------------------------------------------

- Purely kinematic
- Open-loop
- No force feedback
- No dynamics compensation
- No hardware constraints

Tripod coordination is used for all gaits.
Social poses bypass gait logic entirely.


------------------------------------------------------------
DESIGN INTENT
------------------------------------------------------------

- Visual expressiveness in simulation
- Clear separation between:
    locomotion (continuous)
    vs
    gesture (discrete)
- Rapid prototyping of HRI concepts
- No assumptions about real hardware limits


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Simulation-only motion backend.

Acts as:
- locomotion synthesizer
- gesture visualizer
- experimental bridge for HRI (MediaPipe)

It is deliberately NOT a template for the real robot.

Real robot gestures must be re-implemented at the
Arduino / serial-protocol level, not here.


============================================================
2) gz_ir_emulator.py
============================================================

NODE NAME:
----------
gz_ir_emulator


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Emulates binary IR proximity sensors using Gazebo Range sensors.

Converts continuous distance readings into clean digital
obstacle detection signals.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- sensor_msgs/Range
  (left and right simulated IR sensors)

OUTPUT:
-------
- std_msgs/Bool
  Discrete obstacle detection (left / right)


------------------------------------------------------------
LOGIC
------------------------------------------------------------

- Hysteresis-based detection:
    - detect_threshold
    - release_threshold

- Independent state per sensor
- Rate-limited publishing (max ~50 Hz)
- Ignores invalid or non-finite readings


------------------------------------------------------------
DESIGN INTENT
------------------------------------------------------------

- Stable binary signals
- No flickering near thresholds
- Matches behavior of simple real IR modules


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Simulation substitute for real digital IR sensors.

Allows testing obstacle logic without hardware.


============================================================
3) test_pose_robot.py
============================================================

NODE NAME:
----------
pose_robot


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

One-shot pose tester for the hexapod in Gazebo.

Moves the robot into a predefined static configuration
for validation and debugging.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- sensor_msgs/JointState
  Used to read current joint positions

OUTPUT:
-------
- trajectory_msgs/JointTrajectory
  Single pose command


------------------------------------------------------------
BEHAVIOR
------------------------------------------------------------

- Waits until /joint_states are received
- Sends a single trajectory after 1 second
- Executes a smooth transition (2 seconds)
- Runs once and stops


------------------------------------------------------------
DESIGN INTENT
------------------------------------------------------------

- Mechanical validation
- Joint orientation checks
- Initial pose testing
- Debugging tool only


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Utility node.

Not part of navigation, control, or autonomy loops.


============================================================
GLOBAL NOTES
============================================================

- All nodes in this folder are:
    SIMULATION ONLY
    GAZEBO DEPENDENT
    NOT HARDWARE SAFE

- No dynamics, torque limits, or feedback control
- Intended for visualization, validation, and development


------------------------------------------------------------
SUMMARY
------------------------------------------------------------

This folder contains all Gazebo-specific glue logic:
motion generation, sensor emulation, and testing tools.

Nothing here should be executed on the real robot.

------------------------------------------------------------
END OF FILE
------------------------------------------------------------
