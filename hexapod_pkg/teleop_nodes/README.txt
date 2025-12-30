============================================================
TELEOP_NODES — MANUAL COMMAND INTERFACE
============================================================

FOLDER:
-------
teleop_nodes


============================================================
GENERAL CONTEXT
============================================================

This folder contains keyboard-based teleoperation nodes.

Their purpose is deliberately minimal:
- Read raw keyboard input from the terminal
- Translate key presses into high-level motion commands
- Publish commands as std_msgs/String to the appropriate ROS topic

These nodes do NOT:
- Generate trajectories
- Control joints
- Apply safety logic
- Interpret motion semantics

They are pure human → ROS command bridges.


============================================================
DESIGN PHILOSOPHY
============================================================

- Stateless command emission
- Human-in-the-loop control
- No dependency on ROS parameters beyond the output topic
- Identical command vocabulary for:
    - real robot
    - Gazebo simulation

This guarantees that:
- Simulation and hardware behave identically at the command level
- Higher-level control nodes remain interchangeable


============================================================
COMMON BEHAVIOR (BOTH NODES)
============================================================

Both teleop nodes share the same internal structure:

------------------------------------------------------------
INPUT
------------------------------------------------------------
- Non-blocking keyboard input
- Terminal in raw mode (tty / termios)
- Separate thread for keyboard handling

------------------------------------------------------------
OUTPUT
------------------------------------------------------------
- std_msgs/String command messages
- Published only when the command changes
  (simple software debouncing)

------------------------------------------------------------
KEY MAPPING
------------------------------------------------------------

    w  → forward
    s  → backward
    a  → turn_left
    d  → turn_right
    q  → lateral_left
    e  → lateral_right
    x  → stop

------------------------------------------------------------
TERMINAL SAFETY
------------------------------------------------------------

- Terminal is always restored on exit
- Ctrl+C is explicitly handled
- Prevents terminal corruption after node shutdown

This is CRITICAL for long development sessions.


============================================================
1) teleop_hexapod_real.py
============================================================

NODE NAME:
----------
tele_op_hexapod_real


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Manual teleoperation interface for the REAL hexapod robot.


============================================================
2) teleop_hexapod_sim.py
============================================================

NODE NAME:
----------
tele_op_hexapod_real
(Note: node name kept identical intentionally)


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Manual teleoperation interface for Gazebo simulation.


------------------------------------------------------------
INTENTIONAL DUPLICATION
------------------------------------------------------------

Aside from the output topic:
- Code is intentionally identical to the real teleop node
- Key mappings are identical
- Command strings are identical

This ensures perfect command-level symmetry between:
    REAL ↔ SIMULATION


============================================================
ARCHITECTURAL POSITION
============================================================

          [ Keyboard ]
               ↓
        teleop_hexapod_*
               ↓
     std_msgs/String commands
               ↓
    ┌───────────────┬────────────────┐
    │                               │
REAL ROBOT                    GAZEBO SIM
(command_listener)     (gz_low_level_control)
    │                               │
Arduino / Motors          JointTrajectory



------------------------------------------------------------
END OF FILE
------------------------------------------------------------
