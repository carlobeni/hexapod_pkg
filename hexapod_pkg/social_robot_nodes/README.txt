============================================================
social_robot_nodes / hand_gesture_node.py
============================================================

NODE NAME:
----------
hand_gesture_node


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

High-level human–robot interaction (HRI) node.

This node uses MediaPipe Hands to detect **hand expressions**
from a camera stream and translates them into
**symbolic robot commands** (strings).

Its role is purely semantic:
it does NOT generate motion, trajectories, or poses.
It only decides *what the user intends*.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- sensor_msgs/Image
  Camera image stream (USB cam / phone / webcam)

OUTPUT:
-------
- std_msgs/String
  High-level symbolic commands, such as:
    "rock"
    "al_pelo"
    "victory"

These commands are published to the same command topic
used by locomotion and social motion nodes.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This node is the **entry point of social interaction**.

It sits at the very top of the control hierarchy:

  Camera image
        ↓
  MediaPipe hand landmarks
        ↓
  Gesture classification
        ↓
  Symbolic command (String)
        ↓
  Motion / behavior nodes (simulation or real robot)

It deliberately ignores *how* the robot will execute the command.


------------------------------------------------------------
GESTURE DETECTION MODEL
------------------------------------------------------------

Gesture recognition is **geometric**, not learned:

- Uses MediaPipe hand landmarks (21 points)
- Computes distances relative to the palm center
- Applies deterministic thresholds

Defined gestures:

- ROCK
    Index + little finger extended
    Middle + ring folded
    Thumb outside palm

- AL_PELO (thumbs-up–like gesture)
    Thumb extended and upright
    Other fingers folded
    Angle and palm constraints applied

- VICTORY
    Index + middle extended
    Ring + little folded

This makes behavior:
- Deterministic
- Explainable
- Easy to tune
- Easy to break (by design)


------------------------------------------------------------
EVENT-BASED COMMAND PUBLISHING
------------------------------------------------------------

The node works in **event mode**, not continuous mode:

- A command is published ONLY when:
    - a gesture is detected
    - AND it is different from the previous gesture

This avoids:
- Command flooding
- Oscillation
- Continuous re-triggering

State variable:
- last_gesture


------------------------------------------------------------
SIMULATION VS REAL ROBOT
------------------------------------------------------------

IMPORTANT DISTINCTION:

In simulation:
- Commands like "rock" or "al_pelo"
  trigger dedicated SOCIAL poses
  in gz_hexapod_low_level_control.

On the real robot:
- These same commands are forwarded downstream
  to the DDS / serial pipeline.
- The Arduino firmware does NOT implement
  social poses.
- As a result, the robot interprets them as
  generic motion commands or ignores intent.

Outcome:
→ Social gestures visually exist ONLY in simulation.


------------------------------------------------------------
DESIGN INTENT
------------------------------------------------------------

- Prototype human–robot interaction
- Validate gesture semantics
- Decouple perception from motion
- Keep the system modular and replaceable

This node is NOT responsible for:
- safety
- arbitration
- motion validity
- hardware feasibility




