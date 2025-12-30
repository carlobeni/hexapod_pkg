============================================================
launch / OVERVIEW
============================================================

This folder contains launch files intended to be executed
from the **PC** to control the **real hexapod robot**.

Conceptually, this folder is **analogous to launch_sim**,
with one critical difference:

- There is NO Gazebo launch
- Communication with the real robot is handled via DDS


------------------------------------------------------------
CORE DIFFERENCE VS SIMULATION
------------------------------------------------------------

In simulation:
- gazebo_hexapod_sim.launch.py is the mandatory root

On the real robot:
- dds_comunication.launch.py is the mandatory root

ALL real-robot launches depend on DDS being active.


------------------------------------------------------------
MANDATORY EXECUTION ORDER
------------------------------------------------------------

1) dds_comunication.launch.py   ← ALWAYS FIRST
2) sensors_compute_real.launch.py (if navigation is used)
3) One operational mode:
   - follower
   - navigation to target
   - swarm follower
   - social robot


============================================================
launch / dds_comunication.launch.py
============================================================

NODE NAME:
----------
dds_comunication.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Establishes **DDS communication** between the PC and
the real hexapod robot.

This launch is **mandatory** and must always be started first.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This launch acts as the **communication backbone**.

It bridges:
- Sensors coming from the robot → ROS 2 topics
- High-level commands → serial commands sent to the robot

Without this launch:
→ No sensors
→ No commands
→ No robot


------------------------------------------------------------
INTERNAL STRUCTURE
------------------------------------------------------------

1) Preliminary cleanup
   - Kills previous DDS processes
   - Avoids duplicated listeners/talkers

2) DDS Fast Listener
   - High-frequency data
   - IMU, magnetometer, ultrasonic, IR, camera

3) DDS Reliable Listener
   - Low-frequency but critical data
   - GPS only

4) DDS Command Talker
   - Translates cmd_robot → serial commands
   - Applies speed scaling and trims
   - Sends commands to Arduino


------------------------------------------------------------
IMPORTANT NOTES
------------------------------------------------------------

- MUST be launched before any other real-robot launch
- Replaces Gazebo + bridges from simulation
- This is the real robot’s “nervous system”


============================================================
launch / sensors_compute_real.launch.py
============================================================

NODE NAME:
----------
sensors_compute_real.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Computes high-level state estimates
from **real robot sensors**.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This launch is the **estimation and monitoring layer**
for the real robot.

It mirrors sensors_compute_sim.launch.py,
but adapts to real hardware constraints.


------------------------------------------------------------
KEY DIFFERENCES VS SIM
------------------------------------------------------------

- Heading is taken directly from the phone
- IMU + magnetometer fusion is NOT used
- Dead reckoning is dominant


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- compute_gps_to_local_xy
  GPS → local XY coordinates

- compute_estimate_xy
  GPS + odometry → position estimate

- compute_ultrasonic_ranges
  Raw ultrasonic → distance in meters

- master_monitor
  Global system health monitor


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- dds_comunication.launch.py


============================================================
launch / navigation_to_target_real.launch.py
============================================================

NODE NAME:
----------
navigation_to_target_real.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Autonomous navigation toward a target
using real sensors.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Implements the same logic as the simulation version,
but using:
- Real camera
- Real GPS
- Real heading (phone-based)
- Real command channel


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- web_camera_node
  Provides camera images

- cube_viewer_yolo
  Detects target using vision

- target_navigation_with_obstacle_avoidance
  Generates motion commands


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- dds_comunication.launch.py
- sensors_compute_real.launch.py


============================================================
launch / follower_real.launch.py
============================================================

NODE NAME:
----------
follower_real.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Vision-based follower behavior
on the real robot.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

The robot follows a detected object
using camera input and obstacle awareness.


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- web_camera_node
- ball_viewer_yolo_follower
- follower_navigation_node


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- dds_comunication.launch.py
- sensors_compute_real.launch.py (recommended)


============================================================
launch / swarm_follower_real.launch.py
============================================================

NODE NAME:
----------
swarm_follower_real.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Follower behavior designed
for swarm-compatible logic.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Equivalent to swarm_follower_sim.launch.py,
but commanding the real robot.


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- web_camera_node
- ball_viewer_yolo_swarm
- swarm_navigation_with_obstacle_avoidance


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- dds_comunication.launch.py


============================================================
launch / social_robot_real.launch.py
============================================================

NODE NAME:
----------
social_robot_real.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Human–robot interaction via hand gestures
on the real robot.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Maps hand gestures to **real motion commands**.

Unlike simulation:
- Gestures produce actual robot movement
- No social poses exist at firmware level


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- web_camera_node
- hand_gesture_node


------------------------------------------------------------
IMPORTANT NOTES
------------------------------------------------------------

- Gesture-to-command mapping differs from simulation
- Final behavior depends on Arduino firmware
