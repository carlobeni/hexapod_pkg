============================================================
launch_sim / gazebo_hexapod_sim.launch.py
============================================================

NODE NAME:
----------
gazebo_hexapod_sim.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Main entry point for the **hexapod simulation**.

This launch file is **mandatory** to run any simulation mode.
It initializes the complete Gazebo + ROS 2 simulation pipeline
and spawns the robot into the virtual world.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This launch is the **root of the simulation hierarchy**.

It is responsible for:

- Loading the Gazebo world
- Starting Gazebo (ros_gz_sim)
- Launching robot_state_publisher (via rsp.launch.py)
- Spawning the hexapod entity
- Starting controllers
- Bridging Gazebo ↔ ROS 2 topics
- Launching low-level simulated control
- Launching simulated IR sensors

All other simulation modes **depend on this launch**.


------------------------------------------------------------
INTERNAL STRUCTURE
------------------------------------------------------------

1) Includes rsp.launch.py
   - Generates robot_description from XACRO
   - Enables sim time
   - Enables ros2_control

2) Launches Gazebo with selected world
   - Default world:
     navigation_with_box_world.world

3) Spawns the robot in Gazebo
   - Uses /robot_description
   - Entity name is arbitrary (single robot case)

4) Starts controllers
   - joint_state_broadcaster
   - joint_trajectory_controller

5) Starts Gazebo ↔ ROS bridges
   - Generic topics (gz_bridge.yaml)
   - Camera image bridge

6) Starts simulation-specific nodes
   - gz_hexapod_low_level_control
   - gz_ir_emulator


------------------------------------------------------------
IMPORTANT NOTES
------------------------------------------------------------

- This launch MUST be executed before:
  - sensor computation
  - navigation
  - follower modes
  - social interaction

- Without this launch:
  → No robot
  → No Gazebo
  → No simulation

============================================================
launch_sim / rsp.launch.py
============================================================

NODE NAME:
----------
rsp.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Publishes the robot kinematic model to ROS 2.

It converts the XACRO description into URDF and
feeds it to robot_state_publisher.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This launch is responsible for the **existence of the robot model**.

It provides:
- /robot_description
- TF tree
- Joint states (simulation-aware)

It does NOT spawn the robot.
It only describes it.


------------------------------------------------------------
KEY PARAMETERS
------------------------------------------------------------

- use_sim_time
  Enables Gazebo simulation clock

- use_ros2_control
  Enables ros2_control interfaces

- sim_mode
  Passed to XACRO to switch behavior


------------------------------------------------------------
IMPORTANT NOTES
------------------------------------------------------------

- Always launched indirectly by gazebo_hexapod_sim.launch.py
- Should never be launched standalone for simulation

============================================================
launch_sim / sensors_compute_sim.launch.py
============================================================

NODE NAME:
----------
sensors_compute_sim.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Computes **high-level sensor estimates** from raw simulated data.

This launch transforms raw Gazebo sensors into
usable navigation signals.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This launch acts as the **sensor fusion and monitoring layer**.

It must be launched **before navigation to target**.
For other modes, it is optional.


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- compute_heading
  IMU + magnetometer → heading angle

- compute_gps_to_local_xy
  GPS lat/lon → local XY coordinates

- compute_estimate_xy
  Dead-reckoning + GPS → position estimate

- compute_ultrasonic_ranges
  Raw ultrasonic → distance in meters

- master_monitor
  Global health and data consistency monitor


------------------------------------------------------------
IMPORTANT NOTES
------------------------------------------------------------

- Fundamental for autonomous navigation
- Provides all state estimation used by planners
- Acts as the system “nervous system”

============================================================
launch_sim / navigation_to_target_sim.launch.py
============================================================

NODE NAME:
----------
navigation_to_target_sim.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Autonomous navigation toward a target
with obstacle avoidance.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Implements **goal-oriented navigation**.

Consumes:
- Vision
- Estimated pose
- Heading
- Ultrasonic data

Produces:
- Motion commands to the robot


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- cube_viewer_yolo
  Detects target cube using YOLO

- target_navigation_with_obstacle_avoidance
  Plans motion toward target while avoiding obstacles


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- gazebo_hexapod_sim.launch.py
- sensors_compute_sim.launch.py

============================================================
launch_sim / follower_sim.launch.py
============================================================

NODE NAME:
----------
follower_sim.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Vision-based follower behavior.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

The robot follows a detected object
using camera input and obstacle awareness.


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- ball_viewer_yolo_follower
  Detects target object

- follower_navigation_node
  Generates follower motion commands


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- gazebo_hexapod_sim.launch.py
- sensors_compute_sim.launch.py (recommended)

============================================================
launch_sim / swarm_follower_sim.launch.py
============================================================

NODE NAME:
----------
swarm_follower_sim.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Swarm-oriented follower behavior.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Implements follower logic designed
to scale to multiple robots.


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- ball_viewer_yolo_swarm
  Vision node for swarm context

- swarm_navigation_with_obstacle_avoidance
  Swarm-compatible navigation logic


------------------------------------------------------------
DEPENDENCIES
------------------------------------------------------------

Requires:
- gazebo_hexapod_sim.launch.py

============================================================
launch_sim / social_robot_sim.launch.py
============================================================

NODE NAME:
----------
social_robot_sim.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Human–robot interaction via hand gestures.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

Maps **hand gestures** to symbolic commands
that are interpreted as robot actions.

This mode is **purely social and experimental**.


------------------------------------------------------------
NODES LAUNCHED
------------------------------------------------------------

- web_camera_node
  Captures camera images

- hand_gesture_node
  Recognizes gestures and publishes commands


