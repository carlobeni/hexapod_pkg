FOLDER:
-------
master_nodes


OVERVIEW:
---------
This folder contains the master supervision interface of the robot.
Its main purpose is to provide a human–machine interface (HMI) that allows:

- Real-time monitoring of the robot’s internal state
- Manual inspection of sensor and estimation outputs
- Injection of a navigation target expressed in GPS coordinates

This folder does NOT implement navigation or control logic.
It acts as an external observer and high-level command interface.


ARCHITECTURAL ROLE:
-------------------
The master monitor sits outside the autonomous control loop.

It subscribes to multiple internal topics to visualize the robot state,
and publishes a single high-level goal message to be consumed by the
navigation algorithm.

It is intentionally separated from decision-making nodes.


DATA FLOW (CONCEPTUAL):
----------------------
Robot Sensors & Estimators
        ↓
   master_monitor  ─────→  Goal (GPS)
        ↑
   Human Operator (GUI)


------------------------------------------------------------
NODE: master_monitor
------------------------------------------------------------

Purpose:
--------
Provides a graphical user interface (GUI) for supervising the robot’s state
and manually sending a navigation target defined by latitude and longitude.

The node combines ROS2 communication with a Tkinter-based desktop interface.

Inputs (Subscriptions):
-----------------------
- PointStamped (GPS → Local XY)
  Raw GPS-based local position.
- PointStamped (Estimated XY)
  Dead-reckoning / fused local position estimate.
- Float32 (Heading)
  Absolute heading in degrees.
- Float32 (Ultrasonic range)
  Processed ultrasonic distance.
- Bool (IR left)
  Left infrared obstacle sensor state.
- Bool (IR right)
  Right infrared obstacle sensor state.
- String (Robot command)
  Current high-level movement command being executed.

Outputs (Publications):
----------------------
- PointStamped (Goal GPS)
  Target latitude and longitude for the navigation algorithm.

  Convention:
  - point.x → latitude
  - point.y → longitude
  - frame_id = "gps"


GUI FUNCTIONALITY:
------------------
The graphical interface allows the operator to:

- Observe, in real time:
  - GPS-based position
  - Local position estimate
  - Heading
  - Obstacle sensor states
  - Current high-level command
- Input a navigation target using the format:
  <latitude>, <longitude>
- Publish the target with a single action ("Send Target")

All displayed data are read-only except the goal input field.


ASSUMPTIONS:
------------
- The navigation stack expects the goal as a GPS coordinate.
- The goal is expressed in absolute geographic coordinates,
  not local XY.
- This node does not validate reachability or safety of the target.
- All subscribed topics are best-effort and non-critical.

DESIGN NOTES:
-------------
- This node intentionally mixes ROS2 and a desktop GUI:
  it is not meant for embedded deployment.
- ROS spinning and GUI event handling are separated using threads.
- The node must run on a machine with graphical capabilities (PC).

LIMITATIONS:
------------
- No map visualization is provided.
- No trajectory preview or path feedback is shown.
- Intended for debugging, supervision, and experimentation,
  not for autonomous mission execution.

FINAL NOTE:
-----------
The master monitor acts as the operator’s window into the robot’s internal
state and as a controlled injection point for high-level objectives.

It observes.
It commands.
It does not decide.

------------------------------------------------------------
END OF FILE
------------------------------------------------------------