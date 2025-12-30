FOLDER:
-------
localization_and_orientation_nodes


OVERVIEW:
---------
This folder contains the core nodes responsible for estimating the robot’s
planar position (X,Y) and absolute orientation (heading) in a local metric frame.

The architecture is deliberately simple, explicit, and deterministic.
It prioritizes interpretability and controllability over probabilistic optimality.


ARCHITECTURE SUMMARY:
---------------------
The localization pipeline is composed of three main stages:

1) GPS → Local metric conversion
2) Heading estimation (IMU + magnetometer fusion)
3) Position estimation via dead-reckoning corrected by GPS

No SLAM, EKF, or particle filtering is performed in this folder.


DATA FLOW:
----------
NavSatFix (GPS)
      ↓
compute_gps_to_local_xy
      ↓
XY_GPS (PointStamped)
      ↓
compute_estimate_xy  ←── Heading (Float32)
      ↑
compute_heading


GLOBAL ASSUMPTIONS:
-------------------
- The local reference (LAT_REFERENCE, LON_REFERENCE) is consistent across:
  - GPS configuration
  - Gazebo world origin
  - Xacro sensor descriptions
- The environment is planar.
- Heading is defined as:
  - 0° = East
  - 90° = North
  - Range: [0°, 360°)
- All coordinates are expressed in meters in the "map" frame.


------------------------------------------------------------
NODE: compute_gps_to_local_xy
------------------------------------------------------------

Purpose:
--------
Transforms geographic coordinates (latitude, longitude) from a GPS source
into local Cartesian coordinates (X,Y) in meters, using a local Mercator
(ENU-like) approximation.

This node performs a pure geometric transformation and does NOT estimate state.

Inputs:
-------
- NavSatFix
  Raw GPS latitude, longitude, and altitude.

Outputs:
--------
- PointStamped
  Local X,Y position in meters, referenced to the "map" frame.

Assumptions:
------------
- The reference latitude and longitude define the origin of the local map.
- Earth curvature effects are negligible at the operating scale.
- GPS messages with latitude = 0 and longitude = 0 are treated as invalid.

Notes:
------
- No filtering or smoothing is applied.
- All higher-level correction or fusion is handled downstream.


------------------------------------------------------------
NODE: compute_heading
------------------------------------------------------------

Purpose:
--------
Estimates the robot’s absolute planar heading by fusing:
- Yaw extracted from IMU orientation (quaternion)
- Heading derived from magnetometer measurements

A complementary filter is used for fast and stable heading estimation.

Inputs:
-------
- Imu
  Orientation quaternion of the robot.
- MagneticField
  Raw magnetic field vector in the body frame.

Outputs:
--------
- Float32
  Absolute heading angle in degrees [0, 360).

Assumptions:
------------
- IMU yaw is reliable in the short term.
- Magnetometer is calibrated externally.
- Magnetic declination is constant and known.
- Roll and pitch effects are ignored (planar assumption).

Parameters:
-----------
- declination_deg : magnetic declination correction
- alpha           : IMU vs magnetometer fusion weight
- heading_gain    : scale correction factor

Notes:
------
- This is not a Kalman filter.
- Designed for low latency and direct usability by navigation nodes.
- Output is normalized for direct consumption by other modules.


------------------------------------------------------------
NODE: compute_estimate_xy
------------------------------------------------------------

Purpose:
--------
Estimates the robot’s planar position (X,Y) by combining:
- Dead-reckoning based on high-level motion commands
- Direct correction using GPS-derived local coordinates

This node represents a simple state estimator, not a probabilistic localization
system.

Inputs:
-------
- PointStamped (XY_GPS)
  Local GPS-based position.
- Float32 (Heading)
  Absolute heading in degrees.
- String (High-level command)
  Discrete movement command (forward, backward, lateral, stop).

Outputs:
--------
- PointStamped
  Estimated robot position (X,Y) in meters.

Assumptions:
------------
- Movement commands correspond to constant velocities.
- Heading input is reliable.
- GPS noise is present but not strongly biased.

Parameters:
-----------
- linear_v                : forward/backward speed
- lateral_v               : lateral (crab) speed
- dead_reckoning_weight   : blending factor between odometry and GPS
                            (1.0 = pure odometry, 0.0 = pure GPS)

Notes:
------
- The estimator is fully deterministic.
- Intended for simplicity, debugging, and educational clarity.


------------------------------------------------------------
FINAL NOTES:
------------
This folder provides a minimal yet complete localization backbone.
Its simplicity is intentional: every assumption is explicit, and every
numerical effect is traceable.

More advanced localization techniques can be layered on top of this system,
not hidden inside it.

------------------------------------------------------------
END OF FILE
------------------------------------------------------------