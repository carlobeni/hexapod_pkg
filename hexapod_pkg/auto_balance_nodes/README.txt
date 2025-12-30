============================================================
AUTO_BALANCE_NODES — REAL ROBOT STABILIZATION
============================================================

FOLDER:
-------
auto_balance_nodes


============================================================
GENERAL CONTEXT
============================================================

This folder contains control nodes intended to run
EXCLUSIVELY on the real hexapod robot.

These nodes:
- Depend on real IMU data
- Produce direct low-level commands
- Bypass high-level motion abstraction
- Interface directly with the serial command layer

They MUST NOT be executed in Gazebo.


============================================================
1) balance_controller.py
============================================================

NODE NAME:
----------
balance_controller


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Real-time body attitude stabilization for the physical hexapod.

This node estimates roll and pitch angles from IMU data
and sends corrective rotation commands directly to the
serial motor controller.

It is designed as a lightweight, robust balance assist,
not as a full-body dynamic controller.


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- sensor_msgs/Imu
  Topic: cfg.TOPIC_PI_PHONE_IMU_GIR_ACC
  Source: real IMU (gyro + accelerometer)

OUTPUT:
-------
- std_msgs/String
  Topic: cfg.TOPIC_CMD_SERIAL
  Target: serial_cmd.balance_controller.py


------------------------------------------------------------
ARCHITECTURE OVERVIEW
------------------------------------------------------------

This node is split into two strictly separated loops:

1) Sensor fusion loop (IMU callback)
2) Control output loop (fixed-rate timer)

This separation prevents sensor jitter from directly
propagating to the actuators.


------------------------------------------------------------
IMU PROCESSING
------------------------------------------------------------

- Raw accelerometer and gyroscope data
- Angles computed in degrees
- Complementary filter for sensor fusion

    angle = α * (gyro integration) +
            (1 - α) * (accelerometer tilt)

Where:
- α = 0.98 (gyro-dominant, vibration resistant)

Yaw is computed but not used for stabilization.


------------------------------------------------------------
CONTROL STRATEGY
------------------------------------------------------------

- Open-loop attitude compensation
- No PID
- No integral action
- No model-based dynamics

The controller simply maps body tilt
→ corrective rotation commands.


------------------------------------------------------------
OUTPUT GENERATION
------------------------------------------------------------

Steps applied before sending commands:

1) Axis mapping
   - Roll and pitch mapped to RY / RZ corrections

2) Saturation
   - Absolute angle limited to ±12 degrees

3) Output smoothing
   - Exponential Moving Average (EMA)
   - Prevents servo jerks and oscillations

4) Deadband
   - Small angles ignored
   - Avoids buzzing at rest

5) Rate limiting
   - Commands sent at 20 Hz (fixed)
   - Independent from IMU frequency


------------------------------------------------------------
SERIAL COMMAND FORMAT
------------------------------------------------------------

The node emits low-level serial commands:

    "1:ROT <rz> <ry> 0 0"

This format is consumed directly by:
    serial_cmd.balance_controller.py

No intermediate interpretation is performed.


------------------------------------------------------------
QoS AND REAL-TIME CONSIDERATIONS
------------------------------------------------------------

- IMU subscription: BEST_EFFORT
  (prioritizes freshness over reliability)

- Serial command publisher: RELIABLE
  (commands must not be lost)

- dt sanity check:
  - Rejects invalid or stalled IMU updates


------------------------------------------------------------
DESIGN INTENT
------------------------------------------------------------

- Assistive balance, not full locomotion control
- Stable behavior on uneven terrain
- Minimal computational load
- Predictable, smooth actuation

This controller assumes that:
- Gait generation happens elsewhere
- Balance corrections are additive
- Safety limits are enforced downstream


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This node sits BELOW motion planning
and ABOVE raw motor actuation.

It directly influences the robot posture
by injecting corrective rotations at the serial level.


------------------------------------------------------------
CRITICAL WARNING
------------------------------------------------------------

THIS NODE IS REAL-ROBOT ONLY.

- Uses real IMU noise characteristics
- Sends direct serial motor commands
- No simulation safeguards

Running it in Gazebo is meaningless
and potentially misleading.

------------------------------------------------------------
END OF FILE
------------------------------------------------------------
