========================================================
DDS NODES OVERVIEW (COMMANDS & SENSORS)
========================================================

--------------------------------------------------------
NODE: dds_cmd_talker.py
--------------------------------------------------------
ROLE:
This is the MOST CRITICAL DDS NODE in the system.
It acts as a TRANSLATOR between high-level robot commands
(e.g. "forward", "turn_left") and the LOW-LEVEL SERIAL
COMMANDS actually sent to the Arduino Mega.

Conceptually:
  HIGH-LEVEL LOGIC  →  dds_cmd_talker  →  SERIAL PROTOCOL  →  ARDUINO

Any change in the Arduino firmware command format MUST be
reflected HERE. This node is the software contract boundary
between ROS 2 and the real robot controller.

SUBSCRIBES:
- cmd_robot_topic (String)
  Receives abstract commands such as:
  "forward", "backward", "turn_left", "mode_1", etc.

PUBLISHES:
- cmd_serial_topic (String)
  Sends formatted serial commands like:
  "WALK 127 0 -7", "MODE 1", etc.

KEY PARAMETERS:
- linear_speed      : Base linear velocity (default 127)
- angular_speed     : Base angular velocity (default 127)
- walk_yaw_trim     : Lateral/yaw compensation
- qos_depth         : DDS queue depth

INTERNAL LOGIC:
- Each incoming high-level command is mapped inside cmd_cb()
  to a specific SERIAL STRING understood by the Arduino.
- A command ID is prepended:  <id>:<command>
  This helps debugging, ordering, and tracing.

EXAMPLE MAPPING:
  "forward"     → "WALK +V 0 yaw_trim"
  "turn_left"   → "WALK 0 0 +V_ang"
  "stop"        → "WALK 0 0 0"
  "mode_2"      → "MODE 2"

IMPORTANT:
- This node DOES NOT decide behavior.
- It ONLY translates intent into executable motion commands.
- If the robot behaves incorrectly, this node is the FIRST
  place to inspect after the Arduino code.

INITIALIZATION:
On startup it sends:
- SET90
- MODE 1
to ensure the robot starts in a known physical state.

--------------------------------------------------------
NODE: dds_sensors_fast_listener.py
--------------------------------------------------------
ROLE:
Lightweight DDS listener for FAST, HIGH-RATE sensors.
It exists to keep DDS endpoints alive and synchronized,
not to process data.

TYPICAL SENSORS:
- IMU
- Magnetometer
- Ultrasonic
- IR sensors
- Camera

QOS PROFILE:
- BEST_EFFORT
- Low latency
- Packet loss tolerated

BEHAVIOR:
- Subscribes to topics only if parameters are provided.
- Callback intentionally does NOTHING (pass).
- Designed for real-time streams where missing samples
  is acceptable.

PURPOSE:
- DDS discovery stabilization
- Future expansion hooks
- Bandwidth-aware sensor separation

--------------------------------------------------------
NODE: dds_sensors_reliable_listener.py
--------------------------------------------------------
ROLE:
Listener for LOW-RATE, HIGH-VALUE sensor data.

TYPICAL SENSOR:
- GPS (NavSatFix)

QOS PROFILE:
- RELIABLE
- Guaranteed delivery
- Ordered messages

BEHAVIOR:
- Minimal callback (pass)
- Ensures no GPS fix is lost due to DDS volatility

DESIGN RATIONALE:
Fast sensors favor latency.
Global state sensors favor correctness.
This node enforces that distinction at the DDS level.

--------------------------------------------------------
ARCHITECTURAL SUMMARY
--------------------------------------------------------
- dds_cmd_talker.py is the COMMAND TRANSLATION CORE.
- Sensor listeners are DDS STABILIZERS, not processors.
- Behavior logic belongs elsewhere.
- Hardware coupling is intentionally isolated to ONE node.

If the Arduino changes → modify dds_cmd_talker.
If DDS breaks → listeners reveal it.
If behavior fails → logic is upstream.

This separation is deliberate, strict, and non-negotiable.
========================================================
