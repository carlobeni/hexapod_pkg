============================================================
READ_SENSORS_NODES — SENSOR ACQUISITION (RASPBERRY PI SIDE)
============================================================

FOLDER:
-------
read_sensors_nodes


============================================================
GENERAL CONTEXT
============================================================

This folder contains all sensor listener nodes executed on the
Raspberry Pi.

Their main role is to:
- Interface with physical sensors or a smartphone
- Convert raw data into ROS 2 standard messages
- Publish sensor data to the central PC over the ROS network

Two sensor sources coexist:
1) Smartphone-based sensors (SensorServer over WebSocket)
2) Physical breakout-board sensors (BerryIMU, IR, Ultrasonic)

------------------------------------------------------------
IMPORTANT NOTE
------------------------------------------------------------

In the FINAL PROJECT IMPLEMENTATION, only the following nodes
were actually used:

USED NODES:
-----------
- read_ir_node.py
- read_ultrasonic_node.py
- phone_read_gps.py
- phone_read_heading.py
- phone_read_imu.py
- command_listener_node.py

All other nodes remain in the repository strictly as:
- references
- alternatives
- future extension examples

------------------------------------------------------------
EXECUTION MODEL
------------------------------------------------------------

- All nodes run on the Raspberry Pi
- Data is published to the central PC
- No heavy processing or fusion is done here
- Pi acts as a distributed I/O and communication endpoint


============================================================
1) command_listener_node.py
============================================================

NODE NAME:
----------
command_listener


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Serial command bridge between ROS 2 and the Arduino Mega.

This node listens for high-level commands published on a ROS topic
and forwards them to the Arduino via a serial interface.


------------------------------------------------------------
ROLE IN THE SYSTEM
------------------------------------------------------------

This is a CRITICAL COMMUNICATION NODE.

It is the only node responsible for:
- Translating ROS commands into hardware-level commands
- Ensuring that the Arduino receives clean, ordered instructions


------------------------------------------------------------
INPUT / OUTPUT
------------------------------------------------------------

INPUT:
------
- std_msgs/String  (cfg.TOPIC_CMD_SERIAL)

Expected format:
    ID:COMMAND

Example:
    12:FORWARD


OUTPUT:
-------
- Serial write to Arduino Mega
  (COMMAND + newline)


------------------------------------------------------------
COMMAND FORMAT LOGIC
------------------------------------------------------------

- Messages must contain a colon (:)
- The message is split into:
    - cmd_id
    - command

Although the header comment mentions "send only when ID changes",
the current implementation forwards EVERY valid command received.
(The ID is parsed but not used for filtering.)

This leaves room for future de-duplication or sequencing logic.


------------------------------------------------------------
SERIAL COMMUNICATION
------------------------------------------------------------

- Opens serial port defined in config_topics_and_parameters
- Configurable baud rate
- Graceful handling of serial errors
- If serial is unavailable, commands are silently ignored


------------------------------------------------------------
QoS STRATEGY
------------------------------------------------------------

- Reliable delivery
- Keep last N messages
- Volatile durability

This guarantees command delivery while avoiding stale data.


------------------------------------------------------------
DESIGN INTENT
------------------------------------------------------------

- Deterministic command forwarding
- Minimal latency
- No command interpretation or safety logic
- Arduino remains the final authority for actuation


------------------------------------------------------------
SUMMARY
------------------------------------------------------------

This node is the ROS ↔ Arduino bridge.

It deliberately remains simple, transparent, and stateless,
serving only as a transport layer between software and hardware.


============================================================
2) SENSOR READER NODES — GENERAL OVERVIEW
============================================================

All remaining nodes in this folder follow the same philosophy:

- One sensor (or sensor group)
- One ROS topic
- Minimal processing
- Direct publication


============================================================
SMARTPHONE-BASED SENSOR NODES
============================================================

These nodes use the Android SensorServer application and
communicate via WebSocket.


------------------------------------------------------------
phone_read_gps.py
------------------------------------------------------------

- Source: Smartphone GNSS
- Output: sensor_msgs/NavSatFix
- Low rate (~1 Hz)
- Accuracy converted into covariance
- Used as global positioning reference


------------------------------------------------------------
phone_read_heading.py
------------------------------------------------------------

- Source: Game rotation vector
- Computes heading from phone orientation
- Relative heading (0–360°) with automatic tare
- Used as absolute orientation reference


------------------------------------------------------------
phone_read_imu.py
------------------------------------------------------------

- Source: Accelerometer + Gyroscope
- Publishes raw IMU data
- No fusion, no filtering
- Used for motion estimation and logging


------------------------------------------------------------
phone_read_magnetometer.py (NOT USED)
------------------------------------------------------------

- Raw magnetic field from phone
- Kept only as reference
- Replaced by heading node in final system


============================================================
PHYSICAL SENSOR NODES (BREAKOUT BOARDS)
============================================================

------------------------------------------------------------
read_ir_node.py
------------------------------------------------------------

- Hardware: Digital IR obstacle sensors
- GPIO-based (active LOW)
- Publishes std_msgs/Bool
- Simple, robust obstacle detection
- USED in final system


------------------------------------------------------------
read_ultrasonic_node.py
------------------------------------------------------------

- Hardware: Ultrasonic distance sensor
- Publishes sensor_msgs/Range
- Fixed update rate (10 Hz)
- USED in final system


------------------------------------------------------------
read_gps_node.py (NOT USED)
------------------------------------------------------------

- Hardware: BerryGPS serial module
- Parses NMEA sentences
- Alternative to phone GPS
- Replaced by smartphone GPS in final setup


------------------------------------------------------------
read_BerryImu_node.py (NOT USED)
------------------------------------------------------------

- Hardware: BerryIMU
- Publishes IMU + Magnetometer
- Includes:
    - Calibration
    - LPF
    - Median filtering
- More complex but hardware-dependent
- Replaced by phone IMU for simplicity


============================================================
GLOBAL DESIGN PHILOSOPHY
============================================================

- Raspberry Pi as a sensor gateway
- ROS messages as the only interface
- No fusion, no estimation at this level
- Central PC handles higher-level logic


------------------------------------------------------------
SUMMARY
------------------------------------------------------------

This folder encapsulates all sensor acquisition logic for the robot.

Only a minimal subset was used in the final project, prioritizing:
- robustness
- simplicity
- ease of replacement

The remaining nodes serve as technical references for future
hardware-based expansions.

------------------------------------------------------------
END OF FILE
------------------------------------------------------------
