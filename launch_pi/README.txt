============================================================
launch_pi / OVERVIEW
============================================================

This folder contains launch files intended to be executed
**directly on the Raspberry Pi** mounted on the real robot.

These launches are strictly **hardware-facing**.
This is the layer where software decisions
become **physical leg motion**.


------------------------------------------------------------
ROLE OF THE RASPBERRY PI
------------------------------------------------------------

The Raspberry Pi acts as the **execution gateway**
between ROS 2 and the robot body.

It does NOT plan.
It does NOT decide.
It does NOT interpret intent.

Its sole purpose is to:
- Read sensors
- Forward raw data upstream
- Receive finalized motion commands
- Execute them on real hardware


============================================================
launch_pi / read_sensors_rasberrypi.launch.py
============================================================

NODE NAME:
----------
read_sensors_rasberrypi.launch.py


------------------------------------------------------------
PURPOSE
------------------------------------------------------------

Launches all **hardware drivers** running on the Raspberry Pi.

This launch defines the **only path**
through which commands can reach the robot legs.


------------------------------------------------------------
CENTRAL COMPONENT: command_listener_node
------------------------------------------------------------

This is the **most critical node** on the Raspberry Pi.

`command_listener_node` is the **final authority**
over the robot’s physical movement.


------------------------------------------------------------
FUNCTION OF command_listener_node
------------------------------------------------------------

- Subscribes to motion commands coming from the PC in /cmd_serial topic
  (via DDS and ROS 2 topics)

- Sends these instructions **directly to the Arduino**
  via serial communication

- The Arduino firmware then:
    - Drives servos
    - Coordinates leg joints
    - Executes gait primitives


------------------------------------------------------------
IMPORTANT CONSEQUENCE
------------------------------------------------------------

After this node:
- There is no ROS
- There is no DDS
- There is no abstraction

Only:
→ Serial bytes  
→ Microcontroller logic  
→ Motors  
→ Legs  
→ Ground


------------------------------------------------------------
SENSORS READ
------------------------------------------------------------

- IR sensors
  (obstacle proximity)

- Ultrasonic sensor
  (distance measurement)

- GPS (via phone)
  (global position)

- Heading (via phone)
  (absolute orientation)

- IMU (via phone)
  (angular velocity + acceleration)


------------------------------------------------------------
ACTUATION PIPELINE (COMPLETE)
------------------------------------------------------------

PC (navigation / social / control logic)
        ↓
DDS
        ↓
Raspberry Pi
        ↓
command_listener_node
        ↓
Serial (UART)
        ↓
Arduino
        ↓
Servo drivers
        ↓
Hexapod legs


------------------------------------------------------------
NOTES ON DISABLED NODES
------------------------------------------------------------

- Magnetometer reader:
  Disabled in final system

- Camera capture:
  Not executed on Raspberry Pi
  to avoid bandwidth and CPU contention


------------------------------------------------------------
DESIGN PHILOSOPHY
------------------------------------------------------------

This architecture enforces a **hard boundary**
between cognition and execution.

The Raspberry Pi is not allowed to:
- modify commands
- reinterpret intent
- apply autonomy

It executes.
Nothing more.


------------------------------------------------------------
EXECUTION CONTEXT
------------------------------------------------------------

If this launch is not running:
- Commands never reach the Arduino
- Servos never move
- The robot is physically inert

This launch is the **nervous exit point**
of the entire system.
