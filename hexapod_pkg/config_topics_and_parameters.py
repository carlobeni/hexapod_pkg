#!/usr/bin/env python3

import os
# =================   ======   =================
# ================= ROS DOMAIN =================
# =================   ======   =================
"""
It is essential that the ROS domain is the same for both the PC and the Raspberry Pi in order for communication between them.
You can arbitrarily choose the domain as long as you make sure that both the PC and the Raspberry Pi have the same one.
For simulations, the domain can be neglected since all topics are published from the PC.
"""

ROS_DOMAIN_ID = 42

# This function allows you to verify the current domain of each node when it is launched.
def check_domain_id(logger=None):
    env = os.getenv("ROS_DOMAIN_ID")
    msg = (
        f"ROS_DOMAIN_ID={env} (expected {ROS_DOMAIN_ID})"
        if env else
        f"ROS_DOMAIN_ID NOT set (recommended {ROS_DOMAIN_ID})"
    )
    if logger:
        logger.warning(msg)
    else:
        print(msg)

# =================         ======          =================
# ================= SENSORS LECTURES TOPICS =================
# =================         ======          =================
"""
The following topics are those used by the sensor reading nodes (both for the simulation and the real robot), 
each with the specific handle for the type of data it publishes.

Note that the topics for the real robot are repeated and differentiated in "phone" sensor and "no phone sensor", 
this is because it is possible to optionally use real sensors as breakouts or sensors from a phone via an external 
application (such as SensorServer), where the latter was the final implementation for this project.
"""

# Obstacles detection topics (for de real robot)
TOPIC_PI_IR1 = "/pi/sensor/IR_measure1"                     # Topic type: Bool
TOPIC_PI_IR2 = "/pi/sensor/IR_measure2"                     # Topic type: Bool
TOPIC_PI_ULTRASONIC = "/pi/sensor/ultrasonic_read"          # Topic type: Range

# Localization and states topics (in case you use external sensor) (for de real robot)
TOPIC_PI_CAMERA = "/pi/camera/image_raw"                    # Topic type: Image
TOPIC_PI_IMU_GIR_ACC = "/pi/sensor/imu_data"                # Topic type: Imu
TOPIC_PI_MAG = "/pi/sensor/mag"                             # Topic type: MagneticField
TOPIC_PI_GPS = "/pi/sensor/gps_fix"                         # Topic type: Float32  

# Phone Sensor Localizaction and states topics (in case you use App SensorServer or simular) (for de real robot)
TOPIC_PI_PHONE_CAMERA = "/pi/phone/camera/image_raw"        # Topic type: Image
TOPIC_PI_PHONE_IMU_GIR_ACC = "/pi/phone/sensor/imu_data"    # Topic type: Imu
TOPIC_PI_PHONE_MAG = "/pi/phone/sensor/mag"                 # Topic type: MagneticField
TOPIC_PI_PHONE_GPS = "/pi/phone/sensor/gps_fix"             # Topic type: NavSatFix
TOPIC_PI_PHONE_HEADING = "/pi/phone/sensor/heading"         # Topic type: Float32  (heading obtained directly from the phone's sensors)

# TOPICS Only from simulation (From Gazebo)
TOPIC_GZ_CAMERA = '/camera/image_raw'                       # Topic type: Image
TOPIC_GZ_IMU_GIR_ACC = "/gz/sensor/imu_data"                # Topic type: Imu
TOPIC_GZ_MAG = "/gz/sensor/mag"                             # Topic type: MagneticField
TOPIC_GZ_GPS = "/gz/sensor/gps_fix"                         # Topic type: NavSatFix
TOPIC_GZ_IR1 = "/gz/sensor/IR_measure1"                     # Topic type: Bool
TOPIC_GZ_IR2 = "/gz/sensor/IR_measure2"                     # Topic type: Bool
TOPIC_GZ_ULTRASONIC = "/gz/sensor/ultrasonic_read"          # Topic type: Range

# =================                                  ======                                  =================
# ================= TOPICS FOR SENSOR DATA PROCESSING AND EXECUTION OF HIGH-LEVEL ALGORITHMS =================
# =================                                  ======                                  =================
# Image Recognition 
"""
Occupancy matrix used by the image procedure nodes that returns an NxN matrix over the image frame and places 0 in places 
where there are no detections and 1 in grids that coincide with detection positions.
"""
TOPIC_OBSTACLE_OCCUPATION_GRID = "/obstacle/occupation_grid"    # Topic type: Int8MultiArray

# Robot Localizaction and orientation 
"""
Location topics used by the robot's navigation node, where:
- TOPIC_XY_BY_GPS_CURRENT_POSITION: It is the raw conversion of the latitude and longitude read from the Navfix topic 
using the local ENU standard.
- TOPIC_XY_ODOM_CURRENT_POSITION: It is the actual estimate of the robot's location using simple odometry.
- TOPIC_HEADING_COMPASS: This is the heading angle with standard locality (0° east and 90° north). 
This angle must be between 0° and 360° to be compatible with the current algorithms in this project.
"""
TOPIC_XY_BY_GPS_CURRENT_POSITION = "/pc/internal/xy_gps_current_position"   # Topic type: PointStamped
TOPIC_XY_ODOM_CURRENT_POSITION = "/pc/internal/xy_odom_current_position"    # Topic type: PointStamped
TOPIC_HEADING_COMPASS    = "/pc/internal/heading_mag"                       # Topic type: Float32

# Goal Topic
"""
Topic that publishes the current target in latitude and longitude that the navigation algorithm will use
"""
TOPIC_GPS_GOAL = "/goal_gps"      # Topic type: PointStamped

# Ultrasonic manager
"""
Topic that returns the computed value of the ultrasonic in float type, where in case of no detections 
inside the valid range it returns -1.
"""
TOPIC_ULTRASONIC_RANGE = "/ultrasonic_range"      # Topic type: Float32

# =================      ======       =================
# ================= MOVEMENT COMMANDS =================
# =================      ======       =================
# High-level commands for the movement
"""
High-level commands that algorithms use. Commands can have 6 values:
- forward: forward movement at constant speed
- turn_left: left turn on the robot's axis at constant speed
- turn_right: right turn on the robot's axis at constant speed
- backward: backward movement at constant speed
- lateral_left: left sideways movement at constant speed (crab-like)
- lateral_right: right sideways movement at constant speed (crab-like)
- stop: stop de robot
These commands are used for high-level algorithm management, 
but they are not what is sent in raw serial data to the Arduino Mega.
"""
TOPIC_CMD_REAL_ROBOT = "/hl_real_cmd"       # Topic type: String  (cmd use for de real robot)
TOPIC_CMD_GZ_ROBOT = "/hl_gz_cmd"           # Topic type: String  (cmd use for de simulation robot)

# Movement commands prepared for serial transmission (From Raberry Pi to Arduino MEGA)
"""
This topic carries the script that will be sent to the Arduino Mega. 
The node that publishes this topic acts as a translator from the high-level topic 
to the commands that the Arduino Mega expects.
"""
TOPIC_CMD_SERIAL = "/cmd_serial"            # Topic type: String  (cmd use for de real robot)

# =================  ======  =================
# ================= CONSTANS =================
# =================  ======  =================
# Reference Lat an Lon
"""
Reference latitude and longitude for local ENU conversion. Change this to your coordinates. 
(Note that to use the simulation correctly in other coordinates, you must also change the world reference 
coordinates in the SDF files and the GPS reference in their .xacro description.)
"""
LAT_REFERENCE = -25.33057507763572
LON_REFERENCE = -57.51813474688503

# =================          ======          =================
# ================= HARDWARE CONFIGURATION ===================
# =================          ======          =================
# Phone IP (in case yo use the sensors of a phone)
"""
IP address listed in the SensorServer application or similar
"""
PHONEIP = "192.168.0.4"

# --------------- GPIO PIN ASSIGNMENTS (BCM) ---------------
# IMPORTANT:
# - Avoid GPIO2 (SDA) and GPIO3 (SCL) used by I2C (BerryGPS-IMU).
# - Avoid GPIO14/15 used by UART0 (GPS or other serial devices).

# IR SENSORS (digital type, e.g. IR obstacle sensors)
IR1_GPIO = 17    # BCM17
IR2_GPIO = 27    # BCM27

# HC-SR04 ULTRASONIC (5V sensor, level-shift recommended for ECHO to 3.3V)
ULTRASONIC_TRIG = 23   # BCM23
ULTRASONIC_ECHO = 24   # BCM24

# JS40F DIGITAL DISTANCE (3-wire, digital output)
# Brown = 5V, Blue = GND, Black = digital signal
JS40F_GPIO = 16        # BCM16 (only reserved here, node optional)

# BerryGPS-IMU v3
# - IMU via I2C on GPIO2 (SDA), GPIO3 (SCL), 3.3V, GND
# - GPS usually via serial (e.g. /dev/ttyAMA0 or /dev/ttyS0)
I2C_BUS_ID = 1
IMU_GYRO_ACC_ADDR = 0x6A   # gyroscope+accelerometer I2C address (default) :contentReference[oaicite:5]{index=5}
IMU_MAG_ADDR = 0x1C        # magnetometer I2C address (default) :contentReference[oaicite:6]{index=6}

GPS_SERIAL_PORT = "/dev/ttyAMA0"  # adjust if needed
GPS_BAUD = 9600                   # typical NMEA baud, adjust if configured differently

# Raspberry Pi Camera:
CAMERA_DEVICE_INDEX = 0  # for cv2.VideoCapture(0)

# Arduino Mega (serial link for hexapod commands)
MEGA_SERIAL_PORT = "/dev/ttyAMA0"   # adjust according to 'ls /dev/ttyACM*'
MEGA_BAUD = 9600

