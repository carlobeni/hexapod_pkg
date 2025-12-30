#!/usr/bin/env python3
# dds_comunication.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

from hexapod_pkg import config_topics_and_parameters as cfg


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =====================================================
    # PRELIMINARY CLEANING
    # =====================================================
    cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "pkill -f dds_sensors_fast_listener.py || true; "
            "pkill -f dds_sensors_reliable_listener.py || true; "
            "pkill -f dds_cmd_talker.py || true"
        ],
        output="screen"
    )

    return LaunchDescription([
        cleanup,

        # -------- ESPERA CORTA PARA DDS --------
        TimerAction(
            period=1.0,
            actions=[

                # =================================================
                # DDS FAST LISTENER 
                # =================================================
                Node(
                    package=package_name,
                    executable="dds_sensors_fast_listener.py",
                    name="dds_sensors_fast_listener",
                    output="screen",
                    parameters=[{
                        "imu_topic": cfg.TOPIC_PI_PHONE_IMU_GIR_ACC,
                        "mag_topic": cfg.TOPIC_PI_PHONE_MAG,
                        "ultrasonic_topic": cfg.TOPIC_PI_ULTRASONIC,
                        "ir1_topic": cfg.TOPIC_PI_IR1,
                        "ir2_topic": cfg.TOPIC_PI_IR2,
                        "camera_topic": cfg.TOPIC_PI_PHONE_CAMERA,
                    }],
                ),

                # =================================================
                # DDS RELIABLE LISTENER 
                # =================================================
                Node(
                    package=package_name,
                    executable="dds_sensors_reliable_listener.py",
                    name="dds_sensors_reliable_listener",
                    output="screen",
                    parameters=[{
                        "gps_topic": cfg.TOPIC_PI_PHONE_GPS,
                    }],
                ),

                # =================================================
                # DDS COMMAND TALKER (cmd_robot → cmd_serial)
                # =================================================
                Node(
                    package=package_name,
                    executable="dds_cmd_talker.py",
                    name="dds_cmd_talker",
                    output="screen",
                    parameters=[{
                        "cmd_robot_topic": cfg.TOPIC_CMD_REAL_ROBOT,
                        "cmd_serial_topic": cfg.TOPIC_CMD_SERIAL,
                        "linear_speed": 127,
                        "angular_speed": 127,
                        "walk_yaw_trim": -7,
                    }],
                ),
            ]
        ),
    ])
