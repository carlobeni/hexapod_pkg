#!/usr/bin/env python3
# read_sensors_rasberrypi.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="hexapod_pkg",
            executable="read_ir_node.py",
            output="screen",
        ),

        Node(
            package="hexapod_pkg",
            executable="read_ultrasonic_node.py",
            output="screen",
        ),

        Node(
            package="hexapod_pkg",
            executable="phone_read_gps.py",
            output="screen",
        ),

        Node(
            package="hexapod_pkg",
            executable="phone_read_heading.py",
            output="screen",
        ),

        Node(
            package="hexapod_pkg",
            executable="phone_read_imu.py",
            output="screen",
        ),

        # Node(
        #     package="hexapod_pkg",
        #     executable="phone_read_magnetometer.py",
        #     output="screen",
        # ),

        # Node(
        #     package="hexapod_pkg",
        #     executable="web_camera_node.py",
        #     output="screen",
        # ),

        Node(
            package="hexapod_pkg",
            executable="command_listener_node.py",
            output="screen",
        ),

    ])
