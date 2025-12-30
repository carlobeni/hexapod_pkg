#!/usr/bin/env python3
# social_robot_real.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from hexapod_pkg import config_topics_and_parameters as cfg

def generate_launch_description():

    package_name = "hexapod_pkg"

    # =================================================
    # SOCIAL ROBOT LAUNCH (GESTURE RECOGNITION)
    # =================================================
    hand_gesture_node = Node(
        package=package_name,
        executable="hand_gesture_node.py",
        name="hand_gesture_node",
        output="screen",
        parameters=[{
            "topic_image": cfg.TOPIC_PI_PHONE_CAMERA,
            "topic_cmd_robot": cfg.TOPIC_CMD_REAL_ROBOT,
            "cmd_rock_to_send": "forward",   #In the real robot, with the rock gesture, the robot moves forward (this can be changed by modifying cmd_serial and the Arduino code)
            "cmd_al_pelo_to_send": "stop",
            "cmd_victory_to_send": "turn_left"
        }],
    )

    web_camera_node = Node(
        package=package_name,
        executable="web_camera_node.py",
        name="web_camera_node",
        output="screen",
        parameters=[{
            "topic_image": cfg.TOPIC_PI_PHONE_CAMERA,
            "camera_index": 0  
        }],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=[
                hand_gesture_node,
                web_camera_node
            ]
        ),
    ])
