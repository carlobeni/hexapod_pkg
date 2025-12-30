#!/usr/bin/env python3
# swarm_follower_real.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from hexapod_pkg import config_topics_and_parameters as cfg

def generate_launch_description():

    package_name = "hexapod_pkg"

    # =================================================
    # SWARN FOLLOWER
    # =================================================
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

    ball_viewer_yolo_swarm = Node(
        package=package_name,
        executable="ball_viewer_yolo_swarm.py",
        name="ball_viewer_yolo_swarm",
        output="screen",
        parameters=[{
            "topic_image": cfg.TOPIC_PI_PHONE_CAMERA ,
            "topic_obstacle_occupation_grid": cfg.TOPIC_OBSTACLE_OCCUPATION_GRID  # publisher
        }],
    )

    follower_navigation_node = Node(
        package=package_name,
        executable="swarm_navigation_with_obstacle_avoidance.py",
        name="swarm_navigation_with_obstacle_avoidance",
        output="screen",
        parameters=[{
            "topic_obstacle_occupation_grid": cfg.TOPIC_OBSTACLE_OCCUPATION_GRID,
            "topic_cmd_robot": cfg.TOPIC_CMD_REAL_ROBOT
        }],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=[
                web_camera_node,
                ball_viewer_yolo_swarm,
                follower_navigation_node
            ]
        ),
    ])
