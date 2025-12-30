#!/usr/bin/env python3
# swarm_follower_sim.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from hexapod_pkg import config_topics_and_parameters as cfg


def generate_launch_description():

    package_name = "hexapod_pkg"

    # =================================================
    # SWARN FOLLOWER
    # =================================================
    ball_viewer_yolo_swarm = Node(
        package=package_name,
        executable="ball_viewer_yolo_swarm.py",
        name="ball_viewer_yolo_swarm",
        output="screen",
        parameters=[{
            "topic_image": cfg.TOPIC_GZ_CAMERA,
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
            "topic_cmd_robot": cfg.TOPIC_CMD_GZ_ROBOT
        }],
    )

    # =====================================================
    # LAUNCH
    # =====================================================
    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=[
                ball_viewer_yolo_swarm,
                follower_navigation_node
            ]
        ),
    ])
