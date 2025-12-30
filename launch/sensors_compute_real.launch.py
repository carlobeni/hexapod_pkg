#!/usr/bin/env python3
# sensors_compute_real.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from hexapod_pkg import config_topics_and_parameters as cfg

def generate_launch_description():

    package_name = "hexapod_pkg"

    # =================================================
    # COMPUTE
    # =================================================
    """  
    For the final implementation, the heading read directly from the cell phone was used, 
    so it was not necessary to compute the heading based on the IMU and the MAG
    """
    # compute_heading = Node(
    #     package=package_name,
    #     executable="compute_heading.py",
    #     name="compute_heading",
    #     output="screen",
    #     parameters=[{
    #         "topic_imu": cfg.TOPIC_GZ_IMU_GIR_ACC,
    #         "topic_mag": cfg.TOPIC_GZ_MAG,
    #         "topic_estimate_heading": cfg.TOPIC_ESTIMATE_HEADING # publisher
    #     }],
    # )

    compute_gps_to_local_xy = Node(
        package=package_name,
        executable="compute_gps_to_local_xy.py",
        name="compute_gps_to_local_xy",
        output="screen",
        parameters=[{
            "topic_gps_lat_lon_topic": cfg.TOPIC_PI_PHONE_GPS,
            "topic_gps_to_xy": cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,          # publisher
            "LAT0": cfg.LAT_REFERENCE,  # Origen latitud
            "LON0": cfg.LON_REFERENCE   # Origen longitud
        }],
    )

    compute_estimate_xy = Node(
        package=package_name,
        executable="compute_estimate_xy.py",
        name="compute_estimate_xy",
        output="screen",
        parameters=[{
            "topic_gps_to_xy": cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,
            "topic_cmd_robot": cfg.TOPIC_CMD_REAL_ROBOT,
            "topic_estimate_heading": cfg.TOPIC_PI_PHONE_HEADING, # No se computa, viene directo de la PC
            "topic_estimate_xy": cfg.TOPIC_XY_ODOM_CURRENT_POSITION,
            "linear_v": 0.09,
            "lateral_v": 0.1,
            "dead_reckoning_weight": 1,  # Between 0 and 1, if is 0 only GPS data and if is 1 only odom data   
            "LAT0": cfg.LAT_REFERENCE,
            "LON0": cfg.LON_REFERENCE,
            "LAT_ROBOT_INITIAL":cfg.LAT_REFERENCE, # To establish the initial coordenates in the origin
            "LON_ROBOT_INITIAL":cfg.LON_REFERENCE   
        }],
    )

    compute_ultrasonic_ranges = Node(
        package=package_name,
        executable="compute_ultrasonic_ranges.py",
        name="compute_ultrasonic_ranges",
        output="screen",
        parameters=[{
            "topic_ultrasonic_raw": cfg.TOPIC_PI_ULTRASONIC,
            "topic_ultrasonic_range": cfg.TOPIC_ULTRASONIC_RANGE      # publisher
        }],
    )

    master_monitor = Node(
        package=package_name,
        executable="master_monitor.py",
        name="master_monitor",
        output="screen",
        parameters=[{
            "topic_ir_left":cfg.TOPIC_PI_IR1,
            "topic_ir_right":cfg.TOPIC_PI_IR2,
            "topic_gps_to_xy":cfg.TOPIC_XY_BY_GPS_CURRENT_POSITION,
            "topic_estimate_heading":cfg.TOPIC_PI_PHONE_HEADING,
            "topic_ultrasonic_range":cfg.TOPIC_ULTRASONIC_RANGE,
            "topic_estimate_xy":cfg.TOPIC_XY_ODOM_CURRENT_POSITION,
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
                #compute_heading,
                compute_gps_to_local_xy,
                compute_estimate_xy,
                compute_ultrasonic_ranges,
                master_monitor
            ]
        ),
    ])
