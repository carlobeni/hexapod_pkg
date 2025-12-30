cmake_minimum_required(VERSION 3.5)
project(hexapod_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# =====================================================
# Dependencies (Python ROS 2)
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
ament_python_install_package(${PROJECT_NAME})


# =====================================================
# Testing 
# =====================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# =====================================================
# Install floders
# =====================================================
install(
  DIRECTORY 
    config 
    description_hexapod
    launch
    worlds
    meshes
    models
    launch_sim
    launch_pi
    hexapod_pkg
  DESTINATION share/${PROJECT_NAME}
)


# =====================================================
# Install Python nodes (EXACT FILENAMES)
# =====================================================
install(PROGRAMS
  # Comunication DDS Nodes
  hexapod_pkg/dds_nodes/dds_cmd_talker.py
  hexapod_pkg/dds_nodes/dds_sensors_fast_listener.py
  hexapod_pkg/dds_nodes/dds_sensors_reliable_listener.py

  # Gazebo Nodes
  hexapod_pkg/gazebo_nodes/gz_ir_emulator.py
  hexapod_pkg/gazebo_nodes/gz_hexapod_low_level_control.py

  # Localization and Orientation Nodes
  hexapod_pkg/localization_and_orientation_nodes/compute_gps_to_local_xy.py
  hexapod_pkg/localization_and_orientation_nodes/compute_heading.py 
  hexapod_pkg/localization_and_orientation_nodes/compute_estimate_xy.py

  # Ultrasonic Sensor Nodes
  hexapod_pkg/sensors_interfaces_nodes/compute_ultrasonic_ranges.py

  # Master Monitor Node
  hexapod_pkg/master_nodes/master_monitor.py

  # Camera Nodes
  hexapod_pkg/camera_nodes/web_camera_node.py 

  # Image Recognition Nodes
  hexapod_pkg/image_recognition_nodes/cube_viewer_classic_vision.py
  hexapod_pkg/image_recognition_nodes/cube_viewer_yolo.py
  hexapod_pkg/image_recognition_nodes/ball_viewer_yolo_follower.py
  hexapod_pkg/image_recognition_nodes/ball_viewer_yolo_swarm.py

  # TeleOP Nodes
  hexapod_pkg/teleop_nodes/teleop_hexapod_sim.py
  hexapod_pkg/teleop_nodes/teleop_hexapod_real.py

  # Navigation Nodes
  hexapod_pkg/navigation_nodes/target_navigation_with_obstacle_avoidance.py
  hexapod_pkg/navigation_nodes/follower_navigation_node.py 
  hexapod_pkg/navigation_nodes/swarm_navigation_with_obstacle_avoidance.py 

  # Social Nodes
  hexapod_pkg/social_robot_nodes/hand_gesture_node.py

  # CONFIG TOPICS AND PARAMETERS
  hexapod_pkg/config_topics_and_parameters.py

  # Sensors
  hexapod_pkg/read_sensors_nodes/command_listener_node.py
  hexapod_pkg/read_sensors_nodes/phone_read_gps.py
  hexapod_pkg/read_sensors_nodes/phone_read_imu.py
  hexapod_pkg/read_sensors_nodes/phone_read_magnetometer.py
  hexapod_pkg/read_sensors_nodes/phone_read_heading.py
  hexapod_pkg/read_sensors_nodes/read_gps_node.py
  hexapod_pkg/read_sensors_nodes/read_BerryImu_node.py
  hexapod_pkg/read_sensors_nodes/read_ir_node.py
  hexapod_pkg/read_sensors_nodes/read_ultrasonic_node.py

  # Balance Controller
  hexapod_pkg/auto_balance_nodes/balance_controller.py
  
  

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
