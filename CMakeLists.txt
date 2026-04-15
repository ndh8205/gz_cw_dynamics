cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(gz_cw_dynamics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# ROS 2 (used by Thruster plugin for Float32 subscription)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Gazebo Harmonic stack
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED)

# ---- CW dynamics plugin ----
add_library(gz_cw_dynamics-system SHARED
  src/CWDynamics.cc
)

target_include_directories(gz_cw_dynamics-system PRIVATE include)

target_link_libraries(gz_cw_dynamics-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)

# ---- Thruster plugin (ROS 2 Float32 -> world force) ----
add_library(gz_thruster_ros2-system SHARED
  src/Thruster.cc
)

target_include_directories(gz_thruster_ros2-system PRIVATE include)

target_link_libraries(gz_thruster_ros2-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
)

ament_target_dependencies(gz_thruster_ros2-system PUBLIC
  rclcpp
  std_msgs
)

# ---- Reaction wheel plugin (ROS 2 Float32 -> joint torque) ----
add_library(gz_reaction_wheel_ros2-system SHARED src/ReactionWheel.cc)
target_include_directories(gz_reaction_wheel_ros2-system PRIVATE include)
target_link_libraries(gz_reaction_wheel_ros2-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
)
ament_target_dependencies(gz_reaction_wheel_ros2-system PUBLIC
  rclcpp
  std_msgs
)

# ---- Orbit-aware IMU plugin (LVLH + CW pseudo-accel correction) ----
add_library(gz_orbit_imu-system SHARED
  src/OrbitImu.cc
)

target_include_directories(gz_orbit_imu-system PRIVATE include)

target_link_libraries(gz_orbit_imu-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)

ament_target_dependencies(gz_orbit_imu-system PUBLIC
  rclcpp
  sensor_msgs
)

# ---- Chief orbit propagator (Layer A: ECI + LVLH sun vector) ----
add_library(gz_chief_propagator-system SHARED
  src/ChiefPropagator.cc
)

target_include_directories(gz_chief_propagator-system PRIVATE include)

target_link_libraries(gz_chief_propagator-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
)

ament_target_dependencies(gz_chief_propagator-system PUBLIC
  rclcpp
  nav_msgs
  geometry_msgs
)

# ---- Star tracker plugin (attitude quaternion with noise) ----
add_library(gz_star_tracker-system SHARED src/StarTracker.cc)
target_include_directories(gz_star_tracker-system PRIVATE include)
target_link_libraries(gz_star_tracker-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
)
ament_target_dependencies(gz_star_tracker-system PUBLIC
  rclcpp
  geometry_msgs
  nav_msgs
)

# ---- GPS plugin (ECI-frame position + velocity with noise) ----
add_library(gz_gps-system SHARED src/Gps.cc)
target_include_directories(gz_gps-system PRIVATE include)
target_link_libraries(gz_gps-system PRIVATE
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-common5::gz-common5
)
ament_target_dependencies(gz_gps-system PUBLIC
  rclcpp
  nav_msgs
)

# ---- Install ----
install(TARGETS
  gz_cw_dynamics-system
  gz_thruster_ros2-system
  gz_orbit_imu-system
  gz_chief_propagator-system
  gz_star_tracker-system
  gz_gps-system
  gz_reaction_wheel_ros2-system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/verify_gco.py
  scripts/run_gco_verify.sh
  scripts/test_thrusters.py
  scripts/run_thruster_test.sh
  scripts/test_imu.py
  scripts/run_imu_test.sh
  scripts/test_chief.py
  scripts/run_chief_test.sh
  scripts/chief_propagator_node.py
  scripts/test_all_sensors.py
  scripts/run_all_sensors_test.sh
  scripts/test_reaction_wheel.py
  scripts/run_rw_test.sh
  scripts/run_full_system_test.sh
  scripts/student/thruster_commander.py
  scripts/student/sensor_monitor.py
  scripts/student/camera_saver.py
  scripts/student/rw_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
  OPTIONAL
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

ament_package()
