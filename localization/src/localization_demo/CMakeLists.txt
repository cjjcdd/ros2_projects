cmake_minimum_required(VERSION 3.8)
project(localization_demo)

# ── Compiler flags ────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── Find packages ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# ── Shared dependency list (used by all executables) ─────────────────────────
set(DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ─────────────────────────────────────────────────────────────────────────────
# Node 1: odometry_reader
#   Subscribes to /odom, prints every field, publishes trajectory path.
# ─────────────────────────────────────────────────────────────────────────────
add_executable(odometry_reader src/odometry_reader.cpp)
ament_target_dependencies(odometry_reader ${DEPS})

# ─────────────────────────────────────────────────────────────────────────────
# Node 2: dead_reckoning
#   Implements dead reckoning from velocity commands.
#   Compares estimated vs true odometry in RViz.
# ─────────────────────────────────────────────────────────────────────────────
add_executable(dead_reckoning src/dead_reckoning.cpp)
ament_target_dependencies(dead_reckoning ${DEPS})

# ─────────────────────────────────────────────────────────────────────────────
# Node 3: localization_belief
#   Visualizes a Particle Filter / AMCL-style belief cloud.
#   Shows how particles converge as the robot moves.
# ─────────────────────────────────────────────────────────────────────────────
add_executable(localization_belief src/localization_belief.cpp)
ament_target_dependencies(localization_belief ${DEPS})

# ─────────────────────────────────────────────────────────────────────────────
# Node 4: trajectory_logger
#   Records robot path as nav_msgs/Path.
#   Computes total distance traveled, logs it.
# ─────────────────────────────────────────────────────────────────────────────
add_executable(trajectory_logger src/trajectory_logger.cpp)
ament_target_dependencies(trajectory_logger ${DEPS})

# ── Install all executables ───────────────────────────────────────────────────
install(TARGETS
  odometry_reader
  dead_reckoning
  localization_belief
  trajectory_logger
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch and config files ──────────────────────────────────────────
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/   DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
