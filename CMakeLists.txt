cmake_minimum_required(VERSION 3.8)
project(simple_nav)

# SPDX-FileCopyrightText: 2025 Keitaro Nakamura
# SPDX-License-Identifier: MIT License


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(value_iteration2_astar_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Pure Pursuit Controller executable
add_executable(pure_pursuit_controller
  src/PurePursuitController.cpp
)

ament_target_dependencies(pure_pursuit_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  value_iteration2_astar_msgs
)

# A* Path Planner executable
add_executable(a_star_planner
  src/Planner.cpp
)

ament_target_dependencies(a_star_planner
  rclcpp
  geometry_msgs
  nav_msgs
  value_iteration2_astar_msgs
)

# Install executables
install(TARGETS
  pure_pursuit_controller
  a_star_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
