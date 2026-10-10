cmake_minimum_required(VERSION 3.8)
project(automatic_mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# install folders (launch files, configs, robot description, rviz, worlds)
install(
  DIRECTORY
  config
  launch
  description
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# install Python scripts so "ros2 run" can find them
install(
  PROGRAMS
  scripts/navigate.py
  scripts/cmd_vel_relay.py
  scripts/go_to_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
