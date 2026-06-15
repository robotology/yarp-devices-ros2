# SPDX-FileCopyrightText: 2025 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.8)
project(yarp_to_ros2_sound_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(YARP REQUIRED)

include_directories(
  ${YARP_INCLUDE_DIRS}
)

add_executable(yarp_to_ros2_sound_bridge yarp_to_ros2_sound_bridge.cpp)
ament_target_dependencies(yarp_to_ros2_sound_bridge rclcpp std_msgs)
target_link_libraries(yarp_to_ros2_sound_bridge ${YARP_LIBRARIES})

install(TARGETS
  yarp_to_ros2_sound_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
