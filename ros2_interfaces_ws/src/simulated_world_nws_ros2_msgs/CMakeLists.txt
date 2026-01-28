# SPDX-FileCopyrightText: 2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(simulated_world_nws_ros2_msgs)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  "msg/ColorRGB.msg"
  "srv/MakeSphere.srv"
  "srv/MakeBox.srv"
  "srv/MakeCylinder.srv"
  "srv/MakeFrame.srv"
  "srv/MakeModel.srv"
  "srv/ChangeColor.srv"
  "srv/SetPose.srv"
  "srv/EnableGravity.srv"
  "srv/EnableCollision.srv"
  "srv/GetPose.srv"
  "srv/DeleteObject.srv"
  "srv/DeleteAll.srv"
  "srv/GetList.srv"
  "srv/Attach.srv"
  "srv/Detach.srv"
  "srv/Rename.srv"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

# Temporary workaround for https://github.com/ros2/rosidl/pull/605
if(NOT TARGET simulated_world_nws_ros2_msgs::simulated_world_nws_ros2_msgs__rosidl_typesupport_cpp)
  add_library(simulated_world_nws_ros2_msgs::simulated_world_nws_ros2_msgs__rosidl_typesupport_cpp ALIAS simulated_world_nws_ros2_msgs__rosidl_typesupport_cpp)
endif()
