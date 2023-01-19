# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(rclcpp REQUIRED)

add_library(Ros2Utils OBJECT)

target_sources(Ros2Utils PRIVATE
        Ros2Subscriber.h
        Ros2Subscriber.cpp
        Ros2Utils.h
        Ros2Utils.cpp
        Ros2Spinner.h
        Ros2Spinner.cpp)
target_include_directories(Ros2Utils PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(Ros2Utils PRIVATE
        YARP::YARP_os
        YARP::YARP_sig
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
        std_msgs::std_msgs__rosidl_typesupport_c
        YARP::YARP_dev)

set_property(TARGET Ros2Utils PROPERTY FOLDER "Libraries/Msgs")
