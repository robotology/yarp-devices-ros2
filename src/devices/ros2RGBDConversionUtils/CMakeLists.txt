# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

#if(NOT YARP_COMPILE_DEVICE_PLUGINS)
#  return()
#endif()

add_library(Ros2RGBDConversionUtils OBJECT)

target_sources(Ros2RGBDConversionUtils PRIVATE
        Ros2RGBDConversionUtils.cpp
        Ros2RGBDConversionUtils.h
        ros2PixelCode.h
        ros2PixelCode.cpp)

target_include_directories(Ros2RGBDConversionUtils PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(Ros2RGBDConversionUtils PRIVATE YARP::YARP_os
                                                     YARP::YARP_sig
                                                     rclcpp::rclcpp
                                                     sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
                                                     std_msgs::std_msgs__rosidl_typesupport_c
                                                     YARP::YARP_dev)

set_property(TARGET Ros2RGBDConversionUtils PROPERTY FOLDER "Libraries/Msgs")
