# Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(rgbdSensor_nwc_ros2
  CATEGORY device
  TYPE RgbdSensor_nwc_ros2
  INCLUDE RgbdSensor_nwc_ros2.h
  EXTRA_CONFIG WRAPPER=RgbdSensor_nwc_ros2
  INTERNAL ON
)

yarp_add_plugin(yarp_rgbdSensor_nwc_ros2)

target_sources(yarp_rgbdSensor_nwc_ros2
  PRIVATE
  RgbdSensor_nwc_ros2.cpp
  RgbdSensor_nwc_ros2.h
)

target_sources(yarp_rgbdSensor_nwc_ros2 PRIVATE $<TARGET_OBJECTS:Ros2RGBDConversionUtils> $<TARGET_OBJECTS:Ros2Utils>)

target_include_directories(yarp_rgbdSensor_nwc_ros2 PRIVATE $<TARGET_PROPERTY:Ros2RGBDConversionUtils,INTERFACE_INCLUDE_DIRECTORIES> $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)

target_link_libraries(yarp_rgbdSensor_nwc_ros2
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    Ros2RGBDConversionUtils
    Ros2Utils
)

yarp_install(
  TARGETS yarp_rgbdSensor_nwc_ros2
  EXPORT yarp-device-rgbdSensor_nwc_ros2
  COMPONENT yarp-device-rgbdSensor_nwc_ros2
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_rgbdSensor_nwc_ros2 PROPERTY FOLDER "Plugins/Device")
