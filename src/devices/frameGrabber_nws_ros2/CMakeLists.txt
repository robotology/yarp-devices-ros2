# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.



yarp_prepare_plugin(
  frameGrabber_nws_ros2
  CATEGORY device
  TYPE FrameGrabber_nws_ros2
  INCLUDE FrameGrabber_nws_ros2.h
  EXTRA_CONFIG WRAPPER=controlBoard_nws_ros2
  DEFAULT ON
  INTERNAL ON
)

if(NOT SKIP_frameGrabber_nws_ros2)
  yarp_add_plugin(yarp_frameGrabber_nws_ros2)

  target_compile_definitions(yarp_frameGrabber_nws_ros2
    PRIVATE
      LOG_COMPONENT="yarp.devices.frameGrabber_nws_ros2"
      _USE_MATH_DEFINES
  )

  target_sources(yarp_frameGrabber_nws_ros2
    PRIVATE
      FrameGrabber_nws_ros2.cpp
      FrameGrabber_nws_ros2.h
  )
  target_sources(yarp_frameGrabber_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_frameGrabber_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)

  target_link_libraries(yarp_frameGrabber_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )

  yarp_install(
    TARGETS yarp_frameGrabber_nws_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_frameGrabber_nws_ros2 PROPERTY FOLDER "Plugins/Device")
endif()
