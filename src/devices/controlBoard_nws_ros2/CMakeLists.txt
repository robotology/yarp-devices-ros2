# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(
  controlBoard_nws_ros2
  CATEGORY device
  TYPE ControlBoard_nws_ros2
  INCLUDE ControlBoard_nws_ros2.h
  EXTRA_CONFIG WRAPPER=controlBoard_nws_ros2
  DEFAULT ON
  INTERNAL ON
)

if(NOT SKIP_controlBoard_nws_ros2)
  yarp_add_plugin(yarp_controlBoard_nws_ros2)
  generateDeviceParamsParser(ControlBoard_nws_ros2 controlBoard_nws_ros2)

  target_compile_definitions(yarp_controlBoard_nws_ros2
    PRIVATE
      LOG_COMPONENT="yarp.devices.controlBoard_nws_ros2"
      _USE_MATH_DEFINES
  )

  target_sources(yarp_controlBoard_nws_ros2
    PRIVATE
      ControlBoard_nws_ros2.cpp
      ControlBoard_nws_ros2_callbacks.cpp
      ControlBoard_nws_ros2.h
      ControlBoard_nws_ros2_ParamsParser.h
      ControlBoard_nws_ros2_ParamsParser.cpp
  )
  target_sources(yarp_controlBoard_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_controlBoard_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(yarp_controlBoard_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
      std_msgs::std_msgs__rosidl_typesupport_cpp
      yarp_control_msgs::yarp_control_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )

  yarp_install(
    TARGETS yarp_controlBoard_nws_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_controlBoard_nws_ros2 PROPERTY FOLDER "Plugins/Device")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()

endif()
