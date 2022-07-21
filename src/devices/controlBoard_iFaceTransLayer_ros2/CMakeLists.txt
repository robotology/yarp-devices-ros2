# SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(controlBoard_iFaceTransLayer_ros2
  CATEGORY device
  TYPE ControlBoard_iFaceTransLayer_ros2
  INCLUDE ControlBoard_iFaceTransLayer_ros2.h
  DEPENDS "TARGET YARP::YARP_math"
  DEFAULT ON
)

if(YARP_ROS2_USE_SYSTEM_yarp_control_msgs)
  find_package(yarp_control_msgs REQUIRED)
endif()

if(NOT SKIP_controlBoard_iFaceTransLayer_ros2)
  yarp_add_plugin(yarp_controlBoard_iFaceTransLayer_ros2)

  target_sources(yarp_controlBoard_iFaceTransLayer_ros2
    PRIVATE
      ControlBoard_iFaceTransLayer_ros2.cpp
      ControlBoard_iFaceTransLayer_ros2.h
  )
  target_sources(yarp_controlBoard_iFaceTransLayer_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_controlBoard_iFaceTransLayer_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(yarp_controlBoard_iFaceTransLayer_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      YARP::YARP_math
      rclcpp::rclcpp
      std_msgs::std_msgs__rosidl_typesupport_cpp
      yarp_control_msgs::yarp_control_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_sig
    YARP_dev
    YARP_math
    rclcpp::rclcpp
    Ros2Utils
  )

  yarp_install(
    TARGETS yarp_controlBoard_iFaceTransLayer_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_controlBoard_iFaceTransLayer_ros2 PROPERTY FOLDER "Plugins/Device/NWS")
endif()
