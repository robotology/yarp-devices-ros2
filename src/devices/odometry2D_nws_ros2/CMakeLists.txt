# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(odometry2D_nws_ros2
  CATEGORY device
  TYPE Odometry2D_nws_ros2
  INCLUDE Odometry2D_nws_ros2.h
  DEPENDS "TARGET YARP::YARP_math"
  DEFAULT ON
)

if(NOT SKIP_odometry2D_nws_ros2)
  yarp_add_plugin(yarp_odometry2D_nws_ros2)
  generateDeviceParamsParser(Odometry2D_nws_ros2 odometry2D_nws_ros2)

  target_sources(yarp_odometry2D_nws_ros2
    PRIVATE
      Odometry2D_nws_ros2.cpp
      Odometry2D_nws_ros2.h
      Odometry2D_nws_ros2_ParamsParser.h
      Odometry2D_nws_ros2_ParamsParser.cpp
  )
  target_sources(yarp_odometry2D_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_odometry2D_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(yarp_odometry2D_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      YARP::YARP_math
      rclcpp::rclcpp
      nav_msgs::nav_msgs__rosidl_typesupport_cpp
      tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_sig
    YARP_dev
    YARP_math
    rclcpp::rclcpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    Ros2Utils
  )

  yarp_install(
    TARGETS yarp_odometry2D_nws_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_odometry2D_nws_ros2 PROPERTY FOLDER "Plugins/Device/NWS")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()

endif()
