# SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(battery_nws_ros2
  CATEGORY device
  TYPE Battery_nws_ros2
  INCLUDE Battery_nws_ros2.h
  DEFAULT ON
)

if(NOT SKIP_battery_nws_ros2)
  yarp_add_plugin(yarp_battery_nws_ros2)

  target_sources(yarp_battery_nws_ros2
    PRIVATE
      Battery_nws_ros2.cpp
      Battery_nws_ros2.h
  )
  target_sources(yarp_battery_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_battery_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)
  target_link_libraries(yarp_battery_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      Ros2Utils
  )
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_sig
    YARP_dev
    rclcpp::rclcpp
    Ros2Utils
  )

  yarp_install(
    TARGETS yarp_battery_nws_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_battery_nws_ros2 PROPERTY FOLDER "Plugins/Device/NWS")
endif()
